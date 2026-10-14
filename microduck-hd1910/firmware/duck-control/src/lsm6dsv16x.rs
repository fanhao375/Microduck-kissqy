// Added 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; see IMU-MODULE.md at the repository root.
//! An LSM6DSV16X on a breakout, as bytes: its bring-up as data and its FIFO, with no I/O.
//!
//! The chip the bus board carries, wired to the board's own I²C instead. Its SFLP fusion
//! writes game-rotation quaternions into the FIFO beside the gyro; [`BRING_UP`] is the register
//! sequence that gets it there, and [`Lsm6dsv16xAssembler`] turns FIFO bytes into
//! [`HostImuSample`]s. Whoever owns the i2c-dev file (robotd's reader thread, Linux only) runs
//! the one and feeds the other. Read the FIFO the way the board's firmware does:
//! [`FIFO_STATUS1`] and the byte after it for the count ([`FifoStatus`]), then one 7-byte read
//! from [`FIFO_DATA_OUT_TAG`] per word.
//!
//! **Nothing here is designed afresh.** The configuration is the one the bus board's firmware
//! runs — microduck-replica `hardware/imu_to_dxl/firmware/Core/Src/imu.c`, `configure_one_step`,
//! as of 6996f32 — with each ST driver call resolved to the register and bits that driver
//! (`Drivers/LSM6DSV16X/lsm6dsv16x_reg.c` and `.h` in the same tree, ST 2024) touches; the
//! citations below are to those three files. The decoding is [`crate::imu`]'s own, so the same
//! FIFO bytes give the same reading the bus board's block would.

use std::time::{Duration, Instant};

use crate::imu::{lsm6dsv16x_gyro, sflp_quaternion};
use crate::trunk_imu::HostImuSample;

/// Bank select, reachable from every bank. Bit 7 embedded functions, bit 6 sensor hub,
/// bit 2 software power-on reset. `reg.h:200`.
pub const FUNC_CFG_ACCESS: u8 = 0x01;
pub const FIFO_CTRL1: u8 = 0x07; // reg.h:272
pub const FIFO_CTRL3: u8 = 0x09; // reg.h:304
pub const FIFO_CTRL4: u8 = 0x0A; // reg.h:316
pub const WHO_AM_I: u8 = 0x0F; // reg.h:406
/// What [`WHO_AM_I`] reads on this chip and no other. `reg.h:188`.
pub const ID: u8 = 0x70;
pub const CTRL1: u8 = 0x10; // reg.h:416
pub const CTRL2: u8 = 0x11; // reg.h:430
pub const CTRL3: u8 = 0x12; // reg.h:444
pub const CTRL6: u8 = 0x15; // reg.h:498
pub const CTRL8: u8 = 0x17; // reg.h:530
pub const FIFO_STATUS1: u8 = 0x1B; // reg.h:604
pub const HAODR_CFG: u8 = 0x62; // reg.h:1424
pub const FIFO_DATA_OUT_TAG: u8 = 0x78; // reg.h:1646
/// Embedded-function bank. `reg.h:2002`.
pub const EMB_FUNC_EN_A: u8 = 0x04;
/// Embedded-function bank. `reg.h:2280`.
pub const EMB_FUNC_FIFO_EN_A: u8 = 0x44;
/// Embedded-function bank. `reg.h:2562`.
pub const SFLP_ODR: u8 = 0x5E;

/// One step of [`BRING_UP`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    /// Write the whole register.
    Write { register: u8, value: u8 },
    /// Read, set only the bits in `mask` to `value`, write back. Every ST `*_set` call used here
    /// is this shape, so bits the sequence does not own keep what the reset left in them.
    Update { register: u8, mask: u8, value: u8 },
    /// Read; bring-up has failed unless `register & mask == value`.
    Expect { register: u8, mask: u8, value: u8 },
    /// Give the chip this long before the next step.
    Wait(Duration),
    /// Read until `register & mask == value`; bring-up has failed if `timeout` passes first.
    /// A read that fails in the meantime is a chip still resetting, not yet a failure.
    Poll {
        register: u8,
        mask: u8,
        value: u8,
        timeout: Duration,
    },
}

/// `lsm6dsv16x_mem_bank_set` (`reg.c:439`): bits 7:6 of [`FUNC_CFG_ACCESS`], read-modify-write.
const EMBEDDED_BANK: Op = Op::Update {
    register: FUNC_CFG_ACCESS,
    mask: 0xC0,
    value: 0x80,
};
const MAIN_BANK: Op = Op::Update {
    register: FUNC_CFG_ACCESS,
    mask: 0xC0,
    value: 0x00,
};

/// From power-up, or from any state a previous run left the chip in, to SFLP game rotation
/// and gyro streaming through the FIFO at 120 Hz. Ends in the main bank.
///
/// Nothing is written to anything that survives a power cycle, and the gyro-bias call the
/// firmware deliberately skips (`imu.c:115`) is skipped here too.
pub const BRING_UP: &[Op] = &[
    // Main bank, and nothing else selected, before anything is trusted: a previous run may have
    // stopped inside the embedded bank. imu.c:240-241.
    Op::Write {
        register: FUNC_CFG_ACCESS,
        value: 0x00,
    },
    // Is it the chip at all. imu.c:242-249.
    Op::Expect {
        register: WHO_AM_I,
        mask: 0xFF,
        value: ID,
    },
    // Software power-on reset (FUNC_CFG_ACCESS.SW_POR, bit 2), so every register below starts
    // from its reset value whatever ran before. imu.c:250-251.
    Op::Write {
        register: FUNC_CFG_ACCESS,
        value: 0x04,
    },
    // imu.c:259-266: wait 30 ms, then until SW_POR clears, failing at 100 ms.
    Op::Wait(Duration::from_millis(30)),
    Op::Poll {
        register: FUNC_CFG_ACCESS,
        mask: 0x04,
        value: 0x00,
        timeout: Duration::from_millis(70),
    },
    // block_data_update_set: CTRL3.BDU, bit 6. imu.c:100, reg.c:1109.
    Op::Update {
        register: CTRL3,
        mask: 0x40,
        value: 0x40,
    },
    // auto_increment_set: CTRL3.IF_INC, bit 2. imu.c:101, reg.c:1077.
    Op::Update {
        register: CTRL3,
        mask: 0x04,
        value: 0x04,
    },
    // fifo_mode_set(BYPASS = 0): FIFO_CTRL4.FIFO_MODE, bits 2:0. imu.c:102, reg.c:3417,
    // reg.h:5498.
    Op::Update {
        register: FIFO_CTRL4,
        mask: 0x07,
        value: 0x00,
    },
    // xl_full_scale_set(4g = 1): CTRL8.FS_XL, bits 1:0. imu.c:103, reg.c:1343, reg.h:4519.
    Op::Update {
        register: CTRL8,
        mask: 0x03,
        value: 0x01,
    },
    // gy_full_scale_set(500dps = 2): CTRL6.FS_G, bits 3:0 — the ±500 dps that
    // `imu::lsm6dsv16x_gyro` scales by. imu.c:104, reg.c:1278, reg.h:4489.
    Op::Update {
        register: CTRL6,
        mask: 0x0F,
        value: 0x02,
    },
    // fifo_watermark_set(3): FIFO_CTRL1 is all WTM, and the driver writes it whole.
    // imu.c:105, reg.c:2999.
    Op::Write {
        register: FIFO_CTRL1,
        value: 3,
    },
    // fifo_xl_batch_set(120 Hz = 6): FIFO_CTRL3.BDR_XL, bits 3:0. imu.c:106, reg.c:3235,
    // reg.h:5428.
    Op::Update {
        register: FIFO_CTRL3,
        mask: 0x0F,
        value: 0x06,
    },
    // fifo_gy_batch_set(120 Hz = 6): FIFO_CTRL3.BDR_GY, bits 7:4. imu.c:107, reg.c:3326,
    // reg.h:5466.
    Op::Update {
        register: FIFO_CTRL3,
        mask: 0xF0,
        value: 0x60,
    },
    // fifo_sflp_batch_set({game_rotation: 1}): EMB_FUNC_FIFO_EN_A.SFLP_GAME_FIFO_EN (bit 1)
    // on, gravity (bit 4) and gyro bias (bit 5) off — the driver sets all three from the one
    // struct. imu.c:108-110, reg.c:4107.
    EMBEDDED_BANK,
    Op::Update {
        register: EMB_FUNC_FIFO_EN_A,
        mask: 0x32,
        value: 0x02,
    },
    MAIN_BANK,
    // sflp_data_rate_set(120 Hz = 3): SFLP_ODR.SFLP_GAME_ODR, bits 5:3. imu.c:111,
    // reg.c:8140, reg.h:7968.
    EMBEDDED_BANK,
    Op::Update {
        register: SFLP_ODR,
        mask: 0x38,
        value: 0x18,
    },
    MAIN_BANK,
    // xl_data_rate_set(120 Hz = 0x06): CTRL1.ODR_XL, bits 3:0. The value's high nibble (the
    // HAODR selection) is zero, so the driver leaves HAODR_CFG alone. imu.c:112, reg.c:501,
    // reg.h:4239.
    Op::Update {
        register: CTRL1,
        mask: 0x0F,
        value: 0x06,
    },
    // gy_data_rate_set(120 Hz): CTRL2.ODR_G, bits 3:0, likewise. imu.c:113, reg.c:795.
    Op::Update {
        register: CTRL2,
        mask: 0x0F,
        value: 0x06,
    },
    // sflp_game_rotation_set(1): EMB_FUNC_EN_A.SFLP_GAME_EN, bit 1. imu.c:114, reg.c:7701.
    EMBEDDED_BANK,
    Op::Update {
        register: EMB_FUNC_EN_A,
        mask: 0x02,
        value: 0x02,
    },
    MAIN_BANK,
    // fifo_mode_set(STREAM = 6). imu.c:117, reg.h:5503.
    Op::Update {
        register: FIFO_CTRL4,
        mask: 0x07,
        value: 0x06,
    },
    // The firmware's read-backs, imu.c:118-132. Data rate reads both ODR and the HAODR
    // selection, which must be 0 for "120 Hz" (reg.c:539, the 0x06 case).
    Op::Expect {
        register: CTRL1,
        mask: 0x0F,
        value: 0x06,
    },
    Op::Expect {
        register: CTRL2,
        mask: 0x0F,
        value: 0x06,
    },
    Op::Expect {
        register: HAODR_CFG,
        mask: 0x03,
        value: 0x00,
    },
    Op::Expect {
        register: CTRL6,
        mask: 0x0F,
        value: 0x02,
    },
    Op::Expect {
        register: CTRL8,
        mask: 0x03,
        value: 0x01,
    },
    EMBEDDED_BANK,
    Op::Expect {
        register: EMB_FUNC_EN_A,
        mask: 0x02,
        value: 0x02,
    },
    MAIN_BANK,
];

/// What `FIFO_STATUS1` and `FIFO_STATUS2` say, read as one 2-byte transfer
/// (`lsm6dsv16x_fifo_status_get`, `reg.c:3746`; the layout is `reg.h:604-622`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FifoStatus {
    /// Words waiting, 9 bits.
    pub unread: u16,
    pub watermark: bool,
    pub overrun: bool,
    pub full: bool,
}

impl FifoStatus {
    pub fn parse(bytes: [u8; 2]) -> Self {
        Self {
            unread: (u16::from(bytes[1] & 0x01) << 8) | u16::from(bytes[0]),
            watermark: bytes[1] & 0x80 != 0,
            overrun: bytes[1] & 0x40 != 0,
            full: bytes[1] & 0x20 != 0,
        }
    }

    /// Samples were dropped, so the stream has a hole in it. The firmware treats this as a
    /// fault and starts over (`imu.c:172-176`); a reader should too, as a new epoch.
    pub fn lost_data(&self) -> bool {
        self.overrun || self.full
    }
}

/// One FIFO word: a tag byte, then six bytes of data.
pub const WORD_LEN: usize = 7;

/// FIFO tags, bits 7:3 of a word's first byte (`reg.h:1646-1652`, values `reg.h:5720-5741`).
pub mod tag {
    pub const GYRO: u8 = 0x01;
    pub const ACCELEROMETER: u8 = 0x02;
    pub const GAME_ROTATION: u8 = 0x13;
}

/// The tag of a word, from its first byte. Bits 2:1 are a rolling counter and bit 0 is unused.
pub fn word_tag(first: u8) -> u8 {
    first >> 3
}

/// Builds [`HostImuSample`]s from FIFO bytes.
///
/// A sample per game-rotation word, once a gyro word has arrived to pair it with; a gyro on
/// its own never publishes, so it cannot make a stalled orientation look live. Bytes may arrive
/// in any chunks — a word cut across two reads is joined.
#[derive(Debug)]
pub struct Lsm6dsv16xAssembler {
    partial: [u8; WORD_LEN],
    partial_len: usize,
    epoch: u32,
    epoch_started_at: Instant,
    sequence: u32,
    gyro: Option<([f64; 3], Instant)>,
    ignored: u64,
    invalid_quaternions: u64,
}

impl Lsm6dsv16xAssembler {
    pub fn new(epoch: u32, now: Instant) -> Self {
        Self {
            partial: [0; WORD_LEN],
            partial_len: 0,
            epoch,
            epoch_started_at: now,
            sequence: 0,
            gyro: None,
            ignored: 0,
            invalid_quaternions: 0,
        }
    }

    /// The chip was brought up again, or its FIFO lost data. A half word and the last rate
    /// belong to the stream that ended; the counters and the host sequence carry on.
    pub fn restart(&mut self, now: Instant) {
        *self = Self {
            sequence: self.sequence,
            ignored: self.ignored,
            invalid_quaternions: self.invalid_quaternions,
            ..Self::new(self.epoch.wrapping_add(1), now)
        };
    }

    pub fn epoch(&self) -> u32 {
        self.epoch
    }

    /// Words passed over: accelerometer, timestamps, configuration changes, anything not a
    /// gyro or a game rotation. The accelerometer is batched because SFLP needs it running,
    /// not because the host uses it.
    pub fn ignored(&self) -> u64 {
        self.ignored
    }

    /// Game-rotation words that were not a quaternion — non-finite or longer than unit.
    pub fn invalid_quaternions(&self) -> u64 {
        self.invalid_quaternions
    }

    /// FIFO bytes, in the order read. The newest sample they complete, if any.
    pub fn feed(&mut self, bytes: &[u8], now: Instant) -> Option<HostImuSample> {
        let mut newest = None;
        let mut rest = bytes;
        while !rest.is_empty() {
            let take = (WORD_LEN - self.partial_len).min(rest.len());
            self.partial[self.partial_len..self.partial_len + take].copy_from_slice(&rest[..take]);
            self.partial_len += take;
            rest = &rest[take..];
            if self.partial_len == WORD_LEN {
                self.partial_len = 0;
                let word = self.partial;
                newest = self.word(&word, now).or(newest);
            }
        }
        newest
    }

    fn word(&mut self, word: &[u8; WORD_LEN], now: Instant) -> Option<HostImuSample> {
        let data = [word[1], word[2], word[3], word[4], word[5], word[6]];
        match word_tag(word[0]) {
            tag::GYRO => {
                self.gyro = Some((lsm6dsv16x_gyro(data), now));
                None
            }
            tag::GAME_ROTATION => {
                // Unlike the bus board's block, the FIFO has no all-zero "not started" marker:
                // a game-rotation word is always a measurement, and x = y = z = 0 is identity.
                let packed = [0, 2, 4].map(|at| u16::from_le_bytes([data[at], data[at + 1]]));
                let Some(quat) = sflp_quaternion(packed) else {
                    self.invalid_quaternions = self.invalid_quaternions.saturating_add(1);
                    return None;
                };
                let (rate, rate_at) = self.gyro?;
                self.sequence = self.sequence.wrapping_add(1);
                Some(HostImuSample {
                    quat,
                    gyro: rate,
                    received_at: rate_at.min(now),
                    sequence: self.sequence,
                    epoch: self.epoch,
                    epoch_started_at: self.epoch_started_at,
                    accuracy: None,
                })
            }
            _ => {
                self.ignored = self.ignored.saturating_add(1);
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::{IMU_BLOCK_LEN, parse_block};
    use std::collections::HashMap;

    /// The chip as [`BRING_UP`] sees it: two banks of registers, `FUNC_CFG_ACCESS` reachable
    /// from both, SW_POR finishing at once. Registers start from `reset` so a test can see
    /// which bits the sequence leaves alone.
    struct Chip {
        main: HashMap<u8, u8>,
        embedded: HashMap<u8, u8>,
        func_cfg_access: u8,
        resets: u32,
    }

    impl Chip {
        fn new(reset: u8) -> Self {
            let mut main: HashMap<u8, u8> = [
                FIFO_CTRL1, FIFO_CTRL3, FIFO_CTRL4, CTRL1, CTRL2, CTRL3, CTRL6, CTRL8, HAODR_CFG,
            ]
            .into_iter()
            .map(|r| (r, reset))
            .collect();
            main.insert(WHO_AM_I, ID);
            // A chip that comes up with HAODR selected would not be at 120 Hz; the reset value
            // of the selection field is zero.
            main.insert(HAODR_CFG, reset & !0x03);
            let embedded = [EMB_FUNC_EN_A, EMB_FUNC_FIFO_EN_A, SFLP_ODR]
                .into_iter()
                .map(|r| (r, reset))
                .collect();
            Self {
                main,
                embedded,
                // Left in the embedded bank by a run that died halfway.
                func_cfg_access: 0x80,
                resets: 0,
            }
        }

        fn register(&mut self, register: u8) -> &mut u8 {
            if register == FUNC_CFG_ACCESS {
                return &mut self.func_cfg_access;
            }
            let bank = if self.func_cfg_access & 0x80 != 0 {
                &mut self.embedded
            } else {
                &mut self.main
            };
            bank.get_mut(&register)
                .unwrap_or_else(|| panic!("register {register:#04x} is not in the selected bank"))
        }

        /// Every op in turn; the first `Expect` or `Poll` the chip does not satisfy is the error.
        fn run(&mut self, ops: &[Op]) -> Result<(), Op> {
            for op in ops {
                match *op {
                    Op::Write { register, value } => {
                        *self.register(register) = value;
                        if register == FUNC_CFG_ACCESS && value & 0x04 != 0 {
                            self.resets += 1;
                            self.func_cfg_access = 0;
                        }
                    }
                    Op::Update {
                        register,
                        mask,
                        value,
                    } => {
                        let current = self.register(register);
                        *current = (*current & !mask) | value;
                    }
                    Op::Expect {
                        register,
                        mask,
                        value,
                    }
                    | Op::Poll {
                        register,
                        mask,
                        value,
                        ..
                    } => {
                        if *self.register(register) & mask != value {
                            return Err(*op);
                        }
                    }
                    Op::Wait(_) => {}
                }
            }
            Ok(())
        }
    }

    /// The whole sequence, run against a model of the chip, lands on exactly the settings
    /// the bus board's firmware configures — field by field — and leaves every bit it does
    /// not own as the reset made it. It also has to start from a chip left in the embedded
    /// bank, and finish in the main bank, or the next FIFO read reads the wrong page.
    #[test]
    fn bring_up_lands_on_the_firmware_s_configuration() {
        // Bits outside each field keep whatever the reset put there; a pattern makes that
        // visible where zeroes would hide it.
        let reset = 0b1000_0001;
        let mut chip = Chip::new(reset);
        chip.run(BRING_UP).unwrap();
        assert_eq!(chip.resets, 1);
        assert_eq!(chip.func_cfg_access & 0xC0, 0, "must end in the main bank");
        let keep = |mask: u8, value: u8| (reset & !mask) | value;
        for (register, want, what) in [
            (CTRL3, keep(0x44, 0x44), "BDU and IF_INC"),
            (FIFO_CTRL4, keep(0x07, 0x06), "stream mode"),
            (CTRL8, keep(0x03, 0x01), "±4 g"),
            (CTRL6, keep(0x0F, 0x02), "±500 dps"),
            (FIFO_CTRL1, 3, "watermark 3"),
            (FIFO_CTRL3, 0x66, "accel and gyro batched at 120 Hz"),
            (CTRL1, keep(0x0F, 0x06), "accel 120 Hz"),
            (CTRL2, keep(0x0F, 0x06), "gyro 120 Hz"),
        ] {
            assert_eq!(chip.main[&register], want, "{what}");
        }
        for (register, want, what) in [
            (EMB_FUNC_EN_A, keep(0x02, 0x02), "SFLP game rotation on"),
            (
                EMB_FUNC_FIFO_EN_A,
                keep(0x32, 0x02),
                "only game rotation batched",
            ),
            (SFLP_ODR, keep(0x38, 0x18), "SFLP at 120 Hz"),
        ] {
            assert_eq!(chip.embedded[&register], want, "{what}");
        }
    }

    /// Something else on the address, or a chip that is not this one, must fail at the
    /// first read rather than be configured and read as orientation.
    #[test]
    fn bring_up_checks_who_it_is_talking_to_before_anything_else() {
        assert_eq!(
            BRING_UP[1],
            Op::Expect {
                register: WHO_AM_I,
                mask: 0xFF,
                value: 0x70
            }
        );
        assert!(
            matches!(BRING_UP[0], Op::Write { register, .. } if register == FUNC_CFG_ACCESS),
            "only the bank select may come before the identity check"
        );
        let mut chip = Chip::new(0);
        chip.main.insert(WHO_AM_I, 0x6C);
        assert_eq!(chip.run(BRING_UP), Err(BRING_UP[1]));
        assert_eq!(
            chip.resets, 0,
            "nothing written to a chip that is not this one"
        );
    }

    #[test]
    fn fifo_status_is_a_nine_bit_count_and_three_flags() {
        assert_eq!(
            FifoStatus::parse([0, 0]),
            FifoStatus {
                unread: 0,
                watermark: false,
                overrun: false,
                full: false
            }
        );
        let status = FifoStatus::parse([0x05, 0x01]);
        assert_eq!(status.unread, 261);
        assert!(!status.lost_data());
        let status = FifoStatus::parse([0xFF, 0xFF]);
        assert_eq!(status.unread, 511);
        assert!(status.watermark && status.overrun && status.full);
        assert!(FifoStatus::parse([0, 0x40]).lost_data());
        assert!(FifoStatus::parse([0, 0x20]).lost_data());
        // The latched overrun bit (3) is history, not this read's news.
        assert!(!FifoStatus::parse([0, 0x08]).lost_data());
    }

    fn word(tag: u8, data: [u8; 6]) -> [u8; WORD_LEN] {
        let mut out = [tag << 3; WORD_LEN];
        out[1..].copy_from_slice(&data);
        out
    }
    /// The golden table's third block: a gyro and a quaternion the bus board sent.
    const GYRO: [u8; 6] = [24, 252, 250, 0, 184, 11];
    const QUAT: [u8; 6] = [0, 52, 0, 176, 102, 46];

    /// The same chip's same bytes must be the same reading whichever way it is wired: one
    /// gyro word and one game-rotation word give exactly what the bus board's block carrying
    /// those bytes gives — no rounding of their own, no second normalisation.
    #[test]
    fn fifo_words_read_exactly_as_the_bus_board_s_block() {
        let t = Instant::now();
        let mut a = Lsm6dsv16xAssembler::new(1, t);
        let mut bytes = word(tag::GYRO, GYRO).to_vec();
        bytes.extend(word(tag::GAME_ROTATION, QUAT));
        let sample = a.feed(&bytes, t).unwrap();

        let mut block = [0u8; IMU_BLOCK_LEN];
        block[..6].copy_from_slice(&GYRO);
        block[6..].copy_from_slice(&QUAT);
        let board = parse_block(&block);
        assert_eq!(sample.gyro, board.gyro);
        assert_eq!(Some(sample.quat), board.quat);
        assert!(sample.quat[0] >= 0.0);
        assert_eq!((sample.sequence, sample.epoch), (1, 1));
        assert_eq!(sample.received_at, t);
    }

    /// A word may be cut anywhere by the reads that carry it.
    #[test]
    fn a_word_split_across_reads_is_joined() {
        let t = Instant::now();
        let mut whole = Lsm6dsv16xAssembler::new(1, t);
        let mut bytes = word(tag::GYRO, GYRO).to_vec();
        bytes.extend(word(tag::GAME_ROTATION, QUAT));
        let expected = whole.feed(&bytes, t).unwrap();
        for cut in 1..bytes.len() {
            let mut split = Lsm6dsv16xAssembler::new(1, t);
            assert_eq!(split.feed(&bytes[..cut], t), None, "cut at {cut}");
            assert_eq!(split.feed(&bytes[cut..], t), Some(expected), "cut at {cut}");
        }
    }

    /// An empty FIFO is a read of nothing, and changes nothing.
    #[test]
    fn an_empty_fifo_is_no_sample() {
        let t = Instant::now();
        let mut a = Lsm6dsv16xAssembler::new(1, t);
        assert_eq!(a.feed(&[], t), None);
        assert_eq!((a.ignored(), a.invalid_quaternions()), (0, 0));
    }

    /// The accelerometer is batched only because SFLP needs it; it, timestamps and tags this
    /// code has never heard of are counted and passed over without disturbing the words
    /// around them. The counter bits under the tag are not part of it.
    #[test]
    fn other_tags_are_counted_and_passed_over() {
        let t = Instant::now();
        let mut a = Lsm6dsv16xAssembler::new(1, t);
        let mut bytes = word(tag::GYRO, GYRO).to_vec();
        bytes.extend(word(tag::ACCELEROMETER, [1, 2, 3, 4, 5, 6]));
        bytes.extend(word(0x04, [0; 6])); // timestamp
        bytes.extend(word(0x1F, [0xFF; 6])); // not a tag this chip documents
        let mut rotation = word(tag::GAME_ROTATION, QUAT);
        rotation[0] |= 0b110;
        bytes.extend(rotation);
        assert!(a.feed(&bytes, t).is_some());
        assert_eq!(a.ignored(), 3);
    }

    /// A rotation before any rate is not a sample, and a rate alone never is.
    #[test]
    fn a_sample_needs_both_halves() {
        let t = Instant::now();
        let mut a = Lsm6dsv16xAssembler::new(1, t);
        assert_eq!(a.feed(&word(tag::GAME_ROTATION, QUAT), t), None);
        assert_eq!(a.feed(&word(tag::GYRO, GYRO), t), None);
        let later = t + Duration::from_millis(30);
        let sample = a.feed(&word(tag::GAME_ROTATION, QUAT), later).unwrap();
        assert_eq!(
            sample.received_at, t,
            "a sample is as old as its older half"
        );
    }

    /// A NaN component or a vector past unit length is not something SFLP produced. Counted,
    /// not published, and the rate already held stays usable for the next good one.
    #[test]
    fn a_bad_quaternion_is_counted_and_skipped() {
        let t = Instant::now();
        let mut a = Lsm6dsv16xAssembler::new(1, t);
        a.feed(&word(tag::GYRO, GYRO), t);
        let nan = [0x00, 0x7E, 0, 0, 0, 0];
        let long = [0x00, 0x3C, 0x00, 0x38, 0, 0];
        assert_eq!(a.feed(&word(tag::GAME_ROTATION, nan), t), None);
        assert_eq!(a.feed(&word(tag::GAME_ROTATION, long), t), None);
        assert_eq!(a.invalid_quaternions(), 2);
        assert_eq!(
            a.feed(&word(tag::GAME_ROTATION, QUAT), t).unwrap().sequence,
            1
        );
    }

    /// The bus board marks "SFLP not started" with all-zero bytes. The FIFO has no such
    /// marker, so zeroes there are a real identity orientation and must be published.
    #[test]
    fn zero_components_in_the_fifo_are_identity() {
        let t = Instant::now();
        let mut a = Lsm6dsv16xAssembler::new(1, t);
        a.feed(&word(tag::GYRO, [0; 6]), t);
        let sample = a.feed(&word(tag::GAME_ROTATION, [0; 6]), t).unwrap();
        assert_eq!(sample.quat, [1.0, 0.0, 0.0, 0.0]);
    }

    /// Several rotations in one read: each is a new orientation, the newest is what is
    /// published.
    #[test]
    fn the_newest_of_several_rotations_is_published() {
        let t = Instant::now();
        let mut a = Lsm6dsv16xAssembler::new(1, t);
        let mut bytes = word(tag::GYRO, GYRO).to_vec();
        bytes.extend(word(tag::GAME_ROTATION, [0; 6]));
        bytes.extend(word(tag::GAME_ROTATION, QUAT));
        let sample = a.feed(&bytes, t).unwrap();
        assert_eq!(sample.sequence, 2);
        assert_ne!(sample.quat, [1.0, 0.0, 0.0, 0.0]);
    }

    /// After a restart nothing from the old stream survives: not half a word, not a rate.
    #[test]
    fn a_restart_drops_half_a_word_and_the_old_rate() {
        let t = Instant::now();
        let mut a = Lsm6dsv16xAssembler::new(1, t);
        a.feed(&word(tag::GYRO, GYRO), t);
        a.feed(&word(tag::GAME_ROTATION, QUAT), t).unwrap();
        a.feed(&word(tag::GYRO, GYRO)[..3], t);
        let later = t + Duration::from_millis(500);
        a.restart(later);
        assert_eq!(a.epoch(), 2);
        assert_eq!(
            a.feed(&word(tag::GAME_ROTATION, QUAT), later),
            None,
            "no rate in this epoch yet, and no stale bytes in front of the word"
        );
        a.feed(&word(tag::GYRO, GYRO), later);
        let sample = a.feed(&word(tag::GAME_ROTATION, QUAT), later).unwrap();
        assert_eq!((sample.epoch, sample.epoch_started_at), (2, later));
        assert_eq!(sample.sequence, 2, "the host sequence keeps counting");
    }
}
