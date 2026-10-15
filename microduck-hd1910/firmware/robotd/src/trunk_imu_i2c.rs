// Added 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; BNO08x gyro-integrated attitude; see IMU-MODULE.md at the repository root.
//! A trunk IMU module on the board's I²C: the reader thread behind `trunk_imu::HostImu`.
//!
//! With `[trunk_imu] source` set to a module, the trunk IMU is not on the servo bus and the
//! control loop does not read it. This thread owns the i2c-dev file, speaks one of two codecs
//! (`duck_control::bno08x`, `duck_control::lsm6dsv16x`) through a state machine per chip, and
//! publishes every new sample into a lock-free slot that the control thread reads once a tick.
//! Nothing here can make a tick wait: a slow, wedged or absent sensor shows up as an old
//! sample, and `HostImuGate` refuses it with the bus board's own words.
//!
//! Faults go three ways. A single failed transfer or a packet that makes no sense is dropped
//! and counted, and the stream goes on. A device fault — open, lock, a run of failed transfers,
//! a stream with nothing new for its stall time however its reads are going — closes the device
//! and reopens it after a backoff, as a new epoch. A chip that
//! starts over by itself (a BNO08x reset) or loses data (an LSM6DSV16X FIFO overrun) is set up
//! again in place, also as a new epoch. Either way the gate holds samples off for a second while
//! the fusion settles. Nothing written persists on the chip: a BNO08x's report settings live in
//! its RAM and are turned off again on the way out, and an LSM6DSV16X has nothing to persist.
//!
//! The device itself is Linux only (i2c-dev, `flock`, `I2C_SLAVE`, `I2C_RDWR`). Everything else
//! is plain code, tested anywhere against a fake bus; elsewhere the reader runs and reports that
//! it has no device to open.

use std::io;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use arc_swap::ArcSwapOption;
use duck_control::bno08x::{self, Bno08xAssembler, ControlReply, Header, Packet, Writer};
use duck_control::lsm6dsv16x::{self, FifoStatus, Lsm6dsv16xAssembler, Op};
use duck_control::trunk_imu::{FusionHealth, HostImu, HostImuHealth, HostImuSample, TrunkImuKind};

use crate::params::{TrunkImuParams, TrunkImuSource};

/// The source `[trunk_imu]` names, as the control core knows it.
pub fn kind_of(source: TrunkImuSource) -> TrunkImuKind {
    match source {
        TrunkImuSource::Bus => TrunkImuKind::Bus,
        TrunkImuSource::Bno08x => TrunkImuKind::Bno08x,
        TrunkImuSource::Lsm6dsv16x => TrunkImuKind::Lsm6dsv16x,
    }
}

/// Start the reader `[trunk_imu]` asks for; `None` when the trunk IMU is the bus board.
pub fn spawn_for(params: &TrunkImuParams) -> Option<io::Result<TrunkImuReader>> {
    let kind = kind_of(params.source);
    (kind != TrunkImuKind::Bus)
        .then(|| TrunkImuReader::spawn(kind, &params.i2c_device, params.i2c_address))
}

/// One device at one address, as the state machines see it.
pub(crate) trait I2c {
    /// One read transfer of `buf.len()` bytes; returns how many arrived.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
    /// One write transfer; returns how many bytes went.
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize>;
    /// Write `register`, then read `buf.len()` bytes after a repeated start.
    fn read_register(&mut self, register: u8, buf: &mut [u8]) -> io::Result<()>;
}

/// A whole transfer or an error: i2c-dev reads are single transactions, so a short one is a
/// fault to report, never something to top up with a second read.
fn read_exactly(io: &mut dyn I2c, buf: &mut [u8], what: &str) -> Result<(), String> {
    match io.read(buf) {
        Ok(n) if n == buf.len() => Ok(()),
        Ok(n) => Err(format!("{what}: read {n} of {} bytes", buf.len())),
        Err(e) => Err(format!("{what}: {e}")),
    }
}

fn write_all(io: &mut dyn I2c, bytes: &[u8], what: &str) -> Result<(), String> {
    match io.write(bytes) {
        Ok(n) if n == bytes.len() => Ok(()),
        Ok(n) => Err(format!("{what}: wrote {n} of {} bytes", bytes.len())),
        Err(e) => Err(format!("{what}: {e}")),
    }
}

/// What the runner does after a step.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Step {
    /// Step again after this long.
    Again(Duration),
    /// The chip started over or lost data and is being set up again in place: a fault worth
    /// counting, not one worth reopening the device for. Step again after this long.
    Restart(String, Duration),
    /// A transfer or a packet was lost and the stream carries on: counted, not acted on.
    /// Step again after this long.
    Dropped(String, Duration),
    /// The device is unusable: close it, back off, reopen as a new epoch.
    Failed(String),
}

/// A fault while streaming, by what it says about the device.
#[derive(Debug)]
enum Fault {
    /// A transfer failed — a NACK, a short read. One is noise on a bus that shares a supply
    /// with fifteen servos; a run of them is a device that has gone.
    Io(String),
    /// A transfer arrived but its bytes made no sense — a header that lost its framing, a
    /// packet that changed between its two reads. Dropped; the next read starts afresh.
    Garbled(String),
}

/// Failed transfers in a row that make a device gone. Fewer are dropped and counted: a reopen
/// costs a reset, an identification and a second of settling — about two seconds without a
/// trunk IMU, which mid-walk is a fall risk that one missed sample is not.
const IO_FAILURE_LIMIT: u32 = 5;

/// One streaming step's outcome, with `io_run` counting failed transfers in a row. `retry`
/// is the wait after anything dropped: a chip answering garbage is read at the poll interval,
/// never spun on.
fn resolve(io_run: &mut u32, result: Result<Step, Fault>, retry: Duration) -> Step {
    match result {
        Ok(step) => {
            *io_run = 0;
            step
        }
        Err(Fault::Io(reason)) => {
            *io_run += 1;
            if *io_run >= IO_FAILURE_LIMIT {
                Step::Failed(format!(
                    "{} transfers failed in a row; last: {reason}",
                    *io_run
                ))
            } else {
                Step::Dropped(reason, retry)
            }
        }
        // The transfer worked but says nothing good about the device: it neither ends a run
        // of failed transfers nor earns an immediate retry.
        Err(Fault::Garbled(reason)) => Step::Dropped(reason, retry),
    }
}

/// Whatever a streaming step came to, a stream that has published nothing new for `stall`
/// has stopped. Checked after the step, not inside it: a step that returns early on a failed
/// or garbled read must not skip it, or a chip that answers only garbage — or garbage between
/// NACKs — is read forever and never reopened.
fn watchdog(step: Step, now: Instant, last_sample: Instant, stall: Duration, what: &str) -> Step {
    if matches!(step, Step::Failed(_)) || now.saturating_duration_since(last_sample) <= stall {
        return step;
    }
    Step::Failed(format!("{what} for {} s", stall.as_secs_f64()))
}

/// What a reader is doing, for `HostImuHealth::state`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum State {
    Opening,
    Resetting,
    Identifying,
    Configuring,
    Streaming,
    Backoff,
    Stopped,
}

impl State {
    const ALL: [Self; 7] = [
        Self::Opening,
        Self::Resetting,
        Self::Identifying,
        Self::Configuring,
        Self::Streaming,
        Self::Backoff,
        Self::Stopped,
    ];

    fn label(self) -> &'static str {
        match self {
            Self::Opening => "opening",
            Self::Resetting => "resetting",
            Self::Identifying => "identifying",
            Self::Configuring => "configuring",
            Self::Streaming => "streaming",
            Self::Backoff => "backoff",
            Self::Stopped => "stopped",
        }
    }
}

/// One chip's protocol, a step at a time. Never sleeps and never reads the clock: the runner
/// does both, so a test can drive it with a fake bus and invented instants.
pub(crate) trait Machine: Send {
    fn step(
        &mut self,
        io: &mut dyn I2c,
        now: Instant,
        publish: &mut dyn FnMut(HostImuSample),
    ) -> Step;
    fn state(&self) -> State;
    fn epoch(&self) -> u32;
    /// Best effort on the way out: undo what this reader turned on. RAM only.
    fn stop(&mut self, io: &mut dyn I2c);
}

/// A stream with nothing new for this long is a fault, closed and reopened. A BNO08x's 2 s is
/// servo-web's `imu_i2c.py`; an LSM6DSV16X's FIFO is read directly, so a second is plenty.
const BNO08X_STALL: Duration = Duration::from_secs(2);
const LSM6DSV16X_STALL: Duration = Duration::from_secs(1);

/// Between reads of an empty BNO08x: the Python reader's interval. Two reports a tick at 100 Hz.
const BNO08X_POLL: Duration = Duration::from_millis(2);
/// After a soft reset, before talking to the chip again. Its reset-complete and advertisement
/// arrive in this time and are drained.
const BNO08X_RESET_SETTLE: Duration = Duration::from_millis(300);
/// After the chip announces a reset of its own, before re-enabling reports.
const BNO08X_REENABLE_DELAY: Duration = Duration::from_millis(50);
const BNO08X_ID_TIMEOUT: Duration = Duration::from_secs(1);
/// Packets read before giving up on draining a chip that keeps talking.
const BNO08X_DRAIN_LIMIT: usize = 64;
/// Packets read per step while streaming, so a backlog is cleared without starving `stop`.
const BNO08X_PACKETS_PER_STEP: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
enum BnoPhase {
    Drain,
    Reset,
    AfterReset,
    RequestId,
    AwaitId { deadline: Instant },
    Enable,
    Stream { last_sample: Instant },
}

/// BNO085/BNO086: drain → soft reset → product ID → enable the game rotation vector and the
/// calibrated gyro at 100 Hz → stream. A reset the chip announces by itself is met with the
/// same enable, as a new epoch.
pub(crate) struct Bno08x {
    phase: BnoPhase,
    writer: Writer,
    assembler: Bno08xAssembler,
    enabled_once: bool,
    io_run: u32,
}

impl Bno08x {
    pub(crate) fn new(epoch: u32, now: Instant) -> Self {
        Self {
            phase: BnoPhase::Drain,
            writer: Writer::default(),
            assembler: Bno08xAssembler::new(epoch, now),
            enabled_once: false,
            io_run: 0,
        }
    }

    /// [`Self::next_packet`], telling a failed transfer from bytes that made no sense.
    fn next_packet_classified(io: &mut dyn I2c) -> Result<Option<(Header, Vec<u8>)>, Fault> {
        let mut first = [0u8; bno08x::HEADER_LEN];
        read_exactly(io, &mut first, "SHTP header").map_err(Fault::Io)?;
        let header = Header::parse(first).map_err(|e| Fault::Garbled(e.to_string()))?;
        if header.length == 0 {
            return Ok(None);
        }
        let mut body = vec![0u8; header.length];
        read_exactly(io, &mut body, "SHTP packet").map_err(Fault::Io)?;
        Packet::check(&header, &body).map_err(|e| Fault::Garbled(e.to_string()))?;
        Ok(Some((header, body)))
    }

    /// Read until the chip has nothing more or this step's budget is spent. Only a step that
    /// emptied the chip publishes, and only its newest sample: a report read out of a backlog
    /// was measured before it was read, and stamping it with this step's time would pass an old
    /// orientation off as current. A report that makes no sense is dropped and the rest read.
    fn stream(
        &mut self,
        io: &mut dyn I2c,
        now: Instant,
        mut last_sample: Instant,
        publish: &mut dyn FnMut(HostImuSample),
    ) -> Result<Step, Fault> {
        let mut newest = None;
        let mut drained = false;
        let mut garbled = None;
        for _ in 0..BNO08X_PACKETS_PER_STEP {
            let Some((header, body)) = Self::next_packet_classified(io)? else {
                drained = true;
                break;
            };
            let packet =
                Packet::check(&header, &body).map_err(|e| Fault::Garbled(e.to_string()))?;
            if packet.is_reset_complete() {
                self.phase = BnoPhase::Enable;
                return Ok(Step::Restart(
                    "BNO08x reset itself; re-enabling its reports".into(),
                    BNO08X_REENABLE_DELAY,
                ));
            }
            match self.assembler.feed(&packet, now) {
                Ok(Some(sample)) => newest = Some(sample),
                Ok(None) => {}
                Err(e) => garbled = Some(e.to_string()),
            }
        }
        if drained && let Some(sample) = newest {
            publish(sample);
            last_sample = now;
        }
        self.phase = BnoPhase::Stream { last_sample };
        let wait = if drained { BNO08X_POLL } else { Duration::ZERO };
        Ok(match garbled {
            Some(reason) => Step::Dropped(reason, wait),
            None => Step::Again(wait),
        })
    }

    /// The next whole packet, or `None` when the chip has nothing to say.
    fn next_packet(io: &mut dyn I2c) -> Result<Option<(Header, Vec<u8>)>, String> {
        let mut first = [0u8; bno08x::HEADER_LEN];
        read_exactly(io, &mut first, "SHTP header")?;
        let header = Header::parse(first).map_err(|e| e.to_string())?;
        if header.length == 0 {
            return Ok(None);
        }
        let mut body = vec![0u8; header.length];
        read_exactly(io, &mut body, "SHTP packet")?;
        Packet::check(&header, &body).map_err(|e| e.to_string())?;
        Ok(Some((header, body)))
    }

    fn drain(io: &mut dyn I2c) -> Result<(), String> {
        for _ in 0..BNO08X_DRAIN_LIMIT {
            if Self::next_packet(io)?.is_none() {
                break;
            }
        }
        Ok(())
    }

    fn try_step(
        &mut self,
        io: &mut dyn I2c,
        now: Instant,
        publish: &mut dyn FnMut(HostImuSample),
    ) -> Result<Step, String> {
        Ok(match self.phase {
            BnoPhase::Drain => {
                // Whatever an earlier run left queued — an advertisement, old reports.
                Self::drain(io)?;
                self.phase = BnoPhase::Reset;
                Step::Again(Duration::ZERO)
            }
            BnoPhase::Reset => {
                write_all(io, &self.writer.soft_reset(), "soft reset")?;
                self.phase = BnoPhase::AfterReset;
                Step::Again(BNO08X_RESET_SETTLE)
            }
            BnoPhase::AfterReset => {
                Self::drain(io)?;
                self.phase = BnoPhase::RequestId;
                Step::Again(Duration::ZERO)
            }
            BnoPhase::RequestId => {
                write_all(io, &self.writer.product_id_request(), "product ID request")?;
                self.phase = BnoPhase::AwaitId {
                    deadline: now + BNO08X_ID_TIMEOUT,
                };
                Step::Again(BNO08X_POLL)
            }
            BnoPhase::AwaitId { deadline } => {
                while let Some((header, body)) = Self::next_packet(io)? {
                    if header.channel != bno08x::channel::CONTROL {
                        continue;
                    }
                    let packet = Packet::check(&header, &body).map_err(|e| e.to_string())?;
                    let replies =
                        bno08x::control_replies(packet.payload).map_err(|e| e.to_string())?;
                    if let Some(ControlReply::ProductId(id)) = replies
                        .into_iter()
                        .find(|reply| matches!(reply, ControlReply::ProductId(_)))
                    {
                        tracing::info!(
                            part = id.part_number,
                            version = ?id.version,
                            build = id.build,
                            reset_cause = id.reset_cause,
                            "trunk IMU: BNO08x identified"
                        );
                        self.phase = BnoPhase::Enable;
                        return Ok(Step::Again(Duration::ZERO));
                    }
                }
                if now >= deadline {
                    return Err(
                        "no SH-2 product ID reply within 1 s; is a BNO08x at this address?".into(),
                    );
                }
                Step::Again(BNO08X_POLL)
            }
            BnoPhase::Enable => {
                for report in [bno08x::GAME_ROTATION_VECTOR, bno08x::GYROSCOPE] {
                    let packet = self.writer.set_feature(report, bno08x::REPORT_INTERVAL_US);
                    write_all(io, &packet, "set feature")?;
                }
                // The epoch starts when the reports do; the gate's settle time counts from here.
                if self.enabled_once {
                    self.assembler.restart(now);
                } else {
                    self.assembler = Bno08xAssembler::new(self.assembler.epoch(), now);
                    self.enabled_once = true;
                }
                self.phase = BnoPhase::Stream { last_sample: now };
                Step::Again(Duration::ZERO)
            }
            BnoPhase::Stream { last_sample } => {
                let result = self.stream(io, now, last_sample, publish);
                let step = resolve(&mut self.io_run, result, BNO08X_POLL);
                match self.phase {
                    // A step that failed early left the last sample where it was.
                    BnoPhase::Stream { last_sample } => watchdog(
                        step,
                        now,
                        last_sample,
                        BNO08X_STALL,
                        "BNO08x sent no new orientation",
                    ),
                    _ => step,
                }
            }
        })
    }
}

impl Machine for Bno08x {
    fn step(
        &mut self,
        io: &mut dyn I2c,
        now: Instant,
        publish: &mut dyn FnMut(HostImuSample),
    ) -> Step {
        self.try_step(io, now, publish).unwrap_or_else(Step::Failed)
    }

    fn state(&self) -> State {
        match self.phase {
            BnoPhase::Drain | BnoPhase::Reset | BnoPhase::AfterReset => State::Resetting,
            BnoPhase::RequestId | BnoPhase::AwaitId { .. } => State::Identifying,
            BnoPhase::Enable => State::Configuring,
            BnoPhase::Stream { .. } => State::Streaming,
        }
    }

    fn epoch(&self) -> u32 {
        self.assembler.epoch()
    }

    fn stop(&mut self, io: &mut dyn I2c) {
        if !self.enabled_once {
            return;
        }
        // servo-web's reader does the same on close: its reports off, nothing else touched.
        for report in [bno08x::GAME_ROTATION_VECTOR, bno08x::GYROSCOPE] {
            let packet = self.writer.set_feature(report, 0);
            if write_all(io, &packet, "set feature").is_err() {
                break;
            }
        }
    }
}

/// Between reads of an LSM6DSV16X FIFO with nothing to say: half a 120 Hz sample period.
const LSM6DSV16X_POLL: Duration = Duration::from_millis(4);
/// Between reads while waiting for the software reset to finish.
const LSM6DSV16X_POR_POLL: Duration = Duration::from_millis(5);
/// FIFO words read per step. Three arrive per 120 Hz sample; a backlog is cleared over a few
/// steps rather than in one long burst.
const LSM6DSV16X_WORDS_PER_STEP: u16 = 32;
/// A FIFO holding more than this — sixteen samples, about 133 ms — is history, not a stream to
/// catch up on: it is emptied and the stream starts over as a new epoch. The bus board's
/// firmware likewise refuses to call a backlog current (`imu.c:219-221`).
const LSM6DSV16X_BACKLOG_WORDS: u16 = 48;

#[derive(Debug, Clone, Copy, PartialEq)]
enum LsmPhase {
    BringUp {
        index: usize,
        poll_deadline: Option<Instant>,
    },
    Stream {
        last_sample: Instant,
    },
}

/// LSM6DSV16X: `lsm6dsv16x::BRING_UP` op by op — identity, software reset, the bus board's own
/// configuration and its read-backs — then the FIFO, one 7-byte word per read as the bus
/// board's firmware reads it. A FIFO that lost data is brought up again from the start.
pub(crate) struct Lsm6dsv16x {
    phase: LsmPhase,
    assembler: Lsm6dsv16xAssembler,
    brought_up: bool,
    io_run: u32,
}

impl Lsm6dsv16x {
    pub(crate) fn new(epoch: u32, now: Instant) -> Self {
        Self {
            phase: LsmPhase::BringUp {
                index: 0,
                poll_deadline: None,
            },
            assembler: Lsm6dsv16xAssembler::new(epoch, now),
            brought_up: false,
            io_run: 0,
        }
    }

    /// Empty the FIFO: bypass mode clears it, stream mode starts it again.
    fn flush(io: &mut dyn I2c) -> Result<(), String> {
        for mode in [0x00, 0x06] {
            let current = Self::register(io, lsm6dsv16x::FIFO_CTRL4)?;
            write_all(
                io,
                &[lsm6dsv16x::FIFO_CTRL4, (current & !0x07) | mode],
                "FIFO mode",
            )?;
        }
        Ok(())
    }

    fn register(io: &mut dyn I2c, register: u8) -> Result<u8, String> {
        let mut value = [0u8];
        io.read_register(register, &mut value)
            .map_err(|e| format!("read register {register:#04x}: {e}"))?;
        Ok(value[0])
    }

    fn bring_up(
        &mut self,
        io: &mut dyn I2c,
        now: Instant,
        index: usize,
        poll_deadline: Option<Instant>,
    ) -> Result<Step, String> {
        let Some(op) = lsm6dsv16x::BRING_UP.get(index).copied() else {
            if self.brought_up {
                self.assembler.restart(now);
            } else {
                self.assembler = Lsm6dsv16xAssembler::new(self.assembler.epoch(), now);
                self.brought_up = true;
            }
            self.phase = LsmPhase::Stream { last_sample: now };
            return Ok(Step::Again(Duration::ZERO));
        };
        let next = LsmPhase::BringUp {
            index: index + 1,
            poll_deadline: None,
        };
        let step = match op {
            Op::Write { register, value } => {
                write_all(io, &[register, value], "register write")?;
                Step::Again(Duration::ZERO)
            }
            Op::Update {
                register,
                mask,
                value,
            } => {
                let current = Self::register(io, register)?;
                write_all(io, &[register, (current & !mask) | value], "register write")?;
                Step::Again(Duration::ZERO)
            }
            Op::Expect {
                register,
                mask,
                value,
            } => {
                let got = Self::register(io, register)?;
                if got & mask != value {
                    return Err(if register == lsm6dsv16x::WHO_AM_I {
                        format!(
                            "WHO_AM_I reads {got:#04x}, not an LSM6DSV16X ({:#04x})",
                            lsm6dsv16x::ID
                        )
                    } else {
                        format!(
                            "bring-up read-back: register {register:#04x} is {got:#04x}, \
                             expected {value:#04x} under mask {mask:#04x}"
                        )
                    });
                }
                Step::Again(Duration::ZERO)
            }
            Op::Wait(duration) => Step::Again(duration),
            Op::Poll {
                register,
                mask,
                value,
                timeout,
            } => {
                let deadline = poll_deadline.unwrap_or(now + timeout);
                // A chip in the middle of resetting may not answer at all; that is the wait
                // this poll exists for, not a failure yet.
                if Self::register(io, register).is_ok_and(|got| got & mask == value) {
                    Step::Again(Duration::ZERO)
                } else if now >= deadline {
                    return Err(format!(
                        "register {register:#04x} did not settle to {value:#04x} under mask \
                         {mask:#04x} in {} ms (software reset not finished)",
                        timeout.as_millis()
                    ));
                } else {
                    self.phase = LsmPhase::BringUp {
                        index,
                        poll_deadline: Some(deadline),
                    };
                    return Ok(Step::Again(LSM6DSV16X_POR_POLL));
                }
            }
        };
        self.phase = next;
        Ok(step)
    }

    /// One look at the FIFO. Only a step that reached the FIFO's head publishes, and only its
    /// newest sample: a word read out of a backlog was measured before it was read, and
    /// stamping it with this step's time would pass an old orientation off as current.
    fn stream(
        &mut self,
        io: &mut dyn I2c,
        now: Instant,
        mut last_sample: Instant,
        publish: &mut dyn FnMut(HostImuSample),
    ) -> Result<Step, Fault> {
        let mut status = [0u8; 2];
        io.read_register(lsm6dsv16x::FIFO_STATUS1, &mut status)
            .map_err(|e| Fault::Io(format!("FIFO status: {e}")))?;
        let status = FifoStatus::parse(status);
        if status.lost_data() {
            self.phase = LsmPhase::BringUp {
                index: 0,
                poll_deadline: None,
            };
            return Ok(Step::Restart(
                "LSM6DSV16X FIFO overran and lost samples; bringing it up again".into(),
                Duration::ZERO,
            ));
        }
        if status.unread > LSM6DSV16X_BACKLOG_WORDS {
            Self::flush(io).map_err(Fault::Io)?;
            self.assembler.restart(now);
            self.phase = LsmPhase::Stream { last_sample: now };
            return Ok(Step::Restart(
                format!(
                    "LSM6DSV16X FIFO held {} words, ~{} ms behind; emptied, new epoch",
                    status.unread,
                    u64::from(status.unread) * 1000 / 360
                ),
                LSM6DSV16X_POLL,
            ));
        }
        let words = status.unread.min(LSM6DSV16X_WORDS_PER_STEP);
        let mut newest = None;
        for _ in 0..words {
            let mut word = [0u8; lsm6dsv16x::WORD_LEN];
            io.read_register(lsm6dsv16x::FIFO_DATA_OUT_TAG, &mut word)
                .map_err(|e| Fault::Io(format!("FIFO word: {e}")))?;
            newest = self.assembler.feed(&word, now).or(newest);
        }
        let at_head = status.unread <= words;
        if at_head && let Some(sample) = newest {
            publish(sample);
            last_sample = now;
        }
        self.phase = LsmPhase::Stream { last_sample };
        Ok(Step::Again(if at_head {
            LSM6DSV16X_POLL
        } else {
            Duration::ZERO
        }))
    }
}

impl Machine for Lsm6dsv16x {
    fn step(
        &mut self,
        io: &mut dyn I2c,
        now: Instant,
        publish: &mut dyn FnMut(HostImuSample),
    ) -> Step {
        match self.phase {
            LsmPhase::BringUp {
                index,
                poll_deadline,
            } => self
                .bring_up(io, now, index, poll_deadline)
                .unwrap_or_else(Step::Failed),
            LsmPhase::Stream { last_sample } => {
                let result = self.stream(io, now, last_sample, publish);
                let step = resolve(&mut self.io_run, result, LSM6DSV16X_POLL);
                match self.phase {
                    LsmPhase::Stream { last_sample } => watchdog(
                        step,
                        now,
                        last_sample,
                        LSM6DSV16X_STALL,
                        "LSM6DSV16X FIFO delivered no new game rotation",
                    ),
                    LsmPhase::BringUp { .. } => step,
                }
            }
        }
    }

    fn state(&self) -> State {
        match self.phase {
            // Before the software reset: is it the chip at all.
            LsmPhase::BringUp { index: 0..=1, .. } => State::Identifying,
            LsmPhase::BringUp { index: 2..=4, .. } => State::Resetting,
            LsmPhase::BringUp { .. } => State::Configuring,
            LsmPhase::Stream { .. } => State::Streaming,
        }
    }

    fn epoch(&self) -> u32 {
        self.assembler.epoch()
    }

    fn stop(&mut self, _io: &mut dyn I2c) {
        // Nothing to undo that outlives a power cycle, and the next reader resets it anyway.
    }
}

fn machine(kind: TrunkImuKind, epoch: u32, now: Instant) -> Box<dyn Machine> {
    match kind {
        TrunkImuKind::Bno08x => Box::new(Bno08x::new(epoch, now)),
        // `spawn_for` never starts a reader for the bus board; treated as the LSM's sibling
        // rather than a panic, and the bring-up's identity check fails it with a reason.
        TrunkImuKind::Lsm6dsv16x | TrunkImuKind::Bus => Box::new(Lsm6dsv16x::new(epoch, now)),
    }
}

/// What the reader thread and `HostImu` share. Atomics and swapped pointers only: the control
/// thread reads this every tick and must never wait on the reader.
struct Shared {
    kind: TrunkImuKind,
    location: String,
    sample: ArcSwapOption<HostImuSample>,
    state: AtomicU8,
    epoch: AtomicU32,
    samples: AtomicU64,
    errors: AtomicU64,
    dropped: AtomicU64,
    last_error: ArcSwapOption<String>,
}

impl Shared {
    fn new(kind: TrunkImuKind, location: String) -> Self {
        Self {
            kind,
            location,
            sample: ArcSwapOption::empty(),
            state: AtomicU8::new(State::Opening as u8),
            epoch: AtomicU32::new(0),
            samples: AtomicU64::new(0),
            errors: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            last_error: ArcSwapOption::empty(),
        }
    }

    fn set_state(&self, state: State) {
        self.state.store(state as u8, Ordering::Relaxed);
    }

    fn publish(&self, sample: HostImuSample) {
        self.sample.store(Some(Arc::new(sample)));
        self.samples.fetch_add(1, Ordering::Relaxed);
    }

    /// Counted every time; logged when the reason changes, so a missing module retried all
    /// night is one journal line and not thousands.
    fn fault(&self, reason: &str) {
        self.errors.fetch_add(1, Ordering::Relaxed);
        self.note(reason);
    }

    /// A transfer or packet dropped without a reopen.
    fn dropped(&self, reason: &str) {
        self.dropped.fetch_add(1, Ordering::Relaxed);
        self.note(reason);
    }

    fn note(&self, reason: &str) {
        let repeated = self
            .last_error
            .load()
            .as_deref()
            .is_some_and(|last| last == reason);
        if !repeated {
            tracing::warn!(
                imu = self.kind.label(),
                device = %self.location,
                %reason,
                "trunk IMU reader fault"
            );
        }
        self.last_error.store(Some(Arc::new(reason.to_owned())));
    }

    fn health(&self) -> HostImuHealth {
        let state = self.state.load(Ordering::Relaxed);
        HostImuHealth {
            state: State::ALL
                .get(usize::from(state))
                .map_or("unknown", |s| s.label()),
            epoch: self.epoch.load(Ordering::Relaxed),
            samples: self.samples.load(Ordering::Relaxed),
            errors: self.errors.load(Ordering::Relaxed),
            dropped: self.dropped.load(Ordering::Relaxed),
            last_error: self.last_error.load().as_deref().cloned(),
            fusion: self
                .sample
                .load()
                .as_deref()
                .and_then(|s| s.fusion.as_ref().map(FusionHealth::from)),
            raw_log: None,
        }
    }
}

/// Reopen backoff after a device fault. A bus glitch and a missing module look alike from here;
/// one doubling backoff serves both without hammering a shared bus.
pub(crate) struct Timing {
    retry_min: Duration,
    retry_max: Duration,
}

const TIMING: Timing = Timing {
    retry_min: Duration::from_millis(500),
    retry_max: Duration::from_secs(10),
};

/// Sleep, waking within 20 ms of `stop`.
fn pause(stop: &AtomicBool, duration: Duration) {
    let until = Instant::now() + duration;
    while !stop.load(Ordering::Relaxed) {
        let left = until.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return;
        }
        std::thread::sleep(left.min(Duration::from_millis(20)));
    }
}

type Opener = Box<dyn FnMut() -> Result<Box<dyn I2c + Send>, String> + Send>;

/// The reader thread: open, step the chip's machine until it fails or `stop`, back off, reopen.
fn run(kind: TrunkImuKind, mut open: Opener, shared: &Shared, stop: &AtomicBool, timing: &Timing) {
    let mut epoch = 0u32;
    let mut backoff = timing.retry_min;
    while !stop.load(Ordering::Relaxed) {
        shared.set_state(State::Opening);
        let mut io = match open() {
            Ok(io) => io,
            Err(reason) => {
                shared.fault(&reason);
                shared.set_state(State::Backoff);
                pause(stop, backoff);
                backoff = (backoff * 2).min(timing.retry_max);
                continue;
            }
        };
        epoch = epoch.wrapping_add(1);
        let mut chip = machine(kind, epoch, Instant::now());
        loop {
            shared.set_state(chip.state());
            shared.epoch.store(chip.epoch(), Ordering::Relaxed);
            if stop.load(Ordering::Relaxed) {
                chip.stop(io.as_mut());
                shared.set_state(State::Stopped);
                return;
            }
            match chip.step(io.as_mut(), Instant::now(), &mut |s| shared.publish(s)) {
                Step::Again(wait) => {
                    if chip.state() == State::Streaming {
                        backoff = timing.retry_min;
                    }
                    if !wait.is_zero() {
                        pause(stop, wait);
                    }
                }
                Step::Restart(reason, wait) => {
                    shared.fault(&reason);
                    pause(stop, wait);
                }
                Step::Dropped(reason, wait) => {
                    shared.dropped(&reason);
                    pause(stop, wait);
                }
                Step::Failed(reason) => {
                    shared.fault(&reason);
                    break;
                }
            }
        }
        epoch = chip.epoch();
        drop(io);
        shared.set_state(State::Backoff);
        pause(stop, backoff);
        backoff = (backoff * 2).min(timing.retry_max);
    }
    shared.set_state(State::Stopped);
}

struct Inner {
    shared: Arc<Shared>,
    stop: Arc<AtomicBool>,
    thread: Mutex<Option<std::thread::JoinHandle<()>>>,
}

impl Drop for Inner {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.lock().ok().and_then(|mut t| t.take()) {
            let _ = thread.join();
        }
    }
}

/// A running reader. Clones share it; the thread stops and is joined when the last one goes —
/// so the startup retry loop can hand a clone to each attempt and the I/O that opens keeps it.
#[derive(Clone)]
pub struct TrunkImuReader {
    inner: Arc<Inner>,
}

impl TrunkImuReader {
    /// Start reading `kind` at `address` on `device` (an i2c-dev node).
    pub fn spawn(kind: TrunkImuKind, device: &str, address: u8) -> io::Result<Self> {
        let location = format!("{device} @ {address:#04x}");
        Self::spawn_with(kind, opener(device.to_owned(), address), TIMING, location)
    }

    fn spawn_with(
        kind: TrunkImuKind,
        open: Opener,
        timing: Timing,
        location: String,
    ) -> io::Result<Self> {
        let shared = Arc::new(Shared::new(kind, location));
        let stop = Arc::new(AtomicBool::new(false));
        let (thread_shared, thread_stop) = (Arc::clone(&shared), Arc::clone(&stop));
        let thread = std::thread::Builder::new()
            .name(format!("imu-{}", kind.label()))
            .spawn(move || run(kind, open, &thread_shared, &thread_stop, &timing))?;
        Ok(Self {
            inner: Arc::new(Inner {
                shared,
                stop,
                thread: Mutex::new(Some(thread)),
            }),
        })
    }
}

impl HostImu for TrunkImuReader {
    fn kind(&self) -> TrunkImuKind {
        self.inner.shared.kind
    }

    fn latest(&self) -> Option<HostImuSample> {
        self.inner.shared.sample.load().as_deref().copied()
    }

    fn health(&self) -> HostImuHealth {
        self.inner.shared.health()
    }
}

#[cfg(target_os = "linux")]
fn opener(device: String, address: u8) -> Opener {
    Box::new(move || {
        dev::DevI2c::open(&device, address).map(|io| Box::new(io) as Box<dyn I2c + Send>)
    })
}

#[cfg(not(target_os = "linux"))]
fn opener(device: String, _address: u8) -> Opener {
    Box::new(move || Err(format!("{device}: an I²C trunk IMU needs Linux i2c-dev")))
}

#[cfg(target_os = "linux")]
mod dev {
    //! i2c-dev, as the kernel documents it (`Documentation/i2c/dev-interface.rst`).

    use std::io;
    use std::os::fd::AsRawFd;

    const I2C_SLAVE: libc::c_ulong = 0x0703;
    const I2C_RDWR: libc::c_ulong = 0x0707;
    const I2C_M_RD: u16 = 0x0001;

    #[repr(C)]
    struct I2cMsg {
        addr: u16,
        flags: u16,
        len: u16,
        buf: *mut u8,
    }

    #[repr(C)]
    struct I2cRdwrIoctlData {
        msgs: *mut I2cMsg,
        nmsgs: u32,
    }

    pub(super) struct DevI2c {
        file: std::fs::File,
        address: u16,
    }

    impl DevI2c {
        pub(super) fn open(device: &str, address: u8) -> Result<Self, String> {
            let file = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .open(device)
                .map_err(|e| {
                    let hint = match e.kind() {
                        io::ErrorKind::NotFound => {
                            " — is the i2c overlay for this bus enabled in the board's boot config?"
                        }
                        io::ErrorKind::PermissionDenied => {
                            " — robotd's unit needs DeviceAllow=char-i2c rw (packaging/runtime.py)"
                        }
                        _ => "",
                    };
                    format!("cannot open {device}: {e}{hint}")
                })?;
            let fd = file.as_raw_fd();
            // The same advisory lock servo-web's `imu_i2c.py` takes on the same node, so the
            // two can never both be configuring one chip.
            if unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) } != 0 {
                let e = io::Error::last_os_error();
                return Err(if e.kind() == io::ErrorKind::WouldBlock {
                    format!(
                        "{device} is held by another process: servo-web's IMU display \
                         (imu_i2c.py) locks it the same way; stop servo-web's IMU reader"
                    )
                } else {
                    format!("cannot lock {device}: {e}")
                });
            }
            if unsafe { libc::ioctl(fd, I2C_SLAVE, libc::c_ulong::from(address)) } < 0 {
                return Err(format!(
                    "cannot address {address:#04x} on {device}: {}",
                    io::Error::last_os_error()
                ));
            }
            Ok(Self {
                file,
                address: u16::from(address),
            })
        }
    }

    impl super::I2c for DevI2c {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            io::Read::read(&mut self.file, buf)
        }

        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            io::Write::write(&mut self.file, bytes)
        }

        fn read_register(&mut self, register: u8, buf: &mut [u8]) -> io::Result<()> {
            let len = u16::try_from(buf.len()).map_err(|_| io::Error::other("read too long"))?;
            let mut register = [register];
            let mut messages = [
                I2cMsg {
                    addr: self.address,
                    flags: 0,
                    len: 1,
                    buf: register.as_mut_ptr(),
                },
                I2cMsg {
                    addr: self.address,
                    flags: I2C_M_RD,
                    len,
                    buf: buf.as_mut_ptr(),
                },
            ];
            let mut data = I2cRdwrIoctlData {
                msgs: messages.as_mut_ptr(),
                nmsgs: 2,
            };
            // SAFETY: both messages point into buffers that outlive the call, with their true
            // lengths; the kernel copies through them and keeps nothing.
            if unsafe { libc::ioctl(self.file.as_raw_fd(), I2C_RDWR, &mut data) } < 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{HashMap, VecDeque};

    // ── A BNO08x on a fake bus ───────────────────────────────────────────────

    fn shtp(channel: u8, payload: &[u8]) -> Vec<u8> {
        let mut out = ((payload.len() + 4) as u16).to_le_bytes().to_vec();
        out.extend([channel, 0]);
        out.extend_from_slice(payload);
        out
    }
    fn report(id: u8, sequence: u8, values: &[i16]) -> Vec<u8> {
        let mut out = vec![id, sequence, 3, 0];
        for v in values {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }
    /// One input-report packet: a gyro and an upright game rotation, sequence `n`.
    fn reports(n: u8) -> Vec<u8> {
        let mut payload = vec![0xFB, 0, 0, 0, 0];
        payload.extend(report(bno08x::GYROSCOPE, n, &[512, 0, 0]));
        payload.extend(report(bno08x::GAME_ROTATION_VECTOR, n, &[0, 0, 0, 16384]));
        shtp(bno08x::channel::REPORTS, &payload)
    }
    fn product_id() -> Vec<u8> {
        let mut payload = vec![bno08x::PRODUCT_ID_RESPONSE, 1, 3, 2];
        payload.extend(10003608u32.to_le_bytes());
        payload.extend(372u32.to_le_bytes());
        payload.extend([7, 0, 0, 0]);
        shtp(bno08x::channel::CONTROL, &payload)
    }

    /// Packets queue up and come out two transfers at a time, header then whole packet, as
    /// the chip delivers them. Writes are recorded, and the ones a chip answers are answered.
    #[derive(Default)]
    struct FakeBno {
        queue: VecDeque<Vec<u8>>,
        body_next: bool,
        writes: Vec<Vec<u8>>,
        answers_id: bool,
        /// Reads still to fail, as a NACK would.
        fail_reads: u32,
        /// Every header reads all ones, as a wedged chip or a floating bus answers.
        garbage: bool,
        /// Alternate a NACK with a garbage header, read after read.
        flapping: bool,
        reads: u32,
    }

    impl I2c for FakeBno {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            if self.fail_reads > 0 {
                self.fail_reads -= 1;
                return Err(io::Error::other("Remote I/O error"));
            }
            if self.flapping {
                self.reads += 1;
                if self.reads % 2 == 1 {
                    return Err(io::Error::other("Remote I/O error"));
                }
                buf.fill(0xFF);
                return Ok(buf.len());
            }
            if self.garbage {
                buf.fill(0xFF);
                return Ok(buf.len());
            }
            if self.body_next {
                self.body_next = false;
                let packet = self
                    .queue
                    .pop_front()
                    .expect("a body read follows a header");
                buf.copy_from_slice(&packet[..buf.len()]);
                return Ok(buf.len());
            }
            match self.queue.front() {
                Some(packet) => {
                    buf.copy_from_slice(&packet[..4]);
                    self.body_next = true;
                }
                None => buf.fill(0),
            }
            Ok(buf.len())
        }
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            self.writes.push(bytes.to_vec());
            match (bytes[2], bytes.get(4)) {
                (bno08x::channel::EXECUTABLE, Some(&bno08x::EXECUTABLE_RESET)) => {
                    self.queue.clear();
                    self.queue
                        .push_back(shtp(bno08x::channel::COMMAND, &[0; 20]));
                    self.queue.push_back(shtp(
                        bno08x::channel::EXECUTABLE,
                        &[bno08x::EXECUTABLE_RESET],
                    ));
                }
                (bno08x::channel::CONTROL, Some(&bno08x::PRODUCT_ID_REQUEST))
                    if self.answers_id =>
                {
                    self.queue.push_back(product_id());
                }
                _ => {}
            }
            Ok(bytes.len())
        }
        fn read_register(&mut self, _: u8, _: &mut [u8]) -> io::Result<()> {
            Err(io::Error::other("a BNO08x has no registers"))
        }
    }

    /// Step until the machine streams, as the runner would, on invented time.
    fn until_streaming(
        chip: &mut dyn Machine,
        io: &mut dyn I2c,
        mut now: Instant,
    ) -> Result<Instant, String> {
        // Enough for a whole bring-up, or a one-second timeout polled every 2 ms.
        for _ in 0..1000 {
            if chip.state() == State::Streaming {
                return Ok(now);
            }
            match chip.step(io, now, &mut |_| {}) {
                Step::Again(wait) | Step::Restart(_, wait) | Step::Dropped(_, wait) => now += wait,
                Step::Failed(reason) => return Err(reason),
            }
        }
        Err("never reached streaming".into())
    }

    fn set_features(writes: &[Vec<u8>]) -> Vec<(u8, u32)> {
        writes
            .iter()
            .filter(|w| w.len() == 21 && w[2] == bno08x::channel::CONTROL && w[4] == 0xFD)
            .map(|w| (w[5], u32::from_le_bytes(w[9..13].try_into().unwrap())))
            .collect()
    }

    /// Drain whatever was queued, reset, identify, enable the two reports at 100 Hz, stream.
    /// The epoch starts when the reports do, and every new report pair is one sample.
    #[test]
    fn a_bno08x_is_reset_identified_enabled_and_streamed() {
        let t = Instant::now();
        let mut io = FakeBno {
            answers_id: true,
            ..FakeBno::default()
        };
        io.queue.push_back(reports(9));
        let mut chip = Bno08x::new(4, t);
        let now = until_streaming(&mut chip, &mut io, t).unwrap();
        assert_eq!(io.writes[0], [5, 0, 1, 0, 1], "soft reset first");
        assert_eq!(io.writes[1], [6, 0, 2, 0, 0xF9, 0], "then the product ID");
        assert_eq!(
            set_features(&io.writes),
            [
                (bno08x::GAME_ROTATION_VECTOR, 10_000),
                (bno08x::GYROSCOPE, 10_000)
            ]
        );
        let mut published = Vec::new();
        for n in 0..3 {
            io.queue.push_back(reports(n));
            chip.step(&mut io, now, &mut |s| published.push(s));
        }
        assert_eq!(published.len(), 3);
        assert_eq!(chip.state(), State::Streaming);
        assert_eq!(published[2].sequence, 3);
        assert_eq!(published[2].epoch, 4);
        assert_eq!(published[2].epoch_started_at, now);
        assert_eq!(published[2].gyro, [1.0, 0.0, 0.0]);
    }

    /// No product ID within a second is not a BNO08x, or not one that is listening.
    #[test]
    fn a_silent_bno08x_fails_identification() {
        let t = Instant::now();
        let mut io = FakeBno::default();
        let mut chip = Bno08x::new(1, t);
        let reason = until_streaming(&mut chip, &mut io, t).unwrap_err();
        assert!(reason.contains("product ID"), "{reason}");
    }

    /// A chip that resets by itself has forgotten its reports. It is answered in place —
    /// reports re-enabled, a new epoch — and the fault is counted, not reopened.
    #[test]
    fn a_bno08x_that_resets_itself_is_re_enabled_as_a_new_epoch() {
        let t = Instant::now();
        let mut io = FakeBno {
            answers_id: true,
            ..FakeBno::default()
        };
        let mut chip = Bno08x::new(1, t);
        let now = until_streaming(&mut chip, &mut io, t).unwrap();
        io.queue.push_back(shtp(
            bno08x::channel::EXECUTABLE,
            &[bno08x::EXECUTABLE_RESET],
        ));
        let step = chip.step(&mut io, now, &mut |_| {});
        assert!(
            matches!(step, Step::Restart(ref why, _) if why.contains("reset")),
            "{step:?}"
        );
        let later = now + Duration::from_millis(50);
        chip.step(&mut io, later, &mut |_| {});
        assert_eq!(chip.state(), State::Streaming);
        assert_eq!(
            set_features(&io.writes).len(),
            4,
            "both reports enabled again"
        );
        let mut published = Vec::new();
        io.queue.push_back(reports(0));
        chip.step(&mut io, later, &mut |s| published.push(s));
        assert_eq!(published[0].epoch, 2);
        assert_eq!(published[0].epoch_started_at, later);
    }

    /// Reports that stop is a fault: the device is closed and reopened, not waited on.
    #[test]
    fn a_bno08x_that_goes_quiet_fails() {
        let t = Instant::now();
        let mut io = FakeBno {
            answers_id: true,
            ..FakeBno::default()
        };
        let mut chip = Bno08x::new(1, t);
        let now = until_streaming(&mut chip, &mut io, t).unwrap();
        assert!(matches!(
            chip.step(&mut io, now + Duration::from_secs(1), &mut |_| {}),
            Step::Again(_)
        ));
        let step = chip.step(&mut io, now + Duration::from_millis(2001), &mut |_| {});
        assert!(
            matches!(step, Step::Failed(ref why) if why.contains("2 s")),
            "{step:?}"
        );
    }

    fn streaming_bno08x(t: Instant) -> (Bno08x, FakeBno, Instant) {
        let mut io = FakeBno {
            answers_id: true,
            ..FakeBno::default()
        };
        let mut chip = Bno08x::new(1, t);
        let now = until_streaming(&mut chip, &mut io, t).unwrap();
        (chip, io, now)
    }

    /// One malformed report is dropped and counted; the stream goes on and the next good report
    /// is published. Reopening over it would cost two seconds of trunk IMU for one bad packet.
    #[test]
    fn a_malformed_bno08x_report_is_dropped_and_the_stream_goes_on() {
        let t = Instant::now();
        let (mut chip, mut io, now) = streaming_bno08x(t);
        io.queue.push_back(shtp(
            bno08x::channel::REPORTS,
            &report(bno08x::GAME_ROTATION_VECTOR, 0, &[0, 0, 0, 0]),
        ));
        let step = chip.step(&mut io, now, &mut |_| {});
        assert!(
            matches!(step, Step::Dropped(ref why, _) if why.contains("quaternion")),
            "{step:?}"
        );
        assert_eq!(chip.state(), State::Streaming);
        let mut published = Vec::new();
        io.queue.push_back(reports(1));
        chip.step(&mut io, now, &mut |s| published.push(s));
        assert_eq!(published.len(), 1);
        assert_eq!(published[0].epoch, 1, "no new epoch for a dropped packet");
    }

    /// A failed transfer is noise until it repeats: four in a row are dropped, a success
    /// clears the run, and the fifth in a row closes the device.
    #[test]
    fn only_a_run_of_failed_bno08x_transfers_reopens() {
        let t = Instant::now();
        let (mut chip, mut io, now) = streaming_bno08x(t);
        io.fail_reads = 4;
        for _ in 0..4 {
            assert!(matches!(
                chip.step(&mut io, now, &mut |_| {}),
                Step::Dropped(..)
            ));
        }
        assert!(matches!(
            chip.step(&mut io, now, &mut |_| {}),
            Step::Again(_)
        ));
        io.fail_reads = 5;
        for _ in 0..4 {
            assert!(matches!(
                chip.step(&mut io, now, &mut |_| {}),
                Step::Dropped(..)
            ));
        }
        let step = chip.step(&mut io, now, &mut |_| {});
        assert!(
            matches!(step, Step::Failed(ref why) if why.contains("5 transfers failed in a row")),
            "{step:?}"
        );
    }

    /// Step a streaming chip the way the runner would, for `seconds` of invented time with at
    /// least `floor` between steps (the runner never waits less than a step takes). Returns the
    /// waits it asked for, and the failure with the time it came, if one did.
    fn run_for(
        chip: &mut dyn Machine,
        io: &mut dyn I2c,
        start: Instant,
        seconds: u64,
        floor: Duration,
    ) -> (Vec<Duration>, Option<(String, Duration)>) {
        let (mut now, mut waits) = (start, Vec::new());
        while now < start + Duration::from_secs(seconds) {
            match chip.step(io, now, &mut |_| {}) {
                Step::Again(wait) | Step::Restart(_, wait) | Step::Dropped(_, wait) => {
                    waits.push(wait);
                    now += wait.max(floor);
                }
                Step::Failed(reason) => return (waits, Some((reason, now - start))),
            }
        }
        (waits, None)
    }

    /// The reviewer's probe: a chip whose every header reads all ones. Each read is dropped at
    /// the poll interval, not spun on at zero wait, and with nothing published the stall
    /// watchdog reopens it two seconds in — it used to be skipped by the early return, so the
    /// reader read garbage at full speed forever, "streaming".
    #[test]
    fn a_bno08x_answering_garbage_is_polled_then_reopened_at_its_stall() {
        let t = Instant::now();
        let (mut chip, mut io, now) = streaming_bno08x(t);
        io.garbage = true;
        let (waits, failed) = run_for(&mut chip, &mut io, now, 10, Duration::from_millis(10));
        assert!(waits.iter().all(|w| *w >= BNO08X_POLL), "{waits:?}");
        let (reason, at) = failed.expect("never reopened");
        assert!(reason.contains("no new orientation for 2 s"), "{reason}");
        assert!(at <= Duration::from_millis(2_100), "{at:?}");
    }

    /// The reviewer's second probe: a NACK, then a garbage header, over and over. A garbled
    /// read used to clear the run of failed transfers, so the run never reached five and
    /// nothing reopened. Now the run counts on, and the stall bounds it anyway.
    #[test]
    fn nacks_between_garbage_still_add_up_to_a_reopen() {
        let t = Instant::now();
        let (mut chip, mut io, now) = streaming_bno08x(t);
        io.flapping = true;
        let (waits, failed) = run_for(&mut chip, &mut io, now, 10, Duration::from_millis(10));
        assert!(waits.iter().all(|w| *w >= BNO08X_POLL), "{waits:?}");
        let (reason, at) = failed.expect("never reopened");
        assert!(reason.contains("5 transfers failed in a row"), "{reason}");
        assert!(at <= Duration::from_millis(2_100), "{at:?}");
    }

    /// A step that spends its budget without emptying the chip publishes nothing: those
    /// reports were measured before they were read, and stamping them with this step's time
    /// would pass a backlog off as the current orientation. The step that empties it publishes
    /// its newest report once.
    #[test]
    fn a_bno08x_backlog_is_not_published_as_fresh() {
        let t = Instant::now();
        let (mut chip, mut io, now) = streaming_bno08x(t);
        for n in 0..12 {
            io.queue.push_back(reports(n));
        }
        let mut published = Vec::new();
        assert_eq!(
            chip.step(&mut io, now, &mut |s| published.push(s)),
            Step::Again(Duration::ZERO)
        );
        assert!(published.is_empty(), "backlog published as fresh");
        let later = now + Duration::from_millis(1);
        chip.step(&mut io, later, &mut |s| published.push(s));
        assert_eq!(published.len(), 1);
        assert_eq!(published[0].sequence, 12, "the newest report, once");
        assert_eq!(published[0].received_at, later);
    }

    /// On the way out the reader turns off what it turned on — and only that.
    #[test]
    fn stopping_a_bno08x_turns_its_reports_off() {
        let t = Instant::now();
        let mut io = FakeBno {
            answers_id: true,
            ..FakeBno::default()
        };
        let mut chip = Bno08x::new(1, t);
        until_streaming(&mut chip, &mut io, t).unwrap();
        chip.stop(&mut io);
        assert_eq!(
            set_features(&io.writes)[2..],
            [(bno08x::GAME_ROTATION_VECTOR, 0), (bno08x::GYROSCOPE, 0)]
        );
        let mut fresh = FakeBno::default();
        Bno08x::new(1, t).stop(&mut fresh);
        assert!(
            fresh.writes.is_empty(),
            "nothing enabled, nothing to turn off"
        );
    }

    // ── An LSM6DSV16X on a fake bus ──────────────────────────────────────────

    /// Two register banks behind FUNC_CFG_ACCESS, a software reset that completes when
    /// `por_sticks` says so, and a FIFO the test fills (or that fills itself, `streaming`).
    struct FakeLsm {
        main: HashMap<u8, u8>,
        embedded: HashMap<u8, u8>,
        func_cfg_access: u8,
        por_sticks: bool,
        fifo: VecDeque<[u8; 7]>,
        overrun: bool,
        streaming: bool,
        resets: u32,
        /// Register reads still to fail, as a NACK would.
        fail_reads: u32,
    }

    impl FakeLsm {
        fn new() -> Self {
            Self {
                main: HashMap::from([(lsm6dsv16x::WHO_AM_I, lsm6dsv16x::ID)]),
                embedded: HashMap::new(),
                func_cfg_access: 0,
                por_sticks: false,
                fifo: VecDeque::new(),
                overrun: false,
                streaming: false,
                resets: 0,
                fail_reads: 0,
            }
        }
        fn bank(&mut self) -> &mut HashMap<u8, u8> {
            if self.func_cfg_access & 0x80 != 0 {
                &mut self.embedded
            } else {
                &mut self.main
            }
        }
        fn word(tag: u8, data: [u8; 6]) -> [u8; 7] {
            let mut word = [tag << 3; 7];
            word[1..].copy_from_slice(&data);
            word
        }
        fn push_sample(&mut self) {
            self.fifo
                .push_back(Self::word(lsm6dsv16x::tag::GYRO, [0; 6]));
            self.fifo
                .push_back(Self::word(lsm6dsv16x::tag::ACCELEROMETER, [0; 6]));
            self.fifo
                .push_back(Self::word(lsm6dsv16x::tag::GAME_ROTATION, [0; 6]));
        }
    }

    impl I2c for FakeLsm {
        fn read(&mut self, _: &mut [u8]) -> io::Result<usize> {
            Err(io::Error::other("register reads only"))
        }
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            assert_eq!(bytes.len(), 2, "register writes are [register, value]");
            let (register, value) = (bytes[0], bytes[1]);
            if register == lsm6dsv16x::FUNC_CFG_ACCESS {
                self.func_cfg_access = value;
                if value & 0x04 != 0 {
                    self.resets += 1;
                    self.overrun = false;
                    self.fifo.clear();
                    if !self.por_sticks {
                        self.func_cfg_access = 0;
                    }
                }
            } else {
                // Bypass mode empties the FIFO, as on the chip.
                if register == lsm6dsv16x::FIFO_CTRL4
                    && value & 0x07 == 0
                    && self.func_cfg_access & 0x80 == 0
                {
                    self.fifo.clear();
                }
                self.bank().insert(register, value);
            }
            Ok(2)
        }
        fn read_register(&mut self, register: u8, buf: &mut [u8]) -> io::Result<()> {
            if self.fail_reads > 0 {
                self.fail_reads -= 1;
                return Err(io::Error::other("Remote I/O error"));
            }
            match register {
                lsm6dsv16x::FUNC_CFG_ACCESS => buf[0] = self.func_cfg_access,
                lsm6dsv16x::FIFO_STATUS1 => {
                    if self.streaming {
                        self.push_sample();
                    }
                    let unread = self.fifo.len() as u16;
                    buf[0] = unread as u8;
                    buf[1] = (unread >> 8) as u8 | if self.overrun { 0x40 } else { 0 };
                }
                lsm6dsv16x::FIFO_DATA_OUT_TAG => {
                    let word = self.fifo.pop_front().unwrap_or([0; 7]);
                    buf.copy_from_slice(&word);
                }
                _ => buf[0] = self.bank().get(&register).copied().unwrap_or(0),
            }
            Ok(())
        }
    }

    /// The whole bring-up, register by register, on the fake chip: it ends streaming, in the
    /// main bank, with the bus board's configuration, and every FIFO rotation that has a gyro
    /// before it is a sample of the epoch that began when streaming did.
    #[test]
    fn an_lsm6dsv16x_is_brought_up_and_its_fifo_streamed() {
        let t = Instant::now();
        let mut io = FakeLsm::new();
        let mut chip = Lsm6dsv16x::new(7, t);
        assert_eq!(chip.state(), State::Identifying);
        let now = until_streaming(&mut chip, &mut io, t).unwrap();
        assert_eq!(io.resets, 1);
        assert_eq!(io.func_cfg_access & 0xC0, 0, "ends in the main bank");
        assert_eq!(
            io.main[&lsm6dsv16x::FIFO_CTRL4] & 0x07,
            0x06,
            "FIFO streaming"
        );
        assert_eq!(io.embedded[&lsm6dsv16x::EMB_FUNC_EN_A] & 0x02, 0x02);
        io.push_sample();
        io.push_sample();
        let mut published = Vec::new();
        let step = chip.step(&mut io, now, &mut |s| published.push(s));
        assert_eq!(step, Step::Again(LSM6DSV16X_POLL));
        assert_eq!(published.len(), 1, "the newest of the step's samples");
        assert_eq!(published[0].sequence, 2);
        assert_eq!(published[0].quat, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(
            (published[0].epoch, published[0].epoch_started_at),
            (7, now)
        );
        assert_eq!(published[0].accuracy, None);
    }

    /// Something else at the address is refused at the first read, before a reset or a single
    /// configuration write reaches it.
    #[test]
    fn something_that_is_not_an_lsm6dsv16x_is_refused_untouched() {
        let t = Instant::now();
        let mut io = FakeLsm::new();
        io.main.insert(lsm6dsv16x::WHO_AM_I, 0x6C);
        let reason = until_streaming(&mut Lsm6dsv16x::new(1, t), &mut io, t).unwrap_err();
        assert!(reason.contains("WHO_AM_I reads 0x6c"), "{reason}");
        assert_eq!(io.resets, 0);
        assert_eq!(io.main.len(), 1, "nothing configured");
    }

    /// The reset bit is polled, not assumed: one that never clears fails after the firmware's
    /// 100 ms, and the steps until then wait rather than spin.
    #[test]
    fn a_software_reset_that_never_finishes_fails() {
        let t = Instant::now();
        let mut io = FakeLsm::new();
        io.por_sticks = true;
        let reason = until_streaming(&mut Lsm6dsv16x::new(1, t), &mut io, t).unwrap_err();
        assert!(reason.contains("software reset"), "{reason}");
    }

    /// A FIFO that overran has a hole in it. The chip is brought up again from the start, as
    /// a new epoch, and the fault is counted rather than reopened.
    #[test]
    fn an_lsm6dsv16x_fifo_overrun_brings_it_up_again_as_a_new_epoch() {
        let t = Instant::now();
        let mut io = FakeLsm::new();
        let mut chip = Lsm6dsv16x::new(1, t);
        let now = until_streaming(&mut chip, &mut io, t).unwrap();
        io.overrun = true;
        let step = chip.step(&mut io, now, &mut |_| {});
        assert!(
            matches!(step, Step::Restart(ref why, _) if why.contains("FIFO")),
            "{step:?}"
        );
        let later = until_streaming(&mut chip, &mut io, now).unwrap();
        assert_eq!(io.resets, 2);
        io.push_sample();
        let mut published = Vec::new();
        chip.step(&mut io, later, &mut |s| published.push(s));
        assert_eq!(published[0].epoch, 2);
    }

    /// An FIFO that stops filling is a fault, after a second.
    #[test]
    fn an_lsm6dsv16x_that_goes_quiet_fails() {
        let t = Instant::now();
        let mut io = FakeLsm::new();
        let mut chip = Lsm6dsv16x::new(1, t);
        let now = until_streaming(&mut chip, &mut io, t).unwrap();
        let step = chip.step(&mut io, now + Duration::from_millis(1001), &mut |_| {});
        assert!(
            matches!(step, Step::Failed(ref why) if why.contains("1 s")),
            "{step:?}"
        );
    }

    fn streaming_lsm6dsv16x(t: Instant) -> (Lsm6dsv16x, FakeLsm, Instant) {
        let mut io = FakeLsm::new();
        let mut chip = Lsm6dsv16x::new(1, t);
        let now = until_streaming(&mut chip, &mut io, t).unwrap();
        (chip, io, now)
    }

    /// A FIFO that has fallen a second behind — a stalled reader — is history. It is emptied
    /// and the stream starts over as a new epoch, so the gate holds it off; nothing from the
    /// backlog is published as current.
    #[test]
    fn an_lsm6dsv16x_backlog_is_emptied_as_a_new_epoch_not_published() {
        let t = Instant::now();
        let (mut chip, mut io, now) = streaming_lsm6dsv16x(t);
        for _ in 0..120 {
            io.push_sample();
        }
        let mut published = Vec::new();
        let step = chip.step(&mut io, now, &mut |s| published.push(s));
        assert!(
            matches!(step, Step::Restart(ref why, _) if why.contains("360 words")),
            "{step:?}"
        );
        assert!(
            published.is_empty(),
            "a second-old sample published as fresh"
        );
        assert!(io.fifo.is_empty());
        assert_eq!(
            io.main[&lsm6dsv16x::FIFO_CTRL4] & 0x07,
            0x06,
            "streaming again"
        );
        io.push_sample();
        let later = now + Duration::from_millis(5);
        chip.step(&mut io, later, &mut |s| published.push(s));
        assert_eq!(published[0].epoch, 2);
        assert_eq!(published[0].epoch_started_at, now);
    }

    /// A FIFO longer than one step's read is caught up over steps, publishing only once the
    /// head is reached — the newest sample, stamped with the step that reached it.
    #[test]
    fn an_lsm6dsv16x_publishes_only_from_the_fifo_head() {
        let t = Instant::now();
        let (mut chip, mut io, now) = streaming_lsm6dsv16x(t);
        for _ in 0..12 {
            io.push_sample();
        }
        let mut published = Vec::new();
        assert_eq!(
            chip.step(&mut io, now, &mut |s| published.push(s)),
            Step::Again(Duration::ZERO)
        );
        assert!(published.is_empty());
        let later = now + Duration::from_millis(1);
        chip.step(&mut io, later, &mut |s| published.push(s));
        assert_eq!(published.len(), 1);
        assert_eq!(published[0].sequence, 12);
        assert_eq!(published[0].received_at, later);
    }

    /// The same rule as the BNO08x: four failed reads in a row are dropped, the fifth closes.
    #[test]
    fn only_a_run_of_failed_lsm6dsv16x_reads_reopens() {
        let t = Instant::now();
        let (mut chip, mut io, now) = streaming_lsm6dsv16x(t);
        io.fail_reads = 4;
        for _ in 0..4 {
            assert!(matches!(
                chip.step(&mut io, now, &mut |_| {}),
                Step::Dropped(..)
            ));
        }
        assert!(matches!(
            chip.step(&mut io, now, &mut |_| {}),
            Step::Again(_)
        ));
        io.fail_reads = 5;
        for _ in 0..4 {
            chip.step(&mut io, now, &mut |_| {});
        }
        assert!(matches!(
            chip.step(&mut io, now, &mut |_| {}),
            Step::Failed(_)
        ));
    }

    // ── The thread ───────────────────────────────────────────────────────────

    const FAST: Timing = Timing {
        retry_min: Duration::from_millis(5),
        retry_max: Duration::from_millis(20),
    };

    fn wait_for(reader: &TrunkImuReader, what: &str, done: impl Fn(&TrunkImuReader) -> bool) {
        let until = Instant::now() + Duration::from_secs(5);
        while !done(reader) {
            assert!(Instant::now() < until, "{what}: {:?}", reader.health());
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    /// A device someone else holds is a fault named for who holds it, retried with backoff;
    /// once it opens, samples flow and the health says so. Dropping the last handle stops the
    /// thread and joins it.
    #[test]
    fn the_reader_retries_a_busy_device_then_streams_and_stops_on_drop() {
        let opens = Arc::new(AtomicU32::new(0));
        let counted = Arc::clone(&opens);
        let open: Opener = Box::new(move || {
            if counted.fetch_add(1, Ordering::Relaxed) < 2 {
                return Err(
                    "/dev/i2c-4 is held by another process: servo-web's IMU display".into(),
                );
            }
            let mut chip = FakeLsm::new();
            chip.streaming = true;
            Ok(Box::new(chip))
        });
        let reader =
            TrunkImuReader::spawn_with(TrunkImuKind::Lsm6dsv16x, open, FAST, "test".into())
                .unwrap();
        assert_eq!(reader.kind(), TrunkImuKind::Lsm6dsv16x);
        wait_for(&reader, "samples", |r| r.latest().is_some());
        let health = reader.health();
        assert_eq!(health.state, "streaming");
        assert_eq!(health.errors, 2);
        assert!(health.last_error.unwrap().contains("servo-web"));
        assert_eq!(health.epoch, 1);
        assert!(health.samples > 0);
        let clone = reader.clone();
        drop(reader);
        wait_for(&clone, "still running while a clone lives", |r| {
            r.health().samples > health.samples
        });
        let stop = Arc::clone(&clone.inner.stop);
        drop(clone);
        assert!(stop.load(Ordering::Relaxed), "last handle stops the thread");
    }

    /// A device that fails mid-stream is closed and reopened as a new epoch.
    #[test]
    fn a_device_fault_reopens_as_a_new_epoch() {
        let opens = Arc::new(AtomicU32::new(0));
        let counted = Arc::clone(&opens);
        let open: Opener = Box::new(move || {
            let mut chip = FakeLsm::new();
            // The first device answers bring-up and then never fills its FIFO.
            chip.streaming = counted.fetch_add(1, Ordering::Relaxed) > 0;
            Ok(Box::new(chip))
        });
        let reader =
            TrunkImuReader::spawn_with(TrunkImuKind::Lsm6dsv16x, open, FAST, "test".into())
                .unwrap();
        wait_for(&reader, "second epoch", |r| {
            r.latest().is_some_and(|s| s.epoch == 2)
        });
        let health = reader.health();
        assert!(health.last_error.unwrap().contains("no new game rotation"));
        assert_eq!(opens.load(Ordering::Relaxed), 2);
    }

    /// A dropped transfer is counted as dropped — not as a fault, and without a new epoch.
    #[test]
    fn dropped_transfers_are_counted_without_a_reopen() {
        let flaky = Arc::new(AtomicBool::new(false));
        let device_flaky = Arc::clone(&flaky);
        let open: Opener = Box::new(move || {
            struct Flaky {
                chip: FakeLsm,
                flaky: Arc<AtomicBool>,
                streamed: u32,
            }
            impl I2c for Flaky {
                fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
                    self.chip.read(buf)
                }
                fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
                    self.chip.write(bytes)
                }
                fn read_register(&mut self, register: u8, buf: &mut [u8]) -> io::Result<()> {
                    if register == lsm6dsv16x::FIFO_STATUS1 {
                        self.streamed += 1;
                        // Every tenth status read fails once the test says so.
                        if self.flaky.load(Ordering::Relaxed) && self.streamed.is_multiple_of(10) {
                            return Err(io::Error::other("Remote I/O error"));
                        }
                    }
                    self.chip.read_register(register, buf)
                }
            }
            let mut chip = FakeLsm::new();
            chip.streaming = true;
            Ok(Box::new(Flaky {
                chip,
                flaky: Arc::clone(&device_flaky),
                streamed: 0,
            }))
        });
        let reader =
            TrunkImuReader::spawn_with(TrunkImuKind::Lsm6dsv16x, open, FAST, "test".into())
                .unwrap();
        wait_for(&reader, "samples", |r| r.latest().is_some());
        flaky.store(true, Ordering::Relaxed);
        wait_for(&reader, "drops", |r| r.health().dropped >= 3);
        let health = reader.health();
        assert_eq!(health.errors, 0, "{health:?}");
        assert_eq!(health.epoch, 1);
        assert_eq!(health.state, "streaming");
    }

    #[test]
    fn the_params_source_names_its_reader() {
        assert_eq!(kind_of(TrunkImuSource::Bus), TrunkImuKind::Bus);
        assert_eq!(kind_of(TrunkImuSource::Bno08x), TrunkImuKind::Bno08x);
        assert_eq!(
            kind_of(TrunkImuSource::Lsm6dsv16x),
            TrunkImuKind::Lsm6dsv16x
        );
        assert!(
            spawn_for(&TrunkImuParams::default()).is_none(),
            "the bus needs no reader"
        );
    }
}
