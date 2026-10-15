// Added 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; BNO08x gyro-integrated attitude; see IMU-MODULE.md at the repository root.
//! A BNO08x (BNO085/BNO086) as bytes: SHTP framing and the SH-2 reports, with no I/O.
//!
//! The chip is a sensor hub speaking CEVA's SH-2 inside SHTP packets. This module builds the
//! few packets a host sends and parses what comes back; whoever owns the i2c-dev file (robotd's
//! reader thread, Linux only) is this codec plus reads, writes and timing. The subset is the one
//! microduck-replica's `tools/servo-web/imu_i2c.py` has run on a robot.
//!
//! **Reading is two transfers.** Every I²C read from the chip starts with the 4-byte SHTP
//! header of the packet it is holding, so a host reads 4 bytes to learn the length
//! ([`Header::parse`]), then reads the whole packet — header again, then payload — and
//! [`Packet::check`] holds the second transfer to what the first one promised.
//!
//! What the control loop uses is the game rotation vector (gyro and accelerometer, no
//! magnetometer, so a motor cannot drag its heading) and the calibrated gyroscope, both at
//! 100 Hz. [`Bno08xAssembler`] turns those into [`HostImuSample`]s.
//!
//! Beside that, every report also goes, one at a time and in the order the chip sent them, to
//! a [`GyroFusion`]: the gyro carried into an attitude of the host's own, the chip's
//! orientation checked against it (`gyro_fusion` says why). `[trunk_imu] bno08x_fusion` picks
//! which of the two a sample publishes; the default, `chip`, publishes exactly what it did
//! before the fusion existed, and the fusion's view rides along as diagnostics.

use std::panic::AssertUnwindSafe;
use std::time::{Duration, Instant};

use crate::gyro_fusion::{FusionCounts, FusionMode, FusionSample, GyroFusion, Verdict};
use crate::trunk_imu::HostImuSample;

/// Every SHTP packet starts with this many bytes: length (LE, bit 15 = continuation), channel,
/// sequence number.
pub const HEADER_LEN: usize = 4;
/// Longest packet accepted. Far beyond anything this chip sends here; a length past it is a
/// read that lost its framing — `0xFFFF` from a chip mid-reset, for one.
pub const MAX_PACKET_LEN: usize = 4096;

/// SHTP channels.
pub mod channel {
    /// SHTP's own: the advertisement a chip sends after it starts.
    pub const COMMAND: u8 = 0;
    /// Reset requests one way, "reset complete" the other.
    pub const EXECUTABLE: u8 = 1;
    /// SH-2 control: set feature, product ID, and their replies.
    pub const CONTROL: u8 = 2;
    /// Input reports.
    pub const REPORTS: u8 = 3;
    /// Input reports from wake-up sensors. Same format.
    pub const WAKE_REPORTS: u8 = 4;
}

/// Calibrated gyroscope, rad/s in Q9.
pub const GYROSCOPE: u8 = 0x02;
/// Game rotation vector: unit quaternion `i, j, k, real` in Q14.
pub const GAME_ROTATION_VECTOR: u8 = 0x08;
pub const PRODUCT_ID_RESPONSE: u8 = 0xF8;
pub const PRODUCT_ID_REQUEST: u8 = 0xF9;
pub const GET_FEATURE_RESPONSE: u8 = 0xFC;
pub const SET_FEATURE_COMMAND: u8 = 0xFD;
/// First payload byte on [`channel::EXECUTABLE`]: from the host, "reset"; from the chip,
/// "reset complete" — after which every report it was asked for is off again.
pub const EXECUTABLE_RESET: u8 = 0x01;

/// Report period asked for, µs. 100 Hz: two fresh samples per 50 Hz tick, so one lost
/// transfer still leaves a sample inside the 50 ms limit.
pub const REPORT_INTERVAL_US: u32 = 10_000;

/// Q14, Q9: the fixed-point scales of the two reports used.
const QUAT_SCALE: f64 = 16384.0;
const GYRO_SCALE: f64 = 512.0;

/// The step between two consecutive reports of one kind, seconds: what was asked for. The
/// chip's own confirmation is logged by the reader and the host clock's measurement is kept
/// ([`FusionSample::period_us`]); neither replaces this yet.
const REPORT_PERIOD_S: f64 = REPORT_INTERVAL_US as f64 / 1e6;
/// A gyro sequence step this large is still bridged — up to two reports lost, the arriving
/// rate held across the whole step. Larger is a hole in the stream.
const GYRO_BRIDGE: u8 = 3;
/// A sequence step past half the 8-bit range is a report older than the last one taken — a
/// re-read or one out of order — not 129 reports lost. Skipped.
const SEQUENCE_BACKWARDS: u8 = 128;
/// Host time between two gyro reports past which they are not one stream, whatever the
/// sequence says: it could have wrapped.
const GYRO_GAP_HOST: Duration = Duration::from_millis(100);
/// A game rotation vector's step counts for at most this many reports.
const GRV_MAX_STEPS: u8 = 10;
/// The report period is measured against the host clock over this long.
const PERIOD_WINDOW: Duration = Duration::from_secs(10);
/// A measured period further than this from the one asked for is logged.
const PERIOD_TOLERANCE: f64 = 0.05;

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Bno08xError {
    #[error("SHTP length {0} outside 4..=4096")]
    BadLength(usize),
    #[error("SHTP read returned {got} of {expected} bytes")]
    ShortRead { expected: usize, got: usize },
    #[error("SHTP packet changed between its header read and its body read ({0})")]
    PacketChanged(&'static str),
    #[error("unknown SH-2 report 0x{0:02X}; its length is what finds the next one")]
    UnknownReport(u8),
    #[error("SH-2 report 0x{0:02X} truncated")]
    TruncatedReport(u8),
    #[error("BNO08x quaternion length {0:.3}, outside 0.9..=1.1")]
    QuaternionNorm(f64),
}

/// An SHTP header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// Whole packet, header included. Zero: the chip has nothing to send.
    pub length: usize,
    /// The packet continues a transfer an earlier one began.
    pub continuation: bool,
    pub channel: u8,
    pub sequence: u8,
}

impl Header {
    /// The first transfer of a read.
    pub fn parse(bytes: [u8; HEADER_LEN]) -> Result<Self, Bno08xError> {
        let word = u16::from_le_bytes([bytes[0], bytes[1]]);
        let length = usize::from(word & 0x7FFF);
        if length != 0 && !(HEADER_LEN..=MAX_PACKET_LEN).contains(&length) {
            return Err(Bno08xError::BadLength(length));
        }
        Ok(Self {
            length,
            continuation: word & 0x8000 != 0,
            channel: bytes[2],
            sequence: bytes[3],
        })
    }
}

/// A whole packet, read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet<'a> {
    pub channel: u8,
    pub sequence: u8,
    pub payload: &'a [u8],
}

impl<'a> Packet<'a> {
    /// The second transfer of a read, against the header the first one returned. The chip
    /// re-sends the packet it is holding from its start, so anything else means the two
    /// transfers saw different packets and neither can be trusted.
    pub fn check(header: &Header, bytes: &'a [u8]) -> Result<Self, Bno08xError> {
        if header.length < HEADER_LEN {
            return Err(Bno08xError::BadLength(header.length));
        }
        if bytes.len() != header.length {
            return Err(Bno08xError::ShortRead {
                expected: header.length,
                got: bytes.len(),
            });
        }
        let restated = Header::parse([bytes[0], bytes[1], bytes[2], bytes[3]])?;
        if restated.length != header.length {
            return Err(Bno08xError::PacketChanged("length"));
        }
        if restated.channel != header.channel {
            return Err(Bno08xError::PacketChanged("channel"));
        }
        Ok(Self {
            channel: restated.channel,
            sequence: restated.sequence,
            payload: &bytes[HEADER_LEN..],
        })
    }

    /// The chip has (re)started, so every report it had been asked for is off.
    pub fn is_reset_complete(&self) -> bool {
        self.channel == channel::EXECUTABLE && self.payload.first() == Some(&EXECUTABLE_RESET)
    }
}

/// Packets from the host, each channel numbering its own.
#[derive(Debug, Default)]
pub struct Writer {
    sequence: [u8; 6],
}

impl Writer {
    fn packet(&mut self, channel: u8, payload: &[u8]) -> Vec<u8> {
        let slot = &mut self.sequence[usize::from(channel)];
        let length = (HEADER_LEN + payload.len()) as u16;
        let mut out = Vec::with_capacity(HEADER_LEN + payload.len());
        out.extend_from_slice(&length.to_le_bytes());
        out.extend_from_slice(&[channel, *slot]);
        out.extend_from_slice(payload);
        *slot = slot.wrapping_add(1);
        out
    }

    /// Restart the chip's firmware. Everything enabled is disabled; nothing is written to
    /// its flash.
    pub fn soft_reset(&mut self) -> Vec<u8> {
        self.packet(channel::EXECUTABLE, &[EXECUTABLE_RESET])
    }

    /// Ask the chip to name its firmware — the cheapest proof that what answers is a BNO08x.
    pub fn product_id_request(&mut self) -> Vec<u8> {
        self.packet(channel::CONTROL, &[PRODUCT_ID_REQUEST, 0])
    }

    /// Turn a report on at `interval_us`, or off at 0. RAM only: a reset forgets it.
    pub fn set_feature(&mut self, report_id: u8, interval_us: u32) -> Vec<u8> {
        let mut payload = [0u8; 17];
        payload[0] = SET_FEATURE_COMMAND;
        payload[1] = report_id;
        payload[5..9].copy_from_slice(&interval_us.to_le_bytes());
        self.packet(channel::CONTROL, &payload)
    }
}

/// Length of a report, from its first byte. SH-2 reports carry no length of their own, so an
/// ID missing here cannot be skipped and ends the parse.
///
/// Wider than the two reports this host asks for: another client of the same chip
/// (servo-web's display, say) may have left reports enabled, and those must pass by
/// harmlessly rather than stop the stream.
pub fn report_length(id: u8) -> Option<usize> {
    Some(match id {
        // Accelerometer, calibrated gyro, calibrated magnetometer, linear acceleration,
        // gravity.
        0x01..=0x04 | 0x06 => 10,
        // Rotation vector, geomagnetic rotation vector, AR/VR-stabilised rotation vector.
        0x05 | 0x09 | 0x28 => 14,
        // Uncalibrated gyro and magnetometer; raw accelerometer, gyro, magnetometer.
        0x07 | 0x0F | 0x14..=0x16 => 16,
        // Game rotation vector, AR/VR-stabilised game rotation vector.
        0x08 | 0x29 => 12,
        // Command response, product ID response.
        0xF1 | 0xF8 => 16,
        // Timestamp rebase, base timestamp reference.
        0xFA | 0xFB => 5,
        GET_FEATURE_RESPONSE => 17,
        _ => return None,
    })
}

/// The reports in one payload, in order.
pub fn reports(payload: &[u8]) -> Reports<'_> {
    Reports { payload }
}

pub struct Reports<'a> {
    payload: &'a [u8],
}

impl<'a> Iterator for Reports<'a> {
    type Item = Result<&'a [u8], Bno08xError>;

    fn next(&mut self) -> Option<Self::Item> {
        let id = *self.payload.first()?;
        let result = match report_length(id) {
            None => Err(Bno08xError::UnknownReport(id)),
            Some(length) if length > self.payload.len() => Err(Bno08xError::TruncatedReport(id)),
            Some(length) => {
                let (report, rest) = self.payload.split_at(length);
                self.payload = rest;
                return Some(Ok(report));
            }
        };
        // Nothing after a report of unknown length can be located.
        self.payload = &[];
        Some(result)
    }
}

/// The chip's firmware, as a product ID response names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProductId {
    /// Why the chip last reset: 1 power-on, 2 internal, 3 watchdog, 4 external, 5 other.
    pub reset_cause: u8,
    pub version: (u8, u8, u16),
    pub part_number: u32,
    pub build: u32,
}

/// A reply on the control channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlReply {
    /// One per firmware component; a chip sends several.
    ProductId(ProductId),
    /// The chip's statement of what a report is now set to — the confirmation a
    /// [`Writer::set_feature`] took.
    Feature { report_id: u8, interval_us: u32 },
    /// Anything else, by report ID.
    Other(u8),
}

/// Decode a control-channel payload.
pub fn control_replies(payload: &[u8]) -> Result<Vec<ControlReply>, Bno08xError> {
    reports(payload)
        .map(|report| {
            let report = report?;
            let u32_at = |at: usize| u32::from_le_bytes(report[at..at + 4].try_into().unwrap());
            Ok(match report[0] {
                PRODUCT_ID_RESPONSE => ControlReply::ProductId(ProductId {
                    reset_cause: report[1],
                    version: (
                        report[2],
                        report[3],
                        u16::from_le_bytes([report[12], report[13]]),
                    ),
                    part_number: u32_at(4),
                    build: u32_at(8),
                }),
                GET_FEATURE_RESPONSE => ControlReply::Feature {
                    report_id: report[1],
                    interval_us: u32_at(5),
                },
                other => ControlReply::Other(other),
            })
        })
        .collect()
}

fn i16_at(report: &[u8], at: usize) -> f64 {
    f64::from(i16::from_le_bytes([report[at], report[at + 1]]))
}

/// A game rotation vector report's orientation, scalar first, `w ≥ 0`, unit length; `None`
/// for one whose length is no rotation's.
fn grv_quat(report: &[u8]) -> Option<[f64; 4]> {
    let [x, y, z, w] = [4, 6, 8, 10].map(|at| i16_at(report, at) / QUAT_SCALE);
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    if !(0.9..=1.1).contains(&norm) {
        return None;
    }
    let sign = if w < 0.0 { -1.0 / norm } else { 1.0 / norm };
    Some([w * sign, x * sign, y * sign, z * sign])
}

fn gyro_rate(report: &[u8]) -> [f64; 3] {
    [4, 6, 8].map(|at| i16_at(report, at) / GYRO_SCALE)
}

/// The fusion's view of the last game rotation vector it took.
#[derive(Debug, Clone, Copy)]
struct Latest {
    verdict: Verdict,
    fused: [f64; 4],
    theta_deg: f64,
}

/// The report period against the host clock: report steps counted over [`PERIOD_WINDOW`].
#[derive(Debug, Default)]
struct Period {
    window: Option<(Instant, u64)>,
    steps: u64,
    measured_us: Option<f64>,
    off: bool,
}

impl Period {
    fn note(&mut self, steps: u8, now: Instant) {
        let Some((start, base)) = self.window else {
            // The first report anchors the window; its own step is before it.
            self.window = Some((now, self.steps));
            return;
        };
        self.steps += u64::from(steps);
        let elapsed = now.saturating_duration_since(start);
        if elapsed < PERIOD_WINDOW || self.steps == base {
            return;
        }
        let us = elapsed.as_secs_f64() * 1e6 / (self.steps - base) as f64;
        self.measured_us = Some(us);
        let off = (us / f64::from(REPORT_INTERVAL_US) - 1.0).abs() > PERIOD_TOLERANCE;
        if off != self.off {
            if off {
                tracing::warn!(
                    measured_us = us,
                    asked_us = REPORT_INTERVAL_US,
                    "BNO08x report period is not the one asked for; the gyro fusion integrates at the one asked for"
                );
            } else {
                tracing::info!(
                    measured_us = us,
                    "BNO08x report period is back to the one asked for"
                );
            }
            self.off = off;
        }
        self.window = Some((now, self.steps));
    }

    /// A hole: the next report starts a new window.
    fn reset(&mut self) {
        self.window = None;
    }
}

/// The fusion side of the assembler: every gyro and game rotation vector, in packet order,
/// stepped by its own sequence number.
#[derive(Debug)]
struct Fusion {
    mode: FusionMode,
    core: GyroFusion,
    last_gyro: Option<(u8, Instant)>,
    last_grv: Option<u8>,
    latest: Option<Latest>,
    counts: FusionCounts,
    /// It panicked once; it is never stepped again and samples carry the chip.
    disabled: bool,
    period: Period,
    #[cfg(test)]
    panic_next: bool,
}

impl Fusion {
    fn new(mode: FusionMode, counts: FusionCounts) -> Self {
        Self {
            mode,
            core: GyroFusion::new(),
            last_gyro: None,
            last_grv: None,
            latest: None,
            counts,
            disabled: counts.panics > 0,
            period: Period::default(),
            #[cfg(test)]
            panic_next: false,
        }
    }

    /// Every report in a payload that has already parsed whole. A panic in here is caught and
    /// turns the fusion off rather than ending the reader thread, which nothing restarts: a
    /// robot losing its trunk IMU mid-walk is worse than one losing a diagnostic.
    fn take_guarded(&mut self, payload: &[u8], now: Instant) {
        if self.disabled {
            return;
        }
        if std::panic::catch_unwind(AssertUnwindSafe(|| self.take(payload, now))).is_err() {
            self.disabled = true;
            self.counts.panics = self.counts.panics.saturating_add(1);
            tracing::error!(
                "BNO08x gyro fusion panicked; it is off, and samples carry the chip's own orientation"
            );
        }
    }

    fn take(&mut self, payload: &[u8], now: Instant) {
        #[cfg(test)]
        if std::mem::take(&mut self.panic_next) {
            panic!("injected by a test");
        }
        for report in reports(payload).map_while(Result::ok) {
            match report[0] {
                GYROSCOPE => self.gyro(report[1], gyro_rate(report), now),
                GAME_ROTATION_VECTOR => {
                    if let Some(quat) = grv_quat(report) {
                        self.grv(report[1], quat);
                    }
                }
                _ => {}
            }
        }
    }

    fn gyro(&mut self, sequence: u8, rate: [f64; 3], now: Instant) {
        let step = match self.last_gyro {
            None => {
                // An epoch's first rate: nothing to integrate from, and the fusion is following.
                self.last_gyro = Some((sequence, now));
                return;
            }
            Some((last, at)) => {
                let step = sequence.wrapping_sub(last);
                if step == 0 {
                    return;
                }
                if step > SEQUENCE_BACKWARDS {
                    self.counts.out_of_order = self.counts.out_of_order.saturating_add(1);
                    return;
                }
                (step <= GYRO_BRIDGE && now.saturating_duration_since(at) <= GYRO_GAP_HOST)
                    .then_some(step)
            }
        };
        self.last_gyro = Some((sequence, now));
        match step {
            Some(step) if self.core.gyro(rate, f64::from(step) * REPORT_PERIOD_S) => {
                self.counts.gyro = self.counts.gyro.saturating_add(1);
                self.period.note(step, now);
            }
            hole => {
                // A reading the core refused has already told it there is a hole.
                if hole.is_none() {
                    self.core.gap();
                }
                self.counts.gyro_gaps = self.counts.gyro_gaps.saturating_add(1);
                self.period.reset();
            }
        }
    }

    fn grv(&mut self, sequence: u8, quat: [f64; 4]) {
        let step = match self.last_grv {
            None => 1,
            Some(last) => {
                let step = sequence.wrapping_sub(last);
                if step == 0 {
                    return;
                }
                if step > SEQUENCE_BACKWARDS {
                    self.counts.out_of_order = self.counts.out_of_order.saturating_add(1);
                    return;
                }
                step
            }
        };
        self.last_grv = Some(sequence);
        let dt = f64::from(step.min(GRV_MAX_STEPS)) * REPORT_PERIOD_S;
        let verdict = self.core.chip(quat, dt);
        self.counts.note(verdict);
        self.latest = Some(Latest {
            verdict,
            fused: self.core.fused().unwrap_or(quat),
            theta_deg: self.core.theta_deg(),
        });
    }

    /// What a sample publishing `chip` says about the fusion.
    fn sample(&self, chip: [f64; 4]) -> Option<FusionSample> {
        let (verdict, fused, theta_deg) = if self.disabled {
            (Verdict::Disabled, chip, 0.0)
        } else {
            let latest = self.latest?;
            (latest.verdict, latest.fused, latest.theta_deg)
        };
        Some(FusionSample {
            mode: self.mode,
            verdict,
            chip_quat: chip,
            fused_quat: fused,
            theta_deg,
            counts: self.counts,
            period_us: self.period.measured_us,
        })
    }
}

/// Builds [`HostImuSample`]s from input-report packets.
///
/// One sample per payload that carries a new orientation, and only once a rate has arrived to
/// go with it. A report whose SH-2 sequence number repeats the last one taken is a re-read, not
/// a measurement, and refreshes nothing. A rate alone never publishes: it must not keep an old
/// orientation looking live.
///
/// Which orientation it publishes is the [`FusionMode`]'s choice; everything else about a
/// sample — when there is one, its rate, its age, its numbering — is the same in both.
#[derive(Debug)]
pub struct Bno08xAssembler {
    epoch: u32,
    epoch_started_at: Instant,
    sequence: u32,
    last_quat_report: Option<u8>,
    last_gyro_report: Option<u8>,
    quat: Option<([f64; 4], Instant)>,
    gyro: Option<([f64; 3], Instant)>,
    fusion: Fusion,
}

impl Bno08xAssembler {
    /// Publishing the chip's orientation, counting from zero.
    pub fn new(epoch: u32, now: Instant) -> Self {
        Self::with_fusion(epoch, now, FusionMode::Chip, FusionCounts::default())
    }

    /// Publishing `mode`'s orientation, its totals carried on from `counts` — a reader that
    /// reopened the device keeps counting, and a fusion that once panicked stays off.
    pub fn with_fusion(epoch: u32, now: Instant, mode: FusionMode, counts: FusionCounts) -> Self {
        Self {
            epoch,
            epoch_started_at: now,
            sequence: 0,
            last_quat_report: None,
            last_gyro_report: None,
            quat: None,
            gyro: None,
            fusion: Fusion::new(mode, counts),
        }
    }

    /// The chip's reports were (re)started — the device reopened, or the chip reset and was
    /// re-enabled. Nothing from before may be combined with what comes after; the fusion
    /// starts over too, and only its totals carry on.
    pub fn restart(&mut self, now: Instant) {
        *self = Self {
            sequence: self.sequence,
            ..Self::with_fusion(
                self.epoch.wrapping_add(1),
                now,
                self.fusion.mode,
                self.fusion.counts,
            )
        };
    }

    pub fn epoch(&self) -> u32 {
        self.epoch
    }

    /// The fusion's running totals.
    pub fn fusion_counts(&self) -> FusionCounts {
        self.fusion.counts
    }

    pub fn fusion_mode(&self) -> FusionMode {
        self.fusion.mode
    }

    /// Take one packet. Non-report channels pass through; a malformed payload is an error and
    /// changes nothing — the fusion included, which only sees a payload once all of it parsed.
    pub fn feed(
        &mut self,
        packet: &Packet<'_>,
        now: Instant,
    ) -> Result<Option<HostImuSample>, Bno08xError> {
        if !matches!(packet.channel, channel::REPORTS | channel::WAKE_REPORTS) {
            return Ok(None);
        }
        let mut quat = None;
        let mut gyro = None;
        for report in reports(packet.payload) {
            let report = report?;
            match report[0] {
                GAME_ROTATION_VECTOR => {
                    let [x, y, z, w] = [4, 6, 8, 10].map(|at| i16_at(report, at) / QUAT_SCALE);
                    let norm = (x * x + y * y + z * z + w * w).sqrt();
                    if !(0.9..=1.1).contains(&norm) {
                        return Err(Bno08xError::QuaternionNorm(norm));
                    }
                    // `q` and `-q` are one rotation; one sign keeps consecutive samples
                    // comparable component by component.
                    let sign = if w < 0.0 { -1.0 / norm } else { 1.0 / norm };
                    quat = Some((
                        report[1],
                        [w * sign, x * sign, y * sign, z * sign],
                        report[2] & 0x03,
                    ));
                }
                GYROSCOPE => {
                    let rate = [4, 6, 8].map(|at| i16_at(report, at) / GYRO_SCALE);
                    gyro = Some((report[1], rate));
                }
                _ => {}
            }
        }
        // The whole payload parsed. The fusion takes every report in it, in order; what is
        // published below is decided exactly as it was before the fusion existed.
        self.fusion.take_guarded(packet.payload, now);
        if let Some((sequence, rate)) = gyro
            && self.last_gyro_report != Some(sequence)
        {
            self.last_gyro_report = Some(sequence);
            self.gyro = Some((rate, now));
        }
        let Some((sequence, orientation, accuracy)) = quat else {
            return Ok(None);
        };
        if self.last_quat_report == Some(sequence) {
            return Ok(None);
        }
        self.last_quat_report = Some(sequence);
        self.quat = Some((orientation, now));
        let Some((rate, rate_at)) = self.gyro else {
            return Ok(None);
        };
        self.sequence = self.sequence.wrapping_add(1);
        let fusion = self.fusion.sample(orientation);
        let quat = match fusion {
            Some(f) if f.mode == FusionMode::Gyro && f.verdict != Verdict::Disabled => f.fused_quat,
            _ => orientation,
        };
        Ok(Some(HostImuSample {
            quat,
            gyro: rate,
            received_at: rate_at.min(now),
            sequence: self.sequence,
            epoch: self.epoch,
            epoch_started_at: self.epoch_started_at,
            accuracy: Some(accuracy),
            fusion,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    /// An SH-2 input report: id, sequence, status, delay, then little-endian words.
    fn report(id: u8, sequence: u8, values: &[i16]) -> Vec<u8> {
        let mut out = vec![id, sequence, 3, 0];
        for value in values {
            out.extend_from_slice(&value.to_le_bytes());
        }
        out
    }
    fn quat(sequence: u8, xyzw: [i16; 4]) -> Vec<u8> {
        report(GAME_ROTATION_VECTOR, sequence, &xyzw)
    }
    fn gyro(sequence: u8, xyz: [i16; 3]) -> Vec<u8> {
        report(GYROSCOPE, sequence, &xyz)
    }
    /// A base-timestamp reference, which leads every batch the chip sends.
    const BASE: [u8; 5] = [0xFB, 0, 0, 0, 0];
    fn packet(payload: &[u8]) -> Packet<'_> {
        Packet {
            channel: channel::REPORTS,
            sequence: 0,
            payload,
        }
    }
    const UPRIGHT: [i16; 4] = [0, 0, 0, 16384];

    #[test]
    fn a_header_is_length_continuation_channel_sequence() {
        let header = Header::parse([0x15, 0x80, 3, 9]).unwrap();
        assert_eq!(
            header,
            Header {
                length: 21,
                continuation: true,
                channel: 3,
                sequence: 9
            }
        );
        assert_eq!(Header::parse([0, 0, 0, 0]).unwrap().length, 0);
    }

    /// A chip mid-reset answers `FF FF`; three bytes cannot hold a header. Either is a read
    /// that lost its framing, and reading that many bytes next would only lose more.
    #[test]
    fn an_impossible_length_is_refused_before_the_body_is_read() {
        for bytes in [[0xFF, 0xFF, 0, 0], [3, 0, 2, 0], [0x01, 0x10, 2, 0]] {
            assert!(
                matches!(Header::parse(bytes), Err(Bno08xError::BadLength(_))),
                "{bytes:?}"
            );
        }
        assert!(
            Header::parse([0x00, 0x10, 2, 0]).is_ok(),
            "4096 is the limit"
        );
    }

    /// The body transfer must restate the header transfer. A short read, or a different
    /// packet, is refused rather than parsed as if it were the one announced.
    #[test]
    fn the_body_read_must_be_the_packet_the_header_announced() {
        let header = Header::parse([9, 0, 3, 1]).unwrap();
        let mut body = vec![9, 0, 3, 1];
        body.extend_from_slice(&BASE);
        let packet = Packet::check(&header, &body).unwrap();
        assert_eq!((packet.channel, packet.sequence), (3, 1));
        assert_eq!(packet.payload, BASE);

        assert_eq!(
            Packet::check(&header, &body[..8]),
            Err(Bno08xError::ShortRead {
                expected: 9,
                got: 8
            })
        );
        let mut moved = body.clone();
        moved[2] = 2;
        assert_eq!(
            Packet::check(&header, &moved),
            Err(Bno08xError::PacketChanged("channel"))
        );
        let mut longer = body.clone();
        longer[0] = 10;
        assert_eq!(
            Packet::check(&header, &longer),
            Err(Bno08xError::PacketChanged("length"))
        );
        let nothing = Header::parse([0, 0, 0, 0]).unwrap();
        assert_eq!(
            Packet::check(&nothing, &[]),
            Err(Bno08xError::BadLength(0)),
            "an empty header announces no packet to check"
        );
    }

    /// The exact bytes servo-web's reader sends (`imu_i2c.py`), which a chip on a robot has
    /// answered: a 21-byte set-feature on the control channel with the interval at offset 5.
    #[test]
    fn host_packets_are_the_bytes_the_python_reader_sends() {
        let mut writer = Writer::default();
        assert_eq!(
            writer.set_feature(GAME_ROTATION_VECTOR, REPORT_INTERVAL_US),
            [
                21, 0, 2, 0, 0xFD, 0x08, 0, 0, 0, 0x10, 0x27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
        assert_eq!(
            writer.set_feature(GYROSCOPE, 0)[..10],
            [21, 0, 2, 1, 0xFD, 0x02, 0, 0, 0, 0]
        );
        assert_eq!(writer.product_id_request(), [6, 0, 2, 2, 0xF9, 0]);
        // Each channel counts for itself.
        assert_eq!(writer.soft_reset(), [5, 0, 1, 0, 1]);
        assert_eq!(writer.soft_reset(), [5, 0, 1, 1, 1]);
    }

    #[test]
    fn a_channel_sequence_wraps() {
        let mut writer = Writer::default();
        for _ in 0..256 {
            writer.product_id_request();
        }
        assert_eq!(writer.product_id_request()[3], 0);
    }

    #[test]
    fn control_replies_name_the_firmware_and_confirm_features() {
        let mut payload = vec![PRODUCT_ID_RESPONSE, 1, 3, 2];
        payload.extend_from_slice(&10003608u32.to_le_bytes());
        payload.extend_from_slice(&372u32.to_le_bytes());
        payload.extend_from_slice(&[7, 0, 0, 0]);
        let mut feature = [0u8; 17];
        feature[0] = GET_FEATURE_RESPONSE;
        feature[1] = GAME_ROTATION_VECTOR;
        feature[5..9].copy_from_slice(&10_000u32.to_le_bytes());
        payload.extend_from_slice(&feature);
        payload.extend_from_slice(&BASE);
        assert_eq!(
            control_replies(&payload).unwrap(),
            [
                ControlReply::ProductId(ProductId {
                    reset_cause: 1,
                    version: (3, 2, 7),
                    part_number: 10003608,
                    build: 372,
                }),
                ControlReply::Feature {
                    report_id: GAME_ROTATION_VECTOR,
                    interval_us: 10_000
                },
                ControlReply::Other(0xFB),
            ]
        );
    }

    #[test]
    fn a_reset_announcement_is_recognised() {
        let reset = Packet {
            channel: channel::EXECUTABLE,
            sequence: 0,
            payload: &[EXECUTABLE_RESET],
        };
        assert!(reset.is_reset_complete());
        assert!(!packet(&[EXECUTABLE_RESET]).is_reset_complete());
    }

    /// A report this host never asked for is stepped over by its length; one of no known
    /// length cannot be, and stops the parse rather than misreading what follows.
    #[test]
    fn unknown_or_cut_reports_end_the_parse() {
        let mut payload = BASE.to_vec();
        payload.extend(report(0x01, 0, &[0, 0, 2511]));
        payload.extend(quat(0, UPRIGHT));
        assert_eq!(reports(&payload).filter_map(Result::ok).count(), 3);

        let mut unknown = BASE.to_vec();
        unknown.extend([0x42, 0, 0, 0]);
        unknown.extend(quat(0, UPRIGHT));
        let parsed: Vec<_> = reports(&unknown).collect();
        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[1], Err(Bno08xError::UnknownReport(0x42)));
    }

    /// `imu_i2c.py`'s `test_signed_w_and_real_packet_contract`: a base timestamp, then a
    /// quaternion whose real part is negative. `q` and `-q` are one rotation, published with
    /// `w ≥ 0`, scalar first.
    #[test]
    fn a_negative_real_part_is_published_as_the_same_rotation_with_w_positive() {
        let now = Instant::now();
        let mut a = Bno08xAssembler::new(1, now);
        let mut payload = BASE.to_vec();
        payload.extend(gyro(0, [0, 0, 0]));
        payload.extend(quat(0, [0, 0, 0, -16384]));
        let sample = a.feed(&packet(&payload), now).unwrap().unwrap();
        assert_eq!(sample.quat, [1.0, 0.0, 0.0, 0.0]);
        assert!(sample.quat[0].is_sign_positive());
        assert_eq!((sample.sequence, sample.epoch), (1, 1));
        assert_eq!(sample.accuracy, Some(3), "status byte 3: high accuracy");
        assert_eq!(sample.epoch_started_at, now);
    }

    /// `test_duplicate_reports_cannot_keep_orientation_live`: the same report sequence read
    /// twice is one measurement. The second must publish nothing, so the sample's age keeps
    /// growing and the gate refuses it.
    #[test]
    fn a_repeated_report_publishes_nothing() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut payload = gyro(0, [0, 0, 0]);
        payload.extend(quat(0, UPRIGHT));
        assert!(a.feed(&packet(&payload), t).unwrap().is_some());
        let later = t + Duration::from_millis(400);
        assert_eq!(a.feed(&packet(&payload), later).unwrap(), None);
    }

    /// `test_gyro_cannot_hide_stale_orientation`: a rate on its own is not a new sample.
    #[test]
    fn a_rate_alone_does_not_refresh_the_orientation() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut payload = gyro(0, [0, 0, 0]);
        payload.extend(quat(0, UPRIGHT));
        a.feed(&packet(&payload), t).unwrap();
        let later = t + Duration::from_secs(1);
        assert_eq!(
            a.feed(&packet(&gyro(1, [512, -512, 0])), later).unwrap(),
            None
        );
    }

    /// A fresh quaternion arriving with an old rate is only as fresh as the rate: the sample's
    /// age is the older of the two, which is what the gate judges.
    #[test]
    fn a_sample_is_as_old_as_its_older_half() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        a.feed(&packet(&gyro(0, [0, 0, 0])), t).unwrap();
        assert_eq!(
            a.feed(&packet(&[]), t).unwrap(),
            None,
            "an empty payload is no sample"
        );
        let later = t + Duration::from_millis(80);
        let sample = a.feed(&packet(&quat(0, UPRIGHT)), later).unwrap().unwrap();
        assert_eq!(sample.received_at, t);
    }

    /// `test_sequence_wrap_and_sensor_units`: the report sequence wraps 255 → 0 and that is a
    /// new report; Q9 512 is 1 rad/s; an accelerometer report another client left on passes by.
    #[test]
    fn sequences_wrap_and_the_units_are_si() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut first = gyro(255, [512, -512, 0]);
        first.extend(quat(255, UPRIGHT));
        let mut second = report(0x01, 0, &[0, 0, 2511]);
        second.extend(gyro(0, [512, -512, 0]));
        second.extend(quat(0, UPRIGHT));
        assert_eq!(a.feed(&packet(&first), t).unwrap().unwrap().sequence, 1);
        let sample = a.feed(&packet(&second), t).unwrap().unwrap();
        assert_eq!(sample.sequence, 2);
        assert_eq!(sample.gyro, [1.0, -1.0, 0.0]);
        assert_eq!(sample.quat, [1.0, 0.0, 0.0, 0.0]);
    }

    /// `test_bad_quaternion_and_truncated_packet_rejected`: a zero quaternion is no rotation,
    /// and a report one byte short is no report. Both are errors, and neither leaves anything
    /// behind that a later packet could publish.
    #[test]
    fn a_bad_quaternion_or_a_cut_report_is_an_error_that_changes_nothing() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut zero = gyro(0, [0, 0, 0]);
        zero.extend(quat(0, [0, 0, 0, 0]));
        assert!(matches!(
            a.feed(&packet(&zero), t),
            Err(Bno08xError::QuaternionNorm(_))
        ));
        let mut cut = gyro(1, [0, 0, 0]);
        cut.extend(quat(1, UPRIGHT));
        cut.pop();
        assert_eq!(
            a.feed(&packet(&cut), t),
            Err(Bno08xError::TruncatedReport(GAME_ROTATION_VECTOR))
        );
        assert_eq!(
            a.feed(&packet(&quat(2, UPRIGHT)), t).unwrap(),
            None,
            "the rejected payloads' gyro must not have been kept"
        );
    }

    /// Q14 is not exactly unit after rounding; the published quaternion is.
    #[test]
    fn the_published_quaternion_is_unit_length() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut payload = gyro(0, [0, 0, 0]);
        payload.extend(quat(0, [8192, 8192, 8192, 8200]));
        let q = a.feed(&packet(&payload), t).unwrap().unwrap().quat;
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-12, "{norm}");
    }

    /// Only input-report channels carry samples; control replies are the reader's business.
    #[test]
    fn other_channels_pass_through() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut payload = gyro(0, [0, 0, 0]);
        payload.extend(quat(0, UPRIGHT));
        let control = Packet {
            channel: channel::CONTROL,
            sequence: 0,
            payload: &payload,
        };
        assert_eq!(a.feed(&control, t).unwrap(), None);
    }

    /// After a restart the chip numbers its reports from scratch, and its fusion starts
    /// over. Nothing old may pair with something new, and the new epoch is stamped.
    #[test]
    fn a_restart_begins_a_new_epoch_with_nothing_carried_over() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut payload = gyro(0, [0, 0, 0]);
        payload.extend(quat(0, UPRIGHT));
        a.feed(&packet(&payload), t).unwrap().unwrap();
        let later = t + Duration::from_millis(300);
        a.restart(later);
        assert_eq!(a.epoch(), 2);
        assert_eq!(
            a.feed(&packet(&quat(0, UPRIGHT)), later).unwrap(),
            None,
            "no rate yet in this epoch"
        );
        let mut next = gyro(0, [0, 0, 0]);
        next.extend(quat(1, UPRIGHT));
        let sample = a.feed(&packet(&next), later).unwrap().unwrap();
        assert_eq!((sample.epoch, sample.epoch_started_at), (2, later));
        assert_eq!(sample.sequence, 2, "the host sequence keeps counting");
    }

    // ── The gyro fusion beside the publishing path ─────────────────────────────────────────

    use crate::gyro_fusion::tests::{apart_deg, axis_angle, turned};

    /// A unit quaternion in the report's Q14, `x y z w`.
    fn q14(q: [f64; 4]) -> [i16; 4] {
        [q[1], q[2], q[3], q[0]].map(|v| (v * QUAT_SCALE).round() as i16)
    }
    fn fused(a: &Bno08xAssembler) -> [f64; 4] {
        a.fusion.core.fused().unwrap()
    }

    /// The assembler exactly as it was before the fusion: publishing is held to this,
    /// field for field.
    struct Before {
        epoch: u32,
        epoch_started_at: Instant,
        sequence: u32,
        last_quat_report: Option<u8>,
        last_gyro_report: Option<u8>,
        gyro: Option<([f64; 3], Instant)>,
    }
    impl Before {
        fn new(epoch: u32, now: Instant) -> Self {
            Self {
                epoch,
                epoch_started_at: now,
                sequence: 0,
                last_quat_report: None,
                last_gyro_report: None,
                gyro: None,
            }
        }
        fn restart(&mut self, now: Instant) {
            *self = Self {
                sequence: self.sequence,
                ..Self::new(self.epoch.wrapping_add(1), now)
            };
        }
        fn feed(
            &mut self,
            packet: &Packet<'_>,
            now: Instant,
        ) -> Result<Option<HostImuSample>, Bno08xError> {
            if !matches!(packet.channel, channel::REPORTS | channel::WAKE_REPORTS) {
                return Ok(None);
            }
            let mut quat = None;
            let mut gyro = None;
            for report in reports(packet.payload) {
                let report = report?;
                match report[0] {
                    GAME_ROTATION_VECTOR => {
                        let [x, y, z, w] = [4, 6, 8, 10].map(|at| i16_at(report, at) / QUAT_SCALE);
                        let norm = (x * x + y * y + z * z + w * w).sqrt();
                        if !(0.9..=1.1).contains(&norm) {
                            return Err(Bno08xError::QuaternionNorm(norm));
                        }
                        let sign = if w < 0.0 { -1.0 / norm } else { 1.0 / norm };
                        quat = Some((
                            report[1],
                            [w * sign, x * sign, y * sign, z * sign],
                            report[2] & 0x03,
                        ));
                    }
                    GYROSCOPE => {
                        let rate = [4, 6, 8].map(|at| i16_at(report, at) / GYRO_SCALE);
                        gyro = Some((report[1], rate));
                    }
                    _ => {}
                }
            }
            if let Some((sequence, rate)) = gyro
                && self.last_gyro_report != Some(sequence)
            {
                self.last_gyro_report = Some(sequence);
                self.gyro = Some((rate, now));
            }
            let Some((sequence, orientation, accuracy)) = quat else {
                return Ok(None);
            };
            if self.last_quat_report == Some(sequence) {
                return Ok(None);
            }
            self.last_quat_report = Some(sequence);
            let Some((rate, rate_at)) = self.gyro else {
                return Ok(None);
            };
            self.sequence = self.sequence.wrapping_add(1);
            Ok(Some(HostImuSample {
                quat: orientation,
                gyro: rate,
                received_at: rate_at.min(now),
                sequence: self.sequence,
                epoch: self.epoch,
                epoch_started_at: self.epoch_started_at,
                accuracy: Some(accuracy),
                fusion: None,
            }))
        }
    }

    struct Random(u64);
    impl Random {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
        fn unit(&mut self) -> f64 {
            (self.next() >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    /// Packets the chip could send, and some it should not: several reports, repeats, steps
    /// back and forward and over the wrap, accelerometer and timestamp reports, unknown and
    /// cut reports, zero quaternions, empty payloads, control replies.
    fn random_payload(r: &mut Random, gyro_seq: &mut u8, grv_seq: &mut u8) -> (u8, Vec<u8>) {
        let channel = match r.below(20) {
            0 => channel::CONTROL,
            1 => channel::WAKE_REPORTS,
            _ => channel::REPORTS,
        };
        let mut payload = Vec::new();
        let step = |r: &mut Random, seq: &mut u8| {
            let by: u8 = match r.below(20) {
                0 | 1 => 0,
                2 => 2,
                3 => 4,
                4 => 200,
                5 => 255,
                _ => 1,
            };
            *seq = seq.wrapping_add(by);
            *seq
        };
        for _ in 0..r.below(5) {
            match r.below(20) {
                0..=6 => {
                    let seq = step(r, gyro_seq);
                    let full = r.below(30) == 0;
                    let v = [0; 3].map(|_: i16| {
                        if full {
                            [i16::MAX, i16::MIN][r.below(2) as usize]
                        } else {
                            (r.below(1200) as i16) - 600
                        }
                    });
                    payload.extend(gyro(seq, v));
                }
                7..=13 => {
                    let seq = step(r, grv_seq);
                    let q = if r.below(40) == 0 {
                        [0; 4]
                    } else {
                        q14(axis_angle(
                            [r.unit() - 0.5, r.unit() - 0.5, r.unit() - 0.5],
                            r.unit() * 40.0,
                        ))
                    };
                    payload.extend(quat(seq, q));
                }
                14..=16 => payload.extend(BASE),
                17 => payload.extend(report(0x01, 0, &[0, 0, 2511])),
                18 => payload.extend([0x42, 0, 0, 0]),
                _ => {
                    payload.extend(gyro(step(r, gyro_seq), [1, 2, 3]));
                    payload.pop();
                }
            }
        }
        (channel, payload)
    }

    /// M6: in `chip` mode the published samples are the old assembler's, field for field,
    /// over two hundred thousand random packets; in `gyro` mode everything but the orientation
    /// is. When there is a sample, its fusion view names the published orientation as the
    /// chip's.
    #[test]
    fn the_chip_mode_publishes_what_the_assembler_always_did() {
        for seed in [1u64, 0x9e37_79b9_7f4a_7c15, 0xdead_beef_cafe_f00d, 42] {
            let mut r = Random(seed);
            let t = Instant::now();
            let mut before = Before::new(1, t);
            let mut chip = Bno08xAssembler::new(1, t);
            let mut gyro_mode =
                Bno08xAssembler::with_fusion(1, t, FusionMode::Gyro, FusionCounts::default());
            let (mut now, mut gyro_seq, mut grv_seq) = (t, 0u8, 0u8);
            let mut published = 0;
            for _ in 0..50_000 {
                now += Duration::from_micros(r.below(15_000));
                if r.below(50) == 0 {
                    now += Duration::from_millis(200);
                }
                if r.below(2_000) == 0 {
                    before.restart(now);
                    chip.restart(now);
                    gyro_mode.restart(now);
                }
                let (channel, payload) = random_payload(&mut r, &mut gyro_seq, &mut grv_seq);
                let packet = Packet {
                    channel,
                    sequence: 0,
                    payload: &payload,
                };
                let want = before.feed(&packet, now);
                let got = chip.feed(&packet, now);
                let other = gyro_mode.feed(&packet, now);
                match (&want, &got, &other) {
                    (Ok(Some(w)), Ok(Some(g)), Ok(Some(o))) => {
                        published += 1;
                        let fusion = g.fusion.expect("a BNO08x sample carries its fusion view");
                        assert_eq!(HostImuSample { fusion: None, ..*g }, *w);
                        assert_eq!(fusion.chip_quat, g.quat);
                        assert_eq!(fusion.mode, FusionMode::Chip);
                        assert_eq!(
                            HostImuSample {
                                fusion: None,
                                quat: w.quat,
                                ..*o
                            },
                            *w
                        );
                        let fused = o.fusion.unwrap();
                        assert_eq!(fused.chip_quat, w.quat);
                        assert_eq!(o.quat, fused.fused_quat);
                        assert!(o.quat.iter().all(|v| v.is_finite()));
                    }
                    _ => {
                        assert_eq!(got, want);
                        assert_eq!(
                            other.as_ref().map(|s| s.is_some()),
                            want.as_ref().map(|s| s.is_some())
                        );
                    }
                }
            }
            assert!(published > 5_000, "{published}");
        }
    }

    /// The report the review named: `[gyro(7), gyro(6)]` after `gyro(6)`. The publishing path
    /// looks only at the last rate in a payload and keeps the old one; the fusion takes 7 and
    /// skips 6 as a step backwards. What is published does not change.
    #[test]
    fn a_rate_followed_by_an_older_one_publishes_as_before() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut payload = gyro(6, [0, 0, 100]);
        payload.extend(quat(0, UPRIGHT));
        a.feed(&packet(&payload), t).unwrap().unwrap();
        let mut payload = gyro(7, [0, 0, 300]);
        payload.extend(gyro(6, [0, 0, 200]));
        payload.extend(quat(1, UPRIGHT));
        let sample = a.feed(&packet(&payload), t).unwrap().unwrap();
        assert_eq!(
            sample.gyro,
            [0.0, 0.0, 100.0 / 512.0],
            "the old rate, as before"
        );
        assert_eq!(a.fusion_counts().out_of_order, 1);
        assert_eq!(a.fusion_counts().gyro, 1, "7 integrated, 6 skipped");
    }

    /// M2: the fusion's steps are report sequence numbers × 10 ms, never host time. Every
    /// packet here arrives at one host instant, as a backlog read in one step does, two game
    /// rotation vectors to a packet: the first second still ends after 100 reports.
    #[test]
    fn the_fusion_steps_by_report_sequence_not_host_time() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut verdicts = Vec::new();
        for n in 0..60u8 {
            let mut payload = gyro(2 * n, [0, 0, 0]);
            payload.extend(quat(2 * n, UPRIGHT));
            payload.extend(gyro(2 * n + 1, [0, 0, 0]));
            payload.extend(quat(2 * n + 1, UPRIGHT));
            a.feed(&packet(&payload), t).unwrap();
            verdicts.push(a.fusion.latest.unwrap().verdict);
        }
        let follows = verdicts.iter().filter(|v| **v == Verdict::Follow).count();
        assert!((49..=51).contains(&follows), "{follows} packets of two");
        assert_eq!(verdicts.last(), Some(&Verdict::Accepted));

        // Now the gyro alone: one report is 10 ms, a step of 2 is 20 ms held at its rate.
        let start = fused(&a);
        let rate = [0, 0, 512]; // 1 rad/s about z
        a.feed(&packet(&gyro(120, rate)), t).unwrap();
        a.feed(&packet(&gyro(122, rate)), t).unwrap();
        let turned = apart_deg(start, fused(&a));
        assert!((turned - 0.03f64.to_degrees()).abs() < 1e-9, "{turned}");
    }

    /// Steps past the bridge are holes; the 8-bit sequence wraps; a step backwards is a
    /// re-read; host time between two rates beyond 100 ms is a hole whatever the sequence says.
    #[test]
    fn sequence_steps_bridge_wrap_skip_and_break() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut payload = gyro(250, [0, 0, 0]);
        payload.extend(quat(0, UPRIGHT));
        a.feed(&packet(&payload), t).unwrap();
        let rate = [512, 0, 0];
        let mut angle: f64 = 0.0;
        for (seq, at, integrated, gaps, backwards) in [
            (253u8, 0u64, 0.03, 0, 0), // a step of 3: bridged
            (254, 0, 0.01, 0, 0),
            (2, 0, 0.0, 1, 0), // a step of 4: a hole, nothing integrated
            (3, 0, 0.01, 1, 0),
            (3, 0, 0.0, 1, 0),   // a re-read
            (200, 0, 0.0, 1, 1), // backwards
            (4, 0, 0.01, 1, 1),
            (5, 150, 0.0, 2, 1), // 150 ms of host time: a hole
            (6, 150, 0.01, 2, 1),
        ] {
            a.feed(&packet(&gyro(seq, rate)), t + Duration::from_millis(at))
                .unwrap();
            angle += integrated;
            let got = apart_deg([1.0, 0.0, 0.0, 0.0], fused(&a));
            assert!((got - angle.to_degrees()).abs() < 1e-9, "{seq}: {got}°");
            let counts = a.fusion_counts();
            assert_eq!(
                (counts.gyro_gaps, counts.out_of_order),
                (gaps, backwards),
                "{seq}"
            );
        }
        // Over the wrap: 255 → 0 is one step.
        let mut b = Bno08xAssembler::new(1, t);
        let mut payload = gyro(255, [0, 0, 0]);
        payload.extend(quat(0, UPRIGHT));
        b.feed(&packet(&payload), t).unwrap();
        b.feed(&packet(&gyro(0, rate)), t).unwrap();
        let got = apart_deg([1.0, 0.0, 0.0, 0.0], fused(&b));
        assert!((got - 0.01f64.to_degrees()).abs() < 1e-9);
        assert_eq!(b.fusion_counts().gyro_gaps, 0);
    }

    /// The same reports, the rate before the orientation or after it: the two differ by at most
    /// the turn of one report period (an orientation compared before its rate is integrated is
    /// a report behind), and the same order in packets of their own is the same attitude.
    #[test]
    fn the_order_of_reports_and_their_packing_barely_matter() {
        let t = Instant::now();
        let rate = [200, -100, 300];
        let w = rate.map(|v| f64::from(v) / GYRO_SCALE);
        let mut truth = [1.0, 0.0, 0.0, 0.0];
        let mut a = Bno08xAssembler::new(1, t);
        let mut b = Bno08xAssembler::new(1, t);
        let mut c = Bno08xAssembler::new(1, t);
        let speed = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
        for n in 0..200u32 {
            let s = n as u8;
            truth = turned(truth, w, 0.01);
            let q = q14(truth);
            let mut first = gyro(s, rate);
            first.extend(quat(s, q));
            let mut second = quat(s, q);
            second.extend(gyro(s, rate));
            a.feed(&packet(&first), t).unwrap();
            b.feed(&packet(&second), t).unwrap();
            c.feed(&packet(&gyro(s, rate)), t).unwrap();
            c.feed(&packet(&quat(s, q)), t).unwrap();
        }
        let one_period = (speed * 0.01).to_degrees();
        let apart = apart_deg(fused(&a), fused(&b));
        assert!(apart <= one_period + 0.01, "{apart}° against {one_period}°");
        assert!(
            apart_deg(fused(&a), fused(&c)) < 1e-12,
            "split is the same order as a"
        );
        assert!(apart_deg(fused(&a), truth) < 0.05);
    }

    /// A payload that fails to parse leaves the fusion exactly as it was, like the rest.
    #[test]
    fn a_bad_payload_leaves_the_fusion_as_it_was() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::new(1, t);
        let mut payload = gyro(0, [0, 0, 0]);
        payload.extend(quat(0, UPRIGHT));
        a.feed(&packet(&payload), t).unwrap();
        a.feed(&packet(&gyro(1, [300, 0, 0])), t).unwrap();
        let (counts, q, last) = (a.fusion_counts(), fused(&a), a.fusion.last_gyro);
        let mut zero = gyro(2, [900, 0, 0]);
        zero.extend(quat(1, [0, 0, 0, 0]));
        assert!(a.feed(&packet(&zero), t).is_err());
        let mut cut = gyro(2, [900, 0, 0]);
        cut.extend(quat(1, UPRIGHT));
        cut.pop();
        assert!(a.feed(&packet(&cut), t).is_err());
        assert_eq!(
            (a.fusion_counts(), fused(&a), a.fusion.last_gyro),
            (counts, q, last)
        );
    }

    /// `gyro` mode publishes the fusion's orientation, `chip` mode the chip's; both carry both.
    #[test]
    fn each_mode_publishes_its_own_orientation_and_carries_the_other() {
        let t = Instant::now();
        for mode in FusionMode::ALL {
            let mut a = Bno08xAssembler::with_fusion(1, t, mode, FusionCounts::default());
            let mut sample = None;
            for n in 0..150u8 {
                let mut payload = gyro(n, [512, 0, 0]);
                // The chip says 10° about y and stays there while the gyro says x turns.
                payload.extend(quat(n, q14(axis_angle([0.0, 1.0, 0.0], 10.0))));
                sample = a.feed(&packet(&payload), t).unwrap();
            }
            let sample = sample.unwrap();
            let fusion = sample.fusion.unwrap();
            assert_eq!(fusion.mode, mode);
            assert!(apart_deg(fusion.chip_quat, fusion.fused_quat) > 1.0);
            assert_eq!(sample.chip_quat(), fusion.chip_quat);
            match mode {
                FusionMode::Chip => assert_eq!(sample.quat, fusion.chip_quat),
                FusionMode::Gyro => assert_eq!(sample.quat, fusion.fused_quat),
            }
        }
    }

    /// M3: a panic inside the fusion is caught. The reader goes on, the fusion is off for good
    /// — across a restart too — and samples carry the chip's orientation in either mode.
    #[test]
    fn a_panic_turns_the_fusion_off_and_the_chip_is_published() {
        let t = Instant::now();
        for mode in FusionMode::ALL {
            let mut a = Bno08xAssembler::with_fusion(1, t, mode, FusionCounts::default());
            let mut payload = gyro(0, [0, 0, 0]);
            payload.extend(quat(0, UPRIGHT));
            a.feed(&packet(&payload), t).unwrap();
            a.fusion.panic_next = true;
            let tilted = q14(axis_angle([1.0, 0.0, 0.0], 20.0));
            let mut payload = gyro(1, [0, 0, 0]);
            payload.extend(quat(1, tilted));
            let sample = a.feed(&packet(&payload), t).unwrap().unwrap();
            let fusion = sample.fusion.unwrap();
            assert_eq!(fusion.verdict, Verdict::Disabled);
            assert_eq!(fusion.counts.panics, 1);
            assert_eq!(sample.quat, fusion.chip_quat);
            assert!(apart_deg(sample.quat, axis_angle([1.0, 0.0, 0.0], 20.0)) < 0.01);
            a.restart(t);
            let mut payload = gyro(0, [0, 0, 0]);
            payload.extend(quat(0, UPRIGHT));
            let sample = a.feed(&packet(&payload), t).unwrap().unwrap();
            assert_eq!(
                sample.fusion.unwrap().verdict,
                Verdict::Disabled,
                "{mode:?}"
            );
        }
    }

    /// A restart begins the fusion afresh — following the chip — and keeps its mode and totals.
    #[test]
    fn a_restart_keeps_the_mode_and_the_totals() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::with_fusion(1, t, FusionMode::Gyro, FusionCounts::default());
        for n in 0..5u8 {
            let mut payload = gyro(n, [0, 0, 0]);
            payload.extend(quat(n, UPRIGHT));
            a.feed(&packet(&payload), t).unwrap();
        }
        a.restart(t);
        assert_eq!(a.fusion_mode(), FusionMode::Gyro);
        assert_eq!(a.fusion_counts().follow, 5);
        let mut payload = gyro(0, [0, 0, 0]);
        payload.extend(quat(0, UPRIGHT));
        a.feed(&packet(&payload), t).unwrap();
        assert_eq!(a.fusion.latest.unwrap().verdict, Verdict::Follow);
        assert_eq!(a.fusion_counts().follow, 6);
        let carried = Bno08xAssembler::with_fusion(3, t, FusionMode::Chip, a.fusion_counts());
        assert_eq!(carried.fusion_counts(), a.fusion_counts());
    }

    /// Full-scale rates on every axis (Q9 ±64 rad/s) are integrated, not refused, and leave a
    /// finite unit attitude.
    #[test]
    fn full_scale_rates_leave_a_finite_attitude() {
        let t = Instant::now();
        let mut a = Bno08xAssembler::with_fusion(1, t, FusionMode::Gyro, FusionCounts::default());
        for n in 0..300u32 {
            let s = n as u8;
            let v = if n % 2 == 0 { i16::MAX } else { i16::MIN };
            let mut payload = gyro(s, [v, v, -v.saturating_add(1)]);
            payload.extend(quat(s, UPRIGHT));
            if let Some(sample) = a.feed(&packet(&payload), t).unwrap() {
                let norm = sample.quat.iter().map(|v| v * v).sum::<f64>().sqrt();
                assert!((norm - 1.0).abs() < 1e-9, "{n}: {:?}", sample.quat);
            }
        }
        assert_eq!(a.fusion_counts().gyro_gaps, 0);
        assert_eq!(a.fusion_counts().fallback, 0);
    }
}
