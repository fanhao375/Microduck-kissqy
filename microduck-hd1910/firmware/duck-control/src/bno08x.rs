// Added 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; see IMU-MODULE.md at the repository root.
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

use std::time::Instant;

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

/// Builds [`HostImuSample`]s from input-report packets.
///
/// One sample per payload that carries a new orientation, and only once a rate has arrived to
/// go with it. A report whose SH-2 sequence number repeats the last one taken is a re-read, not
/// a measurement, and refreshes nothing. A rate alone never publishes: it must not keep an old
/// orientation looking live.
#[derive(Debug)]
pub struct Bno08xAssembler {
    epoch: u32,
    epoch_started_at: Instant,
    sequence: u32,
    last_quat_report: Option<u8>,
    last_gyro_report: Option<u8>,
    quat: Option<([f64; 4], Instant)>,
    gyro: Option<([f64; 3], Instant)>,
}

impl Bno08xAssembler {
    pub fn new(epoch: u32, now: Instant) -> Self {
        Self {
            epoch,
            epoch_started_at: now,
            sequence: 0,
            last_quat_report: None,
            last_gyro_report: None,
            quat: None,
            gyro: None,
        }
    }

    /// The chip's reports were (re)started — the device reopened, or the chip reset and was
    /// re-enabled. Nothing from before may be combined with what comes after.
    pub fn restart(&mut self, now: Instant) {
        *self = Self {
            sequence: self.sequence,
            ..Self::new(self.epoch.wrapping_add(1), now)
        };
    }

    pub fn epoch(&self) -> u32 {
        self.epoch
    }

    /// Take one packet. Non-report channels pass through; a malformed payload is an error and
    /// changes nothing.
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
        Ok(Some(HostImuSample {
            quat: orientation,
            gyro: rate,
            received_at: rate_at.min(now),
            sequence: self.sequence,
            epoch: self.epoch,
            epoch_started_at: self.epoch_started_at,
            accuracy: Some(accuracy),
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
}
