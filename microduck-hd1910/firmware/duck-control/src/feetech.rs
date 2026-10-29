// Modified 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; optional servo P/D/torque-limit file; BNO08x gyro-integrated attitude; see IMU-MODULE.md at the repository root.
//! Native Feetech servo packets, with explicit or discovered bus-IMU wire/data contracts.
//!
//! HD1910 telemetry uses only the 15-byte feedback block at 56..=70.
//! FT6 shares that 15-byte block length; all 16 devices use one Sync Read. With the trunk
//! IMU on the board's I2C instead ([`crate::trunk_imu`]) that Sync Read is the 15 servos alone.
//! Full IMU age/sequence diagnostics remain at 124 and are not polled per tick.
//! The verified bench surface also includes
//! torque at 40, and the 6-byte position command at 42 (41 is preserved). Model-specific
//! current/gain units are deliberately not guessed from another Feetech model
//! or from XL330.
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::imu::{SensorReading, SflpDecoder, TrunkFrame};
use crate::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::{JOINT_NAMES, NUM_JOINTS};
use crate::trunk_imu::{HostImu, HostImuGate, HostImuHealth, TrunkImuKind};

#[path = "feetech_observations.rs"]
pub mod observations;

pub const BAUD: u32 = 1_000_000;
pub const IMU_ID: u8 = 200;
pub const SYNC_ADDRESS: u8 = 56;
pub const SYNC_LENGTH: u8 = 15;
pub const SERVO_READ_LENGTH: u8 = 15;
pub const IMU_DIAGNOSTIC_ADDRESS: u8 = 124;
pub const IMU_LENGTH: u8 = 15;
pub const IMU_DIAGNOSTIC_LENGTH: u8 = 20;
pub const IMU_IDENTITY: [u8; 5] = [6, 0, 0, 0, 0xf2];
/// Model number of microduck-replica's own bus IMU board (imu_to_dxl), little-endian at 3..5.
pub const IMU_TO_DXL_MODEL: [u8; 2] = [0x44, 0x4d];

/// Legacy boards are identified by their register bytes; Qinmo is an explicit configured
/// profile. The board/layout also decides its position in a Feetech Sync Read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum BusImuBoard {
    /// kissqy's FT6 board: first in the list, as its firmware expects.
    Ft6,
    /// microduck-replica's imu_to_dxl (Feetech protocol since firmware 0.2.0): last. It speaks
    /// after hearing the device before it, while Feetech servos start their predicted slots on a
    /// timer — first in the list, the next servo talks over its checksum (measured on firmware
    /// 1.0.0 with Feetech 3.46 servos: 3/30 replies first, 30/30 middle or last).
    ImuToDxl,
    /// DXL 2.0 SFLP register layout, ID 200, address 124, twelve bytes.
    Dxl2,
    /// Explicit protocol-1.2 data profile; PING and layout validation are not vendor authentication.
    Qinmo,
}
impl BusImuBoard {
    pub fn from_identity(raw: &[u8]) -> Option<Self> {
        if raw == IMU_IDENTITY {
            Some(Self::Ft6)
        } else if raw.len() == 5 && raw[2] == 0 && raw[3..5] == IMU_TO_DXL_MODEL {
            Some(Self::ImuToDxl)
        } else {
            None
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Ft6 => "ft6",
            Self::ImuToDxl => "imu_to_dxl",
            Self::Dxl2 => "sflp_124_12",
            Self::Qinmo => "qinmo",
        }
    }

    pub fn protocol(self) -> BusImuProtocol {
        if self == Self::Dxl2 {
            BusImuProtocol::Dxl2
        } else {
            BusImuProtocol::Feetech
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BusImuProtocol {
    #[default]
    Auto,
    Feetech,
    Dxl2,
}
impl BusImuProtocol {
    pub fn label(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Feetech => "feetech",
            Self::Dxl2 => "dxl2",
        }
    }
}

/// A configured data contract, independent of the on-wire protocol or vendor identity.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BusImuProfile {
    #[default]
    Legacy,
    Qinmo,
}
impl BusImuProfile {
    pub fn label(self) -> &'static str {
        match self {
            Self::Legacy => "legacy",
            Self::Qinmo => "qinmo",
        }
    }
    pub fn gyro_dps_per_lsb(self) -> f64 {
        match self {
            Self::Legacy => 0.0175,
            Self::Qinmo => 1.0 / 16.4,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct BusImuInfo {
    pub protocol: &'static str,
    pub board: &'static str,
    pub identity: Vec<u8>,
    pub profile: &'static str,
    pub gyro_dps_per_lsb: f64,
    pub identification: &'static str,
}

#[cfg(all(test, unix))]
#[path = "feetech_dxl2_tests.rs"]
mod dxl2_transport_tests;

#[cfg(all(test, unix))]
#[path = "feetech_qinmo_tests.rs"]
mod qinmo_transport_tests;

/// The Sync Read list for the 56/15 telemetry: the IMU first or last, by board.
pub fn telemetry_ids(board: BusImuBoard, servo_ids: &[u8]) -> Vec<u8> {
    let mut ids = Vec::with_capacity(servo_ids.len() + 1);
    if board == BusImuBoard::Ft6 {
        ids.push(IMU_ID);
    }
    ids.extend_from_slice(servo_ids);
    if matches!(board, BusImuBoard::ImuToDxl | BusImuBoard::Qinmo) {
        ids.push(IMU_ID);
    }
    ids
}

fn unknown_imu(raw: &[u8]) -> IoError {
    error(format!(
        "ID 200 is neither FT6 (06 00 00 00 f2) nor imu_to_dxl (model 0x4D44, firmware 0.2.0 or later); \
         it reported {raw:02x?}. If this is a Qinmo board, select Qinmo on the IMU page \
         while the main program is stopped, then verify its mounting again; the profile is not autodetected"
    ))
}
/// FT6 sets status bit1 at TX when quaternion age exceeds this bound.
pub const IMU_FRESH_BOUND_MS: u16 = 30;
pub const BROADCAST: u8 = 254;
pub const READ: u8 = 2;
pub const WRITE: u8 = 3;
pub const SYNC_READ: u8 = 0x82;
pub const SYNC_WRITE: u8 = 0x83;
/// Budget for one complete 50 Hz telemetry transaction.
/// Setup/torque verification and explicit read-only diagnostics keep their own budget.
pub const TELEMETRY_DEADLINE_MS: u64 = 20;
/// Full telemetry and its host-side age share one 20 ms budget.
/// Retained in diagnostics so consoles can interpret archived timing records.
pub const TELEMETRY_COMPLETE_MAX_AGE_MS: u64 = TELEMETRY_DEADLINE_MS;
pub const READ_TIMING_REVISION: &str = "r15-read-timing.2";
/// Budget for all fifteen torque/goal confirmations at a state transition.
/// Normal telemetry keeps its separate per-tick deadline.
pub const CONTROL_READBACK_DEADLINE_MS: u64 = 10;

fn error(message: impl Into<String>) -> IoError {
    IoError::Bus(message.into())
}

pub fn packet(id: u8, instruction: u8, params: &[u8]) -> Result<Vec<u8>> {
    if id == 255 || params.len() > 253 {
        return Err(error("Feetech invalid ID or packet length"));
    }
    let mut out = vec![255, 255, id, (params.len() + 2) as u8, instruction];
    out.extend_from_slice(params);
    let sum = out[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
    out.push(!sum);
    Ok(out)
}

#[derive(Debug, PartialEq)]
pub struct Status {
    pub id: u8,
    pub error: u8,
    pub data: Vec<u8>,
}

/// A bounded stream decoder, including resynchronisation after noise/checksum errors.
#[derive(Default)]
pub struct Decoder {
    bytes: Vec<u8>,
    expected_length: Option<u8>,
    pub bad_checksums: u64,
    pub discarded: u64,
}
impl Decoder {
    pub fn for_sync_read(length: u8) -> Self {
        Self {
            expected_length: length.checked_add(2),
            ..Self::default()
        }
    }
    pub fn push(&mut self, byte: u8) -> Option<Status> {
        self.bytes.push(byte);
        loop {
            if self.bytes.len() < 2 {
                return None;
            }
            if self.bytes[..2] != [255, 255] {
                self.bytes.remove(0);
                self.discarded += 1;
                continue;
            }
            if self.bytes.len() < 4 {
                return None;
            }
            if self.bytes[2] >= BROADCAST
                || self.bytes[3] < 2
                || self
                    .expected_length
                    .is_some_and(|length| self.bytes[3] != length)
            {
                self.bytes.remove(0);
                self.discarded += 1;
                continue;
            }
            let length = usize::from(self.bytes[3]) + 4;
            if self.bytes.len() < length {
                return None;
            }
            let sum = self.bytes[2..length]
                .iter()
                .fold(0u8, |a, b| a.wrapping_add(*b));
            if sum != 255 {
                self.bytes.remove(0);
                self.bad_checksums += 1;
                self.discarded += 1;
                continue;
            }
            let status = Status {
                id: self.bytes[2],
                error: self.bytes[4],
                data: self.bytes[5..length - 1].to_vec(),
            };
            self.bytes.drain(..length);
            return Some(status);
        }
    }
}

fn accept_sync_status(
    pending: &mut BTreeSet<u8>,
    replies: &mut BTreeMap<u8, Vec<u8>>,
    status: Status,
    length: u8,
) -> Result<bool> {
    if !pending.contains(&status.id) {
        return Err(error(format!(
            "unexpected/duplicate Feetech Sync Read reply from ID {}",
            status.id
        )));
    }
    if status.error != 0 {
        return Err(error(format!(
            "Feetech ID {} Sync Read status error 0x{:02x}",
            status.id, status.error
        )));
    }
    if status.data.len() != usize::from(length) {
        return Err(IoError::ShortRead {
            what: "Feetech Sync Read status",
            expected: usize::from(length),
            got: status.data.len(),
        });
    }
    pending.remove(&status.id);
    replies.insert(status.id, status.data);
    Ok(pending.is_empty())
}

pub struct Bus {
    port: Box<dyn serialport::SerialPort>,
    timeout: Duration,
    read_only: bool,
    pub last_sync: SyncTrace,
    /// The bus IMU board, once `identify_imu` has read it; FT6 until then.
    imu_board: BusImuBoard,
    imu_protocol: BusImuProtocol,
    imu_profile: BusImuProfile,
    imu_identity: Option<Vec<u8>>,
    observations: observations::Observations,
}

/// Minimal acquisition result. No recovery, raw-packet history or UART polling.
#[derive(Debug, Clone, Default, Serialize)]
pub struct SyncTrace {
    pub address: u8,
    pub length: u8,
    pub missing_ids: Vec<u8>,
    pub bad_checksums: u64,
    pub discarded: u64,
    pub elapsed_us: u64,
    pub rx_bytes: usize,
    pub deadline_exceeded: bool,
    pub complete_but_late: bool,
    /// False if the request could not be sent; do not blame each pending device.
    pub request_sent: bool,
    /// Valid IMU and a strict subset of servo replies; missing IDs stay visible.
    pub partial_received: bool,
}

impl Bus {
    pub fn open(path: &str) -> Result<Self> {
        let builder = serialport::new(path, BAUD).timeout(Duration::from_millis(2));
        #[cfg(unix)]
        let port: Box<dyn serialport::SerialPort> = {
            let mut tty = builder.open_native().map_err(|e| error(e.to_string()))?;
            tty.set_exclusive(true).map_err(|e| error(e.to_string()))?;
            Box::new(tty)
        };
        #[cfg(not(unix))]
        let port = builder.open().map_err(|e| error(e.to_string()))?;
        Ok(Self {
            port,
            timeout: Duration::from_millis(30),
            read_only: false,
            last_sync: SyncTrace::default(),
            imu_board: BusImuBoard::Ft6,
            imu_protocol: BusImuProtocol::Auto,
            imu_profile: BusImuProfile::Legacy,
            imu_identity: None,
            observations: observations::Observations::default(),
        })
    }

    /// Discover ID 200 under the saved profile and establish its telemetry layout/order.
    /// Qinmo's configured profile is never presented as authenticated vendor identity.
    pub fn identify_imu(&mut self) -> Result<BusImuBoard> {
        self.imu_identity = None;
        if self.imu_profile == BusImuProfile::Qinmo {
            // The first valid frame locks Qinmo's protocol for 20 seconds of idle.
            // Never probe DXL2 or assume an undocumented identity table on this path.
            self.ping(IMU_ID)?;
            ImuSample::decode_qinmo(&self.read(IMU_ID, SYNC_ADDRESS, IMU_LENGTH)?)?;
            self.imu_board = BusImuBoard::Qinmo;
            self.imu_identity = Some(Vec::new());
            return Ok(self.imu_board);
        }
        let mut feetech_failure = None;
        if self.imu_protocol != BusImuProtocol::Dxl2 {
            match self.read(IMU_ID, 0, 5) {
                Ok(raw) => {
                    let board =
                        BusImuBoard::from_identity(&raw).ok_or_else(|| unknown_imu(&raw))?;
                    self.imu_board = board;
                    self.imu_identity = Some(raw);
                    return Ok(board);
                }
                Err(problem) if self.imu_protocol == BusImuProtocol::Feetech => {
                    return Err(problem);
                }
                Err(problem) => feetech_failure = Some(problem),
            }
        }
        match self.dxl2_read(0, 3, Instant::now() + self.timeout) {
            Ok(raw) => {
                let core = self.dxl2_read(124, 12, Instant::now() + self.timeout)?;
                ImuSample::decode_dxl2(&core)?;
                self.imu_board = BusImuBoard::Dxl2;
                self.imu_identity = Some(raw);
                Ok(self.imu_board)
            }
            Err(problem) => Err(error(format!(
                "IMU ID 200 detection failed: Feetech {feetech_failure:?}; DXL2 {problem}"
            ))),
        }
    }

    pub fn set_imu_protocol(&mut self, value: &str) -> Result<()> {
        let protocol = match value {
            "auto" => BusImuProtocol::Auto,
            "feetech" => BusImuProtocol::Feetech,
            "dxl2" => BusImuProtocol::Dxl2,
            _ => return Err(error("IMU protocol must be auto, feetech or dxl2")),
        };
        if self.imu_profile == BusImuProfile::Qinmo && protocol == BusImuProtocol::Dxl2 {
            return Err(error("Qinmo profile requires imu_protocol=auto or feetech"));
        }
        self.imu_protocol = protocol;
        self.imu_identity = None;
        Ok(())
    }

    /// Set once before discovery, while the main program is stopped.
    pub fn set_imu_profile(&mut self, value: &str) -> Result<()> {
        let profile = match value {
            "legacy" => BusImuProfile::Legacy,
            "qinmo" => BusImuProfile::Qinmo,
            _ => return Err(error("IMU profile must be legacy or qinmo")),
        };
        if profile == BusImuProfile::Qinmo && self.imu_protocol == BusImuProtocol::Dxl2 {
            return Err(error("Qinmo profile requires imu_protocol=auto or feetech"));
        }
        self.imu_profile = profile;
        self.imu_identity = None;
        Ok(())
    }

    pub fn imu_profile(&self) -> BusImuProfile {
        self.imu_profile
    }

    pub fn imu_info(&self) -> Option<BusImuInfo> {
        Some(BusImuInfo {
            protocol: self.imu_board.protocol().label(),
            board: self.imu_board.label(),
            identity: self.imu_identity.clone()?,
            profile: self.imu_profile.label(),
            gyro_dps_per_lsb: self.imu_profile.gyro_dps_per_lsb(),
            identification: match self.imu_board {
                BusImuBoard::Qinmo => "configured_profile",
                BusImuBoard::Dxl2 => "compatibility_layout",
                BusImuBoard::Ft6 | BusImuBoard::ImuToDxl => "legacy_identity",
            },
        })
    }

    fn dxl2_read(&mut self, address: u16, length: u16, deadline: Instant) -> Result<Vec<u8>> {
        let started = Instant::now();
        let mut trace = SyncTrace {
            address: address as u8,
            length: length as u8,
            missing_ids: vec![IMU_ID],
            ..SyncTrace::default()
        };
        let mut decoder = crate::dxl2::Decoder::default();
        let result = (|| {
            let params = [address.to_le_bytes(), length.to_le_bytes()].concat();
            self.send(&crate::dxl2::packet(IMU_ID, READ, &params), deadline)?;
            trace.request_sent = true;
            let mut chunk = [0u8; 128];
            while Instant::now() < deadline {
                self.poll_timeout(deadline)?;
                match self.port.read(&mut chunk) {
                    Ok(count) => {
                        trace.rx_bytes += count;
                        let mut answer = None;
                        for byte in &chunk[..count] {
                            if let Some(status) = decoder.push(*byte) {
                                if status.id != IMU_ID {
                                    continue;
                                }
                                if status.error != 0 {
                                    return Err(error(format!(
                                        "DXL2 IMU200 status error 0x{:02x}",
                                        status.error
                                    )));
                                }
                                if status.data.len() != usize::from(length) {
                                    return Err(error(format!(
                                        "DXL2 IMU200 expected {length} bytes, got {}",
                                        status.data.len()
                                    )));
                                }
                                if answer.is_none() {
                                    answer = Some((status.data, status.wire_len));
                                }
                            }
                        }
                        if let Some((data, wire_len)) = answer {
                            // Inspect the entire read, including bytes after the
                            // expected frame. Unsolicited frames and even a short
                            // noise tail disqualify clean partial acquisition.
                            trace.discarded = trace.rx_bytes.saturating_sub(wire_len) as u64;
                            if decoder.bad_checksums != 0 {
                                return Err(error("DXL2 IMU200 checksum error"));
                            }
                            if Instant::now() >= deadline {
                                return Err(error(
                                    "DXL2 IMU200 response exceeded transaction deadline",
                                ));
                            }
                            return Ok(data);
                        }
                    }
                    Err(e)
                        if matches!(
                            e.kind(),
                            std::io::ErrorKind::TimedOut
                                | std::io::ErrorKind::WouldBlock
                                | std::io::ErrorKind::Interrupted
                        ) => {}
                    Err(e) => return Err(error(e.to_string())),
                }
            }
            Err(error(format!(
                "DXL2 IMU200 response timeout ({} checksum errors)",
                decoder.bad_checksums
            )))
        })();
        trace.elapsed_us = started.elapsed().as_micros() as u64;
        trace.deadline_exceeded = Instant::now() >= deadline;
        trace.bad_checksums = decoder.bad_checksums;
        trace.discarded = trace.discarded.max(decoder.discarded);
        if result.is_ok() {
            trace.missing_ids.clear();
        }
        self.last_sync = trace;
        result
    }

    pub fn imu_board(&self) -> BusImuBoard {
        self.imu_board
    }

    /// Retain replies before an initialization error drops this UART owner.
    /// This does not read registers or construct a usable control sample.
    pub fn servo_observations(&self, calibration: &Calibration) -> observations::Snapshot {
        let metadata = crate::feetech_snapshot::Metadata {
            joints: calibration.joints.iter().map(|joint| crate::feetech_snapshot::JointMetadata {
                id: joint.id,
                name: joint.name.clone(),
                calibrated: joint.calibrated,
            }).collect(),
            ..Default::default()
        };
        self.observations.snapshot(std::sync::Arc::new(metadata), Instant::now())
    }

    /// Enforced at the packet boundary, including every existing write API.
    pub fn open_read_only(path: &str) -> Result<Self> {
        let mut bus = Self::open(path)?;
        bus.read_only = true;
        bus.timeout = Duration::from_millis(40);
        Ok(bus)
    }

    fn send(&mut self, bytes: &[u8], deadline: Instant) -> Result<()> {
        let instruction = bytes
            .get(if bytes.starts_with(&crate::dxl2::HEADER) {
                7
            } else {
                4
            })
            .copied();
        if self.read_only && !matches!(instruction, Some(1 | READ | SYNC_READ)) {
            return Err(error("read-only motor bus: write instruction refused"));
        }
        self.port
            .clear(serialport::ClearBuffer::Input)
            .map_err(|e| error(e.to_string()))?;
        let mut remaining = bytes;
        while !remaining.is_empty() {
            self.poll_timeout(deadline)?;
            match self.port.write(remaining) {
                Ok(0) => return Err(error("Feetech request write returned zero")),
                Ok(count) => remaining = &remaining[count..],
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::Interrupted
                            | std::io::ErrorKind::TimedOut
                            | std::io::ErrorKind::WouldBlock
                    ) => {}
                Err(e) => return Err(error(e.to_string())),
            }
        }
        // On the HAT, direction is automatic. Start receiving a request's replies
        // immediately: Unix SerialPort::flush calls tcdrain and can delay the
        // control thread after the small request has already left the UART.
        Ok(())
    }

    fn poll_timeout(&mut self, deadline: Instant) -> Result<()> {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Err(error("Feetech transaction deadline exceeded"));
        }
        self.port
            .set_timeout(left.min(Duration::from_millis(2)))
            .map_err(|e| error(e.to_string()))
    }

    fn exchange(&mut self, id: u8, inst: u8, params: &[u8], count: usize) -> Result<Vec<u8>> {
        self.exchange_budget(id, inst, params, count, self.timeout)
    }

    fn exchange_budget(
        &mut self,
        id: u8,
        inst: u8,
        params: &[u8],
        count: usize,
        budget: Duration,
    ) -> Result<Vec<u8>> {
        if id >= BROADCAST {
            return Err(error("unicast reply required"));
        }
        let tx = packet(id, inst, params)?;
        let deadline = Instant::now() + budget;
        self.send(&tx, deadline)?;
        if inst == READ {
            self.observations.requested(&[id], Instant::now());
        }
        let mut decoder = Decoder::default();
        let mut one = [0u8; 1];
        while Instant::now() < deadline {
            self.poll_timeout(deadline)?;
            match self.port.read(&mut one) {
                Ok(0) => continue,
                Ok(_) => {
                    if let Some(status) = decoder.push(one[0]) {
                        if status.id != id {
                            continue;
                        }
                        // READ/PING echo is unambiguously an instruction, not an empty
                        // successful status. Do not interpret the echoed READ as error 2.
                        if status.error == inst && status.data == params {
                            continue;
                        }
                        if inst == READ {
                            self.observations.replied(&status, params[0], params[1], Instant::now());
                        }
                        if status.error != 0 {
                            return Err(error(format!(
                                "Feetech ID {id} status error 0x{:02x}",
                                status.error
                            )));
                        }
                        if status.data.len() != count {
                            return Err(IoError::ShortRead {
                                what: "Feetech status",
                                expected: count,
                                got: status.data.len(),
                            });
                        }
                        return Ok(status.data);
                    }
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::TimedOut
                            | std::io::ErrorKind::WouldBlock
                            | std::io::ErrorKind::Interrupted
                    ) => {}
                Err(e) => return Err(error(e.to_string())),
            }
        }
        Err(error(format!("Feetech ID {id} response timeout")))
    }

    pub fn ping(&mut self, id: u8) -> Result<()> {
        self.exchange(id, 1, &[], 0).map(|_| ())
    }
    pub fn read(&mut self, id: u8, address: u8, length: u8) -> Result<Vec<u8>> {
        if length == 0 || length > 253 || u16::from(address) + u16::from(length) > 256 {
            return Err(error("invalid Feetech register range"));
        }
        self.exchange(id, READ, &[address, length], usize::from(length))
    }

    /// Unicast write requires an acknowledgement. Never retry a motion write blindly.
    pub fn write(&mut self, id: u8, address: u8, data: &[u8]) -> Result<()> {
        if data.is_empty() || usize::from(address) + data.len() > 256 {
            return Err(error("invalid Feetech write range"));
        }
        let mut params = vec![address];
        params.extend_from_slice(data);
        self.exchange(id, WRITE, &params, 0).map(|_| ())
    }

    /// A broadcast has NO acknowledgement. Success here means sent, not confirmed.
    /// The UART preserves write order; HAT direction is automatic. A following
    /// query confirms transitions without tcdrain sleeping on every position.
    pub fn sync_write(&mut self, address: u8, rows: &[(u8, Vec<u8>)]) -> Result<()> {
        self.send(
            &sync_write_packet(address, rows)?,
            Instant::now() + Duration::from_millis(4),
        )
    }

    /// One bounded transition confirmation, never an automatic retry.
    pub fn read_control(&mut self, id: u8, address: u8, length: u8) -> Result<Vec<u8>> {
        self.exchange_budget(
            id,
            READ,
            &[address, length],
            usize::from(length),
            Duration::from_millis(4),
        )
    }

    /// Exactly one request. A bad or missing reply fails this sample.
    pub fn sync_read(
        &mut self,
        address: u8,
        length: u8,
        ids: &[u8],
    ) -> Result<BTreeMap<u8, Vec<u8>>> {
        self.sync_read_inner(address, length, ids, false)
    }

    fn sync_read_inner(
        &mut self,
        address: u8,
        length: u8,
        ids: &[u8],
        allow_partial: bool,
    ) -> Result<BTreeMap<u8, Vec<u8>>> {
        self.sync_read_window(address, length, ids, allow_partial, None, false)
    }

    fn sync_read_window(
        &mut self,
        address: u8,
        length: u8,
        ids: &[u8],
        allow_partial: bool,
        window: Option<(Instant, Instant)>,
        external_imu: bool,
    ) -> Result<BTreeMap<u8, Vec<u8>>> {
        let started = Instant::now();
        let budget = if address == SYNC_ADDRESS {
            Duration::from_millis(TELEMETRY_DEADLINE_MS)
        } else if ids.len() == NUM_JOINTS && matches!((address, length), (40, 1) | (42, 2)) {
            Duration::from_millis(CONTROL_READBACK_DEADLINE_MS)
        } else {
            Duration::from_millis(4)
        };
        let deadline = started
            + if self.read_only
                && !(self.imu_profile == BusImuProfile::Qinmo && address == SYNC_ADDRESS)
            {
                self.timeout
            } else {
                budget
            };
        let (started, deadline) = window.unwrap_or((started, deadline));
        let mut pending = ids.iter().copied().collect::<BTreeSet<_>>();
        let mut decoder = Decoder::for_sync_read(length);
        let mut rx_bytes = 0;
        let mut complete_but_late = false;
        let mut request_sent = false;
        let mut late_bytes = false;
        let result = (|| {
            self.send(&sync_read_packet(address, length, ids)?, deadline)?;
            request_sent = true;
            self.observations.requested(ids, Instant::now());
            let mut replies = BTreeMap::new();
            let mut failure = None;
            let mut bytes = [0u8; 256];
            while Instant::now() < deadline {
                if self.poll_timeout(deadline).is_err() {
                    break;
                }
                match self.port.read(&mut bytes) {
                    Ok(count) => {
                        late_bytes |= count > 0 && Instant::now() >= deadline;
                        rx_bytes += count;
                        for byte in &bytes[..count] {
                            if let Some(status) = decoder.push(*byte) {
                                if pending.contains(&status.id) {
                                    self.observations.replied(&status, address, length, Instant::now());
                                }
                                if let Err(error) =
                                    accept_sync_status(&mut pending, &mut replies, status, length)
                                {
                                    failure.get_or_insert(error);
                                }
                            }
                        }
                        if pending.is_empty() {
                            if let Some(error) = failure {
                                return Err(error);
                            }
                            if decoder.bad_checksums > 0 {
                                return Err(error("Feetech Sync Read checksum error"));
                            }
                            let completed = Instant::now();
                            if completed >= deadline {
                                complete_but_late = true;
                                break;
                            }
                            return Ok(replies);
                        }
                    }
                    Err(e)
                        if matches!(
                            e.kind(),
                            std::io::ErrorKind::TimedOut
                                | std::io::ErrorKind::WouldBlock
                                | std::io::ErrorKind::Interrupted
                        ) => {}
                    Err(e) => return Err(error(e.to_string())),
                }
            }
            if let Some(error) = failure {
                return Err(error);
            }
            if complete_but_late {
                return Err(error(
                    "Feetech Sync Read complete but transaction deadline exceeded",
                ));
            }
            // Partial acceptance is telemetry-only. Never soften register/gain/torque
            // confirmation, IMU loss, protocol errors, or a rejected late frame.
            if allow_partial
                && address == SYNC_ADDRESS
                && length == SYNC_LENGTH
                && (external_imu || replies.contains_key(&IMU_ID))
                && !pending.is_empty()
                && !pending.contains(&IMU_ID)
                && decoder.bad_checksums == 0
                && decoder.discarded == 0
                && !late_bytes
            {
                return Ok(replies);
            }
            Err(error(format!(
                "Feetech Sync Read {}; missing IDs {:?}",
                if decoder.bad_checksums > 0 {
                    "checksum error"
                } else {
                    "timeout"
                },
                pending.iter().copied().collect::<Vec<_>>()
            )))
        })();
        let partial_received = result.is_ok() && !pending.is_empty();
        self.last_sync = SyncTrace {
            address,
            length,
            missing_ids: pending.into_iter().collect(),
            bad_checksums: decoder.bad_checksums,
            discarded: decoder.discarded,
            elapsed_us: started.elapsed().as_micros() as u64,
            rx_bytes,
            deadline_exceeded: Instant::now() >= deadline,
            complete_but_late,
            request_sent,
            partial_received,
        };
        result
    }

    /// FT6 and all fifteen joints share the fixed 56/15 block.
    pub fn telemetry(&mut self, servo_ids: &[u8]) -> Result<BTreeMap<u8, Vec<u8>>> {
        if servo_ids.len() != NUM_JOINTS || servo_ids.contains(&IMU_ID) {
            return Err(error("telemetry requires all 15 joints plus IMU 200"));
        }
        if self.imu_board == BusImuBoard::Dxl2 {
            return self.dxl2_telemetry(servo_ids, false);
        }
        let ids = telemetry_ids(self.imu_board, servo_ids);
        self.sync_read(SYNC_ADDRESS, SYNC_LENGTH, &ids)
    }

    /// The fifteen joints alone, for a robot whose trunk IMU is not on the bus: ID 200 is
    /// neither asked for nor expected. Strict — every servo answers or the sample fails. The
    /// board's reply is the condition `telemetry_partial` accepts a short frame on, so without
    /// it there is no short frame to accept; and on the bus a missing servo already costs the
    /// whole 20 ms budget, which fails the board's IMU age anyway.
    pub fn joint_telemetry(&mut self, servo_ids: &[u8]) -> Result<BTreeMap<u8, Vec<u8>>> {
        if servo_ids.len() != NUM_JOINTS || servo_ids.contains(&IMU_ID) {
            return Err(error(
                "joint telemetry requires all 15 joints and no IMU 200",
            ));
        }
        self.sync_read(SYNC_ADDRESS, SYNC_LENGTH, servo_ids)
    }

    fn telemetry_partial(&mut self, servo_ids: &[u8]) -> Result<BTreeMap<u8, Vec<u8>>> {
        if servo_ids.len() != NUM_JOINTS || servo_ids.contains(&IMU_ID) {
            return Err(error("telemetry requires all 15 joints plus IMU 200"));
        }
        if self.imu_board == BusImuBoard::Dxl2 {
            return self.dxl2_telemetry(servo_ids, true);
        }
        let ids = telemetry_ids(self.imu_board, servo_ids);
        self.sync_read_inner(SYNC_ADDRESS, SYNC_LENGTH, &ids, true)
    }

    fn dxl2_telemetry(
        &mut self,
        servo_ids: &[u8],
        allow_partial: bool,
    ) -> Result<BTreeMap<u8, Vec<u8>>> {
        let started = Instant::now();
        let deadline = started + Duration::from_millis(TELEMETRY_DEADLINE_MS);
        let raw = match self.dxl2_read(124, 12, deadline) {
            Ok(raw) => raw,
            Err(problem) => {
                // Totals describe one telemetry acquisition, including an
                // IMU failure before any servo request was attempted.
                self.last_sync.address = SYNC_ADDRESS;
                self.last_sync.length = SYNC_LENGTH;
                self.last_sync.elapsed_us = started.elapsed().as_micros() as u64;
                return Err(problem);
            }
        };
        let imu_trace = self.last_sync.clone();
        let clean_imu = imu_trace.discarded == 0 && imu_trace.bad_checksums == 0;
        let result = self.sync_read_window(
            SYNC_ADDRESS,
            SYNC_LENGTH,
            servo_ids,
            allow_partial,
            Some((started, deadline)),
            clean_imu,
        );
        self.last_sync.rx_bytes += imu_trace.rx_bytes;
        self.last_sync.bad_checksums += imu_trace.bad_checksums;
        self.last_sync.discarded += imu_trace.discarded;
        let mut blocks = result?;
        blocks.insert(IMU_ID, raw);
        Ok(blocks)
    }

    pub fn decode_imu(&self, raw: &[u8]) -> Result<ImuSample> {
        if self.imu_profile == BusImuProfile::Qinmo {
            ImuSample::decode_qinmo(raw)
        } else if self.imu_board == BusImuBoard::Dxl2 {
            ImuSample::decode_dxl2(raw)
        } else {
            ImuSample::decode(raw)
        }
    }

    pub fn feedback(&mut self, id: u8) -> Result<Feedback> {
        Feedback::decode(&self.read(id, 56, 11)?)
    }

    pub fn imu(&mut self) -> Result<ImuSample> {
        if self.imu_profile == BusImuProfile::Qinmo {
            return ImuSample::decode_qinmo(&self.read(IMU_ID, SYNC_ADDRESS, IMU_LENGTH)?);
        }
        if self.imu_board == BusImuBoard::Dxl2 {
            let raw = self.dxl2_read(124, 12, Instant::now() + self.timeout)?;
            return ImuSample::decode_dxl2(&raw);
        }
        ImuSample::decode_diagnostic(&self.read(
            IMU_ID,
            IMU_DIAGNOSTIC_ADDRESS,
            IMU_DIAGNOSTIC_LENGTH,
        )?)
    }

    pub fn positions(&mut self, rows: &[(u8, u16)]) -> Result<()> {
        let rows = rows
            .iter()
            .map(|(id, p)| Ok((*id, position_command(*p)?.to_vec())))
            .collect::<Result<Vec<_>>>()?;
        self.sync_write(42, &rows)
    }

    /// Motion goals use the servo's signed native coordinate, including its
    /// current turn. Only calibrated joint observations are modulo one turn.
    pub fn signed_positions(&mut self, rows: &[(u8, i32)]) -> Result<()> {
        let rows = rows
            .iter()
            .map(|(id, p)| Ok((*id, signed_position_command(*p)?.to_vec())))
            .collect::<Result<Vec<_>>>()?;
        self.sync_write(42, &rows)
    }

    /// Targeted broadcast, so ID 200 is never included. Read reg 40 afterwards
    /// if the caller needs confirmation (in particular for torque-off).
    pub fn torque(&mut self, ids: &[u8], on: bool) -> Result<()> {
        let rows = ids
            .iter()
            .map(|id| (*id, vec![u8::from(on)]))
            .collect::<Vec<_>>();
        self.sync_write(40, &rows)
    }

    /// One request verifies every requested switch; missing/nonzero replies fail.
    pub fn verify_torque(&mut self, ids: &[u8], on: bool) -> Result<()> {
        let states = self.sync_read(40, 1, ids).map_err(|problem| {
            error(format!("torque readback at register 40: {problem}; request_sent={}, rx_bytes={}, elapsed_us={}, bad_checksums={}",
                self.last_sync.request_sent, self.last_sync.rx_bytes,
                self.last_sync.elapsed_us, self.last_sync.bad_checksums))
        })?;
        for id in ids {
            if states.get(id).map(Vec::as_slice) != Some(&[u8::from(on)][..]) {
                return Err(error(format!("ID {id} torque state {on} not confirmed")));
            }
        }
        Ok(())
    }
}

pub fn sync_read_packet(address: u8, length: u8, ids: &[u8]) -> Result<Vec<u8>> {
    let mut unique = BTreeSet::new();
    if length == 0
        || length > 253
        || u16::from(address) + u16::from(length) > 256
        || ids.is_empty()
        || ids.len() > 251
        || ids.iter().any(|id| *id >= BROADCAST || !unique.insert(*id))
        // The IMU answers first (FT6) or last (imu_to_dxl), never between servos.
        || (ids.contains(&IMU_ID) && ids[0] != IMU_ID && ids[ids.len() - 1] != IMU_ID)
    {
        return Err(error("invalid Feetech Sync Read address/length/ID order"));
    }
    let mut params = Vec::with_capacity(ids.len() + 2);
    params.extend_from_slice(&[address, length]);
    params.extend_from_slice(ids);
    packet(BROADCAST, SYNC_READ, &params)
}

pub fn sync_write_packet(address: u8, rows: &[(u8, Vec<u8>)]) -> Result<Vec<u8>> {
    let Some((_, first)) = rows.first() else {
        return Err(error("empty sync write"));
    };
    let width = first.len();
    let mut ids = BTreeSet::new();
    if width == 0 || usize::from(address) + width > 256 {
        return Err(error("invalid sync write width"));
    }
    let mut params = vec![address, width as u8];
    for (id, data) in rows {
        if *id >= BROADCAST || *id == IMU_ID || !ids.insert(*id) || data.len() != width {
            return Err(error(
                "invalid/duplicate sync write ID or mismatched row length",
            ));
        }
        params.push(*id);
        params.extend_from_slice(data);
    }
    packet(BROADCAST, SYNC_WRITE, &params)
}

/// HD1910 native target: addresses 42..47 only. Register 41 is never written.
pub fn position_command(position: u16) -> Result<[u8; 6]> {
    if position > 4095 {
        return Err(error("HD1910 position outside single-turn range"));
    }
    signed_position_command(i32::from(position))
}

/// HD1910 uses signed magnitude at bit 15 for the position register.
pub fn signed_position_word(position: i32) -> Result<u16> {
    if !(-32767..=32767).contains(&position) {
        return Err(error("HD1910 native position outside signed 15-bit range"));
    }
    Ok(position.unsigned_abs() as u16 | if position < 0 { 0x8000 } else { 0 })
}

fn signed_position_command(position: i32) -> Result<[u8; 6]> {
    let p = signed_position_word(position)?.to_le_bytes();
    Ok([p[0], p[1], 0, 0, 0, 0])
}

fn signed_magnitude(raw: u16, sign_bit: u8) -> i32 {
    let sign = 1u16 << sign_bit;
    let magnitude = i32::from(raw & (sign - 1));
    if raw & sign != 0 {
        -magnitude
    } else {
        magnitude
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Feedback {
    pub position_raw: i32,
    pub position_wire_raw: u16,
    pub position_signed_raw: i32,
    pub speed_raw: i32,
    pub load_raw: i32,
    pub volts: f64,
    pub temperature_c: u8,
    pub register_64_raw: u8,
    pub register_65_raw: u8,
    pub moving: bool,
}
impl Feedback {
    pub fn decode(raw: &[u8]) -> Result<Self> {
        if raw.len() != 11 {
            return Err(IoError::ShortRead {
                what: "HD1910 feedback",
                expected: 11,
                got: raw.len(),
            });
        }
        Ok(Self {
            position_raw: signed_magnitude(u16::from_le_bytes([raw[0], raw[1]]), 15),
            position_wire_raw: u16::from_le_bytes([raw[0], raw[1]]),
            position_signed_raw: signed_magnitude(u16::from_le_bytes([raw[0], raw[1]]), 15),
            speed_raw: signed_magnitude(u16::from_le_bytes([raw[2], raw[3]]), 15),
            load_raw: signed_magnitude(u16::from_le_bytes([raw[4], raw[5]]), 10),
            volts: f64::from(raw[6]) * 0.1,
            temperature_c: raw[7],
            register_64_raw: raw[8],
            register_65_raw: raw[9],
            moving: raw[10] != 0,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ServoSample {
    pub feedback: Feedback,
    pub current_raw: i32,
}
impl ServoSample {
    pub fn decode(raw: &[u8]) -> Result<Self> {
        if raw.len() != usize::from(SERVO_READ_LENGTH) {
            return Err(IoError::ShortRead {
                what: "HD1910 Sync Read block",
                expected: usize::from(SERVO_READ_LENGTH),
                got: raw.len(),
            });
        }
        Ok(Self {
            feedback: Feedback::decode(&raw[..11])?,
            // Address 69..70 inside the address-56 block.
            current_raw: signed_magnitude(u16::from_le_bytes([raw[13], raw[14]]), 15),
        })
    }
}

/// FT6 fast block: 12 core bytes, u8 sample counter, status, reserved zero.
/// The 20-byte schema at 124 is diagnostic only. Do not fabricate an exact age
/// for the fast block: FT6 status provides a conservative age bound instead.
#[derive(Debug, Clone, PartialEq)]
pub struct ImuSample {
    reading: SensorReading,
    pub block: [u8; 12],
    pub sequence: Option<u32>,
    pub age_ms: Option<u16>,
    pub age_upper_bound_ms: Option<u16>,
    pub status: Option<u8>,
    pub ready: bool,
}
impl ImuSample {
    pub fn reading(&self) -> SensorReading {
        self.reading
    }

    pub fn decode_qinmo(raw: &[u8]) -> Result<Self> {
        let block: [u8; 12] = raw
            .get(..12)
            .and_then(|v| v.try_into().ok())
            .filter(|_| raw.len() == 15)
            .ok_or_else(|| error("Qinmo IMU requires 56/15"))?;
        if raw[13] & !1 != 0 || raw[14] != 0 {
            return Err(error("unsupported Qinmo IMU status/reserved byte"));
        }
        let ready = raw[13] & 1 == 0;
        let reading = crate::imu::qinmo_reading(&block, ready)
            .ok_or_else(|| error("Qinmo IMU has an invalid quaternion"))?;
        Ok(Self {
            reading,
            block,
            sequence: Some(u32::from(raw[12])),
            age_ms: None,
            age_upper_bound_ms: None,
            status: Some(raw[13]),
            ready,
        })
    }
    pub fn decode(raw: &[u8]) -> Result<Self> {
        if raw.len() != usize::from(IMU_LENGTH) {
            return Err(IoError::ShortRead {
                what: "Feetech FT6 IMU 56/15",
                expected: usize::from(IMU_LENGTH),
                got: raw.len(),
            });
        }
        if raw[14] != 0 || raw[13] & 0xf0 != 0 {
            return Err(error("unsupported FT6 IMU status/reserved byte"));
        }
        let ready = raw[13] & 0x07 == 0;
        Ok(Self {
            reading: crate::imu::parse_block(raw[..12].try_into().unwrap()),
            block: raw[..12].try_into().unwrap(),
            sequence: Some(u32::from(raw[12])),
            age_ms: None,
            age_upper_bound_ms: ready.then_some(IMU_FRESH_BOUND_MS),
            status: Some(raw[13]),
            ready,
        })
    }

    pub fn decode_diagnostic(raw: &[u8]) -> Result<Self> {
        if raw.len() != usize::from(IMU_DIAGNOSTIC_LENGTH) {
            return Err(error("Feetech IMU diagnostic requires 124/20"));
        }
        if raw[19] != 1 || raw[18] & !1 != 0 {
            return Err(error("unsupported Feetech IMU schema/flags"));
        }
        Ok(Self {
            reading: crate::imu::parse_block(raw[..12].try_into().unwrap()),
            block: raw[..12].try_into().unwrap(),
            sequence: Some(u32::from_le_bytes(raw[12..16].try_into().unwrap())),
            age_ms: Some(u16::from_le_bytes([raw[16], raw[17]])),
            age_upper_bound_ms: Some(u16::from_le_bytes([raw[16], raw[17]])),
            status: Some(u8::from(raw[18] & 1 == 0)),
            ready: raw[18] & 1 != 0,
        })
    }

    pub fn is_fresh(&self, transaction_us: u64) -> bool {
        self.ready
            && if self.age_upper_bound_ms.is_none() {
                transaction_us <= TELEMETRY_DEADLINE_MS * 1000
            } else {
                self.age_upper_bound_ms.is_some_and(|age| {
                    transaction_us.saturating_add(u64::from(age) * 1000) <= 50_000
                })
            }
    }

    pub fn decode_dxl2(raw: &[u8]) -> Result<Self> {
        let block: [u8; 12] = raw
            .try_into()
            .map_err(|_| error("DXL2 SFLP IMU requires 124/12"))?;
        let ready = crate::imu::parse_block(&block).quat.is_some();
        if !ready && block[6..12].iter().any(|b| *b != 0) {
            return Err(error("DXL2 IMU200 has an invalid SFLP quaternion"));
        }
        Ok(Self {
            reading: crate::imu::parse_block(&block),
            block,
            sequence: None,
            age_ms: None,
            age_upper_bound_ms: None,
            status: None,
            ready,
        })
    }
}

/// The same freshness evidence gates commissioning and motion. DXL2 has no
/// counter: only actual repeated bytes can establish a frozen stream.
#[derive(Default)]
pub struct BusImuFreshness {
    last_sequence: Option<u32>,
    last_block: Option<[u8; 12]>,
    pub stale: ImuStale,
}
impl BusImuFreshness {
    pub fn check(&mut self, sample: &ImuSample, transaction_us: u64) -> Result<()> {
        let duplicate = match sample.sequence {
            Some(seq) => self.last_sequence == Some(seq),
            None => self.last_block == Some(sample.block),
        };
        self.stale.run = if duplicate {
            self.stale.run.saturating_add(1)
        } else {
            0
        };
        self.stale.total = self.stale.total.saturating_add(u64::from(duplicate));
        self.last_sequence = sample.sequence;
        self.last_block = Some(sample.block);
        let limit = if sample.sequence.is_some() { 3 } else { 24 };
        if !sample.is_fresh(transaction_us) || self.stale.run >= limit {
            return Err(error(
                "IMU not ready or stale (invalid reading / Feetech counter / DXL2 repeated core bytes)",
            ));
        }
        Ok(())
    }
}

/// Installation-specific joint mapping. An unconfirmed mapping is useful for
/// commissioning, but cannot enable motors. No XL330 unit or gain is reused.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Calibration {
    #[serde(default)]
    pub motion_enabled: bool,
    #[serde(default)]
    pub imu_mount_verified: bool,
    /// The wire format under which this bus mounting was verified. Legacy
    /// files describe Feetech; they cannot silently authorize a DXL2 mount.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imu_bus_protocol: Option<BusImuProtocol>,
    /// Missing in older files means the legacy units and quaternion readiness semantics.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imu_bus_profile: Option<BusImuProfile>,
    /// Sensor-to-trunk rotation [w, x, y, z], not a heading reset.
    #[serde(default = "default_imu_mount")]
    pub imu_mount_quat: [f64; 4],
    pub joints: Vec<JointCalibration>,
    /// Present velocity always uses the pinned HD1910 HD1910 native scale.
    /// This optional calibration is for measured current only.
    #[serde(default)]
    pub current_ma_per_count: f64,
    #[serde(default)]
    pub units_verified: bool,
    /// Mounts of trunk IMUs other than the bus board, by source. The bus board's stays in
    /// `imu_mount_quat`/`imu_mount_verified`, where every existing file already has it, so
    /// such a file reads and writes exactly as before. See [`Calibration::imu_mount`].
    #[serde(default, skip_serializing_if = "ImuMounts::is_empty")]
    pub imu_mounts: ImuMounts,
}
fn default_imu_mount() -> [f64; 4] {
    SflpDecoder::DEFAULT_MOUNT
}

/// One trunk IMU's commissioned mount.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ImuMount {
    /// Sensor-to-trunk rotation [w, x, y, z], not a heading reset.
    pub quat: [f64; 4],
    #[serde(default)]
    pub verified: bool,
}

/// `[imu_mounts.<source>]`: one entry per source that is not the bus board.
///
/// Per source because the two sensors sit in different places, so a mount verified for one
/// says nothing about the other — and a robot switched back to the board must find the
/// board's mount where it left it.
#[derive(Debug, Clone, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ImuMounts {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bno08x: Option<ImuMount>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lsm6dsv16x: Option<ImuMount>,
}
impl ImuMounts {
    fn is_empty(&self) -> bool {
        self.bno08x.is_none() && self.lsm6dsv16x.is_none()
    }
    fn slot(&mut self, kind: TrunkImuKind) -> Option<&mut Option<ImuMount>> {
        match kind {
            TrunkImuKind::Bus => None,
            TrunkImuKind::Bno08x => Some(&mut self.bno08x),
            TrunkImuKind::Lsm6dsv16x => Some(&mut self.lsm6dsv16x),
        }
    }
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct JointCalibration {
    pub name: String,
    pub id: u8,
    pub zero_raw: u16,
    pub direction: i8,
    #[serde(default)]
    pub calibrated: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_rad: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_rad: Option<f64>,
}
impl JointCalibration {
    pub fn position_rad(&self, raw: i32) -> f64 {
        (f64::from(raw) - f64::from(self.zero_raw)) * TAU / 4096.0 * f64::from(self.direction)
    }
    fn raw_position(&self, radians: f64) -> f64 {
        f64::from(self.zero_raw) + radians * f64::from(self.direction) * 4096.0 / TAU
    }
    /// All rows are converted before any packet is sent. Clamp to measured
    /// mechanical limits, never wrap a single-turn joint across its hard stop.
    pub fn target_raw(&self, radians: f64) -> Result<(u16, bool)> {
        if !self.calibrated || !radians.is_finite() {
            return Err(error(format!(
                "{}: target requires a calibrated joint and finite angle",
                self.name
            )));
        }
        let (Some(min), Some(max)) = (self.min_rad, self.max_rad) else {
            return Err(error(format!("{}: joint limits missing", self.name)));
        };
        if !min.is_finite() || !max.is_finite() || min >= max {
            return Err(error(format!("{}: invalid joint limits", self.name)));
        }
        let bounded = radians.clamp(min, max);
        let raw = self.raw_position(bounded).round();
        if !raw.is_finite() || !(0.0..=4095.0).contains(&raw) {
            return Err(error(format!(
                "{}: target outside single-turn encoder range",
                self.name
            )));
        }
        Ok((raw as u16, bounded != radians))
    }
}
impl Calibration {
    pub fn validate_bus_mount_for(
        &self,
        protocol: BusImuProtocol,
        profile: BusImuProfile,
    ) -> Result<()> {
        self.validate_bus_mount(protocol)?;
        let bound = self.imu_bus_profile.unwrap_or_default();
        if bound != profile
            || (profile == BusImuProfile::Qinmo && protocol != BusImuProtocol::Feetech)
        {
            return Err(error(format!(
                "IMU mounting was verified for profile {}, configured {}; redo IMU mounting calibration before motion",
                bound.label(),
                profile.label()
            )));
        }
        Ok(())
    }
    pub fn validate_bus_mount(&self, protocol: BusImuProtocol) -> Result<()> {
        let bound = self.imu_bus_protocol.unwrap_or(BusImuProtocol::Feetech);
        if bound == BusImuProtocol::Auto || protocol != bound {
            return Err(error(format!(
                "IMU mounting was verified for {}, detected {}; redo IMU mounting calibration before motion",
                bound.label(),
                protocol.label()
            )));
        }
        Ok(())
    }

    fn feedback_speed_scale(&self) -> Option<f64> {
        Some(crate::hd1910::SPEED_RAD_S_PER_COUNT)
    }

    pub fn commissioning(ids: [u8; NUM_JOINTS]) -> Self {
        Self {
            motion_enabled: false,
            imu_mount_verified: false,
            imu_bus_protocol: None,
            imu_bus_profile: None,
            imu_mount_quat: default_imu_mount(),
            joints: JOINT_NAMES
                .iter()
                .zip(ids)
                .map(|(name, id)| JointCalibration {
                    name: (*name).to_owned(),
                    id,
                    zero_raw: 2048,
                    direction: 1,
                    calibrated: false,
                    min_rad: None,
                    max_rad: None,
                })
                .collect(),
            current_ma_per_count: 0.0,
            units_verified: false,
            imu_mounts: ImuMounts::default(),
        }
    }
    /// The mount commissioned for `kind`; an unverified default for a source never
    /// commissioned.
    pub fn imu_mount(&self, kind: TrunkImuKind) -> ImuMount {
        let commissioned = match kind {
            TrunkImuKind::Bus => Some(ImuMount {
                quat: self.imu_mount_quat,
                verified: self.imu_mount_verified,
            }),
            TrunkImuKind::Bno08x => self.imu_mounts.bno08x,
            TrunkImuKind::Lsm6dsv16x => self.imu_mounts.lsm6dsv16x,
        };
        commissioned.unwrap_or(ImuMount {
            quat: kind.default_mount(),
            verified: false,
        })
    }
    /// Record `kind`'s mount. The bus board's goes where it always has.
    pub fn set_imu_mount(&mut self, kind: TrunkImuKind, mount: ImuMount) {
        match self.imu_mounts.slot(kind) {
            Some(slot) => *slot = Some(mount),
            None => {
                self.imu_mount_quat = mount.quat;
                self.imu_mount_verified = mount.verified;
            }
        }
    }
    pub fn validate(&self) -> Result<()> {
        if self.imu_bus_protocol == Some(BusImuProtocol::Auto) {
            return Err(error(
                "a verified IMU binding must name feetech or dxl2, not auto",
            ));
        }
        for kind in TrunkImuKind::ALL {
            let mount_norm = self.imu_mount(kind).quat.iter().map(|v| v * v).sum::<f64>();
            if !mount_norm.is_finite() || (mount_norm - 1.0).abs() > 1e-6 {
                return Err(error(match kind {
                    TrunkImuKind::Bus => {
                        "IMU mounting quaternion must be finite unit length [w,x,y,z]".to_owned()
                    }
                    other => format!(
                        "{} IMU mounting quaternion must be finite unit length [w,x,y,z]",
                        other.label()
                    ),
                }));
            }
        }
        if self.units_verified
            && (!self.current_ma_per_count.is_finite() || self.current_ma_per_count <= 0.0)
        {
            return Err(error(
                "verified HD1910 current scale must be positive and finite",
            ));
        }
        if self.joints.len() != NUM_JOINTS {
            return Err(error(
                "Feetech configuration requires all 15 joints in official policy order",
            ));
        }
        let mut ids = BTreeSet::new();
        for (i, joint) in self.joints.iter().enumerate() {
            if joint.name != JOINT_NAMES[i]
                || joint.id >= BROADCAST
                || joint.id == IMU_ID
                || !ids.insert(joint.id)
                || joint.zero_raw > 4095
                || ![-1, 1].contains(&joint.direction)
            {
                return Err(error(format!(
                    "invalid Feetech joint mapping at index {i} ({})",
                    JOINT_NAMES[i]
                )));
            }
            match (joint.min_rad, joint.max_rad) {
                (None, None) if !joint.calibrated => {}
                (Some(min), Some(max)) if min.is_finite() && max.is_finite() && min < max => {
                    if [min, max]
                        .iter()
                        .any(|q| !(0.0..=4095.0).contains(&joint.raw_position(*q).round()))
                    {
                        return Err(error(format!(
                            "{}: calibrated limits cross the single-turn encoder boundary",
                            joint.name
                        )));
                    }
                    if !(min..=max).contains(&crate::deployment::home()[i]) {
                        return Err(error(format!(
                            "{}: limits exclude official home angle {}",
                            joint.name,
                            crate::deployment::home()[i]
                        )));
                    }
                }
                _ => {
                    return Err(error(format!(
                        "{}: complete finite min_rad/max_rad required",
                        joint.name
                    )));
                }
            }
        }
        if self.motion_enabled {
            self.validate_motion_fields(None)?;
        }
        Ok(())
    }
    /// `source`: the trunk IMU motion will run on. `None` when the file is judged on its own,
    /// without knowing which is wired — then a verified mount for any source will do, which for
    /// a file that has only ever known the bus board is exactly the old rule.
    fn validate_motion_fields(&self, source: Option<TrunkImuKind>) -> Result<()> {
        let mount_verified = match source {
            Some(kind) => self.imu_mount(kind).verified,
            None => TrunkImuKind::ALL
                .into_iter()
                .any(|kind| self.imu_mount(kind).verified),
        };
        if !mount_verified
            || self.joints.iter().any(|joint| {
                !joint.calibrated || joint.min_rad.is_none() || joint.max_rad.is_none()
            })
        {
            return Err(error(match source {
                Some(kind) if kind != TrunkImuKind::Bus => format!(
                    "motion requires all 15 calibrated joints, measured limits and verified IMU \
                     mounting for the {} IMU",
                    kind.label()
                ),
                _ => "motion requires all 15 calibrated joints, measured limits and verified IMU mounting"
                    .to_owned(),
            }));
        }
        Ok(())
    }
    pub fn validate_motion(&self) -> Result<()> {
        self.validate()?;
        if !self.motion_enabled {
            return Err(error(
                "Feetech motion_enabled is false; complete commissioning first",
            ));
        }
        self.validate_motion_fields(None)
    }
    /// [`Self::validate_motion`] for the trunk IMU actually wired: that source's own mount must
    /// be verified, whatever the others say.
    pub fn validate_motion_for(&self, source: TrunkImuKind) -> Result<()> {
        self.validate_motion()?;
        self.validate_motion_fields(Some(source))
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct FeetechJointDiagnostic {
    pub name: String,
    pub id: u8,
    pub calibrated: bool,
    pub position_raw: i32,
    pub position_wire_raw: u16,
    pub position_signed_raw: i32,
    pub speed_raw: i32,
    pub load_raw: i32,
    pub current_raw: i32,
    pub volts: f64,
    pub temperature_c: u8,
    pub position_rad: Option<f64>,
    pub velocity_rad_s: Option<f64>,
    pub current_ma: Option<f64>,
    pub feedback_fresh: bool,
    pub sample_age_ms: u64,
    pub missing_total: u64,
    pub consecutive_missing: u32,
    pub read_recovery: &'static str,
}
/// Cumulative telemetry evidence, retained independently of browser polling and
/// torque/goal confirmation reads. A missing reply means no valid status frame.
#[derive(Debug, Clone, Default, Serialize)]
pub struct TelemetryTotals {
    pub session: String,
    pub total_reads: u64,
    pub failed_reads: u64,
    pub complete_but_late: u64,
    pub accepted_complete_but_late: u64,
    pub last_complete_but_late: Option<TelemetryLateRead>,
    pub missing_by_id: BTreeMap<u8, u64>,
    pub last_missing_read_by_id: BTreeMap<u8, u64>,
    pub last_error: Option<String>,
    pub last_failure: Option<TelemetryFailure>,
}

#[derive(Debug, Clone, Serialize)]
pub struct TelemetryFailure {
    pub read: u64,
    pub error: String,
    pub missing_ids: Vec<u8>,
    pub elapsed_us: u64,
}

#[derive(Debug, Clone, Serialize)]
pub struct TelemetryLateRead {
    pub read: u64,
    pub elapsed_us: u64,
    /// Accepted by the transport; the normal IMU freshness checks still follow.
    pub accepted: bool,
}

impl TelemetryTotals {
    fn new() -> Self {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        Self {
            session: format!("{}-{stamp}", std::process::id()),
            ..Self::default()
        }
    }

    fn observe(&mut self, trace: &SyncTrace, failure: Option<String>) {
        // This is called once at the acquisition site, never by a status query.
        if trace.address != SYNC_ADDRESS || trace.length != SYNC_LENGTH {
            return;
        }
        // An accepted partial sample is still an incomplete bus acquisition.
        // Count its missing replies without turning it into a control-loop error.
        let failure = failure.or_else(|| {
            (trace.request_sent && trace.partial_received && !trace.missing_ids.is_empty()).then(
                || {
                    format!(
                        "Feetech telemetry partial; missing IDs {:?}; per-joint recovery active",
                        trace.missing_ids
                    )
                },
            )
        });
        self.total_reads = self.total_reads.saturating_add(1);
        self.last_error = failure.clone();
        if trace.complete_but_late {
            self.complete_but_late = self.complete_but_late.saturating_add(1);
            let accepted = failure.is_none();
            if accepted {
                self.accepted_complete_but_late = self.accepted_complete_but_late.saturating_add(1);
            }
            self.last_complete_but_late = Some(TelemetryLateRead {
                read: self.total_reads,
                elapsed_us: trace.elapsed_us,
                accepted,
            });
        }
        if let Some(error) = failure {
            self.failed_reads = self.failed_reads.saturating_add(1);
            let missing_ids = if trace.request_sent {
                trace
                    .missing_ids
                    .iter()
                    .copied()
                    .collect::<BTreeSet<_>>()
                    .into_iter()
                    .collect::<Vec<_>>()
            } else {
                Vec::new()
            };
            for id in &missing_ids {
                let count = self.missing_by_id.entry(*id).or_default();
                *count = count.saturating_add(1);
                self.last_missing_read_by_id.insert(*id, self.total_reads);
            }
            self.last_failure = Some(TelemetryFailure {
                read: self.total_reads,
                error,
                missing_ids,
                elapsed_us: trace.elapsed_us,
            });
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct FeetechDiagnostics {
    pub servo_observations: observations::Snapshot,
    pub configured_imu_profile: &'static str,
    pub backend: &'static str,
    pub read_timing_revision: &'static str,
    pub motion_enabled: bool,
    pub calibration_complete: bool,
    pub imu_mount_verified: bool,
    /// True only after this process explicitly enabled every configured motor.
    pub torque_enabled: bool,
    /// None at startup or after an unconfirmed transaction; not a claim of limp.
    pub torque_state_confirmed: Option<bool>,
    pub gain_mode: &'static str,
    pub velocity_source: &'static str,
    /// Unfiltered native speed scale, before the installed joint direction.
    /// None for the generic position-derivative fallback.
    pub velocity_rad_s_per_count: Option<f64>,
    pub current_available: bool,
    pub sample_age_ms: Option<u64>,
    pub imu_ready: bool,
    pub imu_sequence: Option<u32>,
    pub imu_age_ms: Option<u16>,
    pub imu_age_upper_bound_ms: Option<u16>,
    pub imu_status: Option<u8>,
    /// Which trunk IMU this is: a [`TrunkImuKind`] label. `imu_mount_verified`, `imu_ready`
    /// and the `imu_*` figures above are that source's.
    pub imu_source: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub imu_bus: Option<BusImuInfo>,
    /// A host-read IMU's reader, in its own words. Absent for the bus board.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub imu_reader: Option<HostImuHealth>,
    pub command_clamps_total: u64,
    pub partial_samples_total: u64,
    pub all_joints_fresh: bool,
    pub stale_joint_ids: Vec<u8>,
    pub held_joint_ids: Vec<u8>,
    pub joints: Vec<FeetechJointDiagnostic>,
    pub sync: SyncTrace,
    pub telemetry: TelemetryTotals,
    pub hd1910: crate::hd1910::Diagnostics,
}

/// The HOME start guard needs these flags before targets are applied, while the
/// published diagnostics must include the writes later in the same tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FeetechControlStatus {
    pub all_joints_fresh: bool,
    pub imu_mount_verified: bool,
    pub imu_ready: bool,
    pub torque_state_confirmed: Option<bool>,
    pub joints_held: bool,
}

impl From<&FeetechDiagnostics> for FeetechControlStatus {
    fn from(diagnostics: &FeetechDiagnostics) -> Self {
        Self {
            all_joints_fresh: diagnostics.all_joints_fresh,
            imu_mount_verified: diagnostics.imu_mount_verified,
            imu_ready: diagnostics.imu_ready,
            torque_state_confirmed: diagnostics.torque_state_confirmed,
            joints_held: !diagnostics.held_joint_ids.is_empty(),
        }
    }
}

// Same nominal allowance as robotd's official Coast: three missed ticks.
// A wall-clock cap also bounds a delayed loop (20 ms * 3 + one read budget).
const JOINT_COAST_TICKS: u32 = 3;
const JOINT_COAST_MAX_AGE: Duration = Duration::from_millis(80);

#[derive(Clone)]
struct JointFeedback {
    at: Instant,
    sample: ServoSample,
    position: f64,
    // Last published (possibly filtered) velocity, retained during the coast.
    velocity: f64,
}

/// Where a [`FeetechIo`]'s trunk IMU comes from, and what it keeps to judge it.
enum TrunkImu {
    /// The FT6 board, ID 200, answering in the servos' own Sync Read. Exactly the state
    /// `FeetechIo` kept before there was any other source.
    Bus {
        decoder: TrunkFrame,
        freshness: BusImuFreshness,
        latest_age_ms: Option<u16>,
        latest_age_upper_bound_ms: Option<u16>,
        latest_status: Option<u8>,
    },
    /// A module the host reads on a thread of its own; this side only takes its newest sample.
    Host {
        reader: Box<dyn HostImu>,
        gate: HostImuGate,
        frame: TrunkFrame,
        /// Age of the last sample a tick accepted, as the bus board's age is.
        latest_age_ms: Option<u16>,
        /// Epoch of the last sample a tick accepted: a new one restarts the frame's median.
        accepted_epoch: Option<u32>,
        /// The gyro fusion's view of that sample, in the frame's axes.
        fusion: Option<crate::feetech_snapshot::ImuFusionTick>,
    },
}

impl TrunkImu {
    fn kind(&self) -> TrunkImuKind {
        match self {
            Self::Bus { .. } => TrunkImuKind::Bus,
            Self::Host { reader, .. } => reader.kind(),
        }
    }
    fn ready(&self) -> bool {
        match self {
            Self::Bus { decoder, .. } => decoder.ready(),
            Self::Host { frame, .. } => frame.ready(),
        }
    }
    fn stale(&self) -> ImuStale {
        match self {
            Self::Bus { freshness, .. } => freshness.stale,
            Self::Host { gate, .. } => gate.stale(),
        }
    }
    fn sequence(&self) -> Option<u32> {
        match self {
            Self::Bus { freshness, .. } => freshness.last_sequence,
            Self::Host { gate, .. } => gate.sequence(),
        }
    }
    /// Age, age bound and FT6 status of the last accepted sample. A host sample's age is
    /// measured, so it is its own bound; it has no FT6 status byte.
    fn latest(&self) -> (Option<u16>, Option<u16>, Option<u8>) {
        match self {
            Self::Bus {
                latest_age_ms,
                latest_age_upper_bound_ms,
                latest_status,
                ..
            } => (*latest_age_ms, *latest_age_upper_bound_ms, *latest_status),
            Self::Host { latest_age_ms, .. } => (*latest_age_ms, *latest_age_ms, None),
        }
    }
    fn reader_health(&self) -> Option<HostImuHealth> {
        match self {
            Self::Bus { .. } => None,
            Self::Host { reader, .. } => Some(reader.health()),
        }
    }
    fn fusion(&self) -> Option<crate::feetech_snapshot::ImuFusionTick> {
        match self {
            Self::Bus { .. } => None,
            Self::Host { fusion, .. } => *fusion,
        }
    }
}

/// Official RobotIo with native FT6 reads and HD1910 position writes. Startup
/// is read-only; an explicit bring-up enables motion.
///
/// The trunk IMU is the FT6 board on the same bus ([`Self::open`], [`Self::from_bus`]) or a
/// module the host reads ([`Self::open_with_imu`], [`Self::from_bus_with_imu`]); which one
/// is fixed when it opens.
pub struct FeetechIo {
    pub bus: Bus,
    calibration: Calibration,
    imu: TrunkImu,
    latest_ready: bool,
    latest_slow: Option<SlowSensors>,
    latest_sample_at: Option<Instant>,
    joint_feedback: [Option<JointFeedback>; NUM_JOINTS],
    joint_missing: [u32; NUM_JOINTS],
    joint_missing_total: [u64; NUM_JOINTS],
    partial_samples_total: u64,
    latest_partial: bool,
    latest_native_positions: Option<[i32; NUM_JOINTS]>,
    latest_joints: Vec<FeetechJointDiagnostic>,
    torque_enabled: bool,
    command_enabled: bool,
    torque_state_confirmed: Option<bool>,
    command_clamps_total: u64,
    gain_notice_logged: bool,
    telemetry: TelemetryTotals,
    telemetry_sync: SyncTrace,
    hd1910: crate::hd1910::State,
    snapshot_metadata: std::sync::OnceLock<std::sync::Arc<crate::feetech_snapshot::Metadata>>,
}
impl FeetechIo {
    pub fn open(path: &str, calibration: Calibration) -> Result<Self> {
        calibration.validate()?;
        Self::from_bus(Bus::open(path)?, calibration)
    }
    /// [`Self::open`], with the trunk IMU read by the host through `reader`.
    pub fn open_with_imu(
        path: &str,
        calibration: Calibration,
        reader: Box<dyn HostImu>,
    ) -> Result<Self> {
        calibration.validate()?;
        Self::from_bus_with_imu(Bus::open(path)?, calibration, reader)
    }
    pub fn from_bus(mut bus: Bus, calibration: Calibration) -> Result<Self> {
        calibration.validate()?;
        bus.identify_imu()?;
        Self::for_startup(bus, calibration)?.adopt_torque_state()
    }
    /// The servos on `bus`; the trunk IMU from `reader`, under that source's own mount. ID 200
    /// is never addressed — not identified, not read — so a robot without the board needs
    /// none, and one that still has it is left alone.
    pub fn from_bus_with_imu(
        bus: Bus,
        calibration: Calibration,
        reader: Box<dyn HostImu>,
    ) -> Result<Self> {
        Self::for_startup_with_imu(bus, calibration, reader)?.adopt_torque_state()
    }
    /// The generic backend open preserves upstream's read-only behaviour.
    /// robotd uses confirm_startup_limp before handing ownership to its loop.
    fn adopt_torque_state(mut self) -> Result<Self> {
        let states = self.bus.sync_read(40, 1, &self.ids())?;
        let all_on = states.values().all(|v| v == &[1]);
        let all_off = states.values().all(|v| v == &[0]);
        self.torque_enabled = all_on;
        self.torque_state_confirmed = if all_on {
            Some(true)
        } else if all_off {
            Some(false)
        } else {
            None
        };
        self.hd1910.inspect(&mut self.bus, &self.calibration)?;
        Ok(self)
    }

    /// Construct without UART traffic, keeping targets disabled until explicit
    /// bring-up. The daemon can retry startup against this one open UART.
    pub fn for_startup(bus: Bus, calibration: Calibration) -> Result<Self> {
        calibration.validate()?;
        let imu = TrunkImu::Bus {
            decoder: TrunkFrame::new(calibration.imu_mount_quat),
            freshness: BusImuFreshness::default(),
            latest_age_ms: None,
            latest_age_upper_bound_ms: None,
            latest_status: None,
        };
        Ok(Self::with_trunk_imu(bus, calibration, imu))
    }
    /// [`Self::for_startup`], with the trunk IMU read by the host through `reader`.
    pub fn for_startup_with_imu(
        bus: Bus,
        calibration: Calibration,
        reader: Box<dyn HostImu>,
    ) -> Result<Self> {
        calibration.validate()?;
        let kind = reader.kind();
        if kind == TrunkImuKind::Bus {
            return Err(error(
                "the bus IMU board is read in the servos' Sync Read, not through a host reader",
            ));
        }
        let imu = TrunkImu::Host {
            frame: TrunkFrame::new(calibration.imu_mount(kind).quat),
            gate: HostImuGate::default(),
            reader,
            latest_age_ms: None,
            accepted_epoch: None,
            fusion: None,
        };
        Ok(Self::with_trunk_imu(bus, calibration, imu))
    }
    fn with_trunk_imu(bus: Bus, calibration: Calibration, imu: TrunkImu) -> Self {
        Self {
            bus,
            calibration,
            imu,
            latest_ready: false,
            latest_slow: None,
            latest_sample_at: None,
            joint_feedback: std::array::from_fn(|_| None),
            joint_missing: [0; NUM_JOINTS],
            joint_missing_total: [0; NUM_JOINTS],
            partial_samples_total: 0,
            latest_partial: false,
            latest_native_positions: None,
            latest_joints: Vec::new(),
            torque_enabled: false,
            command_enabled: false,
            torque_state_confirmed: None,
            command_clamps_total: 0,
            gain_notice_logged: false,
            telemetry: TelemetryTotals::new(),
            telemetry_sync: SyncTrace::default(),
            hd1910: crate::hd1910::State::default(),
            snapshot_metadata: std::sync::OnceLock::new(),
        }
    }

    /// Optional servo P/D/torque limit (servo-test-parameters.json), loaded by robotd before
    /// the bus is opened. None keeps the V167 arm path, its bus traffic and its gain_mode.
    pub fn set_servo_test(&mut self, parameters: Option<crate::hd1910_test::Parameters>) {
        self.hd1910.test_parameters = parameters;
    }
    fn gain_mode(&self) -> &'static str {
        if self.hd1910.test_parameters.is_some() {
            "hd1910_servo_parameters_file_relax_p_restored_d0_limit1000"
        } else {
            "hd1910_model_contract_p_restore_on_relax_native_d_torque_limit_preserved"
        }
    }

    /// Reboot recovery is always limp, including when the IMU is still booting.
    /// Only register 40 is written here; a failed attempt never opens the goal
    /// channel or writes gains/positions. This is a startup-only operation.
    /// ID 200's identity is checked only when the trunk IMU is that bus board: a module read
    /// by the host is never addressed on the servo bus.
    pub fn confirm_startup_limp(&mut self) -> Result<()> {
        self.command_enabled = false;
        self.torque_state_confirmed = None;
        let ids = self.ids();
        self.bus.torque(&ids, false)?;
        self.bus.verify_torque(&ids, false)?;
        self.torque_enabled = false;
        self.torque_state_confirmed = Some(false);
        if matches!(self.imu, TrunkImu::Bus { .. }) {
            self.bus.identify_imu()?;
        }
        Ok(())
    }
    fn ids(&self) -> Vec<u8> {
        self.calibration
            .joints
            .iter()
            .map(|joint| joint.id)
            .collect()
    }

    fn diagnostic_metadata(&self) -> crate::feetech_snapshot::Metadata {
        crate::feetech_snapshot::Metadata {
            configured_imu_profile: self.bus.imu_profile.label(),
            backend: "feetech_ft6",
            read_timing_revision: READ_TIMING_REVISION,
            motion_enabled: self.calibration.motion_enabled,
            calibration_complete: self.calibration.joints.iter().all(|j| j.calibrated),
            imu_mount_verified: self.imu_mount_verified(),
            imu_source: self.imu.kind().label(),
            imu_bus: self.bus.imu_info(),
            velocity_source: crate::hd1910::VELOCITY_SOURCE,
            velocity_rad_s_per_count: self.calibration.feedback_speed_scale(),
            current_available: self.calibration.units_verified,
            joints: self.calibration.joints.iter()
                .map(|j| crate::feetech_snapshot::JointMetadata {
                    name: j.name.clone(), id: j.id, calibrated: j.calibrated,
                }).collect(),
        }
    }

    fn snapshot_metadata(&self) -> std::sync::Arc<crate::feetech_snapshot::Metadata> {
        self.snapshot_metadata
            .get_or_init(|| std::sync::Arc::new(self.diagnostic_metadata()))
            .clone()
    }

    fn imu_mount_verified(&self) -> bool {
        self.calibration.imu_mount(self.imu.kind()).verified
            && (self.imu.kind() != TrunkImuKind::Bus
                || self
                    .calibration
                    .validate_bus_mount_for(self.bus.imu_board.protocol(), self.bus.imu_profile)
                    .is_ok())
    }
    fn require_fresh_sample(&self) -> Result<()> {
        if self
            .latest_sample_at
            .is_none_or(|time| time.elapsed() > Duration::from_millis(100))
            || self.latest_partial
            || self.joint_missing.iter().any(|n| *n != 0)
        {
            return Err(error(
                "Feetech motion requires a complete sample no older than 100 ms",
            ));
        }
        Ok(())
    }
}
impl FeetechIo {
    /// Optional temporary test settings, kept outside the official pad config.
    fn joint_holding(&self, i: usize, now: Instant) -> bool {
        self.joint_feedback[i].as_ref().is_some_and(|previous| {
            self.joint_missing[i] > JOINT_COAST_TICKS
                || now.saturating_duration_since(previous.at) > JOINT_COAST_MAX_AGE
        })
    }

    fn read_sample(&mut self, allow_partial: bool) -> Result<Sensors> {
        let started = Instant::now();
        let result = if matches!(self.imu, TrunkImu::Host { .. }) {
            self.bus.joint_telemetry(&self.ids())
        } else if allow_partial {
            self.bus.telemetry_partial(&self.ids())
        } else {
            self.bus.telemetry(&self.ids())
        };
        self.telemetry_sync = self.bus.last_sync.clone();
        self.telemetry.observe(
            &self.telemetry_sync,
            result.as_ref().err().map(ToString::to_string),
        );
        // Recovery still tracks unavailable samples. A request that was never
        // sent must not increase per-device physical missing-response totals.
        for (i, joint) in self.calibration.joints.iter().enumerate() {
            if self.bus.last_sync.missing_ids.contains(&joint.id) {
                self.joint_missing[i] = self.joint_missing[i].saturating_add(1);
                if self.telemetry_sync.request_sent {
                    self.joint_missing_total[i] = self.joint_missing_total[i].saturating_add(1);
                }
            }
        }
        let mut blocks = result?;
        let (imu, raw_imu, host_age) = match &mut self.imu {
            TrunkImu::Bus {
                decoder, freshness, ..
            } => {
                let raw_imu = self.bus.decode_imu(
                    &blocks
                        .remove(&IMU_ID)
                        .ok_or_else(|| error("Feetech Sync Read omitted IMU ID 200"))?,
                )?;
                freshness.check(&raw_imu, self.bus.last_sync.elapsed_us)?;
                (decoder.push(raw_imu.reading()), Some(raw_imu), None)
            }
            // Taken after the servos' transaction, so it is as fresh as this tick can have.
            TrunkImu::Host {
                reader,
                gate,
                frame,
                accepted_epoch,
                fusion,
                ..
            } => {
                let sample = gate.check(reader.latest(), Instant::now())?;
                if *accepted_epoch != Some(sample.epoch) {
                    frame.restart_median();
                    *accepted_epoch = Some(sample.epoch);
                }
                let reading = SensorReading {
                    gyro: sample.gyro,
                    quat: Some(sample.quat),
                };
                *fusion = sample.fusion.and_then(|f| {
                    let other = match f.mode {
                        crate::gyro_fusion::FusionMode::Chip => f.fused_quat,
                        crate::gyro_fusion::FusionMode::Gyro => f.chip_quat,
                    };
                    Some(crate::feetech_snapshot::ImuFusionTick {
                        mode: f.mode,
                        verdict: f.verdict,
                        theta_deg: f.theta_deg,
                        other_quat: frame.trunk_quat(other)?,
                    })
                });
                (frame.push(reading), None, gate.last_age())
            }
        };
        let mut out = Sensors {
            imu,
            ..Sensors::default()
        };
        let mut fresh = [false; NUM_JOINTS];
        let mut samples = Vec::with_capacity(NUM_JOINTS);
        let mut joints = Vec::with_capacity(NUM_JOINTS);
        let mut volts = 0.0;
        let mut temps = [0.0; NUM_JOINTS];
        let mut raws = [0; NUM_JOINTS];
        for (i, joint) in self.calibration.joints.iter().enumerate() {
            let sample = match blocks.remove(&joint.id) {
                Some(raw) => {
                    fresh[i] = true;
                    ServoSample::decode(&raw).map_err(|e| error(format!("ID {} {e}", joint.id)))?
                }
                None => self.joint_feedback[i]
                    .as_ref()
                    .ok_or_else(|| {
                        error(format!(
                            "ID {} missing without a previous valid sample",
                            joint.id
                        ))
                    })?
                    .sample
                    .clone(),
            };
            let fb = &sample.feedback;
            raws[i] = fb.position_signed_raw;
            out.positions[i] = if joint.calibrated {
                joint.position_rad(fb.position_raw)
            } else {
                f64::NAN
            };
            out.velocities[i] = if !fresh[i] {
                let previous = self.joint_feedback[i].as_ref().unwrap();
                if self.joint_holding(i, started) {
                    0.0
                } else {
                    previous.velocity
                }
            } else if !joint.calibrated {
                f64::NAN
            } else {
                f64::from(fb.speed_raw)
                    * crate::hd1910::SPEED_RAD_S_PER_COUNT
                    * f64::from(joint.direction)
            };
            out.currents_ma[i] = if self.calibration.units_verified {
                f64::from(sample.current_raw.abs()) * self.calibration.current_ma_per_count
            } else {
                f64::NAN
            };
            volts += fb.volts;
            temps[i] = f64::from(fb.temperature_c);
            joints.push(FeetechJointDiagnostic {
                name: joint.name.clone(),
                id: joint.id,
                calibrated: joint.calibrated,
                position_raw: fb.position_raw,
                position_wire_raw: fb.position_wire_raw,
                position_signed_raw: fb.position_signed_raw,
                speed_raw: fb.speed_raw,
                load_raw: fb.load_raw,
                current_raw: sample.current_raw,
                volts: fb.volts,
                temperature_c: fb.temperature_c,
                position_rad: out.positions[i].is_finite().then_some(out.positions[i]),
                velocity_rad_s: None,
                current_ma: out.currents_ma[i].is_finite().then_some(out.currents_ma[i]),
                feedback_fresh: fresh[i],
                sample_age_ms: 0,
                missing_total: self.joint_missing_total[i],
                consecutive_missing: if fresh[i] { 0 } else { self.joint_missing[i] },
                read_recovery: if fresh[i] {
                    "fresh"
                } else if self.joint_holding(i, started) {
                    "holding"
                } else {
                    "coasting"
                },
            });
            samples.push(sample);
        }
        if !blocks.is_empty() {
            return Err(error("unexpected Feetech Sync Read reply IDs"));
        }
        if let Some(raw_imu) = &raw_imu
            && !raw_imu.is_fresh(started.elapsed().as_micros() as u64)
        {
            return Err(error(
                "telemetry cycle exceeded the IMU freshness window",
            ));
        }
        self.hd1910.delay_velocity(started, &mut out, &fresh);
        // Commit together only after the current IMU and every used servo value validate.
        for (i, sample) in samples.into_iter().enumerate() {
            joints[i].velocity_rad_s = out.velocities[i].is_finite().then_some(out.velocities[i]);
            if fresh[i] {
                self.joint_feedback[i] = Some(JointFeedback {
                    at: started,
                    sample,
                    position: out.positions[i],
                    velocity: out.velocities[i],
                });
                self.joint_missing[i] = 0;
            }
        }
        self.latest_partial = fresh.iter().any(|v| !v);
        if self.latest_partial {
            self.partial_samples_total = self.partial_samples_total.saturating_add(1);
        }
        self.latest_sample_at = Some(started); // current IMU/control sample, not every joint
        self.latest_native_positions = Some(raws);
        self.latest_joints = joints;
        match &mut self.imu {
            TrunkImu::Bus {
                latest_age_ms,
                latest_age_upper_bound_ms,
                latest_status,
                ..
            } => {
                if let Some(raw_imu) = raw_imu {
                    *latest_age_ms = raw_imu.age_ms;
                    *latest_age_upper_bound_ms = raw_imu.age_upper_bound_ms;
                    *latest_status = raw_imu.status;
                }
            }
            TrunkImu::Host { latest_age_ms, .. } => {
                *latest_age_ms = host_age.map(|age| age.as_millis().min(u16::MAX.into()) as u16);
            }
        }
        self.latest_slow = Some(SlowSensors {
            volts: volts / NUM_JOINTS as f64,
            temps_c: temps,
        });
        self.latest_ready = true;
        Ok(out)
    }
}
impl RobotIo for FeetechIo {
    fn reboot(&mut self, _id: u8) -> crate::io::Result<()> {
        Err(crate::io::IoError::Bus(
            "HD1910 reboot is not supported by this native backend".into(),
        ))
    }
    fn measures_load(&self) -> bool {
        false
    }
    fn read(&mut self) -> Result<Sensors> {
        self.read_sample(true)
    }

    fn held_joint_positions(&self) -> [Option<f64>; NUM_JOINTS] {
        let now = Instant::now();
        std::array::from_fn(|i| {
            self.joint_feedback[i]
                .as_ref()
                .filter(|_| self.joint_holding(i, now))
                .map(|old| old.position)
        })
    }
    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        // Calibration is immutable here, validated at open and again before
        // torque-on. Per-target conversion below still checks angles and limits.
        // The official loop publishes targets even before bring-up. Do not leave
        // latent motion targets in an unpowered servo, or touch a restarted robot.
        if !self.command_enabled {
            return Ok(());
        }
        // The official control loop owns sample age: briefly coast on the last
        // observation, then command the last measured pose until reads recover.
        // Rejecting that hold because of its age would turn a read loss into a
        // write failure and release torque. Torque-on still requires fresh data.
        if self.latest_sample_at.is_none() {
            return Err(error("Feetech target write requires a completed sample"));
        }
        if targets.positions.iter().any(|v| !v.is_finite()) {
            return Err(error("Feetech target must be finite"));
        }
        let held = self.held_joint_positions();
        let mut clamps = 0;
        let rows = self
            .calibration
            .joints
            .iter()
            .zip(targets.positions)
            .enumerate()
            .map(|(i, (joint, radians))| {
                let (raw, clamped) = joint.target_raw(held[i].unwrap_or(radians))?;
                if clamped {
                    clamps += 1;
                }
                Ok((joint.id, i32::from(raw)))
            })
            .collect::<Result<Vec<_>>>()?;
        self.bus.signed_positions(&rows)?;
        self.command_clamps_total = self.command_clamps_total.saturating_add(clamps);
        Ok(())
    }
    fn set_gain(&mut self, _kp: u16) -> Result<()> {
        // XL330 gains are not a portable unit. Retain the configured HD1910
        // controller gains; explicit relax uses the native torque register.
        if !self.gain_notice_logged {
            tracing::warn!(
                "Feetech retains native servo gains; XL330 gain requests are not written"
            );
            self.gain_notice_logged = true;
        }
        Ok(())
    }
    fn set_torque(&mut self, on: bool) -> Result<()> {
        let ids = self.ids();
        let previously_off = self.torque_state_confirmed == Some(false);
        if on {
            if matches!(self.imu, TrunkImu::Bus { .. }) {
                self.calibration
                    .validate_bus_mount_for(self.bus.imu_board.protocol(), self.bus.imu_profile)?;
            }
            self.calibration.validate_motion_for(self.imu.kind())?;
            self.require_fresh_sample()?;
            if !self.imu_ready() {
                return Err(error("Feetech torque-on requires a fresh converged IMU"));
            }
            {
                self.hd1910.arm(&mut self.bus, &self.calibration)?;
                // Register inspection can take several transactions. Preload a NEW
                // measured position after it, never the sample from before setup.
                self.read_sample(false)?;
                self.require_fresh_sample()?;
            }
            let raw = self
                .latest_native_positions
                .ok_or_else(|| error("Feetech torque-on needs measured positions"))?;
            let rows = ids.iter().copied().zip(raw).collect::<Vec<_>>();
            // Replace any old stored goal BEFORE torque enable, then the official
            // home ramp starts from the same freshly measured pose.
            self.bus.signed_positions(&rows)?;
            let goals = self.bus.sync_read(42, 2, &ids)?;
            for (id, position) in rows {
                if goals[&id] != signed_position_word(position)?.to_le_bytes() {
                    return Err(error(format!(
                        "ID {id}: preloaded hold target not confirmed; torque not enabled"
                    )));
                }
            }
        }
        self.command_enabled = false;
        self.torque_state_confirmed = None;
        let verify = self
            .bus
            .torque(&ids, on)
            .and_then(|()| self.bus.verify_torque(&ids, on))
            .and_then(|()| {
                // The one-shot goal/torque readbacks can outlast a control tick.
                // Refresh from the real FT5 transaction before allowing its first
                // position command; never stretch the sample-age requirement.
                if on {
                    self.read_sample(false).map(|_| ())
                } else {
                    Ok(())
                }
            });
        if let Err(problem) = verify {
            // Roll back only a newly requested OFF -> ON transition. Do not drop
            // already powered motors because a confirmation reply was lost.
            if on && previously_off {
                if let Err(stop) = self.bus.torque(&ids, false) {
                    return Err(error(format!(
                        "{problem}; rollback torque-off also failed: {stop}"
                    )));
                }
                let stopped = self.bus.verify_torque(&ids, false);
                match stopped {
                    Ok(()) => self.torque_state_confirmed = Some(false),
                    Err(stop) => return Err(error(format!("{problem}; {stop}"))),
                }
            }
            return Err(problem);
        }
        self.torque_enabled = on;
        self.command_enabled = on;
        self.torque_state_confirmed = Some(on);
        if !on {
            self.hd1910.disarm(&mut self.bus)?;
        }
        Ok(())
    }
    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        self.latest_slow
            .ok_or_else(|| error("no completed Feetech Sync Read sample yet"))
    }
    fn imu_stale(&self) -> ImuStale {
        self.imu.stale()
    }
    fn imu_ready(&self) -> bool {
        self.latest_ready && self.imu.ready()
    }
    fn feetech_control_status(&self) -> Option<FeetechControlStatus> {
        let now = Instant::now();
        Some(FeetechControlStatus {
            all_joints_fresh: !self.latest_joints.is_empty()
                && (0..self.latest_joints.len()).all(|i| {
                    self.joint_missing[i] == 0
                        && self.joint_feedback[i]
                            .as_ref()
                            .is_some_and(|old| Some(old.at) == self.latest_sample_at)
                }),
            imu_mount_verified: self.imu_mount_verified(),
            imu_ready: self.imu_ready(),
            torque_state_confirmed: self.torque_state_confirmed,
            joints_held: (0..self.latest_joints.len()).any(|i| self.joint_holding(i, now)),
        })
    }
    fn feetech_diagnostics(&self) -> Option<FeetechDiagnostics> {
        let now = Instant::now();
        let mut joints = self.latest_joints.clone();
        let mut stale_joint_ids = Vec::new();
        let mut held_joint_ids = Vec::new();
        for (i, diagnostic) in joints.iter_mut().enumerate() {
            let old = self.joint_feedback[i].as_ref().unwrap();
            diagnostic.sample_age_ms = now.saturating_duration_since(old.at).as_millis() as u64;
            diagnostic.missing_total = self.joint_missing_total[i];
            diagnostic.consecutive_missing = self.joint_missing[i];
            diagnostic.feedback_fresh =
                self.joint_missing[i] == 0 && Some(old.at) == self.latest_sample_at;
            if !diagnostic.feedback_fresh {
                stale_joint_ids.push(diagnostic.id);
            }
            if self.joint_holding(i, now) {
                diagnostic.read_recovery = "holding";
                held_joint_ids.push(diagnostic.id);
            } else if !diagnostic.feedback_fresh {
                diagnostic.read_recovery = "coasting";
            }
        }
        let (imu_age_ms, imu_age_upper_bound_ms, imu_status) = self.imu.latest();
        Some(FeetechDiagnostics {
            servo_observations: self.bus.observations.snapshot(std::sync::Arc::new(self.diagnostic_metadata()), now),
            configured_imu_profile: self.bus.imu_profile.label(),
            backend: "feetech_ft6",
            read_timing_revision: READ_TIMING_REVISION,
            motion_enabled: self.calibration.motion_enabled,
            calibration_complete: self.calibration.joints.iter().all(|j| j.calibrated),
            imu_mount_verified: self.imu_mount_verified(),
            torque_enabled: self.torque_enabled,
            torque_state_confirmed: self.torque_state_confirmed,
            gain_mode: self.gain_mode(),
            velocity_source: crate::hd1910::VELOCITY_SOURCE,
            velocity_rad_s_per_count: self.calibration.feedback_speed_scale(),
            current_available: self.calibration.units_verified,
            sample_age_ms: self
                .latest_sample_at
                .map(|time| time.elapsed().as_millis() as u64),
            imu_ready: self.imu_ready(),
            imu_sequence: self.imu.sequence(),
            imu_age_ms,
            imu_age_upper_bound_ms,
            imu_status,
            imu_source: self.imu.kind().label(),
            imu_bus: self.bus.imu_info(),
            imu_reader: self.imu.reader_health(),
            command_clamps_total: self.command_clamps_total,
            partial_samples_total: self.partial_samples_total,
            all_joints_fresh: stale_joint_ids.is_empty() && !joints.is_empty(),
            stale_joint_ids,
            held_joint_ids,
            joints,
            sync: self.telemetry_sync.clone(),
            telemetry: self.telemetry.clone(),
            hd1910: self.hd1910.diagnostics(),
        })
    }

    fn feetech_sync_trace(&self) -> Option<&SyncTrace> {
        Some(&self.telemetry_sync)
    }

    fn feetech_snapshot(&self) -> Option<crate::feetech_snapshot::Snapshot> {
        use crate::feetech_snapshot as snapshot;
        let metadata = self.snapshot_metadata();
        let now = Instant::now();
        let mut joints = snapshot::Samples::default();
        let mut stale_joint_ids = snapshot::Samples::default();
        let mut held_joint_ids = snapshot::Samples::default();
        for (i, original) in self.latest_joints.iter().enumerate() {
            let previous = self.joint_feedback[i].as_ref().unwrap();
            let fresh = self.joint_missing[i] == 0 && Some(previous.at) == self.latest_sample_at;
            let held = self.joint_holding(i, now);
            if !fresh {
                stale_joint_ids.values[stale_joint_ids.len] = original.id;
                stale_joint_ids.len += 1;
            }
            if held {
                held_joint_ids.values[held_joint_ids.len] = original.id;
                held_joint_ids.len += 1;
            }
            joints.values[i] = snapshot::Joint {
                id: original.id,
                position_raw: original.position_raw,
                position_wire_raw: original.position_wire_raw,
                position_signed_raw: original.position_signed_raw,
                speed_raw: original.speed_raw,
                load_raw: original.load_raw,
                current_raw: original.current_raw,
                volts: original.volts,
                temperature_c: original.temperature_c,
                position_rad: original.position_rad,
                velocity_rad_s: original.velocity_rad_s,
                current_ma: original.current_ma,
                feedback_fresh: fresh,
                sample_age_ms: now.saturating_duration_since(previous.at).as_millis() as u64,
                missing_total: self.joint_missing_total[i],
                consecutive_missing: self.joint_missing[i],
                read_recovery: if held {
                    "holding"
                } else if !fresh {
                    "coasting"
                } else {
                    "fresh"
                },
            };
            joints.len += 1;
        }
        let hd1910 = self.hd1910.diagnostics();
        let (imu_age_ms, imu_age_upper_bound_ms, imu_status) = self.imu.latest();
        Some(snapshot::Snapshot {
            metadata: metadata.clone(),
            dynamic: snapshot::Dynamic {
                servo_observations: self.bus.observations.snapshot(metadata.clone(), now),
                torque_enabled: self.torque_enabled,
                torque_state_confirmed: self.torque_state_confirmed,
                gain_mode: self.gain_mode(),
                sample_age_ms: self
                    .latest_sample_at
                    .map(|at| now.saturating_duration_since(at).as_millis() as u64),
                imu_ready: self.imu_ready(),
                imu_sequence: self.imu.sequence(),
                imu_age_ms,
                imu_age_upper_bound_ms,
                imu_status,
                imu_reader: self.imu.reader_health(),
                command_clamps_total: self.command_clamps_total,
                partial_samples_total: self.partial_samples_total,
                all_joints_fresh: stale_joint_ids.len == 0 && joints.len != 0,
                stale_joint_ids,
                held_joint_ids,
                joints: snapshot::JointSamples {
                    samples: joints,
                    metadata,
                },
                sync: self.telemetry_sync.clone(),
                telemetry: self.telemetry.clone(),
                hd1910: snapshot::HdState {
                    temporary_gains_applied: hd1910.temporary_gains_applied,
                    restore_pending: hd1910.restore_pending,
                },
            },
            hd1910,
            imu_fusion: self.imu.fusion(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn qinmo_profile_binding_requires_a_new_verification_even_on_feetech() {
        let cal = Calibration::commissioning(crate::model::JOINT_IDS);
        // Top-level keys must precede the joint table array.
        let text = format!(
            "imu_bus_profile = 'qinmo'\n{}",
            toml::to_string(&cal).unwrap()
        );
        let parsed = toml::from_str::<Calibration>(&text);
        assert!(
            parsed.is_ok(),
            "explicit calibration profile must deserialize: {parsed:?}"
        );
        let bound = parsed.unwrap();
        assert!(
            bound
                .validate_bus_mount_for(BusImuProtocol::Feetech, BusImuProfile::Qinmo)
                .is_ok()
        );
        assert!(
            bound
                .validate_bus_mount_for(BusImuProtocol::Feetech, BusImuProfile::Legacy)
                .is_err()
        );
        assert!(
            cal.validate_bus_mount_for(BusImuProtocol::Feetech, BusImuProfile::Qinmo)
                .is_err()
        );
        assert!(
            cal.validate_bus_mount_for(BusImuProtocol::Feetech, BusImuProfile::Legacy)
                .is_ok()
        );
        assert!(
            bound
                .validate_bus_mount_for(BusImuProtocol::Dxl2, BusImuProfile::Qinmo)
                .is_err()
        );
        let saved = toml::to_string(&bound).unwrap();
        let restored: Calibration = toml::from_str(&saved).unwrap();
        assert_eq!(restored.imu_bus_profile, Some(BusImuProfile::Qinmo));
    }

    #[test]
    fn qinmo_core_units_ready_identity_and_invalid_frames() {
        let mut raw = [0u8; 15];
        raw[..2].copy_from_slice(&1640i16.to_le_bytes());
        raw[2..4].copy_from_slice(&(-1640i16).to_le_bytes());
        let sample = ImuSample::decode_qinmo(&raw).unwrap();
        let reading = sample.reading();
        assert!((reading.gyro[0].to_degrees() - 100.0).abs() < 1e-10);
        assert!((reading.gyro[1].to_degrees() + 100.0).abs() < 1e-10);
        assert_eq!(reading.quat, Some([1.0, 0.0, 0.0, 0.0]));
        assert_eq!(sample.age_upper_bound_ms, None);
        let legacy = ImuSample::decode(&raw).unwrap().reading();
        assert!((legacy.gyro[0].to_degrees() - 28.7).abs() < 1e-10);
        assert!(legacy.quat.is_none());
        let mut frame = TrunkFrame::new([1.0, 0.0, 0.0, 0.0]);
        for _ in 0..24 {
            frame.push(reading);
        }
        assert!(!frame.ready());
        frame.push(reading);
        assert!(frame.ready());
        raw[13] = 1;
        let initializing = ImuSample::decode_qinmo(&raw).unwrap();
        assert!(!initializing.ready);
        assert!(initializing.reading().quat.is_none());
        assert!(!initializing.is_fresh(0));
        for status in [2, 4, 8, 0x80] {
            raw[13] = status;
            assert!(ImuSample::decode_qinmo(&raw).is_err());
        }
        raw[13] = 0;
        raw[14] = 1;
        assert!(ImuSample::decode_qinmo(&raw).is_err());
        raw[14] = 0;
        for packed in [0x7c00u16, 0x7e00, 0x4000] {
            raw[6..8].copy_from_slice(&packed.to_le_bytes());
            assert!(ImuSample::decode_qinmo(&raw).is_err());
        }
        assert!(ImuSample::decode_qinmo(&raw[..12]).is_err());
    }

    #[test]
    fn qinmo_freshness_uses_counter_wrap_transaction_and_duplicate_guard() {
        let mut gate = BusImuFreshness::default();
        let mut raw = [0u8; 15];
        for seq in [254, 255, 0, 1] {
            raw[12] = seq;
            let sample = ImuSample::decode_qinmo(&raw).unwrap();
            assert!(gate.check(&sample, 10_000).is_ok());
        }
        let sample = ImuSample::decode_qinmo(&raw).unwrap();
        assert!(gate.check(&sample, 10_000).is_ok());
        assert!(gate.check(&sample, 10_000).is_ok());
        assert!(gate.check(&sample, 10_000).is_err());
        raw[12] = 2;
        let sample = ImuSample::decode_qinmo(&raw).unwrap();
        assert!(gate.check(&sample, 20_000).is_ok());
        raw[12] = 3;
        assert!(
            gate.check(&ImuSample::decode_qinmo(&raw).unwrap(), 20_001)
                .is_err()
        );
    }

    /// ID 200 is kissqy's FT6 board or our own imu_to_dxl (0.2.0 and 1.0.0 report model 0x4D44);
    /// anything else, including a big-endian flag, is refused.
    #[test]
    fn bus_imu_board_is_known_by_its_identity_bytes() {
        assert_eq!(BusImuBoard::from_identity(&IMU_IDENTITY), Some(BusImuBoard::Ft6));
        assert_eq!(BusImuBoard::from_identity(&[0, 2, 0, 0x44, 0x4d]), Some(BusImuBoard::ImuToDxl));
        assert_eq!(BusImuBoard::from_identity(&[1, 0, 0, 0x44, 0x4d]), Some(BusImuBoard::ImuToDxl));
        for raw in [&[1u8, 0, 1, 0x44, 0x4d][..], &[6, 0, 0, 0, 0xf3], &[0, 0, 0, 0, 0], &[6, 0, 0, 0]] {
            assert_eq!(BusImuBoard::from_identity(raw), None, "{raw:02x?}");
        }
        assert!(unknown_imu(&[1, 2, 3, 4, 5]).to_string().contains("[01, 02, 03, 04, 05]"));
    }

    /// FT6 answers first; imu_to_dxl last, after hearing the servo before it (first in the list,
    /// the next Feetech servo talks over it). Between servos is never sent.
    #[test]
    fn the_imu_is_first_for_ft6_and_last_for_imu_to_dxl() {
        let servos = [10, 11, 12];
        assert_eq!(super::telemetry_ids(BusImuBoard::Ft6, &servos), vec![IMU_ID, 10, 11, 12]);
        assert_eq!(super::telemetry_ids(BusImuBoard::ImuToDxl, &servos), vec![10, 11, 12, IMU_ID]);
        assert!(sync_read_packet(SYNC_ADDRESS, SYNC_LENGTH, &[IMU_ID, 10, 11]).is_ok());
        assert!(sync_read_packet(SYNC_ADDRESS, SYNC_LENGTH, &[10, 11, IMU_ID]).is_ok());
        assert!(sync_read_packet(SYNC_ADDRESS, SYNC_LENGTH, &[10, IMU_ID, 11]).is_err());
        assert!(sync_read_packet(SYNC_ADDRESS, SYNC_LENGTH, &[IMU_ID]).is_ok());
    }

    #[test]
    fn position_feedback_and_angles_keep_the_unmodified_signed_coordinate() {
        for (wire, expected) in [
            (0u16, 0),
            (1412, 1412),
            (4095, 4095),
            (32768, 0),
            (34180, -1412),
            (36863, -4095),
            (4096, 4096),
            (5000, 5000),
            (0x9000, -4096),
            (0x7fff, 32767),
            (0xffff, -32767),
            (0x8001, -1),
        ] {
            let mut raw = [0u8; 15];
            raw[..2].copy_from_slice(&wire.to_le_bytes());
            let sample = ServoSample::decode(&raw).unwrap();
            assert_eq!(sample.feedback.position_raw, expected);
            assert_eq!(sample.feedback.position_wire_raw, wire);
            assert_eq!(sample.feedback.position_signed_raw, expected);
        }
        let c = calibrated();
        for id in [12, 22] {
            let joint = c.joints.iter().find(|j| j.id == id).unwrap();
            for position in [-32767, -4095, -1412, -1, 4096, 32767] {
                let expected = (f64::from(position) - f64::from(joint.zero_raw)) * TAU / 4096.0
                    * f64::from(joint.direction);
                assert_eq!(joint.position_rad(position), expected);
            }
        }
    }

    #[test]
    fn native_goal_encoding_is_signed_magnitude_not_twos_complement_or_modulo() {
        for position in [
            -32767, -6048, -4096, -1820, -1323, -1, 0, 2773, 4096, 6244, 32767,
        ] {
            let command = signed_position_command(position).unwrap();
            let wire = u16::from_le_bytes([command[0], command[1]]);
            assert_eq!(signed_magnitude(wire, 15), position);
            assert_eq!(command[2..], [0, 0, 0, 0]);
        }
        assert_eq!(signed_position_word(-1323).unwrap(), 34091);
        for position in [-32768, 32768, i32::MIN, i32::MAX] {
            assert!(signed_position_command(position).is_err());
        }
        // The separate commissioning single-turn API keeps its existing range.
        assert!(position_command(4096).is_err());
    }

    #[test]
    fn telemetry_totals_preserve_every_missing_reply_through_recovery() {
        let mut totals = TelemetryTotals::new();
        let mut trace = SyncTrace {
            address: 56,
            length: 15,
            request_sent: true,
            missing_ids: vec![14, 32],
            ..SyncTrace::default()
        };
        totals.observe(&trace, Some("missing 14,32".into()));
        trace.missing_ids = vec![32];
        totals.observe(&trace, Some("missing 32".into()));
        trace.missing_ids.clear();
        totals.observe(&trace, None);
        assert_eq!((totals.total_reads, totals.failed_reads), (3, 2));
        assert_eq!(totals.missing_by_id, BTreeMap::from([(14, 1), (32, 2)]));
        assert_eq!(
            totals.last_missing_read_by_id,
            BTreeMap::from([(14, 1), (32, 2)])
        );
        assert!(totals.last_error.is_none());
        assert_eq!(totals.last_failure.as_ref().unwrap().read, 2);
        assert_eq!(totals.last_failure.as_ref().unwrap().missing_ids, [32]);
        assert!(!totals.session.is_empty());
        assert_ne!(totals.session, TelemetryTotals::new().session);
    }

    #[test]
    fn telemetry_totals_do_not_blame_devices_for_late_or_unsent_requests() {
        let mut totals = TelemetryTotals::new();
        let mut trace = SyncTrace {
            address: 56,
            length: 15,
            request_sent: true,
            complete_but_late: true,
            ..SyncTrace::default()
        };
        totals.observe(
            &trace,
            Some("complete but transaction deadline exceeded".into()),
        );
        trace.request_sent = false;
        trace.complete_but_late = false;
        trace.missing_ids = telemetry_ids();
        totals.observe(&trace, Some("serial send failed".into()));
        assert_eq!(
            (
                totals.total_reads,
                totals.failed_reads,
                totals.complete_but_late
            ),
            (2, 2, 1)
        );
        assert!(totals.missing_by_id.is_empty());
        assert!(totals.last_failure.as_ref().unwrap().missing_ids.is_empty());
        // Torque confirmation is outside the 56/15 telemetry statistics.
        trace.address = 40;
        trace.length = 1;
        trace.request_sent = true;
        totals.observe(&trace, Some("torque confirmation failed".into()));
        assert_eq!((totals.total_reads, totals.failed_reads), (2, 2));
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn read_loss_preserves_ready_and_allows_hold_but_not_stale_torque_on() {
        use serialport::SerialPort;
        use std::sync::{
            Arc,
            atomic::{AtomicBool, AtomicUsize, Ordering},
        };
        let (mut master, mut slave) = serialport::TTYPort::pair().unwrap();
        master.set_timeout(Duration::from_millis(100)).unwrap();
        slave.set_timeout(Duration::from_millis(2)).unwrap();
        let done = Arc::new(AtomicBool::new(false));
        let drop_read = Arc::new(AtomicBool::new(false));
        let goals = Arc::new(AtomicUsize::new(0));
        let (exit, drop, writes) = (done.clone(), drop_read.clone(), goals.clone());
        let device = std::thread::spawn(move || {
            let mut torque = false;
            let mut positions = [2048u16; 256];
            let mut gains = [[32u8, 40u8]; 256];
            let mut sequence = 0u8;
            while !exit.load(Ordering::Relaxed) {
                let mut header = [0u8; 4];
                if master.read_exact(&mut header).is_err() {
                    continue;
                }
                if header == crate::dxl2::HEADER {
                    // This peer is a Feetech-only board. Consume and ignore
                    // a read-only DXL2 fallback probe, just as real firmware does.
                    let mut rest = [0u8; 3];
                    master.read_exact(&mut rest).unwrap();
                    let mut body = vec![0; usize::from(u16::from_le_bytes([rest[1], rest[2]]))];
                    master.read_exact(&mut body).unwrap();
                    continue;
                }
                let mut body = vec![0u8; header[3] as usize];
                master.read_exact(&mut body).unwrap();
                let address = body[1];
                let length = body[2] as usize;
                if body[0] == SYNC_WRITE {
                    for row in body[3..body.len() - 1].chunks_exact(length + 1) {
                        if address == 40 {
                            torque = row[1] != 0;
                        }
                        if address == 42 {
                            assert_eq!(length, 6);
                            positions[row[0] as usize] = u16::from_le_bytes([row[1], row[2]]);
                            assert_eq!(&row[3..7], &[0; 4]);
                            writes.fetch_add(1, Ordering::Relaxed);
                        }
                        if address == 50 {
                            assert_eq!(length, 1, "only P may be written");
                            gains[row[0] as usize][0] = row[1];
                        }
                    }
                    continue;
                }
                if address == 56 && drop.load(Ordering::Relaxed) {
                    continue;
                }
                let ids = if body[0] == SYNC_READ {
                    body[3..body.len() - 1].to_vec()
                } else {
                    vec![header[2]]
                };
                sequence = sequence.wrapping_add(1);
                let mut responses = Vec::new();
                for id in ids {
                    let mut data = vec![0u8; length];
                    match (id, address) {
                        (IMU_ID, 0) => data.copy_from_slice(&IMU_IDENTITY),
                        (IMU_ID, 56) => {
                            data[7] = 0x30;
                            data[12] = sequence;
                        }
                        (_, 21) => data.copy_from_slice(&[32, 40, 0]),
                        (_, 33) => data[0] = 4,
                        (_, 40) => data[0] = u8::from(torque),
                        (_, 41) => {
                            data[1..3].copy_from_slice(&positions[id as usize].to_le_bytes())
                        }
                        (_, 42) => data.copy_from_slice(&positions[id as usize].to_le_bytes()),
                        (_, 50) => {
                            let gain_length = length.min(2);
                            data[..gain_length].copy_from_slice(&gains[id as usize][..gain_length]);
                        }
                        (_, 56) => {
                            data[..2].copy_from_slice(&2048u16.to_le_bytes());
                            data[6] = 73;
                            data[7] = 35;
                        }
                        _ => panic!("unexpected register {address}"),
                    }
                    responses.extend(packet(id, 0, &data).unwrap());
                }
                master.write_all(&responses).unwrap();
            }
        });
        let bus = Bus {
            port: Box::new(slave),
            timeout: Duration::from_millis(30),
            read_only: false,
            last_sync: SyncTrace::default(),
            imu_board: BusImuBoard::Ft6,
            imu_protocol: BusImuProtocol::Auto,
            imu_profile: BusImuProfile::Legacy,
            imu_identity: None,
            observations: observations::Observations::default(),
        };
        let mut io = FeetechIo::from_bus(bus, calibrated()).unwrap();
        for _ in 0..30 {
            io.read().unwrap();
        }
        assert!(io.imu_ready());
        io.set_torque(true).unwrap();
        let reads_before = io.telemetry.total_reads;
        drop_read.store(true, Ordering::Relaxed);
        let last = io.latest_sample_at;
        assert!(io.read().is_err());
        let totals = io.feetech_diagnostics().unwrap().telemetry;
        assert_eq!(
            (totals.total_reads, totals.failed_reads),
            (reads_before + 1, 1)
        );
        assert_eq!(totals.missing_by_id.len(), 16);
        assert!(totals.missing_by_id.values().all(|count| *count == 1));
        assert!(
            io.imu_ready(),
            "missing reply is not an IMU convergence reset"
        );
        assert_eq!(
            io.latest_sample_at, last,
            "old feedback must never be retimestamped as fresh"
        );
        io.latest_sample_at = Some(Instant::now() - Duration::from_millis(200));
        let before = goals.load(Ordering::Relaxed);
        io.write(&JointTargets::new([0.1; NUM_JOINTS])).unwrap();
        assert!(
            io.set_torque(true)
                .unwrap_err()
                .to_string()
                .contains("100 ms")
        );
        io.set_torque(false).unwrap(); // round trip also flushes the preceding goal packet
        assert_eq!(goals.load(Ordering::Relaxed) - before, NUM_JOINTS);
        assert_eq!(io.torque_state_confirmed, Some(false));
        for _ in 0..3 {
            let diagnostic = io.feetech_diagnostics().unwrap();
            assert_eq!(diagnostic.sync.address, 56);
            assert_eq!(diagnostic.sync.missing_ids.len(), 16);
            assert_eq!(diagnostic.telemetry.total_reads, reads_before + 1);
        }
        done.store(true, Ordering::Relaxed);
        device.join().unwrap();
    }

    fn telemetry_ids() -> Vec<u8> {
        std::iter::once(IMU_ID)
            .chain(crate::model::JOINT_IDS)
            .collect()
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn unified_imu_read_requires_all_16_frames_and_no_extra_transaction() {
        use serialport::SerialPort;
        for outcome in 0..4 {
            let (mut master, mut slave) = serialport::TTYPort::pair().unwrap();
            master.set_timeout(Duration::from_secs(1)).unwrap();
            slave.set_timeout(Duration::from_millis(2)).unwrap();
            let (finish, wait) = std::sync::mpsc::channel();
            let device = std::thread::spawn(move || {
                let mut request = [0; 24];
                master.read_exact(&mut request).unwrap();
                assert_eq!(
                    request.to_vec(),
                    sync_read_packet(56, 15, &telemetry_ids()).unwrap()
                );
                let mut responses = Vec::new();
                if outcome != 3 {
                    let mut imu = [0; 15];
                    imu[12] = 255;
                    let mut response =
                        packet(IMU_ID, if outcome == 2 { 4 } else { 0 }, &imu).unwrap();
                    if outcome == 1 {
                        *response.last_mut().unwrap() ^= 0x80;
                    }
                    responses.extend(response);
                }
                for id in crate::model::JOINT_IDS {
                    responses.extend(packet(id, 0, &[0; 15]).unwrap());
                }
                master.write_all(&responses).unwrap();
                wait.recv().unwrap();
                assert_eq!(master.bytes_to_read().unwrap(), 0);
            });
            let mut bus = Bus {
                port: Box::new(slave),
                timeout: Duration::from_millis(30),
                read_only: false,
                last_sync: SyncTrace::default(),
                imu_board: BusImuBoard::Ft6,
                imu_protocol: BusImuProtocol::Auto,
                imu_profile: BusImuProfile::Legacy,
                imu_identity: None,
                observations: observations::Observations::default(),
            };
            let result = bus.telemetry(&crate::model::JOINT_IDS);
            finish.send(()).unwrap();
            device.join().unwrap();
            assert_eq!(result.is_ok(), outcome == 0);

            assert_eq!(bus.last_sync.rx_bytes, if outcome == 3 { 315 } else { 336 });
            assert_eq!(bus.last_sync.bad_checksums, u64::from(outcome == 1));
            if outcome == 0 {
                let replies = result.unwrap();
                assert_eq!(replies.len(), 16);
                assert_eq!(replies[&IMU_ID].len(), 15);
            } else {
                assert_eq!(bus.last_sync.missing_ids, vec![IMU_ID]);
            }
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn sync_torque_checks_all_fifteen_fragmented_replies_and_reports_missing() {
        use serialport::SerialPort;
        let (mut master, mut slave) = serialport::TTYPort::pair().unwrap();
        master.set_timeout(Duration::from_secs(1)).unwrap();
        slave.set_timeout(Duration::from_millis(2)).unwrap();
        let ids = crate::model::JOINT_IDS;
        let (finish, wait) = std::sync::mpsc::channel();
        let device = std::thread::spawn(move || {
            for attempt in 0..3 {
                let mut request = [0u8; 23];
                master.read_exact(&mut request).unwrap();
                assert_eq!(request.to_vec(), sync_read_packet(40, 1, &ids).unwrap());
                for (index, id) in ids.iter().enumerate() {
                    if attempt == 2 && index == 14 {
                        continue;
                    }
                    let value = u8::from(attempt == 1 && index == 14);
                    let response = packet(*id, 0, &[value]).unwrap();
                    for bytes in response.chunks(2) {
                        master.write_all(bytes).unwrap();
                    }
                }
            }
            wait.recv().unwrap();
        });
        let mut bus = Bus {
            port: Box::new(slave),
            timeout: Duration::from_millis(30),
            read_only: false,
            last_sync: SyncTrace::default(),
            imu_board: BusImuBoard::Ft6,
            imu_protocol: BusImuProtocol::Auto,
            imu_profile: BusImuProfile::Legacy,
            imu_identity: None,
            observations: observations::Observations::default(),
        };
        bus.verify_torque(&ids, false).unwrap();
        assert!(bus.last_sync.missing_ids.is_empty());

        assert!(
            bus.verify_torque(&ids, false)
                .unwrap_err()
                .to_string()
                .contains("not confirmed")
        );
        assert!(bus.verify_torque(&ids, false).is_err());
        assert_eq!(bus.last_sync.missing_ids, vec![ids[14]]);
        assert_eq!(bus.last_sync.missing_ids.len(), 1);

        finish.send(()).unwrap();
        device.join().unwrap();
    }
    #[test]
    fn verified_hat_read_and_reply() {
        assert_eq!(
            packet(1, READ, &[56, 11]).unwrap(),
            [255, 255, 1, 4, 2, 56, 11, 181]
        );
        let bytes = [
            255, 255, 1, 13, 0, 0x4d, 7, 0, 0, 0, 0, 59, 30, 0, 0, 0, 0x44,
        ];
        let mut d = Decoder::default();
        let s = bytes.into_iter().find_map(|b| d.push(b)).unwrap();
        let f = Feedback::decode(&s.data).unwrap();
        assert_eq!(f.position_raw, 1869);
        assert!((f.volts - 5.9).abs() < 1e-10);
        assert_eq!(f.temperature_c, 30);
        assert!(!f.moving);
    }
    #[test]
    fn rejects_damage_and_resynchronises() {
        let valid = packet(2, 0, &[7]).unwrap();
        let mut bad = valid.clone();
        *bad.last_mut().unwrap() ^= 1;
        let mut d = Decoder::default();
        for b in [8, 255, 0, 255, 255, 255].into_iter().chain(bad) {
            assert!(d.push(b).is_none());
        }
        let status = valid.into_iter().find_map(|b| d.push(b)).unwrap();
        assert_eq!(status.id, 2);
        assert_eq!(status.data, [7]);
    }
    #[test]
    fn fifteen_joint_sync_fits_single_native_packet() {
        let rows = (1..=15)
            .map(|id| (id, position_command(2048).unwrap().to_vec()))
            .collect::<Vec<_>>();
        let bytes = sync_write_packet(42, &rows).unwrap();
        assert_eq!(bytes.len(), 113);
        assert_eq!(bytes[2..7], [254, 109, 0x83, 42, 6]);
        assert_eq!(bytes[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b)), 255);
    }
    #[test]
    fn combined_sync_read_packet_is_native_and_imu_first() {
        let bytes = sync_read_packet(56, 20, &[200, 1, 2]).unwrap();
        assert_eq!(bytes, [255, 255, 254, 7, 0x82, 56, 20, 200, 1, 2, 0x61]);
        assert!(sync_read_packet(56, 20, &[1, 200, 2]).is_err());
        assert!(sync_read_packet(56, 20, &[200, 1, 1]).is_err());
        assert!(sync_read_packet(250, 20, &[200]).is_err());
    }

    #[test]
    fn sync_read_collector_requires_each_exact_reply_once() {
        let mut pending = [1, 2].into_iter().collect::<BTreeSet<_>>();
        let mut replies = BTreeMap::new();
        assert!(
            !accept_sync_status(
                &mut pending,
                &mut replies,
                Status {
                    id: 2,
                    error: 0,
                    data: vec![2; 20],
                },
                20,
            )
            .unwrap()
        );
        assert!(
            accept_sync_status(
                &mut pending,
                &mut replies,
                Status {
                    id: 1,
                    error: 0,
                    data: vec![1; 20],
                },
                20,
            )
            .unwrap()
        );
        assert_eq!(replies.len(), 2);

        let mut pending = [1].into_iter().collect::<BTreeSet<_>>();
        let mut replies = BTreeMap::new();
        assert!(
            accept_sync_status(
                &mut pending,
                &mut replies,
                Status {
                    id: 2,
                    error: 0,
                    data: vec![0; 20],
                },
                20,
            )
            .is_err()
        );

        let mut pending = [1].into_iter().collect::<BTreeSet<_>>();
        let mut replies = BTreeMap::new();
        assert!(
            accept_sync_status(
                &mut pending,
                &mut replies,
                Status {
                    id: 1,
                    error: 0,
                    data: vec![0; 19],
                },
                20,
            )
            .is_err()
        );
    }
    #[test]
    fn validates_motion_before_send() {
        assert!(position_command(4096).is_err());
        assert_eq!(position_command(2048).unwrap(), [0, 8, 0, 0, 0, 0]);
        assert!(sync_write_packet(40, &[(1, vec![0]), (1, vec![0])]).is_err());
        assert!(sync_write_packet(40, &[(IMU_ID, vec![0])]).is_err());
    }
    #[test]
    fn sign_is_not_twos_complement() {
        assert_eq!(signed_magnitude(0x8064, 15), -100);
        assert_eq!(signed_magnitude(0x040a, 10), -10);
        assert_eq!(signed_magnitude(0x8000, 15), 0);
    }
    #[test]
    fn imu_layout_status_and_freshness_are_mandatory() {
        let mut raw = [0; 15];
        for sequence in [254, 255, 0, 1] {
            raw[12] = sequence;
            let sample = ImuSample::decode(&raw).unwrap();
            assert_eq!(sample.sequence, Some(u32::from(sequence)));
            assert!(sample.age_ms.is_none());
            assert_eq!(sample.age_upper_bound_ms, Some(30));
            assert!(sample.is_fresh(15_000));
            assert!(!sample.is_fresh(20_001));
        }
        for status in [1, 2, 4, 3, 6, 7] {
            raw[13] = status;
            assert!(!ImuSample::decode(&raw).unwrap().is_fresh(0));
        }
        raw[13] = 8; // slow reader is diagnostic, not fabricated sensor failure
        assert!(ImuSample::decode(&raw).unwrap().is_fresh(0));
        raw[13] = 16;
        assert!(ImuSample::decode(&raw).is_err());
        raw[13] = 0;
        raw[14] = 1;
        assert!(ImuSample::decode(&raw).is_err());
        assert!(ImuSample::decode(&[0; 20]).is_err());
        let mut diagnostic = [0; 20];
        diagnostic[12..16].copy_from_slice(&u32::MAX.to_le_bytes());
        diagnostic[16] = 9;
        diagnostic[18] = 1;
        diagnostic[19] = 1;
        let sample = ImuSample::decode_diagnostic(&diagnostic).unwrap();
        assert_eq!(sample.sequence, Some(u32::MAX));
        assert_eq!(sample.age_ms, Some(9));
        assert!(sample.is_fresh(15_000));
        assert!(ImuSample::decode_diagnostic(&raw).is_err());
    }
    #[test]
    fn no_calibration_from_two_bench_ids() {
        let mut cal = Calibration::commissioning(crate::model::JOINT_IDS);
        cal.joints.truncate(2);
        assert!(cal.validate().is_err());
    }
    fn calibrated() -> Calibration {
        let mut cal = Calibration::commissioning(crate::model::JOINT_IDS);
        for joint in &mut cal.joints {
            joint.calibrated = true;
            joint.min_rad = Some(-1.5);
            joint.max_rad = Some(1.5);
        }
        cal.imu_mount_verified = true;
        cal.motion_enabled = true;
        cal
    }
    #[test]
    fn telemetry_config_does_not_implicitly_authorize_motion() {
        let mut cal = Calibration::commissioning(crate::model::JOINT_IDS);
        assert!(cal.validate().is_ok());
        assert!(cal.validate_motion().is_err());
        cal.motion_enabled = true;
        assert!(cal.validate().is_err());
        let mut cal = calibrated();
        assert!(cal.validate_motion().is_ok());
        assert!(
            !cal.units_verified,
            "native velocity requires no per-robot conversion setting"
        );
        cal.joints[3].calibrated = false;
        assert!(cal.validate_motion().is_err());
    }
    #[test]
    fn rejects_joint_order_and_limits_before_bus_open() {
        let mut cal = calibrated();
        cal.joints.swap(0, 1);
        assert!(cal.validate().is_err());
        let mut cal = calibrated();
        cal.joints[0].min_rad = Some(-4.0);
        assert!(cal.validate().is_err());
        let mut cal = calibrated();
        cal.joints[2].min_rad = Some(0.0);
        assert!(
            cal.validate().is_err(),
            "official home must fit the measured range"
        );
        let mut cal = calibrated();
        cal.imu_mount_quat = [2.0, 0.0, 0.0, 0.0];
        assert!(cal.validate().is_err());
    }
    /// Every calibration written before a second trunk IMU existed has the bus board's mount
    /// at the top level and nothing else. It must read, validate and write as it always did —
    /// no new table appearing in a file nobody asked to change.
    #[test]
    fn a_bus_only_calibration_reads_validates_and_writes_as_before() {
        let text = include_str!("../../packaging/hd1910-calibration.robot.toml");
        let cal: Calibration = toml::from_str(text).unwrap();
        assert!(cal.validate().is_ok());
        assert_eq!(
            cal.imu_mount(TrunkImuKind::Bus),
            ImuMount {
                quat: cal.imu_mount_quat,
                verified: true
            }
        );
        for kind in [TrunkImuKind::Bno08x, TrunkImuKind::Lsm6dsv16x] {
            assert_eq!(
                cal.imu_mount(kind),
                ImuMount {
                    quat: kind.default_mount(),
                    verified: false
                },
                "{kind:?} was never commissioned"
            );
        }
        let written = toml::to_string_pretty(&cal).unwrap();
        assert!(!written.contains("imu_mounts"), "{written}");
        let reread: Calibration = toml::from_str(&written).unwrap();
        assert_eq!(
            serde_json::to_value(&reread).unwrap(),
            serde_json::to_value(&cal).unwrap()
        );
    }

    /// Module mounts survive the file robotd writes (`toml::to_string_pretty`, as
    /// `save_calibration` does), each in its own table, with the board's left where it was.
    /// The hand-written form parses too, and a source this build has no slot for is refused
    /// rather than dropped.
    #[test]
    fn module_mounts_round_trip_in_their_own_tables() {
        let mut cal = calibrated();
        cal.set_imu_mount(
            TrunkImuKind::Bno08x,
            ImuMount {
                quat: crate::trunk_imu::BNO08X_DEFAULT_MOUNT,
                verified: true,
            },
        );
        cal.set_imu_mount(
            TrunkImuKind::Lsm6dsv16x,
            ImuMount {
                quat: [0.0, 0.0, 0.0, 1.0],
                verified: false,
            },
        );
        let written = toml::to_string_pretty(&cal).unwrap();
        let reread: Calibration = toml::from_str(&written).unwrap();
        for kind in TrunkImuKind::ALL {
            assert_eq!(reread.imu_mount(kind), cal.imu_mount(kind), "{written}");
        }
        assert_eq!(reread.imu_mount_quat, SflpDecoder::DEFAULT_MOUNT);

        let base = toml::to_string_pretty(&calibrated()).unwrap();
        let hand: Calibration = toml::from_str(&format!(
            "{base}
[imu_mounts.bno08x]
quat = [0.5, 0.5, 0.5, 0.5]
verified = true
"
        ))
        .unwrap();
        assert!(hand.imu_mount(TrunkImuKind::Bno08x).verified);
        assert!(
            toml::from_str::<Calibration>(&format!(
                "{base}
[imu_mounts.bmi088]
quat = [1.0, 0.0, 0.0, 0.0]
"
            ))
            .is_err()
        );
    }

    /// Motion on a module needs that module's mount verified: the board's verification says
    /// nothing about a sensor sitting somewhere else. Judged without a source — as when robotd
    /// saves the file — any verified mount will do, which for a bus-only file is the old rule.
    #[test]
    fn motion_is_authorised_per_source() {
        let mut cal = calibrated();
        assert!(cal.validate_motion().is_ok());
        assert!(cal.validate_motion_for(TrunkImuKind::Bus).is_ok());
        for kind in [TrunkImuKind::Bno08x, TrunkImuKind::Lsm6dsv16x] {
            let error = cal.validate_motion_for(kind).unwrap_err().to_string();
            assert!(error.contains(kind.label()), "{error}");
            assert!(!error.contains("  "), "a broken line continuation: {error}");
        }
        cal.imu_mount_verified = false;
        cal.set_imu_mount(
            TrunkImuKind::Bno08x,
            ImuMount {
                quat: crate::trunk_imu::BNO08X_DEFAULT_MOUNT,
                verified: true,
            },
        );
        assert!(
            cal.validate().is_ok(),
            "a verified module mount carries the file"
        );
        assert!(cal.validate_motion_for(TrunkImuKind::Bno08x).is_ok());
        assert!(cal.validate_motion_for(TrunkImuKind::Bus).is_err());
        cal.set_imu_mount(
            TrunkImuKind::Bno08x,
            ImuMount {
                quat: crate::trunk_imu::BNO08X_DEFAULT_MOUNT,
                verified: false,
            },
        );
        assert!(
            cal.validate().is_err(),
            "motion enabled with no verified mount anywhere"
        );
    }

    /// A module's mount is a rotation like the board's; one that is not unit length is
    /// refused, and the error says whose it is.
    #[test]
    fn a_module_mount_must_be_a_rotation() {
        let mut cal = calibrated();
        cal.set_imu_mount(
            TrunkImuKind::Lsm6dsv16x,
            ImuMount {
                quat: [2.0, 0.0, 0.0, 0.0],
                verified: false,
            },
        );
        let error = cal.validate().unwrap_err().to_string();
        assert!(error.contains("lsm6dsv16x"), "{error}");
    }

    #[test]
    fn calibrated_target_roundtrips_and_clamps_without_wrapping() {
        let mut joint = calibrated().joints.remove(0);
        joint.zero_raw = 1900;
        for direction in [-1, 1] {
            joint.direction = direction;
            let (raw, limited) = joint.target_raw(0.6).unwrap();
            assert!(!limited);
            assert!((joint.position_rad(i32::from(raw)) - 0.6).abs() <= TAU / 8192.0);
            let (raw, limited) = joint.target_raw(9.0).unwrap();
            assert!(limited);
            assert!((joint.position_rad(i32::from(raw)) - 1.5).abs() <= TAU / 8192.0);
            assert!(joint.target_raw(f64::NAN).is_err());
        }
    }
}

#[cfg(all(test, unix))]
mod hd1910_transport_tests {
    use super::*;
    use serialport::SerialPort;
    use std::sync::atomic::{AtomicU64, AtomicUsize};
    use std::sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    };

    // Stall the return of an already-complete serial read, as a descheduled host
    // can do. Delaying the device response instead would merely hit the unchanged
    // 20 ms wait limit and would not exercise the complete-but-late branch.
    struct ReadStallPort {
        inner: Box<dyn SerialPort>,
        stall_us: Arc<AtomicU64>,
        received: AtomicUsize,
        reply_bytes: usize,
    }
    impl std::io::Read for ReadStallPort {
        fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
            let n = self.inner.read(out)?;
            let total = self.received.fetch_add(n, Ordering::Relaxed) + n;
            if n > 0 && total >= self.reply_bytes {
                let us = self.stall_us.swap(0, Ordering::Relaxed);
                if us != 0 {
                    std::thread::sleep(Duration::from_micros(us));
                }
            }
            Ok(n)
        }
    }
    impl std::io::Write for ReadStallPort {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            self.inner.write(bytes)
        }
        fn flush(&mut self) -> std::io::Result<()> {
            self.inner.flush()
        }
    }
    macro_rules! forward_port {
        () => {};
        (fn $name:ident(&self $(, $arg:ident: $ty:ty)*) -> $ret:ty; $($rest:tt)*) => {
            fn $name(&self $(, $arg: $ty)*) -> $ret {
                self.inner.$name($($arg),*)
            }
            forward_port!($($rest)*);
        };
        (fn $name:ident(&mut self $(, $arg:ident: $ty:ty)*) -> $ret:ty; $($rest:tt)*) => {
            fn $name(&mut self $(, $arg: $ty)*) -> $ret {
                self.inner.$name($($arg),*)
            }
            forward_port!($($rest)*);
        };
    }
    impl SerialPort for ReadStallPort {
        forward_port! {
            fn name(&self) -> Option<String>;
            fn baud_rate(&self) -> serialport::Result<u32>;
            fn data_bits(&self) -> serialport::Result<serialport::DataBits>;
            fn flow_control(&self) -> serialport::Result<serialport::FlowControl>;
            fn parity(&self) -> serialport::Result<serialport::Parity>;
            fn stop_bits(&self) -> serialport::Result<serialport::StopBits>;
            fn timeout(&self) -> Duration;
            fn set_baud_rate(&mut self, rate: u32) -> serialport::Result<()>;
            fn set_data_bits(&mut self, bits: serialport::DataBits) -> serialport::Result<()>;
            fn set_flow_control(&mut self, flow: serialport::FlowControl) -> serialport::Result<()>;
            fn set_parity(&mut self, parity: serialport::Parity) -> serialport::Result<()>;
            fn set_stop_bits(&mut self, bits: serialport::StopBits) -> serialport::Result<()>;
            fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()>;
            fn write_request_to_send(&mut self, level: bool) -> serialport::Result<()>;
            fn write_data_terminal_ready(&mut self, level: bool) -> serialport::Result<()>;
            fn read_clear_to_send(&mut self) -> serialport::Result<bool>;
            fn read_data_set_ready(&mut self) -> serialport::Result<bool>;
            fn read_ring_indicator(&mut self) -> serialport::Result<bool>;
            fn read_carrier_detect(&mut self) -> serialport::Result<bool>;
            fn bytes_to_read(&self) -> serialport::Result<u32>;
            fn bytes_to_write(&self) -> serialport::Result<u32>;
            fn set_break(&self) -> serialport::Result<()>;
            fn clear_break(&self) -> serialport::Result<()>;
        }
        fn clear(&self, buffer: serialport::ClearBuffer) -> serialport::Result<()> {
            self.received.store(0, Ordering::Relaxed);
            self.inner.clear(buffer)
        }
        fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
            Ok(Box::new(Self {
                inner: self.inner.try_clone()?,
                stall_us: self.stall_us.clone(),
                received: AtomicUsize::new(0),
                reply_bytes: self.reply_bytes,
            }))
        }
    }
    fn stall_reads(bus: Bus, reply_bytes: usize) -> (Bus, Arc<AtomicU64>) {
        let stall_us = Arc::new(AtomicU64::new(0));
        let port = Box::new(ReadStallPort {
            inner: bus.port,
            stall_us: stall_us.clone(),
            received: AtomicUsize::new(0),
            reply_bytes,
        });
        (Bus { port, ..bus }, stall_us)
    }
    #[derive(Clone, Default)]
    struct Faults {
        missing: Vec<u8>,
        checksum_id: Option<u8>,
        status_id: Option<u8>,
        imu_stale: bool,
        identity_missing_reads: usize,
        reject_torque_off_id: Option<u8>,
    }
    type Requests = Arc<Mutex<Vec<(u8, u8, Vec<u8>)>>>;
    struct Emulator {
        faults: Arc<Mutex<Faults>>,
        done: Arc<AtomicBool>,
        regs: Arc<Mutex<Vec<[u8; 96]>>>,
        writes: Arc<Mutex<Vec<(u8, u8, Vec<u8>)>>>,
        frames: Arc<Mutex<Vec<Vec<u8>>>>,
        /// Every request that is not a write: instruction, address, the IDs it addressed.
        requests: Requests,
        reads: Arc<Mutex<Vec<(u8, usize)>>>,
        thread: Option<std::thread::JoinHandle<()>>,
    }
    impl Drop for Emulator {
        fn drop(&mut self) {
            self.done.store(true, Ordering::Relaxed);
            self.thread.take().unwrap().join().unwrap();
        }
    }
    fn emulator(reject_gain: bool) -> (Bus, Emulator) {
        let (mut master, mut slave) = serialport::TTYPort::pair().unwrap();
        master.set_timeout(Duration::from_millis(20)).unwrap();
        slave.set_timeout(Duration::from_millis(2)).unwrap();
        let regs = Arc::new(Mutex::new(vec![[0u8; 96]; 256]));
        for reg in regs.lock().unwrap().iter_mut() {
            reg[21..24].copy_from_slice(&[32, 40, 0]);
            reg[33] = 4;
            reg[41] = 64;
            reg[42..44].copy_from_slice(&2048u16.to_le_bytes());
            reg[44..46].copy_from_slice(&500u16.to_le_bytes());
            reg[46] = 100;
            reg[48..50].copy_from_slice(&1000u16.to_le_bytes());
            reg[50..53].copy_from_slice(&[32, 40, 0]);
            reg[56..58].copy_from_slice(&2048u16.to_le_bytes());
            reg[62] = 74;
            reg[63] = 35;
        }
        let faults = Arc::new(Mutex::new(Faults::default()));
        let fault_source = faults.clone();
        let done = Arc::new(AtomicBool::new(false));
        let writes = Arc::new(Mutex::new(Vec::new()));
        let frames = Arc::new(Mutex::new(Vec::new()));
        let frame_log = frames.clone();
        let requests = Arc::new(Mutex::new(Vec::new()));
        let reads = Arc::new(Mutex::new(Vec::new()));
        let read_log = reads.clone();
        let (d, r, w, q) = (done.clone(), regs.clone(), writes.clone(), requests.clone());
        let thread = std::thread::spawn(move || {
            let mut seq = 0u8;
            while !d.load(Ordering::Relaxed) {
                let mut header = [0u8; 4];
                if master.read_exact(&mut header).is_err() {
                    continue;
                }
                if header == crate::dxl2::HEADER {
                    // A Feetech-only peer ignores the bounded fallback probe.
                    let mut rest = [0u8; 3];
                    master.read_exact(&mut rest).unwrap();
                    let mut dxl_body = vec![0; usize::from(u16::from_le_bytes([rest[1], rest[2]]))];
                    master.read_exact(&mut dxl_body).unwrap();
                    frame_log
                        .lock()
                        .unwrap()
                        .push([&header[..], &rest[..], &dxl_body[..]].concat());
                    continue;
                }
                let mut body = vec![0u8; header[3] as usize];
                master.read_exact(&mut body).unwrap();
                frame_log
                    .lock()
                    .unwrap()
                    .push([&header[..], &body[..]].concat());
                let addr = body[1];
                let len = body[2] as usize;
                if body[0] == SYNC_WRITE {
                    for row in body[3..body.len() - 1].chunks_exact(len + 1) {
                        let id = row[0];
                        w.lock().unwrap().push((id, addr, row[1..].to_vec()));
                        if addr == 40
                            && row[1] == 0
                            && fault_source.lock().unwrap().reject_torque_off_id == Some(id)
                        {
                            continue;
                        }
                        if reject_gain
                            && id == 22
                            && ((addr == 50 && row[1] == 5)
                                || (addr == 48 && len == 4 && row[3] == 3))
                        {
                            continue;
                        }
                        r.lock().unwrap()[id as usize][addr as usize..addr as usize + len]
                            .copy_from_slice(&row[1..]);
                    }
                    continue;
                }
                read_log.lock().unwrap().push((addr, len));
                let ids = if body[0] == SYNC_READ {
                    body[3..body.len() - 1].to_vec()
                } else {
                    vec![header[2]]
                };
                q.lock().unwrap().push((body[0], addr, ids.clone()));
                seq = seq.wrapping_add(1);
                let mut replies = Vec::new();
                if addr == 0 && header[2] == IMU_ID {
                    let mut fault = fault_source.lock().unwrap();
                    if fault.identity_missing_reads > 0 {
                        fault.identity_missing_reads -= 1;
                        continue;
                    }
                }
                let fault = fault_source.lock().unwrap().clone();
                for id in ids {
                    if addr == 56 && fault.missing.contains(&id) {
                        continue;
                    }
                    let mut data = if id == IMU_ID {
                        vec![0; len]
                    } else {
                        r.lock().unwrap()[id as usize][addr as usize..addr as usize + len].to_vec()
                    };
                    if id == IMU_ID && addr == 0 {
                        data.copy_from_slice(&IMU_IDENTITY);
                    }
                    if id == IMU_ID && addr == 56 {
                        data[7] = 0x30;
                        data[12] = seq;
                        if fault.imu_stale {
                            data[13] = 1;
                        }
                    }
                    let status = if addr == 56 && fault.status_id == Some(id) {
                        4
                    } else {
                        0
                    };
                    let mut response = packet(id, status, &data).unwrap();
                    if addr == 56 && fault.checksum_id == Some(id) {
                        *response.last_mut().unwrap() ^= 1;
                    }
                    replies.extend(response);
                }
                master.write_all(&replies).unwrap();
            }
        });
        (
            Bus {
                port: Box::new(slave),
                timeout: Duration::from_millis(30),
                read_only: false,
                last_sync: SyncTrace::default(),
                imu_board: BusImuBoard::Ft6,
                imu_protocol: BusImuProtocol::Auto,
                imu_profile: BusImuProfile::Legacy,
                imu_identity: None,
                observations: observations::Observations::default(),
            },
            Emulator {
                faults,
                done,
                regs,
                writes,
                frames,
                requests,
                reads,
                thread: Some(thread),
            },
        )
    }
    fn calibration() -> Calibration {
        let mut c = Calibration::commissioning(crate::model::JOINT_IDS);
        for j in &mut c.joints {
            j.calibrated = true;
            j.min_rad = Some(-1.5);
            j.max_rad = Some(1.5);
        }
        c.motion_enabled = true;
        c.imu_mount_verified = true;
        c
    }

    /// A module read by the host that always has a fresh, settled sample: a new sequence on
    /// every look, stamped now, in an epoch that began long ago, held upright in its default
    /// mount. `stalled` freezes it: same sequence, 200 ms old.
    struct FreshHost {
        kind: TrunkImuKind,
        sequence: std::sync::atomic::AtomicU32,
        stalled: Arc<AtomicBool>,
        /// A later epoch and the orientation it reports, once the test restarts the stream.
        restarted: Restart,
    }
    type Restart = Arc<Mutex<Option<(u32, [f64; 4])>>>;
    impl FreshHost {
        fn new(kind: TrunkImuKind) -> Self {
            Self {
                kind,
                sequence: std::sync::atomic::AtomicU32::new(0),
                stalled: Arc::new(AtomicBool::new(false)),
                restarted: Arc::new(Mutex::new(None)),
            }
        }
    }
    impl HostImu for FreshHost {
        fn kind(&self) -> TrunkImuKind {
            self.kind
        }
        fn latest(&self) -> Option<crate::trunk_imu::HostImuSample> {
            let now = Instant::now();
            let stalled = self.stalled.load(Ordering::Relaxed);
            let sequence = if stalled {
                self.sequence.load(Ordering::Relaxed)
            } else {
                self.sequence.fetch_add(1, Ordering::Relaxed) + 1
            };
            let (epoch, quat) = self
                .restarted
                .lock()
                .unwrap()
                .unwrap_or((1, self.kind.default_mount()));
            Some(crate::trunk_imu::HostImuSample {
                quat,
                gyro: [0.0; 3],
                received_at: if stalled {
                    now - Duration::from_millis(200)
                } else {
                    now
                },
                sequence,
                epoch,
                epoch_started_at: now - Duration::from_secs(5),
                accuracy: None,
                fusion: None,
            })
        }
        fn health(&self) -> HostImuHealth {
            HostImuHealth {
                state: "streaming",
                epoch: 1,
                samples: u64::from(self.sequence.load(Ordering::Relaxed)),
                ..HostImuHealth::default()
            }
        }
    }
    /// [`calibration`], with `kind`'s own mount commissioned as well as the board's.
    fn calibration_for(kind: TrunkImuKind) -> Calibration {
        let mut c = calibration();
        c.set_imu_mount(
            kind,
            ImuMount {
                quat: kind.default_mount(),
                verified: true,
            },
        );
        c
    }

    #[test]
    fn startup_servo_observations_survive_missing_host_imu_without_control_data() {
        struct AbsentHost;
        impl HostImu for AbsentHost {
            fn kind(&self) -> TrunkImuKind { TrunkImuKind::Lsm6dsv16x }
            fn latest(&self) -> Option<crate::trunk_imu::HostImuSample> { None }
            fn health(&self) -> HostImuHealth { HostImuHealth::default() }
        }
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::for_startup_with_imu(
            bus, calibration_for(TrunkImuKind::Lsm6dsv16x), Box::new(AbsentHost),
        ).unwrap();
        assert!(io.read().unwrap_err().to_string().contains("IMU"));
        let diagnostic = io.feetech_diagnostics().unwrap();
        let wire = serde_json::to_value(&diagnostic).unwrap();
        let observations = &wire["servo_observations"];
        assert_eq!(observations["attempted"], true);
        let rows = observations["rows"].as_array().unwrap();
        assert_eq!(rows.len(), NUM_JOINTS);
        for (row, joint) in rows.iter().zip(&io.calibration.joints) {
            assert_eq!(row["id"], joint.id);
            assert_eq!(row["name"], joint.name);
            assert_eq!(row["responding"], true);
            assert!(row["check_age_ms"].is_u64());
            assert!(row["reply_age_ms"].is_u64());
            assert!(row["feedback_age_ms"].is_u64());
            assert_eq!(row["position_raw"], 2048);
            assert_eq!(row["volts"], 7.4);
        }
        let fixture: serde_json::Value = serde_json::from_str(include_str!("../tests/fixtures/startup-servo-observations.json")).unwrap();
        let mut normalized = observations.clone();
        normalized["age_ms"] = 0.into();
        for row in normalized["rows"].as_array_mut().unwrap() {
            for key in ["check_age_ms", "reply_age_ms", "feedback_age_ms"] { row[key] = 0.into(); }
        }
        assert_eq!(normalized, fixture["native"]["servo_observations"]);
        assert!(!diagnostic.all_joints_fresh);
        assert!(diagnostic.joints.is_empty());
        assert!(io.latest_sample_at.is_none());
        assert!(io.joint_feedback.iter().all(Option::is_none));
        assert!(io.set_torque(true).is_err());
        assert!(device.writes.lock().unwrap().is_empty());
    }

    #[test]
    fn startup_servo_observations_preserve_partial_replies_and_reject_bad_checksums() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::for_startup(bus, calibration()).unwrap();
        device.faults.lock().unwrap().missing = vec![IMU_ID, 22];
        device.faults.lock().unwrap().checksum_id = Some(32);
        assert!(io.read().is_err());
        let wire = serde_json::to_value(io.feetech_diagnostics().unwrap()).unwrap();
        assert_eq!(wire["servo_observations"]["attempted"], true);
        let rows = wire["servo_observations"]["rows"].as_array().unwrap();
        for row in rows {
            let missing = [22, 32].contains(&(row["id"].as_u64().unwrap() as u8));
            assert_eq!(row["responding"], !missing);
            assert_eq!(row["reply_age_ms"].is_null(), missing);
            assert_eq!(row["position_raw"].is_null(), missing);
        }
        assert!(io.latest_sample_at.is_none());
        assert!(io.joint_feedback.iter().all(Option::is_none));
        assert!(device.writes.lock().unwrap().is_empty());
    }

    #[test]
    fn startup_servo_observations_torque_reply_proves_presence_without_feedback() {
        let (bus, _device) = emulator(false);
        let mut io = FeetechIo::for_startup(bus, calibration()).unwrap();
        let before = serde_json::to_value(io.feetech_diagnostics().unwrap()).unwrap();
        assert_eq!(before["servo_observations"]["attempted"], false);
        assert!(before["servo_observations"]["rows"][0]["responding"].is_null());
        io.bus.sync_read(40, 1, &io.ids()).unwrap();
        let wire = serde_json::to_value(io.feetech_diagnostics().unwrap()).unwrap();
        for row in wire["servo_observations"]["rows"].as_array().unwrap() {
            assert_eq!(row["responding"], true);
            assert!(row["reply_age_ms"].is_u64());
            for key in ["feedback_age_ms", "position_raw", "speed_raw", "load_raw", "current_raw", "volts", "temperature_c"] {
                assert!(row[key].is_null(), "{key} must not be fabricated from torque");
            }
        }
        assert_eq!(io.torque_state_confirmed, None);
        assert!(!io.feetech_control_status().unwrap().all_joints_fresh);
    }

    #[test]
    fn startup_servo_observations_latest_miss_keeps_history_and_unrelated_reads_do_not_refresh_it() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::for_startup(bus, calibration()).unwrap();
        io.read().unwrap();
        let control_at = io.latest_sample_at;
        device.faults.lock().unwrap().missing = vec![IMU_ID, 22];
        assert!(io.read().is_err());
        let snapshot = io.feetech_snapshot().unwrap();
        let observations = &snapshot.dynamic.servo_observations;
        let before = serde_json::to_value(observations).unwrap();
        let row = &before["rows"][index(22)];
        assert_eq!(row["responding"], false);
        assert!(row["check_age_ms"].is_u64());
        assert!(row["reply_age_ms"].is_u64());
        assert_eq!(row["position_raw"], 2048);
        assert_eq!(io.latest_sample_at, control_at);
        // A later read of another servo cannot make ID 22's checked/reply/feedback times new.
        std::thread::sleep(Duration::from_millis(30));
        io.bus.read(10, 40, 1).unwrap();
        let after = serde_json::to_value(io.feetech_snapshot().unwrap().dynamic.servo_observations).unwrap();
        let later = &after["rows"][index(22)];
        for key in ["check_age_ms", "reply_age_ms", "feedback_age_ms"] {
            assert!(later[key].as_u64().unwrap() >= row[key].as_u64().unwrap() + 20, "{key}");
        }
        assert_eq!(later["responding"], false);
        let aged = serde_json::to_value(observations).unwrap();
        assert!(aged["age_ms"].as_u64().unwrap() >= before["age_ms"].as_u64().unwrap() + 20);
        assert_eq!(aged["rows"], before["rows"]);
    }

    #[test]
    fn startup_servo_observations_survive_invalid_bus_imu_without_advancing_atomic_samples() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::for_startup(bus, calibration()).unwrap();
        device.faults.lock().unwrap().imu_stale = true;
        assert!(io.read().is_err());
        let wire = serde_json::to_value(io.feetech_diagnostics().unwrap()).unwrap();
        assert!(wire["servo_observations"]["rows"].as_array().unwrap().iter().all(|row| row["responding"] == true && row["position_raw"] == 2048));
        assert!(io.latest_sample_at.is_none());
        assert!(io.joint_feedback.iter().all(Option::is_none));
        assert!(!io.feetech_control_status().unwrap().all_joints_fresh);
        assert!(!io.imu_ready());
        assert!(device.writes.lock().unwrap().is_empty());
    }

    /// A module source puts nothing on the bus but the servos. Every request is a Sync Read of
    /// exactly the fifteen joints — telemetry, the torque checks, the HD1910 register
    /// inspection — and ID 200 is never addressed, not even to identify it, so a robot without
    /// the board works. The module's samples go through the same frame and ready gate as the
    /// board's, and motion comes up on it like on the board.
    #[test]
    fn a_host_imu_puts_only_the_fifteen_servos_on_the_bus() {
        for kind in [TrunkImuKind::Bno08x, TrunkImuKind::Lsm6dsv16x] {
            let (bus, device) = emulator(false);
            let mut io = FeetechIo::from_bus_with_imu(
                bus,
                calibration_for(kind),
                Box::new(FreshHost::new(kind)),
            )
            .unwrap();
            for _ in 0..24 {
                io.read().unwrap();
            }
            assert!(!io.imu_ready(), "24 samples is not converged");
            let sensors = io.read().unwrap();
            assert!(io.imu_ready());
            let g = sensors.imu.gravity;
            assert!(
                g[0].abs() < 1e-12 && g[1].abs() < 1e-12 && (g[2] + 1.0).abs() < 1e-12,
                "upright in its own mount reads upright: {g:?}"
            );
            io.set_torque(true).unwrap();
            io.write(&JointTargets::new(crate::model::DEFAULT_POSITION))
                .unwrap();
            io.read().unwrap();
            io.set_torque(false).unwrap();

            let requests = device.requests.lock().unwrap().clone();
            assert!(
                requests
                    .iter()
                    .any(|(_, address, _)| *address == SYNC_ADDRESS)
            );
            for (instruction, address, ids) in &requests {
                assert_eq!(*instruction, SYNC_READ, "{kind:?}: request at {address}");
                assert_eq!(
                    ids,
                    &crate::model::JOINT_IDS.to_vec(),
                    "{kind:?}: {address}"
                );
            }
            assert!(
                device
                    .writes
                    .lock()
                    .unwrap()
                    .iter()
                    .all(|(id, _, _)| *id != IMU_ID)
            );

            let diagnostics = io.feetech_diagnostics().unwrap();
            assert_eq!(diagnostics.imu_source, kind.label());
            assert!(diagnostics.imu_mount_verified);
            assert_eq!(diagnostics.imu_status, None);
            assert_eq!(diagnostics.telemetry.failed_reads, 0);
            assert_eq!(diagnostics.partial_samples_total, 0);
            let wire = serde_json::to_value(&diagnostics).unwrap();
            assert_eq!(wire["imu_source"], kind.label());
            assert_eq!(wire["imu_reader"]["state"], "streaming");
            let snapshot = io.feetech_snapshot().unwrap();
            assert_eq!(snapshot.metadata.imu_source, kind.label());
            assert_eq!(
                snapshot.dynamic.imu_reader.unwrap().state,
                "streaming",
                "the reporter sees the reader too"
            );
        }
    }

    /// A BNO08x sample's gyro-fusion view reaches the snapshot in the trunk's axes, under the
    /// mount the tick's own orientation went through — the other orientation in each mode. The
    /// bus board has none, and busStatus does not change shape for it.
    #[test]
    fn the_fusion_view_reaches_the_snapshot_in_trunk_axes() {
        use crate::gyro_fusion::{FusionCounts, FusionMode, FusionSample, Verdict};
        struct Fused {
            sequence: std::sync::atomic::AtomicU32,
            mode: FusionMode,
        }
        const CHIP: [f64; 4] = [0.5, 0.5, 0.5, 0.5];
        const FUSED: [f64; 4] = [0.6, 0.4, 0.5, 0.48];
        impl HostImu for Fused {
            fn kind(&self) -> TrunkImuKind {
                TrunkImuKind::Bno08x
            }
            fn latest(&self) -> Option<crate::trunk_imu::HostImuSample> {
                let now = Instant::now();
                Some(crate::trunk_imu::HostImuSample {
                    quat: match self.mode {
                        FusionMode::Chip => CHIP,
                        FusionMode::Gyro => FUSED,
                    },
                    gyro: [0.0; 3],
                    received_at: now,
                    sequence: self.sequence.fetch_add(1, Ordering::Relaxed) + 1,
                    epoch: 1,
                    epoch_started_at: now - Duration::from_secs(5),
                    accuracy: Some(3),
                    fusion: Some(FusionSample {
                        mode: self.mode,
                        verdict: Verdict::Rejected,
                        chip_quat: CHIP,
                        fused_quat: FUSED,
                        theta_deg: 7.5,
                        counts: FusionCounts::default(),
                        period_us: None,
                    }),
                })
            }
            fn health(&self) -> HostImuHealth {
                HostImuHealth {
                    state: "streaming",
                    ..HostImuHealth::default()
                }
            }
        }
        let mount = [0.459554518, 0.539556409, 0.532044137, 0.463268349];
        for mode in FusionMode::ALL {
            let (bus, _device) = emulator(false);
            let mut c = calibration();
            c.set_imu_mount(
                TrunkImuKind::Bno08x,
                ImuMount {
                    quat: mount,
                    verified: true,
                },
            );
            let host = Fused {
                sequence: std::sync::atomic::AtomicU32::new(0),
                mode,
            };
            let mut io = FeetechIo::from_bus_with_imu(bus, c, Box::new(host)).unwrap();
            for _ in 0..3 {
                io.read().unwrap();
            }
            let tick = io.feetech_snapshot().unwrap().imu_fusion.unwrap();
            let frame = TrunkFrame::new(mount);
            let other = match mode {
                FusionMode::Chip => FUSED,
                FusionMode::Gyro => CHIP,
            };
            assert_eq!(tick.other_quat, frame.trunk_quat(other).unwrap());
            assert_eq!(
                (tick.mode, tick.verdict, tick.theta_deg),
                (mode, Verdict::Rejected, 7.5)
            );
            let wire = serde_json::to_value(io.feetech_diagnostics().unwrap()).unwrap();
            assert!(wire.get("imu_fusion").is_none());
        }
        let (bus, _device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        io.read().unwrap();
        assert!(io.feetech_snapshot().unwrap().imu_fusion.is_none());
    }

    /// With the board, a missing servo still leaves a frame the control loop could coast on
    /// (`telemetry_partial`) — in practice it waits out the 20 ms and fails the board's age.
    /// With a module there is no board reply to accept a short frame on: the whole sample
    /// fails, the missing ID is counted against that servo, nothing is retimestamped, and
    /// torque cannot come on over it.
    #[test]
    fn a_host_imu_sample_fails_whole_when_one_servo_is_missing() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus_with_imu(
            bus,
            calibration_for(TrunkImuKind::Bno08x),
            Box::new(FreshHost::new(TrunkImuKind::Bno08x)),
        )
        .unwrap();
        for _ in 0..30 {
            io.read().unwrap();
        }
        device.faults.lock().unwrap().missing = vec![22];
        let before = io.latest_sample_at;
        let error = io.read().unwrap_err().to_string();
        assert!(error.contains("missing IDs [22]"), "{error}");
        assert_eq!(io.latest_sample_at, before);
        let diagnostics = io.feetech_diagnostics().unwrap();
        assert_eq!(diagnostics.sync.missing_ids, vec![22]);
        assert_eq!(diagnostics.telemetry.missing_by_id[&22], 1);
        assert_eq!(diagnostics.partial_samples_total, 0);
        assert!(io.set_torque(true).is_err());
        device.faults.lock().unwrap().missing.clear();
        io.read().unwrap();
        assert!(io.latest_sample_at > before);
    }

    /// A reader that stops refreshing is refused with the board's own words, and the tick's
    /// data is not retimestamped. Motion on a module needs that module's mount: the board's
    /// verified mount, which this calibration has, does not stand in for it.
    #[test]
    fn a_stalled_host_imu_is_refused_and_motion_needs_its_own_mount() {
        let (bus, _device) = emulator(false);
        let host = FreshHost::new(TrunkImuKind::Bno08x);
        let stalled = host.stalled.clone();
        let mut io = FeetechIo::from_bus_with_imu(bus, calibration(), Box::new(host)).unwrap();
        for _ in 0..30 {
            io.read().unwrap();
        }
        let error = io.set_torque(true).unwrap_err().to_string();
        assert!(error.contains("bno08x"), "{error}");
        assert!(!io.feetech_diagnostics().unwrap().imu_mount_verified);

        stalled.store(true, Ordering::Relaxed);
        let before = io.latest_sample_at;
        let error = io.read().unwrap_err().to_string();
        assert!(error.contains("Feetech IMU not ready or stale"), "{error}");
        assert_eq!(io.latest_sample_at, before);
        assert!(io.imu_ready(), "a stale tick is not a convergence reset");
    }

    /// A module stream that started over (a reopen, a chip reset) is a new epoch, and the
    /// first sample of it is reported as itself: the median does not carry the orientation from
    /// before the outage across it. The bus board's decoder never restarts.
    #[test]
    fn a_new_host_epoch_restarts_the_median_but_not_readiness() {
        let (bus, _device) = emulator(false);
        let host = FreshHost::new(TrunkImuKind::Bno08x);
        let restarted = Arc::clone(&host.restarted);
        let mut io = FeetechIo::from_bus_with_imu(
            bus,
            calibration_for(TrunkImuKind::Bno08x),
            Box::new(host),
        )
        .unwrap();
        for _ in 0..30 {
            io.read().unwrap();
        }
        // Pitched 30° nose-down: sensor = trunk · mount for the BNO08x default mount.
        let (c, v) = (15f64.to_radians().cos(), 15f64.to_radians().sin());
        let pitched = [0.5 * (c - v), 0.5 * (c + v), 0.5 * (c + v), 0.5 * (c - v)];
        *restarted.lock().unwrap() = Some((2, pitched));
        let gravity = io.read().unwrap().imu.gravity;
        assert!(
            (gravity[0] - 30f64.to_radians().sin()).abs() < 1e-9,
            "the old epoch leaked through the median: {gravity:?}"
        );
        assert!(
            io.imu_ready(),
            "a restarted stream is not an unconverged chip"
        );
    }

    /// The board is read in the servos' Sync Read; handing it in as a host reader would skip
    /// its identity check and read nothing.
    #[test]
    fn the_bus_board_cannot_be_a_host_reader() {
        let (bus, _device) = emulator(false);
        let error = FeetechIo::from_bus_with_imu(
            bus,
            calibration(),
            Box::new(FreshHost::new(TrunkImuKind::Bus)),
        )
        .err()
        .unwrap()
        .to_string();
        assert!(error.contains("host reader"), "{error}");
    }

    #[test]
    fn startup_identity_timeout_keeps_servos_limp_and_goal_channel_closed() {
        // A brownout can leave powered servos alive while ID 200 restarts. The
        // identity timeout must neither preserve that torque nor preload a goal.
        // robotd handles the failed owner by dropping it before a fresh open.
        let (bus, device) = emulator(false);
        for id in crate::model::JOINT_IDS {
            device.regs.lock().unwrap()[id as usize][40] = 1;
        }
        device.faults.lock().unwrap().identity_missing_reads = 1;
        let mut io = FeetechIo::for_startup(bus, calibration()).unwrap();
        assert!(device.writes.lock().unwrap().is_empty());
        let problem = io.confirm_startup_limp().unwrap_err().to_string();
        assert!(problem.contains("ID 200 response timeout"), "{problem}");
        assert_eq!(io.torque_state_confirmed, Some(false));
        io.write(&JointTargets::new([0.0; NUM_JOINTS])).unwrap();
        assert!(!io.command_enabled);
        assert!(
            device
                .writes
                .lock()
                .unwrap()
                .iter()
                .all(|(_, addr, data)| *addr == 40 && data == &[0])
        );
        for id in crate::model::JOINT_IDS {
            assert_eq!(device.regs.lock().unwrap()[id as usize][40], 0);
        }
    }

    #[test]
    fn startup_unconfirmed_torque_never_proceeds_to_imu_or_goals() {
        let (bus, device) = emulator(false);
        device.regs.lock().unwrap()[22][40] = 1;
        let mut faults = device.faults.lock().unwrap();
        faults.reject_torque_off_id = Some(22);
        faults.identity_missing_reads = 1;
        drop(faults);
        let mut io = FeetechIo::for_startup(bus, calibration()).unwrap();
        let problem = io.confirm_startup_limp().unwrap_err().to_string();
        assert!(
            problem.contains("ID 22 torque state false not confirmed"),
            "{problem}"
        );
        assert_eq!(io.torque_state_confirmed, None);
        assert_eq!(device.faults.lock().unwrap().identity_missing_reads, 1);
        assert!(!io.command_enabled);
        io.write(&JointTargets::new([0.0; NUM_JOINTS])).unwrap();
        assert!(
            device
                .writes
                .lock()
                .unwrap()
                .iter()
                .all(|(_, addr, _)| *addr == 40)
        );
    }

    #[test]
    fn control_status_matches_full_diagnostics_through_feedback_and_torque_changes() {
        // The allocation-free HOME flags must retain the old guard's verdicts,
        // including no sample, unconfirmed torque and wall-clock feedback holds.
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        let check = |io: &FeetechIo| {
            let full = io.feetech_diagnostics().unwrap();
            let status = io.feetech_control_status().unwrap();
            assert_eq!(status, FeetechControlStatus::from(&full));
            status
        };
        assert!(!check(&io).all_joints_fresh);
        for _ in 0..30 {
            io.read().unwrap();
        }
        let ready = check(&io);
        assert!(ready.all_joints_fresh && ready.imu_ready && !ready.joints_held);
        assert_eq!(ready.torque_state_confirmed, Some(false));
        io.set_torque(true).unwrap();
        assert_eq!(check(&io).torque_state_confirmed, Some(true));
        let i = index(22);
        for missing in 1..=4 {
            io.joint_missing[i] = missing;
            let status = check(&io);
            assert!(!status.all_joints_fresh);
            assert_eq!(status.joints_held, missing > JOINT_COAST_TICKS);
        }
        io.read().unwrap();
        assert!(check(&io).all_joints_fresh);
        io.joint_feedback[i].as_mut().unwrap().at = Instant::now() - Duration::from_millis(200);
        assert!(check(&io).joints_held);
        io.torque_state_confirmed = None;
        assert_eq!(check(&io).torque_state_confirmed, None);
        io.read().unwrap();
        io.set_torque(false).unwrap();
        assert_eq!(check(&io).torque_state_confirmed, Some(false));
        let writes = device.writes.lock().unwrap().len();
        let reads = io.telemetry.total_reads;
        let io = crate::hardware::HardwareIo::Feetech(io);
        let safety = crate::safety::Safety::new(io, Default::default());
        assert_eq!(
            safety.feetech_control_status().unwrap(),
            FeetechControlStatus::from(&safety.feetech_diagnostics().unwrap())
        );
        assert_eq!(device.writes.lock().unwrap().len(), writes);
        assert_eq!(
            safety.feetech_diagnostics().unwrap().telemetry.total_reads,
            reads
        );
    }

    #[test]
    fn diagnostic_registers_share_storage_and_refresh_only_after_a_new_inspection() {
        // Sharing must preserve the before-arm snapshot held by an older frame,
        // while a later inspection and dynamic torque flags reach new frames.
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        let before = io.feetech_diagnostics().unwrap();
        io.torque_state_confirmed = None;
        let next = io.feetech_diagnostics().unwrap();
        assert!(Arc::ptr_eq(
            &before.hd1910.registers_before_arm,
            &next.hd1910.registers_before_arm
        ));
        assert_eq!(before.torque_state_confirmed, Some(false));
        assert_eq!(next.torque_state_confirmed, None);
        let wire = serde_json::to_value(&next).unwrap();
        assert_eq!(
            wire["hd1910"]["registers_before_arm"],
            serde_json::to_value(before.hd1910.registers_before_arm.as_ref()).unwrap()
        );
        device.regs.lock().unwrap()[22][50..53].copy_from_slice(&[7, 8, 9]);
        io.hd1910.inspect(&mut io.bus, &io.calibration).unwrap();
        let after = io.feetech_diagnostics().unwrap();
        assert!(!Arc::ptr_eq(
            &before.hd1910.registers_before_arm,
            &after.hd1910.registers_before_arm
        ));
        assert_eq!(
            before.hd1910.registers_before_arm[index(22)].temporary_pdi,
            [32, 40, 0]
        );
        assert_eq!(
            after.hd1910.registers_before_arm[index(22)].temporary_pdi,
            [7, 8, 9]
        );
        assert!(device.writes.lock().unwrap().is_empty());
    }

    #[test]
    fn numeric_snapshot_matches_native_report_without_uart_or_lost_offline_counts() {
        // The new reporting hand-off must preserve every old diagnostic member,
        // including held joints and post-command torque, without touching the bus.
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        let compare = |io: &FeetechIo| {
            let old = io.feetech_diagnostics().unwrap();
            let snapshot = io.feetech_snapshot().unwrap();
            let mut expected = serde_json::to_value(&old).unwrap();
            let mut actual = serde_json::to_value(snapshot.metadata.as_ref()).unwrap();
            actual.as_object_mut().unwrap().extend(
                serde_json::to_value(&snapshot.dynamic)
                    .unwrap()
                    .as_object()
                    .unwrap()
                    .clone(),
            );
            actual["hd1910"] = serde_json::to_value(&snapshot.hd1910).unwrap();
            // Both APIs timestamp at their own call. Compare the measured ages
            // separately rather than making a scheduler-dependent equality test.
            for key in ["sample_age_ms"] {
                if let (Some(a), Some(b)) = (actual[key].as_u64(), expected[key].as_u64()) {
                    assert!(a >= b && a - b < 100);
                }
                actual.as_object_mut().unwrap().remove(key);
                expected.as_object_mut().unwrap().remove(key);
            }
            for (a, b) in actual["joints"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .zip(expected["joints"].as_array_mut().unwrap())
            {
                assert!(
                    a["sample_age_ms"].as_u64().unwrap() >= b["sample_age_ms"].as_u64().unwrap()
                );
                a.as_object_mut().unwrap().remove("sample_age_ms");
                b.as_object_mut().unwrap().remove("sample_age_ms");
            }
            assert_eq!(actual, expected);
            snapshot
        };
        let before = compare(&io);
        // No reporter exists while acquisition continues: history must still grow.
        for _ in 0..3 {
            io.read().unwrap();
        }
        let ready = compare(&io);
        assert_eq!(
            ready.dynamic.telemetry.total_reads,
            before.dynamic.telemetry.total_reads + 3
        );
        assert!(Arc::ptr_eq(&before.metadata, &ready.metadata));
        let i = index(22);
        io.joint_missing[i] = 4;
        io.joint_missing_total[i] = 7;
        io.joint_feedback[i].as_mut().unwrap().at = Instant::now() - Duration::from_secs(1);
        assert!(!compare(&io).dynamic.all_joints_fresh);
        io.torque_state_confirmed = None;
        assert_eq!(compare(&io).dynamic.torque_state_confirmed, None);
        io.torque_state_confirmed = Some(false);
        assert_eq!(compare(&io).dynamic.torque_state_confirmed, Some(false));
        let writes = device.writes.lock().unwrap().len();
        let reads = io.telemetry.total_reads;
        let safety = crate::safety::Safety::new(
            crate::hardware::HardwareIo::Feetech(io),
            Default::default(),
        );
        assert_eq!(
            safety
                .feetech_snapshot()
                .unwrap()
                .dynamic
                .telemetry
                .total_reads,
            reads
        );
        assert_eq!(safety.feetech_sync_trace().unwrap().address, 56);
        assert_eq!(device.writes.lock().unwrap().len(), writes);
    }

    #[test]
    fn both_hip_pitch_signed_replies_reach_sensors_and_diagnostics() {
        let (bus, device) = emulator(false);
        let c = calibration();
        for id in [12, 22] {
            device.regs.lock().unwrap()[id][56..58].copy_from_slice(&34180u16.to_le_bytes());
        }
        let mut io = FeetechIo::from_bus(bus, c.clone()).unwrap();
        for _ in 0..3 {
            let sensors = io.read().unwrap();
            let diag = io.feetech_diagnostics().unwrap();
            for id in [12, 22] {
                let i = c.joints.iter().position(|j| j.id == id).unwrap();
                assert_eq!(sensors.positions[i], c.joints[i].position_rad(-1412));
                let row = diag.joints.iter().find(|j| j.id == id).unwrap();
                assert_eq!(
                    (
                        row.position_wire_raw,
                        row.position_signed_raw,
                        row.position_raw
                    ),
                    (34180, -1412, -1412)
                );
                assert!(row.feedback_fresh);
            }
        }
        device.regs.lock().unwrap()[22][56..58].copy_from_slice(&0xffffu16.to_le_bytes());
        let sensors = io.read().unwrap();
        assert_eq!(
            sensors.positions[index(22)],
            c.joints[index(22)].position_rad(-32767)
        );
        let row = io
            .feetech_diagnostics()
            .unwrap()
            .joints
            .into_iter()
            .find(|j| j.id == 22)
            .unwrap();
        assert_eq!(
            (
                row.position_wire_raw,
                row.position_signed_raw,
                row.position_raw
            ),
            (65535, -32767, -32767)
        );
    }

    #[test]
    fn negative_feedback_preloads_itself_but_home_targets_stay_calibrated_positive() {
        let (bus, device) = emulator(false);
        let mut c = calibration();
        c.joints[index(22)].zero_raw = 1977;
        c.joints[index(22)].direction = -1;
        c.joints[index(12)].zero_raw = 1799;
        c.joints[index(12)].direction = -1;
        for (id, position) in [(22, -1670), (12, -1), (30, 6244), (31, -6048)] {
            device.regs.lock().unwrap()[id][56..58]
                .copy_from_slice(&signed_position_word(position).unwrap().to_le_bytes());
        }
        let mut io = FeetechIo::from_bus(bus, c.clone()).unwrap();
        let mut measured = Sensors::default();
        for _ in 0..30 {
            measured = io.read().unwrap();
        }
        device.writes.lock().unwrap().clear();
        io.set_torque(true).unwrap();
        let ids = crate::model::JOINT_IDS;
        let goals = io.bus.sync_read(42, 6, &ids).unwrap();
        for (id, position) in [(22, -1670), (12, -1), (30, 6244), (31, -6048)] {
            let goal = &goals[&(id as u8)];
            assert_eq!(
                signed_magnitude(u16::from_le_bytes([goal[0], goal[1]]), 15),
                position
            );
            assert_eq!(goal[2..], [0, 0, 0, 0]);
        }
        {
            let writes = device.writes.lock().unwrap();
            let first_on = writes
                .iter()
                .position(|(_, address, data)| *address == 40 && data[0] == 1)
                .unwrap();
            for id in ids {
                assert!(
                    writes[..first_on]
                        .iter()
                        .any(|(target, address, _)| *target == id && *address == 42)
                );
            }
        }
        // Exercise the HOME-shaped path through the real writer. Mechanical limits
        // still apply, but negative feedback must never select a different target turn.
        let start = measured.positions;
        for step in 0..=50 {
            let target = std::array::from_fn(|i| {
                start[i] + (crate::model::DEFAULT_POSITION[i] - start[i]) * f64::from(step) / 50.0
            });
            io.write(&JointTargets::new(target)).unwrap();
            let goals = io.bus.sync_read(42, 6, &ids).unwrap();
            for (i, id) in ids.into_iter().enumerate() {
                let expected = c.joints[i].target_raw(target[i]).unwrap().0;
                assert_eq!(goals[&id][..2], expected.to_le_bytes());
                assert_eq!(goals[&id][2..], [0, 0, 0, 0]);
            }
        }
        let goals = io.bus.sync_read(42, 2, &ids).unwrap();
        assert_eq!(goals[&22], 2276u16.to_le_bytes());
        assert_eq!(goals[&12], 1500u16.to_le_bytes());
        assert!(
            device
                .writes
                .lock()
                .unwrap()
                .iter()
                .all(|(_, address, _)| [40, 42, 48, 50, 51].contains(address))
        );
        // A different raw revolution on re-arm or service restart never changes HOME.
        io.set_torque(false).unwrap();
        device.regs.lock().unwrap()[22][56..58]
            .copy_from_slice(&signed_position_word(-1820).unwrap().to_le_bytes());
        io.read().unwrap();
        io.set_torque(true).unwrap();
        io.write(&JointTargets::new(crate::model::DEFAULT_POSITION))
            .unwrap();
        assert_eq!(
            io.bus.sync_read(42, 2, &ids).unwrap()[&22],
            2276u16.to_le_bytes()
        );
        let mut restarted = FeetechIo::from_bus(io.bus, c).unwrap();
        for _ in 0..30 {
            measured = restarted.read().unwrap();
        }
        device.writes.lock().unwrap().clear();
        restarted
            .write(&JointTargets::new(measured.positions))
            .unwrap();
        assert!(device.writes.lock().unwrap().is_empty());
        restarted.set_torque(true).unwrap();
        restarted
            .write(&JointTargets::new(crate::model::DEFAULT_POSITION))
            .unwrap();
        let goals = restarted.bus.sync_read(42, 2, &ids).unwrap();
        assert_eq!(goals[&22], 2276u16.to_le_bytes());
        assert_eq!(goals[&12], 1500u16.to_le_bytes());
    }

    #[test]
    fn telemetry_between_old_and_new_deadline_is_fresh_without_late_exception() {
        let (bus, device) = emulator(false);
        let (bus, stall) = stall_reads(bus, 336);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        let before = io.read().unwrap();
        set_positions(&device, 2100);
        stall.store(16_000, Ordering::Relaxed);
        let after = io.read().unwrap();
        let diag = io.feetech_diagnostics().unwrap();
        assert_ne!(before.positions, after.positions);
        assert!(!diag.sync.complete_but_late && !diag.sync.deadline_exceeded);
        assert!((15_000..20_000).contains(&diag.sync.elapsed_us));
        assert!(diag.sync.missing_ids.is_empty());
        assert_eq!(diag.telemetry.complete_but_late, 0);
        assert_eq!(diag.telemetry.failed_reads, 0);
        assert!(diag.telemetry.last_error.is_none());
        assert!(diag.telemetry.last_failure.is_none());
    }

    #[test]
    fn complete_late_telemetry_still_rejects_expired_data_and_bad_imu() {
        let (bus, device) = emulator(false);
        let (bus, stall) = stall_reads(bus, 336);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        io.read().unwrap();
        let before = io.latest_sample_at;
        stall.store(22_000, Ordering::Relaxed);
        assert!(
            io.read()
                .unwrap_err()
                .to_string()
                .contains("complete but transaction deadline exceeded")
        );
        assert_eq!(io.latest_sample_at, before);
        let diag = io.feetech_diagnostics().unwrap();
        assert_eq!(diag.telemetry.failed_reads, 1);
        assert_eq!(diag.telemetry.complete_but_late, 1);
        assert_eq!(diag.telemetry.accepted_complete_but_late, 0);
        assert!(!diag.telemetry.last_complete_but_late.unwrap().accepted);
        assert!(diag.telemetry.last_failure.unwrap().elapsed_us >= 20_000);
        assert!(diag.telemetry.missing_by_id.is_empty());
        device.faults.lock().unwrap().imu_stale = true;
        stall.store(15_200, Ordering::Relaxed);
        assert!(
            io.read()
                .unwrap_err()
                .to_string()
                .contains("IMU not ready or stale")
        );
        assert_eq!(io.latest_sample_at, before);
    }

    #[test]
    fn complete_late_torque_confirmation_keeps_its_original_deadline() {
        let (bus, _device) = emulator(false);
        let (mut bus, stall) = stall_reads(bus, 105);
        stall.store(15_200, Ordering::Relaxed);
        assert!(
            bus.verify_torque(&crate::model::JOINT_IDS, false)
                .unwrap_err()
                .to_string()
                .contains("complete but transaction deadline exceeded")
        );
        assert!(bus.last_sync.complete_but_late);
    }
    #[test]
    fn hd1910_model_p_arm_writes_d0_limit1000_preserving_acceleration_and_eeprom() {
        let (mut bus, device) = emulator(false);
        // Same writer used by commissioning before switching to the motion loop.
        // Every servo retains its own existing acceleration, not a fixed test value.
        for id in crate::model::JOINT_IDS {
            let mut registers = device.regs.lock().unwrap();
            registers[id as usize][41] = id * 3;
            registers[id as usize][51] = id * 2;
            registers[id as usize][52] = id;
            registers[id as usize][48..50].copy_from_slice(&(800u16 + id as u16).to_le_bytes());
            drop(registers);
            bus.positions(&[(id, 2048)]).unwrap();
        }
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        assert_eq!(io.hd1910.diagnostics().registers_before_arm.len(), 15);
        for _ in 0..30 {
            io.read().unwrap();
        }
        device.reads.lock().unwrap().clear();
        io.set_torque(true).unwrap();
        assert_eq!(
            *device.reads.lock().unwrap(),
            vec![
                (40, 1),
                (21, 3),
                (33, 1),
                (50, 3),
                (41, 7),
                (50, 1),
                (56, 15),
                (42, 2),
                (40, 1),
                (56, 15)
            ],
            "arming must retain V164 reads without a new runtime readback"
        );
        io.write(&JointTargets::new([0.1; NUM_JOINTS])).unwrap();
        for id in crate::model::JOINT_IDS {
            let reg = device.regs.lock().unwrap()[id as usize];
            assert_eq!(reg[50], crate::deployment::firmware_p());
            assert_eq!(reg[51], 0);
            assert_eq!(reg[52], id);
            assert_eq!(&reg[48..50], &1000u16.to_le_bytes());
        }
        io.set_torque(false).unwrap();
        for id in crate::model::JOINT_IDS {
            let reg = device.regs.lock().unwrap()[id as usize];
            assert_eq!(reg[40], 0);
            assert_eq!(reg[41], id * 3);
            assert_eq!(&reg[21..24], &[32, 40, 0]);
            assert_eq!(&reg[50..53], &[32, 0, id]);
            assert_eq!(&reg[48..50], &1000u16.to_le_bytes());
            assert_eq!(&reg[46..48], &[0, 0]);
        }
        let writes = device.writes.lock().unwrap();
        let first_on = writes
            .iter()
            .position(|(_, a, v)| *a == 40 && v[0] == 1)
            .unwrap();
        for id in crate::model::JOINT_IDS {
            assert!(
                writes[..first_on]
                    .iter()
                    .any(|(target, a, v)| *target == id && *a == 51 && v == &[0])
            );
            assert!(
                writes[..first_on]
                    .iter()
                    .any(|(target, a, v)| *target == id && *a == 48 && v == &1000u16.to_le_bytes())
            );
        }
        assert!(
            writes
                .iter()
                .filter(|(_, a, _)| *a == 51)
                .all(|(_, _, v)| v == &[0])
        );
        assert!(
            writes
                .iter()
                .filter(|(_, a, _)| *a == 48)
                .all(|(_, _, v)| v == &1000u16.to_le_bytes())
        );
        assert!(
            writes
                .iter()
                .all(|(_, a, _)| [40, 42, 48, 50, 51].contains(a))
        );
        assert!(
            writes
                .iter()
                .filter(|(_, a, _)| *a == 50)
                .all(|(_, _, p)| p.len() == 1),
            "both applying and restoring P must leave register 51 untouched"
        );
        assert!(
            writes
                .iter()
                .filter(|(_, a, _)| *a == 42)
                .all(|(_, _, p)| p.len() == 6 && p[2..] == [0, 0, 0, 0])
        );
        assert!(!io.hd1910.diagnostics().restore_pending);
    }
    #[test]
    fn model_contract_p_is_not_replaced_by_xl330_gain_requests() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        assert!(
            device.writes.lock().unwrap().is_empty(),
            "opening the backend must not write gains or targets"
        );
        for _ in 0..30 {
            io.read().unwrap();
        }
        for _ in 0..2 {
            io.set_torque(true).unwrap();
            // XL330's gain units do not override the exported HD1910 BAM P.
            io.set_gain(200).unwrap();
            for id in crate::model::JOINT_IDS {
                let reg = device.regs.lock().unwrap()[id as usize];
                assert_eq!(
                    &reg[48..52],
                    &[0xe8, 0x03, crate::deployment::firmware_p(), 0]
                );
                assert_eq!(&reg[21..24], &[32, 40, 0]);
                assert_eq!(reg[52], 0);
                assert_eq!(reg[41], 64);
            }
            let diagnostic = io.hd1910.diagnostics();
            assert!(diagnostic.temporary_gains_applied);
            io.set_torque(false).unwrap();
            for id in crate::model::JOINT_IDS {
                let reg = device.regs.lock().unwrap()[id as usize];
                assert_eq!(&reg[48..53], &[0xe8, 0x03, 32, 0, 0]);
            }
            assert!(!io.hd1910.diagnostics().restore_pending);
        }
        assert!(
            device
                .writes
                .lock()
                .unwrap()
                .iter()
                .all(|(_, a, p)| [40, 42, 48, 50, 51].contains(a) && (*a != 50 || p.len() == 1))
        );
    }

    #[test]
    fn hd1910_gain_readback_failure_never_enables_torque_and_remains_restorable() {
        let (bus, device) = emulator(true);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        for _ in 0..30 {
            io.read().unwrap();
        }
        let result = io.set_torque(true).unwrap_err().to_string();
        assert!(result.contains("ID 22"), "{result}");
        assert!(io.hd1910.diagnostics().restore_pending);
        assert!(
            !device
                .writes
                .lock()
                .unwrap()
                .iter()
                .any(|(_, a, v)| *a == 40 && v[0] == 1)
        );
        io.set_torque(false).unwrap();
        for id in crate::model::JOINT_IDS {
            assert_eq!(
                &device.regs.lock().unwrap()[id as usize][50..53],
                &[32, 40, 0]
            );
        }
    }

    #[test]
    fn nonfive_model_contract_p_is_used_with_fixed_d_and_limit() {
        // Deployment is process-scoped, so exercise an alternate P in a child
        // test process instead of changing the other tests' installed model.
        const CHILD: &str = "MICRODUCK_TEST_ALTERNATE_MODEL_P";
        if std::env::var_os(CHILD).is_none() {
            let result=std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact","feetech::hd1910_transport_tests::nonfive_model_contract_p_is_used_with_fixed_d_and_limit","--test-threads=1"])
                .env(CHILD,"1").output().unwrap();
            assert!(
                result.status.success(),
                "{}",
                String::from_utf8_lossy(&result.stderr)
            );
            return;
        }
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../packaging/models/1cbf8730eadd75a5704332c9f3c10f60/policy.onnx");
        let mut contract = crate::deployment::Contract::load(&path).unwrap();
        contract.firmware_p = 7;
        crate::deployment::install(&contract).unwrap();
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        for _ in 0..30 {
            io.read().unwrap();
        }
        io.set_torque(true).unwrap();
        for id in crate::model::JOINT_IDS {
            assert_eq!(
                &device.regs.lock().unwrap()[id as usize][48..52],
                &[0xe8, 0x03, 7, 0]
            );
        }
        io.set_torque(false).unwrap();
    }

    /// Request frames as hex; consecutive repeats are folded into "<hex> x<count>".
    fn folded_frames(device: &Emulator) -> Vec<String> {
        let mut folded: Vec<(String, usize)> = Vec::new();
        for frame in device.frames.lock().unwrap().iter() {
            let hex: String = frame.iter().map(|b| format!("{b:02x}")).collect();
            match folded.last_mut() {
                Some((last, count)) if *last == hex => *count += 1,
                _ => folded.push((hex, 1)),
            }
        }
        folded
            .into_iter()
            .map(|(hex, count)| {
                if count == 1 {
                    hex
                } else {
                    format!("{hex} x{count}")
                }
            })
            .collect()
    }

    // V167 golden: every request frame robotd sends from the startup limp check through two
    // HOME arm/relax cycles, captured by running this same test on unmodified kissqy 78bef11
    // (only the emulator's frame log added). Without servo-test-parameters.json the bus
    // traffic and the hd1910 diagnostics must stay byte for byte the same.
    const V167_FRAMES: &[&str] = &[
        "fffffe22832801140015001600170018001e001f002000210022000a000b000c000d000e00e9",
        "fffffe1382280114151617181e1f2021220a0b0c0d0ef9",
        "ffffc8040200052c",
        "fffffe1482380fc814151617181e1f2021220a0b0c0d0e12 x30",
        "fffffe1382280114151617181e1f2021220a0b0c0d0ef9",
        "fffffe1382150314151617181e1f2021220a0b0c0d0e0a",
        "fffffe1382210114151617181e1f2021220a0b0c0d0e00",
        "fffffe1382320314151617181e1f2021220a0b0c0d0eed",
        "fffffe1382290714151617181e1f2021220a0b0c0d0ef2",
        "fffffe22833201140515051605170518051e051f052005210522050a050b050c050d050e0594",
        "fffffe1382320114151617181e1f2021220a0b0c0d0eef",
        "fffffe22833301140015001600170018001e001f002000210022000a000b000c000d000e00de",
        "fffffe3183300214e80315e80316e80317e80318e8031ee8031fe80320e80321e80322e8030ae8030be8030ce8030de8030ee8030c",
        "fffffe1482380fc814151617181e1f2021220a0b0c0d0e12",
        "fffffe6d832a0614000800000000150008000000001600080000000017000800000000180008000000001e0008000000001f0008000000002000080000000021000800000000220008000000000a0008000000000b0008000000000c0008000000000d0008000000000e0008000000001f",
        "fffffe13822a0214151617181e1f2021220a0b0c0d0ef6",
        "fffffe22832801140115011601170118011e011f012001210122010a010b010c010d010e01da",
        "fffffe1382280114151617181e1f2021220a0b0c0d0ef9",
        "fffffe1482380fc814151617181e1f2021220a0b0c0d0e12",
        "fffffe6d832a0614410800000000154108000000001641080000000017410800000000184108000000001e4108000000001f4108000000002041080000000021410800000000224108000000000a4108000000000b4108000000000c4108000000000d4108000000000e41080000000050",
        "fffffe22832801140015001600170018001e001f002000210022000a000b000c000d000e00e9",
        "fffffe1382280114151617181e1f2021220a0b0c0d0ef9",
        "fffffe22833201142015201620172018201e201f202020212022200a200b200c200d200e20ff",
        "fffffe1382320114151617181e1f2021220a0b0c0d0eef",
        "fffffe1382280114151617181e1f2021220a0b0c0d0ef9",
        "fffffe1382150314151617181e1f2021220a0b0c0d0e0a",
        "fffffe1382210114151617181e1f2021220a0b0c0d0e00",
        "fffffe1382320314151617181e1f2021220a0b0c0d0eed",
        "fffffe1382290714151617181e1f2021220a0b0c0d0ef2",
        "fffffe22833201140515051605170518051e051f052005210522050a050b050c050d050e0594",
        "fffffe1382320114151617181e1f2021220a0b0c0d0eef",
        "fffffe22833301140015001600170018001e001f002000210022000a000b000c000d000e00de",
        "fffffe3183300214e80315e80316e80317e80318e8031ee8031fe80320e80321e80322e8030ae8030be8030ce8030de8030ee8030c",
        "fffffe1482380fc814151617181e1f2021220a0b0c0d0e12",
        "fffffe6d832a0614000800000000150008000000001600080000000017000800000000180008000000001e0008000000001f0008000000002000080000000021000800000000220008000000000a0008000000000b0008000000000c0008000000000d0008000000000e0008000000001f",
        "fffffe13822a0214151617181e1f2021220a0b0c0d0ef6",
        "fffffe22832801140115011601170118011e011f012001210122010a010b010c010d010e01da",
        "fffffe1382280114151617181e1f2021220a0b0c0d0ef9",
        "fffffe1482380fc814151617181e1f2021220a0b0c0d0e12",
        "fffffe6d832a0614410800000000154108000000001641080000000017410800000000184108000000001e4108000000001f4108000000002041080000000021410800000000224108000000000a4108000000000b4108000000000c4108000000000d4108000000000e41080000000050",
        "fffffe22832801140015001600170018001e001f002000210022000a000b000c000d000e00e9",
        "fffffe1382280114151617181e1f2021220a0b0c0d0ef9",
        "fffffe22833201142015201620172018201e201f202020212022200a200b200c200d200e20ff",
        "fffffe1382320114151617181e1f2021220a0b0c0d0eef",
    ];
    const V167_HD1910_DIAGNOSTICS: &str = r#"{"acceleration_policy":"preserve_register_41_never_written","goal_address":42,"goal_length":6,"goal_speed_raw":0,"profile":"hd1910-selftrained","registers_before_arm":[{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":20,"mode":4,"name":"left_hip_yaw","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":21,"mode":4,"name":"left_hip_roll","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":22,"mode":4,"name":"left_hip_pitch","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":23,"mode":4,"name":"left_knee","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":24,"mode":4,"name":"left_ankle","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":30,"mode":4,"name":"neck_pitch","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":31,"mode":4,"name":"head_pitch","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":32,"mode":4,"name":"head_yaw","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":33,"mode":4,"name":"head_roll","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":34,"mode":4,"name":"mouth","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":10,"mode":4,"name":"right_hip_yaw","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":11,"mode":4,"name":"right_hip_roll","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":12,"mode":4,"name":"right_hip_pitch","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":13,"mode":4,"name":"right_knee","temporary_pdi":[32,0,0]},{"acceleration_raw":64,"eeprom_pdi":[32,40,0],"goal_current_raw":0,"goal_speed_raw":0,"id":14,"mode":4,"name":"right_ankle","temporary_pdi":[32,0,0]}],"restore_pending":false,"temporary_gains_applied":false}"#;

    #[test]
    fn v167_golden_frames_and_diagnostics_without_parameter_file() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::for_startup(bus, calibration()).unwrap();
        io.confirm_startup_limp().unwrap();
        for _ in 0..30 {
            io.read().unwrap();
        }
        for _ in 0..2 {
            io.set_torque(true).unwrap();
            io.set_gain(200).unwrap();
            io.write(&JointTargets::new([0.1; NUM_JOINTS])).unwrap();
            io.set_torque(false).unwrap();
        }
        let frames = folded_frames(&device);
        let mut hd1910 = serde_json::to_value(io.hd1910.diagnostics()).unwrap();
        // The only addition allowed without the file: "servo_parameters": null.
        let added = hd1910.as_object_mut().unwrap().remove("servo_parameters");
        assert!(matches!(added, None | Some(serde_json::Value::Null)));
        let gain_mode = io.feetech_diagnostics().unwrap().gain_mode;
        if std::env::var_os("MICRODUCK_PRINT_V167_GOLDEN").is_some() {
            println!("GOLDEN-FRAMES-BEGIN");
            for frame in &frames {
                println!("{frame}");
            }
            println!("GOLDEN-FRAMES-END");
            println!("GOLDEN-DIAG {}", serde_json::to_string(&hd1910).unwrap());
            println!("GOLDEN-GAIN-MODE {gain_mode}");
        }
        assert_eq!(frames, V167_FRAMES);
        assert_eq!(
            hd1910,
            serde_json::from_str::<serde_json::Value>(V167_HD1910_DIAGNOSTICS).unwrap()
        );
        assert_eq!(
            gain_mode,
            "hd1910_model_contract_p_restore_on_relax_native_d_torque_limit_preserved"
        );
    }

    const V167_GAIN_MODE: &str =
        "hd1910_model_contract_p_restore_on_relax_native_d_torque_limit_preserved";
    const FILE_GAIN_MODE: &str = "hd1910_servo_parameters_file_relax_p_restored_d0_limit1000";
    const FILE_3_0_681: crate::hd1910_test::Parameters = crate::hd1910_test::Parameters {
        kp: 3,
        kd: 0,
        torque_limit: 681,
    };

    /// Distinct per-servo acceleration, I and torque limit, so nothing is reset by accident.
    fn distinct_registers(device: &Emulator) {
        for id in crate::model::JOINT_IDS {
            let mut registers = device.regs.lock().unwrap();
            registers[id as usize][41] = id * 3;
            registers[id as usize][52] = id;
            registers[id as usize][48..50].copy_from_slice(&(800u16 + id as u16).to_le_bytes());
        }
    }

    #[test]
    fn servo_file_arm_writes_48_once_and_relax_leaves_p_restored_d0_limit1000() {
        let (bus, device) = emulator(false);
        distinct_registers(&device);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        io.set_servo_test(Some(FILE_3_0_681));
        assert!(
            device.writes.lock().unwrap().is_empty(),
            "loading the file must not write motors"
        );
        for _ in 0..30 {
            io.read().unwrap();
        }
        for _ in 0..2 {
            device.reads.lock().unwrap().clear();
            io.set_torque(true).unwrap();
            assert_eq!(
                *device.reads.lock().unwrap(),
                vec![
                    (40, 1),
                    (21, 3),
                    (33, 1),
                    (50, 3),
                    (41, 7),
                    (48, 4),
                    (56, 15),
                    (42, 2),
                    (40, 1),
                    (56, 15)
                ],
                "the file path reads 48x4 back instead of 50x1; nothing else changes"
            );
            // Policy gain requests and model switches must not overwrite the file values.
            io.set_gain(200).unwrap();
            io.write(&JointTargets::new([0.1; NUM_JOINTS])).unwrap();
            for id in crate::model::JOINT_IDS {
                let reg = device.regs.lock().unwrap()[id as usize];
                assert_eq!(&reg[48..53], &[0xa9, 0x02, 3, 0, id]);
                assert_eq!(&reg[21..24], &[32, 40, 0]);
                assert_eq!(reg[41], id * 3);
            }
            let diagnostic = io.hd1910.diagnostics();
            assert_eq!(diagnostic.servo_parameters, Some(FILE_3_0_681));
            assert!(diagnostic.temporary_gains_applied && diagnostic.restore_pending);
            device.reads.lock().unwrap().clear();
            io.set_torque(false).unwrap();
            assert_eq!(*device.reads.lock().unwrap(), vec![(40, 1), (48, 4)]);
            for id in crate::model::JOINT_IDS {
                let reg = device.regs.lock().unwrap()[id as usize];
                assert_eq!(&reg[48..53], &[0xe8, 0x03, 32, 0, id]);
                assert_eq!(reg[41], id * 3);
            }
            let diagnostic = io.hd1910.diagnostics();
            assert!(!diagnostic.restore_pending && !diagnostic.temporary_gains_applied);
        }
        let writes = device.writes.lock().unwrap();
        assert!(writes.iter().all(|(_, a, _)| [40, 42, 48].contains(a)));
        let at_48: Vec<_> = writes.iter().filter(|(_, a, _)| *a == 48).collect();
        assert_eq!(
            at_48.len(),
            4 * 15,
            "one arm write and one relax write per cycle"
        );
        assert!(at_48.iter().all(|(_, _, v)| v.len() == 4));
    }

    #[test]
    fn servo_file_partial_write_never_enables_torque_and_relax_restores_every_servo() {
        let (bus, device) = emulator(true);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        io.set_servo_test(Some(FILE_3_0_681));
        for _ in 0..30 {
            io.read().unwrap();
        }
        let result = io.set_torque(true).unwrap_err().to_string();
        assert!(result.contains("ID 22"), "{result}");
        assert!(io.hd1910.diagnostics().restore_pending);
        io.read().unwrap();
        let pending = io.set_torque(true).unwrap_err().to_string();
        assert!(pending.contains("restoration pending"), "{pending}");
        io.set_torque(false).unwrap();
        for id in crate::model::JOINT_IDS {
            assert_eq!(
                &device.regs.lock().unwrap()[id as usize][48..53],
                &[0xe8, 0x03, 32, 0, 0]
            );
        }
        assert!(!io.hd1910.diagnostics().restore_pending);
        // After relax a new torque-on is tried again (and the same servo refuses it again).
        io.read().unwrap();
        let retry = io.set_torque(true).unwrap_err().to_string();
        assert!(
            retry.contains("ID 22") && !retry.contains("pending"),
            "{retry}"
        );
        io.set_torque(false).unwrap();
        assert!(
            !device
                .writes
                .lock()
                .unwrap()
                .iter()
                .any(|(_, a, v)| *a == 40 && v[0] == 1),
            "a failed P/D/torque-limit write must never enable torque"
        );
    }

    /// File {contract P, 0, 1000} and no file leave every register byte the same after each
    /// arm and each relax: the file path is a different route to the same V167 RAM state.
    #[test]
    fn servo_file_with_contract_p_d0_limit1000_matches_no_file_register_for_register() {
        let run = |file: Option<crate::hd1910_test::Parameters>| {
            let (bus, device) = emulator(false);
            distinct_registers(&device);
            let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
            io.set_servo_test(file);
            for _ in 0..30 {
                io.read().unwrap();
            }
            let mut states = Vec::new();
            for _ in 0..2 {
                io.set_torque(true).unwrap();
                io.write(&JointTargets::new([0.1; NUM_JOINTS])).unwrap();
                states.push(device.regs.lock().unwrap().clone());
                io.set_torque(false).unwrap();
                states.push(device.regs.lock().unwrap().clone());
            }
            states
        };
        let contract = crate::hd1910_test::Parameters {
            kp: crate::deployment::firmware_p(),
            kd: 0,
            torque_limit: 1000,
        };
        assert!(run(Some(contract)) == run(None));
    }

    #[test]
    fn servo_file_with_torque_already_on_is_verified_never_written() {
        for matching in [true, false] {
            let (bus, device) = emulator(false);
            for id in crate::model::JOINT_IDS {
                let mut registers = device.regs.lock().unwrap();
                registers[id as usize][40] = 1;
                if matching {
                    registers[id as usize][48..52].copy_from_slice(&[0xa9, 0x02, 3, 0]);
                }
            }
            let before = device.regs.lock().unwrap().clone();
            let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
            assert_eq!(io.torque_state_confirmed, Some(true));
            io.set_servo_test(Some(FILE_3_0_681));
            for _ in 0..30 {
                io.read().unwrap();
            }
            let result = io.set_torque(true);
            assert!(
                device
                    .writes
                    .lock()
                    .unwrap()
                    .iter()
                    .all(|(_, a, _)| ![48, 50, 51].contains(a)),
                "with torque on, the file values are verified, never written"
            );
            let diagnostic = io.hd1910.diagnostics();
            assert!(!diagnostic.restore_pending, "no relax data is invented");
            if matching {
                result.unwrap();
                assert!(diagnostic.temporary_gains_applied);
                io.set_torque(false).unwrap();
                assert!(
                    device
                        .writes
                        .lock()
                        .unwrap()
                        .iter()
                        .all(|(_, a, _)| *a != 48)
                );
                for id in crate::model::JOINT_IDS {
                    assert_eq!(
                        &device.regs.lock().unwrap()[id as usize][48..52],
                        &[0xa9, 0x02, 3, 0]
                    );
                }
            } else {
                let error = result.unwrap_err().to_string();
                assert!(
                    error.contains("ID 20")
                        && error.contains("mismatch")
                        && error.contains("torque stays on; relax first"),
                    "{error}"
                );
                assert!(device.writes.lock().unwrap().is_empty());
                assert!(*device.regs.lock().unwrap() == before, "torque stays on");
                assert!(!diagnostic.temporary_gains_applied);
            }
        }
    }

    #[test]
    fn diagnostics_add_only_servo_parameters_and_keep_the_v167_gain_mode_without_the_file() {
        let (bus, _device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        for _ in 0..3 {
            io.read().unwrap();
        }
        let v167: BTreeSet<String> = [
            "acceleration_policy",
            "goal_address",
            "goal_length",
            "goal_speed_raw",
            "profile",
            "registers_before_arm",
            "restore_pending",
            "temporary_gains_applied",
        ]
        .map(String::from)
        .into();
        let mut expected_keys = v167.clone();
        expected_keys.insert("servo_parameters".into());
        for (file, gain_mode, reported) in [
            (None, V167_GAIN_MODE, serde_json::Value::Null),
            (
                Some(FILE_3_0_681),
                FILE_GAIN_MODE,
                serde_json::json!({"kp": 3, "kd": 0, "torque_limit": 681}),
            ),
        ] {
            io.set_servo_test(file);
            let wire = serde_json::to_value(io.feetech_diagnostics().unwrap()).unwrap();
            let keys: BTreeSet<String> = wire["hd1910"]
                .as_object()
                .unwrap()
                .keys()
                .cloned()
                .collect();
            assert_eq!(keys, expected_keys);
            assert_eq!(wire["hd1910"]["servo_parameters"], reported);
            assert_eq!(wire["gain_mode"], gain_mode);
            let snapshot = io.feetech_snapshot().unwrap();
            assert_eq!(snapshot.dynamic.gain_mode, gain_mode);
            assert_eq!(
                serde_json::to_value(&snapshot.hd1910).unwrap()["servo_parameters"],
                reported
            );
        }
    }

    fn set_positions(device: &Emulator, raw: u16) {
        for id in crate::model::JOINT_IDS {
            device.regs.lock().unwrap()[id as usize][56..58].copy_from_slice(&raw.to_le_bytes());
        }
    }
    fn index(id: u8) -> usize {
        crate::model::JOINT_IDS
            .iter()
            .position(|v| *v == id)
            .unwrap()
    }

    #[test]
    fn native_speed_reaches_policy_with_signed_magnitude_local_directions_and_no_current_claim() {
        use crate::obs::{ACTION_LEN, Command, Observation};
        let (bus, device) = emulator(false);
        let mut c = calibration();
        let wire: [u16; NUM_JOINTS] = [
            0, 0x8000, 1, 0x8001, 0x7fff, 0xffff, 13, 0x800d, 64, 0x8040, 7, 0x8007, 123, 0x807b, 2,
        ];
        // These are signed-magnitude values, NOT i16 two's-complement.
        let signed = [
            0, 0, 1, -1, 32767, -32767, 13, -13, 64, -64, 7, -7, 123, -123, 2,
        ];
        for (i, j) in c.joints.iter_mut().enumerate() {
            j.direction = if i % 2 == 0 { 1 } else { -1 };
            // Distinct zero points must have no effect on velocity.
            j.zero_raw = 1800 + i as u16 * 9;
            device.regs.lock().unwrap()[j.id as usize][58..60]
                .copy_from_slice(&wire[i].to_le_bytes());
        }
        assert!(!c.units_verified);
        let expected: [f64; NUM_JOINTS] = std::array::from_fn(|i| {
            (f64::from(signed[i]) * 4.39453125).to_radians() * f64::from(c.joints[i].direction)
        });
        let mut io = FeetechIo::from_bus(bus, c).unwrap();
        // First sample already has speed, despite having no position history.
        let first = io.read().unwrap();
        for (i, want) in expected.iter().enumerate() {
            assert!((first.velocities[i] - want).abs() < 1e-10, "joint {i}");
            assert!(first.currents_ma[i].is_nan());
        }
        let obs = Observation::build(
            &first.imu,
            &first.positions,
            &first.velocities,
            &crate::model::DEFAULT_POSITION,
            &[0.; ACTION_LEN],
            &Command::default(),
        );
        let expected_policy: Vec<_> = expected
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != crate::model::MOUTH_INDEX)
            .map(|(_, v)| *v as f32)
            .collect();
        assert_eq!(&obs.as_slice()[20..34], expected_policy.as_slice());
        let diag = io.feetech_diagnostics().unwrap();
        assert_eq!(diag.velocity_source, crate::hd1910::VELOCITY_SOURCE);
        assert!((diag.velocity_rad_s_per_count.unwrap() - 0.07669903939428206).abs() < 1e-15);
        assert!(!diag.current_available);
        assert!(!io.calibration.units_verified);
        // A large position change cannot manufacture speed when native speed is zero.
        set_positions(&device, 2200);
        for id in crate::model::JOINT_IDS {
            device.regs.lock().unwrap()[id as usize][58..60].copy_from_slice(&[0, 0]);
        }
        io.hd1910 = crate::hd1910::State::default();
        let stopped = io.read().unwrap();
        assert_eq!(stopped.velocities, [0.; NUM_JOINTS]);
        assert_ne!(stopped.positions, first.positions);
    }

    #[test]
    fn twenty_ms_missing_replies_keep_failure_identity_and_never_refresh_expired_imu() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        for _ in 0..30 {
            io.read().unwrap();
        }
        io.set_torque(true).unwrap();
        for id in crate::model::JOINT_IDS {
            let before = io.latest_sample_at;
            let joint_time = io.joint_feedback[index(id)].as_ref().unwrap().at;
            device.faults.lock().unwrap().missing = vec![id];
            // FT6 only reports an age upper bound of 30 ms. Waiting the full
            // 20 ms consumes the 50 ms cap: reject, then let upstream coast.
            assert!(
                io.read()
                    .unwrap_err()
                    .to_string()
                    .contains("IMU not ready or stale")
            );
            assert_eq!(io.latest_sample_at, before);
            assert_eq!(
                io.joint_feedback[index(id)].as_ref().unwrap().at,
                joint_time
            );
            let diag = io.feetech_diagnostics().unwrap();
            assert_eq!(diag.sync.missing_ids, vec![id]);
            assert_eq!(diag.telemetry.missing_by_id[&id], 1);
            assert_eq!(diag.torque_state_confirmed, Some(true));
            assert!(
                io.set_torque(true).is_err(),
                "missing feedback must not re-arm"
            );
            device.faults.lock().unwrap().missing.clear();
            io.read().unwrap();
            assert!(io.latest_sample_at > before);
        }
        device.faults.lock().unwrap().missing = vec![10, 32, 34];
        let before = io.latest_sample_at;
        for _ in 0..4 {
            assert!(io.read().is_err());
        }
        assert_eq!(io.latest_sample_at, before);
        assert_eq!(
            io.feetech_diagnostics().unwrap().sync.missing_ids,
            vec![10, 32, 34]
        );
        assert!(
            !device
                .writes
                .lock()
                .unwrap()
                .iter()
                .any(|(_, addr, data)| *addr == 40 && data == &vec![0])
        );
        device.faults.lock().unwrap().missing.clear();
        io.read().unwrap();
        assert!(io.held_joint_positions().iter().all(Option::is_none));
        io.set_torque(false).unwrap();
    }

    #[test]
    fn joint_recovery_keeps_real_timestamps_and_has_a_wall_clock_limit() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        io.read().unwrap();
        let j = index(24);
        let t = Instant::now() - Duration::from_millis(40);
        io.joint_feedback[j].as_mut().unwrap().at = t;
        io.joint_feedback[j].as_mut().unwrap().velocity = 0.75;
        device.faults.lock().unwrap().missing = vec![24];
        assert!(io.read().is_err());
        assert_eq!(io.joint_feedback[j].as_ref().unwrap().velocity, 0.75);
        assert_eq!(io.joint_feedback[j].as_ref().unwrap().at, t);
        io.joint_feedback[j].as_mut().unwrap().at = Instant::now() - Duration::from_millis(120);
        assert!(io.read().is_err());
        assert!(
            io.held_joint_positions()[j].is_some(),
            "elapsed time must cap a delayed loop"
        );
        // Inspect the recovered native speed without smoothing history. A position
        // jump across missing frames must not be differentiated into a speed spike.
        io.hd1910 = crate::hd1910::State::default();
        let then = Instant::now() - Duration::from_millis(40);
        io.joint_feedback[j].as_mut().unwrap().at = then;
        set_positions(&device, 2100);
        device.regs.lock().unwrap()[24][58..60].copy_from_slice(&0x8008u16.to_le_bytes());
        device.faults.lock().unwrap().missing.clear();
        let recovered = io.read().unwrap();
        let now = io.joint_feedback[j].as_ref().unwrap().at;
        let expected = (-8.0_f64 * 4.39453125).to_radians();
        assert!((recovered.velocities[j] - expected).abs() < 1e-10);
        assert!(now > then);
        assert_eq!(io.joint_missing[j], 0);
        assert!(io.held_joint_positions()[j].is_none());
    }

    #[test]
    fn imu_protocol_faults_and_missing_initial_feedback_stay_strict() {
        let (bus, device) = emulator(false);
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        device.faults.lock().unwrap().missing = vec![32];
        assert!(
            io.read().is_err(),
            "missing initial feedback must never become a usable observation"
        );
        device.faults.lock().unwrap().missing.clear();
        io.read().unwrap();
        for fault in [
            Faults {
                missing: vec![IMU_ID],
                ..Faults::default()
            },
            Faults {
                checksum_id: Some(32),
                ..Faults::default()
            },
            Faults {
                status_id: Some(32),
                ..Faults::default()
            },
            Faults {
                imu_stale: true,
                ..Faults::default()
            },
        ] {
            *device.faults.lock().unwrap() = fault;
            let old = io.latest_sample_at;
            assert!(io.read().is_err());
            assert_eq!(
                io.latest_sample_at, old,
                "invalid transaction must not retimestamp data"
            );
        }
        *device.faults.lock().unwrap() = Faults::default();
        io.read().unwrap();
        device.faults.lock().unwrap().missing = vec![32];
        assert!(
            io.bus.telemetry(&crate::model::JOINT_IDS).is_err(),
            "commissioning API remains strict"
        );
        assert!(
            io.read_sample(false).is_err(),
            "torque-on refresh remains strict"
        );
        device.faults.lock().unwrap().missing = crate::model::JOINT_IDS.to_vec();
        assert!(
            io.read().is_err(),
            "expired IMU plus absent joints is not a fresh observation"
        );
        assert_eq!(io.feetech_diagnostics().unwrap().sync.missing_ids.len(), 15);
    }

    #[test]
    fn restart_preserves_powered_servos_without_writing_until_explicit_init() {
        let (bus, device) = emulator(false);
        for id in crate::model::JOINT_IDS {
            let mut regs = device.regs.lock().unwrap();
            regs[id as usize][40] = 1;
            regs[id as usize][50] = crate::deployment::firmware_p();
            regs[id as usize][51] = id * 2;
        }
        let mut io = FeetechIo::from_bus(bus, calibration()).unwrap();
        assert_eq!(io.torque_state_confirmed, Some(true));
        for _ in 0..30 {
            io.read().unwrap();
        }
        io.write(&JointTargets::new([0.; NUM_JOINTS])).unwrap();
        assert!(
            device.writes.lock().unwrap().is_empty(),
            "opening/restarting must be read-only"
        );
        io.set_torque(true).unwrap();
        for id in crate::model::JOINT_IDS {
            let reg = device.regs.lock().unwrap()[id as usize];
            assert_eq!(
                &reg[48..52],
                &[0xe8, 0x03, crate::deployment::firmware_p(), 0]
            );
        }
        assert!(
            !device
                .writes
                .lock()
                .unwrap()
                .iter()
                .any(|(_, a, v)| *a == 40 && v[0] == 0)
        );
        assert!(
            !device
                .writes
                .lock()
                .unwrap()
                .iter()
                .any(|(_, a, _)| *a == 50),
            "verified existing gains need no rewrite"
        );
        io.set_torque(false).unwrap();
        assert!(
            device
                .writes
                .lock()
                .unwrap()
                .iter()
                .any(|(_, a, v)| *a == 40 && v[0] == 0)
        );
    }
}
