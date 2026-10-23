use super::*;
use serialport::SerialPort;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};

fn core() -> Vec<u8> {
    let mut raw = vec![0u8; 12];
    raw[6..8].copy_from_slice(&0x2e66u16.to_le_bytes()); // finite nonzero SFLP x
    raw
}

struct Peer {
    stop: Arc<AtomicBool>,
    requests: Arc<Mutex<Vec<Vec<u8>>>>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Drop for Peer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.thread.take().unwrap().join().unwrap();
    }
}

fn peer(protocol: BusImuProtocol, imu_delay: Duration, servo_delay: Duration) -> (Bus, Peer) {
    peer_fault(protocol, imu_delay, servo_delay, Fault::None)
}

#[derive(Clone, Copy)]
enum Fault {
    None,
    WrongId,
    Short,
    Status,
    Crc,
    Missing,
    UnknownFeetech,
    NoisyAndMissingServo,
    UnsupportedLayout,
    TrailingNoiseAndMissingServo,
    TrailingForeignAndMissingServo,
    TrailingCrc,
    ForeignPrefixAndMissingServo,
    ChangingCore,
}

fn peer_fault(
    protocol: BusImuProtocol,
    imu_delay: Duration,
    servo_delay: Duration,
    fault: Fault,
) -> (Bus, Peer) {
    let (mut device, mut host) = serialport::TTYPort::pair().unwrap();
    device.set_timeout(Duration::from_millis(2)).unwrap();
    host.set_timeout(Duration::from_millis(2)).unwrap();
    let requests = Arc::new(Mutex::new(Vec::new()));
    let seen = requests.clone();
    let stop = Arc::new(AtomicBool::new(false));
    let done = stop.clone();
    let thread = std::thread::spawn(move || {
        let mut bytes = Vec::new();
        let mut core_reads = 0;
        let mut next = [0];
        while !done.load(Ordering::Relaxed) {
            if device.read(&mut next).unwrap_or(0) == 0 {
                continue;
            }
            bytes.push(next[0]);
            if bytes.len() < 4 {
                continue;
            }
            let dxl = bytes[2] == 253;
            if dxl && bytes.len() < 7 {
                continue;
            }
            let length = if dxl {
                7 + usize::from(u16::from_le_bytes([bytes[5], bytes[6]]))
            } else {
                4 + usize::from(bytes[3])
            };
            if bytes.len() < length {
                continue;
            }
            let tx = std::mem::take(&mut bytes);
            seen.lock().unwrap().push(tx.clone());
            if dxl {
                if protocol != BusImuProtocol::Dxl2 {
                    continue;
                }
                let address = u16::from_le_bytes([tx[8], tx[9]]);
                let mut payload = if address == 0 {
                    vec![0x44, 0x4d, 1]
                } else {
                    core()
                };
                let mut error = 0;
                let mut id = IMU_ID;
                if address == 124 {
                    core_reads += 1;
                    if matches!(fault, Fault::ChangingCore) {
                        payload[0] = core_reads as u8;
                    }
                    if matches!(fault, Fault::UnsupportedLayout) {
                        payload.pop();
                    }
                    if core_reads > 1 {
                        match fault {
                            Fault::Missing => continue,
                            Fault::Short => {
                                payload.pop();
                            }
                            Fault::Status => error = 4,
                            Fault::WrongId => id = 199,
                            _ => {}
                        }
                    }
                }
                std::thread::sleep(imu_delay);
                let mut response =
                    crate::dxl2::packet(id, 0x55, &[&[error][..], &payload].concat());
                if address == 124 && core_reads > 1 && matches!(fault, Fault::Crc) {
                    *response.last_mut().unwrap() ^= 1;
                }
                if address == 124 && core_reads > 1 && matches!(fault, Fault::NoisyAndMissingServo)
                {
                    let _ = device.write_all(b"noise");
                }
                if address == 124 && core_reads > 1 {
                    match fault {
                        Fault::TrailingNoiseAndMissingServo => response.extend_from_slice(b"noise"),
                        Fault::TrailingForeignAndMissingServo => response
                            .extend_from_slice(&crate::dxl2::packet(199, 0x55, &[0, 1, 2, 3])),
                        Fault::ForeignPrefixAndMissingServo => {
                            response = [
                                &crate::dxl2::packet(199, 0x55, &[0, 1, 2, 3])[..],
                                &response[..],
                            ]
                            .concat()
                        }
                        Fault::TrailingCrc => {
                            let mut corrupt = response.clone();
                            *corrupt.last_mut().unwrap() ^= 1;
                            response.extend_from_slice(&corrupt);
                        }
                        _ => {}
                    }
                }
                let _ = device.write_all(&response);
            } else if tx[4] == READ && tx[2] == IMU_ID {
                if protocol != BusImuProtocol::Feetech {
                    continue;
                }
                let identity = if matches!(fault, Fault::UnknownFeetech) {
                    [1, 2, 3, 4, 5]
                } else {
                    IMU_IDENTITY
                };
                let _ = device.write_all(&packet(IMU_ID, 0, &identity).unwrap());
            } else if tx[4] == SYNC_READ {
                std::thread::sleep(servo_delay);
                for id in &tx[7..tx.len() - 1] {
                    if *id == 13
                        && matches!(
                            fault,
                            Fault::NoisyAndMissingServo
                                | Fault::TrailingNoiseAndMissingServo
                                | Fault::TrailingForeignAndMissingServo
                                | Fault::ForeignPrefixAndMissingServo
                        )
                    {
                        continue;
                    }
                    let mut raw = if *id == IMU_ID { core() } else { vec![0; 12] };
                    raw.extend_from_slice(&[1, 0, 0]);
                    let _ = device.write_all(&packet(*id, 0, &raw).unwrap());
                }
            }
        }
    });
    let bus = Bus {
        port: Box::new(host),
        timeout: Duration::from_millis(30),
        read_only: false,
        last_sync: SyncTrace::default(),
        imu_board: BusImuBoard::Ft6,
        imu_protocol: BusImuProtocol::Auto,
        imu_identity: None,
    };
    (
        bus,
        Peer {
            stop,
            requests,
            thread: Some(thread),
        },
    )
}

#[test]
fn auto_detects_feetech_without_sending_dxl2_or_writes() {
    let (mut bus, device) = peer(BusImuProtocol::Feetech, Duration::ZERO, Duration::ZERO);
    assert_eq!(bus.identify_imu().unwrap(), BusImuBoard::Ft6);
    let tx = device.requests.lock().unwrap();
    assert_eq!(tx.len(), 1);
    assert_eq!(tx[0], packet(IMU_ID, READ, &[0, 5]).unwrap());
    assert_eq!(bus.imu_info().unwrap().protocol, "feetech");
}

#[test]
fn auto_detects_dxl2_and_reads_feetech_servos_separately() {
    let (mut bus, device) = peer(BusImuProtocol::Dxl2, Duration::ZERO, Duration::ZERO);
    assert_eq!(bus.identify_imu().unwrap(), BusImuBoard::Dxl2);
    let ids = crate::model::JOINT_IDS;
    let blocks = bus.telemetry(&ids).unwrap();
    assert_eq!(blocks.len(), 16);
    assert_eq!(blocks[&IMU_ID], core());
    let tx = device.requests.lock().unwrap();
    assert_eq!(tx[1], crate::dxl2::packet(IMU_ID, READ, &[0, 0, 3, 0]));
    assert_eq!(tx[2], crate::dxl2::packet(IMU_ID, READ, &[124, 0, 12, 0]));
    assert_eq!(tx[3], crate::dxl2::packet(IMU_ID, READ, &[124, 0, 12, 0]));
    assert_eq!(tx[4], sync_read_packet(56, 15, &ids).unwrap());
    assert!(bus.last_sync.missing_ids.is_empty());
    assert_eq!(bus.imu_info().unwrap().identity, [0x44, 0x4d, 1]);
}

#[test]
fn forced_protocol_does_not_try_the_other_wire_format() {
    let (mut bus, device) = peer(BusImuProtocol::Dxl2, Duration::ZERO, Duration::ZERO);
    bus.set_imu_protocol("feetech").unwrap();
    assert!(bus.identify_imu().is_err());
    assert_eq!(device.requests.lock().unwrap().len(), 1);
    assert!(bus.imu_info().is_none());
    assert!(bus.set_imu_protocol("guess").is_err());
}

#[test]
fn mixed_protocol_telemetry_has_one_total_deadline() {
    let (mut bus, _device) = peer(
        BusImuProtocol::Dxl2,
        Duration::from_millis(12),
        Duration::from_millis(12),
    );
    bus.set_imu_protocol("dxl2").unwrap();
    bus.identify_imu().unwrap();
    let started = Instant::now();
    assert!(bus.telemetry(&crate::model::JOINT_IDS).is_err());
    assert!(started.elapsed() < Duration::from_millis(35));
    assert!(bus.last_sync.deadline_exceeded);
}

#[test]
fn read_only_dxl2_probe_succeeds_but_dxl2_writes_are_rejected() {
    let (mut bus, device) = peer(BusImuProtocol::Dxl2, Duration::ZERO, Duration::ZERO);
    bus.read_only = true;
    bus.identify_imu().unwrap();
    assert!(
        bus.send(
            &crate::dxl2::packet(IMU_ID, 3, &[7, 0, 1]),
            Instant::now() + Duration::from_millis(10)
        )
        .is_err()
    );
    assert_eq!(device.requests.lock().unwrap().len(), 3);
}

#[test]
fn dxl2_sample_has_no_invented_sensor_counter_status_or_age() {
    let sample = ImuSample::decode_dxl2(&core()).unwrap();
    assert_eq!(sample.sequence, None);
    assert_eq!(sample.status, None);
    assert_eq!(sample.age_ms, None);
    assert_eq!(sample.age_upper_bound_ms, None);
    assert!(sample.is_fresh(19_999));
    assert!(!sample.is_fresh(20_001));
    assert!(!ImuSample::decode_dxl2(&[0; 12]).unwrap().ready);
    assert!(ImuSample::decode_dxl2(&[0; 15]).is_err());
    let mut invalid = core();
    invalid[6..8].copy_from_slice(&0x7e00u16.to_le_bytes());
    assert!(ImuSample::decode_dxl2(&invalid).is_err());
}

#[test]
fn dxl2_freeze_detection_uses_real_bytes_and_feetech_keeps_its_counter_guard() {
    let sample = ImuSample::decode_dxl2(&core()).unwrap();
    let mut gate = BusImuFreshness::default();
    for _ in 0..24 {
        gate.check(&sample, 1000).unwrap();
    }
    assert!(gate.check(&sample, 1000).is_err());
    let mut changed = core();
    changed[0] = 1;
    gate.check(&ImuSample::decode_dxl2(&changed).unwrap(), 1000)
        .unwrap();
    let sample = ImuSample::decode(&[core(), vec![5, 0, 0]].concat()).unwrap();
    let mut gate = BusImuFreshness::default();
    for _ in 0..3 {
        gate.check(&sample, 1000).unwrap();
    }
    assert!(gate.check(&sample, 1000).is_err());
}

#[test]
fn a_legacy_feetech_mount_cannot_authorize_dxl2_motion() {
    let mut cal = Calibration::commissioning(crate::model::JOINT_IDS);
    cal.imu_mount_verified = true;
    assert!(cal.validate_bus_mount(BusImuProtocol::Feetech).is_ok());
    assert!(cal.validate_bus_mount(BusImuProtocol::Dxl2).is_err());
    cal.imu_bus_protocol = Some(BusImuProtocol::Dxl2);
    assert!(cal.validate_bus_mount(BusImuProtocol::Dxl2).is_ok());
    assert!(cal.validate_bus_mount(BusImuProtocol::Feetech).is_err());
    let saved = toml::to_string(&cal).unwrap();
    assert!(saved.contains("imu_bus_protocol = \"dxl2\""));
    let restored: Calibration = toml::from_str(&saved).unwrap();
    assert_eq!(restored.imu_bus_protocol, Some(BusImuProtocol::Dxl2));
}

#[test]
fn invalid_dxl2_replies_fail_the_sample_without_sending_servo_queries() {
    for fault in [
        Fault::WrongId,
        Fault::Short,
        Fault::Status,
        Fault::Crc,
        Fault::Missing,
    ] {
        let (mut bus, device) =
            peer_fault(BusImuProtocol::Dxl2, Duration::ZERO, Duration::ZERO, fault);
        bus.set_imu_protocol("dxl2").unwrap();
        bus.identify_imu().unwrap();
        assert!(bus.telemetry(&crate::model::JOINT_IDS).is_err());
        assert_eq!(bus.last_sync.missing_ids, [IMU_ID]);
        assert_eq!(
            (bus.last_sync.address, bus.last_sync.length),
            (SYNC_ADDRESS, SYNC_LENGTH)
        );
        assert!(bus.last_sync.request_sent);
        assert_eq!(device.requests.lock().unwrap().len(), 3);
        if matches!(fault, Fault::Crc) {
            assert_eq!(bus.last_sync.bad_checksums, 1);
        }
        let mut totals = TelemetryTotals::default();
        totals.observe(&bus.last_sync, Some("DXL2 test failure".into()));
        assert_eq!((totals.total_reads, totals.failed_reads), (1, 1));
        assert_eq!(totals.missing_by_id.get(&IMU_ID), Some(&1));
    }
}

#[test]
fn an_unknown_feetech_identity_is_not_reinterpreted_as_another_sensor() {
    let (mut bus, device) = peer_fault(
        BusImuProtocol::Feetech,
        Duration::ZERO,
        Duration::ZERO,
        Fault::UnknownFeetech,
    );
    assert!(
        bus.identify_imu()
            .unwrap_err()
            .to_string()
            .contains("neither FT6")
    );
    assert!(bus.imu_info().is_none());
    assert_eq!(device.requests.lock().unwrap().len(), 1);
}

#[test]
fn noise_in_the_dxl2_part_cannot_soften_a_missing_servo_response() {
    let (mut bus, _device) = peer_fault(
        BusImuProtocol::Dxl2,
        Duration::ZERO,
        Duration::ZERO,
        Fault::NoisyAndMissingServo,
    );
    bus.set_imu_protocol("dxl2").unwrap();
    bus.identify_imu().unwrap();
    assert!(bus.telemetry_partial(&crate::model::JOINT_IDS).is_err());
    assert!(bus.last_sync.discarded > 0);
    assert_eq!(bus.last_sync.missing_ids, [13]);
    assert!(!bus.last_sync.partial_received);
}

#[test]
fn dxl2_identity_alone_cannot_label_an_unsupported_register_layout_as_an_imu() {
    let (mut bus, _device) = peer_fault(
        BusImuProtocol::Dxl2,
        Duration::ZERO,
        Duration::ZERO,
        Fault::UnsupportedLayout,
    );
    bus.set_imu_protocol("dxl2").unwrap();
    assert!(bus.identify_imu().is_err());
    assert!(bus.imu_info().is_none());
}

#[test]
fn bytes_after_a_valid_dxl2_status_are_still_checked_and_counted() {
    for fault in [
        Fault::TrailingNoiseAndMissingServo,
        Fault::TrailingForeignAndMissingServo,
        Fault::ForeignPrefixAndMissingServo,
        Fault::TrailingCrc,
    ] {
        let (mut bus, _device) =
            peer_fault(BusImuProtocol::Dxl2, Duration::ZERO, Duration::ZERO, fault);
        bus.set_imu_protocol("dxl2").unwrap();
        bus.identify_imu().unwrap();
        assert!(bus.telemetry_partial(&crate::model::JOINT_IDS).is_err());
        if matches!(fault, Fault::TrailingCrc) {
            assert_eq!(bus.last_sync.bad_checksums, 1);
        } else {
            assert!(bus.last_sync.discarded > 0);
        }
        assert!(!bus.last_sync.partial_received);
    }
}

#[test]
fn motion_reads_dxl2_and_exposes_real_diagnostics_but_a_legacy_mount_stays_unverified() {
    let (mut bus, _device) = peer_fault(
        BusImuProtocol::Dxl2,
        Duration::ZERO,
        Duration::ZERO,
        Fault::ChangingCore,
    );
    bus.set_imu_protocol("dxl2").unwrap();
    bus.identify_imu().unwrap();
    let mut cal = Calibration::commissioning(crate::model::JOINT_IDS);
    cal.imu_mount_verified = true;
    cal.motion_enabled = true;
    for joint in &mut cal.joints {
        joint.calibrated = true;
        joint.min_rad = Some(-1.5);
        joint.max_rad = Some(1.5);
    }
    let mut io = FeetechIo::for_startup(bus, cal).unwrap();
    for _ in 0..30 {
        io.read().unwrap();
    }
    let diag = io.feetech_diagnostics().unwrap();
    assert!(diag.imu_ready);
    assert!(!diag.imu_mount_verified);
    assert_eq!(diag.imu_bus.unwrap().protocol, "dxl2");
    assert_eq!(
        (
            diag.imu_sequence,
            diag.imu_status,
            diag.imu_age_ms,
            diag.imu_age_upper_bound_ms
        ),
        (None, None, None, None)
    );
    let snapshot = io.feetech_snapshot().unwrap();
    assert_eq!(snapshot.metadata.imu_bus.as_ref().unwrap().protocol, "dxl2");
    assert!(!snapshot.metadata.imu_mount_verified);
    assert!(
        io.set_torque(true)
            .unwrap_err()
            .to_string()
            .contains("redo IMU mounting")
    );
}

struct RefuseServoRequest {
    inner: Box<dyn SerialPort>,
}
impl Read for RefuseServoRequest {
    fn read(&mut self, bytes: &mut [u8]) -> std::io::Result<usize> {
        self.inner.read(bytes)
    }
}
impl Write for RefuseServoRequest {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        if !bytes.starts_with(&crate::dxl2::HEADER) {
            return Err(std::io::Error::new(
                std::io::ErrorKind::BrokenPipe,
                "injected Feetech request failure",
            ));
        }
        self.inner.write(bytes)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}
macro_rules! forward {
    () => {};
    (fn $name:ident(&self $(, $arg:ident: $ty:ty)*) -> $ret:ty; $($rest:tt)*) => {
        fn $name(&self $(, $arg: $ty)*) -> $ret { self.inner.$name($($arg),*) }
        forward!($($rest)*);
    };
    (fn $name:ident(&mut self $(, $arg:ident: $ty:ty)*) -> $ret:ty; $($rest:tt)*) => {
        fn $name(&mut self $(, $arg: $ty)*) -> $ret { self.inner.$name($($arg),*) }
        forward!($($rest)*);
    };
}
impl SerialPort for RefuseServoRequest {
    forward! {
        fn name(&self) -> Option<String>;
        fn baud_rate(&self) -> serialport::Result<u32>;
        fn data_bits(&self) -> serialport::Result<serialport::DataBits>;
        fn flow_control(&self) -> serialport::Result<serialport::FlowControl>;
        fn parity(&self) -> serialport::Result<serialport::Parity>;
        fn stop_bits(&self) -> serialport::Result<serialport::StopBits>;
        fn timeout(&self) -> Duration;
        fn set_baud_rate(&mut self, value: u32) -> serialport::Result<()>;
        fn set_data_bits(&mut self, value: serialport::DataBits) -> serialport::Result<()>;
        fn set_flow_control(&mut self, value: serialport::FlowControl) -> serialport::Result<()>;
        fn set_parity(&mut self, value: serialport::Parity) -> serialport::Result<()>;
        fn set_stop_bits(&mut self, value: serialport::StopBits) -> serialport::Result<()>;
        fn set_timeout(&mut self, value: Duration) -> serialport::Result<()>;
        fn write_request_to_send(&mut self, value: bool) -> serialport::Result<()>;
        fn write_data_terminal_ready(&mut self, value: bool) -> serialport::Result<()>;
        fn read_clear_to_send(&mut self) -> serialport::Result<bool>;
        fn read_data_set_ready(&mut self) -> serialport::Result<bool>;
        fn read_ring_indicator(&mut self) -> serialport::Result<bool>;
        fn read_carrier_detect(&mut self) -> serialport::Result<bool>;
        fn bytes_to_read(&self) -> serialport::Result<u32>;
        fn bytes_to_write(&self) -> serialport::Result<u32>;
        fn clear(&self, value: serialport::ClearBuffer) -> serialport::Result<()>;
        fn set_break(&self) -> serialport::Result<()>;
        fn clear_break(&self) -> serialport::Result<()>;
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Ok(Box::new(Self {
            inner: self.inner.try_clone()?,
        }))
    }
}

#[test]
fn an_unsent_servo_request_does_not_become_fifteen_missing_responses() {
    let (mut bus, _device) = peer(BusImuProtocol::Dxl2, Duration::ZERO, Duration::ZERO);
    bus.set_imu_protocol("dxl2").unwrap();
    bus.identify_imu().unwrap();
    bus.port = Box::new(RefuseServoRequest { inner: bus.port });
    assert!(bus.telemetry(&crate::model::JOINT_IDS).is_err());
    assert!(!bus.last_sync.request_sent);
    let mut totals = TelemetryTotals::default();
    totals.observe(&bus.last_sync, Some("injected write failure".into()));
    assert_eq!((totals.total_reads, totals.failed_reads), (1, 1));
    assert!(totals.missing_by_id.is_empty());
}
