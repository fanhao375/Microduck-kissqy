//! PTY specification model of published Qinmo protocol 1.2; not manufacturer firmware.
use super::*;
use serialport::SerialPort;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};

struct SpecificationModel {
    stop: Arc<AtomicBool>,
    requests: Arc<Mutex<Vec<Vec<u8>>>>,
    worker: Option<std::thread::JoinHandle<()>>,
}
impl Drop for SpecificationModel {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.worker.take().unwrap().join().unwrap();
    }
}

fn model(delay: Duration, answer: bool) -> (Bus, SpecificationModel) {
    let (mut device, mut host) = serialport::TTYPort::pair().unwrap();
    device.set_timeout(Duration::from_millis(2)).unwrap();
    host.set_timeout(Duration::from_millis(2)).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let requests = Arc::new(Mutex::new(Vec::new()));
    let (done, seen) = (stop.clone(), requests.clone());
    let worker = std::thread::spawn(move || {
        let mut pending = Vec::new();
        let mut byte = [0u8];
        let mut counter = 253u8;
        while !done.load(Ordering::Relaxed) {
            if device.read(&mut byte).unwrap_or(0) == 0 {
                continue;
            }
            pending.push(byte[0]);
            if pending.len() < 4 {
                continue;
            }
            assert_ne!(pending[2], 253, "Qinmo must never receive a DXL2 probe");
            if pending.len() < 4 + pending[3] as usize {
                continue;
            }
            let tx = std::mem::take(&mut pending);
            seen.lock().unwrap().push(tx.clone());
            if !answer {
                continue;
            }
            if tx[4] == 1 {
                assert_eq!(tx, packet(IMU_ID, 1, &[]).unwrap());
                let _ = device.write_all(&packet(IMU_ID, 0, &[]).unwrap());
                continue;
            }
            assert!(
                matches!(tx[4], READ | SYNC_READ),
                "read-only model received a write"
            );
            let (address, length) = (tx[5], tx[6]);
            assert_eq!(
                (address, length),
                (56, 15),
                "Qinmo has no assumed identity/diagnostic table"
            );
            let ids = if tx[4] == SYNC_READ {
                tx[7..tx.len() - 1].to_vec()
            } else {
                vec![tx[2]]
            };
            counter = counter.wrapping_add(1);
            let mut reply = Vec::new();
            for id in ids {
                let mut raw = [0u8; 15];
                if id == IMU_ID {
                    raw[..2].copy_from_slice(&1640i16.to_le_bytes());
                    raw[12] = counter;
                } else {
                    raw[..2].copy_from_slice(&2048u16.to_le_bytes());
                }
                reply.extend(packet(id, 0, &raw).unwrap());
            }
            if tx[4] == SYNC_READ {
                std::thread::sleep(delay);
            }
            let _ = device.write_all(&reply);
        }
    });
    let bus = Bus {
        port: Box::new(host),
        timeout: Duration::from_millis(30),
        read_only: true,
        last_sync: SyncTrace::default(),
        imu_board: BusImuBoard::Ft6,
        imu_protocol: BusImuProtocol::Auto,
        imu_profile: BusImuProfile::Legacy,
        imu_identity: None,
    };
    (
        bus,
        SpecificationModel {
            stop,
            requests,
            worker: Some(worker),
        },
    )
}

#[test]
fn qinmo_uses_ping_then_56_15_and_answers_after_fifteen_servos() {
    for protocol in ["auto", "feetech"] {
        let (mut bus, model) = model(Duration::ZERO, true);
        bus.set_imu_protocol(protocol).unwrap();
        bus.set_imu_profile("qinmo").unwrap();
        assert_eq!(bus.identify_imu().unwrap().label(), "qinmo");
        let info = bus.imu_info().unwrap();
        assert_eq!(info.protocol, "feetech");
        assert_eq!(info.profile, "qinmo");
        assert_eq!(info.identification, "configured_profile");
        assert!(info.identity.is_empty());
        assert_eq!(info.gyro_dps_per_lsb, 1.0 / 16.4);
        let ids = crate::model::JOINT_IDS;
        let blocks = bus.telemetry(&ids).unwrap();
        assert_eq!(blocks.len(), 16);
        assert_eq!(
            bus.decode_imu(&blocks[&IMU_ID]).unwrap().reading().gyro[0]
                .to_degrees()
                .round(),
            100.0
        );
        assert!(bus.imu().unwrap().age_upper_bound_ms.is_none());
        let requests = model.requests.lock().unwrap();
        assert_eq!(requests[0], packet(IMU_ID, 1, &[]).unwrap());
        assert_eq!(requests[1], packet(IMU_ID, READ, &[56, 15]).unwrap());
        assert_eq!(
            requests[2],
            sync_read_packet(56, 15, &[&ids[..], &[IMU_ID]].concat()).unwrap()
        );
        assert_eq!(requests[3], packet(IMU_ID, READ, &[56, 15]).unwrap());
    }
}

#[test]
fn qinmo_never_falls_back_and_rejects_dxl2_in_both_configuration_orders() {
    let (mut bus, specification) = model(Duration::ZERO, false);
    bus.set_imu_profile("qinmo").unwrap();
    assert!(bus.set_imu_protocol("dxl2").is_err());
    assert!(bus.identify_imu().is_err());
    assert!(bus.imu_info().is_none());
    assert_eq!(
        specification.requests.lock().unwrap().as_slice(),
        &[packet(IMU_ID, 1, &[]).unwrap()]
    );
    let (mut bus, _model) = model(Duration::ZERO, false);
    bus.set_imu_protocol("dxl2").unwrap();
    assert!(bus.set_imu_profile("qinmo").is_err());
    assert!(bus.set_imu_profile("guess").is_err());
}

#[test]
fn qinmo_and_fifteen_servos_still_have_one_twenty_ms_deadline() {
    let (mut bus, _model) = model(Duration::from_millis(25), true);
    bus.set_imu_profile("qinmo").unwrap();
    bus.identify_imu().unwrap();
    let started = Instant::now();
    assert!(bus.telemetry(&crate::model::JOINT_IDS).is_err());
    assert!(started.elapsed() < Duration::from_millis(40));
    assert!(bus.last_sync.deadline_exceeded);
}

#[test]
fn qinmo_native_motion_decodes_identity_and_keeps_legacy_mount_unverified() {
    let (mut bus, _model) = model(Duration::ZERO, true);
    bus.set_imu_profile("qinmo").unwrap();
    bus.identify_imu().unwrap();
    let mut cal = Calibration::commissioning(crate::model::JOINT_IDS);
    cal.imu_mount_quat = [1.0, 0.0, 0.0, 0.0];
    cal.imu_mount_verified = true;
    let mut io = FeetechIo::for_startup(bus, cal).unwrap();
    for _ in 0..25 {
        io.read().unwrap();
    }
    assert!(io.imu_ready());
    let sample = io.read().unwrap();
    assert_eq!(sample.imu.quat, [1.0, 0.0, 0.0, 0.0]);
    assert!((sample.imu.gyro[0].to_degrees() - 100.0).abs() < 1e-10);
    let diagnostics = io.feetech_diagnostics().unwrap();
    let report = serde_json::to_value(&diagnostics).unwrap();
    assert_eq!(report["configured_imu_profile"], "qinmo");
    assert!(!diagnostics.imu_mount_verified);
    assert!(diagnostics.imu_age_upper_bound_ms.is_none());
}
