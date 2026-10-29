// Modified 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; see IMU-MODULE.md at the repository root.
//! Offline PTY coverage for startup ordering, one-command-per-tick and fail-stop.
use super::feetech_commissioning::{dispatch, run};
use super::*;
use duck_control::model::JOINT_IDS;
use std::io::{Read, Write};
use std::os::fd::FromRawFd;

fn dxl_crc(bytes: &[u8]) -> u16 {
    let mut crc = 0u16;
    for byte in bytes {
        crc ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            crc = (crc << 1) ^ if crc & 0x8000 != 0 { 0x8005 } else { 0 };
        }
    }
    crc
}

fn dxl_status(payload: &[u8]) -> Vec<u8> {
    assert!(!payload.windows(3).any(|w| w == [255, 255, 253]));
    let mut raw = vec![255, 255, 253, 0, 200];
    raw.extend_from_slice(&((payload.len() + 4) as u16).to_le_bytes());
    raw.extend_from_slice(&[0x55, 0]);
    raw.extend_from_slice(payload);
    raw.extend_from_slice(&dxl_crc(&raw).to_le_bytes());
    raw
}

#[test]
fn r9_startup_cadence_and_motion_use_one_bus_without_recovery() {
    commissioning_flow(false, false);
}

#[test]
fn dxl2_commissioning_detects_samples_verifies_and_stops_on_a_servo_fault() {
    commissioning_flow(true, false);
}

#[test]
fn qinmo_commissioning_uses_explicit_profile_and_persists_both_mount_bindings() {
    commissioning_flow(false, true);
}

#[test]
fn startup_open_failure_publishes_servo_presence_without_imu_or_goal_writes() {
    let (mut master, mut slave) = (0, 0);
    let mut name = [0i8; 128];
    assert_eq!(unsafe { libc::openpty(&mut master, &mut slave, name.as_mut_ptr(), std::ptr::null(), std::ptr::null()) }, 0);
    let port = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }.to_str().unwrap().to_owned();
    let mut wire = unsafe { std::fs::File::from_raw_fd(master) };
    unsafe { libc::close(slave); }
    let done = Arc::new(AtomicBool::new(false));
    let stopped = done.clone();
    let writes = Arc::new(std::sync::Mutex::new(Vec::new()));
    let log = writes.clone();
    let device = std::thread::spawn(move || {
        use std::os::fd::AsRawFd;
        while !stopped.load(Ordering::Relaxed) {
            let mut fd = libc::pollfd { fd: wire.as_raw_fd(), events: libc::POLLIN, revents: 0 };
            if unsafe { libc::poll(&mut fd, 1, 10) } <= 0 { continue; }
            let mut header = [0; 4];
            if wire.read_exact(&mut header).is_err() { continue; }
            let mut body = vec![0; header[3] as usize];
            wire.read_exact(&mut body).unwrap();
            if body[0] == duck_control::feetech::SYNC_WRITE {
                log.lock().unwrap().push(body.clone());
            } else if body[0] == duck_control::feetech::SYNC_READ {
                assert_eq!(&body[1..3], &[40, 1]);
                for id in &body[3..body.len()-1] {
                    wire.write_all(&duck_control::feetech::packet(*id, 0, &[0]).unwrap()).unwrap();
                }
            } else {
                assert_eq!(header[2], 200); // The absent IMU never answers.
            }
        }
    });
    let dir = tempfile::tempdir().unwrap();
    let calibration = dir.path().join("calibration.toml");
    std::fs::write(&calibration, include_str!("../../packaging/hd1910-calibration.robot.toml")).unwrap();
    let bus = params::Bus {
        protocol: params::BusProtocol::Feetech,
        imu_protocol: params::ImuBusProtocol::Feetech,
        feetech_calibration: Some(calibration), port, ..Default::default()
    };
    let state = RobotState::new(&Params::default(), Path::new("/test/robotd.toml"), false, false);
    assert!(open_bus(&bus, None, 0, Some(&state)).is_none());
    done.store(true, Ordering::Relaxed);
    device.join().unwrap();
    let report = state.bus_status.load_full().unwrap();
    assert_eq!(report["phase"], "waiting_for_bus");
    assert_eq!(report["native"]["torque_state_confirmed"], false);
    assert_eq!(report["native"]["all_joints_fresh"], false);
    let rows = report["native"]["servo_observations"]["rows"].as_array().unwrap();
    assert_eq!(rows.len(), NUM_JOINTS);
    assert!(rows.iter().all(|row| row["responding"] == true && row["position_raw"].is_null()));
    let writes = writes.lock().unwrap();
    assert_eq!(writes.len(), 1);
    assert_eq!(&writes[0][1..3], &[40, 1]);
    assert!(writes[0][3..writes[0].len()-1].chunks_exact(2).all(|row| row[1] == 0));
    assert_eq!(state.ticks.load(Ordering::Relaxed), 0);
}

#[tokio::test]
async fn commissioning_startup_missing_imu_preserves_servo_presence_and_frozen_rpc_age() {
    // Factory-default commissioning must retain torque replies even when IMU discovery fails.
    let (mut master, mut slave) = (0, 0);
    let mut name = [0i8; 128];
    assert_eq!(unsafe {
        libc::openpty(&mut master, &mut slave, name.as_mut_ptr(), std::ptr::null(), std::ptr::null())
    }, 0);
    let port = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }.to_str().unwrap().to_owned();
    let mut wire = unsafe { std::fs::File::from_raw_fd(master) };
    unsafe { libc::close(slave); }
    let done = Arc::new(AtomicBool::new(false));
    let stopped = done.clone();
    let writes = Arc::new(std::sync::Mutex::new(Vec::new()));
    let log = writes.clone();
    let device = std::thread::spawn(move || {
        use std::os::fd::AsRawFd;
        while !stopped.load(Ordering::Relaxed) {
            let mut fd = libc::pollfd { fd: wire.as_raw_fd(), events: libc::POLLIN, revents: 0 };
            if unsafe { libc::poll(&mut fd, 1, 10) } <= 0 { continue; }
            let mut header = [0; 4];
            if wire.read_exact(&mut header).is_err() { continue; }
            let mut body = vec![0; header[3] as usize];
            wire.read_exact(&mut body).unwrap();
            if body[0] == duck_control::feetech::SYNC_WRITE {
                log.lock().unwrap().push(body.clone());
            } else if body[0] == duck_control::feetech::SYNC_READ {
                assert_eq!(&body[1..3], &[40, 1]);
                for id in &body[3..body.len()-1] {
                    wire.write_all(&duck_control::feetech::packet(*id, 0, &[0]).unwrap()).unwrap();
                }
            } else {
                assert_eq!(header[2], 200);
                assert_eq!(body[0], duck_control::feetech::READ);
            }
        }
    });
    let dir = tempfile::tempdir().unwrap();
    let calibration_path = dir.path().join("missing-calibration.toml");
    let state = Arc::new(RobotState::new(&Params::default(), Path::new("/test/robotd.toml"), false, false));
    state.commissioning.store(true, Ordering::Relaxed);
    let worker_state = state.clone();
    let worker = tokio::spawn(async move {
        run(&port, "feetech", "legacy", &worker_state, Duration::from_millis(20), &calibration_path, None).await;
    });
    tokio::time::timeout(Duration::from_secs(3), async {
        while state.startup_bus_failures.load(Ordering::Relaxed) == 0 {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await.unwrap();
    state.shutdown.store(true, Ordering::Relaxed);
    tokio::time::timeout(Duration::from_secs(2), worker).await.unwrap().unwrap();
    done.store(true, Ordering::Relaxed);
    device.join().unwrap();
    assert!(state.bus_snapshot.load().is_none(), "missing IMU must not enter commissioning ready");
    let query = || serde_json::json!({"jsonrpc":"2.0","id":1,"method":"robot.busStatus"});
    let first: serde_json::Value = serde_json::from_str(&tests::ipc_response(state.clone(), query()).await).unwrap();
    let report = &first["result"];
    assert_eq!(report["mode"], "commissioning");
    assert_eq!(report["phase"], "waiting_for_bus");
    assert_eq!(report["native"]["imu_source"], "bus");
    assert_eq!(report["native"]["configured_imu_profile"], "legacy");
    let observation = &report["native"]["servo_observations"];
    assert_eq!(observation["attempted"], true);
    let rows = observation["rows"].as_array().unwrap();
    assert_eq!(rows.len(), NUM_JOINTS);
    for (row, id) in rows.iter().zip(JOINT_IDS) {
        assert_eq!(row["id"], id);
        assert_eq!(row["responding"], true);
        assert!(row["check_age_ms"].is_u64() && row["reply_age_ms"].is_u64());
        for key in ["feedback_age_ms", "position_raw", "speed_raw", "load_raw", "current_raw", "volts", "temperature_c"] {
            assert!(row[key].is_null(), "torque presence cannot supply {key}");
        }
    }
    tokio::time::sleep(Duration::from_millis(40)).await;
    let second: serde_json::Value = serde_json::from_str(&tests::ipc_response(state.clone(), query()).await).unwrap();
    assert!(second["result"]["native"]["servo_observations"]["age_ms"].as_u64().unwrap()
        >= observation["age_ms"].as_u64().unwrap() + 30);
    assert_eq!(second["result"]["native"]["servo_observations"]["rows"], observation["rows"]);
    let writes = writes.lock().unwrap();
    assert_eq!(writes.len(), 1);
    assert_eq!(&writes[0][1..3], &[40, 1]);
    assert!(writes[0][3..writes[0].len()-1].chunks_exact(2).all(|row| row[1] == 0));
    assert_eq!(state.ticks.load(Ordering::Relaxed), 0);
}

fn commissioning_flow(dxl2: bool, qinmo: bool) {
    let mut master = 0;
    let mut slave = 0;
    let mut name = [0i8; 128];
    assert_eq!(
        unsafe {
            libc::openpty(
                &mut master,
                &mut slave,
                name.as_mut_ptr(),
                std::ptr::null(),
                std::ptr::null(),
            )
        },
        0
    );
    let port = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }
        .to_str()
        .unwrap()
        .to_owned();
    let mut wire = unsafe { std::fs::File::from_raw_fd(master) };
    unsafe {
        libc::close(slave);
    }

    let dir = std::env::temp_dir().join(format!(
        "microduck-r9-{}-{dxl2}-{qinmo}",
        std::process::id()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let calibration_path = dir.join("calibration.toml");
    let mut cal = duck_control::feetech::Calibration::commissioning(JOINT_IDS);
    if qinmo {
        cal.imu_mount_quat = [1.0, 0.0, 0.0, 0.0];
        cal.imu_mount_verified = true; // A persisted legacy verification is not the new binding.
    }
    let mut original = toml::to_string(&cal).unwrap();
    std::fs::write(&calibration_path, &original).unwrap();
    let state = Arc::new(RobotState::new(
        &Params::default(),
        std::path::Path::new("/tmp/unused-robotd.toml"),
        false,
        false,
    ));
    state.commissioning.store(true, Ordering::Relaxed);
    let started = Arc::new(AtomicBool::new(true));
    let corrupt = Arc::new(AtomicBool::new(false));
    let goals = Arc::new(AtomicU64::new(0));
    let exits = Arc::new(AtomicBool::new(false));
    let (device_started, device_corrupt, device_goals, device_exits) = (
        started.clone(),
        corrupt.clone(),
        goals.clone(),
        exits.clone(),
    );
    let device = std::thread::spawn(move || {
        use std::os::fd::AsRawFd;
        let mut pending = Vec::new();
        let mut rounds = 0u8;
        let mut initialization_reply = None;
        let mut writes_in_tick = 0;
        let mut pos = [2048u16; 256];
        let mut torque = [0u8; 256];
        while !device_exits.load(Ordering::Relaxed) {
            let mut fd = libc::pollfd {
                fd: wire.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            if unsafe { libc::poll(&mut fd, 1, 10) } <= 0 {
                continue;
            }
            let mut bytes = [0u8; 1024];
            let n = match wire.read(&mut bytes) {
                Ok(n) => n,
                Err(_) => continue,
            };
            if n == 0 {
                continue;
            }
            assert!(
                device_started.load(Ordering::Acquire),
                "simulated hardware was not started"
            );
            pending.extend_from_slice(&bytes[..n]);
            while pending.len() >= 4 {
                if pending[..4] == [255, 255, 253, 0] {
                    assert!(!qinmo, "Qinmo must not receive a DXL2 probe");
                    if pending.len() < 7 {
                        break;
                    }
                    let size = 7 + usize::from(u16::from_le_bytes([pending[5], pending[6]]));
                    if pending.len() < size {
                        break;
                    }
                    let packet = pending.drain(..size).collect::<Vec<_>>();
                    assert_eq!(
                        dxl_crc(&packet[..size - 2]),
                        u16::from_le_bytes([packet[size - 2], packet[size - 1]])
                    );
                    assert_eq!(
                        (packet[4], packet[7]),
                        (200, 2),
                        "only unicast reads may identify/sample the IMU"
                    );
                    if !dxl2 {
                        continue;
                    }
                    let address = u16::from_le_bytes([packet[8], packet[9]]);
                    let length = u16::from_le_bytes([packet[10], packet[11]]);
                    let data = match (address, length) {
                        (0, 3) => vec![0x44, 0x4d, 1],
                        (124, 12) => {
                            let mut data = vec![0; 12];
                            data[0] = rounds;
                            data[7] = 0x30;
                            data
                        }
                        _ => panic!("unexpected DXL2 read"),
                    };
                    wire.write_all(&dxl_status(&data)).unwrap();
                    continue;
                }
                let size = pending[3] as usize + 4;
                if pending.len() < size {
                    break;
                }
                let packet = pending.drain(..size).collect::<Vec<_>>();
                assert_eq!(&packet[..2], &[255, 255]);
                assert_eq!(packet[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b)), 255);
                if packet[2] == 200 && packet[4] == 1 {
                    assert!(qinmo);
                    wire.write_all(&duck_control::feetech::packet(200, 0, &[]).unwrap())
                        .unwrap();
                    continue;
                }
                let args = &packet[5..size - 1];
                let address = args[0];
                let length = args[1] as usize;
                if dxl2 && packet[2] == 200 {
                    continue;
                }
                if packet[4] == 0x83 {
                    for row in args[2..].chunks_exact(length + 1) {
                        assert!(JOINT_IDS.contains(&row[0]));
                        match address {
                            40 => torque[row[0] as usize] = row[1],
                            42 => {
                                // Outside an emergency stop, transitions and position
                                // writes may consume at most one additional transaction.
                                writes_in_tick += 1;
                                assert!(writes_in_tick <= 1);
                                pos[row[0] as usize] = u16::from_le_bytes([row[1], row[2]]);
                                device_goals.fetch_add(1, Ordering::Relaxed);
                            }
                            _ => panic!("unexpected write"),
                        }
                    }
                    continue;
                }
                assert!(packet[4] == 2 || packet[4] == 0x82);
                let ids = if packet[4] == 0x82 {
                    args[2..].to_vec()
                } else {
                    vec![packet[2]]
                };
                if address == 56 && packet[4] == 0x82 {
                    if rounds == 0 {
                        let gap = Instant::now().duration_since(
                            initialization_reply.expect("startup torque verification"),
                        );
                        assert!(
                            gap >= Duration::from_millis(18),
                            "first telemetry crowded startup: {gap:?}"
                        );
                    }
                    assert_eq!(packet[4], 0x82, "no unicast telemetry recovery");
                    assert_eq!(length, 15);
                    assert_eq!(
                        ids,
                        if dxl2 {
                            JOINT_IDS.to_vec()
                        } else if qinmo {
                            JOINT_IDS.into_iter().chain([200]).collect::<Vec<_>>()
                        } else {
                            std::iter::once(200).chain(JOINT_IDS).collect::<Vec<_>>()
                        }
                    );
                    rounds = rounds.wrapping_add(1);
                    writes_in_tick = 0;
                } else if rounds > 0 && packet[4] == 2 {
                    writes_in_tick += 1;
                    assert!(writes_in_tick <= 1, "multiple transition reads in one tick");
                }
                let fail = address == 56 && device_corrupt.swap(false, Ordering::AcqRel);
                let mut replies = Vec::new();
                for id in ids {
                    let mut data = vec![0u8; length];
                    if id == 200 {
                        if address == 0 {
                            assert!(!qinmo, "Qinmo has no documented identity READ");
                            data.copy_from_slice(&duck_control::feetech::IMU_IDENTITY);
                        } else {
                            if qinmo {
                                data[..2].copy_from_slice(&1640i16.to_le_bytes());
                            } else {
                                data[7] = 0x30;
                            }
                            data[12] = 250u8.wrapping_add(rounds);
                        }
                    } else {
                        match address {
                            56 => {
                                data[..2].copy_from_slice(&pos[id as usize].to_le_bytes());
                                data[6] = 64;
                                data[7] = 34;
                            }
                            40 => data[0] = torque[id as usize],
                            42 => data.copy_from_slice(&pos[id as usize].to_le_bytes()),
                            _ => panic!("unexpected read"),
                        }
                    }
                    let mut frame = vec![255, 255, id, length as u8 + 2, 0];
                    frame.extend(data);
                    let checksum = !frame[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
                    frame.push(if fail && id == 13 {
                        checksum ^ 128
                    } else {
                        checksum
                    });
                    replies.extend(frame);
                }
                wire.write_all(&replies).unwrap();
                if address == 40 && rounds == 0 {
                    initialization_reply = Some(Instant::now());
                }
            }
        }
    });
    let (control_state, path) = (state.clone(), calibration_path.clone());
    let control = std::thread::spawn(move || {
        tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .unwrap()
            .block_on(run(
                &port,
                "auto",
                if qinmo { "qinmo" } else { "legacy" },
                &control_state,
                Duration::from_millis(20),
                &path,
                None,
            ))
    });
    let wait = |predicate: &dyn Fn() -> bool| {
        let until = Instant::now() + Duration::from_secs(4);
        while !predicate() {
            assert!(
                Instant::now() < until,
                "condition timeout: goals={} status={:?}",
                goals.load(Ordering::Relaxed),
                state.bus_status.load_full()
            );
            std::thread::sleep(Duration::from_millis(5));
        }
    };
    wait(&|| state.ticks.load(Ordering::Relaxed) >= 8);
    if dxl2 || qinmo {
        wait(&|| state.imu_ready.load(Ordering::Relaxed));
        let report =
            serde_json::to_value(state.bus_snapshot.load_full().unwrap().as_ref()).unwrap();
        assert_eq!(
            report["imu_bus"]["protocol"],
            if qinmo { "feetech" } else { "dxl2" }
        );
        if qinmo {
            assert_eq!(report["imu_bus"]["profile"], "qinmo");
            assert_eq!(
                report["last_sample"]["imu"]["quaternion_wxyz"],
                serde_json::json!([1.0, 0.0, 0.0, 0.0])
            );
            assert!(
                (report["last_sample"]["imu"]["gyro_rad_s"][0]
                    .as_f64()
                    .unwrap()
                    .to_degrees()
                    - 100.0)
                    .abs()
                    < 1e-10
            );
            let status = state.bus_calibration.load_full().unwrap();
            assert_eq!(
                status["imu_mount"]["verified"], false,
                "legacy mount is effectively invalid"
            );
            assert!(
                status["imu_binding_validation"]
                    .as_str()
                    .unwrap()
                    .contains("profile")
            );
        } else {
            assert!(report["last_sample"]["imu"].get("sequence").is_none());
        }
        let reply = tokio::runtime::Runtime::new().unwrap().block_on(dispatch(
            &state,
            proto::Id::Number(80),
            serde_json::json!({"action":"verify-imu","confirmed":true,
                "mount_wxyz":if qinmo {vec![1.0,0.0,0.0,0.0]} else {vec![(1.0_f64-0.125_f64.powi(2)).sqrt(),0.125,0.0,0.0]} }),
        ));
        let reply = serde_json::to_value(reply).unwrap();
        assert!(reply.get("result").is_some(), "{reply}");
        original = std::fs::read_to_string(&calibration_path).unwrap();
        let saved: duck_control::feetech::Calibration = toml::from_str(&original).unwrap();
        assert_eq!(
            saved.imu_bus_protocol,
            Some(if qinmo {
                duck_control::feetech::BusImuProtocol::Feetech
            } else {
                duck_control::feetech::BusImuProtocol::Dxl2
            })
        );
        assert_eq!(
            saved.imu_bus_profile,
            Some(if qinmo {
                duck_control::feetech::BusImuProfile::Qinmo
            } else {
                duck_control::feetech::BusImuProfile::Legacy
            })
        );
        assert!(saved.imu_mount_verified);
        assert!(!saved.motion_enabled);
    }
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let (command_state, command_started) = (state.clone(), Arc::new(AtomicBool::new(false)));
    let entered = command_started.clone();
    let command = std::thread::spawn(move || {
        entered.store(true, Ordering::Relaxed);
        let response = runtime.block_on(dispatch(
            &command_state,
            proto::Id::Number(1),
            serde_json::json!({"action":"move","id":13,"position":2105,"duration_ms":1000}),
        ));
        response
    });
    wait(&|| command_started.load(Ordering::Relaxed) && goals.load(Ordering::Relaxed) >= 4);
    corrupt.store(true, Ordering::Release);
    // At a 20 ms transaction deadline the next tick can recover before the
    // observer wakes. Inspect the persisted failure, not a transient counter.
    wait(&|| {
        state.bus_snapshot.load_full().is_some_and(|snapshot| {
            serde_json::to_value(snapshot.as_ref()).unwrap()["errors_by_id"]["13"] == 1
        })
    });
    let failure = state.bus_snapshot.load_full().unwrap();
    let failure = serde_json::to_value(failure.as_ref()).unwrap();
    assert_eq!(failure["errors_by_id"]["13"], 1);
    assert!(!failure.as_object().unwrap().contains_key("diagnostics"));
    let reply = serde_json::to_value(command.join().unwrap()).unwrap();
    assert!(reply.get("error").is_some());
    let before = goals.load(Ordering::Relaxed);
    let tick = state.ticks.load(Ordering::Relaxed);
    wait(&|| state.ticks.load(Ordering::Relaxed) >= tick + 8);
    assert_eq!(goals.load(Ordering::Relaxed), before, "motion auto-resumed");
    let reply = tokio::runtime::Runtime::new().unwrap().block_on(dispatch(
        &state,
        proto::Id::Number(2),
        serde_json::json!({"action":"move","id":13,"position":2080,"duration_ms":500}),
    ));
    let reply = serde_json::to_value(reply).unwrap();
    assert_eq!(reply["result"]["torque"], "off");
    state.shutdown.store(true, Ordering::Relaxed);
    control.join().unwrap();
    exits.store(true, Ordering::Relaxed);
    device.join().unwrap();
    assert_eq!(
        std::fs::read_to_string(&calibration_path).unwrap(),
        original
    );
    std::fs::remove_dir_all(dir).unwrap();
}
