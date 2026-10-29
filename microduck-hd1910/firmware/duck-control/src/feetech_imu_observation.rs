//! Startup-only ID 200 evidence. Nothing here can establish control readiness or a mount.
use super::*;
use serde::{Serialize, Serializer};

#[derive(Debug, Clone)]
pub struct Snapshot {
    captured_at: Instant,
    pub responding: bool,
    pub info: Option<BusImuInfo>,
    pub sample_valid: Option<bool>,
    pub sensor_ready: Option<bool>,
    pub sample_fresh: Option<bool>,
    pub sequence: Option<u32>,
    pub error: Option<String>,
}

impl Serialize for Snapshot {
    fn serialize<S: Serializer>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error> {
        #[derive(Serialize)]
        struct Wire<'a> {
            source: &'static str,
            attempted: bool,
            responding: bool,
            age_ms: u64,
            info: &'a Option<BusImuInfo>,
            sample_valid: Option<bool>,
            sensor_ready: Option<bool>,
            sample_fresh: Option<bool>,
            sequence: Option<u32>,
            error: &'a Option<String>,
        }
        Wire {
            source: "bus",
            attempted: true,
            responding: self.responding,
            age_ms: self.captured_at.elapsed().as_millis() as u64,
            info: &self.info,
            sample_valid: self.sample_valid,
            sensor_ready: self.sensor_ready,
            sample_fresh: self.sample_fresh,
            sequence: self.sequence,
            error: &self.error,
        }
        .serialize(serializer)
    }
}

impl Bus {
    /// One bounded read-only capture through the existing startup owner. Discovery here
    /// never changes the control board/layout/identity or its accepted sample clock.
    pub fn observe_startup_imu(&mut self) -> Snapshot {
        let started = Instant::now();
        let deadline = started + Duration::from_millis(60);
        let old_trace = self.last_sync.clone();
        let mut observation = Snapshot {
            captured_at: started,
            responding: false,
            info: None,
            sample_valid: None,
            sensor_ready: None,
            sample_fresh: None,
            sequence: None,
            error: None,
        };
        let result = (|| -> Result<()> {
            let read = |bus: &mut Bus, address, length| {
                bus.exchange_budget(
                    IMU_ID,
                    READ,
                    &[address, length],
                    usize::from(length),
                    deadline
                        .saturating_duration_since(Instant::now())
                        .min(Duration::from_millis(20)),
                )
            };
            let (board, identity, raw) = if self.imu_profile == BusImuProfile::Qinmo {
                self.exchange_budget(
                    IMU_ID,
                    1,
                    &[],
                    0,
                    deadline
                        .saturating_duration_since(Instant::now())
                        .min(Duration::from_millis(20)),
                )?;
                (
                    BusImuBoard::Qinmo,
                    Vec::new(),
                    read(self, SYNC_ADDRESS, IMU_LENGTH),
                )
            } else if self.imu_protocol != BusImuProtocol::Dxl2 {
                match read(self, 0, 5) {
                    Ok(identity) => {
                        let board = BusImuBoard::from_identity(&identity)
                            .ok_or_else(|| unknown_imu(&identity))?;
                        (board, identity, read(self, SYNC_ADDRESS, IMU_LENGTH))
                    }
                    Err(problem) if self.imu_protocol == BusImuProtocol::Feetech => {
                        return Err(problem);
                    }
                    Err(_) => {
                        let identity = self.dxl2_read(0, 3, deadline)?;
                        (
                            BusImuBoard::Dxl2,
                            identity,
                            self.dxl2_read(124, 12, deadline),
                        )
                    }
                }
            } else {
                let identity = self.dxl2_read(0, 3, deadline)?;
                (
                    BusImuBoard::Dxl2,
                    identity,
                    self.dxl2_read(124, 12, deadline),
                )
            };
            // Legacy identity can be known before a bad sample. DXL2/Qinmo use a
            // compatibility/configuration contract, whose sample layout must decode.
            let info = BusImuInfo {
                protocol: board.protocol().label(),
                board: board.label(),
                identity,
                profile: self.imu_profile.label(),
                gyro_dps_per_lsb: self.imu_profile.gyro_dps_per_lsb(),
                identification: match board {
                    BusImuBoard::Qinmo => "configured_profile",
                    BusImuBoard::Dxl2 => "compatibility_layout",
                    _ => "legacy_identity",
                },
            };
            if matches!(board, BusImuBoard::Ft6 | BusImuBoard::ImuToDxl) {
                observation.info = Some(info.clone());
            }
            let raw = raw?;
            observation.sample_valid = Some(false);
            let sample = match board {
                BusImuBoard::Qinmo => ImuSample::decode_qinmo(&raw),
                BusImuBoard::Dxl2 => ImuSample::decode_dxl2(&raw),
                _ => ImuSample::decode(&raw),
            }?;
            observation.info = Some(info);
            observation.sample_valid = Some(sample.reading().quat.is_some());
            observation.sensor_ready = (board != BusImuBoard::Dxl2).then_some(sample.ready);
            observation.sequence = sample.sequence;
            // A single counter/quaternion cannot prove progress. Known stale board
            // status can disprove freshness without borrowing the control filter.
            observation.sample_fresh = (!sample.ready).then_some(false);
            if sample.reading().quat.is_none() {
                return Err(error("IMU diagnostic sample has an invalid quaternion"));
            }
            Ok(())
        })();
        observation.responding = self.last_imu_response_at.is_some_and(|at| at >= started);
        observation.error = result.err().map(|problem| problem.to_string());
        observation.captured_at = Instant::now();
        self.last_sync = old_trace;
        observation
    }
}
