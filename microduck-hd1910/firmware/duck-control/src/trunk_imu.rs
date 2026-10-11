//! Where the trunk IMU is wired, and how a sensor off the bus reaches the control loop.
//!
//! Chosen by configuration and never probed for:
//!
//! - [`TrunkImuKind::Bus`]: the FT6 board, ID 200 on the servo bus. Its block arrives in the
//!   servos' own Sync Read and [`crate::feetech`] judges its freshness from the board's status.
//! - A module on the board's own I²C — [`TrunkImuKind::Bno08x`] (BNO085/BNO086, codec in
//!   [`crate::bno08x`]) or [`TrunkImuKind::Lsm6dsv16x`] (the bus board's own chip on a
//!   breakout, codec in [`crate::lsm6dsv16x`]). Something outside this crate (robotd's reader
//!   thread, Linux only) owns the device, speaks the codec to it, and publishes the newest
//!   [`HostImuSample`] behind [`HostImu`]. The control thread takes one per tick and
//!   [`HostImuGate`] decides whether it is fit to use.
//!
//! All of them end in the same [`crate::imu::TrunkFrame`], so the policy cannot tell them
//! apart except through the mount each one was commissioned with.
//!
//! The control thread never waits on the reader: a slow or wedged I²C bus shows up as an old
//! sample and a refused tick, the same failure the bus board produces, rather than as a
//! control loop blocked inside a read.

use std::time::{Duration, Instant};

use serde::Serialize;

use crate::imu::SflpDecoder;
use crate::io::{ImuStale, IoError, Result};

/// The sources a trunk IMU can be read from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum TrunkImuKind {
    Bus,
    Bno08x,
    Lsm6dsv16x,
}

impl TrunkImuKind {
    pub const ALL: [Self; 3] = [Self::Bus, Self::Bno08x, Self::Lsm6dsv16x];

    /// The name configuration and diagnostics use for it.
    pub fn label(self) -> &'static str {
        match self {
            Self::Bus => "bus",
            Self::Bno08x => "bno08x",
            Self::Lsm6dsv16x => "lsm6dsv16x",
        }
    }

    /// The sensor→trunk rotation an uncommissioned robot starts from, scalar-first.
    pub fn default_mount(self) -> [f64; 4] {
        match self {
            Self::Bus => SflpDecoder::DEFAULT_MOUNT,
            Self::Bno08x => BNO08X_DEFAULT_MOUNT,
            // A breakout has no standard way up. Identity says "unknown" honestly: it is
            // only ever a starting point, and motion waits for a verified mount anyway.
            Self::Lsm6dsv16x => [1.0, 0.0, 0.0, 0.0],
        }
    }
}

/// A BNO08x module mounted sensor X left, Y up, Z forward: trunk gyro = sensor `(z, x, y)`.
///
/// A 120° turn about `(1, 1, 1)`. The mapping microduck-replica's policy trial ran on a robot;
/// still a default, not a measurement — motion waits for this source's own verified mount.
pub const BNO08X_DEFAULT_MOUNT: [f64; 4] = [0.5, 0.5, 0.5, 0.5];

/// The newest reading from a sensor the host reads itself, as the reader published it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HostImuSample {
    /// Fused orientation, sensor→world, scalar-first, `w ≥ 0`. Unit length up to the chip's
    /// own rounding — an LSM6DSV16X's half-precision components are passed on exactly as the
    /// bus board's are, and [`crate::imu::TrunkFrame`] normalises both the same way.
    pub quat: [f64; 4],
    /// Angular velocity in the sensor's axes, rad/s.
    pub gyro: [f64; 3],
    /// When the host received the *older* of the orientation and the rate in this sample, so
    /// one age covers both: a fresh quaternion cannot carry a stale gyro past the gate.
    pub received_at: Instant,
    /// Host-assigned, one step per new orientation. A repeat means nothing new arrived.
    pub sequence: u32,
    /// One step each time the reader (re)starts the chip's reports — a reopened device or a
    /// chip that reset itself. The fusion starts over there.
    pub epoch: u32,
    /// When that epoch began.
    pub epoch_started_at: Instant,
    /// The chip's own estimate of the orientation's accuracy, where it makes one: a BNO08x
    /// report's status bits, 0 unreliable … 3 high. For the operator; nothing gates on it.
    pub accuracy: Option<u8>,
}

/// What the reader is doing, for diagnostics. Its vocabulary, not the gate's verdict.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct HostImuHealth {
    /// One word for the reader's state — `"streaming"`, `"reconnecting"` and so on.
    pub state: &'static str,
    pub epoch: u32,
    /// Samples published since the reader started.
    pub samples: u64,
    /// Faults since the reader started: each one a reopen of the device, or a chip set up
    /// again in place after it reset itself or lost data.
    pub errors: u64,
    pub last_error: Option<String>,
}

/// A trunk IMU the host reads on a thread of its own.
///
/// `Send` because the I/O owning it moves to the control thread, `Sync` because the commissioning
/// and read-only services borrow it across their own awaits. Implementations hand out a copy of
/// the newest sample and never block: the reader thread writes a slot, this reads it.
pub trait HostImu: Send + Sync {
    fn kind(&self) -> TrunkImuKind;
    /// The newest sample, or `None` before the first one.
    fn latest(&self) -> Option<HostImuSample>;
    fn health(&self) -> HostImuHealth;
}

/// Oldest sample a tick may use. The bus board's bound too: status age plus transaction time
/// must stay within 50 ms (`feetech::ImuSample::is_fresh`).
pub const HOST_IMU_MAX_AGE: Duration = Duration::from_millis(50);
/// How long after an epoch starts its samples are refused. A restarted fusion reports an
/// orientation before it has settled on one, and gravity is what fall detection trusts.
pub const HOST_IMU_SETTLE: Duration = Duration::from_secs(1);
/// Consecutive repeats of one sequence that make a stream frozen — the bus board's rule.
pub const HOST_IMU_REPEATS: u64 = 3;

/// Decides, once per tick, whether the reader's newest sample may drive the robot.
///
/// Every refusal is an error with the bus board's prefix, `Feetech IMU not ready or stale`, so
/// the control loop and anything matching on the text treat both sources' IMU faults alike.
#[derive(Debug, Default)]
pub struct HostImuGate {
    epoch: Option<u32>,
    last_sequence: Option<u32>,
    stale: ImuStale,
    last_age: Option<Duration>,
}

fn refused(why: String) -> IoError {
    IoError::Bus(format!("Feetech IMU not ready or stale ({why})"))
}

impl HostImuGate {
    pub fn check(&mut self, sample: Option<HostImuSample>, now: Instant) -> Result<HostImuSample> {
        let Some(sample) = sample else {
            return Err(refused("host IMU has published no sample yet".into()));
        };
        if self.epoch != Some(sample.epoch) {
            // A new epoch's sequence numbers say nothing about the old one's; a run carried
            // across the restart would count a fresh stream as frozen.
            self.epoch = Some(sample.epoch);
            self.last_sequence = None;
            self.stale.run = 0;
        }
        if self.last_sequence == Some(sample.sequence) {
            self.stale.total = self.stale.total.saturating_add(1);
            self.stale.run = self.stale.run.saturating_add(1);
        } else {
            self.stale.run = 0;
        }
        self.last_sequence = Some(sample.sequence);
        let age = now.saturating_duration_since(sample.received_at);
        self.last_age = Some(age);
        if self.stale.run >= HOST_IMU_REPEATS {
            return Err(refused(format!(
                "host IMU sequence {} repeated {} reads",
                sample.sequence, self.stale.run
            )));
        }
        if age > HOST_IMU_MAX_AGE {
            return Err(refused(format!(
                "host IMU sample {} ms old, limit {} ms",
                age.as_millis(),
                HOST_IMU_MAX_AGE.as_millis()
            )));
        }
        let settled = now.saturating_duration_since(sample.epoch_started_at);
        if settled < HOST_IMU_SETTLE {
            return Err(refused(format!(
                "host IMU epoch {} settling, {} of {} ms",
                sample.epoch,
                settled.as_millis(),
                HOST_IMU_SETTLE.as_millis()
            )));
        }
        Ok(sample)
    }

    /// Repeated sequences, the same two numbers the bus board reports.
    pub fn stale(&self) -> ImuStale {
        self.stale
    }

    /// Sequence of the last sample checked, accepted or not.
    pub fn sequence(&self) -> Option<u32> {
        self.last_sequence
    }

    /// Age of the last sample checked, at the moment it was checked.
    pub fn last_age(&self) -> Option<Duration> {
        self.last_age
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::{SensorReading, TrunkFrame};

    fn sample(sequence: u32, received_at: Instant) -> HostImuSample {
        HostImuSample {
            quat: [1.0, 0.0, 0.0, 0.0],
            gyro: [0.0; 3],
            received_at,
            sequence,
            epoch: 1,
            epoch_started_at: received_at - Duration::from_secs(5),
            accuracy: None,
        }
    }

    fn close(actual: [f64; 3], expected: [f64; 3]) -> bool {
        actual
            .iter()
            .zip(expected)
            .all(|(a, e)| (a - e).abs() < 1e-12)
    }

    /// The module's default mount, pinned against the mapping microduck-replica's policy trial
    /// ran on a robot (`tools/servo-web/test_policy_trial.py`, quaternions there being
    /// `xyzw`): trunk gyro = sensor `(z, x, y)`, the upright reading is the mount itself, and
    /// a pitch about trunk +Y tips gravity towards +X. A sign or axis slip here walks the robot
    /// on a mirrored horizon.
    #[test]
    fn the_bno08x_default_mount_is_the_mapping_the_robot_ran() {
        let mut frame = TrunkFrame::new(TrunkImuKind::Bno08x.default_mount());
        let upright = SensorReading {
            gyro: [
                1.0_f64.to_radians(),
                2.0_f64.to_radians(),
                3.0_f64.to_radians(),
            ],
            quat: Some([0.5, 0.5, 0.5, 0.5]),
        };
        let mut out = frame.push(upright);
        for _ in 0..2 {
            out = frame.push(upright);
        }
        let want = [
            3.0_f64.to_radians(),
            1.0_f64.to_radians(),
            2.0_f64.to_radians(),
        ];
        assert!(close(out.gyro, want), "gyro {:?}", out.gyro);
        assert!(
            close(out.gravity, [0.0, 0.0, -1.0]),
            "gravity {:?}",
            out.gravity
        );

        let t = std::f64::consts::PI / 12.0;
        let (c, v) = ((t / 2.0).cos(), (t / 2.0).sin());
        let pitched = SensorReading {
            gyro: [0.0; 3],
            quat: Some([0.5 * (c - v), 0.5 * (c + v), 0.5 * (c + v), 0.5 * (c - v)]),
        };
        for _ in 0..3 {
            out = frame.push(pitched);
        }
        assert!(
            close(out.gravity, [t.sin(), 0.0, -t.cos()]),
            "gravity {:?}",
            out.gravity
        );
    }

    /// The bus board keeps its own default, unchanged by the second source.
    #[test]
    fn each_source_has_its_own_default_mount_and_label() {
        assert_eq!(
            TrunkImuKind::Bus.default_mount(),
            SflpDecoder::DEFAULT_MOUNT
        );
        assert_eq!(TrunkImuKind::Bno08x.default_mount(), BNO08X_DEFAULT_MOUNT);
        assert_eq!(
            TrunkImuKind::Lsm6dsv16x.default_mount(),
            [1.0, 0.0, 0.0, 0.0]
        );
        let labels: Vec<_> = TrunkImuKind::ALL.iter().map(|k| k.label()).collect();
        assert_eq!(labels, ["bus", "bno08x", "lsm6dsv16x"]);
        for kind in TrunkImuKind::ALL {
            assert_eq!(
                serde_json::to_value(kind).unwrap(),
                serde_json::json!(kind.label())
            );
        }
    }

    #[test]
    fn a_fresh_settled_sample_passes_unchanged() {
        let now = Instant::now();
        let mut gate = HostImuGate::default();
        let fresh = sample(7, now - Duration::from_millis(5));
        assert_eq!(gate.check(Some(fresh), now).unwrap(), fresh);
        assert_eq!(gate.sequence(), Some(7));
        assert_eq!(gate.last_age(), Some(Duration::from_millis(5)));
    }

    /// Every refusal carries the bus board's text, which is what the loop and the console
    /// already recognise as an IMU fault rather than a lost servo.
    #[test]
    fn every_refusal_reads_as_the_bus_boards_imu_fault() {
        let now = Instant::now();
        let mut gate = HostImuGate::default();
        let mut settling = sample(1, now);
        settling.epoch_started_at = now - Duration::from_millis(400);
        for (case, result) in [
            ("none", gate.check(None, now)),
            (
                "old",
                gate.check(Some(sample(2, now - Duration::from_millis(51))), now),
            ),
            (
                "settling",
                HostImuGate::default().check(Some(settling), now),
            ),
        ] {
            let error = result.unwrap_err().to_string();
            assert!(
                error.contains("Feetech IMU not ready or stale (host IMU"),
                "{case}: {error}"
            );
        }
    }

    /// 50 ms is the limit, inclusive, as for the bus board.
    #[test]
    fn age_is_judged_against_fifty_milliseconds() {
        let now = Instant::now();
        let mut gate = HostImuGate::default();
        assert!(
            gate.check(Some(sample(1, now - HOST_IMU_MAX_AGE)), now)
                .is_ok()
        );
        let error = gate
            .check(
                Some(sample(2, now - HOST_IMU_MAX_AGE - Duration::from_millis(1))),
                now,
            )
            .unwrap_err()
            .to_string();
        assert!(error.contains("51 ms old"), "{error}");
    }

    /// A reader that keeps publishing one sample is a frozen fusion. Same count as the bus
    /// board: the fourth identical read is refused, and any new sequence clears the run.
    #[test]
    fn three_repeats_of_one_sequence_are_a_frozen_stream() {
        let now = Instant::now();
        let mut gate = HostImuGate::default();
        let same = sample(9, now);
        for repeat in 0..3 {
            assert!(gate.check(Some(same), now).is_ok(), "repeat {repeat}");
        }
        let error = gate.check(Some(same), now).unwrap_err().to_string();
        assert!(error.contains("repeated 3 reads"), "{error}");
        assert_eq!(gate.stale(), ImuStale { total: 3, run: 3 });
        assert!(gate.check(Some(sample(10, now)), now).is_ok());
        assert_eq!(gate.stale(), ImuStale { total: 3, run: 0 });
    }

    /// A restarted reader begins its sequence again. Matching the old epoch's last number is
    /// not a repeat, and must not be counted as one.
    #[test]
    fn a_new_epoch_resets_the_repeat_run_but_not_the_total() {
        let now = Instant::now();
        let mut gate = HostImuGate::default();
        let old = sample(4, now);
        for _ in 0..3 {
            gate.check(Some(old), now).unwrap();
        }
        assert_eq!(gate.stale().run, 2);
        let restarted = HostImuSample { epoch: 2, ..old };
        assert!(gate.check(Some(restarted), now).is_ok());
        assert_eq!(gate.stale(), ImuStale { total: 2, run: 0 });
    }

    /// A fusion that has just restarted reports an orientation before it has one. For a second
    /// after its epoch begins, nothing it says may reach the policy or the fall detector.
    #[test]
    fn a_new_epoch_is_refused_until_it_has_settled() {
        let now = Instant::now();
        let mut gate = HostImuGate::default();
        let mut young = sample(1, now);
        young.epoch_started_at = now - HOST_IMU_SETTLE + Duration::from_millis(1);
        let error = gate.check(Some(young), now).unwrap_err().to_string();
        assert!(error.contains("epoch 1 settling"), "{error}");
        young.epoch_started_at = now - HOST_IMU_SETTLE;
        young.sequence = 2;
        assert!(gate.check(Some(young), now).is_ok());
    }
}
