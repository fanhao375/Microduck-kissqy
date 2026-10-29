// Modified 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; BNO08x gyro-integrated attitude; see IMU-MODULE.md at the repository root.
//! Commission the real Feetech bus through robotd's socket. The control thread is
//! the sole UART owner; raw positions never masquerade as calibrated joint angles.
use super::{Ordering, RobotState};
use clap::Subcommand;
use duck_control::feetech::ImuMount;
use duck_control::feetech::{
    Bus, BusImuFreshness, BusImuInfo, Calibration, IMU_ID, ServoSample, signed_position_word,
};
use duck_control::imu::{ImuData, SensorReading, TrunkFrame};
use duck_control::model::{JOINT_IDS, NUM_JOINTS};
use duck_control::trunk_imu::{HostImu, HostImuGate, TrunkImuKind};
use duck_ipc_proto as proto;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::{BTreeMap, VecDeque};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::sync::oneshot;

pub const BUILD: &str = "0791fcd-feetech-qinmo-control.46";
const MAX_PENDING: usize = 16;
const POSITION_TOLERANCE: u16 = 8;
const SETTLE_TIMEOUT: Duration = Duration::from_secs(4);
type Reply = Result<Value, String>;

#[derive(Debug, Clone, Subcommand, Serialize, Deserialize)]
#[serde(tag = "action", rename_all = "kebab-case", deny_unknown_fields)]
pub enum BusCommand {
    /// Immediately interrupt commissioning movement and cut torque on all joints.
    Relax,
    /// Confirm torque-off, then reboot this Zero after returning the RPC reply.
    Reboot {},
    /// Confirm torque-off, then power off this Zero after returning the RPC reply.
    Shutdown {},
    /// Move one identified servo in encoder counts, then release its torque.
    Move {
        #[arg(long)]
        id: u8,
        #[arg(long)]
        position: u16,
        #[arg(long, default_value_t = 2000)]
        duration_ms: u64,
    },
    /// Sequentially center loose, unassembled servos. Never a joint calibration.
    Center {
        #[arg(long, required = true)]
        unassembled: bool,
        #[arg(long, default_value_t = 2000)]
        duration_ms: u64,
    },
    /// Record a known physical joint angle using its current encoder position.
    Capture {
        #[arg(long)]
        id: u8,
        #[arg(long, allow_hyphen_values = true)]
        angle_deg: f64,
        #[arg(long, allow_hyphen_values = true)]
        direction: i8,
        #[arg(long, allow_hyphen_values = true)]
        min_deg: f64,
        #[arg(long, allow_hyphen_values = true)]
        max_deg: f64,
    },
    /// Show the persisted calibration and what still prevents full control.
    CalibrationStatus,
    /// Record that the official IMU mounting axes have been physically checked — for the
    /// trunk IMU this robotd reads (`[trunk_imu] source`), in that source's own slot. For the
    /// bus board that check is the operator's word. For a module, start with the robot upright
    /// (within 5° under the mount being confirmed), then tip it nose-down by 15–30° within 20 s;
    /// the command answers once it has seen the tip.
    VerifyImu {
        #[arg(long, required = true)]
        confirmed: bool,
        /// Sensor-to-trunk quaternion, scalar first. Omit to retain configured mounting.
        #[arg(long, value_delimiter = ',', num_args = 4, allow_hyphen_values = true)]
        #[serde(default)]
        mount_wxyz: Vec<f64>,
    },
    /// Ramp all fifteen joints together to the HOME pose and keep them there, torque on, so
    /// the robot can stand for a standing IMU check without leaving commissioning. Holds until
    /// `relax` (robot.relax / robot.stop), a sustained bus failure or the service stopping.
    /// The zero is each joint's saved `zero_raw`; `directions` (one -1/+1 per joint, in the
    /// calibration's joint order) stands in for the saved ones, which on a fresh image are
    /// still placeholders. Omit it to use the saved directions.
    HoldHome {
        #[arg(long, value_delimiter = ',', allow_hyphen_values = true)]
        #[serde(default)]
        directions: Vec<i8>,
        #[arg(long, default_value_t = HOME_HOLD_RAMP_MS)]
        #[serde(default = "home_hold_ramp_ms")]
        duration_ms: u64,
    },
}

/// The same two seconds motion's ramp to HOME (`HOME_RAMP` in main.rs) takes, plus one: here
/// nothing balances, so a robot coming up from limp is given a little longer.
const HOME_HOLD_RAMP_MS: u64 = 3000;
fn home_hold_ramp_ms() -> u64 {
    HOME_HOLD_RAMP_MS
}

fn command_timeout(command: &BusCommand) -> Duration {
    match command {
        BusCommand::Move { duration_ms, .. } => {
            Duration::from_millis((*duration_ms).min(60_000)) + Duration::from_secs(15)
        }
        BusCommand::Center { duration_ms, .. } => {
            Duration::from_millis((*duration_ms).min(60_000) * NUM_JOINTS as u64)
                + Duration::from_secs(8 * NUM_JOINTS as u64 + 15)
        }
        BusCommand::VerifyImu { .. } => NOSE_DOWN_WINDOW + Duration::from_secs(10),
        BusCommand::HoldHome { duration_ms, .. } => {
            Duration::from_millis((*duration_ms).min(60_000)) + HOLD_SETTLE + Duration::from_secs(10)
        }
        _ => Duration::from_secs(15),
    }
}

struct Job {
    command: BusCommand,
    reply: oneshot::Sender<Reply>,
}

#[derive(Default)]
pub struct Commands {
    pending: Mutex<VecDeque<Job>>,
    power_pending: AtomicBool,
}
impl Commands {
    fn push(&self, job: Job) -> Result<(), String> {
        let mut queue = self
            .pending
            .lock()
            .map_err(|_| "bus command queue unavailable")?;
        if self.power_pending.load(Ordering::Relaxed)
            && !matches!(
                job.command,
                BusCommand::Relax | BusCommand::CalibrationStatus
            )
        {
            return Err("Zero power operation pending; new movement refused".into());
        }
        // Relax remains available even when the queue is full and takes priority.
        if matches!(job.command, BusCommand::Relax) {
            while let Some(old) = queue.pop_front() {
                let _ = old.reply.send(Err("cancelled by relax".into()));
            }
            queue.push_front(job);
        } else if queue.len() >= MAX_PENDING {
            return Err("bus command queue full".into());
        } else {
            queue.push_back(job);
        }
        Ok(())
    }
    fn pop(&self) -> Option<Job> {
        self.pending.lock().ok()?.pop_front()
    }
    fn cancel_all(&self, reason: &str) {
        if let Ok(mut queue) = self.pending.lock() {
            for job in queue.drain(..) {
                let _ = job.reply.send(Err(reason.to_owned()));
            }
        }
    }
}

pub async fn dispatch(state: &RobotState, id: proto::Id, params: Value) -> proto::Response {
    let fail = |message: String| {
        proto::Response::err(
            Some(id.clone()),
            proto::Error::new(proto::code::INVALID_REQUEST, message),
        )
    };
    let command: BusCommand = match serde_json::from_value(params) {
        Ok(command) => command,
        Err(e) => return fail(format!("invalid bus command: {e}")),
    };
    if matches!(command, BusCommand::CalibrationStatus) {
        return match state.bus_calibration.load_full() {
            Some(calibration) => proto::Response::ok(Some(id), calibration.as_ref()),
            None => fail("calibration not available yet".into()),
        };
    }
    if !state.commissioning.load(Ordering::Relaxed) {
        return fail("raw bus commands require the running robotd commissioning mode".into());
    }
    let power = match command {
        BusCommand::Reboot {} => Some("reboot"),
        BusCommand::Shutdown {} => Some("poweroff"),
        _ => None,
    };
    if power.is_some()
        && state
            .bus_commands
            .power_pending
            .swap(true, Ordering::Relaxed)
    {
        return fail("a Zero power operation is already pending".into());
    }
    let command = if power.is_some() {
        BusCommand::Relax
    } else {
        command
    };
    let timeout = command_timeout(&command);
    let (reply, receive) = oneshot::channel();
    if let Err(error) = state.bus_commands.push(Job { command, reply }) {
        if power.is_some() {
            state
                .bus_commands
                .power_pending
                .store(false, Ordering::Relaxed);
        }
        return fail(error);
    }
    let mut result = match tokio::time::timeout(timeout, receive).await {
        Ok(Ok(value)) => value,
        Ok(Err(_)) => Err("bus command cancelled because its controller stopped".into()),
        Err(_) => Err("bus command timed out; check robot.busStatus".into()),
    };
    if let Some(operation) = power {
        if result.is_ok() {
            // Existing robotd runs as root on this Zero. Do not change sudoers or
            // collect passwords in the web app. Only these two fixed operations.
            result = match std::process::Command::new("/usr/bin/systemd-run")
                .args([
                    "--quiet",
                    "--collect",
                    "--on-active=3s",
                    "--timer-property=AccuracySec=1s",
                    "/usr/bin/systemctl",
                    operation,
                ])
                .output()
            {
                Ok(output) if output.status.success() => Ok(
                    json!({"power_action":operation,"scheduled":true,"delay_seconds":3,"torque":"off"}),
                ),
                Ok(output) => Err(format!(
                    "Zero power request failed: {}",
                    String::from_utf8_lossy(&output.stderr)
                )),
                Err(error) => Err(format!("cannot schedule Zero power request: {error}")),
            };
        }
        if result.is_err() {
            state
                .bus_commands
                .power_pending
                .store(false, Ordering::Relaxed);
        }
    }
    match result {
        Ok(value) => proto::Response::ok(Some(id), &value),
        Err(error) => fail(error),
    }
}

pub async fn cli(socket: &Path, command: &BusCommand) -> ExitCode {
    let run = async {
        let stream = tokio::net::UnixStream::connect(socket).await?;
        let (read, mut write) = stream.into_split();
        let request =
            json!({"jsonrpc":"2.0", "id":1, "method":"robot.busCommand", "params":command});
        let mut encoded = serde_json::to_vec(&request).map_err(std::io::Error::other)?;
        encoded.push(b'\n');
        write.write_all(&encoded).await?;
        let mut lines = BufReader::new(read).lines();
        let line = tokio::time::timeout(
            command_timeout(command) + Duration::from_secs(5),
            lines.next_line(),
        )
        .await
        .map_err(|_| std::io::Error::other("robotd command timed out"))??
        .ok_or_else(|| std::io::Error::other("robotd closed the connection"))?;
        let value: Value = serde_json::from_str(&line).map_err(std::io::Error::other)?;
        if let Some(error) = value.get("error") {
            return Err(std::io::Error::other(error.to_string()));
        }
        println!(
            "{}",
            serde_json::to_string_pretty(&value["result"]).map_err(std::io::Error::other)?
        );
        Ok::<(), std::io::Error>(())
    };
    match run.await {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn read_calibration(path: &Path) -> Result<Calibration, String> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            let calibration: Calibration =
                toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
            calibration.validate().map_err(|e| e.to_string())?;
            Ok(calibration)
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            Ok(Calibration::commissioning(JOINT_IDS))
        }
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

fn save_calibration(path: &Path, calibration: &Calibration) -> Result<(), String> {
    calibration.validate().map_err(|e| e.to_string())?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    let encoded = toml::to_string_pretty(calibration).map_err(|e| e.to_string())?;
    if let Ok(previous) = std::fs::read(path) {
        if previous == encoded.as_bytes() { return Ok(()); }
        let backup = path.with_extension("previous.toml");
        let temporary = backup.with_extension(format!("toml.tmp.{}",std::process::id()));
        std::fs::write(&temporary, &previous).map_err(|e| e.to_string())?;
        std::fs::File::open(&temporary).and_then(|f| f.sync_all()).map_err(|e| e.to_string())?;
        std::fs::rename(&temporary,&backup).map_err(|e| e.to_string())?;
    }
    for target in [path.to_path_buf(), path.with_extension("saved.toml")] {
        let temporary: PathBuf = target.with_extension(format!("toml.tmp.{}", std::process::id()));
        let result = (|| -> std::io::Result<()> {
            use std::io::Write;
            let mut file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&temporary)?;
            file.write_all(encoded.as_bytes())?;
            file.sync_all()?;
            std::fs::rename(&temporary, &target)?;
            if let Some(parent) = target.parent() {
                std::fs::File::open(parent)?.sync_all()?;
            }
            Ok(())
        })();
        if result.is_err() {
            let _ = std::fs::remove_file(&temporary);
        }
        result.map_err(|e| format!("cannot save {}: {e}", target.display()))?;
    }
    Ok(())
}

/// `source`: the trunk IMU this robotd reads, whose mount decides whether motion may start.
///
/// For a module it also carries `imu_mount`, that source's mount as robotd uses it — its own
/// slot, or the source's default while the slot is absent — so the console's IMU panel shows and
/// confirms the mount in use rather than the bus board's top-level keys. The bus board's own
/// report is unchanged: its mount already is those keys.
pub(crate) fn calibration_status(
    calibration: &Calibration,
    path: &Path,
    source: TrunkImuKind,
) -> Value {
    let missing = calibration
        .joints
        .iter()
        .filter(|joint| !joint.calibrated)
        .map(|joint| joint.name.as_str())
        .collect::<Vec<_>>();
    let mut status = json!({"calibration_path":path, "calibration":calibration,
        "uncalibrated_joints":missing, "imu_source": source.label(),
        "motion_validation":calibration.validate_motion_for(source).err().map(|e| e.to_string()),
        "note":"Capture uses a manually known joint angle; encoder center is not joint zero."});
    let mount = calibration.imu_mount(source);
    // A persisted bus verification is not effective until the discovered wire format and
    // configured data profile have both been checked. I2C has its independent mount slot.
    let verified = source != TrunkImuKind::Bus && mount.verified;
    status["imu_mount"] = json!({"quat": mount.quat, "verified": verified});
    status["imu_mount_verified"] = json!(verified);
    if source == TrunkImuKind::Bus {
        let pending = "IMU bus discovery has not established a binding";
        status["imu_binding_validation"] = json!(pending);
        status["motion_validation"] = json!(pending);
    }
    status
}

pub(crate) fn calibration_status_for_bus(
    calibration: &Calibration,
    path: &Path,
    source: TrunkImuKind,
    bus: &Bus,
) -> Value {
    let mut status = calibration_status(calibration, path, source);
    status["configured_imu_profile"] = json!(bus.imu_profile().label());
    if source == TrunkImuKind::Bus {
        let binding = if bus.imu_info().is_none() {
            Some("IMU bus discovery has not established a binding".to_owned())
        } else {
            calibration
                .validate_bus_mount_for(bus.imu_board().protocol(), bus.imu_profile())
                .err()
                .map(|e| e.to_string())
        };
        let verified = calibration.imu_mount_verified && binding.is_none();
        status["imu_mount"]["verified"] = json!(verified);
        status["imu_mount_verified"] = json!(verified);
        status["imu_bus"] = json!(bus.imu_info());
        status["imu_binding_validation"] = json!(binding);
        status["motion_validation"] = json!(binding.or_else(|| {
            calibration
                .validate_motion_for(source)
                .err()
                .map(|e| e.to_string())
        }));
    }
    status
}

fn capture(
    calibration: &Calibration,
    samples: &BTreeMap<u8, ServoSample>,
    id: u8,
    angle_deg: f64,
    direction: i8,
    min_deg: f64,
    max_deg: f64,
) -> Result<Calibration, String> {
    if ![-1, 1].contains(&direction)
        || ![angle_deg, min_deg, max_deg].iter().all(|v| v.is_finite())
        || min_deg >= max_deg
        || !(min_deg..=max_deg).contains(&angle_deg)
    {
        return Err(
            "capture needs direction -1/+1 and finite min < max containing the measured angle"
                .into(),
        );
    }
    let sample = samples
        .get(&id)
        .ok_or_else(|| format!("ID {id} is not a configured joint"))?;
    let zero =
        f64::from(sample.feedback.position_raw) - f64::from(direction) * angle_deg / 360.0 * 4096.0;
    if !(0.0..=4095.0).contains(&zero) {
        return Err("calculated zero is outside 0..4095; check angle and direction".into());
    }
    let mut updated = calibration.clone();
    let joint = updated
        .joints
        .iter_mut()
        .find(|joint| joint.id == id)
        .ok_or("unknown joint ID")?;
    joint.zero_raw = zero.round() as u16;
    joint.direction = direction;
    joint.min_rad = Some(min_deg.to_radians());
    joint.max_rad = Some(max_deg.to_radians());
    joint.calibrated = true;
    updated.motion_enabled = false;
    // Verify both physical travel endpoints before allowing this calibration to persist.
    for angle in [min_deg, max_deg] {
        let raw = f64::from(joint.zero_raw) + f64::from(direction) * angle / 360.0 * 4096.0;
        if !(0.0..=4095.0).contains(&raw) {
            return Err("joint limits cross the single-turn encoder boundary".into());
        }
    }
    updated.validate().map_err(|e| e.to_string())?;
    Ok(updated)
}

#[derive(Clone, Copy)]
enum MotionStage {
    Preload,
    ConfirmGoal,
    Enable,
    ConfirmEnabled,
    Moving,
    Disable,
    ConfirmDisabled,
}

struct Motion {
    remaining: VecDeque<u8>,
    active_id: u8,
    from: i32,
    target: u16,
    started: Instant,
    duration: Duration,
    reply: oneshot::Sender<Reply>,
    completed: Vec<u8>,
    center: bool,
    stage: MotionStage,
    last_target: Option<i32>,
    torque_confirmed: Option<bool>,
    deadline: Instant,
}

/// After the ramp, joints carrying the robot's weight sit a few counts off their goal:
/// `hold-home` answers once every joint is within [`HOLD_TOLERANCE`] or this much time has
/// passed, and keeps holding either way.
const HOLD_SETTLE: Duration = Duration::from_secs(2);
/// About 2.6°.
const HOLD_TOLERANCE: u32 = 30;
/// A joint still this far off once settled (about 20°) is blocked or turning the wrong way:
/// stop rather than keep pushing.
const HOLD_GIVE_UP: u32 = 230;
/// Consecutive failed reads a settled hold rides out: the servos keep their last goal, and
/// dropping a standing robot on one lost packet would be the worse outcome. A ramp, like any
/// commissioning movement, stops on the first.
const HOLD_READ_FAILURES: u32 = 25;

#[derive(Clone, Copy, PartialEq, Debug)]
enum HoldStage {
    Preload,
    Enable,
    Ramping,
    Holding,
}

/// All joints at once, in `ids` order. Unlike [`Motion`] it outlives its request: the reply
/// goes out once the robot is at HOME, and the hold stays until something releases it.
struct Hold {
    from: Vec<i32>,
    targets: Vec<i32>,
    directions: Vec<i8>,
    started: Instant,
    duration: Duration,
    stage: HoldStage,
    reply: Option<oneshot::Sender<Reply>>,
    last_written: Option<Vec<i32>>,
    max_error: u32,
}

#[derive(Clone, Serialize)]
pub struct HoldStatus {
    stage: &'static str,
    max_error_deg: f64,
    directions: Vec<i8>,
}

/// HOME in raw counts for each joint, from its saved zero and `directions` (or its saved
/// direction). Refuses anything a joint's saved limits or the single-turn encoder cannot hold.
fn home_targets(calibration: &Calibration, directions: &[i8]) -> Result<(Vec<i32>, Vec<i8>), String> {
    let joints = &calibration.joints;
    if !directions.is_empty()
        && (directions.len() != joints.len() || directions.iter().any(|d| ![-1, 1].contains(d)))
    {
        return Err(format!(
            "hold-home directions need exactly {} values of -1/+1, in calibration joint order",
            joints.len()
        ));
    }
    let home = duck_control::deployment::home();
    let mut targets = Vec::with_capacity(joints.len());
    let mut used = Vec::with_capacity(joints.len());
    for (i, joint) in joints.iter().enumerate() {
        let direction = directions.get(i).copied().unwrap_or(joint.direction);
        let angle = home[i];
        if let (Some(min), Some(max)) = (joint.min_rad, joint.max_rad)
            && !(min..=max).contains(&angle)
        {
            return Err(format!("{}: HOME is outside its saved physical limits", joint.name));
        }
        let raw = f64::from(joint.zero_raw)
            + f64::from(direction) * angle * 4096.0 / std::f64::consts::TAU;
        if !(0.0..=4095.0).contains(&raw) {
            return Err(format!("{}: HOME falls outside the single-turn encoder", joint.name));
        }
        targets.push(raw.round() as i32);
        used.push(direction);
    }
    Ok((targets, used))
}

fn hold_error(samples: &BTreeMap<u8, ServoSample>, ids: &[u8], targets: &[i32]) -> u32 {
    ids.iter()
        .zip(targets)
        .map(|(id, target)| samples[id].feedback.position_signed_raw.abs_diff(*target))
        .max()
        .unwrap_or(0)
}

/// Forget the hold, telling its requester why if it is still waiting. Torque is the caller's.
fn cancel_hold(hold: &mut Option<Hold>, reason: &str) {
    if let Some(Hold { reply: Some(reply), .. }) = hold.take() {
        let _ = reply.send(Err(format!("hold-home stopped: {reason}")));
    }
}

/// Advance a hold by one cycle. Torque goes on only after every goal has been set to where
/// the joint already is, as a single-joint move does, so enabling never jumps.
fn step_hold(
    bus: &mut Bus,
    samples: &BTreeMap<u8, ServoSample>,
    ids: &[u8],
    hold: &mut Option<Hold>,
    off_pending: &mut Option<String>,
) {
    let Some(h) = hold.as_mut() else {
        return;
    };
    if h.reply.as_ref().is_some_and(|r| r.is_closed()) {
        cancel_hold(hold, "request abandoned");
        *off_pending = torque_off(bus, ids).err();
        return;
    }
    let step = (|| -> Result<(), String> {
        match h.stage {
            HoldStage::Preload => {
                let rows = ids
                    .iter()
                    .map(|id| (*id, samples[id].feedback.position_signed_raw))
                    .collect::<Vec<_>>();
                h.from = ids.iter().map(|id| samples[id].feedback.position_raw).collect();
                bus.signed_positions(&rows).map_err(|e| e.to_string())?;
                h.stage = HoldStage::Enable;
            }
            HoldStage::Enable => {
                bus.torque(ids, true).map_err(|e| e.to_string())?;
                bus.verify_torque(ids, true).map_err(|e| e.to_string())?;
                h.started = Instant::now();
                h.stage = HoldStage::Ramping;
            }
            HoldStage::Ramping => {
                let elapsed = h.started.elapsed();
                let fraction = (elapsed.as_secs_f64() / h.duration.as_secs_f64()).min(1.0);
                let goals = h
                    .from
                    .iter()
                    .zip(&h.targets)
                    .map(|(from, to)| {
                        (f64::from(*from) + f64::from(to - from) * fraction).round() as i32
                    })
                    .collect::<Vec<_>>();
                if h.last_written.as_ref() != Some(&goals) {
                    let rows = ids.iter().copied().zip(goals.iter().copied()).collect::<Vec<_>>();
                    bus.signed_positions(&rows).map_err(|e| e.to_string())?;
                    h.last_written = Some(goals);
                }
                h.max_error = hold_error(samples, ids, &h.targets);
                if elapsed >= h.duration
                    && (h.max_error <= HOLD_TOLERANCE || elapsed >= h.duration + HOLD_SETTLE)
                {
                    if h.max_error > HOLD_GIVE_UP {
                        return Err(format!(
                            "a joint is still {:.0}° from HOME; check its direction or what blocks it",
                            f64::from(h.max_error) * 360.0 / 4096.0
                        ));
                    }
                    h.stage = HoldStage::Holding;
                    if let Some(reply) = h.reply.take() {
                        let _ = reply.send(Ok(json!({"holding":true,"torque":"on",
                            "max_error_deg":f64::from(h.max_error) * 360.0 / 4096.0,
                            "directions":h.directions})));
                    }
                }
            }
            HoldStage::Holding => {
                h.max_error = hold_error(samples, ids, &h.targets);
            }
        }
        Ok(())
    })();
    if let Err(error) = step {
        cancel_hold(hold, &error);
        *off_pending = torque_off(bus, ids).err();
    }
}

fn torque_off(bus: &mut Bus, ids: &[u8]) -> Result<(), String> {
    bus.torque(ids, false).map_err(|e| e.to_string())?;
    bus.verify_torque(ids, false).map_err(|e| e.to_string())
}

fn abort_motion(
    bus: &mut Bus,
    ids: &[u8],
    motion: &mut Option<Motion>,
    reason: &str,
) -> Option<String> {
    let stop_error = torque_off(bus, ids).err();
    if let Some(moving) = motion.take() {
        let suffix = stop_error
            .as_ref()
            .map(|e| format!("; torque-off not confirmed: {e}"))
            .unwrap_or_default();
        let _ = moving.reply.send(Err(format!(
            "{reason}; active ID {}; completed IDs {:?}{suffix}",
            moving.active_id, moving.completed
        )));
    }
    stop_error
}

fn center_partition(ids: &[u8], samples: &BTreeMap<u8, ServoSample>) -> (Vec<u8>, VecDeque<u8>) {
    let (done, pending): (Vec<_>, Vec<_>) = ids.iter().copied().partition(|id| {
        samples.get(id).is_some_and(|s| {
            s.feedback.position_raw.abs_diff(2048) <= u32::from(POSITION_TOLERANCE)
                && !s.feedback.moving
        })
    });
    (done, pending.into())
}

/// Which trunk IMU this commissioning run reads, and its newest accepted reading in the
/// sensor's own axes — what `verify-imu` checks a candidate mount against. No reading for the
/// bus board, whose verification is the operator's word as it always was.
#[derive(Clone, Copy)]
struct ImuContext {
    source: TrunkImuKind,
    reading: Option<SensorReading>,
}

/// The reading under `mount`, settled: three pushes through a fresh frame, so the median sees
/// three equal samples rather than two defaults and this one.
fn under_mount(mount: [f64; 4], reading: SensorReading) -> ImuData {
    let mut frame = TrunkFrame::new(mount);
    frame.push(reading);
    frame.push(reading);
    frame.push(reading)
}

/// How far from upright a module may read, under the mount being confirmed, when its
/// verification starts. Whatever it reads here stays in every later reading as a tilt bias the
/// policy sees, so it is held to half the HOME guard's 10° (`home_start::MAX_TILT_DEG`) and
/// echoed as `residual_tilt_deg`.
const MODULE_UPRIGHT_DEG: f64 = 5.0;

/// What the upright half read: its echo, and the gravity the nose-down half measures from.
#[derive(Debug, Clone)]
struct Upright {
    echo: Value,
    gravity: [f64; 3],
    tilt_deg: f64,
}

/// The first half of a module's verification: the robot standing upright reads upright
/// through the mount being confirmed. A module has no official orientation to take on trust.
fn upright_check(
    source: TrunkImuKind,
    mount: [f64; 4],
    reading: Option<SensorReading>,
) -> Result<Upright, String> {
    let reading = reading.ok_or_else(|| {
        format!(
            "verify-imu needs a live {} sample; check busStatus last_sample.imu and the reader",
            source.label()
        )
    })?;
    let trunk = under_mount(mount, reading);
    let tilt = super::home_start::tilt_deg(trunk.gravity);
    let echo = json!({"imu_source": source.label(), "mount_wxyz": mount,
        "gravity": trunk.gravity, "tilt_deg": tilt, "gyro_rad_s": trunk.gyro,
        "tilt_limit_deg": MODULE_UPRIGHT_DEG});
    match tilt {
        Some(tilt_deg) if tilt_deg <= MODULE_UPRIGHT_DEG => Ok(Upright {
            echo,
            gravity: trunk.gravity,
            tilt_deg,
        }),
        _ => Err(format!(
            "under this mount the trunk reads {} from upright (limit {MODULE_UPRIGHT_DEG}°); \
             stand the robot upright, or the mount is wrong: {echo}",
            tilt.map_or("no valid gravity".to_owned(), |t| format!("{t:.1}°")),
        )),
    }
}

/// Lean at least this far before the nose-down half of a verification judges anything.
const NOSE_DOWN_MIN_DEG: f64 = 12.0;
/// How far from straight ahead the lean may point and still prove the forward axis. A mount a
/// quarter or half turn off about the vertical reads 90° or 180° here; a hand tipping the
/// robot is rarely more than a few degrees off.
const NOSE_DOWN_HEADING_DEG: f64 = 20.0;
/// Readings running that agree before a verdict: five forward to accept, five the wrong way to
/// refuse. A reading the other way, or not tipped, starts the count again, so a jolt or a wobble
/// through forward is never taken for the tip.
const NOSE_DOWN_READINGS: u32 = 5;
/// How long `verify-imu` waits for the nose-down tip once the upright check has passed.
const NOSE_DOWN_WINDOW: Duration = Duration::from_secs(20);

#[derive(Debug)]
enum NoseDown {
    /// Not leaning far enough to judge yet.
    Level,
    /// Leaning nose-down, as the trunk frame should read it: the echo of what it read.
    Forward(Value),
    /// Leaning, but the wrong way round for a nose-down tip under this mount.
    Wrong(String),
}

/// The second half: tipping the robot nose-down proves the forward axis. Upright alone proves
/// only the vertical one — a mount a quarter or half turn off about the vertical reads upright
/// just as well, and then turns a pitch into a roll or flips its sign, and the policy corrects
/// the wrong way and falls. In the trunk frame gravity leans toward the low side, so a
/// nose-down tip moves it toward +x: from upright, 15° is `[0.26, 0, -0.97]`, the convention
/// the policy and `home_start` read.
///
/// Judged from where the robot stood (`upright`), not from vertical: the tip is the change in
/// gravity, so the few degrees of bias the upright half allowed neither swing the heading nor
/// count toward the lean.
fn nose_down(mount: [f64; 4], reading: SensorReading, upright: [f64; 3]) -> NoseDown {
    let trunk = under_mount(mount, reading);
    let unit = |v: [f64; 3]| {
        let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        (n.is_finite() && n > 0.0).then(|| v.map(|c| c / n))
    };
    let (Some(g), Some(u)) = (unit(trunk.gravity), unit(upright)) else {
        return NoseDown::Level;
    };
    let tilt = (g[0] * u[0] + g[1] * u[1] + g[2] * u[2])
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees();
    if tilt < NOSE_DOWN_MIN_DEG {
        return NoseDown::Level;
    }
    // 0° nose-down, ±180° tail-down, +90° the robot's left side low, -90° its right.
    let heading = (g[1] - u[1]).atan2(g[0] - u[0]).to_degrees();
    let echo = json!({"gravity": trunk.gravity, "tilt_from_upright_deg": tilt,
        "lean_heading_deg": heading, "gyro_rad_s": trunk.gyro});
    if heading.abs() <= NOSE_DOWN_HEADING_DEG {
        return NoseDown::Forward(echo);
    }
    let reads = if heading.abs() >= 135.0 {
        "tail-down"
    } else if heading > 0.0 {
        "as the left side going down"
    } else {
        "as the right side going down"
    };
    NoseDown::Wrong(format!(
        "tipped {tilt:.0}°, but under this mount it reads {reads} ({heading:.0}° from \
         nose-down): the mount is about that far off about the vertical axis. If the nose did \
         not go down, tip it nose-down and run verify-imu again. Nothing saved: {echo}"
    ))
}

/// A module's verification between its upright check and the nose-down tip it is waiting for.
struct PendingVerify {
    reply: oneshot::Sender<Reply>,
    mount: [f64; 4],
    upright: Upright,
    deadline: Instant,
    /// Readings running that read nose-down, and that read the wrong way.
    forward: u32,
    wrong: u32,
}

impl PendingVerify {
    fn new(reply: oneshot::Sender<Reply>, mount: [f64; 4], upright: Upright, now: Instant) -> Self {
        Self {
            reply,
            mount,
            upright,
            deadline: now + NOSE_DOWN_WINDOW,
            forward: 0,
            wrong: 0,
        }
    }

    /// One tick's live reading — none on a tick whose read failed. `Some` once decided.
    fn observe(
        &mut self,
        reading: Option<SensorReading>,
        now: Instant,
    ) -> Option<Result<Value, String>> {
        if let Some(reading) = reading {
            match nose_down(self.mount, reading, self.upright.gravity) {
                NoseDown::Level => (self.forward, self.wrong) = (0, 0),
                NoseDown::Forward(echo) => {
                    (self.forward, self.wrong) = (self.forward + 1, 0);
                    if self.forward >= NOSE_DOWN_READINGS {
                        return Some(Ok(json!({"upright": self.upright.echo,
                            "nose_down": echo, "residual_tilt_deg": self.upright.tilt_deg})));
                    }
                }
                NoseDown::Wrong(message) => {
                    (self.forward, self.wrong) = (0, self.wrong + 1);
                    if self.wrong >= NOSE_DOWN_READINGS {
                        return Some(Err(message));
                    }
                }
            }
        }
        (now >= self.deadline).then(|| {
            Err(format!(
                "no nose-down tip of at least {NOSE_DOWN_MIN_DEG}° held within {} s of the \
                 upright check; nothing saved",
                NOSE_DOWN_WINDOW.as_secs()
            ))
        })
    }
}

/// Write `source`'s mount as verified. Motion stays off until it is enabled again.
fn save_verified(
    calibration: &Calibration,
    path: &Path,
    source: TrunkImuKind,
    mount: [f64; 4],
    check: Option<Value>,
) -> Result<(Calibration, Value), String> {
    let mut updated = calibration.clone();
    updated.set_imu_mount(
        source,
        ImuMount {
            quat: mount,
            verified: true,
        },
    );
    updated.motion_enabled = false;
    save_calibration(path, &updated)?;
    let mut status = calibration_status(&updated, path, source);
    if let Some(check) = check {
        status["imu_check"] = check;
    }
    Ok((updated, status))
}

#[allow(clippy::too_many_arguments)]
fn process_job(
    job: Job,
    bus: &mut Bus,
    samples: &BTreeMap<u8, ServoSample>,
    calibration: &mut Calibration,
    path: &Path,
    motion: &mut Option<Motion>,
    ids: &[u8],
    off_pending: &mut Option<String>,
    imu: ImuContext,
    verifying: &mut Option<PendingVerify>,
    hold: &mut Option<Hold>,
) {
    if job.reply.is_closed() {
        return;
    }
    if matches!(job.command, BusCommand::Relax) {
        if let Some(pending) = verifying.take() {
            let _ = pending
                .reply
                .send(Err("verify-imu cancelled by relax; nothing saved".into()));
        }
        cancel_hold(hold, "relax");
        let stop_error = abort_motion(bus, ids, motion, "movement interrupted by relax");
        *off_pending = stop_error.clone();
        let result =
            stop_error.map_or_else(|| Ok(json!({"torque":"off", "confirmed_ids":ids})), Err);
        let _ = job.reply.send(result);
        return;
    }
    if matches!(job.command, BusCommand::CalibrationStatus) {
        let _ = job.reply.send(Ok(calibration_status_for_bus(
            calibration,
            path,
            imu.source,
            bus,
        )));
        return;
    }
    if verifying.is_some() {
        let _ = job.reply.send(Err(
            "verify-imu is waiting for the nose-down tip; relax cancels it".into(),
        ));
        return;
    }
    if off_pending.is_some() {
        let _ = job.reply.send(Err(
            "torque-off has not been confirmed; wait for busStatus torque_off_pending to clear"
                .into(),
        ));
        return;
    }
    if motion.is_some() {
        let _ = job.reply.send(Err(
            "a movement is already active; use bus relax to interrupt it".into(),
        ));
        return;
    }
    // Standing on HOME: the IMU may be checked, nothing may move a joint.
    if let Some(h) = hold.as_ref()
        && !matches!(job.command, BusCommand::VerifyImu { .. })
    {
        let _ = job.reply.send(if matches!(job.command, BusCommand::HoldHome { .. })
            && h.stage == HoldStage::Holding
        {
            Ok(json!({"holding":true,"torque":"on","already":true,"directions":h.directions}))
        } else {
            Err("the robot is holding HOME; relax first".into())
        });
        return;
    }
    match job.command {
        BusCommand::Move {
            id,
            position,
            duration_ms,
        } => {
            if !ids.contains(&id) || position > 4095 || !(500..=60_000).contains(&duration_ms) {
                let _ = job.reply.send(Err("move requires a configured servo ID, position 0..4095 and duration-ms 500..60000".into()));
                return;
            }
            if let Some(joint) = calibration.joints.iter().find(|joint| joint.id == id)
                && let (Some(min), Some(max)) = (joint.min_rad, joint.max_rad)
            {
                let angle = (f64::from(position) - f64::from(joint.zero_raw))
                    * std::f64::consts::TAU
                    / 4096.0
                    * f64::from(joint.direction);
                if !(min..=max).contains(&angle) {
                    let _ = job.reply.send(Err(
                        "requested raw position exceeds this joint's saved physical limits".into(),
                    ));
                    return;
                }
            }
            *motion = Some(Motion {
                remaining: VecDeque::new(),
                active_id: id,
                from: samples[&id].feedback.position_raw,
                target: position,
                started: Instant::now(),
                duration: Duration::from_millis(duration_ms),
                reply: job.reply,
                completed: vec![],
                center: false,
                stage: MotionStage::Preload,
                last_target: None,
                torque_confirmed: Some(false),
                deadline: Instant::now()
                    + Duration::from_millis(duration_ms)
                    + SETTLE_TIMEOUT
                    + Duration::from_secs(1),
            });
        }

        BusCommand::Center {
            unassembled,
            duration_ms,
        } => {
            if !unassembled || !(500..=60_000).contains(&duration_ms) {
                let _ = job.reply.send(Err(
                    "center requires --unassembled and duration-ms 500..60000; it is unsuitable for assembled joints".into(),
                ));
                return;
            }
            if calibration.joints.iter().any(|j| j.calibrated) {
                let _ = job
                    .reply
                    .send(Err("center is unavailable after joint calibration".into()));
                return;
            }
            // A new click always uses fresh positions, including after a previous
            // cancelled job or service restart. Never repeat already-centered joints.
            let (completed, mut remaining) = center_partition(ids, samples);
            let Some(id) = remaining.pop_front() else {
                let _ = job.reply.send(Ok(
                    json!({"completed_ids":completed,"position_raw":2048,"torque":"off"}),
                ));
                return;
            };
            *motion = Some(Motion {
                remaining,
                active_id: id,
                from: samples[&id].feedback.position_raw,
                target: 2048,
                started: Instant::now(),
                duration: Duration::from_millis(duration_ms),
                reply: job.reply,
                completed,
                center: true,
                stage: MotionStage::Preload,
                last_target: None,
                torque_confirmed: Some(false),
                deadline: Instant::now()
                    + Duration::from_millis(duration_ms * ids.len() as u64)
                    + SETTLE_TIMEOUT * ids.len() as u32
                    + Duration::from_secs(30),
            });
        }

        BusCommand::Capture {
            id,
            angle_deg,
            direction,
            min_deg,
            max_deg,
        } => {
            let result = capture(calibration, samples, id, angle_deg, direction, min_deg, max_deg).and_then(|updated| {
                save_calibration(path, &updated)?;
                *calibration = updated;
                Ok(json!({"saved":true,"id":id,"position_raw":samples[&id].feedback.position_raw,
                    "known_angle_deg":angle_deg,"calibration":calibration_status_for_bus(calibration,path,imu.source,bus)}))
            });
            let _ = job.reply.send(result);
        }
        BusCommand::VerifyImu {
            confirmed,
            mount_wxyz,
        } => {
            let result = if confirmed {
                let mount = if mount_wxyz.is_empty() {
                    calibration.imu_mount(imu.source).quat
                } else {
                    match mount_wxyz.try_into() {
                        Ok(mount) => mount,
                        Err(_) => {
                            let _ = job
                                .reply
                                .send(Err("mount-wxyz needs exactly four values: w,x,y,z".into()));
                            return;
                        }
                    }
                };
                if imu.source == TrunkImuKind::Bus {
                    let mut candidate = calibration.clone();
                    candidate.imu_bus_protocol = Some(bus.imu_board().protocol());
                    candidate.imu_bus_profile = Some(bus.imu_profile());
                    save_verified(&candidate, path, imu.source, mount, None).map(
                        |(updated, _status)| {
                            *calibration = updated;
                            calibration_status_for_bus(calibration, path, imu.source, bus)
                        },
                    )
                } else {
                    match upright_check(imu.source, mount, imu.reading) {
                        Ok(upright) => {
                            tracing::warn!(
                                source = imu.source.label(),
                                "verify-imu: upright check passed; now tip the robot nose-down \
                                 by 15-30 degrees"
                            );
                            *verifying = Some(PendingVerify::new(
                                job.reply,
                                mount,
                                upright,
                                Instant::now(),
                            ));
                            return;
                        }
                        Err(error) => Err(error),
                    }
                }
            } else {
                Err(
                    "verify-imu requires --confirmed after checking the official mounting axes"
                        .into(),
                )
            };
            let _ = job.reply.send(result);
        }
        BusCommand::HoldHome {
            directions,
            duration_ms,
        } => {
            if !(500..=60_000).contains(&duration_ms) {
                let _ = job.reply.send(Err("hold-home duration-ms must be 500..60000".into()));
                return;
            }
            match home_targets(calibration, &directions) {
                Ok((targets, directions)) => {
                    *hold = Some(Hold {
                        from: vec![],
                        targets,
                        directions,
                        started: Instant::now(),
                        duration: Duration::from_millis(duration_ms),
                        stage: HoldStage::Preload,
                        reply: Some(job.reply),
                        last_written: None,
                        max_error: 0,
                    });
                }
                Err(error) => {
                    let _ = job.reply.send(Err(error));
                }
            }
        }
        BusCommand::Relax
        | BusCommand::CalibrationStatus
        | BusCommand::Reboot {}
        | BusCommand::Shutdown {} => unreachable!(),
    }
}

fn step_motion(
    bus: &mut Bus,
    samples: &BTreeMap<u8, ServoSample>,
    ids: &[u8],
    motion: &mut Option<Motion>,
    off_pending: &mut Option<String>,
) {
    let Some(m) = motion.as_mut() else {
        return;
    };
    if m.reply.is_closed() || Instant::now() >= m.deadline {
        *off_pending = abort_motion(bus, ids, motion, "movement cancelled or timed out");
        return;
    }
    let step = (|| -> Result<bool, String> {
        match m.stage {
            MotionStage::Preload => {
                let feedback = &samples[&m.active_id].feedback;
                m.from = feedback.position_raw;
                bus.signed_positions(&[(m.active_id, feedback.position_signed_raw)])
                    .map_err(|e| e.to_string())?;
                m.stage = MotionStage::ConfirmGoal;
            }
            MotionStage::ConfirmGoal => {
                if bus
                    .read_control(m.active_id, 42, 2)
                    .map_err(|e| e.to_string())?
                    != signed_position_word(m.from)
                        .map_err(|e| e.to_string())?
                        .to_le_bytes()
                {
                    return Err("hold goal preload was not confirmed".into());
                }
                m.stage = MotionStage::Enable;
            }
            MotionStage::Enable => {
                bus.torque(&[m.active_id], true)
                    .map_err(|e| e.to_string())?;
                m.torque_confirmed = None;
                m.stage = MotionStage::ConfirmEnabled;
            }
            MotionStage::ConfirmEnabled => {
                if bus
                    .read_control(m.active_id, 40, 1)
                    .map_err(|e| e.to_string())?
                    != [1]
                {
                    return Err("torque-on was not confirmed".into());
                }
                m.torque_confirmed = Some(true);
                m.started = Instant::now();
                m.stage = MotionStage::Moving;
            }
            MotionStage::Moving => {
                let elapsed = m.started.elapsed();
                let fraction = (elapsed.as_secs_f64() / m.duration.as_secs_f64()).min(1.0);
                let target = (f64::from(m.from)
                    + (f64::from(m.target) - f64::from(m.from)) * fraction)
                    .round() as i32;
                if m.last_target != Some(target) {
                    bus.signed_positions(&[(m.active_id, target)])
                        .map_err(|e| e.to_string())?;
                    m.last_target = Some(target);
                }
                if elapsed >= m.duration
                    && samples[&m.active_id]
                        .feedback
                        .position_signed_raw
                        .abs_diff(i32::from(m.target))
                        <= u32::from(POSITION_TOLERANCE)
                {
                    m.stage = MotionStage::Disable;
                } else if elapsed > m.duration + SETTLE_TIMEOUT {
                    return Err(format!(
                        "ID {} did not reach {}; stopped",
                        m.active_id, m.target
                    ));
                }
            }
            MotionStage::Disable => {
                bus.torque(&[m.active_id], false)
                    .map_err(|e| e.to_string())?;
                m.torque_confirmed = None;
                m.stage = MotionStage::ConfirmDisabled;
            }
            MotionStage::ConfirmDisabled => {
                if bus
                    .read_control(m.active_id, 40, 1)
                    .map_err(|e| e.to_string())?
                    != [0]
                {
                    return Err("torque-off was not confirmed".into());
                }
                m.torque_confirmed = Some(false);
                m.completed.push(m.active_id);
                if let Some(next) = m.remaining.pop_front() {
                    m.active_id = next;
                    m.stage = MotionStage::Preload;
                    m.last_target = None;
                } else {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    })();
    match step {
        Ok(true) => {
            let done = motion.take().unwrap();
            let _ = done.reply.send(Ok(
                json!({"completed_ids":done.completed,"position_raw":done.target,"torque":"off"}),
            ));
        }
        Ok(false) => {}
        Err(error) => *off_pending = abort_motion(bus, ids, motion, &error),
    }
}

#[derive(Clone, Serialize)]
pub struct ServoStatus {
    id: u8,
    joint: String,
    position_raw: i32,
    position_wire_raw: u16,
    position_signed_raw: i32,
    speed_raw: i32,
    load_raw: i32,
    current_raw: i32,
    voltage_v: f64,
    temperature_c: u8,
    moving: bool,
    calibrated: bool,
}
#[derive(Clone, Serialize)]
pub struct ImuStatus {
    /// The bus board's ID; absent for a module.
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<u8>,
    /// Which trunk IMU: a `TrunkImuKind` label.
    source: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    sequence: Option<u32>,
    ready: bool,
    valid: bool,
    gravity: [f64; 3],
    gyro_rad_s: [f64; 3],
    quaternion_wxyz: [f64; 4],
    mount_verified: bool,
    /// A module's sample age when this tick took it, measured on the host.
    #[serde(skip_serializing_if = "Option::is_none")]
    age_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    epoch: Option<u32>,
    /// A BNO08x's own accuracy estimate, 0 unreliable … 3 high.
    #[serde(skip_serializing_if = "Option::is_none")]
    accuracy: Option<u8>,
    /// Lean from upright under the current mount, what `verify-imu` holds a module to.
    #[serde(skip_serializing_if = "Option::is_none")]
    tilt_deg: Option<f64>,
}
#[derive(Clone, Serialize)]
pub struct Sample {
    cycle: u64,
    at_us: u64,
    servos: Vec<ServoStatus>,
    imu: ImuStatus,
}
#[derive(Clone, Serialize)]
pub struct Failure {
    cycle: u64,
    error: String,
    missing_ids: Vec<u8>,
}
#[derive(Clone, Serialize)]
pub struct MotionStatus {
    id: u8,
    target_raw: u16,
    action: &'static str,
    completed_ids: Vec<u8>,
}
/// Plain snapshot published at 10 Hz. JSON is produced only by the IPC thread.
#[derive(Clone, Serialize)]
pub struct Status {
    configured_imu_profile: &'static str,
    build: &'static str,
    port: String,
    mode: &'static str,
    phase: &'static str,
    session: u32,
    target_hz: f64,
    achieved_hz: f64,
    total_cycles: u64,
    total_success: u64,
    total_communication_failures: u64,
    errors_by_id: BTreeMap<u8, u64>,
    device_communication: super::communication_stats::CommunicationStats,
    last_cycle_error: Option<String>,
    last_failure: Option<Failure>,
    last_sample: Option<Sample>,
    calibrated: bool,
    imu_mount_verified: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    imu_bus: Option<BusImuInfo>,
    /// What `verify-imu` is waiting for, while it waits.
    #[serde(skip_serializing_if = "Option::is_none")]
    imu_verification: Option<&'static str>,
    torque_state: &'static str,
    torque_off_pending: Option<String>,
    power_pending: bool,
    motion: Option<MotionStatus>,
    /// Present while `hold-home` has the joints: `ramping`, then `holding`.
    #[serde(skip_serializing_if = "Option::is_none")]
    hold: Option<HoldStatus>,
    policy_motion_enabled: bool,
    updated_at_us: u64,
}

/// `host_imu`: the trunk IMU module's reader when `[trunk_imu]` names one. The bus then
/// carries the servos alone, ID 200 is neither identified nor read, and `verify-imu` writes
/// that module's mount.
pub async fn run(
    port: &str,
    imu_protocol: &str,
    imu_profile: &str,
    state: &RobotState,
    period: Duration,
    calibration_path: &Path,
    host_imu: Option<&dyn HostImu>,
) {
    let source = host_imu.map_or(TrunkImuKind::Bus, |host| host.kind());
    state.imu_ready.store(false, Ordering::Relaxed);
    let mut calibration = match read_calibration(calibration_path) {
        Ok(c) => c,
        Err(error) => {
            state.bus_status.store(Some(Arc::new(json!({"build":BUILD,"mode":"commissioning","phase":"configuration_error","configured_imu_profile":imu_profile,"error":error}))));
            state.startup_bus_failures.store(1, Ordering::Relaxed);
            return;
        }
    };
    let mut initial_status = calibration_status(&calibration, calibration_path, source);
    initial_status["configured_imu_profile"] = json!(imu_profile);
    state.bus_calibration.store(Some(Arc::new(initial_status)));
    let ids = calibration.joints.iter().map(|j| j.id).collect::<Vec<_>>();
    state.startup_step(
        2,
        "starting",
        "标定模式：读取已有标定，关闭并确认扭矩，初始化 IMU",
    );
    let mut bus = loop {
        if state.shutdown.load(Ordering::Relaxed) { return; }
        let mut startup_observations = None;
        let connect = (|| -> Result<Bus, String> {
            let mut bus = Bus::open(port).map_err(|e| e.to_string())?;
            bus.set_imu_protocol(imu_protocol)
                .map_err(|e| e.to_string())?;
            bus.set_imu_profile(imu_profile)
                .map_err(|e| e.to_string())?;
            let initialized = (|| -> Result<(), String> {
                torque_off(&mut bus, &ids)?;
                if host_imu.is_none() {
                    bus.identify_imu().map_err(|e| format!("IMU200: {e}"))?;
                }
                Ok(())
            })();
            if initialized.is_err() {
                startup_observations = Some(bus.servo_observations(&calibration));
            }
            initialized?;
            Ok(bus)
        })();
        match connect {
            Ok(bus) => break bus,
            Err(error) => {
                state.startup_bus_failures.fetch_add(1, Ordering::Relaxed);
                state.startup_step(2, "waiting", &error);
                let report = json!({"build":BUILD,"mode":"commissioning","phase":"waiting_for_bus","configured_imu_profile":imu_profile,"error":error});
                if let Some(observations) = startup_observations {
                    let native = json!({
                        "backend":"feetech_ft6", "configured_imu_profile":imu_profile,
                        "imu_source":source.label(), "servo_observations":&observations,
                    });
                    state.publish_startup_native_report(report, native, observations);
                } else {
                    state.publish_startup_native(report, None);
                }
                tokio::time::sleep(super::STARTUP_RETRY_INTERVAL).await;
            }
        }
    };
    state
        .bus_calibration
        .store(Some(Arc::new(calibration_status_for_bus(
            &calibration,
            calibration_path,
            source,
            &bus,
        ))));
    state.startup_step(3, "success", "标定模式不加载策略模型");
    state.startup_bus_failures.store(0, Ordering::Relaxed);
    let mut ticker = super::control_ticker(period);
    // Tokio intervals tick immediately on creation. Start periodic telemetry one
    // normal period after startup torque verification, not in the same burst.
    // This runs once: no warm-up read, discarded failure or steady-state delay.
    ticker.reset_after(period);
    let mut rate_start = Instant::now();
    let mut rate_ticks = 0u64;
    let mut total_success = 0u64;
    let mut total_failures = 0u64;
    let mut bus_freshness = BusImuFreshness::default();
    let mut motion: Option<Motion> = None;
    let mut hold: Option<Hold> = None;
    let mut off_pending = None;
    let mut imu_decoder = TrunkFrame::new(calibration.imu_mount_quat);
    let mut imu_frame = TrunkFrame::new(calibration.imu_mount(source).quat);
    let mut imu_gate = HostImuGate::default();
    let mut imu_epoch = None;
    let mut host_reading = None;
    let mut verifying: Option<PendingVerify> = None;
    let mut last_sample = None;
    let mut last_failure = None;
    let mut errors_by_id = BTreeMap::<u8, u64>::new();
    let mut communication_stats = super::communication_stats::CommunicationStats::new(
        ids.iter()
            .copied()
            .chain((host_imu.is_none()).then_some(IMU_ID)),
    );
    let mut next_publish = Instant::now();
    while !state.shutdown.load(Ordering::Relaxed) {
        ticker.tick().await;
        if state.shutdown.load(Ordering::Relaxed) {
            break;
        }
        let tick = Instant::now();
        let cycle = state.ticks.load(Ordering::Relaxed) + 1;
        let publish = tick >= next_publish;
        let mut samples = BTreeMap::new();
        let mut imu_status = None;
        let result = (|| -> Result<(), String> {
            let acquisition = if host_imu.is_some() {
                bus.joint_telemetry(&ids)
            } else {
                bus.telemetry(&ids)
            };
            communication_stats.observe(&bus.last_sync);
            let blocks = acquisition.map_err(|e| e.to_string())?;
            for id in &ids {
                let sample =
                    ServoSample::decode(&blocks[id]).map_err(|e| format!("ID {id}: {e}"))?;
                samples.insert(*id, sample);
            }
            if let Some(host) = host_imu {
                let now = Instant::now();
                let before = imu_gate.stale();
                let checked = imu_gate.check(host.latest(), now);
                let after = imu_gate.stale();
                state
                    .imu_stale_blocks
                    .fetch_add(after.total - before.total, Ordering::Relaxed);
                state.imu_stale_run.store(after.run, Ordering::Relaxed);
                let sample = checked.map_err(|e| format!("IMU {}: {e}", source.label()))?;
                if imu_epoch != Some(sample.epoch) {
                    imu_frame.restart_median();
                    imu_epoch = Some(sample.epoch);
                }
                // The chip's own orientation whatever `bno08x_fusion` publishes: a mount is
                // measured against the sensor, never fitted to the host's fusion of it.
                let reading = SensorReading {
                    gyro: sample.gyro,
                    quat: Some(sample.chip_quat()),
                };
                host_reading = Some(reading);
                let decoded = imu_frame.push(reading);
                let valid = imu_frame.ready();
                state.imu_ready.store(valid, Ordering::Relaxed);
                if publish {
                    imu_status = Some(ImuStatus {
                        id: None,
                        source: source.label(),
                        sequence: Some(sample.sequence),
                        ready: true,
                        valid,
                        gravity: decoded.gravity,
                        gyro_rad_s: decoded.gyro,
                        quaternion_wxyz: decoded.quat,
                        mount_verified: calibration.imu_mount(source).verified,
                        age_ms: Some(
                            now.saturating_duration_since(sample.received_at)
                                .as_secs_f64()
                                * 1000.0,
                        ),
                        epoch: Some(sample.epoch),
                        accuracy: sample.accuracy,
                        tilt_deg: super::home_start::tilt_deg(decoded.gravity),
                    });
                }
                return Ok(());
            }
            let imu = bus
                .decode_imu(&blocks[&IMU_ID])
                .map_err(|e| format!("IMU200: {e}"))?;
            let before = bus_freshness.stale;
            let checked = bus_freshness.check(&imu, bus.last_sync.elapsed_us);
            state
                .imu_stale_blocks
                .fetch_add(bus_freshness.stale.total - before.total, Ordering::Relaxed);
            state
                .imu_stale_run
                .store(bus_freshness.stale.run, Ordering::Relaxed);
            checked.map_err(|e| e.to_string())?;
            let decoded = imu_decoder.push(imu.reading());
            let valid = imu_decoder.ready();
            state.imu_ready.store(valid, Ordering::Relaxed);
            if publish {
                imu_status = Some(ImuStatus {
                    id: Some(IMU_ID),
                    source: source.label(),
                    sequence: imu.sequence,
                    ready: imu.ready,
                    valid,
                    gravity: decoded.gravity,
                    gyro_rad_s: decoded.gyro,
                    quaternion_wxyz: decoded.quat,
                    mount_verified: calibration.imu_mount_verified
                        && calibration
                            .validate_bus_mount_for(bus.imu_board().protocol(), bus.imu_profile())
                            .is_ok(),
                    age_ms: None,
                    epoch: None,
                    accuracy: None,
                    tilt_deg: None,
                });
            }
            Ok(())
        })();
        let error = result.err();
        if let Some(error) = &error {
            total_failures += 1;
            if error.starts_with("IMU200") {
                *errors_by_id.entry(IMU_ID).or_default() += 1;
            }
            for id in &bus.last_sync.missing_ids {
                *errors_by_id.entry(*id).or_default() += 1;
            }
            last_failure = Some(Failure {
                cycle,
                error: error.clone(),
                missing_ids: bus.last_sync.missing_ids.clone(),
            });
            state.imu_ready.store(false, Ordering::Relaxed);
            let consecutive = state.consecutive_errors.fetch_add(1, Ordering::Relaxed) + 1;
            if motion.is_some() {
                off_pending = abort_motion(&mut bus, &ids, &mut motion, error);
            }
            if hold.as_ref().is_some_and(|h| {
                h.stage != HoldStage::Holding || consecutive >= HOLD_READ_FAILURES
            }) {
                cancel_hold(&mut hold, error);
                off_pending = torque_off(&mut bus, &ids).err();
            }
            if consecutive == 1 {
                tracing::warn!(cycle,%error,"bus read failed");
            }
            while let Some(job) = state.bus_commands.pop() {
                if matches!(job.command, BusCommand::Relax) {
                    process_job(
                        job,
                        &mut bus,
                        &samples,
                        &mut calibration,
                        calibration_path,
                        &mut motion,
                        &ids,
                        &mut off_pending,
                        ImuContext {
                            source,
                            reading: None,
                        },
                        &mut verifying,
                        &mut hold,
                    );
                } else {
                    let _ = job.reply.send(Err(error.clone()));
                }
            }
        } else {
            total_success += 1;
            state.consecutive_errors.store(0, Ordering::Relaxed);
            if publish {
                let servos = calibration
                    .joints
                    .iter()
                    .map(|j| {
                        let x = &samples[&j.id];
                        let f = &x.feedback;
                        ServoStatus {
                            id: j.id,
                            joint: j.name.clone(),
                            position_raw: f.position_raw,
                            position_wire_raw: f.position_wire_raw,
                            position_signed_raw: f.position_signed_raw,
                            speed_raw: f.speed_raw,
                            load_raw: f.load_raw,
                            current_raw: x.current_raw,
                            voltage_v: f.volts,
                            temperature_c: f.temperature_c,
                            moving: f.moving,
                            calibrated: j.calibrated,
                        }
                    })
                    .collect();
                last_sample = Some(Sample {
                    cycle,
                    at_us: state.started.elapsed().as_micros() as u64,
                    servos,
                    imu: imu_status.unwrap(),
                });
                let volts =
                    samples.values().map(|s| s.feedback.volts).sum::<f64>() / NUM_JOINTS as f64;
                state.battery_v.store(volts.to_bits(), Ordering::Relaxed);
                let (hottest, temp) = calibration
                    .joints
                    .iter()
                    .enumerate()
                    .map(|(i, j)| (i, samples[&j.id].feedback.temperature_c))
                    .max_by_key(|(_, t)| *t)
                    .unwrap();
                state
                    .motor_max_c
                    .store(f64::from(temp).to_bits(), Ordering::Relaxed);
                state.motor_hottest.store(hottest as u32, Ordering::Relaxed);
                state.motor_mean_c.store(
                    (samples
                        .values()
                        .map(|s| f64::from(s.feedback.temperature_c))
                        .sum::<f64>()
                        / NUM_JOINTS as f64)
                        .to_bits(),
                    Ordering::Relaxed,
                );
            }
            if let Some(job) = state.bus_commands.pop() {
                let changes_calibration = matches!(
                    job.command,
                    BusCommand::Capture { .. } | BusCommand::VerifyImu { .. }
                );
                let old_mount = calibration.imu_mount(source).quat;
                process_job(
                    job,
                    &mut bus,
                    &samples,
                    &mut calibration,
                    calibration_path,
                    &mut motion,
                    &ids,
                    &mut off_pending,
                    ImuContext {
                        source,
                        reading: host_reading,
                    },
                    &mut verifying,
                    &mut hold,
                );
                if changes_calibration {
                    state
                        .bus_calibration
                        .store(Some(Arc::new(calibration_status_for_bus(
                            &calibration,
                            calibration_path,
                            source,
                            &bus,
                        ))));
                }
                if old_mount != calibration.imu_mount(source).quat {
                    imu_decoder = TrunkFrame::new(calibration.imu_mount_quat);
                    imu_frame = TrunkFrame::new(calibration.imu_mount(source).quat);
                }
            }
            step_motion(&mut bus, &samples, &ids, &mut motion, &mut off_pending);
            step_hold(&mut bus, &samples, &ids, &mut hold, &mut off_pending);
        }
        if let Some(mut pending) = verifying.take() {
            let reading = if error.is_none() { host_reading } else { None };
            match pending.observe(reading, Instant::now()) {
                // The requester stopped waiting (its RPC timed out): a mount saved now would be
                // verified with nobody told.
                _ if pending.reply.is_closed() => {
                    tracing::warn!("verify-imu: the request was abandoned; nothing saved");
                }
                None => verifying = Some(pending),
                Some(verdict) => {
                    let old_mount = calibration.imu_mount(source).quat;
                    let result = verdict
                        .and_then(|check| {
                            save_verified(
                                &calibration,
                                calibration_path,
                                source,
                                pending.mount,
                                Some(check),
                            )
                        })
                        .map(|(updated, status)| {
                            calibration = updated;
                            status
                        });
                    if result.is_ok() {
                        state
                            .bus_calibration
                            .store(Some(Arc::new(calibration_status_for_bus(
                                &calibration,
                                calibration_path,
                                source,
                                &bus,
                            ))));
                        if old_mount != calibration.imu_mount(source).quat {
                            imu_frame = TrunkFrame::new(calibration.imu_mount(source).quat);
                        }
                    }
                    let _ = pending.reply.send(result);
                }
            }
        }
        // A settled hold is standing, not moving: safe to restart, as motion's standing is.
        state.moving.store(
            motion.is_some()
                || off_pending.is_some()
                || hold.as_ref().is_some_and(|h| h.stage != HoldStage::Holding),
            Ordering::Relaxed,
        );
        state.ticks.store(cycle, Ordering::Relaxed);
        if error.is_none() && total_success == 1 {
            state.startup_step(2, "success", "标定采集第一份有效反馈已读取");
        } else if total_success == 0 {
            state.startup_step(2, "waiting", error.as_deref().unwrap_or("等待第一份有效反馈"));
        }
        if cycle == 1 { state.startup_step(4, "success", "标定采集首个周期完成；运动策略关闭"); }
        state.last_tick_us.store(
            state.started.elapsed().as_micros() as u64,
            Ordering::Relaxed,
        );
        rate_ticks += 1;
        if rate_start.elapsed() >= super::RATE_WINDOW {
            state.achieved_hz.store(
                (rate_ticks as f64 / rate_start.elapsed().as_secs_f64()).to_bits(),
                Ordering::Relaxed,
            );
            rate_ticks = 0;
            rate_start = Instant::now();
        }
        if publish || error.is_some() {
            let power = state.bus_commands.power_pending.load(Ordering::Relaxed);
            state.bus_snapshot.store(Some(Arc::new(Status {
                configured_imu_profile: bus.imu_profile().label(),
                build: BUILD,
                port: port.to_owned(),
                mode: "commissioning",
                phase: if power {
                    "power_pending"
                } else if error.is_some() {
                    "read_error"
                } else if off_pending.is_some() {
                    "stop_unconfirmed"
                } else if motion.is_some()
                    || hold.as_ref().is_some_and(|h| h.stage != HoldStage::Holding)
                {
                    "moving"
                } else if hold.is_some() {
                    "holding"
                } else {
                    "ready"
                },
                session: std::process::id(),
                target_hz: 1.0 / period.as_secs_f64(),
                achieved_hz: f64::from_bits(state.achieved_hz.load(Ordering::Relaxed)),
                total_cycles: cycle,
                total_success,
                total_communication_failures: total_failures,
                errors_by_id: errors_by_id.clone(),
                device_communication: communication_stats.clone(),
                last_cycle_error: error,
                last_failure: last_failure.clone(),
                last_sample: last_sample.clone(),
                calibrated: calibration.joints.iter().all(|j| j.calibrated),
                imu_mount_verified: calibration.imu_mount(source).verified
                    && (source != TrunkImuKind::Bus
                        || calibration
                            .validate_bus_mount_for(bus.imu_board().protocol(), bus.imu_profile())
                            .is_ok()),
                imu_bus: bus.imu_info(),
                imu_verification: verifying
                    .as_ref()
                    .map(|_| "verify-imu: tip the robot nose-down by 15-30 degrees and hold it"),
                torque_state: if off_pending.is_some() {
                    "unknown"
                } else if let Some(h) = &hold {
                    match h.stage {
                        HoldStage::Preload => "off",
                        HoldStage::Enable => "unknown",
                        HoldStage::Ramping | HoldStage::Holding => "all_enabled",
                    }
                } else if let Some(m) = &motion {
                    match m.torque_confirmed {
                        Some(true) => "single_joint_enabled",
                        Some(false) => "off",
                        None => "unknown",
                    }
                } else {
                    "off"
                },
                torque_off_pending: off_pending.clone(),
                power_pending: power,
                policy_motion_enabled: false,
                motion: motion.as_ref().map(|m| MotionStatus {
                    id: m.active_id,
                    target_raw: m.target,
                    action: if m.center { "center" } else { "move" },
                    completed_ids: m.completed.clone(),
                }),
                hold: hold.as_ref().map(|h| HoldStatus {
                    stage: if h.stage == HoldStage::Holding { "holding" } else { "ramping" },
                    max_error_deg: f64::from(h.max_error) * 360.0 / 4096.0,
                    directions: h.directions.clone(),
                }),
                updated_at_us: state.started.elapsed().as_micros() as u64,
            })));
            // robot.busStatus serves the startup report while its phase is a startup one
            // (`configuring_bus` from main.rs), and commissioning never replaced it: clients saw
            // "configuring_bus" forever and never this snapshot (`ready`, `last_sample`). Once a
            // read has succeeded, drop it — the same thing motion does after its first complete
            // observation (`motion_report_started` in main.rs).
            if total_success > 0 {
                state.bus_status.store(None);
            }
            next_publish = tick + Duration::from_millis(100);
        }
        if tick.elapsed() > period {
            state.missed.fetch_add(1, Ordering::Relaxed);
        }
    }
    cancel_hold(&mut hold, "service stopping");
    let stop_error = abort_motion(&mut bus, &ids, &mut motion, "service stopping");
    if let Some(error) = stop_error {
        tracing::error!(%error,"shutdown torque-off not confirmed");
    }
    if let Some(pending) = verifying.take() {
        let _ = pending
            .reply
            .send(Err("service stopping; nothing saved".into()));
    }
    state.bus_commands.cancel_all("service stopping");
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Optimizing the motion Value cache must not shadow commissioning's typed
    /// snapshot or change its existing serialization and error/status fields.
    #[tokio::test]
    async fn typed_bus_status_keeps_priority_and_original_wire_response() {
        let state = Arc::new(RobotState::new(
            &crate::Params::default(),
            Path::new("/nonexistent/robotd.toml"),
            false,
            false,
        ));
        state.commissioning.store(true, Ordering::Relaxed);
        state.raw.publish(crate::raw_stream::tests::fixture(99));
        state
            .bus_status
            .store(Some(Arc::new(json!({"phase":"older-value-cache"}))));
        let report = Status {
            configured_imu_profile: "legacy",
            imu_bus: None,
            build: BUILD,
            port: "/dev/test-no-uart".into(),
            mode: "commissioning",
            phase: "read_error",
            session: 123,
            target_hz: 50.0,
            achieved_hz: 49.5,
            total_cycles: 42,
            total_success: 40,
            total_communication_failures: 2,
            errors_by_id: [(22, 2)].into_iter().collect(),
            device_communication: crate::communication_stats::CommunicationStats::new([22, 200]),
            last_cycle_error: Some("回包缺失".into()),
            last_failure: Some(Failure {
                cycle: 42,
                error: "missing 22".into(),
                missing_ids: vec![22],
            }),
            last_sample: None,
            calibrated: true,
            imu_mount_verified: true,
            imu_verification: None,
            torque_state: "off",
            torque_off_pending: None,
            power_pending: false,
            motion: None,
            hold: None,
            policy_motion_enabled: false,
            updated_at_us: 840_000,
        };
        let id = proto::Id::Text("typed-report".into());
        let expected = proto::Response::ok(Some(id.clone()), &report);
        state.bus_snapshot.store(Some(Arc::new(report)));
        let line = crate::tests::ipc_response(
            state,
            json!({"jsonrpc":"2.0", "id":id, "method":"robot.busStatus"}),
        )
        .await;
        assert_eq!(
            line,
            format!("{}\n", serde_json::to_string(&expected).unwrap())
        );
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn workbench_moves_from_signed_feedback_to_the_requested_positive_target() {
        use duck_control::feetech::{READ, SYNC_WRITE, packet};
        use std::fs::File;
        use std::io::{Read, Write};
        use std::os::fd::FromRawFd;
        let (mut master, mut slave) = (0, 0);
        let mut name = [0 as libc::c_char; 128];
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
        let path = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }
            .to_str()
            .unwrap()
            .to_owned();
        let _slave_file = unsafe { File::from_raw_fd(slave) };
        let mut master_file = unsafe { File::from_raw_fd(master) };
        assert_ne!(
            unsafe { libc::fcntl(master, libc::F_SETFL, libc::O_NONBLOCK) },
            -1
        );
        let stop = Arc::new(AtomicBool::new(false));
        let written = Arc::new(Mutex::new(Vec::new()));
        let worker_stop = Arc::clone(&stop);
        let worker_written = Arc::clone(&written);
        let worker = std::thread::spawn(move || {
            let mut registers = [0u8; 96];
            let mut pending = Vec::new();
            let mut bytes = [0u8; 512];
            while !worker_stop.load(Ordering::Relaxed) {
                match master_file.read(&mut bytes) {
                    Ok(n) => pending.extend_from_slice(&bytes[..n]),
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                    other => panic!("PTY read: {other:?}"),
                }
                while pending.len() >= 4 && pending.len() >= usize::from(pending[3]) + 4 {
                    let len = usize::from(pending[3]) + 4;
                    let frame: Vec<u8> = pending.drain(..len).collect();
                    assert_eq!(frame[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b)), 255);
                    let address = usize::from(frame[5]);
                    if frame[4] == SYNC_WRITE {
                        let width = usize::from(frame[6]);
                        for row in frame[7..len - 1].chunks_exact(width + 1) {
                            assert_eq!(row[0], 22);
                            registers[address..address + width].copy_from_slice(&row[1..]);
                            worker_written
                                .lock()
                                .unwrap()
                                .push((address, row[1..].to_vec()));
                        }
                    } else {
                        assert_eq!((frame[2], frame[4]), (22, READ));
                        let count = usize::from(frame[6]);
                        master_file
                            .write_all(
                                &packet(22, 0, &registers[address..address + count]).unwrap(),
                            )
                            .unwrap();
                    }
                }
                std::thread::sleep(Duration::from_micros(200));
            }
        });
        let sample = |position| {
            let mut raw = [0u8; 15];
            raw[..2].copy_from_slice(&signed_position_word(position).unwrap().to_le_bytes());
            ServoSample::decode(&raw).unwrap()
        };
        let mut samples = [(22, sample(-1323))].into_iter().collect();
        let (reply, mut receive) = oneshot::channel();
        let mut motion = Some(Motion {
            remaining: VecDeque::new(),
            active_id: 22,
            from: -1323,
            target: 2276,
            started: Instant::now(),
            duration: Duration::from_secs(1),
            reply,
            completed: vec![],
            center: false,
            stage: MotionStage::Preload,
            last_target: None,
            torque_confirmed: Some(false),
            deadline: Instant::now() + Duration::from_secs(10),
        });
        let mut bus = Bus::open(&path).unwrap();
        let mut off_pending = None;
        for _ in 0..4 {
            step_motion(&mut bus, &samples, &[22], &mut motion, &mut off_pending);
        }
        assert!(matches!(
            motion.as_ref().unwrap().stage,
            MotionStage::Moving
        ));
        assert_eq!(
            bus.read_control(22, 42, 2).unwrap(),
            signed_position_word(-1323).unwrap().to_le_bytes()
        );
        assert_eq!(
            written.lock().unwrap()[..2],
            [(42, vec![0x2b, 0x85, 0, 0, 0, 0]), (40, vec![1])]
        );
        motion.as_mut().unwrap().started = Instant::now() - Duration::from_secs(1);
        // Arrival uses the requested raw value, never its modulo-equivalent.
        samples.insert(22, sample(-1820));
        step_motion(&mut bus, &samples, &[22], &mut motion, &mut off_pending);
        assert!(matches!(
            motion.as_ref().unwrap().stage,
            MotionStage::Moving
        ));
        assert_eq!(
            bus.read_control(22, 42, 6).unwrap(),
            [0xe4, 0x08, 0, 0, 0, 0]
        );
        samples.insert(22, sample(2276));
        for _ in 0..3 {
            step_motion(&mut bus, &samples, &[22], &mut motion, &mut off_pending);
        }
        assert!(motion.is_none() && off_pending.is_none());
        assert_eq!(
            receive.try_recv().unwrap().unwrap()["completed_ids"],
            json!([22])
        );
        assert_eq!(bus.read_control(22, 40, 1).unwrap(), [0]);
        stop.store(true, Ordering::Relaxed);
        worker.join().unwrap();
    }

    #[test]
    fn pending_power_blocks_new_motion_but_allows_confirmed_stop() {
        let commands = Commands::default();
        commands.power_pending.store(true, Ordering::Relaxed);
        let (reply, _receive) = oneshot::channel();
        assert!(
            commands
                .push(Job {
                    command: BusCommand::Move {
                        id: 20,
                        position: 2048,
                        duration_ms: 2000
                    },
                    reply
                })
                .is_err()
        );
        let (reply, _receive) = oneshot::channel();
        commands
            .push(Job {
                command: BusCommand::Relax,
                reply,
            })
            .unwrap();
        assert!(matches!(commands.pop().unwrap().command, BusCommand::Relax));
        assert!(matches!(
            serde_json::from_value::<BusCommand>(json!({"action":"reboot"})).unwrap(),
            BusCommand::Reboot {}
        ));
        assert!(
            serde_json::from_value::<BusCommand>(json!({"action":"reboot","command":"arbitrary"}))
                .is_err()
        );
    }

    #[test]
    fn capture_uses_known_angle_and_direction_without_enabling_motion() {
        let calibration = Calibration::commissioning(JOINT_IDS);
        let mut bytes = [0u8; 15];
        bytes[..2].copy_from_slice(&2300u16.to_le_bytes());
        let samples = [(JOINT_IDS[0], ServoSample::decode(&bytes).unwrap())]
            .into_iter()
            .collect();
        let changed = capture(&calibration, &samples, JOINT_IDS[0], 22.5, -1, -90.0, 90.0).unwrap();
        assert_eq!(changed.joints[0].zero_raw, 2556);
        assert_eq!(changed.joints[0].direction, -1);
        assert!(changed.joints[0].calibrated);
        assert!(!changed.motion_enabled);
        assert!(capture(&calibration, &samples, JOINT_IDS[0], 0.0, 1, 90.0, -90.0).is_err());
        assert!(
            capture(
                &calibration,
                &samples,
                JOINT_IDS[0],
                f64::NAN,
                1,
                -90.0,
                90.0
            )
            .is_err()
        );
    }

    #[test]
    fn relax_cancels_pending_movement_and_takes_priority() {
        let commands = Commands::default();
        let (reply, mut cancelled) = oneshot::channel();
        commands
            .push(Job {
                command: BusCommand::Move {
                    id: 10,
                    position: 2048,
                    duration_ms: 2000,
                },
                reply,
            })
            .unwrap();
        let (reply, _receive) = oneshot::channel();
        commands
            .push(Job {
                command: BusCommand::Relax,
                reply,
            })
            .unwrap();
        assert!(matches!(commands.pop().unwrap().command, BusCommand::Relax));
        assert!(cancelled.try_recv().unwrap().is_err());
    }

    fn temp_calibration(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("microduck-{name}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir.join("calibration.toml")
    }

    /// Hamilton product, scalar-first, for building sensor readings.
    fn mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
        let ([aw, ax, ay, az], [bw, bx, by, bz]) = (a, b);
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    }

    /// The trunk pitched `degrees` nose-down (about +Y), as a sensor mounted `mount` reports it:
    /// sensor = trunk · mount.
    fn pitched(mount: [f64; 4], degrees: f64) -> [f64; 4] {
        let half = degrees.to_radians() / 2.0;
        mul([half.cos(), 0.0, half.sin(), 0.0], mount)
    }

    fn reading(quat: [f64; 4], gyro: [f64; 3]) -> SensorReading {
        SensorReading {
            gyro,
            quat: Some(quat),
        }
    }

    const BNO: [f64; 4] = duck_control::trunk_imu::BNO08X_DEFAULT_MOUNT;

    /// The CLI help states the limit the check enforces, and the bus board's own rule, so the
    /// two cannot drift apart again (it said ≤ 10° after the module limit became 5°).
    #[test]
    fn verify_imu_help_states_both_rules() {
        let command = BusCommand::augment_subcommands(clap::Command::new("bus"));
        let verify = command.find_subcommand("verify-imu").unwrap();
        let help = verify
            .get_long_about()
            .or(verify.get_about())
            .unwrap()
            .to_string();
        assert!(
            help.contains(&format!("within {MODULE_UPRIGHT_DEG}°")),
            "{help}"
        );
        assert!(help.contains("bus board"), "{help}");
    }

    /// The upright half: under the mount being confirmed the trunk must read within 5° of
    /// upright — whatever it reads here stays as a tilt bias, so half the HOME guard's 10° —
    /// with what it read echoed back: gravity, tilt and the yaw rate's sign.
    #[test]
    fn the_upright_check_holds_a_module_to_five_degrees() {
        // Sensor Y is trunk Z in the BNO08x default mount: a yaw rate.
        let upright = upright_check(
            TrunkImuKind::Bno08x,
            BNO,
            Some(reading(pitched(BNO, 4.0), [0.0, 0.3, 0.0])),
        )
        .unwrap();
        assert!((upright.tilt_deg - 4.0).abs() < 1e-9);
        let echo = &upright.echo;
        assert_eq!(echo["imu_source"], "bno08x");
        assert!(
            (echo["tilt_deg"].as_f64().unwrap() - 4.0).abs() < 1e-9,
            "{echo}"
        );
        assert!((echo["gyro_rad_s"][2].as_f64().unwrap() - 0.3).abs() < 1e-9);
        let error = upright_check(
            TrunkImuKind::Bno08x,
            BNO,
            Some(reading(pitched(BNO, 7.0), [0.0; 3])),
        )
        .unwrap_err();
        assert!(
            error.contains("7.0°") && error.contains("limit 5°") && error.contains("gravity"),
            "{error}"
        );
        assert!(!error.contains("  "), "a broken line continuation: {error}");
        let error =
            upright_check(TrunkImuKind::Lsm6dsv16x, [1.0, 0.0, 0.0, 0.0], None).unwrap_err();
        assert!(error.contains("live lsm6dsv16x sample"), "{error}");
    }

    /// The reviewer's probe. A breakout mounted as identity, confirmed under a mount a half or a
    /// quarter turn off about the vertical: both read upright, so the upright check alone took
    /// them — and then a 15° nose-down pitch read as tail-down, or as a roll, and the policy
    /// would correct the wrong way. The nose-down tip refuses both, naming how the lean read;
    /// the right mount reads it as +x, gravity ≈ [0.26, 0, -0.97].
    #[test]
    fn a_mount_a_quarter_or_half_turn_off_about_the_vertical_is_refused() {
        let truth = [1.0, 0.0, 0.0, 0.0];
        let upright = reading(truth, [0.0; 3]);
        let level = [0.0, 0.0, -1.0];
        let nose_down_15 = reading(pitched(truth, 15.0), [0.0; 3]);
        let half = std::f64::consts::FRAC_1_SQRT_2;
        for (mount, says) in [
            ([0.0, 0.0, 0.0, 1.0], "tail-down"),
            ([half, 0.0, 0.0, half], "side going down"),
            ([half, 0.0, 0.0, -half], "side going down"),
        ] {
            assert!(
                upright_check(TrunkImuKind::Lsm6dsv16x, mount, Some(upright)).is_ok(),
                "a yaw error reads upright: {mount:?}"
            );
            match nose_down(mount, nose_down_15, level) {
                NoseDown::Wrong(message) => {
                    assert!(message.contains(says), "{mount:?}: {message}");
                    assert!(!message.contains("  "), "{message}");
                }
                other => panic!("{mount:?} accepted: {other:?}"),
            }
        }
        match nose_down(truth, nose_down_15, level) {
            NoseDown::Forward(echo) => {
                let g = &echo["gravity"];
                assert!((g[0].as_f64().unwrap() - 15f64.to_radians().sin()).abs() < 1e-9);
                assert!(g[1].as_f64().unwrap().abs() < 1e-9);
                assert!((g[2].as_f64().unwrap() + 15f64.to_radians().cos()).abs() < 1e-9);
            }
            other => panic!("the right mount refused: {other:?}"),
        }
        assert!(matches!(
            nose_down(truth, reading(pitched(truth, 5.0), [0.0; 3]), level),
            NoseDown::Level
        ));
    }

    /// The tip is judged from where the robot stood. Standing 4.9° rolled (inside the upright
    /// limit) and tipped 12.5° nose-down, gravity from vertical points 22° off straight ahead
    /// and was refused as the right side going down; measured from the upright reading it is
    /// straight ahead. The lean is from upright too: the same 12.5° counts, a 9° one does not.
    #[test]
    fn the_tip_is_judged_from_where_the_robot_stood() {
        let truth = [1.0, 0.0, 0.0, 0.0];
        let rolled = |degrees: f64| {
            let half = degrees.to_radians() / 2.0;
            [half.cos(), half.sin(), 0.0, 0.0]
        };
        let stood = rolled(4.9);
        let upright = upright_check(
            TrunkImuKind::Lsm6dsv16x,
            truth,
            Some(reading(stood, [0.0; 3])),
        )
        .unwrap();
        assert!((upright.tilt_deg - 4.9).abs() < 1e-9);
        // Tipped about the robot's own pitch axis: stood · pitch.
        let tipped = |degrees: f64| {
            let half = degrees.to_radians() / 2.0;
            reading(mul(stood, [half.cos(), 0.0, half.sin(), 0.0]), [0.0; 3])
        };
        match nose_down(truth, tipped(12.5), upright.gravity) {
            NoseDown::Forward(echo) => {
                assert!(
                    echo["lean_heading_deg"].as_f64().unwrap().abs() < 1.0,
                    "{echo}"
                );
                // A shade under 12.5°: the pitch axis is itself tilted by the roll.
                let tilt = echo["tilt_from_upright_deg"].as_f64().unwrap();
                assert!((tilt - 12.5).abs() < 0.1, "{echo}");
            }
            other => panic!("a straight tip from a slightly rolled stance refused: {other:?}"),
        }
        assert!(matches!(
            nose_down(truth, tipped(9.0), upright.gravity),
            NoseDown::Level
        ));
    }

    fn pending_for(mount: [f64; 4], now: Instant) -> PendingVerify {
        let (reply, _receive) = oneshot::channel();
        let upright = upright_check(
            TrunkImuKind::Lsm6dsv16x,
            mount,
            Some(reading(mount, [0.0; 3])),
        )
        .unwrap();
        PendingVerify::new(reply, mount, upright, now)
    }

    /// The wait: a verdict needs five tipped readings in a row (a jolt is not a lean), a read
    /// that failed contributes nothing, and with no tip inside the window nothing is saved.
    #[test]
    fn verification_waits_for_a_held_tip_and_gives_up() {
        let truth = [1.0, 0.0, 0.0, 0.0];
        let now = Instant::now();
        let pending = || pending_for(truth, now);
        let level = reading(truth, [0.0; 3]);
        let tipped = reading(pitched(truth, 20.0), [0.0; 3]);
        let mut held = pending();
        for _ in 0..4 {
            assert!(held.observe(Some(tipped), now).is_none());
        }
        assert!(
            held.observe(None, now).is_none(),
            "a failed read is not a reading"
        );
        assert!(held.observe(Some(tipped), now).unwrap().is_ok());
        let mut jolted = pending();
        for _ in 0..3 {
            jolted.observe(Some(tipped), now);
        }
        jolted.observe(Some(level), now);
        for _ in 0..4 {
            assert!(
                jolted.observe(Some(tipped), now).is_none(),
                "a jolt restarts the count"
            );
        }
        let mut held = pending();
        for _ in 0..4 {
            held.observe(Some(tipped), now);
        }
        let check = held.observe(Some(tipped), now).unwrap().unwrap();
        assert!(
            check["residual_tilt_deg"].as_f64().unwrap().abs() < 1e-9,
            "{check}"
        );
        let mut never = pending();
        let error = never
            .observe(Some(level), now + NOSE_DOWN_WINDOW)
            .unwrap()
            .unwrap_err();
        assert!(
            error.contains("nothing saved") && !error.contains("  "),
            "{error}"
        );
    }

    /// The reviewer's probe: tipped throughout, four readings read sideways and the fifth
    /// forward. Counting every tipped reading took the fifth as the verdict and saved a mount a
    /// quarter turn off; only readings running that agree decide — five forward accept, five
    /// the wrong way refuse, and a change of mind starts the count again.
    #[test]
    fn only_readings_running_that_agree_decide_the_tip() {
        let truth = [1.0, 0.0, 0.0, 0.0];
        let now = Instant::now();
        let sideways = {
            let half = 15f64.to_radians() / 2.0;
            reading([half.cos(), half.sin(), 0.0, 0.0], [0.0; 3])
        };
        let forward = reading(pitched(truth, 15.0), [0.0; 3]);
        let mut wobbly = pending_for(truth, now);
        for _ in 0..4 {
            assert!(wobbly.observe(Some(sideways), now).is_none());
        }
        assert!(
            wobbly.observe(Some(forward), now).is_none(),
            "one forward reading accepted"
        );
        for _ in 0..3 {
            assert!(wobbly.observe(Some(forward), now).is_none());
        }
        assert!(
            wobbly.observe(Some(forward), now).unwrap().is_ok(),
            "five forward running"
        );
        let mut wrong = pending_for(truth, now);
        for _ in 0..4 {
            assert!(wrong.observe(Some(sideways), now).is_none());
        }
        let error = wrong.observe(Some(sideways), now).unwrap().unwrap_err();
        assert!(error.contains("side going down"), "{error}");
    }

    /// `hold-home` stands on the policy's HOME: each joint's saved zero, turned by the
    /// direction the caller gives (a fresh image's saved ones are placeholders) or the saved one.
    #[test]
    fn hold_home_targets_follow_zero_direction_and_home() {
        let mut calibration = Calibration::commissioning(JOINT_IDS);
        let home = duck_control::deployment::home();
        let counts = |rad: f64| rad * 4096.0 / std::f64::consts::TAU;
        let (saved, used) = home_targets(&calibration, &[]).unwrap();
        assert_eq!(used, vec![1; NUM_JOINTS]);
        for i in 0..NUM_JOINTS {
            assert_eq!(saved[i], (2048.0 + counts(home[i])).round() as i32);
        }
        let mut flipped = vec![1i8; NUM_JOINTS];
        flipped[5] = -1;
        calibration.joints[5].zero_raw = 2000;
        let (targets, used) = home_targets(&calibration, &flipped).unwrap();
        assert_eq!(used, flipped);
        assert_eq!(targets[5], (2000.0 - counts(home[5])).round() as i32);
        assert_eq!(targets[0], saved[0]);
    }

    #[test]
    fn hold_home_refuses_bad_directions_and_limits() {
        let mut calibration = Calibration::commissioning(JOINT_IDS);
        assert!(home_targets(&calibration, &[1, -1]).unwrap_err().contains("exactly"));
        let mut zero = vec![1i8; NUM_JOINTS];
        zero[3] = 0;
        assert!(home_targets(&calibration, &zero).is_err());
        // left_hip_pitch HOME is about -26°: a saved range that excludes it refuses.
        calibration.joints[2].min_rad = Some(0.0);
        calibration.joints[2].max_rad = Some(1.0);
        assert!(home_targets(&calibration, &[]).unwrap_err().contains("limits"));
    }

    #[test]
    fn hold_home_parses_with_and_without_its_options() {
        let bare: BusCommand = serde_json::from_value(json!({"action":"hold-home"})).unwrap();
        assert!(matches!(bare, BusCommand::HoldHome { ref directions, duration_ms: 3000 } if directions.is_empty()));
        let full: BusCommand = serde_json::from_value(
            json!({"action":"hold-home","directions":[1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1],"duration_ms":2500}),
        )
        .unwrap();
        assert!(matches!(full, BusCommand::HoldHome { ref directions, duration_ms: 2500 } if directions[1] == -1));
        assert!(serde_json::from_value::<BusCommand>(json!({"action":"hold-home","speed":1})).is_err());
    }

    /// A verified mount lands in that module's slot; the bus board's mount is not touched, and
    /// motion stays off until enabled again.
    #[test]
    fn a_verified_module_mount_lands_in_its_own_slot() {
        let path = temp_calibration("verify-module");
        let calibration = Calibration::commissioning(JOINT_IDS);
        let (updated, status) = save_verified(
            &calibration,
            &path,
            TrunkImuKind::Bno08x,
            BNO,
            Some(json!({"checked": true})),
        )
        .unwrap();
        assert!(updated.imu_mount(TrunkImuKind::Bno08x).verified);
        assert!(
            !updated.imu_mount_verified,
            "the bus board's slot is untouched"
        );
        assert_eq!(updated.imu_mount_quat, calibration.imu_mount_quat);
        assert!(!updated.motion_enabled);
        assert_eq!(status["imu_check"]["checked"], true);
        assert_eq!(status["imu_source"], "bno08x");
        let saved: Calibration = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert!(saved.imu_mount(TrunkImuKind::Bno08x).verified);
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    /// The calibration report names the mount in use for a module — its slot, or its default
    /// while uncommissioned — so the console's IMU panel shows and confirms that one, not the
    /// bus board's top-level keys it used to read on every robot. A bus mount remains
    /// effectively unverified until discovery also checks its protocol/profile binding.
    #[test]
    fn the_calibration_report_names_a_modules_mount_in_use() {
        let path = Path::new("/tmp/unused-calibration.toml");
        let mut calibration = Calibration::commissioning(JOINT_IDS);
        let fresh = calibration_status(&calibration, path, TrunkImuKind::Bno08x);
        assert_eq!(fresh["imu_mount"]["quat"], json!(BNO), "{fresh}");
        assert_eq!(fresh["imu_mount"]["verified"], false);
        calibration.set_imu_mount(
            TrunkImuKind::Lsm6dsv16x,
            ImuMount {
                quat: [0.0, 0.0, 0.0, 1.0],
                verified: true,
            },
        );
        let lsm = calibration_status(&calibration, path, TrunkImuKind::Lsm6dsv16x);
        assert_eq!(
            lsm["imu_mount"],
            json!({"quat": [0.0, 0.0, 0.0, 1.0], "verified": true})
        );
        let bus = calibration_status(&calibration, path, TrunkImuKind::Bus);
        assert_eq!(bus["imu_mount"]["verified"], false, "{bus}");
    }

    /// The bus board keeps its old rule: the operator's confirmation, no reading required, the
    /// mount in the top-level keys where every existing file has it.
    #[test]
    fn verify_imu_for_the_bus_board_is_unchanged() {
        let path = temp_calibration("verify-bus");
        let calibration = Calibration::commissioning(JOINT_IDS);
        let mount = [0.0, 1.0, 0.0, 0.0];
        let (updated, status) =
            save_verified(&calibration, &path, TrunkImuKind::Bus, mount, None).unwrap();
        assert!(updated.imu_mount_verified);
        assert_eq!(updated.imu_mount_quat, mount);
        assert!(updated.imu_mounts.bno08x.is_none());
        assert!(status.get("imu_check").is_none());
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(!text.contains("imu_mounts"), "{text}");
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    /// Commissioning with a module, end to end through the socket: ID 200 is neither
    /// identified nor read, every Sync Read is the fifteen servos, and the published IMU is the
    /// module's. `verify-imu` under a mount a half turn off about the vertical passes the
    /// upright check, then refuses once the robot is tipped nose-down and saves nothing; while
    /// one waits other commands are refused and relax cancels it; one whose requester gave up
    /// saves nothing either; under the right mount the same tip verifies the module's slot.
    #[cfg(target_os = "linux")]
    #[test]
    fn commissioning_with_a_module_never_addresses_id_200() {
        use duck_control::feetech::{READ, SYNC_ADDRESS, SYNC_LENGTH, SYNC_WRITE, packet};
        use std::fs::File;
        use std::io::{Read, Write};
        use std::os::fd::FromRawFd;

        /// A breakout mounted as identity, holding whatever orientation the test sets.
        struct Breakout {
            sequence: std::sync::atomic::AtomicU32,
            quat: Arc<Mutex<[f64; 4]>>,
        }
        impl HostImu for Breakout {
            fn kind(&self) -> TrunkImuKind {
                TrunkImuKind::Lsm6dsv16x
            }
            fn latest(&self) -> Option<duck_control::trunk_imu::HostImuSample> {
                let now = Instant::now();
                Some(duck_control::trunk_imu::HostImuSample {
                    quat: *self.quat.lock().unwrap(),
                    gyro: [0.0; 3],
                    received_at: now,
                    sequence: self.sequence.fetch_add(1, Ordering::Relaxed) + 1,
                    epoch: 1,
                    epoch_started_at: now - Duration::from_secs(5),
                    accuracy: None,
                    fusion: None,
                })
            }
            fn health(&self) -> duck_control::trunk_imu::HostImuHealth {
                duck_control::trunk_imu::HostImuHealth::default()
            }
        }

        let (mut master, mut slave) = (0, 0);
        let mut name = [0 as libc::c_char; 128];
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
        let _slave_file = unsafe { File::from_raw_fd(slave) };
        let mut wire = unsafe { File::from_raw_fd(master) };
        assert_ne!(
            unsafe { libc::fcntl(master, libc::F_SETFL, libc::O_NONBLOCK) },
            -1
        );
        let stop = Arc::new(AtomicBool::new(false));
        let device_stop = Arc::clone(&stop);
        let device = std::thread::spawn(move || {
            let mut pending = Vec::new();
            let mut torque = [0u8; 256];
            let mut bytes = [0u8; 512];
            while !device_stop.load(Ordering::Relaxed) {
                match wire.read(&mut bytes) {
                    Ok(n) => pending.extend_from_slice(&bytes[..n]),
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                    other => panic!("PTY read: {other:?}"),
                }
                while pending.len() >= 4 && pending.len() >= usize::from(pending[3]) + 4 {
                    let len = usize::from(pending[3]) + 4;
                    let frame: Vec<u8> = pending.drain(..len).collect();
                    let (address, width) = (frame[5], usize::from(frame[6]));
                    match frame[4] {
                        SYNC_WRITE => {
                            assert_eq!(address, 40, "only torque is written here");
                            for row in frame[7..len - 1].chunks_exact(width + 1) {
                                assert_ne!(row[0], IMU_ID);
                                torque[usize::from(row[0])] = row[1];
                            }
                        }
                        READ => panic!("unicast read of ID {}", frame[2]),
                        _ => {
                            let ids = &frame[7..len - 1];
                            assert_eq!(ids, JOINT_IDS, "only the servos, at {address}");
                            let mut replies = Vec::new();
                            for id in ids {
                                let mut data = vec![0u8; width];
                                if address == 40 {
                                    data[0] = torque[usize::from(*id)];
                                } else {
                                    assert_eq!(
                                        (address, width),
                                        (SYNC_ADDRESS, usize::from(SYNC_LENGTH))
                                    );
                                    data[..2].copy_from_slice(&2048u16.to_le_bytes());
                                }
                                replies.extend(packet(*id, 0, &data).unwrap());
                            }
                            wire.write_all(&replies).unwrap();
                        }
                    }
                }
                std::thread::sleep(Duration::from_micros(200));
            }
        });

        let truth = [1.0, 0.0, 0.0, 0.0];
        let quat = Arc::new(Mutex::new(truth));
        let path = temp_calibration("commission-module");
        // Commissioning no longer writes a default file at startup; start from one on disk, as a
        // robot being commissioned does, so "nothing saved" below reads what was there.
        save_calibration(&path, &Calibration::commissioning(JOINT_IDS)).unwrap();
        let state = Arc::new(RobotState::new(
            &crate::Params::default(),
            Path::new("/tmp/unused-robotd.toml"),
            false,
            false,
        ));
        state.commissioning.store(true, Ordering::Relaxed);
        let (control_state, control_path, control_quat) =
            (Arc::clone(&state), path.clone(), Arc::clone(&quat));
        let control = std::thread::spawn(move || {
            let host = Breakout {
                sequence: std::sync::atomic::AtomicU32::new(0),
                quat: control_quat,
            };
            tokio::runtime::Builder::new_current_thread()
                .enable_time()
                .build()
                .unwrap()
                .block_on(run(
                    &port,
                    "auto",
                    "legacy",
                    &control_state,
                    Duration::from_millis(20),
                    &control_path,
                    Some(&host),
                ));
        });
        let snapshot = || {
            state
                .bus_snapshot
                .load_full()
                .and_then(|status| serde_json::to_value(status.as_ref()).ok())
                .unwrap_or_default()
        };
        let wait = |what: &str, done: &dyn Fn(&Value) -> bool| {
            let until = Instant::now() + Duration::from_secs(5);
            loop {
                let status = snapshot();
                if done(&status) {
                    return status;
                }
                assert!(Instant::now() < until, "{what}: {status}");
                std::thread::sleep(Duration::from_millis(10));
            }
        };
        let imu = wait("a valid sample", &|s| {
            s["last_sample"]["imu"]["valid"] == true
        })["last_sample"]["imu"]
            .clone();
        assert_eq!(imu["source"], "lsm6dsv16x");
        assert!(imu.get("id").is_none(), "{imu}");
        assert!(imu["tilt_deg"].as_f64().unwrap() < 1e-6);

        let verify = |params: Value| {
            let state = Arc::clone(&state);
            std::thread::spawn(move || {
                let reply = tokio::runtime::Runtime::new().unwrap().block_on(dispatch(
                    &state,
                    proto::Id::Number(1),
                    params,
                ));
                serde_json::to_value(reply).unwrap()
            })
        };
        // A half turn off about the vertical: upright passes, the nose-down tip refuses.
        let wrong =
            verify(json!({"action":"verify-imu","confirmed":true,"mount_wxyz":[0.0,0.0,0.0,1.0]}));
        wait("the upright check", &|s| s["imu_verification"].is_string());
        *quat.lock().unwrap() = pitched(truth, 15.0);
        let reply = wrong.join().unwrap();
        let error = reply["error"]["message"]
            .as_str()
            .unwrap_or_default()
            .to_owned();
        assert!(error.contains("tail-down"), "{reply}");
        let saved: Calibration = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert!(
            !saved.imu_mount(TrunkImuKind::Lsm6dsv16x).verified,
            "nothing saved"
        );
        let upright_again = |s: &Value| {
            s["last_sample"]["imu"]["tilt_deg"]
                .as_f64()
                .is_some_and(|t| t < 1.0)
                && s["imu_verification"].is_null()
        };

        // While it waits, other commands are refused (a capture or a move would run with the
        // operator holding the robot tipped), and relax cancels it with nothing saved.
        *quat.lock().unwrap() = truth;
        wait("upright again", &upright_again);
        let relaxed = verify(json!({"action":"verify-imu","confirmed":true}));
        wait("the upright check", &|s| s["imu_verification"].is_string());
        let refused = verify(json!({"action":"verify-imu","confirmed":true}))
            .join()
            .unwrap();
        assert!(
            refused["error"]["message"]
                .as_str()
                .is_some_and(|m| m.contains("waiting for the nose-down tip")),
            "{refused}"
        );
        let relax = verify(json!({"action":"relax"})).join().unwrap();
        assert!(relax["result"].is_object(), "{relax}");
        let reply = relaxed.join().unwrap();
        assert!(
            reply["error"]["message"]
                .as_str()
                .is_some_and(|m| m.contains("cancelled by relax")),
            "{reply}"
        );

        // The right mount, but the requester gives up before the tip (as an RPC that timed out
        // does): the tip saves nothing, or a mount would be verified with nobody told.
        *quat.lock().unwrap() = truth;
        wait("upright again", &upright_again);
        let (reply, abandoned) = oneshot::channel();
        state
            .bus_commands
            .push(Job {
                command: BusCommand::VerifyImu {
                    confirmed: true,
                    mount_wxyz: Vec::new(),
                },
                reply,
            })
            .unwrap();
        wait("the upright check", &|s| s["imu_verification"].is_string());
        drop(abandoned);
        *quat.lock().unwrap() = pitched(truth, 15.0);
        wait("the abandoned wait to end", &|s| {
            s["imu_verification"].is_null()
        });
        std::thread::sleep(Duration::from_millis(200));
        let saved: Calibration = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert!(
            !saved.imu_mount(TrunkImuKind::Lsm6dsv16x).verified,
            "an abandoned verification saves nothing"
        );

        // The right mount, the same tip: verified.
        *quat.lock().unwrap() = truth;
        wait("upright again", &upright_again);
        let right = verify(json!({"action":"verify-imu","confirmed":true}));
        wait("the upright check", &|s| s["imu_verification"].is_string());
        *quat.lock().unwrap() = pitched(truth, 15.0);
        let reply = right.join().unwrap();
        let nose = &reply["result"]["imu_check"]["nose_down"];
        assert!(nose["gravity"][0].as_f64().unwrap() > 0.25, "{reply}");
        assert!(reply["result"]["motion_validation"].is_string());
        state.shutdown.store(true, Ordering::Relaxed);
        control.join().unwrap();
        stop.store(true, Ordering::Relaxed);
        device.join().unwrap();
        let saved: Calibration = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert!(saved.imu_mount(TrunkImuKind::Lsm6dsv16x).verified);
        assert_eq!(saved.imu_mount(TrunkImuKind::Lsm6dsv16x).quat, truth);
        assert!(
            !saved.imu_mount_verified,
            "the bus board's slot is untouched"
        );
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
}
