// Added 2026-10 by fanhao375 (microduck-replica): UDP pad ([netpad]) and pad-loss sit; ported from Pollen Robotics microduck 0.17.1 netpadd/src/health.rs (Apache-2.0); see IMU-MODULE.md at the repository root.
//! What the robot is — its health and its posture — kept fresh for the driving client's report
//! ([`super::wire::RobotReport`]).
//!
//! **Off the pad loop, on connections of their own**, as upstream's `netpadd/src/health.rs` does
//! with two tokio tasks; here two threads. A `robot.health` answer can take `robotd` up to a second,
//! and the pad loop must never wait a second for a report nobody's safety rests on. A thread that
//! cannot reach `robotd` writes why into the snapshot and tries again — it never ends `padd`,
//! because a robot with no health to report still drives.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use duck_ipc_proto as proto;

use super::wire::Posture;

/// How often `robot.health` is asked: as often as the report goes out.
const HEALTH_EVERY: Duration = super::wire::REPORT_EVERY;
/// The rate `robot.state` is subscribed at. Posture changes on a human timescale.
const STATE_HZ: u32 = 2;
/// No `robot.state` for this long is a stream that stopped.
const STATE_SILENT: Duration = Duration::from_secs(3);
/// Between attempts to reach `robotd` again.
const RETRY: Duration = Duration::from_secs(1);
/// A health answer slower than this is a robotd that is not answering.
const HEALTH_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Debug, Clone, Default)]
pub struct Snapshot {
    pub health: Option<proto::HealthResult>,
    pub health_error: Option<String>,
    pub posture: Option<Posture>,
    pub posture_error: Option<String>,
}

/// The newest snapshot, shared with the two threads that keep it.
#[derive(Clone)]
pub struct Health(Arc<Mutex<Snapshot>>);

impl Health {
    pub fn snapshot(&self) -> Snapshot {
        self.0.lock().map(|s| s.clone()).unwrap_or_default()
    }

    fn update(&self, f: impl FnOnce(&mut Snapshot)) {
        if let Ok(mut s) = self.0.lock() {
            f(&mut s);
        }
    }
}

/// Start both threads. A thread that cannot be spawned leaves its half of the report saying so.
pub fn spawn(socket: &Path) -> Health {
    let health = Health(Arc::new(Mutex::new(Snapshot {
        health_error: Some("not asked yet".to_owned()),
        posture_error: Some("not asked yet".to_owned()),
        ..Default::default()
    })));
    let (h, s) = (health.clone(), socket.to_path_buf());
    if let Err(e) = std::thread::Builder::new()
        .name("netpad-health".into())
        .spawn(move || poll_health(s, h))
    {
        tracing::warn!(error = %e, "no health thread — the UDP pad's report carries no health");
    }
    let (h, s) = (health.clone(), socket.to_path_buf());
    if let Err(e) = std::thread::Builder::new()
        .name("netpad-state".into())
        .spawn(move || follow_state(s, h))
    {
        tracing::warn!(error = %e, "no state thread — the UDP pad's report carries no posture");
    }
    health
}

fn poll_health(socket: PathBuf, health: Health) {
    let set = |result: Option<proto::HealthResult>, error: Option<String>| {
        health.update(|s| (s.health, s.health_error) = (result, error));
    };
    loop {
        match ask_health_forever(&socket, &set) {
            Ok(()) => {}
            Err(e) => set(None, Some(format!("robot.health: {e}"))),
        }
        std::thread::sleep(RETRY);
    }
}

/// Ask once a second on one connection until it fails; the error says how.
fn ask_health_forever(
    socket: &Path,
    set: &impl Fn(Option<proto::HealthResult>, Option<String>),
) -> std::io::Result<()> {
    let stream = UnixStream::connect(socket)
        .map_err(|e| std::io::Error::new(e.kind(), format!("cannot reach robotd: {e}")))?;
    stream.set_read_timeout(Some(HEALTH_TIMEOUT))?;
    stream.set_write_timeout(Some(HEALTH_TIMEOUT))?;
    let mut writer = stream.try_clone()?;
    let mut reader = BufReader::new(stream);
    let mut id = 1u64;
    let mut text = String::new();
    loop {
        let mut line = serde_json::to_vec(&proto::Request::call(
            proto::Id::Number(id),
            &proto::Call::RobotHealth,
        ))?;
        line.push(b'\n');
        writer.write_all(&line)?;
        writer.flush()?;
        text.clear();
        if reader.read_line(&mut text)? == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "robotd closed the connection",
            ));
        }
        match serde_json::from_str::<proto::Response>(&text) {
            Ok(r) if r.id == Some(proto::Id::Number(id)) => {
                match r.result_as::<proto::HealthResult>() {
                    Ok(h) => set(Some(h), None),
                    Err(e) => set(None, Some(format!("unreadable robot.health: {e}"))),
                }
            }
            Ok(_) => set(
                None,
                Some("robot.health answered with another id".to_owned()),
            ),
            Err(e) => set(None, Some(format!("unparsable robot.health: {e}"))),
        }
        id += 1;
        std::thread::sleep(HEALTH_EVERY);
    }
}

fn follow_state(socket: PathBuf, health: Health) {
    loop {
        let why = match subscribe(&socket, &health) {
            Ok(()) => "robotd closed the state stream".to_owned(),
            Err(e) => e.to_string(),
        };
        health.update(|s| (s.posture, s.posture_error) = (None, Some(why)));
        std::thread::sleep(RETRY);
    }
}

/// Follow `robot.state` until it stops; `Ok` when `robotd` closed it.
fn subscribe(socket: &Path, health: &Health) -> std::io::Result<()> {
    let stream = UnixStream::connect(socket)
        .map_err(|e| std::io::Error::new(e.kind(), format!("cannot reach robotd: {e}")))?;
    stream.set_read_timeout(Some(STATE_SILENT))?;
    stream.set_write_timeout(Some(STATE_SILENT))?;
    let call = proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(STATE_HZ) });
    let mut line = serde_json::to_vec(&proto::Request::call(proto::Id::Number(1), &call))?;
    line.push(b'\n');
    (&stream).write_all(&line)?;
    let mut reader = BufReader::new(stream);
    let mut text = String::new();
    loop {
        text.clear();
        match reader.read_line(&mut text) {
            Ok(0) => return Ok(()),
            Ok(_) => {}
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    format!("no robot.state for {} s", STATE_SILENT.as_secs()),
                ));
            }
            Err(e) => return Err(e),
        }
        if let Some(p) = posture(&text) {
            health.update(|s| (s.posture, s.posture_error) = (Some(p), None));
        }
    }
}

/// The posture in one `robot.state` notification; `None` for any other line — the subscribe
/// acknowledgement, or a state too unlike this build's to read. Only the four fields this needs
/// are read, so a field added to the state, or one this robotd lacks, cannot cost the posture.
fn posture(line: &str) -> Option<Posture> {
    let v: serde_json::Value = serde_json::from_str(line).ok()?;
    if v.get("method")?.as_str()? != proto::method::ROBOT_STATE {
        return None;
    }
    let state = v.get("params")?;
    let safety = state.get("safety")?;
    let flag = |k: &str| safety.get(k).and_then(|f| f.as_bool()).unwrap_or(false);
    Some(Posture {
        policy: state.get("policy")?.as_str()?.to_owned(),
        fallen: flag("fallen"),
        limp: flag("limp"),
        picked_up: flag("picked_up"),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_posture_is_read_from_a_state_line_and_nothing_else() {
        let state = r#"{"jsonrpc":"2.0","method":"robot.state","params":{"policy":"walk","safety":{"fallen":true,"limp":false,"gravity":[0,0,-1]},"joints":[1,2]}}"#;
        let p = posture(state).expect("a posture");
        assert_eq!(p.policy, "walk");
        assert!(
            p.fallen && !p.limp && !p.picked_up,
            "absent picked_up is false"
        );
        assert_eq!(posture(r#"{"jsonrpc":"2.0","id":1,"result":{}}"#), None);
        assert_eq!(posture("not json"), None);
    }

    /// Against a stand-in robotd: health is asked on its own connection and lands in the
    /// snapshot; a robotd that is not there is said, not fatal.
    #[test]
    fn health_lands_in_the_snapshot_and_a_missing_robotd_is_said() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.sock");
        let listener = std::os::unix::net::UnixListener::bind(&path).unwrap();
        std::thread::spawn(move || {
            for stream in listener.incoming() {
                let Ok(stream) = stream else { return };
                std::thread::spawn(move || {
                    let mut reader = BufReader::new(stream.try_clone().unwrap());
                    let mut out = stream;
                    let mut line = String::new();
                    while reader.read_line(&mut line).unwrap_or(0) > 0 {
                        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
                        let id = v["id"].clone();
                        let answer = if v["method"] == "robot.health" {
                            serde_json::json!({"jsonrpc":"2.0","id":id,"result":{"healthy":true,"cpu_temp_c":61.5}})
                        } else {
                            serde_json::json!({"jsonrpc":"2.0","id":id,"result":{}})
                        };
                        writeln!(out, "{answer}").unwrap();
                        if v["method"] == "robot.subscribe" {
                            let state = serde_json::json!({"jsonrpc":"2.0","method":"robot.state","params":{"policy":"stand","safety":{"fallen":false,"limp":true}}});
                            writeln!(out, "{state}").unwrap();
                        }
                        line.clear();
                    }
                });
            }
        });
        let h = spawn(&path);
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            let s = h.snapshot();
            if let (Some(health), Some(posture)) = (&s.health, &s.posture) {
                assert_eq!(health.cpu_temp_c, Some(61.5));
                assert!(posture.policy == "stand" && posture.limp);
                break;
            }
            assert!(std::time::Instant::now() < deadline, "no snapshot: {s:?}");
            std::thread::sleep(Duration::from_millis(20));
        }

        let gone = spawn(&dir.path().join("absent.sock"));
        std::thread::sleep(Duration::from_millis(200));
        let s = gone.snapshot();
        assert!(s.health.is_none());
        assert!(
            s.health_error
                .as_deref()
                .is_some_and(|e| e.contains("cannot reach robotd")),
            "{s:?}"
        );
    }
}
