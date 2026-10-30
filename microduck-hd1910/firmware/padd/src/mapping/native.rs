use super::*;
use evdev::raw_stream::RawDevice;
use gilrs::LinuxGamepadExt;
use serde_json::{Value, json};
use std::{
    io::{BufRead, BufReader, Read, Write},
    os::unix::{
        fs::{OpenOptionsExt, PermissionsExt},
        net::{UnixListener, UnixStream},
    },
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
        mpsc::{self, Receiver, SyncSender},
    },
    thread,
    time::{Duration, Instant},
};
const MAX_LINE: usize = 32768;
const IO_TIMEOUT: Duration = Duration::from_millis(800);
pub struct Pending {
    pub request: Value,
    pub reply: SyncSender<Value>,
}
pub struct Runtime {
    pub engine: Engine,
    requests: Receiver<Pending>,
    device: Option<RawDevice>,
    wanted: Option<PathBuf>,
    retry: Instant,
    sentinel: PathBuf,
    published: Option<Value>,
    pub edges: Vec<crate::PadEdge>,
}
fn hardware(device: &proto::PadInputDevice, virtual_node: bool) -> bool {
    if device.name == "Microduck R17 Web Xbox" {
        return false;
    }
    device.name == "Xbox 360 Controller (GameSir Nova 2 Lite)"
        || (!virtual_node && [3, 5].contains(&device.bus))
}
fn stopped(state: &Value, homed: bool) -> bool {
    let zero = |v: &Value| {
        v.as_array().is_some_and(|a| {
            a.len() == 3
                && a.iter()
                    .all(|v| v.as_f64().is_some_and(|v| v.is_finite() && v.abs() < 0.001))
        })
    };
    state["policy"] == "held"
        && state["t_ns"].as_u64().is_some_and(|t| t > 0)
        && (homed || state["safety"]["limp"] == true)
        && zero(&state["move"]["requested"])
        && zero(&state["move"]["applied"])
}
fn line_request(reader: impl BufRead) -> Result<Value, String> {
    let mut bytes = Vec::new();
    reader
        .take((MAX_LINE + 1) as u64)
        .read_until(b'\n', &mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > MAX_LINE || bytes.last() != Some(&b'\n') {
        return Err("expected one newline-terminated JSON object, at most 32768 bytes".into());
    }
    let value: Value = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    if !value.is_object() {
        return Err("expected a JSON object".into());
    }
    Ok(value)
}
impl Runtime {
    pub fn serve(socket: &Path, directory: PathBuf) -> std::io::Result<Self> {
        if let Some(parent) = socket.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let _ = std::fs::remove_file(socket);
        let listener = UnixListener::bind(socket)?;
        std::fs::set_permissions(socket, std::fs::Permissions::from_mode(0o660))?;
        group(socket)?;
        let (tx, requests) = mpsc::sync_channel::<Pending>(8);
        thread::Builder::new()
            .name("pad-mapping-accept".into())
            .spawn(move || {
                let clients = Arc::new(AtomicUsize::new(0));
                for stream in listener.incoming().flatten() {
                    if clients.fetch_add(1, Ordering::SeqCst) >= 4 {
                        clients.fetch_sub(1, Ordering::SeqCst);
                        continue;
                    }
                    let tx = tx.clone();
                    let active = Arc::clone(&clients);
                    if let Err(e) = thread::Builder::new()
                        .name("pad-mapping-client".into())
                        .spawn(move || {
                            serve_client(stream, &tx);
                            active.fetch_sub(1, Ordering::SeqCst);
                        })
                    {
                        clients.fetch_sub(1, Ordering::SeqCst);
                        tracing::warn!(error=%e,"cannot serve mapping client");
                    }
                }
            })?;
        let mut runtime = Self {
            engine: Engine::new(directory),
            requests,
            device: None,
            wanted: None,
            retry: Instant::now(),
            sentinel: socket.with_file_name("mapping-state.json"),
            published: None,
            edges: vec![],
        };
        runtime.publish()?;
        Ok(runtime)
    }
    pub fn next(&self) -> Option<Pending> {
        self.requests.try_recv().ok()
    }
    pub fn publish(&mut self) -> std::io::Result<()> {
        let status = self.engine.status(Instant::now());
        let state = json!({"version":1,"pid":std::process::id(),"output_blocked":self.engine.blocked(),"device":status["device"].as_object().map(|d|json!({"id":d["id"],"node":d["node"]})),"profile":status["profile"]});
        if self.published.as_ref() == Some(&state) {
            return Ok(());
        }
        let temp = self.sentinel.with_extension("json.tmp");
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .mode(0o640)
            .open(&temp)?;
        std::fs::set_permissions(&temp, std::fs::Permissions::from_mode(0o640))?;
        group(&temp)?;
        serde_json::to_writer(&mut file, &state)?;
        file.write_all(b"\n")?;
        file.sync_all()?;
        std::fs::rename(&temp, &self.sentinel)?;
        self.published = Some(state);
        Ok(())
    }
    pub fn watch(&mut self, pad: Option<&gilrs::Gamepad<'_>>, now: Instant) {
        self.edges.clear();
        if let Some(pad) = pad {
            self.engine.set_default_axes(
                [
                    gilrs::Axis::LeftStickX,
                    gilrs::Axis::LeftStickY,
                    gilrs::Axis::RightStickX,
                    gilrs::Axis::RightStickY,
                ]
                .into_iter()
                .filter_map(|a| pad.axis_code(a).map(|c| c.into_u32()))
                .filter(|c| c >> 16 == 3)
                .map(|c| (c & 0xffff) as u16)
                .collect(),
            );
        }
        let wanted = pad
            .filter(|p| p.name() != "Microduck R17 Web Xbox")
            .map(|p| p.devpath().to_path_buf());
        if self.wanted != wanted {
            self.device = None;
            if self.wanted.is_some() {
                self.engine.attach(None);
            }
            self.wanted = wanted;
            self.retry = now;
        }
        if self.device.is_none() && now >= self.retry {
            if let Some(node) = &self.wanted {
                match RawDevice::open(node).and_then(|device| {
                    nonblocking(&device)?;
                    let d = describe(&device, node)?;
                    Ok((device, d))
                }) {
                    Ok((device, d)) => {
                        let virtual_node = node
                            .file_name()
                            .and_then(|n| {
                                std::fs::canonicalize(Path::new("/sys/class/input").join(n)).ok()
                            })
                            .is_none_or(|p| p.starts_with("/sys/devices/virtual"));
                        if hardware(&d, virtual_node) {
                            self.engine.attach(Some(d));
                            self.device = Some(device);
                        } else {
                            self.wanted = None;
                        }
                    }
                    Err(e) => {
                        tracing::warn!(error=%e,node=%node.display(),"physical mapping reader unavailable");
                        self.engine.reader_failed();
                    }
                }
            }
            self.retry = now + Duration::from_millis(500);
        }
        let mut failed = false;
        if let Some(device) = self.device.as_mut() {
            // Bound work per tick even if a malicious/defective device never stops reporting.
            for _ in 0..16 {
                match device.fetch_events() {
                    Ok(events) => {
                        for event in events {
                            let before = self.engine.custom();
                            self.engine
                                .event(event.event_type().0, event.code(), event.value());
                            if let (Some(before), Some(after)) = (before, self.engine.custom()) {
                                self.edges.extend(after.edges(&before));
                            }
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => {
                        failed = true;
                        break;
                    }
                }
            }
        }
        if failed || self.engine.needs_resync() {
            self.device = None;
            self.engine.attach(None);
            self.retry = now + Duration::from_millis(100);
        }
    }
}
fn nonblocking(device: &RawDevice) -> std::io::Result<()> {
    use std::os::fd::AsRawFd;
    // This reader belongs only to the control loop. Blocking on an idle pad would also stop
    // mapping commands and lease expiration, so preserve other descriptor flags and add O_NONBLOCK.
    unsafe {
        let flags = libc::fcntl(device.as_raw_fd(), libc::F_GETFL);
        if flags < 0 || libc::fcntl(device.as_raw_fd(), libc::F_SETFL, flags | libc::O_NONBLOCK) < 0
        {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(())
}
fn serve_client(mut stream: UnixStream, tx: &SyncSender<Pending>) {
    let _ = stream.set_read_timeout(Some(IO_TIMEOUT));
    let _ = stream.set_write_timeout(Some(IO_TIMEOUT));
    let response = match socket_request(&mut stream) {
        Err(e) => session::failure("invalid_request", &e),
        Ok(request) => {
            let (reply, rx) = mpsc::sync_channel(1);
            match tx.try_send(Pending { request, reply }) {
                Err(_) => session::failure("busy", "mapping command queue is full"),
                Ok(()) => rx
                    .recv_timeout(Duration::from_millis(1500))
                    .unwrap_or_else(|_| {
                        session::failure(
                            "unavailable",
                            "padd control loop did not acknowledge request",
                        )
                    }),
            }
        }
    };
    let _ = serde_json::to_writer(&mut stream, &response);
    let _ = stream.write_all(b"\n");
}
fn socket_request(stream: &mut UnixStream) -> Result<Value, String> {
    let deadline = Instant::now() + IO_TIMEOUT;
    let mut bytes = Vec::new();
    let mut chunk = [0u8; 1024];
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err("mapping request deadline exceeded".into());
        }
        stream
            .set_read_timeout(Some(remaining))
            .map_err(|e| e.to_string())?;
        let n = stream.read(&mut chunk).map_err(|e| e.to_string())?;
        if n == 0 {
            return Err("mapping request ended before newline".into());
        }
        let end = chunk[..n].iter().position(|b| *b == b'\n').map(|n| n + 1);
        bytes.extend_from_slice(&chunk[..end.unwrap_or(n)]);
        if bytes.len() > MAX_LINE {
            return Err("mapping request is too large".into());
        }
        if end.is_some() {
            return line_request(std::io::Cursor::new(bytes));
        }
    }
}
fn group(path: &Path) -> std::io::Result<()> {
    use std::os::unix::ffi::OsStrExt;
    let group = std::ffi::CString::new("robot").unwrap();
    let path = std::ffi::CString::new(path.as_os_str().as_bytes())?;
    // getgrnam returns static memory. This module is the only group lookup after tap startup.
    unsafe {
        let entry = libc::getgrnam(group.as_ptr());
        if entry.is_null() {
            return Ok(());
        }
        if libc::chown(path.as_ptr(), u32::MAX, (*entry).gr_gid) != 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(())
}
fn describe(device: &RawDevice, node: &Path) -> std::io::Result<proto::PadInputDevice> {
    let id = device.input_id();
    let keys = device.get_key_state()?;
    let axes = device
        .get_absinfo()?
        .map(|(code, a)| proto::PadAxis {
            code: code.0,
            name: format!("{code:?}"),
            min: a.minimum(),
            max: a.maximum(),
            flat: a.flat(),
            fuzz: a.fuzz(),
            value: a.value(),
        })
        .collect::<Vec<_>>();
    let buttons = device
        .supported_keys()
        .map(|keys_cap| {
            keys_cap
                .iter()
                .map(|code| proto::PadKey {
                    code: code.0,
                    name: format!("{code:?}"),
                    pressed: keys.contains(code),
                })
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    let name = device.name().unwrap_or("unnamed controller");
    let unique = device.unique_name();
    if name.len() > 256
        || unique.is_some_and(|s| s.len() > 256)
        || axes.len() > 64
        || buttons.len() > 256
    {
        return Err(std::io::Error::other(
            "controller capability limits exceeded",
        ));
    }
    Ok(proto::PadInputDevice {
        name: name.into(),
        node: node.display().to_string(),
        unique: unique.map(str::to_owned),
        bus: id.bus_type().0,
        vendor: id.vendor(),
        product: id.product(),
        axes,
        buttons,
    })
}
/// A separate read-only robot connection supplies two fresh control ticks immediately before begin.
/// Unknown/missing fields fail closed; no API version change or robot intent is needed.
pub fn prove_stopped(socket: &Path) -> bool {
    let run = || -> Result<bool, String> {
        let deadline = Instant::now() + Duration::from_millis(300);
        let mut stream = connect_proof(socket).map_err(|e| e.to_string())?;
        stream
            .set_read_timeout(Some(Duration::from_millis(300)))
            .map_err(|e| e.to_string())?;
        stream
            .set_write_timeout(Some(Duration::from_millis(300)))
            .map_err(|e| e.to_string())?;
        let policies = json!({"jsonrpc":"2.0","id":1,"method":"robot.busStatus"});
        let subscribe =
            json!({"jsonrpc":"2.0","id":2,"method":"robot.subscribe","params":{"hz":50}});
        for request in [policies, subscribe] {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(false);
            }
            stream
                .set_write_timeout(Some(remaining))
                .map_err(|e| e.to_string())?;
            serde_json::to_writer(&mut stream, &request).map_err(|e| e.to_string())?;
            stream.write_all(b"\n").map_err(|e| e.to_string())?;
        }
        let mut reader = BufReader::new(stream);
        let mut homed = false;
        let mut policy_disabled = false;
        let mut subscribed = false;
        let mut previous = None;
        for _ in 0..16 {
            if Instant::now() >= deadline {
                return Ok(false);
            }
            let v = robot_line(&mut reader, deadline)?;
            if v["id"] == 1 {
                homed = v["result"]["homed"] == true;
                policy_disabled = v["result"]["policy_enabled"].as_bool() == Some(false);
            } else if v["id"] == 2 {
                subscribed = v["result"]["accepted"] == true;
            } else if v["method"] == "robot.state" && subscribed {
                let state = &v["params"];
                if !policy_disabled || !stopped(state, homed) {
                    return Ok(false);
                }
                let at = state["t_ns"].as_u64().unwrap();
                if let Some(prev) = previous {
                    return Ok(at > prev && at - prev <= 100_000_000);
                }
                previous = Some(at);
            }
        }
        Ok(false)
    };
    run().unwrap_or(false)
}
fn connect_proof(path: &Path) -> std::io::Result<UnixStream> {
    use std::os::{
        fd::{AsRawFd, FromRawFd, OwnedFd},
        unix::ffi::OsStrExt,
    };
    let bytes = path.as_os_str().as_bytes();
    // A saturated local listen queue can block ordinary connect indefinitely. Learning may
    // simply refuse that attempt and be retried; it must not freeze the control loop or leak workers.
    unsafe {
        let mut address: libc::sockaddr_un = std::mem::zeroed();
        if bytes.len() >= address.sun_path.len() || bytes.contains(&0) {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "invalid robot socket path",
            ));
        }
        address.sun_family = libc::AF_UNIX as libc::sa_family_t;
        for (slot, byte) in address.sun_path.iter_mut().zip(bytes) {
            *slot = *byte as libc::c_char;
        }
        let raw = libc::socket(
            libc::AF_UNIX,
            libc::SOCK_STREAM | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC,
            0,
        );
        if raw < 0 {
            return Err(std::io::Error::last_os_error());
        }
        let fd = OwnedFd::from_raw_fd(raw);
        let len = std::mem::offset_of!(libc::sockaddr_un, sun_path) + bytes.len() + 1;
        if libc::connect(
            fd.as_raw_fd(),
            (&address as *const libc::sockaddr_un).cast(),
            len as libc::socklen_t,
        ) != 0
        {
            return Err(std::io::Error::last_os_error());
        }
        let stream = UnixStream::from(fd);
        stream.set_nonblocking(false)?;
        Ok(stream)
    }
}
fn robot_line(reader: &mut BufReader<UnixStream>, deadline: Instant) -> Result<Value, String> {
    let mut bytes = Vec::new();
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err("robot proof deadline exceeded".into());
        }
        reader
            .get_ref()
            .set_read_timeout(Some(remaining))
            .map_err(|e| e.to_string())?;
        let available = reader.fill_buf().map_err(|e| e.to_string())?;
        if available.is_empty() {
            return Err("robot proof connection closed".into());
        }
        let end = available.iter().position(|b| *b == b'\n').map(|n| n + 1);
        let n = end.unwrap_or(available.len());
        bytes.extend_from_slice(&available[..n]);
        reader.consume(n);
        if bytes.len() > 131072 {
            return Err("robot proof response too large".into());
        }
        if end.is_some() {
            return serde_json::from_slice(&bytes).map_err(|e| e.to_string());
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn physical_and_known_gamesir_bridge_are_eligible_but_web_virtual_devices_are_not() {
        let mut d = crate::mapping::tests::device();
        assert!(hardware(&d, false));
        d.bus = 6;
        assert!(!hardware(&d, true));
        d.name = "Xbox 360 Controller (GameSir Nova 2 Lite)".into();
        assert!(hardware(&d, true));
        d.name = "Microduck R17 Web Xbox".into();
        d.bus = 3;
        assert!(!hardware(&d, false));
    }
    #[test]
    fn stopped_proof_rejects_active_transition_movement_missing_and_nonfinite_state() {
        let good = json!({"policy":"held","t_ns":1,"move":{"requested":[0,0,0],"applied":[0,0,0]},"safety":{"limp":false}});
        assert!(stopped(&good, true));
        assert!(!stopped(&good, false));
        let mut v = good.clone();
        v["policy"] = json!("homing");
        assert!(!stopped(&v, true));
        v["policy"] = json!("walk");
        assert!(!stopped(&v, true));
        v = good.clone();
        v["move"]["requested"][0] = json!(0.1);
        assert!(!stopped(&v, true));
        assert!(!stopped(&json!({}), true));
    }
    #[test]
    fn one_line_request_is_bounded_rejects_partial_multiline_and_nonobjects() {
        assert_eq!(
            line_request(std::io::Cursor::new(b"{\"op\":\"status\"}\n")).unwrap()["op"],
            "status"
        );
        assert!(line_request(std::io::Cursor::new(b"{\"op\":\"status\"}")).is_err());
        assert!(line_request(std::io::Cursor::new(b"[]\n")).is_err());
        assert!(line_request(std::io::Cursor::new(vec![b' '; 32770])).is_err());
    }
    #[test]
    fn actual_unix_capability_and_voice_state_publish_no_token_with_private_permissions() {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("mapping.sock");
        let mut runtime = Runtime::serve(&socket, dir.path().join("profiles")).unwrap();
        assert_eq!(
            std::fs::metadata(&socket).unwrap().permissions().mode() & 0o777,
            0o660
        );
        runtime.engine.attach(Some(crate::mapping::tests::device()));
        let id = identity(&crate::mapping::tests::device());
        let client = thread::spawn(move || {
            let mut s = UnixStream::connect(socket).unwrap();
            s.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
            serde_json::to_writer(&mut s, &json!({"op":"begin","device_id":id})).unwrap();
            s.write_all(b"\n").unwrap();
            line_request(BufReader::new(s)).unwrap()
        });
        let until = Instant::now() + Duration::from_secs(1);
        let pending = loop {
            if let Some(p) = runtime.next() {
                break p;
            }
            assert!(Instant::now() < until);
            thread::sleep(Duration::from_millis(2));
        };
        let response = runtime
            .engine
            .command(pending.request, true, Instant::now());
        runtime.publish().unwrap();
        let sentinel = dir.path().join("mapping-state.json");
        let state: Value = serde_json::from_slice(&std::fs::read(&sentinel).unwrap()).unwrap();
        assert_eq!(state["version"], 1);
        assert_eq!(state["pid"], std::process::id());
        assert_eq!(state["output_blocked"], true);
        assert_eq!(state["device"]["node"], "/dev/input/event7");
        assert!(state["profile"].is_null());
        assert!(state.get("token").is_none());
        assert_eq!(
            std::fs::metadata(&sentinel).unwrap().permissions().mode() & 0o777,
            0o640
        );
        let modified = std::fs::metadata(&sentinel).unwrap().modified().unwrap();
        runtime.publish().unwrap();
        assert_eq!(
            modified,
            std::fs::metadata(&sentinel).unwrap().modified().unwrap()
        );
        pending.reply.try_send(response).unwrap();
        let received = client.join().unwrap();
        assert_eq!(received["ok"], true);
        assert_eq!(received["token"].as_str().unwrap().len(), 64);
    }
    #[test]
    fn slow_socket_client_times_out_without_blocking_control_loop() {
        let (client, server) = UnixStream::pair().unwrap();
        let (tx, rx) = mpsc::sync_channel(1);
        let started = Instant::now();
        let worker = thread::spawn(move || serve_client(server, &tx));
        client
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let reply = line_request(BufReader::new(client)).unwrap();
        worker.join().unwrap();
        assert_eq!(reply["ok"], false);
        assert_eq!(reply["code"], "invalid_request");
        assert!(rx.try_recv().is_err());
        assert!(started.elapsed() < Duration::from_secs(2));
    }
    #[test]
    fn byte_dribbling_cannot_extend_the_total_request_deadline() {
        let (client, server) = UnixStream::pair().unwrap();
        let mut dribble = client.try_clone().unwrap();
        let (tx, _rx) = mpsc::sync_channel(1);
        let worker = thread::spawn(move || serve_client(server, &tx));
        let sender = thread::spawn(move || {
            for _ in 0..30 {
                if dribble.write_all(b" ").is_err() {
                    break;
                }
                thread::sleep(Duration::from_millis(100));
            }
        });
        client
            .set_read_timeout(Some(Duration::from_secs(4)))
            .unwrap();
        let started = Instant::now();
        let reply = line_request(BufReader::new(client)).unwrap();
        assert_eq!(reply["ok"], false);
        assert!(started.elapsed() < Duration::from_millis(1400));
        worker.join().unwrap();
        sender.join().unwrap();
    }
    fn robot_proof_fixture(policy: &str, homed: bool, moving: bool, second_ns: u64) -> bool {
        robot_enabled_fixture(policy, homed, moving, second_ns, Some(false))
    }
    fn robot_enabled_fixture(
        policy: &str,
        homed: bool,
        moving: bool,
        second_ns: u64,
        enabled: Option<bool>,
    ) -> bool {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("robot.sock");
        let server = UnixListener::bind(&socket).unwrap();
        let policy = policy.to_owned();
        let worker = thread::spawn(move || {
            let (s, _) = server.accept().unwrap();
            let mut r = BufReader::new(s);
            let mut methods = vec![];
            for _ in 0..2 {
                let mut line = String::new();
                r.read_line(&mut line).unwrap();
                methods.push(
                    serde_json::from_str::<Value>(&line).unwrap()["method"]
                        .as_str()
                        .unwrap()
                        .to_owned(),
                );
            }
            assert_eq!(methods, vec!["robot.busStatus", "robot.subscribe"]);
            let states = [
                // robotd::raw_stream::DynamicBus flattens BusDynamic into the report.
                json!({"jsonrpc":"2.0","id":1,"result":{"homed":homed,"policy_enabled":enabled}}),
                json!({"jsonrpc":"2.0","id":2,"result":{"accepted":true}}),
                json!({"jsonrpc":"2.0","method":"robot.state","params":{"policy":policy,"t_ns":1_000_000_000u64,"move":{"requested":[if moving{0.1}else{0.0},0,0],"applied":[0,0,0]},"safety":{"limp":false}}}),
                json!({"jsonrpc":"2.0","method":"robot.state","params":{"policy":policy,"t_ns":second_ns,"move":{"requested":[0,0,0],"applied":[0,0,0]},"safety":{"limp":false}}}),
            ];
            for v in states {
                if serde_json::to_writer(r.get_mut(), &v).is_err()
                    || r.get_mut().write_all(b"\n").is_err()
                {
                    break;
                }
            }
        });
        let result = prove_stopped(&socket);
        worker.join().unwrap();
        result
    }
    #[test]
    fn real_robot_socket_proof_requires_two_fresh_stopped_ticks_and_sends_no_intents() {
        assert!(robot_proof_fixture("held", true, false, 1_020_000_000));
        assert!(!robot_proof_fixture("walk", true, false, 1_020_000_000));
        assert!(!robot_proof_fixture("homing", true, false, 1_020_000_000));
        assert!(!robot_proof_fixture("held", false, false, 1_020_000_000));
        assert!(!robot_proof_fixture("held", true, true, 1_020_000_000));
        assert!(!robot_proof_fixture("held", true, false, 1_000_000_000));
        assert!(!robot_enabled_fixture(
            "held",
            true,
            false,
            1_020_000_000,
            Some(true)
        ));
        assert!(!robot_enabled_fixture(
            "held",
            true,
            false,
            1_020_000_000,
            None
        ));
    }
    #[test]
    fn actual_robotd_serializer_fixture_accepts_configured_but_stopped_policy() {
        let wire: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/robotd-stopped-wire.json"
        ))
        .unwrap();
        assert_eq!(wire["policies"]["enabled"], true);
        assert_eq!(wire["bus"]["policy_enabled"], false);
        assert!(wire["bus"].get("dynamic").is_none());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robot.sock");
        let listener = UnixListener::bind(&path).unwrap();
        let worker = thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let mut reader = BufReader::new(stream);
            for method in ["robot.busStatus", "robot.subscribe"] {
                let mut line = String::new();
                reader.read_line(&mut line).unwrap();
                assert_eq!(
                    serde_json::from_str::<Value>(&line).unwrap()["method"],
                    method
                );
            }
            for value in [
                json!({"jsonrpc":"2.0","id":1,"result":wire["bus"]}),
                json!({"jsonrpc":"2.0","id":2,"result":{"accepted":true}}),
                json!({"jsonrpc":"2.0","method":"robot.state","params":wire["states"][0]}),
                json!({"jsonrpc":"2.0","method":"robot.state","params":wire["states"][1]}),
            ] {
                serde_json::to_writer(reader.get_mut(), &value).unwrap();
                reader.get_mut().write_all(b"\n").unwrap();
            }
        });
        assert!(prove_stopped(&path));
        worker.join().unwrap();
    }
    #[test]
    fn robot_proof_also_has_an_absolute_deadline_for_dribbling_responses() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robot.sock");
        let listener = UnixListener::bind(&path).unwrap();
        let worker = thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let mut reader = BufReader::new(stream);
            for _ in 0..2 {
                let mut line = String::new();
                reader.read_line(&mut line).unwrap();
            }
            for _ in 0..30 {
                if reader.get_mut().write_all(b" ").is_err() {
                    break;
                }
                thread::sleep(Duration::from_millis(100));
            }
        });
        let now = Instant::now();
        assert!(!prove_stopped(&path));
        assert!(now.elapsed() < Duration::from_secs(1));
        worker.join().unwrap();
    }
    #[test]
    fn robot_proof_connect_does_not_block_behind_a_full_unix_backlog() {
        use std::os::fd::AsRawFd;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robot.sock");
        let listener = UnixListener::bind(&path).unwrap();
        assert_eq!(unsafe { libc::listen(listener.as_raw_fd(), 0) }, 0);
        let _occupied = UnixStream::connect(&path).unwrap();
        let worker = thread::spawn(move || {
            thread::sleep(Duration::from_millis(700));
            drop(listener);
        });
        let now = Instant::now();
        assert!(!prove_stopped(&path));
        assert!(now.elapsed() < Duration::from_millis(500));
        worker.join().unwrap();
    }
    /// Opt-in cross-language fixture. It cannot open a device or robot socket; only this test
    /// function accepts event injection. The public mapping socket uses the production server.
    #[test]
    fn cross_language_fixture() {
        let Some(directory) = std::env::var_os("PAD_MAPPING_FIXTURE_DIR").map(PathBuf::from) else {
            return;
        };
        std::fs::create_dir_all(&directory).unwrap();
        let mut d = crate::mapping::tests::device();
        d.axes.push(proto::PadAxis {
            code: 17,
            name: "ABS_HAT0Y".into(),
            min: -1,
            max: 1,
            flat: 0,
            fuzz: 0,
            value: 0,
        });
        for code in [307u16, 308, 310, 311, 316, 317, 318] {
            d.buttons.push(proto::PadKey {
                code,
                name: format!("BTN_{code}"),
                pressed: false,
            });
        }
        let profiles = directory.join("profiles");
        let socket = directory.join("mapping.sock");
        let mut runtime = Runtime::serve(&socket, profiles.clone()).unwrap();
        runtime.engine.attach(Some(d.clone()));
        let injection = directory.join("events.jsonl");
        std::fs::write(&injection, b"").unwrap();
        let mut events = BufReader::new(std::fs::File::open(&injection).unwrap());
        let mut event_line = String::new();
        let (mut controls, _controls_reader) = UnixStream::pair().unwrap();
        let mut input = crate::PadInput::default();
        let mut continuous = crate::Continuous::default();
        let mut generation = runtime.engine.generation;
        std::fs::write(
            directory.join("ready.json"),
            serde_json::to_vec(&runtime.engine.status(Instant::now())).unwrap(),
        )
        .unwrap();
        let until = Instant::now() + Duration::from_secs(180);
        while Instant::now() < until && !directory.join("stop").exists() {
            loop {
                let n = events.read_line(&mut event_line).unwrap();
                if n == 0 {
                    break;
                }
                if !event_line.ends_with('\n') {
                    break;
                }
                let event: Value = serde_json::from_str(&event_line).unwrap();
                event_line.clear();
                match event["op"].as_str() {
                    Some("reload") => {
                        runtime.engine = Engine::new(profiles.clone());
                        runtime.engine.attach(Some(d.clone()));
                        generation = u64::MAX;
                    }
                    Some("disconnect") => runtime.engine.attach(None),
                    Some("attach") => runtime.engine.attach(Some(d.clone())),
                    _ => runtime.engine.event(
                        event["kind"].as_u64().unwrap() as u16,
                        event["code"].as_u64().unwrap() as u16,
                        event["value"].as_i64().unwrap() as i32,
                    ),
                }
            }
            runtime.engine.tick(Instant::now());
            crate::mapping_control::sync(
                &mut input,
                &mut continuous,
                &mut controls,
                generation != runtime.engine.generation,
                runtime.engine.blocked(),
                || runtime.publish(),
            )
            .unwrap();
            generation = runtime.engine.generation;
            for _ in 0..8 {
                let Some(pending) = runtime.next() else { break };
                let response = runtime
                    .engine
                    .command(pending.request, true, Instant::now());
                crate::mapping_control::sync(
                    &mut input,
                    &mut continuous,
                    &mut controls,
                    generation != runtime.engine.generation,
                    runtime.engine.blocked(),
                    || runtime.publish(),
                )
                .unwrap();
                generation = runtime.engine.generation;
                let _ = pending.reply.try_send(response);
            }
            thread::sleep(Duration::from_millis(2));
        }
        assert!(
            directory.join("stop").exists(),
            "fixture timed out without stop marker"
        );
    }
}
