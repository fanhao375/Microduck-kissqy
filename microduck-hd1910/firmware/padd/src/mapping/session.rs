use super::*;
use serde_json::{Value, json};
use std::{
    collections::VecDeque,
    io::{Read, Write},
    path::PathBuf,
    time::{Duration, Instant},
};
const LEASE: Duration = Duration::from_secs(5);
const NEUTRAL: Duration = Duration::from_millis(300);
const MAX_FILE: u64 = 262_144;
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    schema: u8,
    name: String,
    bindings: Bindings,
}
#[derive(Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Store {
    schema: u8,
    profiles: BTreeMap<String, Profile>,
}
struct Session {
    token: String,
    until: Instant,
    draft: Option<Bindings>,
}
#[derive(Serialize)]
struct Event {
    seq: u64,
    kind: u16,
    code: u16,
    value: i32,
}
pub struct Engine {
    directory: PathBuf,
    store: Store,
    load_error: Option<String>,
    device: Option<proto::PadInputDevice>,
    valid: bool,
    seq: u64,
    events: VecDeque<Event>,
    session: Option<Session>,
    waiting: bool,
    neutral_since: Option<Instant>,
    default: Mapped,
    default_axes: Vec<u16>,
    pub generation: u64,
}
impl Engine {
    pub fn new(directory: PathBuf) -> Self {
        let load = || -> Result<Option<Store>, String> {
            let file = match std::fs::File::open(directory.join("mappings.json")) {
                Ok(f) => f,
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
                Err(e) => return Err(e.to_string()),
            };
            if file.metadata().map_err(|e| e.to_string())?.len() > MAX_FILE {
                return Err("mapping file is too large".into());
            }
            let store: Store =
                serde_json::from_reader(file.take(MAX_FILE + 1)).map_err(|e| e.to_string())?;
            if store.schema != 1
                || store.profiles.len() > 32
                || store.profiles.iter().any(|(id, p)| {
                    id.len() > 128
                        || !id.starts_with("evdev-v1-")
                        || p.schema != 1
                        || !valid_name(&p.name)
                        || p.bindings.len() > 21
                })
            {
                return Err("invalid mapping store".into());
            }
            Ok(Some(store))
        };
        let (store, load_error) = match load() {
            Ok(Some(s)) => (s, None),
            Ok(None) => (
                Store {
                    schema: 1,
                    ..Default::default()
                },
                None,
            ),
            Err(e) => {
                tracing::warn!(error=%e,"invalid controller profiles: writes disabled until the file is repaired and padd restarted");
                (
                    Store {
                        schema: 1,
                        ..Default::default()
                    },
                    Some(e),
                )
            }
        };
        Self {
            directory,
            store,
            load_error,
            device: None,
            valid: false,
            seq: 0,
            events: VecDeque::new(),
            session: None,
            waiting: false,
            neutral_since: None,
            default: map(&Bindings::new(), &empty_device()),
            default_axes: vec![0, 1, 3, 4],
            generation: 0,
        }
    }
    pub fn attach(&mut self, device: Option<proto::PadInputDevice>) {
        if self.profile().is_some() || self.session.is_some() || self.waiting {
            self.wait();
        }
        self.events.clear();
        self.valid = device.is_some();
        self.device = device;
        if self.profile().is_some() {
            self.wait();
        }
    }
    pub fn event(&mut self, kind: u16, code: u16, value: i32) {
        if kind == 0 && code == 3 {
            self.valid = false;
            self.events.clear();
            self.wait();
            return;
        }
        if !self.valid {
            return;
        }
        if let Some(d) = self.device.as_mut() {
            match kind {
                1 => {
                    if let Some(b) = d.buttons.iter_mut().find(|b| b.code == code) {
                        b.pressed = value != 0;
                    }
                }
                3 => {
                    if let Some(a) = d.axes.iter_mut().find(|a| a.code == code) {
                        a.value = value;
                    }
                }
                _ => {}
            }
        }
        if kind == 1 || kind == 3 {
            self.seq = self.seq.saturating_add(1);
            if self.events.len() == 256 {
                self.events.pop_front();
            }
            self.events.push_back(Event {
                seq: self.seq,
                kind,
                code,
                value,
            });
        }
        if self.waiting && !self.neutral() {
            self.neutral_since = None;
        }
    }
    pub fn set_default(&mut self, mapped: Mapped) {
        self.default = mapped;
    }
    pub fn set_default_axes(&mut self, codes: Vec<u16>) {
        self.default_axes = codes;
    }
    pub fn needs_resync(&self) -> bool {
        self.device.is_some() && !self.valid
    }
    pub fn invalidate(&mut self) {
        self.valid = false;
        self.wait();
    }
    pub fn reader_failed(&mut self) {
        let required = self.blocked() || !self.store.profiles.is_empty();
        self.attach(None);
        if required {
            self.invalidate();
        }
    }
    pub fn blocked(&self) -> bool {
        self.session.is_some() || self.waiting
    }
    pub fn requires_stopped_proof(&self, request: &Value) -> bool {
        match serde_json::from_value::<Request>(request.clone()) {
            Ok(Request::Begin { device_id }) => {
                !self.blocked()
                    && self.valid
                    && self
                        .device
                        .as_ref()
                        .is_some_and(|d| identity(d) == device_id)
            }
            _ => false,
        }
    }
    pub fn custom(&self) -> Option<Mapped> {
        self.valid
            .then(|| {
                self.profile()
                    .map(|p| map(&p.bindings, self.device.as_ref().unwrap()))
            })
            .flatten()
    }
    pub fn mapped(&self) -> Mapped {
        self.custom().unwrap_or_else(|| self.default.clone())
    }
    fn profile(&self) -> Option<&Profile> {
        let d = self.device.as_ref()?;
        self.store
            .profiles
            .get(&identity(d))
            .filter(|p| validate(&p.bindings, d, true).is_ok())
    }
    fn wait(&mut self) {
        let changed = self.session.is_some() || !self.waiting;
        self.session = None;
        self.waiting = true;
        self.neutral_since = None;
        if changed {
            self.generation += 1;
        }
    }
    fn neutral(&self) -> bool {
        let Some(d) = self.device.as_ref().filter(|_| self.valid) else {
            return false;
        };
        let raw_sticks_centered = self.profile().is_some()
            || d.axes
                .iter()
                .filter(|a| self.default_axes.contains(&a.code))
                .all(|a| {
                    let span = f64::from(a.max) - f64::from(a.min);
                    let center = (f64::from(a.max) + f64::from(a.min)) / 2.0;
                    (f64::from(a.value) - center).abs() <= span * 0.05
                });
        raw_sticks_centered
            && !d.buttons.iter().any(|b| b.pressed)
            && self
                .mapped()
                .axes
                .values()
                .all(|v| v.is_finite() && v.abs() <= 0.1)
            && d.axes
                .iter()
                .filter(|a| a.min == -1 && a.max == 1)
                .all(|a| a.value == 0)
    }
    pub fn tick(&mut self, now: Instant) {
        if self.session.as_ref().is_some_and(|s| now >= s.until) {
            self.wait();
        }
        if self.waiting {
            if self.neutral() {
                let since = self.neutral_since.get_or_insert(now);
                if now.saturating_duration_since(*since) >= NEUTRAL {
                    self.waiting = false;
                    self.neutral_since = None;
                    self.generation += 1;
                }
            } else {
                self.neutral_since = None;
            }
        }
    }
    pub fn status(&self, now: Instant) -> Value {
        let device = self.device.as_ref().map(|d| {
            let mut v = serde_json::to_value(d).unwrap();
            v["id"] = json!(identity(d));
            v
        });
        let raw=self.device.as_ref().filter(|_|self.valid).map(|d|json!({"seq":self.seq,"buttons":d.buttons.iter().map(|b|(b.code.to_string(),b.pressed)).collect::<BTreeMap<_,_>>(),"axes":d.axes.iter().map(|a|(a.code.to_string(),a.value)).collect::<BTreeMap<_,_>>(),"events":self.events}));
        let mapped = self
            .device
            .as_ref()
            .filter(|_| self.valid)
            .map(|_| self.mapped());
        let preview = self
            .session
            .as_ref()
            .and_then(|s| s.draft.as_ref())
            .zip(self.device.as_ref().filter(|_| self.valid))
            .map(|(b, d)| map(b, d));
        let session = if let Some(s) = &self.session {
            Some(
                json!({"phase":"learning","remaining_ms":s.until.saturating_duration_since(now).as_millis() as u64}),
            )
        } else if self.waiting {
            Some(json!({"phase":"waiting_neutral","remaining_ms":0}))
        } else {
            None
        };
        json!({"device":device,"raw":raw,"mapped":mapped,"preview":preview,"profile":self.profile(),"session":session,"output_blocked":self.blocked()})
    }
    pub fn command(&mut self, request: Value, stopped: bool, now: Instant) -> Value {
        self.tick(now);
        let result = self.handle(request, stopped, now);
        match result {
            Ok(token) => {
                let mut r = json!({"ok":true,"version":1,"status":self.status(now)});
                if let Some(token) = token {
                    r["token"] = json!(token);
                }
                r
            }
            Err((code, error)) => failure(code, &error),
        }
    }
    fn handle(
        &mut self,
        request: Value,
        stopped: bool,
        now: Instant,
    ) -> Result<Option<String>, (&'static str, String)> {
        let req: Request =
            serde_json::from_value(request).map_err(|e| ("invalid_request", e.to_string()))?;
        if matches!(req, Request::Status) {
            return Ok(None);
        }
        if let Request::Begin { device_id } = req {
            if self.blocked() {
                return Err(("busy", "controller output is already paused".into()));
            }
            let d = self
                .device
                .as_ref()
                .filter(|_| self.valid)
                .ok_or(("no_device", "no fresh physical controller".into()))?;
            if device_id != identity(d) {
                return Err(("device_changed", "selected controller changed".into()));
            }
            if !stopped {
                return Err((
                    "not_stopped",
                    "stop the robot policy before learning".into(),
                ));
            }
            let token = random_token().map_err(|e| ("unavailable", e.to_string()))?;
            self.session = Some(Session {
                token: token.clone(),
                until: now + LEASE,
                draft: None,
            });
            self.generation += 1;
            return Ok(Some(token));
        }
        let token = match &req {
            Request::Renew { token }
            | Request::Cancel { token }
            | Request::Reset { token }
            | Request::Preview { token, .. }
            | Request::Save { token, .. } => token,
            _ => unreachable!(),
        };
        if !self.session.as_ref().is_some_and(|s| s.token == *token) {
            return Err((
                "invalid_token",
                "session token is invalid or expired".into(),
            ));
        }
        match req {
            Request::Renew { .. } => self.session.as_mut().unwrap().until = now + LEASE,
            Request::Cancel { .. } => self.wait(),
            Request::Preview { bindings, .. } => {
                validate(&bindings, self.device.as_ref().unwrap(), false)
                    .map_err(|e| ("invalid_bindings", e))?;
                self.session.as_mut().unwrap().draft = Some(bindings);
            }
            Request::Save { name, bindings, .. } => {
                if !valid_name(&name) {
                    return Err((
                        "invalid_request",
                        "profile name must contain 1–128 non-control characters".into(),
                    ));
                }
                let d = self.device.as_ref().unwrap();
                validate(&bindings, d, true).map_err(|e| ("invalid_bindings", e))?;
                let id = identity(d);
                if self.store.profiles.len() >= 32 && !self.store.profiles.contains_key(&id) {
                    return Err(("persistence", "profile limit reached".into()));
                }
                let mut next = self.store.profiles.clone();
                next.insert(
                    id,
                    Profile {
                        schema: 1,
                        name,
                        bindings,
                    },
                );
                self.persist(&next)
                    .map_err(|e| ("persistence", e.to_string()))?;
                self.store.profiles = next;
                self.wait();
            }
            Request::Reset { .. } => {
                let mut next = self.store.profiles.clone();
                next.remove(&identity(self.device.as_ref().unwrap()));
                self.persist(&next)
                    .map_err(|e| ("persistence", e.to_string()))?;
                self.store.profiles = next;
                self.wait();
            }
            _ => unreachable!(),
        }
        Ok(None)
    }
    fn persist(&self, profiles: &BTreeMap<String, Profile>) -> std::io::Result<()> {
        use std::os::unix::fs::OpenOptionsExt;
        if let Some(error) = &self.load_error {
            return Err(std::io::Error::other(format!(
                "existing profile store was not readable: {error}; repair it and restart padd"
            )));
        }
        std::fs::create_dir_all(&self.directory)?;
        let bytes = serde_json::to_vec(&json!({"schema":1,"profiles":profiles}))?;
        if bytes.len() > MAX_FILE as usize {
            return Err(std::io::Error::other("profile store too large"));
        }
        let temp = self
            .directory
            .join(format!(".mappings-{}.tmp", random_token()?));
        let result = (|| {
            let mut file = std::fs::OpenOptions::new()
                .create_new(true)
                .write(true)
                .mode(0o600)
                .open(&temp)?;
            file.write_all(&bytes)?;
            file.sync_all()?;
            std::fs::rename(&temp, self.directory.join("mappings.json"))?;
            std::fs::File::open(&self.directory)?.sync_all()
        })();
        if result.is_err() {
            let _ = std::fs::remove_file(temp);
        }
        result
    }
}
fn empty_device() -> proto::PadInputDevice {
    proto::PadInputDevice {
        name: String::new(),
        node: String::new(),
        unique: None,
        bus: 0,
        vendor: 0,
        product: 0,
        axes: vec![],
        buttons: vec![],
    }
}
fn valid_name(name: &str) -> bool {
    !name.trim().is_empty() && name.len() <= 128 && !name.chars().any(char::is_control)
}
fn random_token() -> std::io::Result<String> {
    let mut bytes = [0u8; 32];
    std::fs::File::open("/dev/urandom")?.read_exact(&mut bytes)?;
    Ok(bytes.iter().map(|b| format!("{b:02x}")).collect())
}
pub fn failure(code: &str, error: &str) -> Value {
    json!({"ok":false,"version":1,"code":code,"error":error})
}
#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
enum Request {
    Status,
    Begin {
        device_id: String,
    },
    Renew {
        token: String,
    },
    Preview {
        token: String,
        bindings: Bindings,
    },
    Save {
        token: String,
        name: String,
        bindings: Bindings,
    },
    Cancel {
        token: String,
    },
    Reset {
        token: String,
    },
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping::tests::{complete, device};
    use std::time::Duration;
    fn setup() -> (tempfile::TempDir, Engine, Instant) {
        let dir = tempfile::tempdir().unwrap();
        let e = Engine::new(dir.path().into());
        (dir, e, Instant::now())
    }
    fn begin(e: &mut Engine, t: Instant) -> String {
        let id = identity(&device());
        let r = e.command(json!({"op":"begin","device_id":id}), true, t);
        assert_eq!(r["ok"], true, "{r}");
        r["token"].as_str().unwrap().to_owned()
    }
    #[test]
    fn begin_requires_stopped_device_and_unpredictable_owned_lease() {
        let (_dir, mut e, t) = setup();
        let id = identity(&device());
        assert_eq!(
            e.command(json!({"op":"begin","device_id":id}), true, t)["code"],
            "no_device"
        );
        e.attach(Some(device()));
        assert_eq!(
            e.command(json!({"op":"begin","device_id":id}), false, t)["code"],
            "not_stopped"
        );
        let token = begin(&mut e, t);
        assert_eq!(token.len(), 64);
        assert!(e.status(t)["output_blocked"].as_bool().unwrap());
        assert!(e.status(t).get("token").is_none());
        assert_eq!(
            e.command(json!({"op":"renew","token":"wrong"}), true, t)["code"],
            "invalid_token"
        );
        assert_eq!(
            e.command(json!({"op":"begin","device_id":id}), true, t)["code"],
            "busy"
        );
        assert_eq!(
            e.command(
                json!({"op":"renew","token":token}),
                true,
                t + Duration::from_secs(4)
            )["ok"],
            true
        );
        e.tick(t + Duration::from_secs(8));
        assert_eq!(
            e.status(t + Duration::from_secs(8))["session"]["phase"],
            "learning"
        );
        e.tick(t + Duration::from_secs(10));
        assert_eq!(
            e.status(t + Duration::from_secs(10))["session"]["phase"],
            "waiting_neutral"
        );
    }
    #[test]
    fn preview_save_restart_reset_and_corruption_use_atomic_profiles() {
        let (dir, mut e, t) = setup();
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        let mut b = complete();
        b.insert("east".into(), Binding::Key { code: 304 });
        assert_eq!(
            e.command(json!({"op":"preview","token":token,"bindings":b}), true, t)["ok"],
            true
        );
        e.event(1, 304, 1);
        assert_eq!(e.status(t)["preview"]["buttons"]["east"], true);
        assert!(!dir.path().join("mappings.json").exists());
        assert_eq!(
            e.command(
                json!({"op":"save","token":token,"name":"My pad","bindings":b}),
                true,
                t
            )["ok"],
            true
        );
        assert_eq!(e.status(t)["session"]["phase"], "waiting_neutral");
        assert!(e.status(t)["output_blocked"].as_bool().unwrap());
        let mut restarted = Engine::new(dir.path().into());
        restarted.attach(Some(device()));
        assert_eq!(restarted.status(t)["profile"]["name"], "My pad");
        restarted.tick(t);
        restarted.tick(t + Duration::from_millis(301));
        assert_eq!(restarted.status(t)["output_blocked"], false);
        let token = begin(&mut restarted, t + Duration::from_secs(1));
        assert_eq!(
            restarted.command(
                json!({"op":"reset","token":token}),
                true,
                t + Duration::from_secs(1)
            )["ok"],
            true
        );
        assert!(restarted.status(t)["profile"].is_null());
        std::fs::write(dir.path().join("mappings.json"), "{broken").unwrap();
        let mut bad = Engine::new(dir.path().into());
        bad.attach(Some(device()));
        assert!(bad.status(t)["profile"].is_null());
    }
    #[test]
    fn save_failure_preserves_live_profile_and_learning_session() {
        let (dir, mut e, t) = setup();
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        std::fs::create_dir(dir.path().join("mappings.json")).unwrap();
        let r = e.command(
            json!({"op":"save","token":token,"name":"failed","bindings":complete()}),
            true,
            t,
        );
        assert_eq!(r["code"], "persistence");
        assert!(e.status(t)["profile"].is_null());
        assert_eq!(e.status(t)["session"]["phase"], "learning");
    }
    #[test]
    fn held_input_cancel_expiry_drop_and_replacement_wait_for_stable_neutral() {
        let (_dir, mut e, t) = setup();
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        e.event(1, 314, 1);
        assert_eq!(
            e.command(json!({"op":"cancel","token":token}), true, t)["ok"],
            true
        );
        e.tick(t + Duration::from_secs(3));
        assert_eq!(e.status(t)["output_blocked"], true);
        e.event(1, 314, 0);
        e.tick(t + Duration::from_secs(4));
        e.tick(t + Duration::from_millis(4301));
        assert_eq!(e.status(t)["output_blocked"], false);
        let _token = begin(&mut e, t + Duration::from_secs(5));
        e.event(0, 3, 0);
        assert!(e.status(t)["raw"].is_null());
        assert_eq!(e.status(t)["session"]["phase"], "waiting_neutral");
        e.attach(None);
        assert_eq!(e.status(t)["output_blocked"], true);
        e.tick(t + Duration::from_secs(10));
        assert_eq!(e.status(t)["output_blocked"], true);
        e.attach(Some(device()));
        e.tick(t + Duration::from_secs(11));
        e.tick(t + Duration::from_millis(11301));
        assert_eq!(e.status(t)["output_blocked"], false);
    }
    #[test]
    fn raw_history_is_bounded_monotonic_and_new_device_has_no_old_samples() {
        let (_dir, mut e, t) = setup();
        e.attach(Some(device()));
        for n in 0..600 {
            e.event(1, 304, n % 2);
        }
        let s = e.status(t);
        let events = s["raw"]["events"].as_array().unwrap();
        assert!(events.len() <= 256);
        assert!(!events.is_empty());
        assert!(events.first().unwrap()["seq"].as_u64().unwrap() > 1);
        let seq = s["raw"]["seq"].as_u64().unwrap();
        e.attach(None);
        e.attach(Some(device()));
        assert_eq!(e.status(t)["raw"]["events"].as_array().unwrap().len(), 0);
        e.event(1, 304, 1);
        assert!(e.status(t)["raw"]["seq"].as_u64().unwrap() > seq);
    }
    #[test]
    fn strict_requests_do_not_accept_unknown_fields_names_or_invalid_profiles() {
        let (_dir, mut e, t) = setup();
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        assert_eq!(
            e.command(json!({"op":"renew","token":token,"surprise":1}), true, t)["code"],
            "invalid_request"
        );
        assert_eq!(
            e.command(
                json!({"op":"save","token":token,"name":"x","bindings":{}}),
                true,
                t
            )["code"],
            "invalid_bindings"
        );
        assert_eq!(
            e.command(
                json!({"op":"save","token":token,"name":"x".repeat(129),"bindings":complete()}),
                true,
                t
            )["code"],
            "invalid_request"
        );
    }
    #[test]
    fn only_a_valid_available_begin_can_spend_time_proving_robot_state() {
        let (_dir, mut e, t) = setup();
        let request = json!({"op":"begin","device_id":identity(&device())});
        assert!(!e.requires_stopped_proof(&request));
        e.attach(Some(device()));
        assert!(e.requires_stopped_proof(&request));
        assert!(!e.requires_stopped_proof(&json!({"op":"begin","device_id":"wrong"})));
        assert!(!e.requires_stopped_proof(
            &json!({"op":"begin","device_id":identity(&device()),"unknown":1})
        ));
        assert!(!e.requires_stopped_proof(&json!({"op":"status"})));
        begin(&mut e, t);
        assert!(!e.requires_stopped_proof(&request));
    }
    #[test]
    fn ordinary_default_disconnect_keeps_existing_pad_loss_behavior() {
        let (_dir, mut e, t) = setup();
        e.attach(Some(device()));
        e.attach(None);
        assert_eq!(e.status(t)["output_blocked"], false);
    }
    #[test]
    fn even_a_short_press_inside_one_tick_restarts_neutral_stability() {
        let (_dir, mut e, t) = setup();
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        e.command(json!({"op":"cancel","token":token}), true, t);
        e.tick(t);
        e.event(1, 304, 1);
        e.event(1, 304, 0);
        e.tick(t + Duration::from_millis(301));
        assert_eq!(e.status(t)["output_blocked"], true);
        e.tick(t + Duration::from_millis(602));
        assert_eq!(e.status(t)["output_blocked"], false);
    }
    #[test]
    fn repeated_reader_failures_do_not_repeat_the_pause_transition() {
        let (_dir, mut e, _t) = setup();
        e.invalidate();
        let generation = e.generation;
        e.invalidate();
        assert_eq!(e.generation, generation);
    }
    #[test]
    fn default_reader_failure_preserves_gilrs_but_saved_profiles_fail_closed() {
        let (dir, mut e, t) = setup();
        e.reader_failed();
        assert!(!e.blocked());
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        e.command(
            json!({"op":"save","token":token,"name":"custom","bindings":complete()}),
            true,
            t,
        );
        let mut restarted = Engine::new(dir.path().into());
        restarted.reader_failed();
        assert!(restarted.blocked());
    }
    #[test]
    fn damaged_existing_store_is_never_overwritten_by_save_or_reset() {
        let (dir, _, t) = setup();
        let path = dir.path().join("mappings.json");
        let damaged = b"{invalid-profile-store";
        std::fs::write(&path, damaged).unwrap();
        let mut e = Engine::new(dir.path().into());
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        assert_eq!(
            e.command(
                json!({"op":"save","token":token,"name":"replacement","bindings":complete()}),
                true,
                t
            )["code"],
            "persistence"
        );
        assert_eq!(
            e.command(json!({"op":"reset","token":token}), true, t)["code"],
            "persistence"
        );
        assert_eq!(std::fs::read(path).unwrap(), damaged);
    }
    #[test]
    fn failed_replacement_save_keeps_existing_profile_and_disk_contents() {
        let (dir, mut e, t) = setup();
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        assert_eq!(
            e.command(
                json!({"op":"save","token":token,"name":"original","bindings":complete()}),
                true,
                t
            )["ok"],
            true
        );
        e.tick(t);
        e.tick(t + Duration::from_millis(301));
        let token = begin(&mut e, t + Duration::from_secs(1));
        let path = dir.path().join("mappings.json");
        let bytes = std::fs::read(&path).unwrap();
        // A directory at the atomic replacement target simulates an actual rename failure.
        std::fs::rename(&path, dir.path().join("original.json")).unwrap();
        std::fs::create_dir(&path).unwrap();
        assert_eq!(
            e.command(
                json!({"op":"save","token":token,"name":"replacement","bindings":complete()}),
                true,
                t + Duration::from_secs(1)
            )["code"],
            "persistence"
        );
        assert_eq!(e.status(t)["profile"]["name"], "original");
        assert_eq!(
            std::fs::read(dir.path().join("original.json")).unwrap(),
            bytes
        );
        assert_eq!(e.status(t)["session"]["phase"], "learning");
    }
    #[cfg(target_os = "linux")]
    #[test]
    fn real_linux_permission_denial_keeps_saved_profile() {
        use std::os::unix::{fs::PermissionsExt, process::CommandExt};
        let check = |path: PathBuf| {
            let before = std::fs::read(path.join("mappings.json")).unwrap();
            let mut e = Engine::new(path.clone());
            let t = Instant::now();
            e.attach(Some(device()));
            e.tick(t);
            e.tick(t + Duration::from_millis(301));
            let token = begin(&mut e, t + Duration::from_secs(1));
            let r = e.command(
                json!({"op":"save","token":token,"name":"should fail","bindings":complete()}),
                true,
                t + Duration::from_secs(1),
            );
            assert_eq!(r["code"], "persistence");
            assert_eq!(e.status(t)["profile"]["name"], "original");
            assert_eq!(std::fs::read(path.join("mappings.json")).unwrap(), before);
        };
        if let Some(path) = std::env::var_os("PADD_PERMISSION_FIXTURE") {
            check(path.into());
            return;
        }
        let (dir, mut e, t) = setup();
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        assert_eq!(
            e.command(
                json!({"op":"save","token":token,"name":"original","bindings":complete()}),
                true,
                t
            )["ok"],
            true
        );
        assert_eq!(
            std::fs::metadata(dir.path().join("mappings.json"))
                .unwrap()
                .permissions()
                .mode()
                & 0o777,
            0o600
        );
        std::fs::set_permissions(
            dir.path().join("mappings.json"),
            std::fs::Permissions::from_mode(0o644),
        )
        .unwrap();
        let binary = dir.path().join("permission-test");
        std::fs::copy(std::env::current_exe().unwrap(), &binary).unwrap();
        std::fs::set_permissions(&binary, std::fs::Permissions::from_mode(0o755)).unwrap();
        std::fs::set_permissions(dir.path(), std::fs::Permissions::from_mode(0o555)).unwrap();
        if unsafe { libc::geteuid() } == 0 {
            let status = std::process::Command::new(binary)
                .args([
                    "--exact",
                    "mapping::session::tests::real_linux_permission_denial_keeps_saved_profile",
                    "--nocapture",
                ])
                .env("PADD_PERMISSION_FIXTURE", dir.path())
                .uid(65534)
                .gid(65534)
                .status()
                .unwrap();
            assert!(status.success());
        } else {
            check(dir.path().into());
        }
        std::fs::set_permissions(dir.path(), std::fs::Permissions::from_mode(0o755)).unwrap();
    }
    #[test]
    fn an_axis_pulse_inside_one_tick_also_restarts_neutral_stability() {
        let (_dir, mut e, t) = setup();
        e.attach(Some(device()));
        let token = begin(&mut e, t);
        e.command(json!({"op":"cancel","token":token}), true, t);
        e.tick(t);
        e.event(3, 0, 100);
        e.event(3, 0, 0);
        e.tick(t + Duration::from_millis(301));
        assert_eq!(e.status(t)["output_blocked"], true);
        e.tick(t + Duration::from_millis(602));
        assert_eq!(e.status(t)["output_blocked"], false);
    }
}
