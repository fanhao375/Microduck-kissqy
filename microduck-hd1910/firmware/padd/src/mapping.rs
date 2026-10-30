//! Private physical-controller mapping capability, version 1.
use duck_ipc_proto as proto;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
mod session;
pub use session::Engine;
#[cfg(target_os = "linux")]
mod native;
#[cfg(target_os = "linux")]
pub use native::{Runtime, prove_stopped};

pub type Bindings = BTreeMap<String, Binding>;
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Binding {
    Key {
        code: u16,
    },
    AxisButton {
        code: u16,
        value: i32,
    },
    Axis {
        code: u16,
        min: i32,
        max: i32,
        center: i32,
        invert: bool,
        deadzone: f64,
    },
}
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Mapped {
    pub buttons: BTreeMap<String, bool>,
    pub axes: BTreeMap<String, f64>,
}
impl Mapped {
    pub fn from_pad(pad: &crate::PadNow<'_>) -> Self {
        use gilrs::{Axis, Button};
        Self {
            buttons: ROLE_BUTTONS
                .into_iter()
                .map(|(role, b)| (role.into(), pad.is_pressed(b)))
                .collect(),
            axes: [
                ("lx", f64::from(pad.value(Axis::LeftStickX))),
                ("ly", -f64::from(pad.value(Axis::LeftStickY))),
                ("rx", f64::from(pad.value(Axis::RightStickX))),
                ("ry", -f64::from(pad.value(Axis::RightStickY))),
                ("lt", pad.trigger(Button::LeftTrigger2)),
                ("rt", pad.trigger(Button::RightTrigger2)),
            ]
            .into_iter()
            .map(|(r, v)| (r.into(), v))
            .collect(),
        }
    }
    pub fn is_pressed(&self, button: gilrs::Button) -> bool {
        ROLE_BUTTONS
            .iter()
            .find(|(_, b)| *b == button)
            .and_then(|(role, _)| self.buttons.get(*role))
            .copied()
            .unwrap_or(false)
    }
    pub fn value(&self, axis: gilrs::Axis) -> f32 {
        use gilrs::Axis;
        let (role, sign) = match axis {
            Axis::LeftStickX => ("lx", 1.0),
            Axis::LeftStickY => ("ly", -1.0),
            Axis::RightStickX => ("rx", 1.0),
            Axis::RightStickY => ("ry", -1.0),
            _ => ("", 0.0),
        };
        (self.axes.get(role).copied().unwrap_or(0.0) * sign) as f32
    }
    pub fn trigger(&self, button: gilrs::Button) -> f64 {
        let role = match button {
            gilrs::Button::LeftTrigger2 => "lt",
            gilrs::Button::RightTrigger2 => "rt",
            _ => "",
        };
        self.axes.get(role).copied().unwrap_or(0.0)
    }
    pub fn edges(&self, before: &Self) -> Vec<crate::PadEdge> {
        ROLE_BUTTONS
            .into_iter()
            .filter_map(|(_, b)| {
                let a = self.is_pressed(b);
                let old = before.is_pressed(b);
                (a != old).then_some(if a {
                    crate::PadEdge::Pressed(b)
                } else {
                    crate::PadEdge::Released(b)
                })
            })
            .collect()
    }
}
const ROLE_BUTTONS: [(&str, gilrs::Button); 15] = {
    use gilrs::Button::*;
    [
        ("south", South),
        ("east", East),
        ("west", West),
        ("north", North),
        ("lb", LeftTrigger),
        ("rb", RightTrigger),
        ("select", Select),
        ("start", Start),
        ("du", DPadUp),
        ("dd", DPadDown),
        ("dl", DPadLeft),
        ("dr", DPadRight),
        ("l3", LeftThumb),
        ("r3", RightThumb),
        ("home", Mode),
    ]
};
const BUTTON_ROLES: [&str; 15] = [
    "south", "east", "west", "north", "lb", "rb", "select", "start", "du", "dd", "dl", "dr", "l3",
    "r3", "home",
];
const AXIS_ROLES: [&str; 6] = ["lx", "ly", "rx", "ry", "lt", "rt"];
fn map(bindings: &Bindings, device: &proto::PadInputDevice) -> Mapped {
    let mut result = Mapped {
        buttons: BUTTON_ROLES
            .into_iter()
            .map(|r| (r.into(), false))
            .collect(),
        axes: AXIS_ROLES.into_iter().map(|r| (r.into(), 0.0)).collect(),
    };
    for (role, binding) in bindings {
        let value = match binding {
            Binding::Key { code } => {
                f64::from(device.buttons.iter().any(|b| b.code == *code && b.pressed))
            }
            Binding::AxisButton { code, value } => f64::from(
                device
                    .axes
                    .iter()
                    .any(|a| a.code == *code && a.value == *value),
            ),
            Binding::Axis {
                code,
                min,
                max,
                center,
                invert,
                deadzone,
            } => {
                let Some(axis) = device.axes.iter().find(|a| a.code == *code) else {
                    continue;
                };
                let raw = f64::from(axis.value);
                let low = f64::from(*min);
                let high = f64::from(*max);
                let mid = f64::from(*center);
                let v = if role == "lt" || role == "rt" {
                    let v = ((raw - low) / (high - low)).clamp(0.0, 1.0);
                    if *invert { 1.0 - v } else { v }
                } else {
                    let v = if raw >= mid {
                        (raw - mid) / (high - mid)
                    } else {
                        (raw - mid) / (mid - low)
                    };
                    v.clamp(-1.0, 1.0) * if *invert { -1.0 } else { 1.0 }
                };
                let magnitude = if role == "lt" || role == "rt" {
                    v
                } else {
                    v.abs()
                };
                if magnitude <= *deadzone {
                    0.0
                } else {
                    v.signum() * ((magnitude - *deadzone) / (1.0 - *deadzone))
                }
            }
        };
        if AXIS_ROLES.contains(&role.as_str()) {
            result.axes.insert(role.clone(), value);
        } else {
            result.buttons.insert(role.clone(), value != 0.0);
        }
    }
    result
}
fn validate(
    bindings: &Bindings,
    device: &proto::PadInputDevice,
    complete: bool,
) -> Result<(), String> {
    let mut used = std::collections::BTreeSet::new();
    let mut axes = std::collections::BTreeSet::new();
    for (role, binding) in bindings {
        let analog = AXIS_ROLES.contains(&role.as_str());
        let trigger = role == "lt" || role == "rt";
        if !analog && !BUTTON_ROLES.contains(&role.as_str()) {
            return Err(format!("unknown role {role}"));
        }
        match binding {
            Binding::Key { code } => {
                if analog && !trigger
                    || !device.buttons.iter().any(|b| b.code == *code)
                    || !used.insert((1, *code, 0))
                {
                    return Err(format!("invalid or duplicate key for {role}"));
                }
            }
            Binding::AxisButton { code, value } => {
                if analog
                    || ![-1, 1].contains(value)
                    || !device
                        .axes
                        .iter()
                        .any(|a| a.code == *code && a.min == -1 && a.max == 1)
                    || axes.contains(code)
                    || !used.insert((3, *code, *value))
                {
                    return Err(format!("invalid or duplicate hat for {role}"));
                }
            }
            Binding::Axis {
                code,
                min,
                max,
                center,
                deadzone,
                ..
            } => {
                let capability = device.axes.iter().find(|a| a.code == *code);
                if !analog
                    || !deadzone.is_finite()
                    || !(0.0..=0.5).contains(deadzone)
                    || *min >= *max
                    || *center < *min
                    || *center > *max
                    || (!trigger && (*center == *min || *center == *max))
                    || capability.is_none_or(|a| *min < a.min || *max > a.max)
                    || !axes.insert(*code)
                    || used.iter().any(|&(kind, c, _)| kind == 3 && c == *code)
                {
                    return Err(format!("invalid or duplicate axis for {role}"));
                }
            }
        }
    }
    if complete
        && ["select", "start", "lx", "ly", "rx", "ry"]
            .iter()
            .any(|r| !bindings.contains_key(*r))
    {
        return Err("Select, Start and four stick axes are required".into());
    }
    Ok(())
}
fn identity(device: &proto::PadInputDevice) -> String {
    // Two independent FNV streams provide a stable 128-bit capability fingerprint; no process-random hasher.
    let capabilities = serde_json::to_vec(&(
        device.bus,
        device.vendor,
        device.product,
        &device.name,
        &device.unique,
        device
            .axes
            .iter()
            .map(|a| (a.code, a.min, a.max, a.flat, a.fuzz))
            .collect::<Vec<_>>(),
        device.buttons.iter().map(|b| b.code).collect::<Vec<_>>(),
    ))
    .expect("serializable capabilities");
    let hash = |seed: u64| {
        capabilities
            .iter()
            .fold(seed, |h, b| (h ^ u64::from(*b)).wrapping_mul(0x100000001b3))
    };
    format!(
        "evdev-v1-{:04x}-{:04x}-{:04x}-{:016x}{:016x}",
        device.bus,
        device.vendor,
        device.product,
        hash(0xcbf29ce484222325),
        hash(0x84222325cbf29ce4)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    pub(super) fn device() -> proto::PadInputDevice {
        serde_json::from_value(serde_json::json!({"name":"Test physical pad","node":"/dev/input/event7","bus":3,"vendor":4660,"product":22136,
            "axes":(0..6).map(|code| serde_json::json!({"code":code,"name":format!("ABS_{code}"),"min":-100,"max":100,"flat":0,"fuzz":0,"value":0})).chain([serde_json::json!({"code":16,"name":"ABS_HAT0X","min":-1,"max":1,"flat":0,"fuzz":0,"value":0})]).collect::<Vec<_>>(),
            "buttons":[{"code":304,"name":"BTN_SOUTH","pressed":false},{"code":305,"name":"BTN_EAST","pressed":false},{"code":314,"name":"BTN_SELECT","pressed":false},{"code":315,"name":"BTN_START","pressed":false}]})).unwrap()
    }
    fn axis(code: u16) -> Binding {
        Binding::Axis {
            code,
            min: -100,
            max: 100,
            center: 0,
            invert: false,
            deadzone: 0.1,
        }
    }
    pub(super) fn complete() -> Bindings {
        [
            ("select", Binding::Key { code: 314 }),
            ("start", Binding::Key { code: 315 }),
            ("lx", axis(0)),
            ("ly", axis(1)),
            ("rx", axis(2)),
            ("ry", axis(3)),
        ]
        .into_iter()
        .map(|(role, b)| (role.into(), b))
        .collect()
    }
    #[test]
    fn raw_ab_swap_and_normalized_axes_drive_the_same_vocabulary() {
        let mut d = device();
        d.buttons[0].pressed = true;
        d.axes[1].value = 100;
        d.axes[0].value = 5;
        let mut b = complete();
        b.insert("east".into(), Binding::Key { code: 304 });
        b.insert("south".into(), Binding::Key { code: 305 });
        b.insert(
            "lx".into(),
            Binding::Axis {
                code: 0,
                min: -100,
                max: 100,
                center: 0,
                invert: true,
                deadzone: 0.1,
            },
        );
        let m = map(&b, &d);
        assert_eq!(m.buttons.get("east"), Some(&true));
        assert_eq!(m.buttons.get("south"), Some(&false));
        assert_eq!(m.axes.get("lx"), Some(&0.0));
        assert_eq!(m.axes.get("ly"), Some(&1.0));
        d.axes[0].value = -100;
        assert_eq!(map(&b, &d).axes["lx"], 1.0);
    }
    #[test]
    fn logical_mapping_converts_y_once_and_preserves_action_edges() {
        let mut d = device();
        d.buttons[0].pressed = true;
        d.axes[1].value = 100;
        let mut b = complete();
        b.insert("south".into(), Binding::Key { code: 304 });
        b.insert("lt".into(), axis(4));
        let m = map(&b, &d);
        assert!(m.is_pressed(gilrs::Button::South));
        assert_eq!(m.value(gilrs::Axis::LeftStickY), -1.0);
        assert!((m.trigger(gilrs::Button::LeftTrigger2) - 4.0 / 9.0).abs() < 1e-10);
        assert!(
            m.edges(&map(&Bindings::new(), &d))
                .contains(&crate::PadEdge::Pressed(gilrs::Button::South))
        );
        d.buttons[0].pressed = false;
        assert!(
            map(&b, &d)
                .edges(&m)
                .contains(&crate::PadEdge::Released(gilrs::Button::South))
        );
    }
    #[test]
    fn hat_and_digital_or_analog_triggers_normalize() {
        let mut d = device();
        d.axes[6].value = -1;
        d.buttons[0].pressed = true;
        d.axes[4].value = 100;
        let b = Bindings::from([
            (
                "dl".into(),
                Binding::AxisButton {
                    code: 16,
                    value: -1,
                },
            ),
            ("lt".into(), Binding::Key { code: 304 }),
            ("rt".into(), axis(4)),
        ]);
        let m = map(&b, &d);
        assert!(m.buttons["dl"]);
        assert_eq!(m.axes["lt"], 1.0);
        assert_eq!(m.axes["rt"], 1.0);
        d.axes[4].value = -100;
        assert_eq!(map(&b, &d).axes["rt"], 0.0);
    }
    #[test]
    fn binding_validation_rejects_unknown_roles_capabilities_duplicates_and_bad_ranges() {
        let d = device();
        assert!(validate(&complete(), &d, true).is_ok());
        assert!(validate(&Bindings::new(), &d, true).is_err());
        for (role, binding) in [
            ("fly", Binding::Key { code: 304 }),
            ("south", Binding::Key { code: 999 }),
            ("lx", Binding::Key { code: 304 }),
            ("south", axis(4)),
            ("south", Binding::AxisButton { code: 0, value: 1 }),
            ("dl", Binding::AxisButton { code: 16, value: 0 }),
            (
                "lt",
                Binding::Axis {
                    code: 4,
                    min: -200,
                    max: 100,
                    center: 0,
                    invert: false,
                    deadzone: 0.1,
                },
            ),
            (
                "lt",
                Binding::Axis {
                    code: 4,
                    min: -100,
                    max: 100,
                    center: 0,
                    invert: false,
                    deadzone: f64::NAN,
                },
            ),
        ] {
            let mut b = complete();
            b.insert(role.into(), binding);
            assert!(validate(&b, &d, true).is_err(), "accepted {role}: {b:?}");
        }
        let mut b = complete();
        b.insert("south".into(), Binding::Key { code: 314 });
        assert!(validate(&b, &d, true).is_err());
        let mut b = complete();
        b.insert("lt".into(), axis(0));
        assert!(validate(&b, &d, true).is_err());
        let b = Bindings::from([
            (
                "dl".into(),
                Binding::AxisButton {
                    code: 16,
                    value: -1,
                },
            ),
            ("dr".into(), Binding::AxisButton { code: 16, value: 1 }),
        ]);
        assert!(validate(&b, &d, false).is_ok());
    }
    #[test]
    fn stable_identity_separates_bus_capabilities_and_unique_but_not_event_node_or_values() {
        let a = device();
        let mut b = device();
        b.node = "/dev/input/event99".into();
        b.axes[0].value = 50;
        assert_eq!(identity(&a), identity(&b));
        b.bus = 5;
        assert_ne!(identity(&a), identity(&b));
        b = device();
        b.buttons.pop();
        assert_ne!(identity(&a), identity(&b));
        b = device();
        b.unique = Some("serial".into());
        assert_ne!(identity(&a), identity(&b));
    }
}
