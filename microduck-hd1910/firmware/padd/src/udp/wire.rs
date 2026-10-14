// Added 2026-10 by fanhao375 (microduck-replica): UDP pad ([netpad]) and pad-loss sit; ported from Pollen Robotics microduck 0.17.1 pad-map/src/wire.rs (Apache-2.0); see IMU-MODULE.md at the repository root.
//! The pad on the wire: one UDP datagram per state, 28 bytes, little-endian.
//!
//! Ported unchanged in its bytes from Pollen microduck 0.17.1 `pad-map/src/wire.rs`, which
//! `docs/robot/udp-pad.md` upstream owns: the point of this file is that the official
//! `duckctl udp-pad` drives this robot without knowing it is not running upstream's `netpadd`.
//! Change nothing here that upstream has not changed first.
//!
//! Every datagram is the pad's *whole* state, never a delta, so a lost one costs nothing — the
//! next replaces it — and nothing ever has to be retransmitted or reordered.
//!
//! [`Status`] is the answer sent back, so a sender can see the link: which datagram it answers,
//! and the receiver's running counts. [`RobotReport`] is the robot's health, sent to the driving
//! client every [`REPORT_EVERY`]. A sender may ignore both entirely.

use duck_ipc_proto as proto;

pub const MAGIC: [u8; 4] = *b"DKPD";
pub const VERSION: u8 = 1;
pub const LEN: usize = 28;

const FULL: f64 = 32767.0;

/// The pad's buttons as a bitset. The bit order **is** the wire's, so a datagram's `buttons`
/// field is a `Buttons` without translation.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Buttons(pub u16);

impl Buttons {
    pub const NONE: Self = Self(0);
    pub const A: Self = Self(1 << 0);
    pub const B: Self = Self(1 << 1);
    pub const X: Self = Self(1 << 2);
    pub const Y: Self = Self(1 << 3);
    pub const LB: Self = Self(1 << 4);
    pub const RB: Self = Self(1 << 5);
    pub const START: Self = Self(1 << 6);
    pub const SELECT: Self = Self(1 << 7);
    pub const UP: Self = Self(1 << 8);
    pub const DOWN: Self = Self(1 << 9);
    pub const LEFT: Self = Self(1 << 10);
    pub const RIGHT: Self = Self(1 << 11);
    /// The middle button — Home, Xbox, PS.
    pub const HOME: Self = Self(1 << 12);
    /// Every bit that means something. The rest are reserved.
    pub const KNOWN: u16 = 0x1FFF;

    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0 && other.0 != 0
    }

    /// Bits set in `self` that were not set in `before` — the presses between two states.
    pub fn newly_set(self, before: Self) -> Self {
        Self(self.0 & !before.0)
    }
}

impl std::ops::BitOr for Buttons {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for Buttons {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Packet {
    /// +1 per datagram, wrapping. The receiver drops anything not newer than what it has, or
    /// more than 1000 ahead of it.
    pub seq: u32,
    /// The sender's own clock, ms. For jitter in logs only; never compared with the robot's.
    pub t_ms: u32,
    /// lx ly rx ry (±32767 is ±1, up positive), lt rt (0..32767 is 0..1).
    pub axes: [i16; 6],
    pub buttons: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    Short(usize),
    Magic,
    Version(u8),
}

impl std::fmt::Display for WireError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Short(n) => write!(f, "{n} bytes, a pad datagram is {LEN}"),
            Self::Magic => write!(f, "not a pad datagram (magic)"),
            Self::Version(v) => write!(f, "pad datagram version {v}, this build reads {VERSION}"),
        }
    }
}

impl Packet {
    #[cfg(test)]
    pub fn encode(&self) -> [u8; LEN] {
        let mut b = [0u8; LEN];
        b[..4].copy_from_slice(&MAGIC);
        b[4] = VERSION;
        b[6..10].copy_from_slice(&self.seq.to_le_bytes());
        b[10..14].copy_from_slice(&self.t_ms.to_le_bytes());
        for (i, a) in self.axes.iter().enumerate() {
            b[14 + 2 * i..16 + 2 * i].copy_from_slice(&a.to_le_bytes());
        }
        b[26..28].copy_from_slice(&self.buttons.to_le_bytes());
        b
    }

    /// Longer than [`LEN`] is fine — appended fields this build does not know are ignored.
    pub fn decode(b: &[u8]) -> Result<Self, WireError> {
        if b.len() < LEN {
            return Err(WireError::Short(b.len()));
        }
        if b[..4] != MAGIC {
            return Err(WireError::Magic);
        }
        if b[4] != VERSION {
            return Err(WireError::Version(b[4]));
        }
        let u32_at = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let mut axes = [0i16; 6];
        for (i, a) in axes.iter_mut().enumerate() {
            *a = i16::from_le_bytes([b[14 + 2 * i], b[15 + 2 * i]]);
        }
        Ok(Self {
            seq: u32_at(6),
            t_ms: u32_at(10),
            axes,
            buttons: u16::from_le_bytes([b[26], b[27]]),
        })
    }

    pub fn held(&self) -> Buttons {
        Buttons(self.buttons & Buttons::KNOWN)
    }

    pub fn reserved(&self) -> u16 {
        self.buttons & !Buttons::KNOWN
    }

    /// Sticks in -1..1 (up positive), triggers in 0..1: `[lx, ly, rx, ry, lt, rt]`.
    pub fn values(&self) -> [f64; 6] {
        let stick = |v: i16| (f64::from(v) / FULL).clamp(-1.0, 1.0);
        let trigger = |v: i16| (f64::from(v) / FULL).clamp(0.0, 1.0);
        let a = self.axes;
        [
            stick(a[0]),
            stick(a[1]),
            stick(a[2]),
            stick(a[3]),
            trigger(a[4]),
            trigger(a[5]),
        ]
    }

    #[cfg(test)]
    /// For a sender: sticks `[lx, ly, rx, ry]` in -1..1, triggers `[lt, rt]` in 0..1.
    pub fn from_axes(
        seq: u32,
        t_ms: u32,
        sticks: [f64; 4],
        triggers: [f64; 2],
        held: Buttons,
    ) -> Self {
        let q = |v: f64, lo: f64| (v.clamp(lo, 1.0) * FULL).round() as i16;
        Self {
            seq,
            t_ms,
            axes: [
                q(sticks[0], -1.0),
                q(sticks[1], -1.0),
                q(sticks[2], -1.0),
                q(sticks[3], -1.0),
                q(triggers[0], 0.0),
                q(triggers[1], 0.0),
            ],
            buttons: held.0,
        }
    }
}

pub const STATUS_MAGIC: [u8; 4] = *b"DKST";
pub const STATUS_VERSION: u8 = 1;
/// No longer than a pad datagram, so an answer never amplifies what it answers.
pub const STATUS_LEN: usize = 24;
const _: () = assert!(
    STATUS_LEN <= LEN,
    "an answer must never outgrow what it answers"
);

/// How often a client that does not hold the robot is told so, at most.
pub const BUSY_EVERY: std::time::Duration = std::time::Duration::from_secs(1);

/// What became of the datagram a [`Status`] answers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// From the client that holds the robot — taken, or dropped as stale.
    Driving,
    /// Another client holds the robot; this one is dropped until it has been quiet `timeout_ms`.
    Busy,
}

impl Verdict {
    fn code(self) -> u8 {
        match self {
            Self::Driving => 1,
            Self::Busy => 2,
        }
    }
}

/// The answer to one pad datagram. The counters are the receiver's own, wrapping, since it
/// started: a sender reads their change, never their value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    pub verdict: Verdict,
    /// The `seq` of the datagram answered.
    pub seq: u32,
    /// Datagrams taken from the driving client.
    pub accepted: u32,
    /// Datagrams from the driving client dropped as not newer, or too far ahead.
    pub stale: u32,
    /// Datagrams that were not pad state at all, from anyone.
    pub malformed: u32,
}

impl Status {
    pub fn encode(&self) -> [u8; STATUS_LEN] {
        let mut b = [0u8; STATUS_LEN];
        b[..4].copy_from_slice(&STATUS_MAGIC);
        b[4] = STATUS_VERSION;
        b[5] = self.verdict.code();
        b[6..10].copy_from_slice(&self.seq.to_le_bytes());
        b[10..14].copy_from_slice(&self.accepted.to_le_bytes());
        b[14..18].copy_from_slice(&self.stale.to_le_bytes());
        b[18..22].copy_from_slice(&self.malformed.to_le_bytes());
        b
    }
}

pub const REPORT_MAGIC: [u8; 4] = *b"DKRB";
pub const REPORT_VERSION: u8 = 1;
/// How often the driving client is sent a [`RobotReport`].
pub const REPORT_EVERY: std::time::Duration = std::time::Duration::from_secs(1);
/// A refusal is reported for this long after it happened, then dropped.
pub const REFUSAL_KEPT: std::time::Duration = std::time::Duration::from_secs(30);

/// What the robot is, as last heard from `robotd`: `magic`, `version`, then JSON.
///
/// JSON because it carries `robotd`'s own [`proto::HealthResult`] unchanged. Every field is
/// optional: an absent one is *not known*, never zero.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RobotReport {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub health: Option<proto::HealthResult>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub health_error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub posture: Option<Posture>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub posture_error: Option<String>,
    /// The last request the robot said no to, within [`REFUSAL_KEPT`].
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refused: Option<Refusal>,
}

/// What the robot is doing, from `robot.state`.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Posture {
    pub policy: String,
    pub fallen: bool,
    pub limp: bool,
    pub picked_up: bool,
}

/// A request from the pad the robot did not carry out, and its reason.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Refusal {
    pub method: String,
    pub reason: String,
    pub ago_ms: u64,
}

impl RobotReport {
    pub fn encode(&self) -> Vec<u8> {
        let mut b = REPORT_MAGIC.to_vec();
        b.push(REPORT_VERSION);
        // A struct of options, strings and numbers: serialising it cannot fail.
        b.extend(serde_json::to_vec(self).unwrap_or_default());
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Packet {
        Packet {
            seq: 0xDEAD_BEEF,
            t_ms: 42,
            axes: [32767, -32767, 0, 100, 32767, 0],
            buttons: 0b1_0100_0001,
        }
    }

    /// Upstream's own bytes for this packet (pad-map wire.rs tests), so a drift from the official
    /// client shows here rather than as a pad that does nothing.
    #[test]
    fn a_packet_round_trips_at_upstreams_offsets() {
        let b = sample().encode();
        assert_eq!(b.len(), 28);
        assert_eq!(&b[..5], b"DKPD\x01");
        assert_eq!(
            u32::from_le_bytes(b[6..10].try_into().unwrap()),
            0xDEAD_BEEF
        );
        assert_eq!(u32::from_le_bytes(b[10..14].try_into().unwrap()), 42);
        assert_eq!(i16::from_le_bytes([b[14], b[15]]), 32767);
        assert_eq!(u16::from_le_bytes([b[26], b[27]]), 0b1_0100_0001);
        assert_eq!(Packet::decode(&b), Ok(sample()));
    }

    #[test]
    fn garbage_is_refused_not_panicked_on() {
        assert_eq!(Packet::decode(&[]), Err(WireError::Short(0)));
        assert_eq!(Packet::decode(&[0u8; 27]), Err(WireError::Short(27)));
        let mut b = sample().encode();
        b[0] = b'X';
        assert_eq!(Packet::decode(&b), Err(WireError::Magic));
        let mut b = sample().encode();
        b[4] = 2;
        assert_eq!(Packet::decode(&b), Err(WireError::Version(2)));
    }

    #[test]
    fn a_longer_v1_packet_is_read_and_its_tail_ignored() {
        let mut long = sample().encode().to_vec();
        long.extend_from_slice(&[9, 9, 9, 9]);
        assert_eq!(Packet::decode(&long), Ok(sample()));
    }

    #[test]
    fn values_clamp_and_triggers_are_never_negative() {
        let p = Packet {
            axes: [i16::MIN, 0, 0, 0, i16::MIN, 0],
            ..Default::default()
        };
        let v = p.values();
        assert_eq!(v[0], -1.0);
        assert_eq!(v[4], 0.0);
        let p = Packet::from_axes(7, 9, [0.5, -1.0, 0.0, 1.0], [1.0, 0.25], Buttons::START);
        let v = p.values();
        assert!((v[0] - 0.5).abs() < 1e-4 && v[1] == -1.0 && v[3] == 1.0);
        assert!((v[4] - 1.0).abs() < 1e-4 && (v[5] - 0.25).abs() < 1e-4);
        assert_eq!(p.held(), Buttons::START);
    }

    #[test]
    fn reserved_bits_are_split_from_buttons() {
        let p = Packet {
            buttons: 0xF001,
            ..Default::default()
        };
        assert_eq!(p.held(), Buttons::A | Buttons::HOME, "bit 12 is Home");
        assert_eq!(p.reserved(), 0xE000);
    }

    /// Upstream's status layout: magic, version, verdict, then four little-endian counters.
    #[test]
    fn a_status_is_upstreams_24_bytes() {
        let s = Status {
            verdict: Verdict::Busy,
            seq: 0xDEAD_BEEF,
            accepted: 7,
            stale: 2,
            malformed: 1,
        };
        let b = s.encode();
        assert_eq!(b.len(), 24);
        assert_eq!(&b[..6], b"DKST\x01\x02");
        assert_eq!(
            u32::from_le_bytes(b[6..10].try_into().unwrap()),
            0xDEAD_BEEF
        );
        assert_eq!(u32::from_le_bytes(b[10..14].try_into().unwrap()), 7);
        assert_eq!(u32::from_le_bytes(b[14..18].try_into().unwrap()), 2);
        assert_eq!(u32::from_le_bytes(b[18..22].try_into().unwrap()), 1);
    }

    /// Everything a report can carry, filled: still one unfragmented datagram on any LAN, and an
    /// empty one is every field unknown.
    #[test]
    fn a_robot_report_fits_one_datagram() {
        let r = RobotReport {
            health: Some(proto::HealthResult {
                healthy: false,
                reason: Some("control loop at 43.9 Hz, below 47.5 Hz".to_owned()),
                battery: Some(proto::Battery {
                    volts: 7.62,
                    percent: 61.0,
                }),
                cpu_temp_c: Some(71.2),
                ..Default::default()
            }),
            posture: Some(Posture {
                policy: "walk".to_owned(),
                fallen: true,
                ..Default::default()
            }),
            refused: Some(Refusal {
                method: "robot.do".to_owned(),
                reason: "the robot is not home yet; try again in a second".to_owned(),
                ago_ms: 1_234,
            }),
            ..Default::default()
        };
        let b = r.encode();
        assert_eq!(&b[..5], b"DKRB\x01");
        assert!(b.len() < 1200, "{} bytes", b.len());
        let back: RobotReport = serde_json::from_slice(&b[5..]).unwrap();
        assert_eq!(back, r);
        assert_eq!(RobotReport::default().encode(), b"DKRB\x01{}");
    }
}
