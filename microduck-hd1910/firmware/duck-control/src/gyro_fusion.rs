// Added 2026-10 by fanhao375 (microduck-replica): BNO08x gyro-integrated attitude; see IMU-MODULE.md at the repository root.
//! A BNO08x's orientation, carried between the chip's answers by the chip's own gyro.
//!
//! **Why.** Walking, a BNO085's game rotation vector (report 0x08) now and then jumps 20–60°
//! inside one 20 ms tick while its calibrated gyro (0x02) says the trunk barely turned. Robots
//! recorded at 50 Hz showed it on 1.9–23 % of walking ticks depending on the feet, and on
//! 0.05 % standing still; the gyro, the legs' kinematics and the recorded falls all agree that
//! the jumps are the chip's fusion, not the trunk. Gravity is what the policy observes and what
//! fall detection judges, so a jump is a policy suddenly told it is falling.
//!
//! **What.** Every gyro report is integrated into an attitude of the host's own. Every game
//! rotation vector is compared with it, as a tilt between the two gravity directions:
//!
//! - under [`GATE_DEG`] the chip is taken, slowly — the fused attitude moves `1 − e^(−Δt/τ)`
//!   of the way, [`TILT_TAU_S`], which leaves a 0.1°/s gyro bias at well under 0.1°;
//! - at or over it the chip is refused and the gyro alone carries the attitude;
//! - refused for [`REALIGN_AFTER_S`]: if the chip has been calm (agreeing with the gyro report
//!   to report) for as long *and* the trunk has been still ([`STILL_RMS_DEG_S`]), its tilt is
//!   taken whole — the drift a long refusal accumulates, put right where a step cannot hurt;
//!   otherwise the tilt is pulled towards the chip at [`SLEW_DEG_S`], so a long refusal can
//!   never leave the attitude open-loop for long, and never with a step in it.
//!
//! Only tilt is fused. Heading follows the chip at the same rate while it is taken and is held
//! while it is refused; nothing here gates or slews on heading, because heading reaches only
//! the roller mode's course and odometry, never the policy or fall detection.
//!
//! A new epoch (the reader restarted the chip) follows the chip outright for [`FOLLOW_S`], the
//! same second the gate refuses samples for. A hole in the gyro stream ([`GyroFusion::gap`])
//! cannot be integrated across; the chip is taken whole, heading included, once it has been
//! calm for [`REALIGN_AFTER_S`], and judged as usual until then.
//!
//! No I/O and no clock. Time is what the caller says it is: the integration step and the
//! chip's step come from the reports' own sequence numbers ([`crate::bno08x`] does that), so
//! a burst of reports read in one go is still 10 ms apart. Every result is checked finite and
//! unit length, and anything else follows the chip ([`Verdict::Fallback`]): a NaN that reached
//! [`crate::imu::TrunkFrame`] would hold an old orientation while the samples kept looking
//! fresh.
//!
//! The numbers are a first setting from that 50 Hz replay (tilt gate 6°, still-and-calm
//! realignment, 5°/s slew: 4 one-tick steps on 113 443 walking ticks, none of them while the
//! robot was upright, longest open-loop run 1.5 s); a 100 Hz raw-report log from a robot is
//! what they are to be settled on (`docs/trunk-imu-i2c.zh-CN.md`).

use serde::Serialize;

/// Scalar-first `[w, x, y, z]`.
pub type Quat = [f64; 4];

/// A chip orientation whose tilt is this far from the fused one is refused.
pub const GATE_DEG: f64 = 6.0;
/// How fast a taken chip orientation pulls the fused one: `1 − e^(−Δt/τ)` of the way per
/// report, 1.5 % at 100 Hz.
pub const TILT_TAU_S: f64 = 0.66;
/// A refusal this long, by the chip's own report clock, is no longer a glitch to sit out.
pub const REALIGN_AFTER_S: f64 = 0.5;
/// The chip is calm while, report to report, it moves no further from where the gyro carried
/// its last answer than this rate allows: 2° between two 20 ms ticks.
pub const CALM_RATE_DEG_S: f64 = 100.0;
/// How long a new epoch follows the chip outright — `trunk_imu::HOST_IMU_SETTLE`.
pub const FOLLOW_S: f64 = 1.0;
/// The trunk is still while its gyro's RMS over the last [`WINDOW_S`] is under this.
pub const STILL_RMS_DEG_S: f64 = 30.0;
/// How fast a long refusal pulls the tilt towards the chip while a whole step is not allowed.
pub const SLEW_DEG_S: f64 = 5.0;
/// Longest step either input may claim. A longer gyro step is a hole, not something to
/// integrate across; a longer chip step only counts this much.
pub const MAX_STEP_S: f64 = 0.1;
/// The stillness window.
pub const WINDOW_S: f64 = 0.5;
/// A gyro report is Q9 `i16`, so ±64 rad/s on each axis. Anything faster is no reading.
const MAX_RATE_RAD_S: f64 = 120.0;
/// Gyro reports kept for the stillness window: 50 at 100 Hz, with room.
const WINDOW_CAP: usize = 64;
const IDENTITY: Quat = [1.0, 0.0, 0.0, 0.0];

/// Which orientation a BNO08x sample publishes. The fusion runs in either; only this choice
/// differs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum FusionMode {
    /// The chip's own game rotation vector, exactly as before this module existed; the fused
    /// attitude rides along in the sample's diagnostics.
    #[default]
    Chip,
    /// The fused attitude.
    Gyro,
}

impl FusionMode {
    pub const ALL: [Self; 2] = [Self::Chip, Self::Gyro];

    /// What `[trunk_imu] bno08x_fusion` says for it.
    pub fn label(self) -> &'static str {
        match self {
            Self::Chip => "chip",
            Self::Gyro => "gyro",
        }
    }
}

/// What the fusion made of one chip orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Verdict {
    /// A new epoch's first second: the chip, as it is.
    Follow,
    /// Within the gate, and taken slowly.
    Accepted,
    /// Outside it; the gyro carries the attitude.
    Rejected,
    /// Refused for long, and the tilt pulled towards the chip at [`SLEW_DEG_S`].
    Slew,
    /// Refused for long while chip and trunk were both calm: the chip's tilt taken whole.
    RealignLong,
    /// After a hole in the gyro stream, once the chip was calm: the chip taken whole.
    RealignGap,
    /// A result that was not a finite unit rotation; the chip taken instead.
    Fallback,
    /// Never [`GyroFusion::chip`]'s answer: [`crate::bno08x`]'s word for a fusion it shut off
    /// after it panicked, which publishes the chip from then on.
    Disabled,
}

impl Verdict {
    pub fn label(self) -> &'static str {
        match self {
            Self::Follow => "follow",
            Self::Accepted => "accepted",
            Self::Rejected => "rejected",
            Self::Slew => "slew",
            Self::RealignLong => "realign_long",
            Self::RealignGap => "realign_gap",
            Self::Fallback => "fallback",
            Self::Disabled => "disabled",
        }
    }
}

/// Running totals, for `HostImuHealth`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct FusionCounts {
    pub follow: u64,
    pub accepted: u64,
    pub rejected: u64,
    pub slew: u64,
    pub realign_long: u64,
    pub realign_gap: u64,
    pub fallback: u64,
    /// Gyro reports integrated.
    pub gyro: u64,
    /// Holes in the gyro stream: a report sequence step over 3, a gap of host time, or a
    /// reading that is no reading.
    pub gyro_gaps: u64,
    /// Reports whose sequence went backwards: a re-read, skipped.
    pub out_of_order: u64,
    /// Panics caught. The first one turns the fusion off for the life of the reader.
    pub panics: u64,
}

impl FusionCounts {
    pub fn note(&mut self, verdict: Verdict) {
        let slot = match verdict {
            Verdict::Follow => &mut self.follow,
            Verdict::Accepted => &mut self.accepted,
            Verdict::Rejected => &mut self.rejected,
            Verdict::Slew => &mut self.slew,
            Verdict::RealignLong => &mut self.realign_long,
            Verdict::RealignGap => &mut self.realign_gap,
            Verdict::Fallback => &mut self.fallback,
            Verdict::Disabled => return,
        };
        *slot = slot.saturating_add(1);
    }
}

/// What a BNO08x sample says about the fusion, as of the chip orientation it carries.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FusionSample {
    pub mode: FusionMode,
    pub verdict: Verdict,
    /// The chip's own game rotation vector, sensor→world, `w ≥ 0`. What commissioning
    /// measures a mount against, whichever mode publishes.
    pub chip_quat: Quat,
    /// The fused attitude just after that report, sensor→world, `w ≥ 0`.
    pub fused_quat: Quat,
    /// Tilt between the two before the report's correction, degrees.
    pub theta_deg: f64,
    pub counts: FusionCounts,
    /// The report period measured against the host clock over at least ten seconds, µs.
    pub period_us: Option<f64>,
}

/// The attitude, the chip's last answer carried forward, and the clocks the rules need.
#[derive(Debug, Clone)]
pub struct GyroFusion {
    /// Sensor→world. `None` until the first chip orientation.
    q: Option<Quat>,
    /// The chip's last orientation carried by the gyro since: what this report should say if
    /// the chip is calm.
    pred: Quat,
    pred_valid: bool,
    /// A gyro step produced something that is not a rotation; the next chip report is taken.
    broken: bool,
    /// Σ chip steps, seconds.
    clock: f64,
    follow_until: f64,
    rejected_since: Option<f64>,
    calm_since: f64,
    gap_pending: bool,
    /// `(clock, |ω|²)` of recent gyro reports, a ring.
    window: [(f64, f64); WINDOW_CAP],
    window_len: usize,
    window_next: usize,
    theta_deg: f64,
}

impl Default for GyroFusion {
    fn default() -> Self {
        Self::new()
    }
}

impl GyroFusion {
    pub fn new() -> Self {
        Self {
            q: None,
            pred: IDENTITY,
            pred_valid: false,
            broken: false,
            clock: 0.0,
            follow_until: 0.0,
            rejected_since: None,
            calm_since: 0.0,
            gap_pending: false,
            window: [(0.0, 0.0); WINDOW_CAP],
            window_len: 0,
            window_next: 0,
            theta_deg: 0.0,
        }
    }

    /// One gyro report: `rate` rad/s in the sensor's axes, held for `dt` seconds. A reading
    /// that is not one — non-finite, faster than the chip can report, or a step outside
    /// `(0, MAX_STEP_S]` — is a hole ([`Self::gap`]); returns whether it was taken.
    pub fn gyro(&mut self, rate: [f64; 3], dt: f64) -> bool {
        let speed_sq = rate[0] * rate[0] + rate[1] * rate[1] + rate[2] * rate[2];
        let valid = speed_sq.is_finite()
            && speed_sq <= MAX_RATE_RAD_S * MAX_RATE_RAD_S
            && dt.is_finite()
            && dt > 0.0
            && dt <= MAX_STEP_S;
        if !valid {
            self.gap();
            return false;
        }
        let Some(q) = self.q else {
            return true;
        };
        let step = exp_rotation(rate, dt);
        let next = normalise(mul(q, step));
        if !is_unit(next) {
            self.broken = true;
            return true;
        }
        self.q = Some(next);
        if self.pred_valid {
            self.pred = normalise(mul(self.pred, step));
            self.pred_valid = is_unit(self.pred);
        }
        self.window[self.window_next] = (self.clock, speed_sq);
        self.window_next = (self.window_next + 1) % WINDOW_CAP;
        self.window_len = (self.window_len + 1).min(WINDOW_CAP);
        true
    }

    /// The gyro stream has a hole that cannot be integrated across.
    pub fn gap(&mut self) {
        self.gap_pending = true;
        self.pred_valid = false;
    }

    /// One chip orientation, sensor→world, `dt` seconds of report clock after the last. An
    /// orientation that is not one (non-finite, or far from unit length) changes nothing and is
    /// answered [`Verdict::Rejected`].
    pub fn chip(&mut self, chip: Quat, dt: f64) -> Verdict {
        let Some(chip) = unit_positive(chip) else {
            return Verdict::Rejected;
        };
        let dt = if dt.is_finite() && dt > 0.0 {
            dt.min(MAX_STEP_S)
        } else {
            0.0
        };
        self.clock += dt;
        let Some(q) = self.q else {
            self.seed(chip);
            self.follow_until = self.clock + FOLLOW_S;
            self.calm_since = self.clock;
            return Verdict::Follow;
        };
        if self.broken {
            self.broken = false;
            self.seed(chip);
            return Verdict::Fallback;
        }
        // Calm: the chip's last answer, carried here by the gyro, is where this one is.
        if !self.pred_valid || tilt_deg(self.pred, chip) > CALM_RATE_DEG_S * dt {
            self.calm_since = self.clock;
        }
        self.pred = chip;
        self.pred_valid = true;
        if self.clock < self.follow_until {
            self.seed(chip);
            return Verdict::Follow;
        }
        let (twist, swing, theta) = decompose(mul(chip, conj(q)));
        self.theta_deg = theta;
        let calm = self.clock - self.calm_since >= REALIGN_AFTER_S;
        let (next, verdict) = if self.gap_pending && calm {
            self.gap_pending = false;
            self.rejected_since = None;
            (chip, Verdict::RealignGap)
        } else if theta < GATE_DEG {
            self.rejected_since = None;
            let k = -(-dt / TILT_TAU_S).exp_m1();
            let next = mul(power(twist, k), mul(power(swing, k), q));
            (normalise(next), Verdict::Accepted)
        } else {
            let since = *self.rejected_since.get_or_insert(self.clock);
            let long = self.clock - since >= REALIGN_AFTER_S;
            if long && calm && self.still() {
                self.rejected_since = None;
                (normalise(mul(swing, q)), Verdict::RealignLong)
            } else if long {
                let k = (SLEW_DEG_S * dt / theta).min(1.0);
                (normalise(mul(power(swing, k), q)), Verdict::Slew)
            } else {
                (q, Verdict::Rejected)
            }
        };
        if !is_unit(next) {
            self.seed(chip);
            return Verdict::Fallback;
        }
        self.q = Some(next);
        verdict
    }

    /// The fused attitude, sensor→world, `w ≥ 0`; `None` before the first chip orientation.
    pub fn fused(&self) -> Option<Quat> {
        self.q.map(positive)
    }

    /// Tilt between chip and fusion at the last chip report, before its correction, degrees.
    /// Zero while following.
    pub fn theta_deg(&self) -> f64 {
        self.theta_deg
    }

    fn seed(&mut self, chip: Quat) {
        self.q = Some(chip);
        self.pred = chip;
        self.pred_valid = true;
        self.rejected_since = None;
        self.gap_pending = false;
        self.theta_deg = 0.0;
    }

    /// Whether the gyro's RMS over the last [`WINDOW_S`] of report clock is under
    /// [`STILL_RMS_DEG_S`]. No reports is still.
    fn still(&self) -> bool {
        let since = self.clock - WINDOW_S;
        let (mut sum, mut n) = (0.0, 0u32);
        for &(at, speed_sq) in &self.window[..self.window_len] {
            if at >= since {
                sum += speed_sq;
                n += 1;
            }
        }
        n == 0 || (sum / f64::from(n)).sqrt().to_degrees() < STILL_RMS_DEG_S
    }
}

/// Hamilton product, scalar-first.
fn mul(a: Quat, b: Quat) -> Quat {
    let ([aw, ax, ay, az], [bw, bx, by, bz]) = (a, b);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

fn conj(q: Quat) -> Quat {
    [q[0], -q[1], -q[2], -q[3]]
}

fn norm(q: Quat) -> f64 {
    (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt()
}

/// Unit length. A zero or non-finite input comes back non-finite, for [`is_unit`] to catch.
fn normalise(q: Quat) -> Quat {
    let n = norm(q);
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// `q` and `-q` are one rotation; this is the one with `w ≥ 0`.
fn positive(q: Quat) -> Quat {
    if q[0] < 0.0 {
        [-q[0], -q[1], -q[2], -q[3]]
    } else {
        q
    }
}

fn is_unit(q: Quat) -> bool {
    q.iter().all(|v| v.is_finite()) && (norm(q) - 1.0).abs() < 1e-3
}

fn unit_positive(q: Quat) -> Option<Quat> {
    let n = norm(q);
    (n.is_finite() && (0.5..=2.0).contains(&n)).then(|| positive(normalise(q)))
}

/// The rotation `rate · dt` as a unit quaternion: `|rate|·dt` about `rate`.
fn exp_rotation(rate: [f64; 3], dt: f64) -> Quat {
    let r = [rate[0] * dt, rate[1] * dt, rate[2] * dt];
    let angle = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
    if angle < 1e-9 {
        return normalise([1.0, r[0] / 2.0, r[1] / 2.0, r[2] / 2.0]);
    }
    let s = (angle / 2.0).sin() / angle;
    [(angle / 2.0).cos(), r[0] * s, r[1] * s, r[2] * s]
}

/// `q^k`: the same axis, `k` times the angle (of the `w ≥ 0` representative).
fn power(q: Quat, k: f64) -> Quat {
    let q = positive(q);
    let s = (q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if s < 1e-12 {
        return IDENTITY;
    }
    let half = s.atan2(q[0]);
    let f = (k * half).sin() / s;
    [(k * half).cos(), q[1] * f, q[2] * f, q[3] * f]
}

/// Gravity in the sensor's axes, `q⁻¹ (0, 0, −1) q` — what [`crate::imu::TrunkFrame`] makes of
/// an orientation.
fn gravity(q: Quat) -> [f64; 3] {
    let [w, x, y, z] = q;
    [
        -2.0 * (x * z - w * y),
        -2.0 * (y * z + w * x),
        -(1.0 - 2.0 * (x * x + y * y)),
    ]
}

/// Angle between two vectors, degrees, `atan2(|a×b|, a·b)` — exact near zero, where `acos` is
/// not.
fn angle_deg(a: [f64; 3], b: [f64; 3]) -> f64 {
    let c = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    let cross = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
    cross
        .atan2(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])
        .to_degrees()
}

/// Angle between two orientations' gravity directions, degrees: their tilt apart.
fn tilt_deg(a: Quat, b: Quat) -> f64 {
    angle_deg(gravity(a), gravity(b))
}

/// A world-frame error `e = twist ⊗ swing`: `twist` about world z on the left, `swing` about a
/// horizontal axis on the right, and the swing's angle in degrees. The order matters — the
/// other one leaves a tilt error behind after "aligning the tilt". At an error of 180° of tilt
/// the heading is undefined and the twist is taken as none.
fn decompose(error: Quat) -> (Quat, Quat, f64) {
    let e = positive(error);
    let n = e[0].hypot(e[3]);
    let twist = if n < 1e-9 {
        IDENTITY
    } else {
        [e[0] / n, 0.0, 0.0, e[3] / n]
    };
    let swing = mul(conj(twist), e);
    let s = (swing[1] * swing[1] + swing[2] * swing[2] + swing[3] * swing[3]).sqrt();
    (twist, swing, (2.0 * s.atan2(swing[0].abs())).to_degrees())
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) fn axis_angle(axis: [f64; 3], deg: f64) -> Quat {
        let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let h = deg.to_radians() / 2.0;
        let s = h.sin() / n;
        [h.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
    }
    const X: [f64; 3] = [1.0, 0.0, 0.0];
    const Y: [f64; 3] = [0.0, 1.0, 0.0];
    const Z: [f64; 3] = [0.0, 0.0, 1.0];

    /// `q` turned at body `rate` for `dt`.
    pub(crate) fn turned(q: Quat, rate: [f64; 3], dt: f64) -> Quat {
        normalise(mul(q, exp_rotation(rate, dt)))
    }

    /// Whole rotation angle between two orientations, degrees.
    pub(crate) fn apart_deg(a: Quat, b: Quat) -> f64 {
        let d = mul(conj(a), b);
        let v = (d[1] * d[1] + d[2] * d[2] + d[3] * d[3]).sqrt();
        (2.0 * v.atan2(d[0].abs())).to_degrees()
    }
    fn tilt_of(a: Quat, b: Quat) -> f64 {
        tilt_deg(a, b)
    }
    /// The heading difference between two orientations: the twist of `b ⊗ a⁻¹`, degrees.
    fn yaw_apart_deg(a: Quat, b: Quat) -> f64 {
        let (twist, _, _) = decompose(mul(b, conj(a)));
        (2.0 * twist[3].atan2(twist[0])).to_degrees()
    }

    /// A robot at 100 Hz: its true attitude, and the fusion fed its gyro and some chip.
    struct Run {
        truth: Quat,
        fusion: GyroFusion,
        dt: f64,
    }
    impl Run {
        /// Seeded and through the first second, chip = truth, standing still.
        fn settled(truth: Quat) -> Self {
            let mut run = Self {
                truth,
                fusion: GyroFusion::new(),
                dt: 0.01,
            };
            for _ in 0..110 {
                let v = run.step([0.0; 3], |t| t);
                assert!(matches!(v, Verdict::Follow | Verdict::Accepted), "{v:?}");
            }
            assert!(
                run.fusion.clock >= run.fusion.follow_until,
                "past the first second"
            );
            run
        }
        /// Turn at `rate` (body axes) for one report, then show the fusion `chip(truth)`.
        fn step(&mut self, rate: [f64; 3], chip: impl Fn(Quat) -> Quat) -> Verdict {
            self.truth = normalise(mul(self.truth, exp_rotation(rate, self.dt)));
            self.fusion.gyro(rate, self.dt);
            self.fusion.chip(chip(self.truth), self.dt)
        }
        fn fused(&self) -> Quat {
            self.fusion.fused().unwrap()
        }
    }

    /// ① A constant rate about any axis is integrated exactly (the step is the exact
    /// exponential), and a chip that agrees is taken every time.
    #[test]
    fn a_constant_rate_about_any_axis_integrates_exactly() {
        let axis = [1.0, -2.0, 0.5];
        let n = (1.0f64 + 4.0 + 0.25).sqrt();
        let rate = axis.map(|a| a / n * 90f64.to_radians());
        let start = axis_angle([0.3, 0.2, 1.0], 40.0);
        let mut fusion = GyroFusion::new();
        assert_eq!(fusion.chip(start, 0.01), Verdict::Follow);
        for _ in 0..100 {
            assert!(fusion.gyro(rate, 0.01));
        }
        let want = mul(start, axis_angle(axis, 90.0));
        assert!(apart_deg(fusion.fused().unwrap(), want) < 1e-9);

        let mut run = Run::settled(start);
        for _ in 0..200 {
            assert_eq!(run.step(rate, |t| t), Verdict::Accepted);
        }
        assert!(apart_deg(run.fused(), run.truth) < 1e-9);
    }

    /// ① A rate that changes all the time (a coning trunk) against the analytic attitude
    /// `q(t) = Rz(a t) Ry(b sin c t) Rx(d t)`, sampled mid-step at 100 Hz for 2 s.
    #[test]
    fn a_coning_trunk_is_integrated_against_its_analytic_attitude() {
        let (a, b, c, d) = (0.9, 0.5, 6.0, 1.3);
        let attitude = |t: f64| {
            mul(
                axis_angle(Z, (a * t).to_degrees()),
                mul(
                    axis_angle(Y, (b * (c * t).sin()).to_degrees()),
                    axis_angle(X, (d * t).to_degrees()),
                ),
            )
        };
        // ω_body = 2 · vec(q⁻¹ q̇), q̇ by the product rule.
        let rate = |t: f64| {
            let (qz, qy, qx) = (
                axis_angle(Z, (a * t).to_degrees()),
                axis_angle(Y, (b * (c * t).sin()).to_degrees()),
                axis_angle(X, (d * t).to_degrees()),
            );
            let phi_dot = b * c * (c * t).cos();
            let terms = [
                mul(mul(mul(qz, [0.0, 0.0, 0.0, a / 2.0]), qy), qx),
                mul(mul(mul(qz, qy), [0.0, 0.0, phi_dot / 2.0, 0.0]), qx),
                mul(mul(mul(qz, qy), qx), [0.0, d / 2.0, 0.0, 0.0]),
            ];
            let q_dot = [0, 1, 2, 3].map(|i| terms.iter().map(|q| q[i]).sum::<f64>());
            let w = mul(conj(attitude(t)), q_dot);
            [2.0 * w[1], 2.0 * w[2], 2.0 * w[3]]
        };
        let mut fusion = GyroFusion::new();
        fusion.chip(attitude(0.0), 0.01);
        let dt = 0.01;
        for k in 0..200 {
            assert!(fusion.gyro(rate((k as f64 + 0.5) * dt), dt));
        }
        let err = apart_deg(fusion.fused().unwrap(), attitude(2.0));
        assert!(err < 0.05, "{err}°");
    }

    /// The first second of an epoch is the chip, exactly, whatever the gyro says.
    #[test]
    fn a_new_epoch_follows_the_chip_for_its_first_second() {
        assert_eq!(
            FOLLOW_S,
            crate::trunk_imu::HOST_IMU_SETTLE.as_secs_f64(),
            "the fusion follows for exactly the second the gate refuses"
        );
        let mut fusion = GyroFusion::new();
        for k in 0..120 {
            fusion.gyro([0.0, 2.0, 0.0], 0.01);
            let chip = axis_angle(X, f64::from(k));
            let verdict = fusion.chip(chip, 0.01);
            // Report k is k × 10 ms after the first; 99 and 100 sit on the boundary.
            if k <= 98 {
                assert_eq!(verdict, Verdict::Follow, "{k}");
                assert_eq!(fusion.fused().unwrap(), positive(normalise(chip)));
            } else if k >= 101 {
                assert_ne!(verdict, Verdict::Follow, "{k}");
            }
        }
    }

    /// ③ The chip jumps 30° for 0.3 s while the trunk is still: every one of those is refused,
    /// the attitude stays within half a degree, and nothing is realigned to.
    #[test]
    fn a_thirty_degree_glitch_for_a_third_of_a_second_is_refused_throughout() {
        let mut run = Run::settled(axis_angle(Y, 5.0));
        let wrong = axis_angle([1.0, 1.0, 0.0], 30.0);
        for k in 0..100 {
            let glitch = (20..50).contains(&k);
            let verdict = run.step([0.0; 3], |t| if glitch { mul(wrong, t) } else { t });
            if glitch {
                assert_eq!(verdict, Verdict::Rejected, "{k}");
            } else {
                assert_eq!(verdict, Verdict::Accepted, "{k}");
            }
            assert!(apart_deg(run.fused(), run.truth) < 0.5);
        }
    }

    /// ④ A chip that settles 10° of tilt (and 15° of heading) away while the trunk stands
    /// still: refused, then after half a second its tilt is taken whole — gravity then agrees
    /// exactly — and the heading is left where it was.
    #[test]
    fn a_steady_wrong_tilt_on_a_still_trunk_is_taken_whole_after_half_a_second() {
        let mut run = Run::settled(IDENTITY);
        let offset = mul(axis_angle(Z, 15.0), axis_angle(X, 10.0));
        let mut realigned = None;
        for k in 0..100 {
            let before = run.fused();
            let verdict = run.step([0.0; 3], |t| mul(offset, t));
            match verdict {
                Verdict::Rejected => assert!(realigned.is_none()),
                Verdict::RealignLong => {
                    realigned = Some(k);
                    let chip = mul(offset, run.truth);
                    assert!(tilt_of(run.fused(), chip) < 1e-6);
                    assert!(
                        yaw_apart_deg(before, run.fused()).abs() < 1e-9,
                        "heading moved"
                    );
                    assert!((yaw_apart_deg(run.fused(), chip) - 15.0).abs() < 1e-6);
                }
                Verdict::Accepted => assert!(realigned.is_some(), "{k}"),
                other => panic!("{k}: {other:?}"),
            }
        }
        let at = realigned.expect("never realigned");
        // Rejected from the first wrong report (k = 0, clock t0); taken at t0 + 0.5 s ± a report.
        assert!((49..=51).contains(&at), "realigned at report {at}");
    }

    /// ⑤ A chip that jumps every report for two seconds is never calm, so it is never taken
    /// whole; after half a second the tilt is pulled towards it no faster than 5°/s.
    #[test]
    fn a_chip_jumping_for_two_seconds_is_never_realigned_to() {
        let mut run = Run::settled(IDENTITY);
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut random = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut slews = 0;
        for _ in 0..200 {
            let wrong = axis_angle(
                [random() - 0.5, random() - 0.5, 0.0],
                10.0 + 30.0 * random(),
            );
            let before = run.fused();
            let verdict = run.step([0.0; 3], |t| mul(wrong, t));
            assert!(
                matches!(verdict, Verdict::Rejected | Verdict::Slew),
                "{verdict:?}"
            );
            slews += usize::from(verdict == Verdict::Slew);
            assert!(apart_deg(before, run.fused()) <= SLEW_DEG_S * run.dt + 1e-9);
        }
        assert!(
            (149..=150).contains(&slews),
            "slewing from half a second on: {slews}"
        );
    }

    /// The slew rule pinned on a moving trunk: the chip 7° off but steady — calm, agreeing
    /// with the gyro — while the trunk turns at 100°/s. Not still, so never a whole step: the
    /// tilt closes at 5°/s until the chip is inside the gate again.
    #[test]
    fn steadily_wrong_while_moving_is_slewed_at_five_degrees_a_second() {
        let mut run = Run::settled(IDENTITY);
        let offset = axis_angle(Y, 7.0);
        let rate = [0.0, 0.0, 100f64.to_radians()];
        let mut seen: Vec<Verdict> = Vec::new();
        let mut last_theta = 0.0;
        for _ in 0..150 {
            let verdict = run.step(rate, |t| mul(offset, t));
            if seen.last() == Some(&Verdict::Slew) {
                // θ is read before this report's correction, so it shows the last report's slew.
                let closed = last_theta - run.fusion.theta_deg();
                assert!(closed <= SLEW_DEG_S * run.dt + 1e-6, "closed {closed}°");
                assert!(closed > SLEW_DEG_S * run.dt - 1e-6, "closed {closed}°");
            }
            last_theta = run.fusion.theta_deg();
            seen.push(verdict);
        }
        assert!(!seen.contains(&Verdict::RealignLong));
        let first_slew = seen.iter().position(|v| *v == Verdict::Slew).unwrap();
        assert!((49..=51).contains(&first_slew), "{first_slew}");
        assert_eq!(
            seen.last(),
            Some(&Verdict::Accepted),
            "back inside the gate"
        );
    }

    /// The same steady wrong tilt with the trunk shaking (RMS over 30°/s): never a whole step.
    #[test]
    fn a_whole_step_needs_a_still_trunk() {
        let mut run = Run::settled(IDENTITY);
        let offset = axis_angle(X, 12.0);
        for k in 0..100 {
            let w = 60f64.to_radians() * (f64::from(k) * 0.6).sin();
            let verdict = run.step([w, 0.0, 0.0], |t| mul(offset, t));
            assert_ne!(verdict, Verdict::RealignLong, "{k}");
        }
    }

    /// ⑥ A hole in the gyro stream: the chip, once calm for half a second, is taken whole —
    /// heading included.
    #[test]
    fn after_a_gyro_gap_a_calm_chip_is_taken_whole_heading_included() {
        let mut run = Run::settled(IDENTITY);
        run.fusion.gap();
        let offset = mul(axis_angle(Z, 20.0), axis_angle(X, 8.0));
        let mut verdicts = Vec::new();
        for _ in 0..60 {
            verdicts.push(run.step([0.0; 3], |t| mul(offset, t)));
        }
        let at = verdicts
            .iter()
            .position(|v| *v == Verdict::RealignGap)
            .expect("never realigned");
        // Calm is counted from the first report after the hole.
        assert!((49..=51).contains(&at), "{at}");
        assert_eq!(
            verdicts
                .iter()
                .filter(|v| **v == Verdict::RealignGap)
                .count(),
            1
        );
        assert!(apart_deg(run.fused(), mul(offset, run.truth)) < 0.01);
    }

    /// A hole followed straight away by a glitching chip: no whole step until it is calm.
    #[test]
    fn a_gap_into_a_glitch_waits_for_the_chip_to_calm() {
        let mut run = Run::settled(IDENTITY);
        run.fusion.gap();
        let mut verdicts = Vec::new();
        for k in 0..100 {
            let wrong = axis_angle(
                X,
                if k < 30 {
                    20.0 + f64::from(k % 7) * 5.0
                } else {
                    0.0
                },
            );
            verdicts.push(run.step([0.0; 3], |t| mul(wrong, t)));
        }
        let at = verdicts
            .iter()
            .position(|v| *v == Verdict::RealignGap)
            .expect("never realigned");
        assert!(
            (79..=81).contains(&at),
            "calm from report 30, taken half a second on: {at}"
        );
        assert!(apart_deg(run.fused(), run.truth) < 1e-9);
    }

    /// Heading: taken at the tilt's own rate while the chip is taken — a 3° heading error is
    /// down to 1/e of itself after τ — and held exactly while it is refused.
    #[test]
    fn heading_follows_at_the_tilt_rate_and_is_held_while_refused() {
        let mut run = Run::settled(IDENTITY);
        let off = axis_angle(Z, 3.0);
        let steps = (TILT_TAU_S / run.dt).round() as usize;
        for _ in 0..steps {
            assert_eq!(run.step([0.0; 3], |t| mul(off, t)), Verdict::Accepted);
        }
        let left = yaw_apart_deg(run.fused(), mul(off, run.truth));
        assert!((left - 3.0 / std::f64::consts::E).abs() < 0.05, "{left}");
        let held = run.fused();
        let wrong = mul(axis_angle(Z, 10.0), axis_angle(X, 30.0));
        for _ in 0..20 {
            assert_eq!(run.step([0.0; 3], |t| mul(wrong, t)), Verdict::Rejected);
        }
        assert!(apart_deg(held, run.fused()) < 1e-12);
    }

    /// ⑭ A tilt error inside the gate decays with τ: 3° is 3/e after τ.
    #[test]
    fn a_tilt_error_inside_the_gate_decays_with_tau() {
        let mut run = Run::settled(IDENTITY);
        let off = axis_angle(X, 3.0);
        let steps = (TILT_TAU_S / run.dt).round() as usize;
        for _ in 0..steps {
            assert_eq!(run.step([0.0; 3], |t| mul(off, t)), Verdict::Accepted);
        }
        let left = tilt_of(run.fused(), mul(off, run.truth));
        assert!((left - 3.0 / std::f64::consts::E).abs() < 0.05, "{left}");
    }

    /// ⑬ A gyro biased 0.5°/s on a still trunk leaves the tilt off by bias × τ, not more.
    #[test]
    fn a_biased_gyro_on_a_still_trunk_leaves_a_small_tilt_error() {
        let mut run = Run::settled(axis_angle(Y, 3.0));
        let bias = [0.5f64.to_radians(), 0.0, 0.0];
        let truth = run.truth;
        for _ in 0..3000 {
            run.fusion.gyro(bias, run.dt);
            assert_eq!(run.fusion.chip(truth, run.dt), Verdict::Accepted);
        }
        let err = tilt_of(run.fused(), truth);
        assert!(err < 0.5, "{err}°");
        assert!(
            (err - 0.5 * TILT_TAU_S).abs() < 0.05,
            "{err}° against bias × τ"
        );
    }

    /// ⑮ The decomposition: pure tilt, pure heading, both, and the 180° corner.
    #[test]
    fn the_decomposition_puts_heading_on_the_left() {
        let (twist, swing, theta) = decompose(axis_angle([1.0, 1.0, 0.0], 12.0));
        assert!(apart_deg(twist, IDENTITY) < 1e-12);
        assert!((theta - 12.0).abs() < 1e-9);
        assert!(apart_deg(swing, axis_angle([1.0, 1.0, 0.0], 12.0)) < 1e-12);

        let (twist, _, theta) = decompose(axis_angle(Z, -40.0));
        assert!(theta.abs() < 1e-9);
        assert!(apart_deg(twist, axis_angle(Z, -40.0)) < 1e-12);

        let e = mul(axis_angle(Z, 25.0), axis_angle([0.3, -1.0, 0.0], 17.0));
        let (twist, swing, theta) = decompose(e);
        assert!(swing[3].abs() < 1e-15, "the swing has no heading in it");
        assert!(apart_deg(mul(twist, swing), e) < 1e-12);
        assert!((theta - 17.0).abs() < 1e-9);
        // Aligning only the tilt leaves gravity exactly the chip's.
        let q = axis_angle([0.2, 0.4, 1.0], 70.0);
        let chip = mul(e, q);
        assert!(tilt_of(mul(swing, q), chip) < 1e-6);
        // The other order — the twist taken off the right — does not.
        let wrong = mul(e, conj(twist));
        assert!(tilt_of(mul(wrong, q), chip) > 1.0);

        let (twist, _, theta) = decompose(axis_angle(X, 180.0));
        assert_eq!(twist, IDENTITY);
        assert!((theta - 180.0).abs() < 1e-9);
    }

    /// M3: what the chip cannot send still never makes the attitude anything but a finite
    /// unit rotation — full-scale Q9 on every axis, NaN, infinity, zero and negative steps, a
    /// zero quaternion, a NaN quaternion, an unnormalised one.
    #[test]
    fn extreme_inputs_never_leave_a_non_finite_attitude() {
        let mut run = Run::settled(IDENTITY);
        let full = 32767.0 / 512.0;
        let rates = [
            [full, full, full],
            [-full, full, -full],
            [f64::NAN, 0.0, 0.0],
            [f64::INFINITY, 0.0, 0.0],
            [1.0, 1e300, 0.0],
        ];
        for rate in rates {
            for dt in [0.01, 0.0, -0.01, f64::NAN, 10.0, 2.55] {
                run.fusion.gyro(rate, dt);
                assert!(is_unit(run.fusion.fused().unwrap()), "{rate:?} {dt}");
            }
        }
        for chip in [
            [0.0; 4],
            [f64::NAN, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0],
            [-0.7, 0.7, 0.0, 0.0],
            [1e-300, 0.0, 0.0, 0.0],
        ] {
            for dt in [0.01, f64::NAN, -1.0, 1e9] {
                run.fusion.chip(chip, dt);
                assert!(is_unit(run.fusion.fused().unwrap()), "{chip:?} {dt}");
                assert!(run.fusion.theta_deg().is_finite());
            }
        }
        assert_eq!(run.fusion.chip([0.0; 4], 0.01), Verdict::Rejected);
        assert_eq!(
            unit_positive([2.0, 0.0, 0.0, 0.0]),
            Some(IDENTITY),
            "a quaternion twice unit length is still a rotation"
        );
    }

    /// A gyro step that does not come out a rotation is caught, and the next chip report is
    /// taken instead.
    #[test]
    fn a_broken_step_falls_back_to_the_chip() {
        let mut run = Run::settled(IDENTITY);
        run.fusion.broken = true;
        let chip = axis_angle(X, 20.0);
        assert_eq!(run.fusion.chip(chip, 0.01), Verdict::Fallback);
        assert_eq!(run.fusion.fused(), Some(positive(normalise(chip))));
        let mut counts = FusionCounts::default();
        counts.note(Verdict::Fallback);
        counts.note(Verdict::Disabled);
        assert_eq!((counts.fallback, counts.accepted), (1, 0));
    }

    #[test]
    fn labels_are_the_wire_names() {
        for mode in FusionMode::ALL {
            assert_eq!(
                serde_json::to_value(mode).unwrap(),
                serde_json::json!(mode.label())
            );
        }
        for verdict in [
            Verdict::Follow,
            Verdict::Accepted,
            Verdict::Rejected,
            Verdict::Slew,
            Verdict::RealignLong,
            Verdict::RealignGap,
            Verdict::Fallback,
            Verdict::Disabled,
        ] {
            assert_eq!(
                serde_json::to_value(verdict).unwrap(),
                serde_json::json!(verdict.label())
            );
        }
    }
}

/// The fusion on robot recordings, the whole way to what the policy sees: fused attitude →
/// [`crate::imu::TrunkFrame`] → three-point median → the glitch count the recordings were
/// judged by. `test-data/bno08x-fusion-replay/` holds the recordings and says where each is
/// from; the phase-0 Python reference (same algorithm, same inputs) got the figures each test
/// compares against, to the tolerances below.
#[cfg(test)]
mod replay_tests {
    use super::*;
    use crate::imu::{SensorReading, TrunkFrame};

    struct Row {
        t: f64,
        policy: String,
        limp: bool,
        fallen: bool,
        quat: Quat,
        gyro: [f64; 3],
    }

    fn parse(csv: &str) -> Vec<Row> {
        csv.lines()
            .filter(|line| !line.starts_with('#') && !line.starts_with("t,"))
            .map(|line| {
                let c: Vec<&str> = line.split(',').collect();
                let f = |i: usize| c[i].parse::<f64>().unwrap();
                Row {
                    t: f(0),
                    policy: c[1].to_owned(),
                    limp: c[2] == "1",
                    fallen: c[3] == "1",
                    quat: [f(5), f(6), f(7), f(8)],
                    gyro: [f(9), f(10), f(11)],
                }
            })
            .collect()
    }

    /// The recording is 50 Hz and its gyro is already a tick late (it is the median's output):
    /// each tick integrates the *next* tick's rate over this tick's interval, as the reference
    /// does.
    fn run(rows: &[Row]) -> (Vec<Quat>, Vec<Verdict>) {
        let mut fusion = GyroFusion::new();
        let (mut out, mut verdicts) = (Vec::new(), Vec::new());
        for (k, row) in rows.iter().enumerate() {
            let verdict = if k == 0 {
                fusion.chip(row.quat, 0.02)
            } else {
                let dt = row.t - rows[k - 1].t;
                let next = &rows[(k + 1).min(rows.len() - 1)];
                let span = next.t - rows[k - 1].t;
                let rate = if span > 0.0 && span <= 0.2 {
                    next.gyro
                } else {
                    row.gyro
                };
                fusion.gyro(rate, dt);
                fusion.chip(row.quat, dt)
            };
            out.push(fusion.fused().unwrap());
            verdicts.push(verdict);
        }
        (out, verdicts)
    }

    /// What the policy would see: each orientation through a trunk frame (identity mount — the
    /// recording is in trunk axes already) and its median.
    fn through_the_frame(rows: &[Row], quats: &[Quat]) -> Vec<[f64; 3]> {
        let mut frame = TrunkFrame::new(IDENTITY);
        frame.restart_median();
        rows.iter()
            .zip(quats)
            .map(|(row, q)| {
                frame
                    .push(SensorReading {
                        gyro: row.gyro,
                        quat: Some(*q),
                    })
                    .gravity
            })
            .collect()
    }

    /// The recordings' glitch count: the last gravity carried 20 ms by the recorded gyro, more
    /// than 5° from this one.
    fn jump_errors(g: &[[f64; 3]], rows: &[Row]) -> Vec<Option<f64>> {
        let mut errors = vec![None; g.len()];
        for k in 1..g.len() {
            let (p, c) = (&rows[k - 1], &rows[k]);
            let dt = c.t - p.t;
            if !(0.015 < dt && dt < 0.03) || c.limp || p.limp {
                continue;
            }
            let (g0, w) = (g[k - 1], c.gyro);
            let pred = [
                g0[0] - (w[1] * g0[2] - w[2] * g0[1]) * dt,
                g0[1] - (w[2] * g0[0] - w[0] * g0[2]) * dt,
                g0[2] - (w[0] * g0[1] - w[1] * g0[0]) * dt,
            ];
            let dot = pred[0] * g[k][0] + pred[1] * g[k][1] + pred[2] * g[k][2];
            let n = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            let cos = (dot / (n(pred) * n(g[k]))).clamp(-1.0, 1.0);
            errors[k] = Some(cos.acos().to_degrees());
        }
        errors
    }

    fn percentile(mut xs: Vec<f64>, p: f64) -> f64 {
        xs.sort_by(f64::total_cmp);
        if xs.is_empty() {
            return 0.0;
        }
        xs[((xs.len() as f64 * p) as usize).min(xs.len() - 1)]
    }

    #[derive(Debug)]
    struct Metrics {
        counts: FusionCounts,
        chip_steps: usize,
        fused_steps: usize,
        fused_step_max: f64,
        longest_open_loop_s: f64,
        /// Whole realignments: (t, walking upright, fallen within the second before).
        realigns: Vec<(f64, bool, bool)>,
        clean_med: f64,
        clean_p99: f64,
        held_med: f64,
        held_p99: f64,
        held_max: f64,
        last_second_max: f64,
    }

    fn metrics(csv: &str) -> Metrics {
        let rows = parse(csv);
        let (fused, verdicts) = run(&rows);
        let walking: Vec<bool> = rows.iter().map(|r| r.policy == "walk" && !r.limp).collect();
        let chip: Vec<Quat> = rows.iter().map(|r| positive(normalise(r.quat))).collect();
        let chip_errors = jump_errors(&through_the_frame(&rows, &chip), &rows);
        let fused_errors = jump_errors(&through_the_frame(&rows, &fused), &rows);
        // A chip glitch, ±25 ticks: the part of a recording the fusion is meant to differ in.
        let raw: Vec<[f64; 3]> = chip.iter().map(|q| gravity(*q)).collect();
        let mut episode = vec![false; rows.len()];
        for (k, e) in jump_errors(&raw, &rows).iter().enumerate() {
            if e.is_some_and(|e| e > 5.0) {
                for flag in &mut episode[k.saturating_sub(25)..(k + 26).min(rows.len())] {
                    *flag = true;
                }
            }
        }
        let dev: Vec<f64> = (0..rows.len())
            .map(|k| angle_deg(gravity(fused[k]), raw[k]))
            .collect();
        let pick = |keep: &dyn Fn(usize) -> bool| -> Vec<f64> {
            (0..rows.len())
                .filter(|&k| keep(k))
                .map(|k| dev[k])
                .collect()
        };
        let clean = pick(&|k| !episode[k] && !rows[k].limp && verdicts[k] != Verdict::Follow);
        let held =
            pick(&|k| rows[k].policy == "held" && !rows[k].limp && verdicts[k] != Verdict::Follow);
        let steps: Vec<f64> = (0..rows.len())
            .filter_map(|k| fused_errors[k].filter(|e| walking[k] && *e > 5.0))
            .collect();
        let (mut run_s, mut longest) = (0.0, 0.0f64);
        for k in 1..rows.len() {
            if matches!(verdicts[k], Verdict::Rejected | Verdict::Slew)
                && walking[k]
                && !rows[k].fallen
            {
                run_s += rows[k].t - rows[k - 1].t;
                longest = longest.max(run_s);
            } else {
                run_s = 0.0;
            }
        }
        let mut counts = FusionCounts::default();
        verdicts.iter().for_each(|v| counts.note(*v));
        let end = rows.last().unwrap().t;
        Metrics {
            counts,
            chip_steps: (0..rows.len())
                .filter(|&k| walking[k] && chip_errors[k].is_some_and(|e| e > 5.0))
                .count(),
            fused_steps: steps.len(),
            fused_step_max: steps.iter().copied().fold(0.0, f64::max),
            longest_open_loop_s: longest,
            realigns: (0..rows.len())
                .filter(|&k| verdicts[k] == Verdict::RealignLong)
                .map(|k| {
                    let fallen = rows[k.saturating_sub(50)..=k].iter().any(|r| r.fallen);
                    (rows[k].t, walking[k] && !rows[k].fallen, fallen)
                })
                .collect(),
            clean_med: percentile(clean.clone(), 0.5),
            clean_p99: percentile(clean, 0.99),
            held_med: percentile(held.clone(), 0.5),
            held_p99: percentile(held.clone(), 0.99),
            held_max: held.iter().copied().fold(0.0, f64::max),
            last_second_max: (0..rows.len())
                .filter(|&k| rows[k].t >= end - 1.0)
                .map(|k| dev[k])
                .fold(0.0, f64::max),
        }
    }

    /// The reference's figure, to 0.01°.
    fn near(name: &str, rust: f64, python: f64) {
        assert!(
            (rust - python).abs() <= 0.01,
            "{name}: Rust {rust} against the Python reference's {python}"
        );
    }

    fn verdicts(m: &Metrics) -> [u64; 7] {
        let c = m.counts;
        [
            c.follow,
            c.accepted,
            c.rejected,
            c.slew,
            c.realign_long,
            c.realign_gap,
            c.fallback,
        ]
    }

    /// Verdict by verdict within 1 % (or one) of the reference's.
    fn like_python(m: &Metrics, python: [u64; 7]) {
        for (rust, python) in verdicts(m).into_iter().zip(python) {
            let slack = (python as f64 * 0.01).max(1.0);
            assert!(
                (rust as f64 - python as f64).abs() <= slack,
                "{:?} against the Python reference's {python:?}",
                verdicts(m)
            );
        }
    }

    /// Walking through two glitch runs: the chip's 72 one-tick steps (the 50 Hz glitch count)
    /// are none in what the policy would see; the longest the gyro carries the attitude alone
    /// is a third of a second; outside the glitches it stays within 1.5° of the chip.
    #[test]
    fn walking_through_glitches_takes_none_of_them() {
        let m = metrics(include_str!(
            "../../test-data/bno08x-fusion-replay/walk-glitch.csv"
        ));
        assert_eq!(verdicts(&m), [50, 567, 132, 0, 0, 0, 0], "{m:?}");
        like_python(&m, [50, 567, 132, 0, 0, 0, 0]);
        assert_eq!(m.chip_steps, 72);
        assert_eq!(m.fused_steps, 0);
        assert!(m.realigns.is_empty());
        near(
            "longest open loop",
            m.longest_open_loop_s,
            0.3402308600000019,
        );
        near("clean median", m.clean_med, 0.24022472540063058);
        near("clean P99", m.clean_p99, 1.4976655595583248);
        assert!(m.clean_p99 < 2.0, "{m:?}");
    }

    /// Standing still: nothing refused, and the fusion is the chip to a twentieth of a degree.
    #[test]
    fn standing_still_is_the_chip() {
        let m = metrics(include_str!(
            "../../test-data/bno08x-fusion-replay/home-still.csv"
        ));
        assert_eq!(verdicts(&m), [50, 250, 0, 0, 0, 0, 0], "{m:?}");
        like_python(&m, [50, 250, 0, 0, 0, 0, 0]);
        near("held median", m.held_med, 0.01835686127561385);
        near("held P99", m.held_p99, 0.04822575047501972);
        near("held max", m.held_max, 0.05048697450199928);
        assert!(m.held_med < 0.5 && m.held_p99 < 1.0, "{m:?}");
    }

    /// Glitches and two falls: no step, no whole realignment, and lying on the floor at the end
    /// the fusion is back within a degree of the chip.
    #[test]
    fn glitches_and_falls_leave_no_step() {
        let m = metrics(include_str!(
            "../../test-data/bno08x-fusion-replay/fall-walk.csv"
        ));
        assert_eq!(verdicts(&m), [51, 632, 88, 29, 0, 0, 0], "{m:?}");
        like_python(&m, [51, 632, 88, 29, 0, 0, 0]);
        assert_eq!(m.chip_steps, 43);
        assert_eq!(m.fused_steps, 0);
        near(
            "longest open loop",
            m.longest_open_loop_s,
            0.46032800000011775,
        );
        near("clean median", m.clean_med, 0.3739451721765738);
        near("clean P99", m.clean_p99, 7.556046261165057);
        near("last second", m.last_second_max, 0.6577658035897217);
        assert!(m.last_second_max < 1.0);
    }

    /// The one whole realignment in these recordings: after the robot fell, with chip and trunk
    /// still — its 7° is the only step, and it lands on a robot already on the floor.
    #[test]
    fn the_one_whole_realignment_comes_after_the_fall() {
        let m = metrics(include_str!(
            "../../test-data/bno08x-fusion-replay/fall-realign.csv"
        ));
        assert_eq!(verdicts(&m), [51, 536, 112, 0, 1, 0, 0], "{m:?}");
        like_python(&m, [51, 536, 112, 0, 1, 0, 0]);
        assert_eq!(m.realigns.len(), 1);
        let (t, walking_upright, fallen) = m.realigns[0];
        assert!((t - 102.9348469).abs() < 1e-6, "{t}");
        assert!(!walking_upright && fallen, "{m:?}");
        assert_eq!(m.fused_steps, 1);
        near("the step", m.fused_step_max, 6.9553327162602345);
        near("last second", m.last_second_max, 0.10386545743043264);
        near("clean median", m.clean_med, 0.34989472653901743);
    }

    /// The fusion runs in the sensor's axes and the policy reads the trunk's. With a real
    /// mount `m` — `q_s = q_t ⊗ m`, `ω_s = m⁻¹ ω_t m` — the sensor-frame fusion through
    /// `TrunkFrame(m)` is the trunk-frame fusion through an identity frame, value for value.
    #[test]
    fn fusing_in_sensor_axes_is_fusing_in_trunk_axes() {
        let rows = parse(include_str!(
            "../../test-data/bno08x-fusion-replay/fall-walk.csv"
        ));
        let (trunk, trunk_verdicts) = run(&rows);
        for mount in [
            crate::trunk_imu::BNO08X_DEFAULT_MOUNT,
            [0.459554518, 0.539556409, 0.532044137, 0.463268349],
        ] {
            let mount = normalise(mount);
            let sensor_rows: Vec<Row> = rows
                .iter()
                .map(|r| {
                    let v = [0.0, r.gyro[0], r.gyro[1], r.gyro[2]];
                    let w = mul(mul(conj(mount), v), mount);
                    Row {
                        t: r.t,
                        policy: r.policy.clone(),
                        limp: r.limp,
                        fallen: r.fallen,
                        quat: mul(r.quat, mount),
                        gyro: [w[1], w[2], w[3]],
                    }
                })
                .collect();
            let (sensor, sensor_verdicts) = run(&sensor_rows);
            assert_eq!(sensor_verdicts, trunk_verdicts);
            let mut in_mount = TrunkFrame::new(mount);
            let mut identity = TrunkFrame::new(IDENTITY);
            for k in 0..rows.len() {
                let a = in_mount.push(SensorReading {
                    gyro: sensor_rows[k].gyro,
                    quat: Some(sensor[k]),
                });
                let b = identity.push(SensorReading {
                    gyro: rows[k].gyro,
                    quat: Some(trunk[k]),
                });
                for i in 0..3 {
                    assert!((a.gravity[i] - b.gravity[i]).abs() < 1e-9, "{k}");
                    assert!((a.gyro[i] - b.gyro[i]).abs() < 1e-9, "{k}");
                }
            }
        }
    }
}
