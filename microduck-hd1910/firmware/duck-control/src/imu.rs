// Modified 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; BNO08x gyro-integrated attitude; see IMU-MODULE.md at the repository root.
//! The robot's trunk orientation: what every trunk IMU shares, and the bus board's bytes.
//!
//! Two parts, because the sensor can be wired more than one way
//! ([`crate::trunk_imu::TrunkImuKind`]) and only the bus board is this module's byte format:
//!
//! - [`TrunkFrame`] is the post-processing: the sensor→trunk mount, holding the last good
//!   quaternion, the 25-sample ready gate and the three-point median. Every source feeds it
//!   a reading in the sensor's own axes, so [`ImuData`] means the same thing whichever chip
//!   produced it.
//! - [`SflpDecoder`] is the `imu_to_dxl` v2 board (LSM6DSV16X) on the servo bus: its 12-byte
//!   block parsed into such a reading, then the frame. The block is fetched in the same
//!   `sync_read` as the servos, and the chip's SFLP ships a game-rotation quaternion and
//!   estimates its own gyro bias, so there is no fusion to run on the host. Its gyro and
//!   quaternion conversions are shared with [`crate::lsm6dsv16x`], the same chip on a breakout.
//!
//! Block layout at address 124:
//!
//! | bytes | contents |
//! |---|---|
//! | 0..6  | gyro x/y/z, `i16` LE raw counts, ±500 dps |
//! | 6..12 | SFLP quaternion x/y/z as IEEE half-precision; `w = √(1 − x² − y² − z²)` |
//!
//! The board's full diagnostic block is 20 bytes (it also carries raw accelerometer, a
//! sample counter and status flags). The control loop consumes only the first 12 so the
//! read fits alongside the servos in one transaction.

use crate::model::NUM_JOINTS;

/// Bytes of the v2 board's block consumed per tick.
pub const IMU_BLOCK_LEN: usize = 12;

/// Accepted fused quaternion samples required before orientation is ready.
pub const SFLP_READY_SAMPLES: u32 = 25;

/// What the control loop knows about the robot's orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuData {
    /// Angular velocity in the trunk frame, rad/s.
    pub gyro: [f64; 3],
    /// Projected gravity in the trunk frame, unit vector. Upright is `[0, 0, -1]`.
    ///
    /// This is what the policy observes, and what fall detection thresholds on.
    pub gravity: [f64; 3],
    /// Orientation, trunk→world, scalar-first `[w, x, y, z]`.
    pub quat: [f64; 4],
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            gravity: [0.0, 0.0, -1.0],
            quat: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

/// ±500 dps at 17.5 mdps/LSB, in rad/s.
const GYRO_RAD_PER_LSB: f64 = 0.0175 * std::f64::consts::PI / 180.0;

/// One IMU reading in the sensor's own axes, before the mount is applied.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorReading {
    /// Angular velocity, rad/s.
    pub gyro: [f64; 3],
    /// Fused orientation, sensor→world, scalar-first. `None` when this reading carries no
    /// quaternion the chip actually produced; [`TrunkFrame`] then holds the last one.
    pub quat: Option<[f64; 4]>,
}

/// Sensor readings, expressed in the trunk frame.
///
/// Stateful only for spike rejection and for holding the last good quaternion — there is
/// no filter here. Keeping the state explicit means [`crate::io::FakeIo`] can drive the
/// same decoder in tests.
pub struct TrunkFrame {
    /// Sensor→trunk mounting rotation, scalar-first.
    mount: [f64; 4],
    /// Last quaternion the sensor actually produced. Held across readings that arrive before
    /// the fusion has output anything — snapping to identity would report a robot as upright
    /// when its orientation is simply unknown, which is the worst possible lie to tell fall
    /// detection.
    last_quat: [f64; 4],
    /// Readings carrying a live quaternion. Gates [`TrunkFrame::ready`].
    quat_samples: u32,
    gyro_history: [[f64; 3]; 2],
    gravity_history: [[f64; 3]; 2],
    /// The next reading fills the median's history instead of joining it.
    seed_median: bool,
}

impl TrunkFrame {
    pub fn new(mount: [f64; 4]) -> Self {
        Self {
            mount,
            last_quat: [1.0, 0.0, 0.0, 0.0],
            quat_samples: 0,
            gyro_history: [[0.0; 3]; 2],
            gravity_history: [[0.0, 0.0, -1.0]; 2],
            seed_median: false,
        }
    }

    /// Start the three-point median afresh at the next reading. For a stream that has just
    /// started over: the two readings before an outage say nothing about the robot after it,
    /// and a median across the gap would report the old orientation as the new one. The ready
    /// count is untouched — a restarted stream is not an unconverged chip.
    pub fn restart_median(&mut self) {
        self.seed_median = true;
    }

    /// Whether the chip has demonstrably produced fused output — roughly 0.25 s at 100 Hz.
    ///
    /// Until this is true the orientation is a default, not a measurement. Slice 2's fall
    /// detection must not run before it.
    pub fn ready(&self) -> bool {
        self.quat_samples >= SFLP_READY_SAMPLES
    }

    /// A sensor→world orientation as this frame's trunk→world one: the transform
    /// [`Self::push`] applies to the quaternion it keeps, with no state touched. `None` for
    /// one that is no rotation. How a second orientation — the gyro fusion's, beside the one
    /// the frame was given — is put in the same axes as the first.
    pub fn trunk_quat(&self, sensor_quat: [f64; 4]) -> Option<[f64; 4]> {
        let mount_inv = [
            self.mount[0],
            -self.mount[1],
            -self.mount[2],
            -self.mount[3],
        ];
        let q = mul(sensor_quat, mount_inv);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        (norm > 0.5).then(|| [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm])
    }

    pub fn push(&mut self, reading: SensorReading) -> ImuData {
        let gyro = rotate(self.mount, reading.gyro);
        if let Some(sensor_quat) = reading.quat {
            let mount_inv = [
                self.mount[0],
                -self.mount[1],
                -self.mount[2],
                -self.mount[3],
            ];
            let q = mul(sensor_quat, mount_inv);
            let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
            if norm > 0.5 {
                self.last_quat = [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm];
                self.quat_samples = self.quat_samples.saturating_add(1);
            }
        }

        // Normalise *before* the median, matching the runtime. Note the consequence: a
        // component-wise median across three unit vectors is not itself unit-norm, so
        // during a transient the policy sees a slightly short vector. Steady state is
        // exact. Whether the training env expects strict unit norm is worth settling when
        // slice 2 wires up observations — normalising after the median would guarantee it,
        // but that is a behaviour change to a path that currently walks.
        let gravity = normalise(rotate_inverse(self.last_quat, [0.0, 0.0, -1.0]));
        if self.seed_median {
            self.seed_median = false;
            self.gyro_history = [gyro; 2];
            self.gravity_history = [gravity; 2];
        }

        let out = ImuData {
            gyro: median3_each(&self.gyro_history, gyro),
            gravity: median3_each(&self.gravity_history, gravity),
            quat: self.last_quat,
        };
        self.gyro_history = [self.gyro_history[1], gyro];
        self.gravity_history = [self.gravity_history[1], gravity];
        out
    }
}

/// Decodes the bus board's block into [`ImuData`]: [`parse_block`], then a [`TrunkFrame`].
pub struct SflpDecoder {
    frame: TrunkFrame,
}

impl Default for SflpDecoder {
    fn default() -> Self {
        Self::new(Self::DEFAULT_MOUNT)
    }
}

impl SflpDecoder {
    /// The board is mounted so that trunk = `[+raw_z, +raw_y, −raw_x]`, a +90° rotation
    /// about Y.
    pub const DEFAULT_MOUNT: [f64; 4] = [
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    ];

    pub fn new(mount: [f64; 4]) -> Self {
        Self {
            frame: TrunkFrame::new(mount),
        }
    }

    /// See [`TrunkFrame::ready`].
    pub fn ready(&self) -> bool {
        self.frame.ready()
    }

    pub fn decode(&mut self, raw: &[u8; IMU_BLOCK_LEN]) -> ImuData {
        self.frame.push(parse_block(raw))
    }
}

/// The board's 12 bytes as a reading in its own axes.
pub fn parse_block(raw: &[u8; IMU_BLOCK_LEN]) -> SensorReading {
    let gyro = lsm6dsv16x_gyro([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]]);

    // All-zero quaternion bytes mean SFLP has not written its table yet — the board
    // just powered up, or its init failed. The frame keeps the last good value.
    let packed = [
        u16::from_le_bytes([raw[6], raw[7]]),
        u16::from_le_bytes([raw[8], raw[9]]),
        u16::from_le_bytes([raw[10], raw[11]]),
    ];
    let quat = if packed != [0, 0, 0] {
        sflp_quaternion(packed)
    } else {
        None
    };
    SensorReading { gyro, quat }
}

/// Qinmo protocol 1.2: the same binary16 XYZ representation, with explicit readiness
/// and different gyro units. Zero XYZ is identity here, not the legacy SFLP sentinel.
pub(crate) fn qinmo_reading(raw: &[u8; IMU_BLOCK_LEN], ready: bool) -> Option<SensorReading> {
    let scale = std::f64::consts::PI / (16.4 * 180.0);
    let gyro = std::array::from_fn(|axis| {
        i16::from_le_bytes([raw[axis * 2], raw[axis * 2 + 1]]) as f64 * scale
    });
    let packed =
        std::array::from_fn(|axis| u16::from_le_bytes([raw[6 + axis * 2], raw[7 + axis * 2]]));
    let q = sflp_quaternion(packed)?;
    let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    Some(SensorReading {
        gyro,
        quat: ready.then(|| q.map(|v| v / norm)),
    })
}

/// The chip's gyro words — `i16` LE x/y/z at ±500 dps — in rad/s.
///
/// Shared with [`crate::lsm6dsv16x`], whose FIFO carries the same chip's same words: one
/// conversion, so the same bytes are the same rate whichever way the chip is wired.
pub(crate) fn lsm6dsv16x_gyro(raw: [u8; 6]) -> [f64; 3] {
    [
        i16::from_le_bytes([raw[0], raw[1]]) as f64 * GYRO_RAD_PER_LSB,
        i16::from_le_bytes([raw[2], raw[3]]) as f64 * GYRO_RAD_PER_LSB,
        i16::from_le_bytes([raw[4], raw[5]]) as f64 * GYRO_RAD_PER_LSB,
    ]
}

/// SFLP's game rotation — x/y/z as IEEE half-precision bit patterns — as a scalar-first
/// quaternion with `w = √(1 − x² − y² − z²)`, or `None` for one the chip never produced.
///
/// Shared with [`crate::lsm6dsv16x`] for the reason [`lsm6dsv16x_gyro`] is.
pub(crate) fn sflp_quaternion(packed: [u16; 3]) -> Option<[f64; 4]> {
    let (x, y, z) = (half(packed[0]), half(packed[1]), half(packed[2]));
    let norm_sq = x * x + y * y + z * z;
    // A quaternion the chip never produced would fail this; ≤ 1.02 allows for
    // half-precision rounding at full scale.
    if x.is_finite() && y.is_finite() && z.is_finite() && norm_sq <= 1.02 {
        let w = (1.0 - norm_sq).max(0.0).sqrt();
        Some([w, x, y, z])
    } else {
        None
    }
}

/// Single-sample spike rejection. A dropped or corrupted block shows up as one wild value;
/// a median over three discards it without the lag of an average.
fn median3_each(history: &[[f64; 3]; 2], now: [f64; 3]) -> [f64; 3] {
    let m = |a: f64, b: f64, c: f64| a.max(b).min(c).max(a.min(b));
    [
        m(history[0][0], history[1][0], now[0]),
        m(history[0][1], history[1][1], now[1]),
        m(history[0][2], history[1][2], now[2]),
    ]
}

/// IEEE 754 binary16 → f64. The LSM6DSV16X ships quaternion components in this format.
fn half(bits: u16) -> f64 {
    let sign = if bits & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = ((bits >> 10) & 0x1F) as i32;
    let frac = (bits & 0x3FF) as f64;
    match exp {
        0 => sign * frac * 2.0_f64.powi(-24),
        0x1F if frac == 0.0 => sign * f64::INFINITY,
        0x1F => f64::NAN,
        _ => sign * (1.0 + frac / 1024.0) * 2.0_f64.powi(exp - 15),
    }
}

/// Hamilton product, scalar-first.
fn mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let ([aw, ax, ay, az], [bw, bx, by, bz]) = (a, b);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// `q · v · q⁻¹`
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (t, c) = cross_terms(q, v);
    [
        v[0] + q[0] * t[0] + c[0],
        v[1] + q[0] * t[1] + c[1],
        v[2] + q[0] * t[2] + c[2],
    ]
}

/// `q⁻¹ · v · q` — world vector expressed in the body frame.
fn rotate_inverse(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (t, c) = cross_terms(q, v);
    [
        v[0] - q[0] * t[0] + c[0],
        v[1] - q[0] * t[1] + c[1],
        v[2] - q[0] * t[2] + c[2],
    ]
}

fn cross_terms(q: [f64; 4], v: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let (x, y, z) = (q[1], q[2], q[3]);
    let t = [
        (y * v[2] - z * v[1]) * 2.0,
        (z * v[0] - x * v[2]) * 2.0,
        (x * v[1] - y * v[0]) * 2.0,
    ];
    let c = [
        y * t[2] - z * t[1],
        z * t[0] - x * t[2],
        x * t[1] - y * t[0],
    ];
    (t, c)
}

/// Unit vector, falling back to "upright" rather than dividing by ~zero.
fn normalise(v: [f64; 3]) -> [f64; 3] {
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if mag > 0.1 {
        [v[0] / mag, v[1] / mag, v[2] / mag]
    } else {
        [0.0, 0.0, -1.0]
    }
}

/// Compile-time assurance that the joint count and this module stay independent — the IMU
/// block is fixed-size regardless of how many servos share the bus.
const _: () = assert!(NUM_JOINTS > 0);

#[cfg(test)]
mod tests {
    use super::*;

    /// [`TrunkFrame::trunk_quat`] is exactly what [`TrunkFrame::push`] keeps, and changes
    /// nothing.
    #[test]
    fn the_mount_transform_alone_is_what_push_keeps() {
        let mount = [0.459554518, 0.539556409, 0.532044137, 0.463268349];
        let mut frame = TrunkFrame::new(mount);
        let sensor = [0.8, 0.1, -0.5, 0.3];
        let alone = frame.trunk_quat(sensor).unwrap();
        assert_eq!(
            frame
                .push(SensorReading {
                    gyro: [0.0; 3],
                    quat: Some(sensor)
                })
                .quat,
            alone
        );
        assert_eq!(frame.trunk_quat([0.0; 4]), None);
    }

    /// Half-precision decoding is hand-rolled, so pin the cases that matter: zero, one,
    /// a negative, and a subnormal. A wrong exponent bias here silently tilts the horizon.
    #[test]
    fn half_precision_decodes_known_values() {
        assert_eq!(half(0x0000), 0.0);
        assert_eq!(half(0x3C00), 1.0);
        assert_eq!(half(0xBC00), -1.0);
        assert_eq!(half(0x3800), 0.5);
        assert!((half(0x3555) - 0.333).abs() < 1e-3);
    }

    /// A block of zeroes is the board saying "SFLP has not started". Reporting identity —
    /// i.e. perfectly upright — would tell fall detection the robot is fine when its
    /// orientation is simply unknown.
    #[test]
    fn all_zero_quaternion_bytes_hold_the_last_good_value() {
        let mut d = SflpDecoder::default();
        assert!(!d.ready());

        let zero = d.decode(&[0u8; IMU_BLOCK_LEN]);
        assert_eq!(zero.quat, [1.0, 0.0, 0.0, 0.0]);
        assert!(!d.ready(), "a zero block must not count as a live sample");

        // 0x3C00 = 1.0 in the x slot would exceed unit norm on its own; use a modest
        // rotation the chip could actually emit.
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[6..8].copy_from_slice(&0x3000u16.to_le_bytes()); // x = 0.125
        let live = d.decode(&block);
        assert_ne!(live.quat, [1.0, 0.0, 0.0, 0.0]);

        let held = d.decode(&[0u8; IMU_BLOCK_LEN]);
        assert_eq!(held.quat, live.quat, "zero block must hold, not reset");
    }

    /// `ready()` gates fall detection in slice 2. If it were true from the first block,
    /// the robot would be judged on a default orientation for the first quarter second.
    #[test]
    fn not_ready_until_the_chip_has_produced_output() {
        let mut d = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[6..8].copy_from_slice(&0x3000u16.to_le_bytes());
        for _ in 0..24 {
            d.decode(&block);
        }
        assert!(!d.ready());
        d.decode(&block);
        assert!(d.ready());
    }

    /// In steady state gravity must be a unit vector at any orientation — the policy
    /// observes it directly and was trained on normalised input.
    ///
    /// Three identical blocks per orientation so the median settles. Mid-transient the
    /// median blends three different unit vectors component-wise and the result is
    /// slightly short; that is the runtime's behaviour and is deliberately preserved (see
    /// the note in `decode`).
    #[test]
    fn gravity_is_a_unit_vector_in_steady_state() {
        let mut d = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        for packed in [0x0000u16, 0x3000, 0x3800, 0xB000] {
            block[6..8].copy_from_slice(&packed.to_le_bytes());
            d.decode(&block);
            d.decode(&block);
            let g = d.decode(&block).gravity;
            let mag = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-9,
                "gravity magnitude {mag} for {packed:#06x}"
            );
        }
    }

    /// The transient case, pinned so the bound is known rather than assumed: switching
    /// orientation between ticks blends three vectors and shortens the result. If this
    /// ever gets far from 1.0 the policy is being fed something training never saw.
    #[test]
    fn gravity_stays_close_to_unit_through_a_transient() {
        let mut d = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[6..8].copy_from_slice(&0x0000u16.to_le_bytes());
        d.decode(&block);
        block[6..8].copy_from_slice(&0x3800u16.to_le_bytes());
        let g = d.decode(&block).gravity;
        let mag = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
        assert!(mag > 0.5, "transient gravity collapsed to {mag}");
        assert!(mag <= 1.0 + 1e-9);
    }

    /// The decoder's output for a fixed run of FT6 blocks, pinned exactly: zeros before SFLP
    /// starts, held quaternions, the half-precision cases the chip can emit (NaN, an over-unit
    /// vector, a `w` clamped to zero, subnormals, a negative zero), a gyro spike for the median
    /// to eat, under the default mount and under the one a commissioned robot carries
    /// (`packaging/hd1910-calibration.robot.toml`).
    ///
    /// Exists for the change that put a second IMU behind the same post-processing. These
    /// numbers are what the policy walks on, so "the same arithmetic" is an equality check
    /// here rather than a claim in a commit message. If this fails, the bus board's
    /// observations changed — do not update the table to make it pass.
    #[test]
    // Exact outputs, not approximations: 0.7071… is what the decoder computed.
    #[allow(clippy::approx_constant)]
    fn golden_outputs_for_a_fixed_block_sequence() {
        const BLOCKS: [[u8; IMU_BLOCK_LEN]; 12] = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [120, 0, 172, 254, 75, 0, 0, 48, 0, 0, 0, 0],
            [24, 252, 250, 0, 184, 11, 0, 52, 0, 176, 102, 46],
            [255, 127, 0, 128, 1, 0, 0, 0, 0, 0, 0, 0],
            [15, 0, 16, 0, 17, 0, 0, 60, 0, 56, 0, 0],
            [251, 255, 5, 0, 251, 255, 0, 126, 0, 0, 0, 0],
            [200, 0, 200, 0, 56, 255, 0, 58, 0, 50, 0, 180],
            [0, 0, 0, 0, 0, 0, 0, 60, 0, 44, 0, 0],
            [199, 207, 133, 26, 82, 247, 1, 0, 1, 128, 255, 3],
            [200, 0, 200, 0, 56, 255, 0, 58, 0, 50, 0, 180],
            [200, 0, 200, 0, 56, 255, 0, 58, 0, 50, 0, 180],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128],
        ];
        let imu = |gyro, gravity, quat| ImuData {
            gyro,
            gravity,
            quat,
        };
        let default_mount = [
            imu([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0, 0.0]),
            imu(
                [0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [
                    0.701560760020114,
                    0.08838834764831843,
                    -0.701560760020114,
                    -0.08838834764831843,
                ],
            ),
            imu(
                [0.022907446432425566, 0.0, 0.0],
                [-0.84375, -0.24803918541230535, 0.0],
                [
                    0.5868449017612666,
                    0.24747011006614159,
                    -0.7636215970579036,
                    -0.1060832805271322,
                ],
            ),
            imu(
                [
                    0.022907446432425566,
                    -0.10384709049366261,
                    -0.03665191429188093,
                ],
                [-0.84375, -0.45246811305576984, 0.28871879773882275],
                [
                    0.5868449017612666,
                    0.24747011006614159,
                    -0.7636215970579036,
                    -0.1060832805271322,
                ],
            ),
            imu(
                [
                    0.005192354524683132,
                    0.004886921905584123,
                    -0.004581489286485117,
                ],
                [-0.84375, -0.45246811305576984, 0.28871879773882275],
                [
                    0.5868449017612666,
                    0.24747011006614159,
                    -0.7636215970579036,
                    -0.1060832805271322,
                ],
            ),
            imu(
                [
                    0.00030543261909699027,
                    0.0015271630954950385,
                    -0.004581489286485117,
                ],
                [-0.84375, -0.45246811305576984, 0.28871879773882275],
                [
                    0.5868449017612666,
                    0.24747011006614159,
                    -0.7636215970579036,
                    -0.1060832805271322,
                ],
            ),
            imu(
                [
                    -0.0015271630954950387,
                    0.004886921905584123,
                    -0.004581489286485117,
                ],
                [-0.84375, -0.45246811305576984, 0.28871879773882275],
                [
                    0.5447983326685064,
                    0.35355339059327384,
                    -0.2796332897235511,
                    -0.7071067811865476,
                ],
            ),
            imu(
                [-0.0015271630954950387, 0.0015271630954950385, 0.0],
                [
                    0.19531250000000006,
                    -0.45246811305576984,
                    -1.1102230246251568e-16,
                ],
                [
                    0.04410810913912309,
                    0.7057297462259694,
                    0.04410810913912309,
                    -0.7057297462259694,
                ],
            ),
            imu(
                [-0.06108652381980155, 0.061086523819801536, 0.0],
                [
                    0.19531250000000006,
                    -1.1920202047699028e-7,
                    -1.1102230246251568e-16,
                ],
                [
                    0.7071067377251787,
                    4.315837287515549e-5,
                    -0.7071068220188756,
                    4.30740791781337e-5,
                ],
            ),
            imu(
                [-0.06108652381980155, 0.061086523819801536, 0.0],
                [
                    0.19531250000000006,
                    -1.1920202047699028e-7,
                    -1.1102230246251568e-16,
                ],
                [
                    0.5447983326685064,
                    0.35355339059327384,
                    -0.2796332897235511,
                    -0.7071067811865476,
                ],
            ),
            imu(
                [
                    -0.06108652381980155,
                    0.061086523819801536,
                    -0.06108652381980153,
                ],
                [
                    0.19531250000000006,
                    -0.7806917862270766,
                    -0.5936104465567689,
                ],
                [
                    0.5447983326685064,
                    0.35355339059327384,
                    -0.2796332897235511,
                    -0.7071067811865476,
                ],
            ),
            imu(
                [
                    -0.06108652381980155,
                    0.061086523819801536,
                    -0.06108652381980153,
                ],
                [
                    0.19531250000000006,
                    -0.7806917862270766,
                    -0.5936104465567689,
                ],
                [0.7071067811865476, 0.0, -0.7071067811865476, 0.0],
            ),
        ];
        let robot_mount = [
            imu([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0, 0.0]),
            imu(
                [0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [
                    0.12495170864752603,
                    -0.6911553370316859,
                    0.12445044956609345,
                    -0.7008590844110374,
                ],
            ),
            imu(
                [0.03306433616955812, 0.0, 0.0],
                [
                    -0.7883188755230409,
                    0.3471665368808003,
                    -0.30097023291887215,
                ],
                [
                    0.28677963032465864,
                    -0.5750659075108415,
                    0.13716444714027093,
                    -0.7538186519915041,
                ],
            ),
            imu(
                [0.9046365853673919, 0.10102524762180379, 0.0367828924195899],
                [
                    -0.7883188755230409,
                    0.5366286140254248,
                    -0.30097023291887215,
                ],
                [
                    0.28677963032465864,
                    -0.5750659075108415,
                    0.13716444714027093,
                    -0.7538186519915041,
                ],
            ),
            imu(
                [
                    0.9046365853673919,
                    -0.005384745436993037,
                    0.004660219961653226,
                ],
                [
                    -0.7883188755230409,
                    0.5366286140254248,
                    -0.30097023291887215,
                ],
                [
                    0.28677963032465864,
                    -0.5750659075108415,
                    0.13716444714027093,
                    -0.7538186519915041,
                ],
            ),
            imu(
                [
                    0.0045955915689434015,
                    -0.0013653052847908155,
                    0.004660219961653226,
                ],
                [
                    -0.7883188755230409,
                    0.5366286140254248,
                    -0.30097023291887215,
                ],
                [
                    0.28677963032465864,
                    -0.5750659075108415,
                    0.13716444714027093,
                    -0.7538186519915041,
                ],
            ),
            imu(
                [
                    -0.0016567753915614455,
                    -0.005384745436993037,
                    0.004660219961653226,
                ],
                [
                    -0.7883188755230409,
                    0.5366286140254248,
                    -0.30097023291887215,
                ],
                [
                    0.3634664797908743,
                    -0.5050093135671382,
                    0.7364819130867856,
                    -0.26542815030607864,
                ],
            ),
            imu(
                [-0.0016567753915614455, -0.0013653052847908155, 0.0],
                [
                    0.26728560078366925,
                    0.5366286140254248,
                    0.013633963942761397,
                ],
                [
                    0.6977502401293072,
                    -0.006841785409611863,
                    0.7118779192697224,
                    0.07954634138735346,
                ],
            ),
            imu(
                [-0.06790602603077543, -0.05423128860696595, 0.0],
                [
                    0.26728560078366925,
                    0.10370697134673999,
                    0.013633963942761397,
                ],
                [
                    0.037620655498035636,
                    -0.7013555318342231,
                    0.03582424164460604,
                    -0.7109161187904106,
                ],
            ),
            imu(
                [-0.06790602603077543, -0.05423128860696595, 0.0],
                [
                    0.26728560078366925,
                    0.10370697134673999,
                    0.013633963942761397,
                ],
                [
                    0.3634664797908743,
                    -0.5050093135671382,
                    0.7364819130867856,
                    -0.26542815030607864,
                ],
            ),
            imu(
                [
                    -0.06790602603077543,
                    -0.05423128860696595,
                    0.06035254074641234,
                ],
                [0.26728560078366925, 0.7580739787767224, 0.5948800301870474],
                [
                    0.3634664797908743,
                    -0.5050093135671382,
                    0.7364819130867856,
                    -0.26542815030607864,
                ],
            ),
            imu(
                [
                    -0.06790602603077543,
                    -0.05423128860696595,
                    0.06035254074641234,
                ],
                [0.26728560078366925, 0.7580739787767224, 0.5948800301870474],
                [
                    0.03757726298627153,
                    -0.7013533907437678,
                    0.03586696698689637,
                    -0.7109183717402732,
                ],
            ),
        ];
        for (mount, expected) in [
            (SflpDecoder::DEFAULT_MOUNT, default_mount),
            (
                [0.037577263, 0.701353391, -0.035866967, 0.710918372],
                robot_mount,
            ),
        ] {
            let mut d = SflpDecoder::new(mount);
            for (i, (block, want)) in BLOCKS.iter().zip(expected).enumerate() {
                assert_eq!(d.decode(block), want, "block {i} under mount {mount:?}");
                assert!(!d.ready());
            }
            // Eight of those twelve carried a live quaternion; seventeen more make 25.
            for _ in 0..16 {
                d.decode(&BLOCKS[6]);
            }
            assert!(!d.ready());
            d.decode(&BLOCKS[6]);
            assert!(d.ready());
        }
    }

    /// After `restart_median` the next reading is reported as itself, not as the median of
    /// two readings from before the restart and this one; and the frame stays ready.
    #[test]
    fn a_restarted_median_reports_the_first_new_reading_as_itself() {
        let mut frame = TrunkFrame::new([1.0, 0.0, 0.0, 0.0]);
        let upright = SensorReading {
            gyro: [0.0; 3],
            quat: Some([1.0, 0.0, 0.0, 0.0]),
        };
        for _ in 0..30 {
            frame.push(upright);
        }
        // Pitched 30° about +Y: the half angle is 15°.
        let (c, v) = (15f64.to_radians().cos(), 15f64.to_radians().sin());
        let pitched = SensorReading {
            gyro: [0.0, 0.5, 0.0],
            quat: Some([c, 0.0, v, 0.0]),
        };
        frame.restart_median();
        let out = frame.push(pitched);
        assert!(frame.ready());
        assert!(
            (out.gravity[0] - 30f64.to_radians().sin()).abs() < 1e-12,
            "{:?}",
            out.gravity
        );
        assert_eq!(out.gyro, [0.0, 0.5, 0.0]);
        // Without a restart the same jump is held off by the median, as it always was.
        let mut plain = TrunkFrame::new([1.0, 0.0, 0.0, 0.0]);
        for _ in 0..30 {
            plain.push(upright);
        }
        assert_eq!(plain.push(pitched).gravity, [0.0, 0.0, -1.0]);
    }

    /// Gyro counts are signed. Reading them as unsigned would turn every negative rate
    /// into a large positive one, which reads as a robot spinning.
    #[test]
    fn gyro_counts_are_signed() {
        let mut d = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[0..2].copy_from_slice(&(-1000i16).to_le_bytes());
        // Three identical blocks so the median settles on this sample.
        d.decode(&block);
        d.decode(&block);
        let out = d.decode(&block);
        let magnitude: f64 = out.gyro.iter().map(|v| v.abs()).sum();
        assert!(magnitude > 0.0);
        let expected = 1000.0 * GYRO_RAD_PER_LSB;
        assert!(
            (magnitude - expected).abs() < 1e-9,
            "expected |gyro| {expected}, got {magnitude}"
        );
    }
}
