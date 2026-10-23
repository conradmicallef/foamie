//! Small fixed-size vector and quaternion helpers.
//! World frame: x east, y north, z up. Body frame: x nose, y left wing, z canopy.

pub type V3 = [f64; 3];
/// Quaternion `[w, x, y, z]`, rotating body vectors into the world frame.
pub type Q = [f64; 4];

pub const PI: f64 = core::f64::consts::PI;
pub const D2R: f64 = PI / 180.0;

#[inline] pub fn sin(x: f64) -> f64 { libm::sin(x) }
#[inline] pub fn cos(x: f64) -> f64 { libm::cos(x) }
#[inline] pub fn tan(x: f64) -> f64 { libm::tan(x) }
#[inline] pub fn atan(x: f64) -> f64 { libm::atan(x) }
#[inline] pub fn atan2(y: f64, x: f64) -> f64 { libm::atan2(y, x) }
#[inline] pub fn asin(x: f64) -> f64 { libm::asin(x) }
#[inline] pub fn acos(x: f64) -> f64 { libm::acos(x) }
#[inline] pub fn sqrt(x: f64) -> f64 { libm::sqrt(x) }
#[inline] pub fn exp(x: f64) -> f64 { libm::exp(x) }
#[inline] pub fn ln(x: f64) -> f64 { libm::log(x) }
#[inline] pub fn pow(x: f64, y: f64) -> f64 { libm::pow(x, y) }
#[inline] pub fn hypot(x: f64, y: f64) -> f64 { libm::hypot(x, y) }
#[inline] pub fn abs(x: f64) -> f64 { libm::fabs(x) }
#[inline] pub fn round(x: f64) -> f64 { libm::round(x) }
#[inline] pub fn signum(x: f64) -> f64 { if x > 0.0 { 1.0 } else if x < 0.0 { -1.0 } else { 0.0 } }

#[inline] pub fn clamp(x: f64, a: f64, b: f64) -> f64 { if x < a { a } else if x > b { b } else { x } }
#[inline] pub fn cl1(x: f64) -> f64 { clamp(x, -1.0, 1.0) }
#[inline] pub fn min(a: f64, b: f64) -> f64 { if a < b { a } else { b } }
#[inline] pub fn max(a: f64, b: f64) -> f64 { if a > b { a } else { b } }

#[inline] pub fn add(a: V3, b: V3) -> V3 { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
#[inline] pub fn sub(a: V3, b: V3) -> V3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
#[inline] pub fn mul(a: V3, k: f64) -> V3 { [a[0] * k, a[1] * k, a[2] * k] }
#[inline] pub fn dot(a: V3, b: V3) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
#[inline] pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline] pub fn len(a: V3) -> f64 { sqrt(dot(a, a)) }
#[inline] pub fn norm(a: V3) -> V3 { let l = len(a); if l > 0.0 { mul(a, 1.0 / l) } else { a } }

/// Unit horizontal direction and the horizontal length of `a`: `[x, y, length]`.
#[inline] pub fn horiz(a: V3) -> [f64; 3] {
    let l = hypot(a[0], a[1]);
    if l > 1e-6 { [a[0] / l, a[1] / l, l] } else { [0.0, 0.0, 0.0] }
}
#[inline] pub fn len2(a: [f64; 2]) -> f64 { hypot(a[0], a[1]) }
#[inline] pub fn clamp_len2(v: [f64; 2], m: f64) -> [f64; 2] {
    let l = len2(v);
    if l > m { [v[0] * m / l, v[1] * m / l] } else { v }
}
#[inline] pub fn mul2(v: [f64; 2], k: f64) -> [f64; 2] { [v[0] * k, v[1] * k] }

#[inline] pub fn qmul(a: Q, b: Q) -> Q {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}
#[inline] pub fn conj(q: Q) -> Q { [q[0], -q[1], -q[2], -q[3]] }
/// Body → world.
#[inline] pub fn qrot(q: Q, v: V3) -> V3 {
    let u = [q[1], q[2], q[3]];
    let t = mul(cross(u, v), 2.0);
    add(add(v, mul(t, q[0])), cross(u, t))
}
/// World → body.
#[inline] pub fn qinv(q: Q, v: V3) -> V3 { qrot(conj(q), v) }
#[inline] pub fn qnorm(q: Q) -> Q {
    let l = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    [q[0] / l, q[1] / l, q[2] / l, q[3] / l]
}
/// Integrate body rates `w` over `dt`.
#[inline] pub fn qintegrate(q: Q, w: V3, dt: f64) -> Q {
    let dq = qmul(q, [0.0, w[0], w[1], w[2]]);
    qnorm([q[0] + 0.5 * dq[0] * dt, q[1] + 0.5 * dq[1] * dt, q[2] + 0.5 * dq[2] * dt, q[3] + 0.5 * dq[3] * dt])
}
/// Heading (from east, anticlockwise), pitch (+ nose up) and bank (+ right wing down) as a quaternion.
pub fn quat_euler(psi: f64, th: f64, phi: f64) -> Q {
    qmul(
        qmul([cos(psi / 2.0), 0.0, 0.0, sin(psi / 2.0)], [cos(-th / 2.0), 0.0, sin(-th / 2.0), 0.0]),
        [cos(phi / 2.0), sin(phi / 2.0), 0.0, 0.0],
    )
}
/// Body axes in the world frame: nose, left wing, canopy.
#[inline] pub fn axes(q: Q) -> (V3, V3, V3) {
    (qrot(q, [1.0, 0.0, 0.0]), qrot(q, [0.0, 1.0, 0.0]), qrot(q, [0.0, 0.0, 1.0]))
}

/// Deterministic xorshift64* generator with Gaussian samples (sensor noise, gusts, airframe variation).
#[derive(Clone, Copy)]
pub struct Rng { s: u64, spare: Option<f64> }
impl Rng {
    pub fn new(seed: u64) -> Self { Rng { s: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1, spare: None } }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.s;
        x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
        self.s = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// Uniform in [0, 1).
    pub fn uni(&mut self) -> f64 { (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64 }
    /// Standard normal.
    pub fn gauss(&mut self) -> f64 {
        if let Some(s) = self.spare.take() { return s; }
        let u1 = max(self.uni(), 1e-12);
        let u2 = self.uni();
        let r = sqrt(-2.0 * ln(u1));
        self.spare = Some(r * sin(2.0 * PI * u2));
        r * cos(2.0 * PI * u2)
    }
    pub fn gauss3(&mut self, s: f64) -> V3 { [self.gauss() * s, self.gauss() * s, self.gauss() * s] }
}
