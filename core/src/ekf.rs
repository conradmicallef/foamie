//! Error-state extended Kalman filter for "Real plane" mode — the same structure as ArduPilot's EKF.
//!
//! Nominal state: position, velocity, attitude quaternion, gyro bias, accelerometer bias.
//! Error state (15): δp, δv, δθ (body-frame attitude error), δb_gyro, δb_accel, with covariance P.
//! - Prediction at IMU rate (covariance every 10 ms).
//! - Delayed GPS position/velocity fused against the state at the time of the fix (innovation from history).
//! - Barometer and tail lidar for height, compass for heading, zero-velocity updates while sitting on the pad.
//! - Every update is innovation-gated (χ² test), so GPS glitches and lidar dropouts are rejected, not followed.
//! Because GPS velocity and the accelerometer are tied together through the attitude, tilt is observable during
//! manoeuvres without trusting the accelerometer as a gravity reference.

use crate::estimator::Nav;
use crate::math::*;
use crate::model::{Truth, AC, G, LEGX, PTS, RHO};
use crate::sensors::{Meas, GPS_LATENCY, MAG_EARTH};

const N: usize = 15;
type Mat = [[f64; N]; N];
const HIST: usize = 128;
/// Covariance prediction interval (steps).
const PRED_EVERY: usize = 5;

// noise (1σ)
const ACC_NOISE: f64 = 0.35;         // m/s²/√Hz, includes motor vibration (scaled up with throttle)
const GYRO_NOISE: f64 = 0.006;       // rad/s/√Hz
const GYRO_BIAS_RW: f64 = 5e-4;      // rad/s/√s
const ACC_BIAS_RW: f64 = 4e-3;       // m/s²/√s
const GPS_POS_H: f64 = 1.0;          // m (the receiver's slowly wandering error, treated as noise)
const GPS_POS_V: f64 = 1.8;
const GPS_VEL: f64 = 0.1;            // m/s
const GATE: f64 = 25.0;              // χ² gate (5σ) for one scalar measurement
/// Compass-motor interference calibration (body frame, per unit throttle²): 85% of the real effect.
const COMPASS_MOT: V3 = [0.034, 0.06, -0.026];

pub struct Ekf {
    pub q: Q, pub p: V3, pub v: V3, pub bg: V3, pub ba: V3,
    pub pm: Mat,
    step_n: usize,
    hist: [(V3, V3); HIST],
    hi: usize,
    w: V3,
    af: V3,
    pitot: f64,
    lidar: Option<f64>,
    lidar_age: f64,
    baro_ref: f64,
    baro_init: u32,
    thr_est: f64,
    wind_est: [f64; 2],
    last_acc: V3,
    last_gyro: V3,
    was_ground: bool,
    /// Low-passed body side forces for hover drag-fusion wind estimation.
    f_side: [f64; 2],
    /// Seconds of hover drag-fusion wind estimation so far (the estimate is used once it has settled).
    wind_t: f64,
    /// Count of rejected measurements (gating), for diagnostics.
    pub rejected: u32,
}

fn zero() -> Mat { [[0.0; N]; N] }
fn skew(a: V3) -> [[f64; 3]; 3] { [[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]] }
/// Body → world rotation matrix.
fn rot(q: Q) -> [[f64; 3]; 3] {
    let (x, y, z) = axes(q);
    [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
}
fn m3mul(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 { for j in 0..3 { for k in 0..3 { r[i][j] += a[i][k] * b[k][j]; } } }
    r
}

impl Ekf {
    /// Aligned on the pad (like an EKF that has settled before arming).
    pub fn new(tr: &Truth) -> Self {
        let mut pm = zero();
        let d = [0.05, 0.05, 0.05, 0.02, 0.02, 0.02, 0.01, 0.01, 0.03, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1];
        for i in 0..N { pm[i][i] = d[i] * d[i]; }
        Ekf {
            q: tr.q, p: tr.p, v: tr.v, bg: [0.0; 3], ba: [0.0; 3], pm, step_n: 0,
            hist: [(tr.p, tr.v); HIST], hi: 0, w: [0.0; 3], af: [0.0; 3], pitot: 0.0, lidar: None, lidar_age: 1.0,
            baro_ref: 0.0, baro_init: 0, thr_est: 0.0, wind_est: [0.0; 2], last_acc: [0.0, 0.0, G], last_gyro: [0.0; 3],
            was_ground: true, rejected: 0, f_side: [0.0; 2], wind_t: 0.0,
        }
    }

    fn predict_cov(&mut self, dt: f64) {
        let r = rot(self.q);
        let f = sub(self.last_acc, self.ba);
        let w = sub(self.last_gyro, self.bg);
        // Φ = I + F dt (first order), built in blocks
        let mut phi = zero();
        for i in 0..N { phi[i][i] = 1.0; }
        for i in 0..3 { phi[i][3 + i] = dt; }                       // δp ← δv
        let rf = m3mul(r, skew(f));
        for i in 0..3 { for j in 0..3 {
            phi[3 + i][6 + j] = -rf[i][j] * dt;                       // δv ← δθ
            phi[3 + i][12 + j] = -r[i][j] * dt;                       // δv ← δb_a
        } }
        let sw = skew(w);
        for i in 0..3 { for j in 0..3 { phi[6 + i][6 + j] -= sw[i][j] * dt; } phi[6 + i][9 + i] = -dt; } // δθ ← δθ, δb_g
        // P = Φ P Φᵀ + Q
        let mut tmp = zero();
        for i in 0..N { for k in 0..N { let a = phi[i][k]; if a != 0.0 { for j in 0..N { tmp[i][j] += a * self.pm[k][j]; } } } }
        let mut np = zero();
        for i in 0..N { for k in 0..N { let a = tmp[i][k]; if a != 0.0 { for j in 0..N { np[i][j] += a * phi[j][k]; } } } }
        let vib = 1.0 + 1.5 * self.thr_est;
        for i in 0..3 {
            np[3 + i][3 + i] += pow(ACC_NOISE * vib, 2.0) * dt;
            np[6 + i][6 + i] += pow(GYRO_NOISE * vib, 2.0) * dt;
            np[9 + i][9 + i] += GYRO_BIAS_RW * GYRO_BIAS_RW * dt;
            np[12 + i][12 + i] += ACC_BIAS_RW * ACC_BIAS_RW * dt;
        }
        // keep symmetric
        for i in 0..N { for j in (i + 1)..N { let m = 0.5 * (np[i][j] + np[j][i]); np[i][j] = m; np[j][i] = m; } }
        self.pm = np;
    }

    /// One scalar measurement update with a sparse row `h` (index, coefficient). Returns false if gated out.
    fn update(&mut self, dx: &mut [f64; N], h: &[(usize, f64)], innov: f64, r: f64) -> bool {
        let mut ph = [0.0; N];
        for i in 0..N { for &(j, c) in h { ph[i] += self.pm[i][j] * c; } }
        let mut s = r;
        for &(j, c) in h { s += c * ph[j]; }
        // innovation consistent with the error already applied in this batch
        let mut pred = 0.0;
        for &(j, c) in h { pred += c * dx[j]; }
        let y = innov - pred;
        if y * y / s > GATE { self.rejected += 1; return false; }
        let mut k = [0.0; N];
        for i in 0..N { k[i] = ph[i] / s; dx[i] += k[i] * y; }
        for i in 0..N { for j in 0..N { self.pm[i][j] -= k[i] * ph[j]; } }
        true
    }

    fn inject(&mut self, dx: &[f64; N]) {
        for i in 0..3 {
            self.p[i] += dx[i];
            self.v[i] += dx[3 + i];
            self.bg[i] += dx[9 + i];
            self.ba[i] += dx[12 + i];
        }
        let dth = [dx[6], dx[7], dx[8]];
        self.q = qnorm(qmul(self.q, [1.0, 0.5 * dth[0], 0.5 * dth[1], 0.5 * dth[2]]));
        // shift the history by the same correction so delayed innovations stay consistent
        for h in self.hist.iter_mut() { for i in 0..3 { h.0[i] += dx[i]; h.1[i] += dx[3 + i]; } }
    }

    pub fn update_all(&mut self, m: &Meas, thr_cmd: f64, dt: f64, wind_hint: Option<[f64; 2]>) -> Nav {
        // ---- prediction on the IMU
        let w_meas = sub(m.gyro, self.bg);
        let f = sub(m.acc, self.ba);
        self.last_acc = m.acc;
        self.last_gyro = m.gyro;
        self.q = qintegrate(self.q, w_meas, dt);
        let a_w = sub(qrot(self.q, f), [0.0, 0.0, G]);
        self.v = add(self.v, mul(a_w, dt));
        self.p = add(self.p, mul(self.v, dt));
        self.af = add(mul(self.af, 1.0 - min(1.0, dt / 0.25)), mul(a_w, min(1.0, dt / 0.25)));
        self.w = add(mul(self.w, 0.6), mul(w_meas, 0.4));
        self.hi = (self.hi + 1) % HIST;
        self.hist[self.hi] = (self.p, self.v);
        self.step_n += 1;
        if self.step_n % PRED_EVERY == 0 { self.predict_cov(dt * PRED_EVERY as f64); }
        self.thr_est += (thr_cmd - self.thr_est) * min(1.0, dt / 0.08);

        let mut dx = [0.0; N];
        let mut any = false;
        // ---- delayed GPS, compared with the state when the fix was taken
        if let Some(fix) = m.gps {
            let lag = ((GPS_LATENCY / dt) as usize).min(HIST - 1);
            let (pd, vd) = self.hist[(self.hi + HIST - lag) % HIST];
            let sig = [GPS_POS_H, GPS_POS_H, GPS_POS_V];
            for i in 0..3 { any |= self.update(&mut dx, &[(i, 1.0)], fix.p[i] - pd[i], sig[i] * sig[i]); }
            for i in 0..3 { any |= self.update(&mut dx, &[(3 + i, 1.0)], fix.v[i] - vd[i], GPS_VEL * GPS_VEL); }
        }
        // ---- tail lidar: exact height near the ground
        self.lidar_age += dt;
        if let Some(l) = m.lidar {
            self.lidar = l;
            self.lidar_age = 0.0;
            if let Some(r) = l {
                let dir = qrot(self.q, [-1.0, 0.0, 0.0]);
                let z = r * -dir[2] - qrot(self.q, [LEGX, 0.0, 0.0])[2];
                let sd = 0.015 + 0.006 * r;
                any |= self.update(&mut dx, &[(2, 1.0)], z - self.p[2], sd * sd);
            }
        }
        // ---- barometer (height above the pad); prop wash makes it read high at high throttle
        if let Some(b) = m.baro {
            if self.baro_init < 25 { self.baro_ref += (b - self.p[2] - self.baro_ref) / (self.baro_init as f64 + 1.0); self.baro_init += 1; }
            else {
                let near = self.lidar.is_some() && self.lidar_age < 0.2;
                let sd = 0.35 + 0.3 * self.thr_est + if near { 0.5 } else { 0.0 };
                any |= self.update(&mut dx, &[(2, 1.0)], b - self.baro_ref - 0.35 * self.thr_est - self.p[2], sd * sd);
            }
        }
        // ---- compass heading. Motor current bends the field; subtract it with a compass-motor calibration
        // (ArduPilot COMPASS_MOT), which in practice is only ~85% right
        {
            let mag = sub(m.mag, mul(COMPASS_MOT, self.thr_est * self.thr_est));
            let mw = qrot(self.q, mag);
            let (mh, rh) = (horiz(mw), horiz(MAG_EARTH));
            if mh[2] > 0.15 && self.step_n % 10 == 0 {
                let yaw_err = atan2(mh[0] * rh[1] - mh[1] * rh[0], mh[0] * rh[0] + mh[1] * rh[1]);
                let (x, y, z) = axes(self.q);
                let sd = 0.06 + 0.1 * self.thr_est * self.thr_est;
                // world yaw of the body-frame error: third row of the rotation matrix
                any |= self.update(&mut dx, &[(6, x[2]), (7, y[2]), (8, z[2])], yaw_err, sd * sd);
            }
        }
        // ---- zero-velocity update while sitting on the legs with the motor stopped
        if self.was_ground && self.thr_est < 0.05 && self.step_n % 10 == 0 {
            for i in 0..3 { any |= self.update(&mut dx, &[(3 + i, 1.0)], -self.v[i], 0.03 * 0.03); }
        }
        if any { self.inject(&dx); }
        if let Some(pt) = m.pitot { self.pitot += (pt - self.pitot) * 0.5; }

        // ---- derived values for the controllers (same meaning as the ideal-mode Nav)
        let vax = self.pitot;
        let vf = max(0.0, vax);
        let thrust = AC.tmax * self.thr_est * clamp(1.0 - vf / AC.vp, 0.0, 1.0);
        let vi = sqrt(vf * vf + 2.0 * thrust / (RHO * AC.disc)) - vf;
        let tail_off = qrot(self.q, [LEGX, 0.0, 0.0])[2];
        let agl_tail = match (self.lidar, self.lidar_age < 0.2) { (Some(r), true) => r * -qrot(self.q, [-1.0, 0.0, 0.0])[2], _ => self.p[2] + tail_off };
        let mut low = f64::INFINITY;
        for pt in PTS.iter() { low = min(low, self.p[2] + qrot(self.q, pt.r)[2]); }
        let air = max(vax, 0.0);
        let slip = clamp(-m.acc[1] * 4.3 / max(25.0, air * air), -0.5, 0.5);
        let xb = qrot(self.q, [1.0, 0.0, 0.0]);
        // hover wind from drag (like ArduPilot EK3_DRAG_*): with the nose up, the body y/z accelerometer axes are
        // horizontal and feel only aerodynamic side force, which grows with the square of the air speed
        let kf = min(1.0, dt / 0.4);
        self.f_side[0] += (f[1] - self.f_side[0]) * kf;
        self.f_side[1] += (f[2] - self.f_side[1]) * kf;
        if xb[2] > 0.8 && self.thr_est > 0.2 && len(self.v) < 4.0 {
            // same calibrated drag as the hover feed-forward (1.7 × flat plate)
            let inv = |fs: f64, cda: f64| signum(fs) * sqrt(abs(fs) * AC.m / (1.7 * 0.5 * RHO * cda));
            let rel = qrot(self.q, [0.0, inv(self.f_side[0], 0.125), inv(self.f_side[1], 0.24)]);
            let k = min(1.0, dt / 1.5);
            for i in 0..2 { self.wind_est[i] += (self.v[i] + rel[i] - self.wind_est[i]) * k; }
            self.wind_t += dt;
        }
        if vax > 8.0 && abs(xb[2]) < 0.35 {
            let k = min(1.0, dt / 6.0);
            for i in 0..2 { self.wind_est[i] += (self.v[i] - vax * xb[i] - self.wind_est[i]) * k; }
        }
        let on_ground = agl_tail < 0.07 && abs(self.v[2]) < 0.6 && len2([self.v[0], self.v[1]]) < 0.8;
        self.was_ground = on_ground;
        Nav {
            p: self.p, v: self.v, q: self.q, w: self.w, af: self.af,
            vax, air, vi, slip, agl_tail, low, on_ground,
            // weathervane on the estimated wind once it has settled; before that the hover trim is the hint
            wind: if self.wind_t > 3.0 { Some(self.wind_est) } else { wind_hint },
            wind_est: self.wind_est,
        }
    }
}
