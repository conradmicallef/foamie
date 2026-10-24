//! What the flight controller believes. In ideal mode the controllers read the true state; in "Real plane"
//! mode they read this estimate, built from the sensors the way a small autopilot does it:
//! - attitude: complementary (Mahony/DCM-style) filter on the gyro, corrected by gravity from the
//!   accelerometer with centripetal compensation, and by the compass for heading
//! - position and velocity: accelerometer dead reckoning corrected by delayed GPS fixes
//! - height: barometer and GPS
//! - airspeed: wingtip pitot

use crate::math::*;
use crate::model::{Truth, Wind, AC, G, LEGX, RHO, PTS};
use crate::sensors::{Meas, GPS_LATENCY, MAG_EARTH};

/// The flight controller's view of the aircraft. Controllers only ever read this.
#[derive(Clone, Copy)]
pub struct Nav {
    pub p: V3, pub v: V3, pub q: Q, pub w: V3,
    /// Total acceleration, world frame, low-pass filtered (for the impact predictor).
    pub af: V3,
    /// Forward airspeed and its magnitude (m/s), prop-wash speed estimate.
    pub vax: f64, pub air: f64, pub vi: f64,
    /// Sideslip proxy (+ = air arriving from the left), used for turn coordination.
    pub slip: f64,
    /// Height of the tail feet and of the lowest point above the ground.
    pub agl_tail: f64, pub low: f64,
    pub on_ground: bool,
    /// Wind direction hint for weathervaning (world, horizontal), if known.
    pub wind: Option<[f64; 2]>,
    /// Wind estimate (world, horizontal): exact in ideal mode, from GPS and pitot in wing-borne flight otherwise.
    pub wind_est: [f64; 2],
}

/// Exact state, for ideal mode.
pub fn nav_from_truth(tr: &Truth, wind: &Wind, af: V3) -> Nav {
    let va = qinv(tr.q, sub(tr.v, wind.at(tr.t, tr.p[2])));
    let air = len(va);
    let w = wind.at(tr.t, tr.p[2]);
    Nav {
        p: tr.p, v: tr.v, q: tr.q, w: tr.w, af,
        vax: tr.aux.vax, air, vi: tr.aux.vi,
        slip: va[1] / max(5.0, air),
        agl_tail: tr.p[2] + qrot(tr.q, [LEGX, 0.0, 0.0])[2],
        low: tr.lowest(),
        on_ground: tr.on_feet(),
        wind: Some([w[0], w[1]]),
        wind_est: [w[0], w[1]],
    }
}

const HIST: usize = 256;
/// GPS velocity window used for velocity-aided attitude (one fix interval).
const WIN: f64 = 0.2;

pub struct Estimator {
    pub q: Q, pub p: V3, pub v: V3,
    gyro_bias: V3,
    w: V3,
    af: V3,
    hist: [(V3, V3); HIST],
    /// Running integral of the accelerometer rotated into the world frame (for velocity-aided attitude).
    cum: [V3; HIST],
    acc_sum: V3,
    last_gps_v: Option<V3>,
    /// Attitude correction rate (world frame) from the last GPS fix, applied until the next one.
    att_corr: V3,
    hi: usize,
    pitot: f64,
    baro_ref: f64,
    baro_init: u32,
    thr_est: f64,
    wind_est: [f64; 2],
}

impl Estimator {
    /// Aligned on the pad, like an EKF that has settled before arming.
    pub fn new(tr: &Truth) -> Self {
        Estimator {
            q: tr.q, p: tr.p, v: tr.v, gyro_bias: [0.0; 3], w: [0.0; 3], af: [0.0; 3],
            hist: [(tr.p, tr.v); HIST], cum: [[0.0; 3]; HIST], acc_sum: [0.0; 3], last_gps_v: None, att_corr: [0.0; 3],
            hi: 0, pitot: 0.0,
            baro_ref: 0.0, baro_init: 0, thr_est: 0.0, wind_est: [0.0; 2],
        }
    }

    pub fn update(&mut self, m: &Meas, thr_cmd: f64, dt: f64, wind_hint: Option<[f64; 2]>) -> Nav {
        // ---- attitude
        let w_meas = sub(m.gyro, self.gyro_bias);
        self.w = add(mul(self.w, 0.6), mul(w_meas, 0.4)); // light filter for the rate loops
        let v_body = qinv(self.q, self.v);
        // expected specific force: centripetal (w x v) plus gravity reaction, in the body frame
        let f_exp = add(cross(w_meas, v_body), qinv(self.q, [0.0, 0.0, G]));
        let fm = len(m.acc);
        let trust = exp(-pow((fm - len(f_exp)) / 2.5, 2.0));
        let e_acc = if fm > 1.0 && len(f_exp) > 1.0 { cross(norm(m.acc), norm(f_exp)) } else { [0.0; 3] };
        // compass: heading error about world vertical
        let mw = qrot(self.q, m.mag);
        let (mh, rh) = (horiz(mw), horiz(MAG_EARTH));
        let yaw_err = if mh[2] > 0.1 { atan2(mh[0] * rh[1] - mh[1] * rh[0], mh[0] * rh[0] + mh[1] * rh[1]) } else { 0.0 };
        let e_mag = qinv(self.q, [0.0, 0.0, yaw_err]);
        // gravity reference is only trusted weakly (it is wrong whenever the aircraft accelerates);
        // the main tilt correction comes from comparing GPS velocity change with the integrated accelerometer
        let e_gps = qinv(self.q, self.att_corr);
        let k_acc = if self.last_gps_v.is_some() { 0.08 } else { 0.8 };
        let e = add(add(mul(e_acc, k_acc * trust), mul(e_mag, 0.25)), e_gps);
        self.gyro_bias = sub(self.gyro_bias, mul(add(add(mul(e_acc, k_acc * trust), mul(e_mag, 0.3)), e_gps), 0.05 * dt));
        self.q = qintegrate(self.q, add(w_meas, e), dt);

        // ---- velocity and position: dead reckoning on the accelerometer
        let a_w = sub(qrot(self.q, m.acc), [0.0, 0.0, G]);
        self.af = add(mul(self.af, 1.0 - min(1.0, dt / 0.25)), mul(a_w, min(1.0, dt / 0.25)));
        self.v = add(self.v, mul(a_w, dt));
        self.p = add(self.p, mul(self.v, dt));
        self.acc_sum = add(self.acc_sum, mul(add(a_w, [0.0, 0.0, G]), dt));
        self.hi = (self.hi + 1) % HIST;
        self.hist[self.hi] = (self.p, self.v);
        self.cum[self.hi] = self.acc_sum;

        // delayed GPS: compare with where we thought we were when the fix was taken
        if let Some(fix) = m.gps {
            let lag = ((GPS_LATENCY / dt) as usize).min(HIST - 1);
            let (pd, vd) = self.hist[(self.hi + HIST - lag) % HIST];
            let ep = sub(fix.p, pd);
            let ev = sub(fix.v, vd);
            // velocity-aided attitude (DCM style): over the last fix interval, the accelerometer integrated in the
            // world frame should equal the GPS velocity change plus gravity; the rotation between them is tilt error
            let wn = (WIN / dt) as usize;
            if let Some(v_old) = self.last_gps_v {
                let i1 = (self.hi + HIST - lag) % HIST;
                let i0 = (self.hi + HIST - (lag + wn).min(HIST - 1)) % HIST;
                let meas = sub(self.cum[i1], self.cum[i0]);
                let refv = add(sub(fix.v, v_old), [0.0, 0.0, G * WIN]);
                if len(meas) > 0.5 && len(refv) > 0.5 {
                    self.att_corr = mul(cross(norm(meas), norm(refv)), 0.5 / WIN);
                }
            }
            self.last_gps_v = Some(fix.v);
            let kp = [0.22, 0.22, 0.03];
            for i in 0..3 {
                self.p[i] += kp[i] * ep[i];
                self.v[i] += 0.3 * ev[i] + 0.08 * ep[i] * if i == 2 { 0.2 } else { 1.0 };
            }
            for h in self.hist.iter_mut() {
                for i in 0..3 { h.0[i] += kp[i] * ep[i]; h.1[i] += 0.3 * ev[i]; }
            }
        }
        // barometer: height above the pad (zeroed at arming)
        if let Some(b) = m.baro {
            if self.baro_init < 25 { self.baro_ref += (b - self.p[2] - self.baro_ref) / (self.baro_init as f64 + 1.0); self.baro_init += 1; }
            let e = b - self.baro_ref - self.p[2];
            let k = 0.06;
            self.p[2] += k * e; self.v[2] += 0.5 * k * e;
        }
        if let Some(pt) = m.pitot { self.pitot += (pt - self.pitot) * 0.5; }

        // ---- derived values for the controllers
        self.thr_est += (thr_cmd - self.thr_est) * min(1.0, dt / 0.08);
        let vax = self.pitot;
        let vf = max(0.0, vax);
        let thrust = AC.tmax * self.thr_est * clamp(1.0 - vf / AC.vp, 0.0, 1.0);
        let vi = sqrt(vf * vf + 2.0 * thrust / (RHO * AC.disc)) - vf;
        let tail_off = qrot(self.q, [LEGX, 0.0, 0.0])[2];
        let agl_tail = self.p[2] + tail_off;
        let mut low = f64::INFINITY;
        for pt in PTS.iter() { low = min(low, self.p[2] + qrot(self.q, pt.r)[2]); }
        let air = max(vax, 0.0);
        // lateral accelerometer as the sideslip proxy (no sideslip vane on a foamie)
        let slip = clamp(-m.acc[1] * 4.3 / max(25.0, air * air), -0.5, 0.5);
        // wind = ground velocity minus air velocity (pitot along the nose), only in steady wing-borne flight
        let xb = qrot(self.q, [1.0, 0.0, 0.0]);
        if vax > 8.0 && abs(xb[2]) < 0.35 {
            let k = min(1.0, dt / 6.0);
            for i in 0..2 { self.wind_est[i] += (self.v[i] - vax * xb[i] - self.wind_est[i]) * k; }
        }
        let on_ground = agl_tail < 0.07 && abs(self.v[2]) < 0.6 && len2([self.v[0], self.v[1]]) < 0.8;
        Nav {
            p: self.p, v: self.v, q: self.q, w: self.w, af: self.af,
            vax, air, vi, slip, agl_tail, low, on_ground, wind: wind_hint, wind_est: self.wind_est,
        }
    }
}
