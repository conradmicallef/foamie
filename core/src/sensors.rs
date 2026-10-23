//! Sensor models for "Real plane" mode: what a small flight controller actually measures, with noise,
//! bias, drift, vibration, update rates and latency typical of hobby-grade parts.

use crate::math::*;
use crate::model::{Truth, Wind, LEGX, G};

pub const GPS_RATE: f64 = 5.0;
pub const GPS_LATENCY: f64 = 0.16;
pub const BARO_RATE: f64 = 25.0;
pub const LIDAR_RATE: f64 = 25.0;
pub const LIDAR_MAX: f64 = 12.0;
pub const PITOT_RATE: f64 = 25.0;
/// Earth field at the field (unit-ish, north = +y, dipping 55° down).
pub const MAG_EARTH: V3 = [0.0, 0.574, -0.819];

#[derive(Clone, Copy)]
pub struct GpsFix { pub p: V3, pub v: V3 }

/// One step of measurements. `None` means that sensor has no new sample this step.
#[derive(Clone, Copy)]
pub struct Meas {
    pub gyro: V3,
    pub acc: V3,
    pub mag: V3,
    pub baro: Option<f64>,
    pub gps: Option<GpsFix>,
    /// Range along the tail (looks straight down in the hover); `Some(None)` = sample taken but out of range.
    pub lidar: Option<Option<f64>>,
    pub pitot: Option<f64>,
}

const HIST: usize = 128;

pub struct Sensors {
    rng: Rng,
    gyro_bias: V3,
    acc_bias: V3,
    baro_drift: f64,
    gps_err: V3,
    t_baro: f64, t_gps: f64, t_lidar: f64, t_pitot: f64,
    /// Ring buffer of true position and velocity for GPS latency.
    hist: [(V3, V3); HIST],
    hi: usize,
    hist_dt: f64,
}

impl Sensors {
    pub fn new(seed: u64) -> Self {
        let mut rng = Rng::new(seed ^ 0x5EED_5E25);
        let gyro_bias = rng.gauss3(0.01);
        let acc_bias = rng.gauss3(0.08);
        Sensors {
            rng, gyro_bias, acc_bias, baro_drift: 0.0, gps_err: [0.0; 3],
            t_baro: 0.0, t_gps: 0.0, t_lidar: 0.0, t_pitot: 0.0,
            hist: [([0.0; 3], [0.0; 3]); HIST], hi: 0, hist_dt: 0.002,
        }
    }

    pub fn sample(&mut self, tr: &Truth, wind: &Wind, dt: f64) -> Meas {
        let r = &mut self.rng;
        // vibration from the motor grows with throttle (soft-mounted board)
        let vib = tr.thr;
        // gyro: bias random walk + white noise + vibration
        self.gyro_bias = add(self.gyro_bias, r.gauss3(0.0004 * sqrt(dt)));
        let gyro = add(add(tr.w, self.gyro_bias), add(r.gauss3(0.006), r.gauss3(0.02 * vib)));
        // accelerometer: specific force + bias + noise + vibration
        let acc = add(add(tr.f_body, self.acc_bias), add(r.gauss3(0.12), r.gauss3(0.9 * vib)));
        // compass: earth field + interference from motor current near the board
        let mag = add(add(qinv(tr.q, MAG_EARTH), mul([0.04, 0.07, -0.03], tr.thr * tr.thr)), r.gauss3(0.008));

        // history for delayed GPS
        self.hist_dt = dt;
        self.hi = (self.hi + 1) % HIST;
        self.hist[self.hi] = (tr.p, tr.v);

        self.t_baro += dt; self.t_gps += dt; self.t_lidar += dt; self.t_pitot += dt;
        let mut baro = None;
        if self.t_baro >= 1.0 / BARO_RATE {
            self.t_baro = 0.0;
            self.baro_drift += r.gauss() * 0.02 * sqrt(1.0 / BARO_RATE);
            // prop wash over the board lowers static pressure: reads high at high throttle
            baro = Some(tr.p[2] + self.baro_drift + 0.35 * tr.thr + r.gauss() * 0.22);
        }
        let mut gps = None;
        if self.t_gps >= 1.0 / GPS_RATE {
            self.t_gps = 0.0;
            // slowly wandering position error (Gauss–Markov, ~1 m horizontal) + white noise
            let a = (-(1.0 / GPS_RATE) / 25.0f64).min(0.0);
            let k = exp(a);
            let sig = [0.8, 0.8, 1.4];
            for i in 0..3 { self.gps_err[i] = k * self.gps_err[i] + sqrt(1.0 - k * k) * sig[i] * r.gauss(); }
            let lag = (GPS_LATENCY / self.hist_dt) as usize;
            let (pd, vd) = self.hist[(self.hi + HIST - lag.min(HIST - 1)) % HIST];
            gps = Some(GpsFix {
                p: add(add(pd, self.gps_err), r.gauss3(0.12)),
                v: add(vd, r.gauss3(0.08)),
            });
        }
        let mut lidar = None;
        if self.t_lidar >= 1.0 / LIDAR_RATE {
            self.t_lidar = 0.0;
            // mounted at the tail, looking out along -x: straight down in the hover
            let dir = qrot(tr.q, [-1.0, 0.0, 0.0]);
            let tail = add(tr.p, qrot(tr.q, [LEGX, 0.0, 0.0]));
            lidar = Some(if dir[2] < -0.82 {
                // below its minimum range a lidar reports the minimum
                let rng_m = max(tail[2] / -dir[2], 0.02);
                if rng_m < LIDAR_MAX { Some(max(0.02, rng_m + r.gauss() * (0.01 + 0.004 * rng_m))) } else { None }
            } else { None });
        }
        let mut pitot = None;
        if self.t_pitot >= 1.0 / PITOT_RATE {
            self.t_pitot = 0.0;
            // wingtip pitot, outside the prop wash: forward airspeed, useless below ~3 m/s
            let va = qinv(tr.q, sub(tr.v, wind.at(tr.t, tr.p[2])));
            pitot = Some(max(0.0, va[0] + r.gauss() * 0.25));
        }
        let _ = G;
        Meas { gyro, acc, mag, baro, gps, lidar, pitot }
    }
}
