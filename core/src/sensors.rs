//! Sensor models for "Real plane" mode: what a small flight controller actually measures, with noise,
//! bias, drift, vibration, update rates and latency typical of hobby-grade parts.

use crate::math::*;
use crate::model::{Truth, Wind, G};

pub const GPS_RATE: f64 = 5.0;
pub const GPS_LATENCY: f64 = 0.16;
pub const BARO_RATE: f64 = 25.0;
/// Tail camera looking for the AprilTag on the pad.
pub const CAM_RATE: f64 = 20.0;
pub const CAM_LATENCY: f64 = 0.08;
/// Camera position in the body (in the tail, 8 cm above the feet) — it looks out along -x, straight down in the hover.
pub const CAM_POS: V3 = [-0.70, 0.0, 0.0];
/// 640 × 480 px, 70° horizontal field of view: focal length in pixels.
pub const CAM_W: f64 = 640.0;
pub const CAM_H: f64 = 480.0;
pub const CAM_F: f64 = 457.0;
/// Pixel noise of the tag corner detector (1σ).
pub const CAM_PX: f64 = 0.7;
/// Nested AprilTags centred on the pad: a big one for the approach, a small one in its centre for the last metre.
pub const TAGS: [f64; 2] = [0.40, 0.08];
/// Smallest tag the detector decodes reliably, in pixels across.
pub const TAG_MIN_PX: f64 = 22.0;
pub const PITOT_RATE: f64 = 25.0;
/// Earth field at the field (unit-ish, north = +y, dipping 55° down).
pub const MAG_EARTH: V3 = [0.0, 0.574, -0.819];

#[derive(Clone, Copy)]
pub struct GpsFix { pub p: V3, pub v: V3 }

/// One AprilTag detection: tag centre in camera coordinates (x right, y down, z along the optical axis), its
/// 1σ errors (lateral, depth), which tag, and where it was in the image (for the picture-in-picture).
#[derive(Clone, Copy)]
pub struct TagObs { pub t: V3, pub sd_lat: f64, pub sd_depth: f64, pub tag: usize, pub px: [f64; 3] }

/// Camera → body: camera x = body y (left wing), camera y = body -z (back), camera z = body -x (out of the tail).
pub fn cam_to_body(c: V3) -> V3 { [-c[2], c[0], -c[1]] }
pub fn body_to_cam(b: V3) -> V3 { [b[1], -b[2], -b[0]] }

/// What the tail camera would see of the tags from pose (p, q), without noise: the best decodable tag.
/// Returns (tag index, tag centre in camera frame, image u, v, size in px).
pub fn tag_view(p: V3, q: Q) -> Option<(usize, V3, f64, f64, f64)> {
    let cam = add(p, qrot(q, CAM_POS));
    for (i, &s) in TAGS.iter().enumerate() {
        let h = s / 2.0;
        let mut ok = true;
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for c in [[h, h], [h, -h], [-h, -h], [-h, h], [0.0, 0.0]] {
            let b = qinv(q, sub([c[0], c[1], 0.0], cam));
            let k = body_to_cam(b);
            if k[2] < 0.03 { ok = false; break; }
            let u = CAM_W / 2.0 + CAM_F * k[0] / k[2];
            let v = CAM_H / 2.0 + CAM_F * k[1] / k[2];
            if u < 4.0 || u > CAM_W - 4.0 || v < 4.0 || v > CAM_H - 4.0 { ok = false; break; }
            lo = [min(lo[0], u), min(lo[1], v)]; hi = [max(hi[0], u), max(hi[1], v)];
        }
        if !ok { continue; }
        let size = min(hi[0] - lo[0], hi[1] - lo[1]);
        // seen too obliquely (> 60° from straight on) the code does not decode
        let look = norm(sub(cam, [0.0, 0.0, 0.0]));
        if size < TAG_MIN_PX || look[2] < 0.5 { continue; }
        let t = body_to_cam(qinv(q, sub([0.0, 0.0, 0.0], cam)));
        return Some((i, t, 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), size));
    }
    None
}

/// One step of measurements. `None` means that sensor has no new sample this step.
#[derive(Clone, Copy)]
pub struct Meas {
    pub gyro: V3,
    pub acc: V3,
    pub mag: V3,
    pub baro: Option<f64>,
    pub gps: Option<GpsFix>,
    /// AprilTag detection, delivered `CAM_LATENCY` after the frame was taken.
    pub tag: Option<TagObs>,
    pub pitot: Option<f64>,
}

const HIST: usize = 128;

pub struct Sensors {
    rng: Rng,
    gyro_bias: V3,
    acc_bias: V3,
    baro_drift: f64,
    gps_err: V3,
    t_baro: f64, t_gps: f64, t_cam: f64, t_pitot: f64,
    /// Detections taken but not yet delivered (latency): (deliver time, observation).
    cam_q: [Option<(f64, TagObs)>; 4],
    /// Camera mounting error (small rotation, rad) — every build is a little off.
    cam_mis: V3,
    pub cam_on: bool,
    t: f64,
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
        let cam_mis = rng.gauss3(0.008);
        Sensors {
            rng, gyro_bias, acc_bias, baro_drift: 0.0, gps_err: [0.0; 3],
            t_baro: 0.0, t_gps: 0.0, t_cam: 0.0, t_pitot: 0.0, cam_q: [None; 4], cam_mis, cam_on: true, t: 0.0,
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

        self.t_baro += dt; self.t_gps += dt; self.t_cam += dt; self.t_pitot += dt; self.t += dt;
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
        // ---- tail camera: AprilTag detections, delivered after the processing latency
        if self.t_cam >= 1.0 / CAM_RATE {
            self.t_cam = 0.0;
            // motion blur above ~2.5 rad/s, and the odd frame lost to glare or a failed decode
            let blur = len(tr.w) > 2.5;
            if self.cam_on && !blur && r.uni() > 0.04 {
                // the camera is mounted slightly crooked: it sees the world through a small extra rotation
                let qm = qnorm([1.0, 0.5 * self.cam_mis[0], 0.5 * self.cam_mis[1], 0.5 * self.cam_mis[2]]);
                if let Some((tag, t, u, v, size)) = tag_view(tr.p, qmul(tr.q, qm)) {
                    let d = len(t);
                    let sd_lat = d * CAM_PX / CAM_F;
                    // depth from the tag's apparent size: error grows with the square of range
                    let sd_depth = 0.6 * d * d * CAM_PX / (CAM_F * TAGS[tag]) + 0.003;
                    let n = [r.gauss() * sd_lat, r.gauss() * sd_lat, r.gauss() * sd_depth];
                    // depth error scales the whole vector (the tag looks bigger or smaller)
                    let k = 1.0 + n[2] / d;
                    let tm = [t[0] * k + n[0], t[1] * k + n[1], t[2] * k];
                    let obs = TagObs { t: tm, sd_lat, sd_depth, tag, px: [u, v, size] };
                    if let Some(slot) = self.cam_q.iter_mut().find(|s| s.is_none()) { *slot = Some((self.t + CAM_LATENCY, obs)); }
                }
            }
        }
        let mut tag = None;
        for slot in self.cam_q.iter_mut() {
            if let Some((td, o)) = *slot { if self.t >= td - 1e-9 { tag = Some(o); *slot = None; break; } }
        }
        let mut pitot = None;
        if self.t_pitot >= 1.0 / PITOT_RATE {
            self.t_pitot = 0.0;
            // wingtip pitot, outside the prop wash: forward airspeed, useless below ~3 m/s
            let va = qinv(tr.q, sub(tr.v, wind.at(tr.t, tr.p[2])));
            pitot = Some(max(0.0, va[0] + r.gauss() * 0.25));
        }
        let _ = G;
        Meas { gyro, acc, mag, baro, gps, tag, pitot }
    }
}
