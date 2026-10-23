//! Flight software: rate and attitude loops, pilot modes (ArduPilot-style), autopilot tasks and crash recovery.
//! Everything here reads only `Nav` (the flight controller's estimate), never the true state.

use crate::estimator::Nav;
use crate::math::*;
use crate::model::{Act, AC, G, LEGX};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Mode { Manual = 0, Fbwa = 1, Acro = 2, QStab = 3, QHover = 4, QLoiter = 5 }
impl Mode {
    pub fn from(c: u32) -> Mode {
        match c { 0 => Mode::Manual, 1 => Mode::Fbwa, 2 => Mode::Acro, 3 => Mode::QStab, 4 => Mode::QHover, _ => Mode::QLoiter }
    }
    pub fn is_q(self) -> bool { matches!(self, Mode::QStab | Mode::QHover | Mode::QLoiter) }
    /// Left stick is a climb rate about centre (otherwise raw throttle).
    pub fn climb_stick(self) -> bool { matches!(self, Mode::QHover | Mode::QLoiter) }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Task { Takeoff = 1, Circuit = 2, Land = 3, Recover = 4, Show = 5 }

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Phase { Lift = 1, PitchOver = 2, Fly = 3, Approach = 4, Pullup = 5, Brake = 6, Descend = 7, Pull = 8, HoverUp = 9, Hold = 10, Show = 11 }

#[derive(Clone, Copy)]
pub struct Auto {
    pub task: Task, pub phase: Phase, pub t: f64,
    pub hd: [f64; 2], pub ly0: f64, pub back: Mode, pub wing: bool, pub c: f64,
    /// Aerobatic show progress (used by `Task::Show`).
    pub show: crate::aerobatics::Show,
    /// A show to start once a takeoff from the ground has pitched over.
    pub then_show: Option<crate::aerobatics::Show>,
}

#[derive(Clone, Copy, Default)]
pub struct Sticks { pub rx: f64, pub ry: f64, pub lx: f64, pub ly: f64, pub l_active: bool }
impl Sticks {
    /// Throttle stick position 0…1 (left stick fully down = 0).
    pub fn thr(&self) -> f64 { (1.0 - self.ly) / 2.0 }
}

#[derive(Clone, Copy, Default)]
pub struct Integrators { pub p: f64, pub q: f64, pub r: f64, pub h: f64, pub v: f64, pub vx: f64, pub vy: f64 }

// ---- landing pad, circuit and tuning
pub const CIRC_ALT: f64 = 20.0;
pub const CIRC_V: f64 = 13.0;
pub const NCIRC: usize = 34;
pub fn circuit() -> [[f64; 2]; NCIRC] {
    let (cx, cy, r, half) = (0.0, 22.0, 26.0, 35.0);
    let mut pts = [[0.0; 2]; NCIRC];
    for i in 0..=16 { let a = -PI / 2.0 + PI * i as f64 / 16.0; pts[i] = [cx + half + r * cos(a), cy + r * sin(a)]; }
    for i in 0..=16 { let a = PI / 2.0 + PI * i as f64 / 16.0; pts[17 + i] = [cx - half + r * cos(a), cy + r * sin(a)]; }
    pts
}
const KV: f64 = 0.3;
pub struct LandTune { pub alt: f64, pub v: f64, pub k_trig: f64, pub b_trig: f64, pub a_brake: f64, pub v_max: f64, pub h_hold: f64 }
pub const LAND: LandTune = LandTune { alt: 9.0, v: 13.0, k_trig: 0.07, b_trig: 3.0, a_brake: 1.6, v_max: 3.0, h_hold: 6.0 };

/// Braking speed profile into a point: arrive with zero speed, never faster than `vmax`.
pub fn brake_to(to: [f64; 2], d: f64, a: f64, vmax: f64) -> [f64; 2] {
    let sp = min(min(vmax, sqrt(2.0 * a * max(0.0, d - 0.15))), 0.9 * d);
    if d > 1e-3 { [to[0] / d * sp, to[1] / d * sp] } else { [0.0, 0.0] }
}
fn expo(s: f64) -> f64 { s * (0.6 + 0.4 * s * s) }

/// The flight controller's loops. `nav` is refreshed every step before any of these run.
pub struct Fc {
    pub nav: Nav,
    pub i: Integrators,
    pub act: Act,
    pub circ: [[f64; 2]; NCIRC],
    pub loiter: Option<[f64; 2]>,
    pub land_t: f64,
}

impl Fc {
    pub fn new(nav: Nav) -> Self { Fc { nav, i: Integrators::default(), act: Act::default(), circ: circuit(), loiter: None, land_t: 0.0 } }
    pub fn reset_i(&mut self) { self.i = Integrators::default(); }
    fn axes(&self) -> (V3, V3, V3) { axes(self.nav.q) }

    /// Body rate loop: roll right, pitch up, yaw right (rad/s). Gains scheduled on the airflow over each surface.
    pub fn rate_loop(&mut self, pd: f64, qd: f64, rd: f64, dt: f64) {
        let n = &self.nav;
        let vt = max(0.0, n.vax) + n.vi;
        let vw = max(0.0, n.vax) + 0.4 * n.vi;
        let gt = clamp(12.0 / (vt + 1.0), 0.12, 1.6);
        let gw = clamp(12.0 / (vw + 1.0), 0.12, 1.6);
        let (ep, eq, er) = (pd - n.w[0], qd + n.w[1], rd + n.w[2]);
        let i = &mut self.i;
        i.p = clamp(i.p + 1.5 * ep * dt * gw, -0.35, 0.35);
        i.q = clamp(i.q + 1.0 * eq * dt * gt, -0.3, 0.3);
        i.r = clamp(i.r + 1.0 * er * dt * gt, -0.45, 0.45);
        self.act.a = cl1(0.12 * gw * ep + i.p);
        self.act.e = cl1(0.22 * gt * eq + i.q);
        self.act.r = cl1(0.22 * gt * er + i.r);
    }

    /// Point the nose along world direction `d` (quaternion-safe, works straight up). `spin` = roll rate about the nose.
    pub fn dir_ctl(&mut self, d: V3, spin: f64, dt: f64) {
        let db = qinv(self.nav.q, d);
        let up = atan2(db[2], db[0]);
        let right = atan2(-db[1], db[0]);
        self.rate_loop(spin, clamp(5.0 * up, -6.0, 6.0), clamp(5.0 * right, -6.0, 6.0), dt);
    }

    /// Hover: nose up, tilt toward a desired ground velocity, throttle holds a climb rate.
    pub fn hover_ctl(&mut self, vdes: [f64; 2], vz: f64, spin: f64, dt: f64, tilt_max: f64) {
        let ex = vdes[0] - self.nav.v[0];
        let ey = vdes[1] - self.nav.v[1];
        // integrator trims out wind and steady drag so the hover holds station
        let iv = clamp_len2([self.i.vx + 0.18 * ex * dt, self.i.vy + 0.18 * ey * dt], 0.45);
        self.i.vx = iv[0]; self.i.vy = iv[1];
        let t = clamp_len2([KV * ex + iv[0], KV * ey + iv[1]], tilt_max);
        self.dir_ctl(norm([t[0], t[1], 1.0]), spin, dt);
        self.hover_throttle(vz, dt);
    }

    /// Throttle that holds a climb rate whatever the tilt.
    pub fn hover_throttle(&mut self, vz: f64, dt: f64) {
        let (x, _, _) = self.axes();
        let cos_t = max(0.35, x[2]);
        let err = vz - self.nav.v[2];
        self.i.h = clamp(self.i.h + 1.5 * err * dt, -3.0, 3.0);
        let tn = AC.m * (G + 2.2 * err) / cos_t + self.i.h;
        let fac = clamp(1.0 - max(0.0, self.nav.vax) / AC.vp, 0.2, 1.0);
        self.act.thr = clamp(tn / (AC.tmax * fac), 0.0, 1.0);
    }

    /// Drive the attitude to `qt` along the shortest rotation, plus feed-forward rates.
    pub fn att_ctl(&mut self, qt: Q, ff: V3, dt: f64, k: f64) {
        let mut qe = qmul(conj(self.nav.q), qt);
        if qe[0] < 0.0 { qe = [-qe[0], -qe[1], -qe[2], -qe[3]]; }
        let sv = sqrt(qe[1] * qe[1] + qe[2] * qe[2] + qe[3] * qe[3]);
        let ang = 2.0 * atan2(sv, qe[0]);
        let e = if sv > 1e-6 { [qe[1] / sv * ang, qe[2] / sv * ang, qe[3] / sv * ang] } else { [0.0; 3] };
        self.rate_loop(clamp(k * e[0] + ff[0], -5.0, 5.0), clamp(-k * e[1] + ff[1], -5.0, 5.0), clamp(-k * e[2] + ff[2], -5.0, 5.0), dt);
    }

    /// Heading to fly: the nose direction, or the belly direction when the nose is near vertical.
    pub fn flight_heading(&self) -> f64 {
        let (x, _, z) = self.axes();
        let h = horiz(x);
        if h[2] > 0.35 { return atan2(h[1], h[0]); }
        let b = horiz(mul(z, if x[2] > 0.0 { -1.0 } else { 1.0 }));
        if b[2] > 0.1 { atan2(b[1], b[0]) } else { 0.0 }
    }
    fn bank(&self) -> f64 { let (_, y, z) = self.axes(); atan2(y[2], z[2]) }

    /// FBWA: bank and pitch angles from the stick, coordinated turns, wings level hands-off.
    pub fn fbwa(&mut self, sx: f64, sy: f64, yaw: f64, dt: f64) {
        let (phi_d, th_d) = (0.87 * sx, 0.06 + 0.5 * sy);
        let phi = self.bank();
        let v = max(5.0, self.nav.air);
        let ff = [0.0, (G / v) * sin(phi) * tan(clamp(phi, -1.2, 1.2)), (G / v) * sin(phi) - 1.5 * self.nav.slip + 1.5 * yaw];
        let qt = quat_euler(self.flight_heading(), th_d, phi_d);
        self.att_ctl(qt, ff, dt, 4.0);
    }

    /// Wing-borne guidance: bank and height targets, coordinated yaw, throttle on airspeed.
    pub fn fwd_ctl(&mut self, phi_d: f64, h_des: f64, v_des: f64, dt: f64) {
        let (x, _, _) = self.axes();
        let phi = self.bank();
        let th = asin(clamp(x[2], -1.0, 1.0));
        let v = max(5.0, self.nav.air);
        let th_d = clamp(0.06 + 0.045 * (h_des - self.nav.p[2]) - 0.05 * self.nav.v[2], -0.3, 0.45);
        let pd = clamp(4.0 * (phi_d - phi), -3.0, 3.0);
        let qd = clamp(4.0 * (th_d - th), -2.0, 2.0) + (G / v) * sin(phi) * tan(clamp(phi, -1.2, 1.2));
        let rd = (G / v) * sin(phi) - 1.5 * self.nav.slip;
        self.rate_loop(pd, qd, rd, dt);
        self.i.v = clamp(self.i.v + 0.06 * (v_des - v) * dt, -0.25, 0.35);
        self.act.thr = clamp(0.36 + 0.07 * (v_des - v) + self.i.v + 0.5 * max(0.0, th_d - 0.06), 0.05, 1.0);
    }

    /// L1-style lateral guidance toward a point; returns a bank target (right positive).
    pub fn bank_toward(&self, tx: f64, ty: f64, l: f64) -> f64 {
        let n = &self.nav;
        let sp = hypot(n.v[0], n.v[1]);
        let hdg = if sp > 3.0 { [n.v[0] / sp, n.v[1] / sp] } else {
            let h = horiz(self.axes().0);
            [if h[2] > 0.0 { h[0] } else { 1.0 }, h[1]]
        };
        let (dx, dy) = (tx - n.p[0], ty - n.p[1]);
        let eta = atan2(hdg[0] * dy - hdg[1] * dx, hdg[0] * dx + hdg[1] * dy);
        let v = max(6.0, sp);
        -clamp(atan(2.0 * v * v * sin(eta) / (l * G)), -0.85, 0.85)
    }

    /// Nearest point on the racetrack, then a carrot 22 m further along.
    pub fn circuit_target(&self) -> [f64; 2] {
        let p = self.nav.p;
        let (mut best, mut bd) = (0usize, f64::INFINITY);
        for (i, c) in self.circ.iter().enumerate() {
            let d = hypot(c[0] - p[0], c[1] - p[1]);
            if d < bd { bd = d; best = i; }
        }
        let (mut need, mut i, mut pt) = (22.0, best, self.circ[best]);
        while need > 0.0 {
            let j = (i + 1) % NCIRC;
            let (a, b) = (self.circ[i], self.circ[j]);
            let sl = hypot(b[0] - a[0], b[1] - a[1]);
            if sl >= need { pt = [a[0] + (b[0] - a[0]) * need / sl, a[1] + (b[1] - a[1]) * need / sl]; break; }
            need -= sl; i = j; pt = b;
        }
        pt
    }

    /// Wind as the flight controller knows it: given in ideal mode, otherwise inferred from the hover trim
    /// (the station-keeping integrator leans into the wind, like ArduPilot's weathervane using demanded attitude).
    fn wind_guess(&self) -> [f64; 2] {
        match self.nav.wind { Some(w) => w, None => [-self.i.vx * 15.0, -self.i.vy * 15.0] }
    }

    /// Hover weathervane: spin about the nose until the wing is edge-on to the airflow it will see.
    pub fn edge_on_spin(&self, vdes: [f64; 2]) -> f64 {
        let wd = self.wind_guess();
        let w = [wd[0] - vdes[0], wd[1] - vdes[1]];
        if len2(w) < 0.4 { return 0.0; }
        let yh = horiz(self.axes().1);
        if yh[2] < 0.3 { return 0.0; }
        let mut e = atan2(yh[0] * w[1] - yh[1] * w[0], yh[0] * w[0] + yh[1] * w[1]);
        if e > PI / 2.0 { e -= PI; }
        if e < -PI / 2.0 { e += PI; }
        clamp(2.0 * e, -1.2, 1.2)
    }

    /// Spin from the pilot's stick, or weathervaning when it is centred.
    pub fn spin_cmd(&self, yaw: f64, vdes: [f64; 2], vane: bool) -> f64 {
        if abs(yaw) > 0.08 || !vane { return 2.5 * yaw; }
        if self.axes().0[2] > 0.8 { self.edge_on_spin(vdes) } else { 0.0 }
    }

    /// Q-mode stick frame: the aircraft's own hover axes (belly = forward, right wing = right).
    pub fn hover_frame(&self) -> ([f64; 3], [f64; 3]) {
        let (x, y, z) = self.axes();
        let mut ef = horiz(mul(z, -1.0));
        if ef[2] < 0.25 { ef = horiz(x); }
        let mut er = horiz(mul(y, -1.0));
        if er[2] < 0.25 { er = [ef[1], -ef[0], 1.0]; }
        (ef, er)
    }

    /// Climb rate from the left stick, with ArduPilot-style land-speed limiting for the last metre.
    pub fn climb_from_stick(&self, thr: f64) -> f64 {
        let c = thr - 0.5;
        let vz = if abs(c) < 0.08 { 0.0 } else { (c - signum(c) * 0.08) / 0.42 * 3.0 };
        max(vz, -(0.5 + 0.6 * max(0.0, self.nav.agl_tail - 1.0)))
    }

    /// Belly direction for pitching over from the hover.
    pub fn belly_dir(&self) -> [f64; 2] {
        let (x, _, z) = self.axes();
        let b = horiz(mul(z, -1.0));
        if b[2] > 0.25 { return [b[0], b[1]]; }
        let h = horiz(x);
        if h[2] > 0.2 { [h[0], h[1]] } else { [0.0, 1.0] }
    }

    /// One step of a pilot mode.
    pub fn pilot(&mut self, mode: Mode, s: &Sticks, vane: bool, dt: f64) {
        // Stick convention as on a real transmitter: pushing the right stick up (forward) is nose down,
        // pulling it back is nose up. In the hover that same forward push tilts toward the belly.
        let (sx, sy, yaw, thr) = (s.rx, -s.ry, s.lx, s.thr());
        let pitch = -sy;
        match mode {
            Mode::Manual => { self.act = Act { a: expo(sx), e: expo(pitch), r: expo(yaw), thr }; }
            Mode::Acro => { self.rate_loop(6.0 * expo(sx), 5.0 * expo(pitch), 3.0 * expo(yaw), dt); self.act.thr = thr; }
            Mode::Fbwa => { self.fbwa(sx, pitch, yaw, dt); self.act.thr = thr; }
            Mode::QStab | Mode::QHover => {
                // stick = tilt angle in the hover frame (30° belly/back, 17° sideways)
                let (ef, er) = self.hover_frame();
                let (kf, ks) = (0.58, 0.3);
                let d = norm([ks * sx * er[0] + kf * sy * ef[0], ks * sx * er[1] + kf * sy * ef[1], 1.0]);
                let spin = self.spin_cmd(yaw, [0.0, 0.0], mode == Mode::QHover && vane);
                self.dir_ctl(d, spin, dt);
                if mode == Mode::QStab { self.act.thr = thr; } else { let vz = self.climb_from_stick(thr); self.hover_throttle(vz, dt); }
            }
            Mode::QLoiter => {
                // stick = ground velocity; centred = hold this spot
                let (ef, er) = self.hover_frame();
                let mut vdes = [2.0 * sx * er[0] + 3.0 * sy * ef[0], 2.0 * sx * er[1] + 3.0 * sy * ef[1]];
                let n = self.nav;
                if abs(sx) < 0.06 && abs(sy) < 0.06 {
                    let l = *self.loiter.get_or_insert([n.p[0] + n.v[0] * 0.4, n.p[1] + n.v[1] * 0.4]);
                    let dl = [l[0] - n.p[0], l[1] - n.p[1]];
                    vdes = brake_to(dl, len2(dl), 1.6, 2.0);
                } else { self.loiter = None; }
                let spin = self.spin_cmd(yaw, vdes, vane);
                let vz = self.climb_from_stick(thr);
                self.hover_ctl(vdes, vz, spin, dt, 0.6);
            }
        }
        // land detector: descending demand but no longer sinking, right at the ground
        let descending = if mode == Mode::QStab { thr < 0.06 } else { thr < 0.42 };
        let settled = self.nav.agl_tail < 0.35 && abs(self.nav.v[2]) < 0.25 && hypot(self.nav.v[0], self.nav.v[1]) < 0.6;
        self.land_t = if mode.is_q() && descending && (self.nav.on_ground || settled) { self.land_t + dt } else { 0.0 };
        if mode.is_q() && descending && (self.nav.on_ground || self.land_t > 0.4) {
            self.act.thr = 0.0; self.reset_i(); self.loiter = None;
        }
        self.tipped_guard();
    }

    /// Motor cut if the aircraft is lying on the ground (tipped over): protects the prop, like a crash check.
    pub fn tipped_guard(&mut self) {
        let x = self.axes().0;
        if self.nav.low < 0.15 && x[2] < 0.5 && len(self.nav.v) < 2.5 && self.nav.air < 4.0 { self.act.thr = 0.0; }
    }

    // ---- crash prediction
    /// Time until the lowest point reaches the ground (≤ 10 s ahead) and the sink rate then.
    pub fn predict_impact(&self) -> (f64, f64) {
        let n = &self.nav;
        let low = n.low;
        if low <= 0.02 { return (f64::INFINITY, 0.0); }
        let sp = len(n.v);
        if sp > 4.0 {
            // wing-borne: follow the current curved flight path (velocity vector turning at its present rate)
            let om_v = mul(cross(n.v, n.af), 1.0 / (sp * sp));
            let om = len(om_v);
            let ax = if om > 1e-6 { mul(om_v, 1.0 / om) } else { [0.0, 0.0, 1.0] };
            let (mut v, mut z, h) = (n.v, low, 0.05);
            let mut t = h;
            while t <= 10.0 {
                if om > 1e-6 {
                    let (c, s) = (cos(om * h), sin(om * h));
                    v = add(add(mul(v, c), mul(cross(ax, v), s)), mul(ax, dot(ax, v) * (1.0 - c)));
                }
                z += v[2] * h;
                if z <= 0.0 { return (t, abs(v[2])); }
                t += h;
            }
            return (f64::INFINITY, 0.0);
        }
        let (vz, az) = (n.v[2], clamp(n.af[2], -G, 0.0));
        let t = if az < -0.05 { let d = vz * vz - 2.0 * az * low; (-vz - sqrt(d)) / az }
            else if vz < -0.05 { low / -vz } else { f64::INFINITY };
        if t > 10.0 { (f64::INFINITY, 0.0) } else { (t, abs(vz + az * t)) }
    }
    /// Time this airframe needs to stop the sink.
    pub fn time_to_recover(&self) -> f64 {
        let (x, _, z) = self.axes();
        let n = &self.nav;
        let sp = len(n.v);
        if n.air > 8.0 && x[2] < 0.6 {
            let gam = asin(clamp(-n.v[2] / max(sp, 0.1), 0.0, 1.0));
            return 0.35 + if z[2] < 0.0 { 0.4 } else { 0.0 } + gam * sp / (2.5 * G);
        }
        let turn = (1.0 - x[2]) * 0.45 + if z[2] < -0.3 { 0.3 } else { 0.0 };
        0.5 + turn + max(0.0, -n.v[2]) / ((AC.tmax / (AC.m * G) - 1.0) * G) * 1.4
    }
    pub fn deliberate_landing(&self) -> bool {
        let x = self.axes().0;
        x[2] > 0.75 && self.nav.v[2] > -1.6 && hypot(self.nav.v[0], self.nav.v[1]) < 2.5
    }
    pub fn tail_z(&self) -> f64 { self.nav.p[2] + qrot(self.nav.q, [LEGX, 0.0, 0.0])[2] }
}
