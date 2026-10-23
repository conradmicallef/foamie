//! Airframe, aerodynamics, propeller, battery, wind and ground contact: the "real world" the controllers fly in.

use crate::math::*;

pub const RHO: f64 = 1.225;
pub const G: f64 = 9.81;

/// 0.45 kg profile 3D foamie, 0.9 m span, 8 N static thrust (T/W 1.8), X tail legs.
pub struct Airframe {
    pub m: f64, pub tmax: f64, pub vp: f64, pub disc: f64, pub kq: f64,
    pub inertia: V3, pub ip: f64, pub wmax: f64, pub dmax: f64,
}
pub const AC: Airframe = Airframe {
    m: 0.45, tmax: 8.0, vp: 28.0, disc: 0.05, kq: 0.012,
    inertia: [0.015, 0.022, 0.032], ip: 3e-5, wmax: 1100.0, dmax: 0.75,
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Ctl { None, AilL, AilR, Elev, Rud }

/// A flat aerodynamic surface: position, area, normal, lift slope, stall angle, share of prop wash, control effectiveness.
pub struct Surf { pub r: V3, pub s: f64, pub n: V3, pub a: f64, pub stall: f64, pub wash: f64, pub tau: f64, pub ctl: Ctl, pub ar: f64 }
pub const SURF: [Surf; 5] = [
    Surf { r: [0.0, 0.23, 0.0], s: 0.095, n: [0.0, 0.0, 1.0], a: 4.3, stall: 0.28, wash: 0.4, tau: 0.45, ctl: Ctl::AilL, ar: 4.3 },
    Surf { r: [0.0, -0.23, 0.0], s: 0.095, n: [0.0, 0.0, 1.0], a: 4.3, stall: 0.28, wash: 0.4, tau: 0.45, ctl: Ctl::AilR, ar: 4.3 },
    Surf { r: [-0.6, 0.0, 0.0], s: 0.05, n: [0.0, 0.0, 1.0], a: 3.8, stall: 0.45, wash: 1.0, tau: 0.8, ctl: Ctl::Elev, ar: 3.0 },
    Surf { r: [-0.62, 0.0, 0.07], s: 0.04, n: [0.0, 1.0, 0.0], a: 2.8, stall: 0.5, wash: 0.95, tau: 0.8, ctl: Ctl::Rud, ar: 1.6 },
    Surf { r: [-0.08, 0.0, 0.0], s: 0.035, n: [0.0, 1.0, 0.0], a: 1.8, stall: 0.45, wash: 0.5, tau: 0.0, ctl: Ctl::None, ar: 1.0 },
];

pub const LEG: f64 = 0.22;
pub const LEGX: f64 = -0.78;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Kind { Foot = 0, Nose = 1, Top = 2, Belly = 3, Tip = 4 }

pub struct Point { pub r: V3, pub k: Kind, pub mu: f64 }
pub const NPTS: usize = 13;
pub const PTS: [Point; NPTS] = [
    Point { r: [LEGX, LEG, LEG], k: Kind::Foot, mu: 0.7 },
    Point { r: [LEGX, LEG, -LEG], k: Kind::Foot, mu: 0.7 },
    Point { r: [LEGX, -LEG, LEG], k: Kind::Foot, mu: 0.7 },
    Point { r: [LEGX, -LEG, -LEG], k: Kind::Foot, mu: 0.7 },
    Point { r: [0.37, 0.0, 0.0], k: Kind::Nose, mu: 0.5 },
    Point { r: [0.05, 0.0, 0.09], k: Kind::Top, mu: 0.5 },
    Point { r: [0.02, 0.0, -0.06], k: Kind::Belly, mu: 0.45 },
    Point { r: [0.0, 0.45, 0.0], k: Kind::Tip, mu: 0.5 },
    Point { r: [0.0, -0.45, 0.0], k: Kind::Tip, mu: 0.5 },
    Point { r: [-0.66, 0.19, 0.0], k: Kind::Tip, mu: 0.5 },
    Point { r: [-0.66, -0.19, 0.0], k: Kind::Tip, mu: 0.5 },
    Point { r: [-0.62, 0.0, 0.16], k: Kind::Top, mu: 0.5 },
    Point { r: [LEGX, 0.0, 0.0], k: Kind::Foot, mu: 0.7 },
];

/// Differences between the ideal model and one real, hand-built airframe ("Real plane" mode).
#[derive(Clone, Copy)]
pub struct Variation {
    pub thrust: f64,
    pub surf: [f64; 5],
    /// Centre of gravity offset from the design point, body frame (m).
    pub cg: V3,
    /// Built-in roll trim error from a slightly warped wing (rad of aileron).
    pub roll_trim: f64,
    pub servo_tau: f64,
    pub motor_tau: f64,
    pub torque: f64,
    pub battery: bool,
}
impl Variation {
    pub const IDEAL: Variation = Variation {
        thrust: 1.0, surf: [1.0; 5], cg: [0.0; 3], roll_trim: 0.0,
        servo_tau: 0.025, motor_tau: 0.06, torque: 1.0, battery: false,
    };
    pub fn random(rng: &mut Rng) -> Variation {
        let mut surf = [1.0; 5];
        for s in surf.iter_mut() { *s = 1.0 + 0.08 * clamp(rng.gauss(), -2.0, 2.0); }
        Variation {
            thrust: 1.0 + 0.05 * clamp(rng.gauss(), -2.0, 2.0),
            surf,
            cg: [0.012 * clamp(rng.gauss(), -2.0, 2.0), 0.004 * clamp(rng.gauss(), -2.0, 2.0), 0.003 * clamp(rng.gauss(), -2.0, 2.0)],
            roll_trim: 0.03 * clamp(rng.gauss(), -2.0, 2.0),
            servo_tau: 0.045,
            motor_tau: 0.09,
            torque: 1.0 + 0.15 * clamp(rng.gauss(), -2.0, 2.0),
            battery: true,
        }
    }
}

/// 3S 1000 mAh LiPo: open-circuit voltage from charge, sag under load, thrust follows voltage squared.
#[derive(Clone, Copy)]
pub struct Battery { pub soc: f64, pub volts: f64, pub amps: f64 }
impl Battery {
    pub const CAP_AH: f64 = 1.0;
    pub fn full() -> Self { Battery { soc: 1.0, volts: 12.6, amps: 0.0 } }
    pub fn ocv(soc: f64) -> f64 { 3.0 * (3.55 + 0.55 * soc + 0.1 * pow(soc, 8.0) - 0.25 * exp(-30.0 * soc)) }
    pub fn step(&mut self, thr: f64, dt: f64) {
        self.amps = 1.0 + 28.0 * pow(clamp(thr, 0.0, 1.0), 1.5);
        self.soc = max(0.0, self.soc - self.amps * dt / 3600.0 / Self::CAP_AH);
        self.volts = Self::ocv(self.soc) - 0.045 * self.amps;
    }
    /// Thrust available relative to the 11.7 V the motor was sized for.
    pub fn thrust_factor(&self) -> f64 { clamp(pow(self.volts / 11.7, 2.0), 0.3, 1.2) }
}

/// Gusty wind from the east with a surface gradient.
#[derive(Clone, Copy)]
pub struct Wind { pub speed: f64, pub phase: [f64; 4] }
impl Wind {
    pub fn at(&self, t: f64, z: f64) -> V3 {
        let w = self.speed * clamp(pow(max(z, 0.1) / 10.0, 0.3), 0.35, 1.15);
        if w == 0.0 { return [0.0; 3]; }
        let p = self.phase;
        let g = 1.0 + 0.28 * sin(t * 0.7 + p[0]) + 0.18 * sin(t * 1.9 + 1.0 + p[1]) + 0.1 * sin(t * 4.3 + 2.0 + p[2]);
        [-w * g, 0.25 * w * sin(t * 0.43 + p[3]), 0.12 * w * sin(t * 1.3 + p[0])]
    }
}

/// Values the rest of the sim reads off the airflow each step.
#[derive(Clone, Copy, Default)]
pub struct Aux { pub vax: f64, pub vi: f64, pub thrust: f64, pub alpha: f64, pub air: f64 }

#[derive(Clone, Copy)]
pub struct Truth {
    pub p: V3, pub v: V3, pub q: Q, pub w: V3,
    /// Actual (lagged) throttle and surface deflections (aileron, elevator, rudder; -1…1).
    pub thr: f64, pub sv: V3,
    pub contact: [bool; NPTS],
    pub t: f64,
    pub aux: Aux,
    /// Non-gravitational force / mass in the body frame (what an accelerometer feels).
    pub f_body: V3,
    pub batt: Battery,
}

#[derive(Clone, Copy)]
pub struct Contact { pub k: Kind, pub vz: f64, pub vh: f64, pub sp: f64 }

/// Actuator commands: aileron, elevator, rudder (-1…1), throttle (0…1).
#[derive(Clone, Copy, Default)]
pub struct Act { pub a: f64, pub e: f64, pub r: f64, pub thr: f64 }

pub fn lift_coefs(alpha: f64, s: &Surf) -> (f64, f64) {
    let near0 = if abs(alpha) < PI / 2.0 { abs(alpha) } else { PI - abs(alpha) };
    let sig = 1.0 / (1.0 + exp(-(near0 - s.stall) / 0.045));
    let lin = s.a * sin(alpha) * cos(alpha);
    let plate = 1.05 * sin(2.0 * alpha);
    let cl = (1.0 - sig) * lin + sig * plate;
    let sa = sin(alpha);
    let cd = 0.025 + (1.0 - sig) * cl * cl / (PI * s.ar * 0.8) + sig * 1.25 * sa * sa + 0.05 * sa * sa;
    (cl, cd)
}

impl Truth {
    pub fn standing() -> Self {
        // standing on its X legs on the pad, nose up, belly facing north
        let q1 = [core::f64::consts::FRAC_1_SQRT_2, 0.0, -core::f64::consts::FRAC_1_SQRT_2, 0.0];
        let qz = [core::f64::consts::FRAC_1_SQRT_2, 0.0, 0.0, core::f64::consts::FRAC_1_SQRT_2];
        Truth {
            p: [0.0, 0.0, -LEGX + 0.003], v: [0.0; 3], q: qmul(qz, q1), w: [0.0; 3],
            thr: 0.0, sv: [0.0; 3], contact: [true; NPTS], t: 0.0, aux: Aux::default(),
            f_body: [0.0, 0.0, 0.0], batt: Battery::full(),
        }
    }

    /// Advance the rigid body one step. Contact starts are pushed into `contacts`.
    pub fn step(&mut self, cmd: &Act, wind: &Wind, var: &Variation, dt: f64, contacts: &mut [Option<Contact>; 4]) {
        // motor and servo lag
        self.thr += (cmd.thr - self.thr) * min(1.0, dt / var.motor_tau);
        let tgt = [cl1(cmd.a + var.roll_trim), cmd.e, cmd.r];
        for i in 0..3 { self.sv[i] += (tgt[i] - self.sv[i]) * min(1.0, dt / var.servo_tau); }
        if var.battery { self.batt.step(self.thr, dt); }

        let wv = wind.at(self.t, self.p[2]);
        let va = qinv(self.q, sub(self.v, wv));
        let vax = va[0];
        let vf = max(0.0, vax);
        let tmax = AC.tmax * var.thrust * if var.battery { self.batt.thrust_factor() } else { 1.0 };
        let thrust = tmax * self.thr * clamp(1.0 - vf / AC.vp, 0.0, 1.0);
        let vi = sqrt(vf * vf + 2.0 * thrust / (RHO * AC.disc)) - vf;
        let cg = var.cg;
        let mut fb: V3 = [thrust, 0.0, 0.0];
        let prop_arm = sub([0.37, 0.0, 0.0], cg);
        let mut m: V3 = add([-AC.kq * var.torque * thrust, 0.0, 0.0], cross(prop_arm, [thrust, 0.0, 0.0]));
        let d = AC.dmax;
        let defl = |c: Ctl| match c {
            Ctl::AilL => self.sv[0] * d, Ctl::AilR => -self.sv[0] * d,
            Ctl::Elev => -self.sv[1] * d, Ctl::Rud => self.sv[2] * d, Ctl::None => 0.0,
        };
        let mut wing_alpha = 0.0;
        for (i, s) in SURF.iter().enumerate() {
            let r = sub(s.r, cg);
            let mut u = add(va, cross(self.w, r));
            u[0] += s.wash * vi;
            let un = dot(u, s.n);
            let u0 = u[0];
            let v2 = u0 * u0 + un * un;
            if v2 < 1e-4 { continue; }
            let mut alpha = atan2(-un, u0);
            if s.ctl == Ctl::AilL { wing_alpha = alpha; }
            if s.ctl != Ctl::None { alpha += s.tau * defl(s.ctl) * if u0 >= 0.0 { 1.0 } else { -1.0 }; }
            alpha = atan2(sin(alpha), cos(alpha));
            let (cl, cd) = lift_coefs(alpha, s);
            let up = add([u0, 0.0, 0.0], mul(s.n, un));
            let vm = sqrt(v2);
            let span = cross([1.0, 0.0, 0.0], s.n);
            let ld = norm(cross(span, up));
            let dd = mul(up, -1.0 / vm);
            let qs = 0.5 * RHO * v2 * s.s * var.surf[i];
            let f = add(mul(ld, qs * cl), mul(dd, qs * cd));
            fb = add(fb, f);
            m = add(m, cross(r, f));
        }
        let vam = len(va);
        for i in 0..3 { fb[i] -= 0.5 * RHO * 0.006 * vam * va[i]; }

        let mut fw = qrot(self.q, fb);
        let mut f_contact_w: V3 = [0.0; 3];
        let mut nc = 0usize;
        for (i, pt) in PTS.iter().enumerate() {
            let r = sub(pt.r, cg);
            let rw = qrot(self.q, r);
            let pz = self.p[2] + rw[2];
            let in_c = pz < 0.0;
            if in_c {
                let vp = add(self.v, qrot(self.q, cross(self.w, r)));
                if !self.contact[i] && nc < contacts.len() {
                    contacts[nc] = Some(Contact { k: pt.k, vz: vp[2], vh: hypot(vp[0], vp[1]), sp: len(vp) });
                    nc += 1;
                }
                let n = max(0.0, -700.0 * pz - 14.0 * vp[2]);
                let vh = hypot(vp[0], vp[1]);
                let fr = pt.mu * n / (vh + 0.05);
                let fp = [-vp[0] * fr, -vp[1] * fr, n];
                f_contact_w = add(f_contact_w, fp);
                m = add(m, cross(r, qinv(self.q, fp)));
            }
            self.contact[i] = in_c;
        }
        fw = add(fw, f_contact_w);
        // what an accelerometer feels: everything but gravity
        self.f_body = mul(qinv(self.q, fw), 1.0 / AC.m);
        fw[2] -= AC.m * G;

        self.v = add(self.v, mul(fw, dt / AC.m));
        self.p = add(self.p, mul(self.v, dt));

        // rotation with prop gyroscopics: I w' = M - w x (I w + H)
        let iw = [AC.inertia[0] * self.w[0], AC.inertia[1] * self.w[1], AC.inertia[2] * self.w[2]];
        let h = [AC.ip * AC.wmax * sqrt(self.thr), 0.0, 0.0];
        let gyro = cross(self.w, add(iw, h));
        for i in 0..3 { self.w[i] += ((m[i] - gyro[i]) / AC.inertia[i] - 0.05 * self.w[i]) * dt; }
        self.q = qintegrate(self.q, self.w, dt);

        self.aux = Aux { vax, vi, thrust, alpha: wing_alpha, air: vam };
        self.t += dt;
    }

    pub fn on_feet(&self) -> bool { self.contact[0] || self.contact[1] || self.contact[2] || self.contact[3] || self.contact[12] }
    pub fn any_contact(&self) -> bool { self.contact.iter().any(|c| *c) }
    /// Height of the lowest airframe point above the ground.
    pub fn lowest(&self) -> f64 {
        let mut lo = f64::INFINITY;
        for pt in PTS.iter() { lo = min(lo, self.p[2] + qrot(self.q, pt.r)[2]); }
        lo
    }
}
