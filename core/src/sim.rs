//! The whole simulation: true world + (optionally) sensors and estimator + flight software + autopilot,
//! with an event queue the UI turns into messages.

use crate::estimator::{nav_from_truth, Estimator, Nav};
use crate::ekf::Ekf;
use crate::flight::*;
use crate::math::*;
use crate::model::*;
use crate::sensors::Sensors;
use crate::aerobatics::{Fig, Show, Step};

pub const DT: f64 = 0.002;
pub const WINDS: [f64; 3] = [0.0, 2.0, 4.0];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Ev {
    TakeoffDone = 1, YouHaveControl = 2, TailLanding = 3, HardTail = 4, Belly = 5, Touched = 6, Crash = 7,
    Recovery = 8, PulledOut = 9, Recovered = 10, StickLy = 11, ModeChanged = 12, BatteryLow = 13, BatteryEmpty = 14,
    FigureStarted = 15, FigureDone = 16, ShowDone = 17,
}
#[derive(Clone, Copy)]
pub struct Event { pub code: Ev, pub a: f64, pub b: f64, pub c: f64, pub d: f64 }
pub const MAX_EV: usize = 16;

pub struct Sim {
    pub truth: Truth,
    pub wind: Wind,
    pub var: Variation,
    pub real: bool,
    sensors: Sensors,
    est: Estimator,
    pub ekf: Ekf,
    /// Real-plane mode: tail camera + AprilTag precision landing on/off.
    pub cam_on: bool,
    /// Real-plane mode estimator: the EKF (default) or the simpler complementary filter, for comparison.
    pub use_ekf: bool,
    pub fc: Fc,
    pub mode: Mode,
    pub auto: Option<Auto>,
    pub recover_on: bool,
    pub vane_on: bool,
    pub wind_lvl: usize,
    pub crashed: bool,
    pub airborne: bool,
    pub impact: (f64, f64),
    pub sticks: Sticks,
    pub events: [Option<Event>; MAX_EV],
    af_truth: V3,
    batt_warned: u8,
    seed: u64,
    rng: Rng,
}

impl Sim {
    pub fn new(seed: u64) -> Sim {
        let truth = Truth::standing();
        let wind = Wind { speed: 0.0, phase: [0.0; 4] };
        let nav = nav_from_truth(&truth, &wind, [0.0; 3]);
        let mut s = Sim {
            truth, wind, var: Variation::IDEAL, real: false,
            sensors: Sensors::new(seed), est: Estimator::new(&truth), ekf: Ekf::new(&truth), use_ekf: true, cam_on: true, fc: Fc::new(nav),
            mode: Mode::QLoiter, auto: None, recover_on: true, vane_on: true, wind_lvl: 0,
            crashed: false, airborne: false, impact: (f64::INFINITY, 0.0),
            sticks: Sticks { ly: 1.0, ..Sticks::default() }, events: [None; MAX_EV],
            af_truth: [0.0; 3], batt_warned: 0, seed, rng: Rng::new(seed),
        };
        s.reset();
        s
    }

    /// Back on the pad. In real-plane mode each reset is a new airframe with its own small build errors.
    pub fn reset(&mut self) {
        self.seed = self.seed.wrapping_add(1);
        self.rng = Rng::new(self.seed);
        self.truth = Truth::standing();
        let ph = [self.rng.uni() * 6.28, self.rng.uni() * 6.28, self.rng.uni() * 6.28, self.rng.uni() * 6.28];
        self.wind = Wind { speed: WINDS[self.wind_lvl], phase: if self.real { ph } else { [0.0; 4] } };
        self.var = if self.real { Variation::random(&mut self.rng) } else { Variation::IDEAL };
        self.sensors = Sensors::new(self.seed);
        self.sensors.cam_on = self.cam_on;
        self.est = Estimator::new(&self.truth);
        self.ekf = Ekf::new(&self.truth);
        self.fc = Fc::new(nav_from_truth(&self.truth, &self.wind, [0.0; 3]));
        self.auto = None;
        self.crashed = false;
        self.airborne = false;
        self.impact = (f64::INFINITY, 0.0);
        self.af_truth = [0.0; 3];
        self.batt_warned = 0;
        self.sticks.ly = 1.0; self.sticks.lx = 0.0; self.sticks.rx = 0.0; self.sticks.ry = 0.0;
        self.emit(Ev::StickLy, 1.0, 0.0, 0.0, 0.0);
    }

    pub fn set_real(&mut self, on: bool) { self.real = on; self.reset(); }
    /// Tail camera on/off; takes effect immediately (a camera that fails or is switched off in flight).
    pub fn set_cam(&mut self, on: bool) { self.cam_on = on; self.sensors.cam_on = on; }
    pub fn set_wind(&mut self, lvl: usize) { self.wind_lvl = lvl.min(2); self.wind.speed = WINDS[self.wind_lvl]; }

    pub fn emit(&mut self, code: Ev, a: f64, b: f64, c: f64, d: f64) {
        for e in self.events.iter_mut() { if e.is_none() { *e = Some(Event { code, a, b, c, d }); return; } }
    }
    pub fn take_events(&mut self) -> [Option<Event>; MAX_EV] { core::mem::replace(&mut self.events, [None; MAX_EV]) }

    fn set_ly(&mut self, v: f64) { if !self.sticks.l_active { self.sticks.ly = v; self.emit(Ev::StickLy, v, 0.0, 0.0, 0.0); } }

    /// Switch pilot mode; re-centre the left stick when its meaning changes between raw throttle and climb rate.
    pub fn set_mode(&mut self, m: Mode) {
        let was = self.mode;
        self.mode = m; self.auto = None; self.fc.loiter = None; self.fc.reset_i();
        if m.climb_stick() != was.climb_stick() {
            let v = if m.climb_stick() { if self.fc.nav.on_ground && !self.airborne { 1.0 } else { 0.0 } } else { 1.0 - 2.0 * self.truth.thr };
            self.set_ly(v);
        }
        self.emit(Ev::ModeChanged, m as u8 as f64, 0.0, 0.0, 0.0);
    }

    fn ground_now(&self) -> bool { self.fc.nav.on_ground }

    pub fn start_auto(&mut self, task: Task) {
        if self.crashed { return; }
        let (x, _, _) = axes(self.fc.nav.q);
        self.fc.reset_i();
        let z = self.fc.nav.p[2];
        let mk = |task, phase| Auto { task, phase, t: 0.0, hd: [0.0, 1.0], ly0: 0.0, back: Mode::QLoiter, wing: false, c: 0.0, show: Show::full(), then_show: None };
        let a = match task {
            Task::Takeoff => {
                if !self.ground_now() && z > 1.5 { return self.start_auto(Task::Circuit); }
                mk(Task::Takeoff, Phase::Lift)
            }
            Task::Circuit => {
                if self.ground_now() || z < 1.5 { mk(Task::Takeoff, Phase::Lift) }
                else if x[2] > 0.6 { let mut a = mk(Task::Circuit, Phase::PitchOver); a.hd = self.fc.belly_dir(); a }
                else { mk(Task::Circuit, Phase::Fly) }
            }
            Task::Land => {
                if self.ground_now() && !self.airborne { return; }
                let far = hypot(self.fc.nav.p[0], self.fc.nav.p[1]) > 12.0 && x[2] < 0.6;
                mk(Task::Land, if far { Phase::Approach } else if x[2] < 0.6 { Phase::Pullup } else { Phase::Brake })
            }
            Task::Recover | Task::Show => return,
        };
        let mut a = a;
        a.ly0 = self.sticks.ly;
        self.auto = Some(a);
    }
    /// Fly the whole aerobatic show, or one figure. From the ground it takes off first.
    pub fn start_show(&mut self, fig: Option<Fig>) {
        if self.crashed { return; }
        let mut show = match fig { Some(f) => Show::single(f), None => Show::full() };
        show.seed_wind(self.fc.nav.wind_est);
        self.fc.reset_i();
        if self.ground_now() || self.fc.nav.p[2] < 1.5 {
            self.start_auto(Task::Takeoff);
            if let Some(a) = self.auto.as_mut() { a.then_show = Some(show); }
            return;
        }
        self.auto = Some(Auto { task: Task::Show, phase: Phase::Show, t: 0.0, hd: [0.0, 1.0], ly0: self.sticks.ly,
            back: self.mode, wing: false, c: 0.0, show, then_show: None });
    }
    pub fn toggle_circuit(&mut self) {
        if matches!(self.auto, Some(a) if a.task == Task::Circuit) { self.auto = None; self.fc.reset_i(); }
        else { self.start_auto(Task::Circuit); }
    }

    fn sticks_moved(&self, ly0: f64) -> bool {
        let s = &self.sticks;
        abs(s.rx) > 0.3 || abs(s.ry) > 0.3 || abs(s.lx) > 0.3 || abs(s.ly - ly0) > 0.25
    }

    // ---- autopilot
    fn autopilot(&mut self, dt: f64) {
        let mut a = self.auto.unwrap();
        let n = self.fc.nav;
        let (x, y, z) = axes(n.q);
        let tail = n.agl_tail;
        let to_pad = [-n.p[0], -n.p[1]];
        let d_pad = len2(to_pad);
        let vh = hypot(n.v[0], n.v[1]);
        a.t += dt;
        match a.phase {
            Phase::Lift => {
                let spin = self.fc.edge_on_spin([0.0, 0.0]);
                self.fc.hover_ctl(clamp_len2(mul2(to_pad, 0.4), 1.5), if n.p[2] < 8.0 { 2.0 } else { 0.0 }, spin, dt, 0.55);
                if n.p[2] > 8.0 { a.phase = Phase::PitchOver; a.t = 0.0; a.hd = self.fc.belly_dir(); self.fc.reset_i(); }
            }
            Phase::PitchOver => {
                let th = max(0.22, PI / 2.0 - a.t * 0.95);
                self.fc.dir_ctl([a.hd[0] * cos(th), a.hd[1] * cos(th), sin(th)], 0.0, dt);
                self.fc.act.thr = 0.95;
                if th <= 0.22 && x[2] < 0.5 {
                    a.task = Task::Circuit; a.phase = Phase::Fly; a.t = 0.0; self.fc.reset_i();
                    if let Some(sh) = a.then_show.take() { a.task = Task::Show; a.phase = Phase::Show; a.show = sh; }
                    else { self.emit(Ev::TakeoffDone, 0.0, 0.0, 0.0, 0.0); }
                }
            }
            Phase::Show => {
                let mut sh = a.show;
                let r = sh.step(&mut self.fc, dt);
                a.show = sh;
                match r {
                    Step::Running => {}
                    Step::FigureStarted(f) => self.emit(Ev::FigureStarted, f as u8 as f64, (sh.fig + 1) as f64, sh.n as f64, 0.0),
                    Step::FigureDone(f) => {
                        self.emit(Ev::FigureDone, f as u8 as f64, sh.path_rms(), sh.errmax, 0.0);
                        if sh.single { a.task = Task::Circuit; a.phase = Phase::Fly; a.t = 0.0; self.fc.reset_i(); }
                    }
                    Step::ShowDone => {
                        self.emit(Ev::ShowDone, 8.0, sh.path_rms(), sh.errmax, n.p[2]);
                        a.task = Task::Circuit; a.phase = Phase::Fly; a.t = 0.0; self.fc.reset_i();
                    }
                }
            }
            Phase::Fly => {
                let c = self.fc.circuit_target();
                let phi = self.fc.bank_toward(c[0], c[1], 22.0);
                self.fc.fwd_ctl(phi, CIRC_ALT, CIRC_V, dt);
            }
            Phase::Approach => {
                // straight in at low height; pull up when the remaining distance matches the braking distance
                let phi = self.fc.bank_toward(0.0, 0.0, 16.0);
                self.fc.fwd_ctl(phi, LAND.alt, LAND.v, dt);
                let v_close = max(0.0, (n.v[0] * to_pad[0] + n.v[1] * to_pad[1]) / max(d_pad, 1e-3));
                // ... or whenever it passes close by, so it can never end up circling the pad (hover braking does the rest)
                if d_pad < LAND.k_trig * v_close * v_close + LAND.b_trig || d_pad < 8.0 { a.phase = Phase::Pullup; a.t = 0.0; self.fc.reset_i(); }
            }
            Phase::Pullup => {
                // wings level, then pull the nose straight up: the blown wing broadside is the air brake
                let phi = atan2(y[2], z[2]);
                if x[2] < 0.5 && abs(phi) > 0.35 { self.fc.fwd_ctl(0.0, n.p[2], LAND.v, dt); }
                else { self.fc.dir_ctl([0.0, 0.0, 1.0], 0.0, dt); self.fc.act.thr = if x[2] > 0.6 { 0.75 } else { 0.35 }; }
                if x[2] > 0.85 || a.t > 3.0 { a.phase = Phase::Brake; a.t = 0.0; self.fc.reset_i(); }
            }
            Phase::Brake => {
                let vd = brake_to(to_pad, d_pad, LAND.a_brake, LAND.v_max);
                let spin = if x[2] > 0.8 { self.fc.edge_on_spin(vd) } else { 0.0 };
                self.fc.hover_ctl(vd, clamp(0.8 * (LAND.h_hold - n.p[2]), -2.0, 1.5), spin, dt, if x[2] > 0.8 { 0.6 } else { 0.45 });
                if x[2] > 0.85 && d_pad < 1.2 && vh < 0.9 { a.phase = Phase::Descend; a.t = 0.0; }
            }
            Phase::Descend => {
                // fast while high, 0.45 m/s for the last metre; below 2.5 m wait for the drift to settle
                let k = if tail < 2.5 { 0.6 } else { 1.0 };
                let vd = mul2(brake_to(to_pad, d_pad, LAND.a_brake, 1.2), k);
                let mut vz = -min(2.4, 0.45 + 0.7 * max(0.0, tail - 0.9));
                if tail < 2.5 { vz *= clamp(1.6 - 2.0 * vh, 0.0, 1.0); }
                let spin = self.fc.edge_on_spin(vd);
                self.fc.hover_ctl(vd, vz, spin, dt, 0.55);
                if d_pad > 3.0 { a.phase = Phase::Brake; a.t = 0.0; }
            }
            Phase::Pull => {
                // fast and nose-low: roll wings level and pull to 35° nose-up at full power
                let phi = atan2(y[2], z[2]);
                let h = horiz(n.v);
                let psi = if h[2] > 1.0 { atan2(h[1], h[0]) } else { self.fc.flight_heading() };
                self.fc.att_ctl(quat_euler(psi, if abs(phi) > 1.2 { 0.0 } else { 0.6 }, 0.0), [0.0; 3], dt, 6.0);
                self.fc.act.thr = 1.0;
                if n.v[2] > 0.5 && x[2] > 0.3 { self.to_hold(&mut a); }
                else if a.t > 4.0 { a.phase = Phase::HoverUp; a.t = 0.0; }
            }
            Phase::HoverUp => {
                // slow, tumbling or inverted: point the nose straight up and climb on thrust
                if x[2] < 0.7 { self.fc.dir_ctl([0.0, 0.0, 1.0], 0.0, dt); self.fc.act.thr = if x[2] > 0.2 { 1.0 } else { 0.5 }; }
                else { self.fc.hover_ctl([0.0, 0.0], 3.0, 0.0, dt, 0.4); }
                if x[2] > 0.85 && n.v[2] > 0.3 && n.p[2] > 3.0 { self.to_hold(&mut a); }
            }
            Phase::Hold => {
                // safe again: fly level (or hover) until the pilot centres the sticks, then hand back
                if a.wing { self.fc.fbwa(0.0, 0.0, 0.0, dt); self.fc.act.thr = 0.5; }
                else { self.fc.hover_ctl([0.0, 0.0], 0.0, 0.0, dt, 0.55); }
                let s = self.sticks;
                let centred = abs(s.rx) < 0.15 && abs(s.ry) < 0.15 && abs(s.lx) < 0.2;
                a.c = if centred { a.c + dt } else { 0.0 };
                if let Some(sh) = a.then_show {
                    // the recovery interrupted a show: carry on with the next figure
                    if a.wing && n.p[2] > 8.0 {
                        self.auto = Some(Auto { task: Task::Show, phase: Phase::Show, t: 0.0, show: sh, then_show: None, ..a });
                        self.emit(Ev::Recovered, self.mode as u8 as f64, n.p[2], 1.0, 0.0);
                        return;
                    }
                }
                if a.c > 0.4 {
                    let m = if x[2] > 0.7 { if a.back.is_q() { if a.back == Mode::QStab { Mode::QHover } else { a.back } } else { Mode::QLoiter } } else { Mode::Fbwa };
                    self.auto = None;
                    self.set_mode(m);
                    self.emit(Ev::Recovered, m as u8 as f64, n.p[2], 0.0, 0.0);
                    return;
                }
            }
        }
        if self.auto.is_some() { self.auto = Some(a); }
    }
    fn to_hold(&mut self, a: &mut Auto) {
        a.phase = Phase::Hold; a.t = 0.0; a.c = 0.0; a.wing = axes(self.fc.nav.q).0[2] < 0.7;
        self.fc.reset_i();
        self.emit(Ev::PulledOut, 0.0, 0.0, 0.0, 0.0);
    }

    fn check_recovery(&mut self) {
        let (t, v) = self.impact;
        if !self.airborne || self.crashed || t == f64::INFINITY || self.fc.deliberate_landing() || v < 1.8 { return; }
        // a show figure on its planned path will pull out as planned; only step in if impact is very close
        if let Some(a) = self.auto { if a.task == Task::Show && a.show.on_track && t > 0.8 { return; } }
        if t < self.fc.time_to_recover() {
            let x = axes(self.fc.nav.q).0;
            let fast = x[2] < 0.6 && self.fc.nav.air > 8.0;
            // remember an interrupted show so it can carry on with the next figure once safe
            let resume = match self.auto { Some(a) if a.task == Task::Show => { let mut sh = a.show; if sh.skip_figure() { Some(sh) } else { None } } _ => None };
            self.auto = Some(Auto {
                task: Task::Recover, phase: if fast { Phase::Pull } else { Phase::HoverUp }, t: 0.0,
                hd: [0.0, 1.0], ly0: self.sticks.ly, back: self.mode, wing: false, c: 0.0,
                show: Show::full(), then_show: resume,
            });
            self.fc.reset_i();
            self.emit(Ev::Recovery, t, v, 0.0, 0.0);
        }
    }

    fn control(&mut self, dt: f64) {
        if self.crashed { self.fc.act = Act::default(); return; }
        if let Some(a) = self.auto {
            if a.task != Task::Recover && self.sticks_moved(a.ly0) {
                let x = axes(self.fc.nav.q).0;
                let m = if x[2] > 0.7 { if self.mode.is_q() { self.mode } else { Mode::QLoiter } } else if self.mode.is_q() { Mode::Fbwa } else { self.mode };
                self.set_mode(m);
                self.emit(Ev::YouHaveControl, m as u8 as f64, 0.0, 0.0, 0.0);
            }
        }
        // recovery also watches over the aerobatic show
        if self.recover_on && self.auto.map_or(true, |a| a.task == Task::Show) { self.check_recovery(); }
        if self.auto.is_some() { self.autopilot(dt); return; }
        let s = self.sticks;
        self.fc.pilot(self.mode, &s, self.vane_on, dt);
    }

    fn touchdown(&mut self, c: Contact) {
        if self.crashed { return; }
        let x = axes(self.truth.q).0;
        if self.airborne {
            self.airborne = false;
            let tilt = acos(clamp(x[2], -1.0, 1.0)) / D2R;
            let dist = hypot(self.truth.p[0], self.truth.p[1]);
            if c.k == Kind::Foot && x[2] > 0.8 && c.vz > -1.4 && c.vh < 1.3 {
                let score = round(clamp(100.0 - 35.0 * abs(c.vz) - 25.0 * c.vh - 6.0 * min(dist, 8.0) - 0.6 * tilt, 0.0, 100.0));
                self.landed();
                self.emit(Ev::TailLanding, score, abs(c.vz), c.vh, dist);
            } else if c.k == Kind::Foot && c.sp < 4.5 && c.vz > -3.0 {
                self.landed();
                self.emit(Ev::HardTail, abs(c.vz), tilt, 0.0, 0.0);
            } else if (c.k == Kind::Belly || c.k == Kind::Tip) && c.sp < 3.0 {
                self.landed();
                self.emit(Ev::Belly, c.sp, 0.0, 0.0, 0.0);
            } else if c.sp > 2.5 {
                self.crash(c);
            } else {
                self.landed();
                self.emit(Ev::Touched, c.k as u8 as f64, c.sp, 0.0, 0.0);
            }
        } else if (c.k != Kind::Foot && c.sp > 3.5) || c.sp > 5.0 {
            self.crash(c);
        }
    }
    fn landed(&mut self) {
        if self.auto.is_some() { self.auto = None; self.fc.act.thr = 0.0; }
        self.truth.thr = min(self.truth.thr, 0.3);
        self.set_ly(1.0);
        self.fc.loiter = None;
        self.fc.reset_i();
    }
    fn crash(&mut self, c: Contact) {
        self.crashed = true; self.auto = None;
        self.emit(Ev::Crash, c.k as u8 as f64, c.sp, 0.0, 0.0);
    }

    /// The flight controller's view this step: exact in ideal mode, estimated from sensors in real-plane mode.
    fn refresh_nav(&mut self, dt: f64) {
        if self.real {
            let m = self.sensors.sample(&self.truth, &self.wind, dt);
            self.fc.nav = if self.use_ekf { self.ekf.update_all(&m, self.fc.act.thr, dt, None) } else { self.est.update(&m, self.fc.act.thr, dt, None) };
        } else {
            self.fc.nav = nav_from_truth(&self.truth, &self.wind, self.af_truth);
        }
    }

    /// One 2 ms physics + control step.
    pub fn step(&mut self) {
        if self.crashed { return; }
        self.impact = self.fc.predict_impact();
        self.control(DT);
        let v0 = self.truth.v;
        let mut contacts: [Option<Contact>; 4] = [None; 4];
        let act = self.fc.act;
        self.truth.step(&act, &self.wind, &self.var, DT, &mut contacts);
        let k = min(1.0, DT / 0.25);
        self.af_truth = add(mul(self.af_truth, 1.0 - k), mul(sub(self.truth.v, v0), k / DT));
        for c in contacts.iter().flatten() { self.touchdown(*c); if self.crashed { break; } }
        if !self.truth.any_contact() && self.truth.p[2] > 1.3 { self.airborne = true; }
        self.refresh_nav(DT);
        if self.var.battery {
            let v = self.truth.batt.volts;
            if self.batt_warned == 0 && v < 10.6 && self.truth.thr > 0.2 { self.batt_warned = 1; self.emit(Ev::BatteryLow, v, self.truth.batt.soc, 0.0, 0.0); }
            if self.batt_warned == 1 && self.truth.batt.soc < 0.05 { self.batt_warned = 2; self.emit(Ev::BatteryEmpty, v, 0.0, 0.0, 0.0); }
        }
    }

    /// Advance by a frame time (seconds), in fixed 2 ms steps.
    pub fn advance(&mut self, ft: f64) {
        let n = max(1.0, round(min(ft, 0.05) / DT)) as usize;
        for _ in 0..n { self.step(); if self.crashed { break; } }
    }

    pub fn nav(&self) -> &Nav { &self.fc.nav }
}
