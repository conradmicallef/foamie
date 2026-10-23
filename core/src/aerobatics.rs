//! Aerobatic figures flown by the autopilot.
//!
//! Each figure is a short program of segments. Rotating segments (pull, roll, yaw) turn a *reference attitude*
//! at a fixed body rate and the quaternion attitude controller follows it with feed-forward, so a loop is a
//! true constant-rate loop at any attitude, including inverted and vertical. Between figures the aircraft flies
//! short level lines on the normal wing-borne guidance.

use crate::flight::Fc;
use crate::math::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Fig { Loop = 1, Roll = 2, PointRoll = 3, Immelmann = 4, SplitS = 5, Hammerhead = 6, CubanEight = 7, TorqueRoll = 8 }
impl Fig {
    pub fn from(c: u32) -> Option<Fig> {
        Some(match c {
            1 => Fig::Loop, 2 => Fig::Roll, 3 => Fig::PointRoll, 4 => Fig::Immelmann,
            5 => Fig::SplitS, 6 => Fig::Hammerhead, 7 => Fig::CubanEight, 8 => Fig::TorqueRoll, _ => return None,
        })
    }
    /// Height needed at entry (m above ground) for the lowest point of the figure to stay safe.
    pub fn min_alt(self) -> f64 {
        match self { Fig::SplitS => 32.0, Fig::CubanEight => 24.0, Fig::Hammerhead => 24.0, Fig::Loop => 20.0, _ => 15.0 }
    }
}

/// The full show, alternating direction so it stays in front of the pilot:
/// loop east, Immelmann to turn west and climb, roll, split-S back east and down, four-point roll,
/// hammerhead west, Cuban eight, then a prop-hang torque roll as the tail-sitter finale.
pub const SHOW: [Fig; 8] = [Fig::Loop, Fig::Immelmann, Fig::Roll, Fig::SplitS, Fig::PointRoll, Fig::Hammerhead, Fig::CubanEight, Fig::TorqueRoll];

#[derive(Clone, Copy, Debug)]
pub enum Seg {
    /// Pitch through `deg` (negative = push) at `rate` rad/s.
    Pull(f64, f64, f64),
    /// Roll right through `deg` at `rate`.
    Roll(f64, f64, f64),
    /// Yaw right through `deg` at `rate` (stall-turn pivot).
    Yaw(f64, f64, f64),
    /// Hold the reference attitude for `t` s.
    Hold(f64, f64),
    /// Hold the attitude on a vertical up-line until airspeed falls below `v` (or `t` s pass).
    UpLine(f64, f64, f64),
    /// Prop hang: hover in place spinning about the nose at `spin` rad/s for `t` s.
    PropHang(f64, f64),
    /// Pitch over from the hover into wing-borne flight along the belly.
    PitchOut,
    /// Straight and level for `t` s at the current height.
    Level(f64),
}
use Seg::*;

/// Rates: pull 1.6 rad/s (loop diameter ≈ 18 m at 14 m/s), roll 4 rad/s (230°/s).
pub fn program(f: Fig) -> &'static [Seg] {
    match f {
        Fig::Loop => &[Pull(180.0, 1.6, 1.0), Pull(180.0, 2.1, 0.6), Level(1.5)],
        Fig::Roll => &[Pull(8.0, 1.0, 0.8), Roll(360.0, 4.0, 0.8), Pull(-8.0, 1.0, 0.7), Level(1.5)],
        Fig::PointRoll => &[
            Pull(8.0, 1.0, 0.85),
            Roll(90.0, 3.5, 0.85), Hold(0.35, 0.85), Roll(90.0, 3.5, 0.85), Hold(0.35, 0.85),
            Roll(90.0, 3.5, 0.85), Hold(0.35, 0.85), Roll(90.0, 3.5, 0.85),
            Pull(-8.0, 1.0, 0.7), Level(1.5),
        ],
        Fig::Immelmann => &[Pull(180.0, 1.6, 1.0), Roll(180.0, 3.5, 0.9), Level(1.5)],
        Fig::SplitS => &[Roll(180.0, 3.5, 0.6), Pull(180.0, 1.6, 0.3), Level(1.5)],
        Fig::Hammerhead => &[
            Pull(90.0, 1.8, 0.9), UpLine(5.0, 3.0, 0.12), Yaw(180.0, 2.5, 0.9),
            Hold(0.5, 0.2), Pull(90.0, 2.2, 0.6), Level(1.5),
        ],
        // 5/8 loop, half roll on the 45° down-line, 3/4 loop, half roll, pull level
        Fig::CubanEight => &[
            Pull(225.0, 1.6, 1.0), Roll(180.0, 3.5, 0.5), Pull(270.0, 1.6, 0.9),
            Roll(180.0, 3.5, 0.45), Pull(45.0, 1.6, 0.8), Level(1.5),
        ],
        Fig::TorqueRoll => &[Pull(90.0, 1.8, 0.9), UpLine(3.5, 3.0, 0.12), PropHang(4.0, 4.0), PitchOut, Level(2.0)],
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Step { Running, FigureStarted(Fig), FigureDone(Fig), ShowDone }

/// Progress through a show (or a single figure).
#[derive(Clone, Copy)]
pub struct Show {
    pub seq: [Fig; 8],
    pub n: usize,
    pub fig: usize,
    pub seg: usize,
    /// Angle turned (rad) or time spent (s) in the current segment.
    pub prog: f64,
    pub qref: Q,
    pub h_line: f64,
    pub hd: [f64; 2],
    pub seg_started: bool,
    /// Getting into position: pitched out of a hover, at speed and height.
    pub prepared: bool,
    pub announced: bool,
    pub single: bool,
}

impl Show {
    pub fn full() -> Show { Show::new(SHOW, SHOW.len(), false) }
    pub fn single(f: Fig) -> Show { Show::new([f; 8], 1, true) }
    fn new(seq: [Fig; 8], n: usize, single: bool) -> Show {
        Show { seq, n, fig: 0, seg: 0, prog: 0.0, qref: [1.0, 0.0, 0.0, 0.0], h_line: 0.0, hd: [1.0, 0.0],
               seg_started: false, prepared: false, announced: false, single }
    }
    pub fn current(&self) -> Fig { self.seq[self.fig.min(self.n - 1)] }

    /// One control step.
    pub fn step(&mut self, fc: &mut Fc, dt: f64) -> Step {
        let n = fc.nav;
        let (x, _, _) = axes(n.q);
        let fig = self.current();
        // ---- get into position: out of the hover, up to the figure's height, at speed, wings level
        if !self.prepared {
            if x[2] > 0.6 && n.air < 6.0 {
                if !self.seg_started { self.hd = fc.belly_dir(); self.prog = 0.0; self.seg_started = true; }
                self.prog += dt;
                let th = max(0.22, PI / 2.0 - self.prog * 0.95);
                fc.dir_ctl([self.hd[0] * cos(th), self.hd[1] * cos(th), sin(th)], 0.0, dt);
                fc.act.thr = 0.95;
                return Step::Running;
            }
            let h = fig.min_alt() + 4.0;
            fc.fwd_ctl(0.0, h, 15.0, dt);
            let (_, y, z) = axes(n.q);
            let level = abs(atan2(y[2], z[2])) < 0.15 && abs(x[2]) < 0.2;
            if n.p[2] > h - 2.0 && n.air > 12.0 && level {
                self.prepared = true; self.seg_started = false; self.seg = 0; self.prog = 0.0;
            } else { return Step::Running; }
        }
        let prog = program(fig);
        if !self.announced { self.announced = true; return Step::FigureStarted(fig); }
        let seg = prog[self.seg];
        if !self.seg_started {
            self.seg_started = true;
            self.prog = 0.0;
            self.h_line = n.p[2];
            if self.seg == 0 { self.qref = clean_attitude(fc); }
            if let PitchOut = seg { self.hd = fc.belly_dir(); }
        }
        let done = match seg {
            Pull(deg, rate, thr) => self.rotate(fc, [0.0, -signum(deg), 0.0], abs(deg) * D2R, rate, thr, dt),
            Roll(deg, rate, thr) => self.rotate(fc, [1.0, 0.0, 0.0], deg * D2R, rate, thr, dt),
            Yaw(deg, rate, thr) => self.rotate(fc, [0.0, 0.0, -1.0], deg * D2R, rate, thr, dt),
            Hold(t, thr) => { fc.att_ctl(self.qref, [0.0; 3], dt, 6.0); fc.act.thr = thr; self.prog += dt; self.prog >= t }
            UpLine(v, t, thr) => { fc.att_ctl(self.qref, [0.0; 3], dt, 6.0); fc.act.thr = thr; self.prog += dt; n.air < v || self.prog >= t }
            PropHang(t, spin) => { fc.hover_ctl([0.0, 0.0], 0.0, spin, dt, 0.4); self.prog += dt; self.prog >= t }
            PitchOut => {
                self.prog += dt;
                let th = max(0.22, PI / 2.0 - self.prog * 0.95);
                fc.dir_ctl([self.hd[0] * cos(th), self.hd[1] * cos(th), sin(th)], 0.0, dt);
                fc.act.thr = 0.95;
                th <= 0.22 && x[2] < 0.5
            }
            Level(t) => { fc.fwd_ctl(0.0, self.h_line, 14.0, dt); self.prog += dt; self.prog >= t }
        };
        if !done { return Step::Running; }
        // ---- next segment / figure
        self.seg += 1;
        self.seg_started = false;
        if let Level(_) = seg { self.qref = clean_attitude(fc); }
        if self.seg < prog.len() {
            // carry the reference across rotation boundaries; re-seed it after anything that is not a pure rotation
            if matches!(seg, PropHang(..) | PitchOut) { self.qref = clean_attitude(fc); }
            return Step::Running;
        }
        self.seg = 0;
        self.fig += 1;
        self.announced = false;
        if self.fig >= self.n { return if self.single { Step::FigureDone(fig) } else { Step::ShowDone }; }
        // the next figure may need more height (the split-S after the roll): climb first
        let next = self.current();
        if n.p[2] < next.min_alt() { self.prepared = false; }
        Step::FigureDone(fig)
    }

    /// Turn the reference attitude about a body axis at `rate` until `total` rad, flying it with feed-forward.
    fn rotate(&mut self, fc: &mut Fc, axis_b: V3, total: f64, rate: f64, thr: f64, dt: f64) -> bool {
        let sgn = if total < 0.0 { -1.0 } else { 1.0 };
        let step = min(rate * dt, abs(total) - self.prog);
        let a = sgn * step;
        let h = 0.5 * a;
        self.qref = qnorm(qmul(self.qref, [cos(h), axis_b[0] * sin(h), axis_b[1] * sin(h), axis_b[2] * sin(h)]));
        self.prog += step;
        // feed-forward in the controller's terms: roll right, pitch up, yaw right
        let w = mul(axis_b, sgn * rate);
        fc.att_ctl(self.qref, [w[0], -w[1], -w[2]], dt, 6.0);
        fc.act.thr = thr;
        self.prog >= abs(total) - 1e-9
    }
}

/// Wings level at the current heading and pitch: a clean starting attitude for a figure.
fn clean_attitude(fc: &Fc) -> Q {
    let (x, _, _) = axes(fc.nav.q);
    quat_euler(fc.flight_heading(), asin(clamp(x[2], -1.0, 1.0)), 0.0)
}
