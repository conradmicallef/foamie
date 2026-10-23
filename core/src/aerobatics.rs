//! Aerobatic figures, flown on a path.
//!
//! Every figure is real geometry in a vertical plane along the show line: straight lines (with rolls placed on
//! them), circular arcs, a stall-turn pivot and a prop hang. The figure is centred on the show centre in front of
//! the pilot and entered into the wind, so wind does not drag it downwind of centre.
//!
//! Path tracking: from the cross-track error and the path curvature the guidance asks for an acceleration, adds
//! gravity, and points the nose so lift (angle of attack) and fuselage side force (sideslip) produce it — at
//! knife-edge that is top rudder, inverted it is push. The quaternion attitude controller flies that attitude.
//!
//! Before each figure the aircraft repositions: out past the entry gate, turns back, and only starts the figure
//! when it crosses the gate on the line, at the right height, heading and speed.

use crate::flight::Fc;
use crate::math::*;
use crate::model::{AC, G, RHO};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Fig {
    Loop = 1, Roll = 2, PointRoll = 3, Immelmann = 4, SplitS = 5, Hammerhead = 6, CubanEight = 7, TorqueRoll = 8,
    KnifeEdge = 9, RollingCircle = 10, Harrier = 11, Waterfall = 12,
}
impl Fig {
    pub fn from(c: u32) -> Option<Fig> {
        Some(match c {
            1 => Fig::Loop, 2 => Fig::Roll, 3 => Fig::PointRoll, 4 => Fig::Immelmann,
            5 => Fig::SplitS, 6 => Fig::Hammerhead, 7 => Fig::CubanEight, 8 => Fig::TorqueRoll,
            9 => Fig::KnifeEdge, 10 => Fig::RollingCircle, 11 => Fig::Harrier, 12 => Fig::Waterfall, _ => return None,
        })
    }
}

/// The full show.
pub const NSHOW: usize = 12;
pub const SHOW: [Fig; NSHOW] = [
    Fig::Loop, Fig::Roll, Fig::KnifeEdge, Fig::Immelmann, Fig::PointRoll, Fig::RollingCircle,
    Fig::SplitS, Fig::Hammerhead, Fig::Harrier, Fig::CubanEight, Fig::Waterfall, Fig::TorqueRoll,
];
/// Show centre: on the show line 46 m in front of the pilot (who stands at y = -14).
pub const CENTRE: [f64; 2] = [0.0, 32.0];
/// Lowest any part of a figure may be planned (m).
pub const FLOOR: f64 = 14.0;
const R: f64 = 9.0;
const V_FIG: f64 = 14.0;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum RollKind {
    None, Full, Half, Points4,
    /// roll back from the angle held on entry to wings-normal
    Unroll,
    /// quarter roll, hold knife-edge, quarter roll back
    KnifeEdge,
}

#[derive(Clone, Copy, Debug)]
pub enum El {
    /// Straight line `len` m; roll of `kind` between fractions `a`..`b` of its length.
    Line { len: f64, kind: RollKind, a: f64, b: f64 },
    /// Pull (toward the canopy) through `deg` on radius `r`.
    Arc { r: f64, deg: f64 },
    /// Vertical line until airspeed drops below `v` (or `max` m); `quarter` = quarter roll in the first 5 m so a
    /// following pivot turns the nose within the figure plane.
    UpLine { max: f64, v: f64, quarter: bool },
    /// Stall-turn pivot: 180° of rudder at the top; continues pointing down.
    Pivot,
    /// Prop hang spinning at `spin` rad/s for `t` s, then line the belly up with the show line.
    Hang { t: f64, spin: f64 },
    /// Pitch over from the hang into level flight along the show line.
    PitchOut,
    /// Level horizontal circle of radius `r` through `deg`, turning away from the pilot, with `rolls` rolls.
    HCircle { r: f64, deg: f64, rolls: f64 },
    /// Harrier: slow, nose-high level flight at `v` m/s for `len` m, held up on the prop.
    Harrier { len: f64, v: f64 },
    /// Waterfall tumble: nose-down rotation through `deg` at `rate` rad/s on full power, nearly in place.
    Tumble { deg: f64, rate: f64 },
    /// Accelerate on the line back to `v` m/s after slow flight.
    SpeedUp { v: f64 },
}
use El::*;
const fn line(len: f64) -> El { Line { len, kind: RollKind::None, a: 0.0, b: 0.0 } }
const fn rolled(len: f64, kind: RollKind, a: f64, b: f64) -> El { Line { len, kind, a, b } }
const fn arc(deg: f64) -> El { Arc { r: R, deg } }

const P_LOOP: &[El] = &[line(10.0), arc(360.0), line(12.0)];
const P_ROLL: &[El] = &[rolled(48.0, RollKind::Full, 0.22, 0.78)];
const P_POINTS: &[El] = &[rolled(64.0, RollKind::Points4, 0.1, 0.9)];
const P_IMMELMANN: &[El] = &[line(10.0), arc(180.0), rolled(26.0, RollKind::Half, 0.08, 0.5)];
const P_SPLIT_S: &[El] = &[rolled(24.0, RollKind::Half, 0.3, 0.75), arc(180.0), line(12.0)];
/// Stall turn: pull up, quarter roll so the pivot is in the figure plane, rudder pivot, quarter roll back, pull out.
const P_HAMMERHEAD: &[El] = &[line(10.0), Arc { r: 8.0, deg: 90.0 }, UpLine { max: 20.0, v: 5.0, quarter: true }, Pivot, rolled(8.0, RollKind::Unroll, 0.0, 0.6), Arc { r: 10.0, deg: 90.0 }, line(12.0)];
/// 5/8 loop, half roll on the 45° down-line, 3/4 loop, half roll, 1/8 loop to level.
const P_CUBAN: &[El] = &[line(10.0), arc(225.0), rolled(20.0, RollKind::Half, 0.3, 0.7), arc(270.0), rolled(20.0, RollKind::Half, 0.3, 0.7), arc(45.0), line(12.0)];
const P_KNIFE: &[El] = &[rolled(70.0, RollKind::KnifeEdge, 0.06, 0.94)];
const P_RCIRCLE: &[El] = &[line(8.0), HCircle { r: 36.0, deg: 360.0, rolls: 2.0 }, line(8.0)];
const P_HARRIER: &[El] = &[line(10.0), Harrier { len: 34.0, v: 4.5 }, SpeedUp { v: 12.0 }, line(10.0)];
const P_WATERFALL: &[El] = &[line(10.0), Harrier { len: 10.0, v: 4.0 }, Tumble { deg: 360.0, rate: 6.0 }, Harrier { len: 6.0, v: 4.0 }, SpeedUp { v: 12.0 }, line(10.0)];
const P_TORQUE: &[El] = &[line(10.0), Arc { r: 8.0, deg: 90.0 }, UpLine { max: 14.0, v: 3.5, quarter: false }, Hang { t: 4.0, spin: 4.0 }, PitchOut, line(14.0)];

pub fn program(f: Fig) -> &'static [El] {
    match f {
        Fig::Loop => P_LOOP, Fig::Roll => P_ROLL, Fig::PointRoll => P_POINTS, Fig::Immelmann => P_IMMELMANN,
        Fig::SplitS => P_SPLIT_S, Fig::Hammerhead => P_HAMMERHEAD, Fig::CubanEight => P_CUBAN, Fig::TorqueRoll => P_TORQUE,
        Fig::KnifeEdge => P_KNIFE, Fig::RollingCircle => P_RCIRCLE, Fig::Harrier => P_HARRIER, Fig::Waterfall => P_WATERFALL,
    }
}

/// Position, path angle and canopy side in the figure plane: u along the show line (into wind), w up.
/// `c` = +1 when the canopy is on the left of the path direction in (u, w), i.e. the normal upright sense.
#[derive(Clone, Copy, Debug)]
pub struct Pose { pub u: f64, pub w: f64, pub th: f64, pub c: f64 }

/// Nominal end of an element (for planning extents and anchoring the next element).
fn advance(p: Pose, e: El) -> Pose {
    let t = [cos(p.th), sin(p.th)];
    match e {
        Line { len, kind, .. } => Pose { u: p.u + len * t[0], w: p.w + len * t[1], th: p.th, c: if matches!(kind, RollKind::Half) { -p.c } else { p.c } },
        Arc { r, deg } => {
            let nrm = [-t[1] * p.c, t[0] * p.c];
            let cen = [p.u + r * nrm[0], p.w + r * nrm[1]];
            let th = p.th + p.c * deg * D2R;
            let n2 = [-sin(th) * p.c, cos(th) * p.c];
            Pose { u: cen[0] - r * n2[0], w: cen[1] - r * n2[1], th, c: p.c }
        }
        UpLine { max, .. } => Pose { u: p.u + 0.6 * max * t[0], w: p.w + 0.6 * max * t[1], ..p },
        Pivot => Pose { th: -PI / 2.0, c: -p.c, ..p },
        Hang { .. } => p,
        PitchOut => Pose { u: p.u + 6.0, w: p.w + 3.0, th: 0.0, c: 1.0 },
        HCircle { .. } | Tumble { .. } => p,
        Harrier { len, .. } => Pose { u: p.u + len, ..p },
        SpeedUp { .. } => Pose { u: p.u + 25.0, th: 0.0, c: 1.0, ..p },
    }
}

/// Lowest point and horizontal extent of a figure planned from (0, 0) heading into wind, as seen from the ground
/// when the figure frame drifts along the line at `d` m/s: the path is walked in time and each point shifted.
fn extents(f: Fig, d: f64) -> (f64, f64, f64) {
    let (mut wmin, mut umin, mut umax) = (0.0f64, 0.0f64, 0.0f64);
    let mut p = Pose { u: 0.0, w: 0.0, th: 0.0, c: 1.0 };
    let mut t = 0.0;
    let mut mark = |u: f64, w: f64, t: f64| { let ug = u + d * t; wmin = wmin.min(w); umin = umin.min(ug); umax = umax.max(ug); };
    for e in program(f) {
        match *e {
            Arc { r, deg } => {
                let steps = (deg / 5.0) as usize + 1;
                for k in 1..=steps {
                    let q = advance(p, Arc { r, deg: deg * k as f64 / steps as f64 });
                    mark(q.u, q.w, t + r * deg * D2R * k as f64 / steps as f64 / V_FIG);
                }
                t += r * deg * D2R / V_FIG;
            }
            Line { len, .. } => {
                let steps = (len as usize).max(1);
                for k in 1..=steps {
                    let fr = k as f64 / steps as f64;
                    mark(p.u + len * fr * cos(p.th), p.w + len * fr * sin(p.th), t + len * fr / V_FIG);
                }
                t += len / V_FIG;
            }
            UpLine { max, .. } => { t += 0.6 * max / 9.0; }
            Pivot => { t += 1.3; }
            Hang { t: th, .. } => { t += th; }
            PitchOut => { t += 1.7; }
            HCircle { r, deg, .. } => {
                let steps = (deg / 5.0) as usize + 1;
                for k in 1..=steps {
                    let a = deg * D2R * k as f64 / steps as f64;
                    mark(p.u + r * sin(a), p.w, t + r * a / V_FIG);
                }
                t += r * deg * D2R / V_FIG;
            }
            Harrier { len, v } => {
                for k in 1..=10 { let fr = k as f64 / 10.0; mark(p.u + len * fr, p.w, t + len * fr / v); }
                t += len / v;
            }
            Tumble { deg, rate } => { t += deg * D2R / rate; }
            SpeedUp { .. } => { t += 2.5; }
        }
        p = advance(p, *e);
        mark(p.u, p.w, t);
    }
    (wmin, umin, umax)
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Step { Running, FigureStarted(Fig), FigureDone(Fig), ShowDone }

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Stage { PitchOutOfHover = 0, Positioning = 1, Flying = 2 }

/// Progress through a show (or a single figure).
#[derive(Clone, Copy)]
pub struct Show {
    pub seq: [Fig; NSHOW],
    pub n: usize,
    pub fig: usize,
    pub single: bool,
    pub stage: Stage,
    /// Show-line direction for this figure (into wind), and the figure plane's origin on the ground.
    pub h: [f64; 2],
    pub origin: [f64; 2],
    /// Height of the entry gate.
    pub entry_w: f64,
    /// Positioning: 0 = flying out to the turn point, 1 = established on the line toward the gate.
    pub leg: u8,
    // flying a figure
    pub el: usize,
    pub start: Pose,
    pub prog: f64,
    pub qref: Q,
    pub t: f64,
    pub hd: [f64; 2],
    pub thr_i: f64,
    /// Path error statistics for the current figure (m): sum of squares, samples, worst.
    pub err2: f64, pub errn: f64, pub errmax: f64,
    /// On the planned path (within 4 m) of a figure whose plan stays above the floor: recovery may trust the plan.
    pub on_track: bool,
    /// Roll angle held on entry to an Unroll element (rad).
    pub phi0: f64,
    /// "Flying the wind": mean wind (smoothed while positioning), the along-line drift of the figure frame (m/s)
    /// during the figure, the frame origin at the gate, and time since the gate.
    pub wmean: [f64; 2],
    pub drift: f64,
    pub origin0: [f64; 2],
    pub tfig: f64,
}

impl Show {
    pub fn full() -> Show { Show::new(SHOW, SHOW.len(), false) }
    /// Seed the mean wind with the current estimate when the show starts.
    pub fn seed_wind(&mut self, w: [f64; 2]) { self.wmean = w; }
    pub fn single(f: Fig) -> Show { Show::new([f; NSHOW], 1, true) }
    fn new(seq: [Fig; NSHOW], n: usize, single: bool) -> Show {
        Show {
            seq, n, fig: 0, single, stage: Stage::Positioning, h: [1.0, 0.0], origin: CENTRE, entry_w: 20.0, leg: 0,
            el: 0, start: Pose { u: 0.0, w: 0.0, th: 0.0, c: 1.0 }, prog: 0.0, qref: [1.0, 0.0, 0.0, 0.0], t: 0.0,
            hd: [1.0, 0.0], thr_i: 0.0, err2: 0.0, errn: 0.0, errmax: 0.0, on_track: false, phi0: 0.0, wmean: [0.0; 2], drift: 0.0, origin0: CENTRE, tfig: 0.0,
        }
    }
    pub fn current(&self) -> Fig { self.seq[self.fig.min(self.n - 1)] }
    /// Abandon the figure in progress (after a recovery) and line up for the next one.
    pub fn skip_figure(&mut self) -> bool {
        self.fig += 1; self.stage = Stage::Positioning; self.t = 0.0; self.leg = 0; self.on_track = false;
        self.fig < self.n
    }
    pub fn path_rms(&self) -> f64 { if self.errn > 0.0 { sqrt(self.err2 / self.errn) } else { 0.0 } }

    // ---- plane coordinates
    fn h3(&self) -> V3 { [self.h[0], self.h[1], 0.0] }
    /// Normal of the figure plane: left of the show line direction.
    fn nrm(&self) -> V3 { [-self.h[1], self.h[0], 0.0] }
    fn world(&self, u: f64, w: f64) -> V3 { [self.origin[0] + self.h[0] * u, self.origin[1] + self.h[1] * u, w] }
    fn to_uw(&self, p: V3) -> (f64, f64, f64) {
        let d = [p[0] - self.origin[0], p[1] - self.origin[1], 0.0];
        (dot(d, self.h3()), p[2], dot(d, self.nrm()))
    }
    fn dir(&self, th: f64) -> V3 { add(mul(self.h3(), cos(th)), [0.0, 0.0, sin(th)]) }

    /// Plan the next figure: into the wind, centred on the show centre, lowest point at or above the floor.
    fn plan(&mut self, fc: &Fc) {
        let w = fc.nav.wind_est;
        // into wind along the show line (parallel to the crowd line, x); calm: keep the previous direction
        if abs(w[0]) > 0.6 { self.h = [if w[0] < 0.0 { 1.0 } else { -1.0 }, 0.0]; }
        let f = self.current();
        // the figure drifts with the along-line mean wind: centre the drifted shape, as seen from the ground
        let d = self.wmean[0] * self.h[0] + self.wmean[1] * self.h[1];
        let (wmin, umin, umax) = extents(f, d);
        // measured centring trim: where the flown figure really sits relative to its nominal plan
        let trim = match f { Fig::Hammerhead => -5.8, Fig::Harrier => 8.3, Fig::Waterfall => 8.4, _ => 0.0 };
        let u_gate = -(umin + umax) / 2.0 + trim;
        self.origin = [CENTRE[0] + self.h[0] * u_gate, CENTRE[1] + self.h[1] * u_gate];
        self.origin0 = self.origin;
        self.entry_w = max(20.0, FLOOR - wmin);
        self.leg = 0;
    }

    pub fn step(&mut self, fc: &mut Fc, dt: f64) -> Step {
        let n = fc.nav;
        let (x, _, _) = axes(n.q);
        match self.stage {
            Stage::PitchOutOfHover => {
                self.t += dt;
                let th = max(0.22, PI / 2.0 - self.t * 0.95);
                fc.dir_ctl([self.hd[0] * cos(th), self.hd[1] * cos(th), sin(th)], 0.0, dt);
                fc.act.thr = 0.95;
                if th <= 0.22 && x[2] < 0.5 { self.stage = Stage::Positioning; self.t = 0.0; }
                Step::Running
            }
            Stage::Positioning => {
                if x[2] > 0.6 && n.air < 6.0 {
                    self.stage = Stage::PitchOutOfHover; self.t = 0.0; self.hd = fc.belly_dir();
                    return Step::Running;
                }
                for i in 0..2 { self.wmean[i] += (n.wind_est[i] - self.wmean[i]) * min(1.0, dt / 8.0); }
                if self.leg == 0 && (self.t == 0.0 || (self.t % 1.0) < dt) { self.plan(fc); }
                self.t += dt;
                self.position(fc, dt)
            }
            Stage::Flying => self.fly(fc, dt),
        }
    }

    /// Fly out beyond the gate, turn back onto the show line, cross the gate on line, height, heading and speed.
    fn position(&mut self, fc: &mut Fc, dt: f64) -> Step {
        let n = fc.nav;
        let (u, w, off) = self.to_uw(n.p);
        let vh = [n.v[0], n.v[1]];
        let along = dot([vh[0], vh[1], 0.0], self.h3()) / max(len2(vh), 1.0);
        // turn point: 70 m before the gate, on the far side of the line from the pilot
        let back = 70.0;
        if self.leg == 0 {
            let tp = self.world(-back, 0.0);
            let side = mul(self.nrm(), if self.nrm()[1] > 0.0 { 25.0 } else { -25.0 });
            let tgt = add(tp, side);
            let phi = fc.bank_toward(tgt[0], tgt[1], 18.0);
            fc.fwd_ctl(phi, self.entry_w, 15.0, dt);
            if hypot(n.p[0] - tgt[0], n.p[1] - tgt[1]) < 18.0 || (u < -back + 10.0 && along > 0.3) { self.leg = 1; }
        } else {
            // L1 along the show line toward the gate
            let carrot = self.world(u + 25.0, 0.0);
            let phi = fc.bank_toward(carrot[0], carrot[1], 25.0);
            fc.fwd_ctl(phi, self.entry_w, 15.0, dt);
            if u >= 0.0 {
                let ok = abs(off) < 5.0 && abs(w - self.entry_w) < 3.0 && along > 0.95 && n.air > 11.5;
                if ok { return self.begin(fc); }
                self.leg = 0; // not set up: go round again
            }
        }
        Step::Running
    }

    fn begin(&mut self, fc: &Fc) -> Step {
        self.stage = Stage::Flying;
        self.el = 0;
        self.prog = 0.0;
        self.t = 0.0;
        self.thr_i = 0.0;
        self.err2 = 0.0; self.errn = 0.0; self.errmax = 0.0;
        self.drift = self.wmean[0] * self.h[0] + self.wmean[1] * self.h[1];
        self.tfig = 0.0;
        self.origin = self.origin0;
        self.start = Pose { u: 0.0, w: self.entry_w, th: 0.0, c: 1.0 };
        self.qref = fc.nav.q;
        Step::FigureStarted(self.current())
    }

    /// Re-anchor the next element where the aircraft actually is (after a pivot, hang or pitch-out).
    fn anchor_here(&mut self, fc: &Fc, th: f64, c: f64) {
        let (u, w, _) = self.to_uw(fc.nav.p);
        self.start = Pose { u, w, th, c };
        // roll angle of the actual attitude relative to this element's wings-normal frame
        let t = self.dir(th);
        let y0 = mul(self.nrm(), c);
        let z0 = cross(t, y0);
        let y = axes(fc.nav.q).1;
        self.phi0 = atan2(dot(y, z0), dot(y, y0));
    }

    fn fly(&mut self, fc: &mut Fc, dt: f64) -> Step {
        let f = self.current();
        let prog = program(f);
        let e = prog[self.el];
        let n = fc.nav;
        let mut done = false;
        let mut next_anchor: Option<(f64, f64)> = None;
        self.t += dt;
        // the figure frame drifts with the along-line mean wind
        self.tfig += dt;
        self.origin = [self.origin0[0] + self.h[0] * self.drift * self.tfig, self.origin0[1] + self.h[1] * self.drift * self.tfig];
        if !matches!(e, Line { .. } | Arc { .. } | UpLine { .. } | HCircle { .. }) { self.on_track = n.p[2] > FLOOR; }
        match e {
            Line { .. } | Arc { .. } | UpLine { .. } | HCircle { .. } => done = self.track(fc, e, dt),
            Harrier { len, v } => done = self.harrier(fc, len, v, dt),
            Tumble { deg, rate } => {
                // waterfall: rotate the attitude reference nose-down about the wing axis, full power in the prop wash
                if self.prog == 0.0 { self.qref = n.q; }
                let step = min(rate * dt, deg * D2R - self.prog);
                let h = 0.5 * step;
                self.qref = qnorm(qmul(self.qref, [cos(h), 0.0, sin(h), 0.0]));
                self.prog += step;
                fc.att_ctl(self.qref, [0.0, -rate, 0.0], dt, 6.0);
                fc.act.thr = 1.0;
                if self.prog >= deg * D2R - 1e-9 { done = true; next_anchor = Some((0.0, 1.0)); }
            }
            SpeedUp { v } => {
                let (u, _, _) = self.to_uw(n.p);
                let carrot = self.world(u + 25.0, 0.0);
                let phi = fc.bank_toward(carrot[0], carrot[1], 25.0);
                fc.fwd_ctl(phi, self.start.w, 15.0, dt);
                if n.air > v && abs(axes(n.q).0[2]) < 0.3 { done = true; next_anchor = Some((0.0, 1.0)); }
            }
            Pivot => {
                // rudder pivot at the top: rotate the attitude reference about the body z axis
                if self.prog == 0.0 {
                    // nose straight up, yaw axis exactly along the figure-plane normal: the pivot stays in the plane
                    let z_b = axes(n.q).2;
                    let z = mul(self.nrm(), if dot(z_b, self.nrm()) >= 0.0 { 1.0 } else { -1.0 });
                    let x = [0.0, 0.0, 1.0];
                    self.qref = quat_from_axes(x, cross(z, x), z);
                }
                let step = min(2.5 * dt, PI - self.prog);
                let h = -0.5 * step;
                self.qref = qnorm(qmul(self.qref, [cos(h), 0.0, 0.0, sin(h)]));
                self.prog += step;
                fc.att_ctl(self.qref, [0.0, 0.0, 2.5], dt, 6.0);
                fc.act.thr = 0.9;
                if self.prog >= PI - 1e-9 { done = true; next_anchor = Some((-PI / 2.0, -self.start.c)); }
            }
            Hang { t, spin } => {
                // prop hang over the spot; for the last 1.5 s turn so the belly faces along the show line
                let mut sp = spin;
                if self.prog > t - 1.5 {
                    let b = horiz(mul(axes(n.q).2, -1.0));
                    let err = atan2(b[0] * self.h[1] - b[1] * self.h[0], b[0] * self.h[0] + b[1] * self.h[1]);
                    sp = clamp(2.0 * err, -2.0, 2.0);
                }
                fc.hover_ctl([0.0, 0.0], 0.0, sp, dt, 0.4);
                self.prog += dt;
                if self.prog >= t { done = true; }
            }
            PitchOut => {
                self.prog += dt;
                let th = max(0.22, PI / 2.0 - self.prog * 0.95);
                fc.dir_ctl([self.h[0] * cos(th), self.h[1] * cos(th), sin(th)], 0.0, dt);
                fc.act.thr = 0.95;
                if th <= 0.22 && axes(n.q).0[2] < 0.5 { done = true; next_anchor = Some((0.0, 1.0)); }
            }
        }
        if !done { return Step::Running; }
        // ---- next element
        let planned_next = advance(self.start, e);
        self.el += 1;
        self.prog = 0.0;
        match next_anchor {
            Some((th, c)) => self.anchor_here(fc, th, c),
            None => {
                if let UpLine { .. } = e { let (u, w, _) = self.to_uw(n.p); self.start = Pose { u, w, ..planned_next }; }
                else { self.start = planned_next; }
            }
        }
        if self.el < prog.len() { return Step::Running; }
        // ---- figure finished
        self.on_track = false;
        self.drift = 0.0;
        self.fig += 1;
        self.stage = Stage::Positioning;
        self.t = 0.0;
        self.leg = 0;
        if self.fig >= self.n { return if self.single { Step::FigureDone(f) } else { Step::ShowDone }; }
        Step::FigureDone(f)
    }

    /// Path tracking for lines, arcs and up-lines. Returns true when the element is complete.
    fn track(&mut self, fc: &mut Fc, e: El, dt: f64) -> bool {
        let n = fc.nav;
        let up = [0.0, 0.0, 1.0];
        let s0 = self.start;
        let (u, w, _) = self.to_uw(n.p);
        let vf = if self.stage == Stage::Flying { sub(n.v, mul(self.h3(), self.drift)) } else { n.v };
        let v = len(vf);
        // desired point, tangent, inward normal, curvature, completion, fraction done, wings-normal frame y axis
        let (p_des, t, nin, kappa, done, frac, y0);
        match e {
            Arc { r, deg } => {
                let t0 = [cos(s0.th), sin(s0.th)];
                let nrm0 = [-t0[1] * s0.c, t0[0] * s0.c];
                let cen = [s0.u + r * nrm0[0], s0.w + r * nrm0[1]];
                let d = [u - cen[0], w - cen[1]];
                let a0 = atan2(-nrm0[1], -nrm0[0]);
                let a = atan2(d[1], d[0]);
                // angle swept in the turning direction, unwrapped near the progress so far (360° loops)
                let mut swept = (a - a0) * s0.c;
                let k = round((self.prog - swept) / (2.0 * PI));
                swept += k * 2.0 * PI;
                self.prog = max(self.prog, min(swept, self.prog + 0.5));
                let ang = a0 + self.prog * s0.c;
                let th = s0.th + s0.c * self.prog;
                p_des = self.world(cen[0] + r * cos(ang), cen[1] + r * sin(ang));
                t = self.dir(th);
                nin = self.dir(th + s0.c * PI / 2.0);
                kappa = 1.0 / r;
                done = self.prog >= deg * D2R;
                frac = self.prog / (deg * D2R);
                y0 = mul(self.nrm(), s0.c);
            }
            Line { len: l, .. } | UpLine { max: l, .. } => {
                let t0 = [cos(s0.th), sin(s0.th)];
                let s = (u - s0.u) * t0[0] + (w - s0.w) * t0[1];
                self.prog = max(self.prog, s);
                p_des = self.world(s0.u + s * t0[0], s0.w + s * t0[1]);
                t = self.dir(s0.th);
                nin = self.dir(s0.th + s0.c * PI / 2.0);
                kappa = 0.0;
                let slow = matches!(e, UpLine { v: vs, .. } if n.air < vs);
                done = s >= l || slow;
                frac = clamp(s / l, 0.0, 1.0);
                y0 = mul(self.nrm(), s0.c);
            }
            HCircle { r, deg, .. } => {
                // level circle turning away from the pilot (south of the show line)
                let sgn = if self.nrm()[1] > 0.0 { 1.0 } else { -1.0 };
                let p0 = self.world(s0.u, s0.w);
                let cen = add(p0, mul(self.nrm(), sgn * r));
                let rel = [n.p[0] - cen[0], n.p[1] - cen[1]];
                let a0 = atan2(p0[1] - cen[1], p0[0] - cen[0]);
                let a = atan2(rel[1], rel[0]);
                let mut swept = (a - a0) * sgn;
                let k = round((self.prog - swept) / (2.0 * PI));
                swept += k * 2.0 * PI;
                self.prog = max(self.prog, min(swept, self.prog + 0.5));
                let ang = a0 + sgn * self.prog;
                let rh = [cos(ang), sin(ang), 0.0];
                p_des = [cen[0] + r * rh[0], cen[1] + r * rh[1], s0.w];
                t = mul([-rh[1], rh[0], 0.0], sgn);
                nin = mul(rh, -1.0);
                kappa = 1.0 / r;
                done = self.prog >= deg * D2R;
                frac = self.prog / (deg * D2R);
                y0 = cross([0.0, 0.0, 1.0], t);
            }
            _ => unreachable!(),
        }
        // ---- roll on this element
        let (phi, phi_rate) = match e {
            UpLine { quarter: true, .. } => { let s = self.prog; (PI / 2.0 * clamp(s / 5.0, 0.0, 1.0), if s < 5.0 { PI / 2.0 / 5.0 * v } else { 0.0 }) }
            HCircle { r, deg, rolls } => (rolls * 2.0 * PI * frac, rolls * 2.0 * PI * v / (r * deg * D2R)),
            _ => roll_angle(e, frac, v, self.phi0),
        };
        // reference frame: x along the path, y = wings-normal axis, then rolled
        let z0 = cross(t, y0);
        let (cp, sp) = (cos(phi), sin(phi));
        let yr = add(mul(y0, cp), mul(z0, sp));
        let zr = sub(mul(z0, cp), mul(y0, sp));
        // ---- guidance: curvature + cross-track PD (in-plane and out-of-plane), plus gravity
        let mut e_pos = sub(p_des, n.p);
        e_pos = sub(e_pos, mul(t, dot(e_pos, t)));
        let el = len(e_pos);
        self.on_track = el < 4.0;
        if !matches!(e, UpLine { .. }) { self.err2 += el * el; self.errn += 1.0; self.errmax = self.errmax.max(el); }
        let v_perp = sub(vf, mul(t, dot(vf, t)));
        let a_des = add(add(mul(nin, kappa * v * v), mul(e_pos, 1.6)), mul(v_perp, -2.2));
        let a_req = add(a_des, [0.0, 0.0, G]);
        // ---- point the nose so lift and side force make that acceleration
        let wv = [n.wind_est[0], n.wind_est[1], 0.0];
        let va = sub(n.v, wv);
        let vam = max(len(va), 4.0);
        let vah = mul(va, 1.0 / vam);
        let a_perp = sub(a_req, mul(vah, dot(a_req, vah)));
        let qs = 0.5 * RHO * vam * vam;
        let alpha = clamp(dot(a_perp, zr) * AC.m / (qs * 0.91), -0.32, 0.32);
        let beta = clamp(dot(a_perp, yr) * AC.m / (qs * 0.16), -0.45, 0.45);
        let xd = norm(add(add(vah, mul(zr, alpha)), mul(yr, beta)));
        let zd = norm(sub(zr, mul(xd, dot(zr, xd))));
        let yd = cross(zd, xd);
        let qd = quat_from_axes(xd, yd, zd);
        fc.att_ctl(qd, [phi_rate, kappa * v, 0.0], dt, 5.0);
        // ---- throttle: hold figure speed, more going up, less coming down; up-lines bleed speed
        let climb = dot(t, up);
        let ve = V_FIG - n.air;
        self.thr_i = clamp(self.thr_i + 0.04 * ve * dt, -0.2, 0.2);
        fc.act.thr = match e {
            // keep pulling with power until the path is truly vertical, then bleed the speed off
            UpLine { .. } => if n.v[2] > 0.97 * v { 0.12 } else { 0.7 },
            // at knife-edge the thrust, angled up by the top rudder, helps hold the line
            _ => clamp(0.36 + 0.55 * climb + 0.08 * ve + self.thr_i + 0.35 * abs(sin(phi)), 0.05, 1.0),
        };
        done
    }
}

impl Show {
    /// Harrier: nose-high slow flight. Pitch sets the speed, throttle holds the height (as in the hover), heading
    /// holds the line. Returns true at the end of the line.
    fn harrier(&mut self, fc: &mut Fc, len: f64, v_t: f64, dt: f64) -> bool {
        let n = fc.nav;
        let (u, w, off) = self.to_uw(n.p);
        let vf = sub(n.v, mul(self.h3(), self.drift));
        let along = dot(vf, self.h3());
        let vlat = dot(vf, self.nrm());
        let psi = atan2(self.h[1], self.h[0]) - clamp(0.06 * off + 0.25 * vlat, -0.4, 0.4);
        // nose-up for speed only gradually (pitching straight up at 12 m/s balloons), lower it when high
        let th = clamp(0.75 + clamp(0.06 * (along - v_t), -0.2, 0.25) - clamp(0.04 * (w - self.start.w), -0.1, 0.15), 0.35, 1.15);
        fc.att_ctl(quat_euler(psi, th, 0.0), [0.0; 3], dt, 5.0);
        // throttle on vertical speed: wing and prop share the weight in a harrier, so no hover feed-forward
        let vz_cmd = clamp(0.8 * (self.start.w - w), -1.5, 1.5);
        let ez = vz_cmd - n.v[2];
        self.thr_i = clamp(self.thr_i + 0.25 * ez * dt, -0.35, 0.45);
        fc.act.thr = clamp(0.3 + 0.18 * ez + self.thr_i, 0.0, 1.0);
        let e = hypot(off, w - self.start.w);
        self.on_track = e < 4.0 && n.p[2] > FLOOR - 4.0;
        self.err2 += e * e; self.errn += 1.0; self.errmax = self.errmax.max(e);
        u - self.start.u >= len
    }
}

/// Roll angle (rad, right positive) and rate along an element at progress `frac`.
fn roll_angle(e: El, frac: f64, v: f64, phi0: f64) -> (f64, f64) {
    let (len, kind, a, b) = match e { Line { len, kind, a, b } => (len, kind, a, b), _ => return (0.0, 0.0) };
    if kind == RollKind::None { return (0.0, 0.0); }
    let total = match kind { RollKind::Full | RollKind::Points4 => 2.0 * PI, RollKind::Half => PI, RollKind::None | RollKind::Unroll | RollKind::KnifeEdge => 0.0 };
    let span = (b - a) * len;
    let x = clamp((frac - a) / (b - a), 0.0, 1.0);
    let moving = frac > a && frac < b;
    match kind {
        RollKind::Unroll => (phi0 * (1.0 - x), if moving { -phi0 / span * v } else { 0.0 }),
        RollKind::KnifeEdge => {
            // quarter roll over the first 12%, knife-edge, quarter roll back over the last 12%
            let q = PI / 2.0;
            let (ang, r) = if x < 0.12 { (q * x / 0.12, q / (0.12 * span) * v) } else if x > 0.88 { (q * (1.0 - x) / 0.12, -q / (0.12 * span) * v) } else { (q, 0.0) };
            (ang, if moving { r } else { 0.0 })
        }
        RollKind::Points4 => {
            // four 90° points: roll for 65% of each quarter, hold for the rest
            let q = min(x * 4.0, 3.9999);
            let k = q as u32 as f64;
            let r = clamp((q - k) / 0.65, 0.0, 1.0);
            let ang = (k + r) * PI / 2.0;
            let rate = if moving && (q - k) < 0.65 { (PI / 2.0) / (0.65 * span / 4.0) * v } else { 0.0 };
            (if x >= 1.0 { total } else { ang }, rate)
        }
        _ => (total * x, if moving { total / span * v } else { 0.0 }),
    }
}
