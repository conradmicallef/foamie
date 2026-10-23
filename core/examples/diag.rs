//! Estimator accuracy in real-plane mode. `cargo run --release --example diag [ideal_airframe]`
use foamie_core::flight::{Mode, Task};
use foamie_core::math::*;
use foamie_core::model::Variation;
use foamie_core::sim::Sim;

fn att_err(a: Q, b: Q) -> f64 { let e = qmul(conj(a), b); 2.0 * atan2((e[1]*e[1]+e[2]*e[2]+e[3]*e[3]).sqrt(), e[0].abs()) / D2R }
fn report(s: &Sim, tag: &str) {
    let n = s.nav(); let t = &s.truth;
    println!("{:14} t {:5.1} att {:5.1}°  pos {:5.2} m  alt {:+5.2}  vel {:4.2}  vz {:+5.2}/{:+5.2}  agl {:5.2}/{:5.2} air {:4.1}/{:4.1} thr {:.2} mode {:?} {:?} ground {}",
        tag, t.t, att_err(n.q, t.q), ((n.p[0]-t.p[0]).powi(2)+(n.p[1]-t.p[1]).powi(2)).sqrt(), n.p[2]-t.p[2],
        len(sub(n.v, t.v)), n.v[2], t.v[2], n.agl_tail, t.p[2] + qrot(t.q,[foamie_core::model::LEGX,0.0,0.0])[2], n.air, t.aux.air, t.thr, s.mode, s.auto.map(|a| a.phase), n.on_ground);
}
fn main() {
    let ideal_af = std::env::args().any(|a| a == "ideal_airframe");
    let mut s = Sim::new(42); s.set_real(true); if ideal_af { s.var = Variation::IDEAL; }
    s.set_mode(Mode::QLoiter); s.sticks.ly = -0.7;
    for i in 0..8 { for _ in 0..500 { s.step(); } report(&s, if i < 4 { "QLoiter climb" } else { "QLoiter hold" }); if i == 3 { s.sticks.ly = 0.0; } }
    let mut s = Sim::new(43); s.set_real(true); if ideal_af { s.var = Variation::IDEAL; }
    s.start_auto(Task::Takeoff);
    for i in 0..20 { for _ in 0..1000 { s.step(); if s.crashed { break; } } report(&s, "takeoff/circ"); if s.crashed { println!("CRASHED"); break; } let _ = i; }
    s.start_auto(Task::Land);
    for _ in 0..40 { for _ in 0..500 { s.step(); if s.crashed { break; } } report(&s, "land"); if s.crashed { println!("CRASHED"); break; } if s.auto.is_none() { break; } }
}
