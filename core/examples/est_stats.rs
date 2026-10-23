//! Real-plane estimator accuracy: RMS error of the flight controller's estimate vs truth.
//! `cargo run --release --example est_stats`
use foamie_core::aerobatics::Fig;
use foamie_core::flight::{Mode, Task};
use foamie_core::math::*;
use foamie_core::sim::Sim;

#[derive(Default)] struct Acc { att: f64, vel: f64, pos: f64, alt: f64, n: f64, att_max: f64, tilt: f64 }
fn sample(s: &Sim, a: &mut Acc) {
    let n = s.nav(); let t = &s.truth;
    let e = qmul(conj(n.q), t.q);
    let att = 2.0 * atan2((e[1] * e[1] + e[2] * e[2] + e[3] * e[3]).sqrt(), e[0].abs()) / D2R;
    let dv = len(sub(n.v, t.v));
    let dp = ((n.p[0] - t.p[0]).powi(2) + (n.p[1] - t.p[1]).powi(2)).sqrt();
    let dz = n.p[2] - t.p[2];
    let (un, ut) = (qinv(n.q, [0.0, 0.0, 1.0]), qinv(t.q, [0.0, 0.0, 1.0]));
    let tilt = acos(clamp(dot(un, ut), -1.0, 1.0)) / D2R; a.tilt += tilt * tilt;
    a.att += att * att; a.vel += dv * dv; a.pos += dp * dp; a.alt += dz * dz; a.n += 1.0; a.att_max = a.att_max.max(att);
}
fn report(name: &str, a: &Acc) {
    println!("{:22} tilt {:4.2}° attitude {:5.2}° (max {:5.1}°)  velocity {:4.2} m/s  position {:4.2} m  height {:4.2} m",
        name, (a.tilt / a.n).sqrt(), (a.att / a.n).sqrt(), a.att_max, (a.vel / a.n).sqrt(), (a.pos / a.n).sqrt(), (a.alt / a.n).sqrt());
}
fn main() {
    let ekf = !std::env::args().any(|a| a == "old");
    println!("estimator: {}", if ekf { "EKF" } else { "complementary (old)" });
    let mut total = Acc::default();
    for seed in [11u64, 12, 13] {
        // hover in wind
        let mut s = Sim::new(seed); s.set_real(true); s.use_ekf = ekf; s.set_wind(2); s.set_mode(Mode::QLoiter);
        s.sticks.ly = -0.7; for _ in 0..2000 { s.step(); } s.sticks.ly = 0.0;
        let mut a = Acc::default();
        for i in 0..15000 { s.step(); if i % 10 == 0 { sample(&s, &mut a); sample(&s, &mut total); } }
        report(&format!("hover, 4 m/s wind #{seed}"), &a);
        // circuit
        let mut s = Sim::new(seed + 100); s.set_real(true); s.use_ekf = ekf; s.set_wind(1); s.start_auto(Task::Takeoff);
        for _ in 0..8000 { s.step(); }
        let mut a = Acc::default();
        for i in 0..15000 { s.step(); if i % 10 == 0 { sample(&s, &mut a); sample(&s, &mut total); } }
        report(&format!("circuit #{seed}"), &a);
        // aerobatics
        let mut s = Sim::new(seed + 200); s.set_real(true); s.use_ekf = ekf; s.start_auto(Task::Takeoff);
        for _ in 0..8000 { s.step(); }
        s.start_show(Some(Fig::CubanEight));
        let mut a = Acc::default();
        for i in 0..30000 { s.step(); if i % 10 == 0 { sample(&s, &mut a); sample(&s, &mut total); } if s.auto.map_or(true, |x| x.task == Task::Circuit) { break; } }
        report(&format!("Cuban eight #{seed}"), &a);
    }
    report("ALL", &total);
}
