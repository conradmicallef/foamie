//! Trace one real-plane autoland. `cargo run --release --example land_trace <seed> <wind> <circuit s> [nocam]`
use foamie_core::flight::Task;
use foamie_core::sim::Sim;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let seed: u64 = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(125);
    let wl: usize = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let off: f64 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(25.0);
    let mut s = Sim::new(seed); s.set_cam(!a.iter().any(|x| x == "nocam")); s.set_real(true); s.set_wind(wl);
    s.start_auto(Task::Takeoff);
    for _ in 0..((14.0 + off) / 0.002) as usize { s.step(); }
    s.take_events();
    s.start_auto(Task::Land);
    for i in 0..60000 { s.step(); if s.crashed || s.auto.is_none() { break; }
        if i % 500 == 0 { let n = s.nav(); let t = &s.truth;
            println!("t {:5.1} phase {:?} true {:6.2} {:6.2} {:5.2} est {:6.2} {:6.2} {:5.2} bgps {:5.2} {:5.2} tag {:4.1} n {}",
                t.t, s.auto.map(|a| a.phase as u8), t.p[0], t.p[1], t.p[2], n.p[0], n.p[1], n.p[2], s.ekf.bgps[0], s.ekf.bgps[1], s.ekf.tag_age.min(9.9), s.ekf.tag_n); } }
    for e in s.take_events().iter().flatten() { println!("event {:?} {:.2} {:.2} {:.2} {:.2}", e.code, e.a, e.b, e.c, e.d); }
}
