//! Scenario benchmark: autoland, QLoiter in wind, takeoff/circuit, recovery. `cargo run --release --example bench [real]`
use foamie_core::flight::{Mode, Task};
use foamie_core::sim::{Ev, Sim};

fn run(s: &mut Sim, sec: f64) { let n = (sec / 0.002) as usize; for _ in 0..n { s.step(); if s.crashed { break; } } }
fn drain(s: &mut Sim) -> Vec<(Ev, [f64; 4])> { s.take_events().iter().flatten().map(|e| (e.code, [e.a, e.b, e.c, e.d])).collect() }

fn main() {
    let real = std::env::args().any(|a| a == "real");
    let seeds: u64 = if real { 3 } else { 1 };
    println!("== {} ==", if real { "REAL PLANE (sensors + estimator + build errors + battery)" } else { "IDEAL" });
    // ---- autoland
    let (mut tt, mut miss, mut score, mut n, mut fails) = (0.0, 0.0, 0.0, 0, 0);
    for seed in 0..seeds { for wl in 0..3 { for off in [0.0, 5.0, 10.0, 15.0, 20.0, 25.0] {
        let mut s = Sim::new(100 + seed * 7 + wl as u64 * 31 + off as u64);
        if real { s.set_real(true); }
        s.set_wind(wl); s.start_auto(Task::Takeoff); run(&mut s, 14.0 + off); drain(&mut s);
        s.start_auto(Task::Land); let t0 = s.truth.t;
        let mut res = None;
        for _ in 0..60000 { s.step(); if s.crashed || s.auto.is_none() { break; } }
        for (c, a) in drain(&mut s) { if c == Ev::TailLanding { res = Some(a); } if c == Ev::Crash { res = None; } }
        let d = (s.truth.p[0].powi(2) + s.truth.p[1].powi(2)).sqrt();
        let t = s.truth.t - t0;
        match res { Some(a) => { tt += t; miss += d; score += a[0]; n += 1; }
                    None => { fails += 1; println!("   FAIL wind {} off {} crashed {} t {:.1} d {:.1}", wl, off, s.crashed, t, d); } }
    }}}
    println!("autoland      n {:2}  mean time {:5.1} s  miss {:4.2} m  score {:4.1}  fails {}", n, tt / n.max(1) as f64, miss / n.max(1) as f64, score / n.max(1) as f64, fails);

    // ---- QLoiter hold in wind, including after spinning broadside
    let (mut dsum, mut dmax, mut k) = (0.0f64, 0.0f64, 0);
    for wl in [1usize, 2] { for spin in [false, true] {
        let mut s = Sim::new(500 + wl as u64 * 3 + spin as u64);
        if real { s.set_real(true); }
        s.set_wind(wl); s.set_mode(Mode::QLoiter); s.sticks.ly = -0.7; run(&mut s, 4.0); s.sticks.ly = 0.0;
        if spin { s.sticks.lx = 1.0; run(&mut s, 0.6); s.sticks.lx = 0.0; }
        run(&mut s, 3.0);
        let h0 = [s.truth.p[0], s.truth.p[1]];
        for _ in 0..15000 { s.step(); let d = ((s.truth.p[0] - h0[0]).powi(2) + (s.truth.p[1] - h0[1]).powi(2)).sqrt(); dsum += d; dmax = dmax.max(d); k += 1; }
        if s.crashed { println!("   QLoiter CRASH wind {}", wl); }
    }}
    println!("QLoiter wind  mean drift {:4.2} m  max {:4.2} m", dsum / k as f64, dmax);

    // ---- recovery scenarios
    let sc = |name: &str, f: &dyn Fn(&mut Sim)| {
        let mut s = Sim::new(900); if real { s.set_real(true); }
        s.start_auto(Task::Takeoff); run(&mut s, 12.0); drain(&mut s);
        f(&mut s);
        let ev = drain(&mut s);
        let rec = ev.iter().filter(|e| e.0 == Ev::Recovery).count();
        println!("{:24} recoveries {}  crashed {}  alt {:5.1}", name, rec, s.crashed, s.truth.p[2]);
    };
    sc("manual full-down dive", &|s| { s.set_mode(Mode::Manual); s.sticks.ly = 1.0 - 2.0 * 0.6; s.sticks.ry = -1.0; run(s, 4.0); s.sticks.ry = 0.0; run(s, 4.0); });
    sc("acro split-S at 18 m", &|s| { s.set_mode(Mode::Acro); s.sticks.ly = 1.0 - 2.0 * 0.4; s.sticks.rx = 1.0; run(s, 0.55); s.sticks.rx = 0.0; s.sticks.ry = 1.0; run(s, 3.0); s.sticks.ry = 0.0; run(s, 4.0); });
    sc("inverted, motor off", &|s| { run(s, 3.0); s.set_mode(Mode::Manual); s.sticks.ly = 1.0; s.sticks.rx = 1.0; run(s, 0.45); s.sticks.rx = 0.0; run(s, 9.0); });
    sc("FBWA cruise (no alarm)", &|s| { s.set_mode(Mode::Fbwa); s.sticks.ly = 1.0 - 2.0 * 0.42; run(s, 10.0); });
    let mut s = Sim::new(901); if real { s.set_real(true); }
    s.set_mode(Mode::QLoiter); s.sticks.ly = -1.0; run(&mut s, 4.0); s.sticks.ly = 0.85; run(&mut s, 14.0);
    let ev = drain(&mut s);
    println!("{:24} recoveries {}  landed {}", "QLoiter full-down land", ev.iter().filter(|e| e.0 == Ev::Recovery).count(), ev.iter().any(|e| e.0 == Ev::TailLanding));
    if real { println!("battery after that      {:.2} V  {:.0}% ", s.truth.batt.volts, s.truth.batt.soc * 100.0); }
}
