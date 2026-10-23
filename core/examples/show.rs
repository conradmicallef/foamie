//! Fly each aerobatic figure, then the whole show. `cargo run --release --example show [real]`
use foamie_core::aerobatics::Fig;
use foamie_core::flight::Task;
use foamie_core::math::*;
use foamie_core::sim::{Ev, Sim};

fn heading(s: &Sim) -> f64 { atan2(s.truth.v[1], s.truth.v[0]) / D2R }
fn fly(real: bool, fig: Option<Fig>, seed: u64) {
    let mut s = Sim::new(seed);
    if real { s.set_real(true); }
    s.start_auto(Task::Takeoff);
    for _ in 0..(16.0 / 0.002) as usize { s.step(); }
    s.take_events();
    s.start_show(fig);
    let (mut t_start, mut h0, mut hd0, mut minz, mut rec) = (None, 0.0, 0.0, 1e9f64, 0);
    let mut log = String::new();
    for _ in 0..(150.0 / 0.002) as usize {
        s.step();
        if t_start.is_some() { minz = minz.min(s.truth.lowest()); }
        for e in s.take_events().iter().flatten() {
            match e.code {
                Ev::FigureStarted => { t_start = Some(s.truth.t); h0 = s.truth.p[2]; hd0 = heading(&s); minz = 1e9;
                    log += &format!("  {:<11}", format!("{:?}", Fig::from(e.a as u32).unwrap())); }
                Ev::FigureDone | Ev::ShowDone if e.code == Ev::FigureDone => {
                    let mut dh = heading(&s) - hd0; while dh > 180.0 { dh -= 360.0; } while dh < -180.0 { dh += 360.0; }
                    log += &format!(" {:5.1} s  alt {:5.1}→{:5.1} m  low {:5.1} m  heading {:+5.0}°\n", s.truth.t - t_start.unwrap(), h0, s.truth.p[2], minz, dh);
                }
                Ev::Recovery => { rec += 1; log += " RECOVERY "; }
                Ev::Crash => log += " CRASH ",
                _ => {}
            }
        }
        if s.crashed || s.auto.map_or(true, |a| a.task != Task::Show) { break; }
    }
    print!("{}", log);
    if fig.is_none() { println!("  -> show finished {}  crashed {}  recoveries {}", s.auto.map_or(false, |a| a.task == Task::Circuit), s.crashed, rec); }
}
fn main() {
    let real = std::env::args().any(|a| a == "real");
    println!("== single figures ({}) ==", if real { "real plane" } else { "ideal" });
    for f in 1..=8 { fly(real, Fig::from(f), 40 + f as u64); }
    println!("== full show ==");
    fly(real, None, 77);
}
