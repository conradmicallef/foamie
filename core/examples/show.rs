//! Fly each aerobatic figure, then the whole show. `cargo run --release --example show [real] [wind]`
use foamie_core::aerobatics::{Fig, CENTRE};
use foamie_core::flight::Task;
use foamie_core::sim::{Ev, Sim};

fn fly(real: bool, wind: usize, fig: Option<Fig>, seed: u64) -> (usize, bool, usize) {
    let mut s = Sim::new(seed);
    if real { s.set_real(true); }
    s.set_wind(wind);
    s.start_auto(Task::Takeoff);
    for _ in 0..(16.0 / 0.002) as usize { s.step(); }
    s.take_events();
    s.start_show(fig);
    let (mut t_prev, mut t_fig, mut figs, mut rec) = (s.truth.t, 0.0, 0, 0);
    let (mut xmin, mut xmax, mut low, mut yoff, mut hd) = (1e9f64, -1e9f64, 1e9f64, 0.0f64, 0.0);
    let mut in_fig = false;
    for _ in 0..(400.0 / 0.002) as usize {
        s.step();
        if in_fig { xmin = xmin.min(s.truth.p[0]); xmax = xmax.max(s.truth.p[0]); low = low.min(s.truth.lowest()); yoff = yoff.max((s.truth.p[1] - CENTRE[1]).abs()); }
        for e in s.take_events().iter().flatten() {
            match e.code {
                Ev::FigureStarted => { in_fig = true; t_fig = s.truth.t; hd = s.truth.v[0].signum();
                    print!("  {:<11} positioning {:5.1} s |", format!("{:?}", Fig::from(e.a as u32).unwrap()), s.truth.t - t_prev);
                    xmin = 1e9; xmax = -1e9; low = 1e9; yoff = 0.0; figs += 1; }
                Ev::FigureDone | Ev::ShowDone => if in_fig {
                    in_fig = false; t_prev = s.truth.t;
                    println!(" {:4.1} s  entry {}  path rms {:4.2} m max {:4.2} m  centre x {:+5.1} m  off-line {:4.1} m  low {:5.1} m",
                        s.truth.t - t_fig, if hd > 0.0 { "→E" } else { "←W" }, e.b, e.c, (xmin + xmax) / 2.0 - CENTRE[0], yoff, low);
                },
                Ev::Recovery => { rec += 1; if in_fig { println!(" RECOVERY → next figure"); in_fig = false; t_prev = s.truth.t; } }
                Ev::Crash => print!(" CRASH"),
                _ => {}
            }
        }
        if s.crashed || s.auto.map_or(true, |a| a.task == Task::Circuit) { break; }
    }
    if in_fig { println!(" (unfinished)"); }
    (figs, s.crashed, rec)
}
fn main() {
    let real = std::env::args().any(|a| a == "real");
    let wind = if std::env::args().any(|a| a == "wind") { 2 } else { 0 };
    println!("== single figures ({}, wind {} m/s from E) ==", if real { "real plane" } else { "ideal" }, [0, 2, 4][wind]);
    for f in 1..=12 { fly(real, wind, Fig::from(f), 40 + f as u64); }
    println!("== full show ==");
    let (figs, crashed, rec) = fly(real, wind, None, 77);
    println!("  -> {} figures, crashed {}, recoveries {}", figs, crashed, rec);
}
