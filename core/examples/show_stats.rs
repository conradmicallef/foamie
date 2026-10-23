//! Full show over several seeds: figures completed, recoveries, mean path error, worst centring.
use foamie_core::aerobatics::CENTRE;
use foamie_core::flight::Task;
use foamie_core::sim::{Ev, Sim};
fn main() {
    let real = std::env::args().any(|a| a == "real");
    let (mut figs, mut rec, mut rms, mut nrms, mut cmax, mut crashes) = (0, 0, 0.0, 0.0, 0.0f64, 0);
    for seed in 0..5u64 {
        let mut s = Sim::new(300 + seed * 13); if real { s.set_real(true); } s.set_wind(2);
        s.start_auto(Task::Takeoff); for _ in 0..8000 { s.step(); } s.take_events();
        s.start_show(None);
        let (mut xmin, mut xmax, mut infig) = (0.0f64, 0.0f64, false);
        for _ in 0..(450.0 / 0.002) as usize {
            s.step();
            if infig { xmin = xmin.min(s.truth.p[0]); xmax = xmax.max(s.truth.p[0]); }
            for e in s.take_events().iter().flatten() { match e.code {
                Ev::FigureStarted => { infig = true; xmin = 1e9; xmax = -1e9; }
                Ev::FigureDone | Ev::ShowDone => if infig { infig = false; figs += 1; rms += e.b; nrms += 1.0; cmax = cmax.max(((xmin + xmax) / 2.0 - CENTRE[0]).abs()); },
                Ev::Recovery => { rec += 1; infig = false; }
                _ => {} } }
            if s.crashed || s.auto.map_or(true, |a| a.task == Task::Circuit) { break; }
        }
        if s.crashed { crashes += 1; }
    }
    println!("{}: figures completed {}/40, recoveries {}, crashes {}, mean path rms {:.2} m, worst centring {:.1} m",
        if real { "real plane" } else { "ideal" }, figs, rec, crashes, rms / nrms, cmax);
}
