//! Flight scenarios that must keep working. `cargo test --release`
use foamie_core::flight::{Mode, Task};
use foamie_core::sim::{Ev, Sim};

fn run(s: &mut Sim, sec: f64) { for _ in 0..(sec / 0.002) as usize { s.step(); if s.crashed { break; } } }
fn events(s: &mut Sim) -> Vec<(Ev, [f64; 4])> { s.take_events().iter().flatten().map(|e| (e.code, [e.a, e.b, e.c, e.d])).collect() }

/// Take off, fly part of the circuit, then autoland. Returns (tail landing score, miss distance m, time s).
fn autoland(seed: u64, real: bool, wind: usize, circuit_s: f64) -> Option<(f64, f64, f64)> {
    let mut s = Sim::new(seed);
    if real { s.set_real(true); }
    s.set_wind(wind);
    s.start_auto(Task::Takeoff);
    run(&mut s, 14.0 + circuit_s);
    events(&mut s);
    s.start_auto(Task::Land);
    let t0 = s.truth.t;
    for _ in 0..60_000 { s.step(); if s.crashed || s.auto.is_none() { break; } }
    let d = s.truth.p[0].hypot(s.truth.p[1]);
    events(&mut s).iter().find(|e| e.0 == Ev::TailLanding).map(|e| (e.1[0], d, s.truth.t - t0))
}

#[test]
fn ideal_autoland_is_fast_and_accurate() {
    let mut n = 0.0; let (mut miss, mut time) = (0.0, 0.0);
    for wind in 0..3 { for off in [0.0, 10.0, 20.0] {
        let (score, d, t) = autoland(100 + wind as u64 * 31 + off as u64, false, wind, off).expect("tail landing");
        assert!(score > 55.0, "score {score}");
        miss += d; time += t; n += 1.0;
    }}
    assert!(miss / n < 0.4, "mean miss {}", miss / n);
    assert!(time / n < 25.0, "mean time {}", time / n);
}

#[test]
fn real_plane_autoland_lands_within_gps_accuracy() {
    let mut ok = 0; let mut miss = 0.0;
    for wind in 0..3 { for off in [0.0, 10.0, 20.0] {
        if let Some((_, d, _)) = autoland(200 + wind as u64 * 31 + off as u64, true, wind, off) { ok += 1; miss += d; }
    }}
    assert!(ok >= 8, "only {ok}/9 tail landings");
    assert!(miss / ok as f64 <= 2.0, "mean miss {}", miss / ok as f64);
}

fn loiter_drift(real: bool, wind: usize, spin: bool) -> f64 {
    let mut s = Sim::new(500 + wind as u64);
    if real { s.set_real(true); }
    s.set_wind(wind);
    s.set_mode(Mode::QLoiter);
    s.sticks.ly = -0.7; run(&mut s, 4.0); s.sticks.ly = 0.0;
    if spin { s.sticks.lx = 1.0; run(&mut s, 0.6); s.sticks.lx = 0.0; }
    run(&mut s, 3.0);
    let h0 = [s.truth.p[0], s.truth.p[1]];
    run(&mut s, 20.0);
    assert!(!s.crashed);
    (s.truth.p[0] - h0[0]).hypot(s.truth.p[1] - h0[1])
}

#[test]
fn qloiter_holds_position_in_wind_even_after_spinning_broadside() {
    for wind in [1, 2] { for spin in [false, true] {
        assert!(loiter_drift(false, wind, spin) < 1.0);
        // real plane: GPS-limited at 2 m/s; at 4 m/s gusting 6.6 m/s the profile fuselage slab catches nearly as much
        // side force as the tilted thrust can resist (as on a real 3D foamie), so allow it to give a few metres
        let real = loiter_drift(true, wind, spin);
        assert!(real < if wind == 1 { 2.0 } else { 7.0 }, "real-plane drift {real:.1} m in wind level {wind}");
    }}
}

fn flying(real: bool) -> Sim {
    let mut s = Sim::new(900);
    if real { s.set_real(true); }
    s.start_auto(Task::Takeoff);
    run(&mut s, 12.0);
    events(&mut s);
    s
}

#[test]
fn recovery_saves_a_full_down_dive() {
    for real in [false, true] {
        let mut s = flying(real);
        s.set_mode(Mode::Manual); s.sticks.ly = 1.0 - 2.0 * 0.6; s.sticks.ry = -1.0;
        run(&mut s, 4.0); s.sticks.ry = 0.0; run(&mut s, 3.0);
        assert!(!s.crashed);
        assert_eq!(events(&mut s).iter().filter(|e| e.0 == Ev::Recovery).count(), 1);
    }
}

#[test]
fn recovery_saves_an_inverted_motor_off_glide() {
    let mut s = flying(false);
    run(&mut s, 3.0);
    s.set_mode(Mode::Manual); s.sticks.ly = 1.0; s.sticks.rx = 1.0; run(&mut s, 0.45); s.sticks.rx = 0.0; run(&mut s, 9.0);
    assert!(!s.crashed);
}

#[test]
fn no_false_alarms_for_split_s_cruise_or_hover_landing() {
    let mut s = flying(false);
    s.set_mode(Mode::Acro); s.sticks.ly = 1.0 - 2.0 * 0.4; s.sticks.rx = 1.0; run(&mut s, 0.55);
    s.sticks.rx = 0.0; s.sticks.ry = 1.0; run(&mut s, 3.0); s.sticks.ry = 0.0; run(&mut s, 3.0);
    assert!(!s.crashed);
    assert!(events(&mut s).iter().all(|e| e.0 != Ev::Recovery), "split-S that clears the ground");

    let mut s = flying(false);
    s.set_mode(Mode::Fbwa); s.sticks.ly = 1.0 - 2.0 * 0.42; run(&mut s, 10.0);
    assert!(events(&mut s).iter().all(|e| e.0 != Ev::Recovery), "level FBWA cruise");

    for real in [false, true] {
        let mut s = Sim::new(901);
        if real { s.set_real(true); }
        s.set_mode(Mode::QLoiter); s.sticks.ly = -1.0; run(&mut s, 4.0); s.sticks.ly = 0.85; run(&mut s, 14.0);
        let ev = events(&mut s);
        assert!(ev.iter().all(|e| e.0 != Ev::Recovery), "QLoiter descent");
        assert!(ev.iter().any(|e| e.0 == Ev::TailLanding), "QLoiter lands on its legs (real = {real})");
    }
}

#[test]
fn fbwa_levels_hands_off_and_transitions_from_hover() {
    let mut s = Sim::new(7);
    s.set_mode(Mode::QLoiter); s.sticks.ly = -0.6; run(&mut s, 5.0); s.sticks.ly = 0.0; run(&mut s, 2.0);
    s.set_mode(Mode::Fbwa); s.sticks.ly = 1.0 - 2.0 * 0.6; run(&mut s, 8.0);
    let x = foamie_core::math::axes(s.truth.q).0;
    assert!(x[2].abs() < 0.2, "level flight after transition, nose z {}", x[2]);
    assert!(s.truth.aux.air > 10.0);
    s.set_mode(Mode::QHover); run(&mut s, 6.0);
    assert!(foamie_core::math::axes(s.truth.q).0[2] > 0.9, "back in the hover");
    assert!(!s.crashed);
}

/// Fly a show (or one figure). Returns (crashed, figures started, recoveries, lowest point, worst centring error,
/// share of figures entered heading east).
fn show(real: bool, wind: usize, fig: Option<foamie_core::aerobatics::Fig>, seed: u64) -> (bool, usize, usize, f64, f64, f64) {
    use foamie_core::aerobatics::CENTRE;
    let mut s = Sim::new(seed);
    if real { s.set_real(true); }
    s.set_wind(wind);
    s.start_auto(Task::Takeoff);
    run(&mut s, 16.0);
    events(&mut s);
    s.start_show(fig);
    let (mut figs, mut rec, mut low, mut worst_c, mut east) = (0, 0, f64::INFINITY, 0.0f64, 0.0);
    let (mut xmin, mut xmax, mut in_fig) = (0.0f64, 0.0f64, false);
    for _ in 0..(400.0 / 0.002) as usize {
        s.step();
        if in_fig { low = low.min(s.truth.lowest()); xmin = xmin.min(s.truth.p[0]); xmax = xmax.max(s.truth.p[0]); }
        for e in events(&mut s) {
            match e.0 {
                Ev::FigureStarted => { figs += 1; in_fig = true; xmin = 1e9; xmax = -1e9; if s.truth.v[0] > 0.0 { east += 1.0; } }
                Ev::FigureDone | Ev::ShowDone => { if in_fig { worst_c = worst_c.max(((xmin + xmax) / 2.0 - CENTRE[0]).abs()); } in_fig = false; }
                Ev::Recovery => { rec += 1; in_fig = false; }
                _ => {}
            }
        }
        if s.crashed || s.auto.map_or(true, |a| a.task == Task::Circuit) { break; }
    }
    (s.crashed, figs, rec, low, worst_c, east / figs.max(1) as f64)
}

#[test]
fn every_aerobatic_figure_flies_cleanly_and_centred() {
    for f in 1..=8 {
        let fig = foamie_core::aerobatics::Fig::from(f).unwrap();
        let (crashed, figs, rec, low, centre, _) = show(false, 0, Some(fig), 40 + f as u64);
        assert!(!crashed && figs == 1 && rec == 0, "{fig:?}: crashed {crashed}, recoveries {rec}");
        assert!(low > 10.0, "{fig:?} got down to {low:.1} m");
        assert!(centre < 3.0, "{fig:?} centred {centre:.1} m off show centre");
    }
}

#[test]
fn full_aerobatic_show_ideal_and_real() {
    for real in [false, true] {
        let (crashed, figs, rec, _, _, _) = show(real, 0, None, 77);
        assert!(!crashed, "real = {real}");
        assert_eq!(figs, 8, "all eight figures (real = {real})");
        if !real { assert_eq!(rec, 0, "no recovery needed in calm air"); }
    }
}

#[test]
fn figures_enter_into_wind_and_show_survives_gusts() {
    // wind from the east: every figure starts heading east; recoveries may abandon a figure, never the aircraft
    let (crashed, figs, _, _, _, east) = show(false, 2, None, 78);
    assert!(!crashed && figs == 8);
    assert!(east > 0.99, "into wind: {east}");
}

#[test]
fn show_from_the_pad_takes_off_first() {
    let mut s = Sim::new(3);
    s.start_show(Some(foamie_core::aerobatics::Fig::Loop));
    let mut started = false;
    for _ in 0..(90.0 / 0.002) as usize { s.step(); if events(&mut s).iter().any(|e| e.0 == Ev::FigureStarted) { started = true; break; } }
    assert!(started && !s.crashed);
}

#[test]
fn ekf_estimate_is_accurate_in_hover_and_manoeuvres() {
    use foamie_core::math::*;
    let (mut att, mut vel, mut n, mut worst) = (0.0, 0.0, 0.0, 0.0f64);
    let mut s = Sim::new(12);
    s.set_real(true); s.set_wind(1);
    s.start_auto(Task::Takeoff);
    for i in 0..(60.0 / 0.002) as usize {
        s.step();
        if i == 15_000 { s.start_show(Some(foamie_core::aerobatics::Fig::Loop)); }
        if i % 10 == 0 {
            let e = qmul(conj(s.nav().q), s.truth.q);
            let a = 2.0 * atan2((e[1] * e[1] + e[2] * e[2] + e[3] * e[3]).sqrt(), e[0].abs()) / D2R;
            att += a * a; worst = worst.max(a); vel += len(sub(s.nav().v, s.truth.v)).powi(2); n += 1.0;
        }
    }
    assert!(!s.crashed);
    assert!((att / n).sqrt() < 1.5, "attitude rms {:.2}°", (att / n).sqrt());
    assert!(worst < 6.0, "attitude worst {worst:.1}°");
    assert!((vel / n).sqrt() < 0.25, "velocity rms {:.2} m/s", (vel / n).sqrt());
}
