use foamie_core::flight::Mode;
use foamie_core::sim::Sim;
fn run(s: &mut Sim, sec: f64) { for _ in 0..(sec / 0.002) as usize { s.step(); } }
fn main() {
    for real in [false, true] { for wind in [1usize, 2] { for spin in [false, true] {
        let (mut tot, mut mxa) = (0.0, 0.0f64);
        for seed in 0..3u64 {
            let mut s = Sim::new(500 + wind as u64 + seed * 17); if real { s.set_real(true); } s.set_wind(wind); s.set_mode(Mode::QLoiter);
            s.sticks.ly = -0.7; run(&mut s, 4.0); s.sticks.ly = 0.0;
            if spin { s.sticks.lx = 1.0; run(&mut s, 0.6); s.sticks.lx = 0.0; }
            run(&mut s, 3.0);
            let h0 = [s.truth.p[0], s.truth.p[1]];
            let mut mx: f64 = 0.0; let mut sum = 0.0;
            for _ in 0..20 { run(&mut s, 1.0); let d = ((s.truth.p[0]-h0[0]).powi(2)+(s.truth.p[1]-h0[1]).powi(2)).sqrt(); mx = mx.max(d); sum += d; }
            tot += sum / 20.0; mxa = mxa.max(mx);
            if s.crashed { println!("CRASH"); }
        }
        println!("{} wind {} spin {:5}: mean drift {:.2} m  worst {:.2} m", if real { "real " } else { "ideal" }, [0,2,4][wind], spin, tot / 3.0, mxa);
    }}}
}
