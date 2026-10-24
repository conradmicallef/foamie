//! Precision-landing diagnostics: tag lock, GPS offset and estimate error over a hover. `cargo run --release --example cam_diag`
use foamie_core::flight::Mode;
use foamie_core::sim::Sim;
fn main() {
    for cam in [true, false] { for wl in [1usize, 2] { for spin in [false, true] {
        let mut s = Sim::new(500 + wl as u64 * 3 + spin as u64);
        s.set_cam(cam); s.set_real(true);
        s.set_wind(wl); s.set_mode(Mode::QLoiter); s.sticks.ly = -0.7;
        for _ in 0..2000 { s.step(); } s.sticks.ly = 0.0;
        if spin { s.sticks.lx = 1.0; for _ in 0..300 { s.step(); } s.sticks.lx = 0.0; }
        for _ in 0..1500 { s.step(); }
        let h0 = [s.truth.p[0], s.truth.p[1]];
        let (mut e, mut d, mut lock, mut k) = (0.0, 0.0, 0, 0);
        for i in 0..15000 { s.step(); if i % 50 == 0 {
            let n = s.nav();
            e += ((n.p[0] - s.truth.p[0]).powi(2) + (n.p[1] - s.truth.p[1]).powi(2)).sqrt();
            d += ((s.truth.p[0] - h0[0]).powi(2) + (s.truth.p[1] - h0[1]).powi(2)).sqrt();
            if s.ekf.tag_age < 0.3 { lock += 1; } k += 1; } }
        println!("cam {cam:5} wind {wl} spin {spin:5}: alt {:4.1} pos {:5.1} {:5.1}  est err {:4.2} m  drift {:4.2} m  tag lock {:3.0}%  tags {}",
            s.truth.p[2], s.truth.p[0], s.truth.p[1], e / k as f64, d / k as f64, 100.0 * lock as f64 / k as f64, s.ekf.tag_n);
    }}}
}
