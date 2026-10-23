//! ArduPilot SITL "JSON" physics backend for the foamie tail-sitter.
//!
//! Run `arduplane --model JSON:127.0.0.1 ...`; ArduPilot sends servo PWM over UDP to port 9002 each physics frame,
//! this program steps the same rigid-body/aero model the browser sim uses (`foamie_core::model::Truth`) and replies
//! with IMU, position, velocity and attitude as one line of JSON.
//!
//! Frames: the core works in ENU world / FLU body (x nose, y left wing, z top); ArduPilot wants NED / FRD.
//!
//! Usage: foamie-sitl [--wind m/s] [--log file.csv] [--port 9002]

use foamie_core::math::*;
use foamie_core::model::{Act, Contact, Truth, Variation, Wind, LEGX};
use std::io::Write;
use std::net::UdpSocket;

const MAGIC16: u16 = 18458;
const MAGIC32: u16 = 29569;

/// ENU → NED vector.
fn ned(v: V3) -> V3 { [v[1], v[0], -v[2]] }
/// FLU → FRD vector.
fn frd(v: V3) -> V3 { [v[0], -v[1], -v[2]] }
/// Attitude of the FRD body in NED: q_T ⊗ q ⊗ q_D (T: ENU→NED, 180° about (1,1,0)/√2; D: 180° about body x).
fn q_ned(q: Q) -> Q {
    let s = core::f64::consts::FRAC_1_SQRT_2;
    qnorm(qmul(qmul([0.0, s, s, 0.0], q), [0.0, 1.0, 0.0, 0.0]))
}
fn servo(pwm: u16) -> f64 { if pwm == 0 { 0.0 } else { clamp((pwm as f64 - 1500.0) / 500.0, -1.0, 1.0) } }
fn throttle(pwm: u16) -> f64 { if pwm == 0 { 0.0 } else { clamp((pwm as f64 - 1000.0) / 1000.0, 0.0, 1.0) } }

/// Tail lidar: range along the tail (-x body) from the tail end to flat ground, like the browser sim's sensor.
fn lidar(t: &Truth) -> Option<f64> {
    let d = qrot(t.q, [-1.0, 0.0, 0.0]);
    if d[2] > -0.3 { return None; }
    let h = t.p[2] + qrot(t.q, [LEGX, 0.0, 0.0])[2];
    let r = h / -d[2];
    if r > 12.0 { None } else { Some(max(r, 0.02)) }
}

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let arg = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned();
    let wind_speed: f64 = arg("--wind").and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let port: u16 = arg("--port").and_then(|s| s.parse().ok()).unwrap_or(9002);
    let mut log = arg("--log").map(|p| std::fs::File::create(p).expect("log file"));
    if let Some(f) = log.as_mut() { writeln!(f, "t,x_e,y_n,z_up,roll,pitch,yaw,vx,vy,vz,thr,ail,elev,rud,contact")?; }

    let sock = UdpSocket::bind(("0.0.0.0", port))?;
    eprintln!("foamie-sitl: listening on udp {port}, wind {wind_speed} m/s");
    let wind = Wind { speed: wind_speed, phase: [0.3, 1.1, 2.0, 0.7] };
    let var = Variation::IDEAL;
    let mut truth = Truth::standing();
    let mut last_frame: Option<u32> = None;
    let mut next_log = 0.0;
    let mut buf = [0u8; 2048];
    loop {
        let (n, from) = sock.recv_from(&mut buf)?;
        if n < 8 { continue; }
        let magic = u16::from_le_bytes([buf[0], buf[1]]);
        let nch = match magic { MAGIC16 => 16, MAGIC32 => 32, _ => continue };
        if n < 8 + 2 * nch { continue; }
        let rate = u16::from_le_bytes([buf[2], buf[3]]).max(50) as f64;
        let frame = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        let pwm = |i: usize| u16::from_le_bytes([buf[8 + 2 * i], buf[9 + 2 * i]]);
        match last_frame {
            Some(l) if frame < l => { eprintln!("foamie-sitl: SITL restarted, resetting"); truth = Truth::standing(); next_log = 0.0; }
            Some(l) if frame == l => continue,
            Some(l) if frame > l + 1 => eprintln!("foamie-sitl: missed {} frames", frame - l - 1),
            _ => {}
        }
        last_frame = Some(frame);

        // SERVO1 aileron, SERVO2 elevator, SERVO3 throttle, SERVO4 rudder (ArduPilot's default plane outputs)
        let act = Act { a: servo(pwm(0)), e: servo(pwm(1)), thr: throttle(pwm(2)), r: servo(pwm(3)) };
        let mut contacts: [Option<Contact>; 4] = [None; 4];
        let dt = 1.0 / rate;
        truth.step(&act, &wind, &var, dt, &mut contacts);

        let p = ned(truth.p);
        let v = ned(truth.v);
        let g = frd(truth.w);
        let a = frd(truth.f_body);
        let q = q_ned(truth.q);
        let wv = ned(wind.at(truth.t, truth.p[2]));
        let air = max(0.0, truth.aux.vax);
        let mut js = format!(
            "\n{{\"timestamp\":{:.6},\"imu\":{{\"gyro\":[{:.6},{:.6},{:.6}],\"accel_body\":[{:.5},{:.5},{:.5}]}},\
\"position\":[{:.5},{:.5},{:.5}],\"velocity\":[{:.5},{:.5},{:.5}],\"quaternion\":[{:.7},{:.7},{:.7},{:.7}],\
\"airspeed\":{:.3},\"velocity_wind\":[{:.3},{:.3},{:.3}]",
            truth.t, g[0], g[1], g[2], a[0], a[1], a[2], p[0], p[1], p[2], v[0], v[1], v[2], q[0], q[1], q[2], q[3],
            air, wv[0], wv[1], wv[2]);
        if let Some(r) = lidar(&truth) { js += &format!(",\"rng_1\":{:.3}", r); }
        js += "}\n";
        sock.send_to(js.as_bytes(), from)?;

        if truth.t >= next_log {
            next_log += 0.1;
            if let Some(f) = log.as_mut() {
                let (r, pi, y) = euler_frd(q);
                writeln!(f, "{:.2},{:.3},{:.3},{:.3},{:.1},{:.1},{:.1},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{}",
                    truth.t, truth.p[0], truth.p[1], truth.p[2], r, pi, y, truth.v[0], truth.v[1], truth.v[2],
                    truth.thr, truth.sv[0], truth.sv[1], truth.sv[2], truth.any_contact() as u8)?;
            }
        }
    }
}

/// Roll, pitch, yaw (degrees, ZYX) of an FRD-in-NED quaternion.
fn euler_frd(q: Q) -> (f64, f64, f64) {
    let [w, x, y, z] = q;
    let r = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let p = asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    let yw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    (r.to_degrees(), p.to_degrees(), yw.to_degrees())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn near(a: V3, b: V3) -> bool { (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9) }
    #[test]
    fn standing_frames() {
        let t = Truth::standing();
        let q = q_ned(t.q);
        // nose (FRD x) points up = NED -z; belly (FRD z) faces north; right wing (FRD y) points east... or west
        assert!(near(qrot(q, [1.0, 0.0, 0.0]), [0.0, 0.0, -1.0]));
        assert!(near(qrot(q, [0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]));
        // consistency: FRD vectors mapped through q_ned equal ENU-mapped FLU vectors
        for v in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
            assert!(near(qrot(q, frd(v)), ned(qrot(t.q, v))));
        }
        // at rest the accelerometer reads -g along FRD z... rotated: specific force points up (NED -z)
        let f = [0.0, 0.0, 9.81];
        assert!(near(qrot(q, frd(qinv(t.q, f))), [0.0, 0.0, -9.81]));
    }
}
