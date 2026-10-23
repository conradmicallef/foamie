//! foamie-core: the flight core of the 3D foamie tail-sitter simulator.
//!
//! Physics, sensor models, state estimation, flight modes, autopilot and crash recovery live here and are
//! compiled to WebAssembly; the TypeScript front end only draws, reads the sticks and shows messages.
//! The C-ABI below is the whole interface: plain numbers in, a flat `f64` state buffer and an event list out.

#![cfg_attr(target_arch = "wasm32", no_std)]
#![allow(static_mut_refs)]

pub mod math;
pub mod model;
pub mod sensors;
pub mod estimator;
pub mod ekf;
pub mod flight;
pub mod sim;
pub mod aerobatics;

use flight::{Mode, Task};
use math::*;
use sim::{Sim, MAX_EV};

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { core::arch::wasm32::unreachable() }

static mut SIM: Option<Sim> = None;
/// State layout (keep in step with `web/src/core.ts`).
pub const NSTATE: usize = 64;
static mut STATE: [f64; NSTATE] = [0.0; NSTATE];
static mut EVBUF: [f64; MAX_EV * 5] = [0.0; MAX_EV * 5];

fn sim() -> &'static mut Sim {
    unsafe {
        if SIM.is_none() { SIM = Some(Sim::new(1)); }
        SIM.as_mut().unwrap()
    }
}

#[no_mangle] pub extern "C" fn fz_init(seed: f64) { unsafe { SIM = Some(Sim::new(seed as u64)); } }
#[no_mangle] pub extern "C" fn fz_reset() { sim().reset(); }
#[no_mangle] pub extern "C" fn fz_set_sticks(rx: f64, ry: f64, lx: f64, ly: f64, l_active: u32) {
    let s = &mut sim().sticks;
    s.rx = cl1(rx); s.ry = cl1(ry); s.lx = cl1(lx); s.ly = cl1(ly); s.l_active = l_active != 0;
}
#[no_mangle] pub extern "C" fn fz_set_mode(m: u32) { sim().set_mode(Mode::from(m)); }
/// 1 takeoff, 2 circuit (toggles), 3 land, 4 aerobatic show, 10 + figure: one aerobatic figure.
#[no_mangle] pub extern "C" fn fz_cmd(c: u32) {
    let s = sim();
    match c {
        1 => s.start_auto(Task::Takeoff), 2 => s.toggle_circuit(), 3 => s.start_auto(Task::Land),
        4 => s.start_show(None),
        _ => if let Some(f) = aerobatics::Fig::from(c.wrapping_sub(10)) { s.start_show(Some(f)); },
    }
}
/// 1 wind level (0–2), 2 recovery on/off, 3 weathervane on/off, 4 real plane on/off (resets).
#[no_mangle] pub extern "C" fn fz_set_opt(id: u32, v: f64) {
    let s = sim();
    match id { 1 => s.set_wind(v as usize), 2 => s.recover_on = v != 0.0, 3 => s.vane_on = v != 0.0, 4 => s.set_real(v != 0.0), _ => {} }
}
#[no_mangle] pub extern "C" fn fz_advance(ft: f64) { sim().advance(ft); }

/// Refresh and return the state buffer.
#[no_mangle] pub extern "C" fn fz_state() -> *const f64 {
    let s = sim();
    let tr = &s.truth;
    let n = s.fc.nav;
    let w4 = s.wind.at(tr.t, 4.0);
    let mut o = [0.0f64; NSTATE];
    o[0..3].copy_from_slice(&tr.p);
    o[3..7].copy_from_slice(&tr.q);
    o[7..10].copy_from_slice(&tr.v);
    o[10..13].copy_from_slice(&tr.w);
    o[13] = tr.thr;
    o[14..17].copy_from_slice(&tr.sv);
    o[17] = tr.aux.thrust; o[18] = tr.aux.vi; o[19] = tr.aux.air; o[20] = tr.aux.alpha;
    o[21] = s.mode as u8 as f64;
    o[22] = s.auto.map(|a| a.task as u8 as f64).unwrap_or(0.0);
    o[23] = s.auto.map(|a| a.phase as u8 as f64).unwrap_or(0.0);
    o[24] = s.crashed as u8 as f64;
    o[25] = s.airborne as u8 as f64;
    o[26] = n.on_ground as u8 as f64;
    o[27] = if s.impact.0.is_finite() { s.impact.0 } else { -1.0 };
    o[28] = s.impact.1;
    o[29] = s.fc.time_to_recover();
    o[30] = s.fc.deliberate_landing() as u8 as f64;
    o[31] = s.recover_on as u8 as f64;
    o[32] = s.vane_on as u8 as f64;
    o[33] = s.real as u8 as f64;
    o[34] = s.wind_lvl as f64;
    o[35..38].copy_from_slice(&w4);
    o[38..41].copy_from_slice(&n.p);
    o[41..45].copy_from_slice(&n.q);
    o[45] = tr.batt.volts; o[46] = tr.batt.soc; o[47] = tr.batt.amps;
    o[48] = tr.t;
    o[49] = s.sticks.ly;
    o[50] = n.agl_tail;
    o[51] = n.air;
    o[52] = hypot(n.p[0] - tr.p[0], n.p[1] - tr.p[1]);
    o[53] = n.p[2] - tr.p[2];
    o[54] = s.fc.loiter.is_some() as u8 as f64;
    if let Some(l) = s.fc.loiter { o[55] = l[0]; o[56] = l[1]; }
    o[57] = tr.aux.thrust / (model::AC.m * model::G);
    o[58] = n.v[2];
    o[59] = s.var.battery as u8 as f64;
    if let Some(a) = s.auto { if a.task == Task::Show { o[60] = a.show.current() as u8 as f64; o[61] = a.show.fig as f64; o[62] = a.show.n as f64; o[63] = a.show.stage as u8 as f64; } }
    unsafe { STATE = o; STATE.as_ptr() }
}

/// Drain the event queue into the event buffer; returns how many (5 numbers each: code, a, b, c, d).
#[no_mangle] pub extern "C" fn fz_events() -> u32 {
    let evs = sim().take_events();
    let mut n = 0usize;
    unsafe {
        for e in evs.iter().flatten() {
            EVBUF[n * 5] = e.code as u32 as f64;
            EVBUF[n * 5 + 1] = e.a; EVBUF[n * 5 + 2] = e.b; EVBUF[n * 5 + 3] = e.c; EVBUF[n * 5 + 4] = e.d;
            n += 1;
        }
    }
    n as u32
}
#[no_mangle] pub extern "C" fn fz_events_ptr() -> *const f64 { unsafe { EVBUF.as_ptr() } }
#[no_mangle] pub extern "C" fn fz_nstate() -> u32 { NSTATE as u32 }
