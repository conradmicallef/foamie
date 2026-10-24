// Loads the Rust flight core (WebAssembly) and gives typed access to its state buffer and events.
// Layout must match core/src/lib.rs `fz_state`.
import wasmBytes from '../gen/foamie_core.wasm';
import type { Quat, V3 } from './vec';

interface Exports {
  memory: { buffer: ArrayBuffer };
  fz_init(seed: number): void;
  fz_reset(): void;
  fz_set_sticks(rx: number, ry: number, lx: number, ly: number, lActive: number): void;
  fz_set_mode(m: number): void;
  fz_cmd(c: number): void;
  fz_set_opt(id: number, v: number): void;
  fz_advance(ft: number): void;
  fz_state(): number;
  fz_events(): number;
  fz_events_ptr(): number;
  fz_nstate(): number;
}

export const enum Mode { Manual = 0, Fbwa = 1, Acro = 2, QStab = 3, QHover = 4, QLoiter = 5 }
export const enum Task { None = 0, Takeoff = 1, Circuit = 2, Land = 3, Recover = 4, Show = 5 }
export const enum Ev {
  TakeoffDone = 1, YouHaveControl = 2, TailLanding = 3, HardTail = 4, Belly = 5, Touched = 6, Crash = 7,
  Recovery = 8, PulledOut = 9, Recovered = 10, StickLy = 11, ModeChanged = 12, BatteryLow = 13, BatteryEmpty = 14,
  FigureStarted = 15, FigureDone = 16, ShowDone = 17,
}
export const enum Opt { Wind = 1, Recovery = 2, Weathervane = 3, RealPlane = 4, Camera = 5 }
export const enum Cmd { Takeoff = 1, Circuit = 2, Land = 3, Show = 4 }

export interface Event { code: Ev; a: number; b: number; c: number; d: number }

/** Snapshot of the core's state for one frame. */
export class State {
  constructor(private s: Float64Array) {}
  get p(): V3 { return [this.s[0], this.s[1], this.s[2]]; }
  get q(): Quat { return [this.s[3], this.s[4], this.s[5], this.s[6]]; }
  get v(): V3 { return [this.s[7], this.s[8], this.s[9]]; }
  get thr() { return this.s[13]; }
  /** Actual surface deflections, -1…1: aileron, elevator, rudder. */
  get sv(): V3 { return [this.s[14], this.s[15], this.s[16]]; }
  get thrust() { return this.s[17]; }
  get washSpeed() { return this.s[18]; }
  get airspeed() { return this.s[19]; }
  get aoa() { return this.s[20]; }
  get mode(): Mode { return this.s[21]; }
  get task(): Task { return this.s[22]; }
  get phase() { return this.s[23]; }
  get crashed() { return this.s[24] !== 0; }
  get airborne() { return this.s[25] !== 0; }
  get onGround() { return this.s[26] !== 0; }
  /** Predicted ground impact in seconds (Infinity if none within 10 s) and sink rate then. */
  get impactT() { return this.s[27] < 0 ? Infinity : this.s[27]; }
  get impactV() { return this.s[28]; }
  get timeToRecover() { return this.s[29]; }
  get deliberateLanding() { return this.s[30] !== 0; }
  get recoveryOn() { return this.s[31] !== 0; }
  get weathervaneOn() { return this.s[32] !== 0; }
  get real() { return this.s[33] !== 0; }
  get windLevel() { return this.s[34]; }
  get windAt4m(): V3 { return [this.s[35], this.s[36], this.s[37]]; }
  /** Where the flight controller believes it is (differs from truth in real-plane mode). */
  get navP(): V3 { return [this.s[38], this.s[39], this.s[40]]; }
  get battVolts() { return this.s[45]; }
  get battSoc() { return this.s[46]; }
  get battAmps() { return this.s[47]; }
  get t() { return this.s[48]; }
  get coreLy() { return this.s[49]; }
  get navAglTail() { return this.s[50]; }
  get navAir() { return this.s[51]; }
  get estErrH() { return this.s[52]; }
  get estErrZ() { return this.s[53]; }
  get thrustToWeight() { return this.s[57]; }
  get navVz() { return this.s[58]; }
  get battery() { return this.s[59] !== 0; }
  /** Aerobatic figure being flown (1–8, 0 = none), its index in the show and the show length. */
  get figure() { return this.s[60]; }
  get figureIndex() { return this.s[61]; }
  get showLength() { return this.s[62]; }
  /** 0 pitching out of a hover, 1 positioning for the entry gate, 2 flying the figure. */
  get showStage() { return this.s[63]; }
  /** Tail camera (real-plane mode): switched on, AprilTag locked, which tag (0 = 40 cm, 1 = 8 cm). */
  get camOn() { return this.s[64] !== 0; }
  get tagLock() { return this.s[65] !== 0; }
  get tagId() { return this.s[66]; }
  /** Last detection in the 640 × 480 image: centre u, v and size in pixels. */
  get tagPx(): V3 { return [this.s[67], this.s[68], this.s[69]]; }
  get tagAge() { return this.s[70]; }
  /** The EKF's estimate of the GPS's slowly wandering offset (horizontal, m). */
  get gpsOffset() { return this.s[71]; }
}

export class Core {
  private constructor(private x: Exports) {}

  static async load(seed: number): Promise<Core> {
    if (typeof WebAssembly === 'undefined') throw new Error('this browser has no WebAssembly');
    const { instance } = await WebAssembly.instantiate(wasmBytes, {});
    const x = instance.exports as unknown as Exports;
    x.fz_init(seed);
    return new Core(x);
  }

  reset() { this.x.fz_reset(); }
  sticks(rx: number, ry: number, lx: number, ly: number, lActive: boolean) { this.x.fz_set_sticks(rx, ry, lx, ly, lActive ? 1 : 0); }
  setMode(m: Mode) { this.x.fz_set_mode(m); }
  cmd(c: Cmd) { this.x.fz_cmd(c); }
  /** Fly one aerobatic figure (1–8), or the whole show with 0. */
  figure(f: number) { this.x.fz_cmd(f === 0 ? Cmd.Show : 10 + f); }
  opt(id: Opt, v: number | boolean) { this.x.fz_set_opt(id, typeof v === 'boolean' ? (v ? 1 : 0) : v); }
  advance(ft: number) { this.x.fz_advance(ft); }

  state(): State {
    const ptr = this.x.fz_state();
    return new State(new Float64Array(this.x.memory.buffer, ptr, this.x.fz_nstate()).slice());
  }
  events(): Event[] {
    const n = this.x.fz_events();
    const b = new Float64Array(this.x.memory.buffer, this.x.fz_events_ptr(), n * 5);
    const out: Event[] = [];
    for (let i = 0; i < n; i++) out.push({ code: b[i * 5] as Ev, a: b[i * 5 + 1], b: b[i * 5 + 2], c: b[i * 5 + 3], d: b[i * 5 + 4] });
    return out;
  }
}
