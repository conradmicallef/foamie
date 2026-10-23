// 3D view on a 2D canvas: sky, field, circuit, pad, windsock, trees, smoke, the aircraft with moving
// control surfaces and its shadow, plus (in real-plane mode) where the flight controller thinks it is.
import type { State } from './core';
import { Task } from './core';
import { add, clamp, cross, dot, horiz, len, mul, norm, qrot, sub, type V3 } from './vec';

const COL = {
  skyTop: '#5f97c9', skyLow: '#cfe0ea', haze: '#dfe8e4', grass: '#5f8a45', grassFar: '#7c9a63', strip: '#78a457',
  grid: 'rgba(255,255,255,0.13)', pad: '#d9d4c4', padRing: '#ff8a2a', circuit: 'rgba(255,255,255,0.55)',
  tree: '#2f5130', trunk: '#4a3a2a', shadow: 'rgba(16,30,10,0.32)',
  wingTop: '#f07a22', wingBot: '#1d3557', fus: '#eceee8', fus2: '#c9cec6', leg: '#262b2f', edge: 'rgba(10,14,18,0.5)',
  prop: 'rgba(240,244,248,0.3)', smoke: 'rgba(255,255,255,0.55)', sock: '#ff6a2a', pole: '#e8e8e8', nav: '#38e0ff',
};
const TREES: [number, number][] = [[-48, 40], [-60, 12], [-55, -22], [52, 48], [66, 8], [58, -30], [10, 78], [-22, 82], [35, 82], [-80, 55], [84, -8]];
const DMAX = 0.75, LEG = 0.22, LEGX = -0.78;
const PILOT: V3 = [0, -14, 1.7];

/** The racetrack the core flies (same geometry as core/src/flight.rs `circuit`). */
export const CIRCUIT: [number, number][] = (() => {
  const cx = 0, cy = 22, R = 26, half = 35, pts: [number, number][] = [];
  for (let i = 0; i <= 16; i++) { const a = -Math.PI / 2 + Math.PI * i / 16; pts.push([cx + half + R * Math.cos(a), cy + R * Math.sin(a)]); }
  for (let i = 0; i <= 16; i++) { const a = Math.PI / 2 + Math.PI * i / 16; pts.push([cx - half + R * Math.cos(a), cy + R * Math.sin(a)]); }
  return pts;
})();

interface Poly { pts: V3[]; n: V3; t: string; b: string; prop?: boolean }

function planeGeom(st: State): Poly[] {
  const [sa, se, sr] = st.sv;
  const aL = sa * DMAX, aR = -sa * DMAX, de = -se * DMAX, dr = sr * DMAX;
  const hz = (pts: V3[], hx: number, dl: number): V3[] => pts.map(([x, y, z]) => { const dx = x - hx; return [hx + dx * Math.cos(dl), y, z + dx * Math.sin(dl)]; });
  const hy = (pts: V3[], hx: number, dl: number): V3[] => pts.map(([x, y, z]) => { const dx = x - hx; return [hx + dx * Math.cos(dl), y + dx * Math.sin(dl), z]; });
  const W = (s: number): V3[] => [[0.12, 0.02 * s, 0], [0.10, 0.45 * s, 0], [-0.04, 0.45 * s, 0], [-0.04, 0.1 * s, 0], [-0.12, 0.1 * s, 0], [-0.12, 0.02 * s, 0]];
  const Ail = (s: number): V3[] => [[-0.04, 0.1 * s, 0], [-0.04, 0.45 * s, 0], [-0.10, 0.45 * s, 0], [-0.115, 0.1 * s, 0]];
  const Stab = (s: number): V3[] => [[-0.5, 0.03 * s, 0], [-0.55, 0.19 * s, 0], [-0.62, 0.19 * s, 0], [-0.62, 0.03 * s, 0]];
  const wing = { n: [0, 0, 1] as V3, t: COL.wingTop, b: COL.wingBot };
  const polys: Poly[] = [
    { pts: W(1), ...wing }, { pts: W(-1), ...wing },
    { pts: hz(Ail(1), -0.04, aL), ...wing }, { pts: hz(Ail(-1), -0.04, aR), ...wing },
    { pts: Stab(1), ...wing }, { pts: Stab(-1), ...wing },
    { pts: hz([[-0.62, 0.19, 0], [-0.71, 0.17, 0], [-0.71, -0.17, 0], [-0.62, -0.19, 0]], -0.62, de), ...wing },
    { pts: [[-0.5, 0, 0.03], [-0.6, 0, 0.15], [-0.64, 0, 0.15], [-0.64, 0, 0.03]], n: [0, 1, 0], t: COL.fus, b: COL.fus2 },
    { pts: hy([[-0.64, 0, -0.03], [-0.64, 0, 0.16], [-0.71, 0, 0.15], [-0.72, 0, -0.02]], -0.64, dr), n: [0, 1, 0], t: COL.wingTop, b: COL.wingTop },
    { pts: [[0.34, 0, 0.02], [0.28, 0, 0.06], [0.05, 0, 0.09], [-0.15, 0, 0.04], [-0.66, 0, 0.02], [-0.66, 0, -0.01], [-0.2, 0, -0.04], [0.1, 0, -0.06], [0.34, 0, -0.02]], n: [0, 1, 0], t: COL.fus, b: COL.fus2 },
  ];
  // X landing legs: four thin gussets from the tail out to the feet
  for (const [s, t] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    polys.push({ pts: [[-0.5, s * 0.015, t * 0.015], [-0.68, s * 0.03, t * 0.03], [LEGX, s * LEG, t * LEG], [LEGX + 0.035, s * LEG, t * LEG]], n: norm([0, s, -t]), t: COL.leg, b: COL.leg });
  }
  if (st.thr > 0.04) {
    const pd: V3[] = [];
    for (let i = 0; i < 16; i++) { const a = i / 16 * Math.PI * 2; pd.push([0.37, 0.127 * Math.cos(a), 0.127 * Math.sin(a)]); }
    polys.push({ pts: pd, n: [1, 0, 0], t: COL.prop, b: COL.prop, prop: true });
  }
  return polys;
}

export type View = 'pilot' | 'chase';

export class Renderer {
  private ctx: CanvasRenderingContext2D;
  private tgt: V3 = [0, 0, 1];
  private half = 0.5;
  private dir: V3 = [0, 1, 0];
  private right: V3 = [1, 0, 0];
  view: View = 'pilot';
  trail: V3[] = [];

  constructor(private cv: HTMLCanvasElement) { this.ctx = cv.getContext('2d')!; }

  draw(st: State, ft: number): void {
    const { cv, ctx } = this;
    const dpr = Math.min(devicePixelRatio || 1, 2), rect = cv.getBoundingClientRect();
    const w = Math.max(10, rect.width | 0), h = Math.max(10, rect.height | 0);
    if (cv.width !== w * dpr || cv.height !== h * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const P = st.p, q = st.q, UP: V3 = [0, 0, 1];

    // ---- camera: the pilot standing south of the pad (auto-zoom), or a chase camera behind
    let C: V3, T: V3, half: number;
    this.tgt = add(this.tgt, mul(sub(P, this.tgt), Math.min(1, ft * 10)));
    if (this.view === 'pilot') {
      C = PILOT; T = this.tgt;
      const want = clamp(Math.atan2(3.0, len(sub(P, C))) + 0.02, 0.055, 0.6);
      this.half += (want - this.half) * Math.min(1, ft * 3);
      half = this.half;
    } else {
      const vh = horiz(st.v), X = qrot(q, [1, 0, 0]), Z = qrot(q, [0, 0, 1]);
      let dd = vh[3] > 2.5 ? vh : horiz(X);
      if (dd[3] < 0.3 && vh[3] <= 2.5) dd = horiz(mul(Z, -1));
      const k = Math.min(1, ft * 2);
      if (dd[3] > 0.05) this.dir = norm(add(mul(this.dir, 1 - k), mul([dd[0], dd[1], 0], k)));
      C = add(add(P, mul(this.dir, -4.2)), [0, 0, 1.4]); C[2] = Math.max(0.5, C[2]);
      T = add(this.tgt, mul(this.dir, 1.2)); half = 0.52;
    }
    const f = norm(sub(T, C));
    let r = cross(f, UP); r = len(r) < 1e-3 ? this.right : norm(r); this.right = r;
    const u = cross(r, f);
    const focal = (Math.min(w, h) / 2) / Math.tan(half);
    const NEAR = 0.05;
    const c2 = (p: V3): V3 => { const d = sub(p, C); return [dot(d, r), dot(d, u), dot(d, f)]; };
    const scr = (c: V3): [number, number] => [w / 2 + focal * c[0] / c[2], h / 2 - focal * c[1] / c[2]];
    const proj = (p: V3) => { const c = c2(p); return c[2] > NEAR ? scr(c) : null; };
    const seg = (a: V3, b: V3) => {
      let ca = c2(a), cb = c2(b);
      if (ca[2] <= NEAR && cb[2] <= NEAR) return null;
      if (ca[2] <= NEAR) ca = add(ca, mul(sub(cb, ca), (NEAR - ca[2]) / (cb[2] - ca[2])));
      if (cb[2] <= NEAR) cb = add(cb, mul(sub(ca, cb), (NEAR - cb[2]) / (ca[2] - cb[2])));
      return [scr(ca), scr(cb)];
    };
    const line = (a: V3, b: V3) => { const s = seg(a, b); if (s) { ctx.moveTo(s[0][0], s[0][1]); ctx.lineTo(s[1][0], s[1][1]); } };
    const polyC = (cs: V3[]) => {
      const out: V3[] = [];
      for (let i = 0; i < cs.length; i++) {
        const a = cs[i], b = cs[(i + 1) % cs.length], ia = a[2] > NEAR, ib = b[2] > NEAR;
        if (ia) out.push(a);
        if (ia !== ib) out.push(add(a, mul(sub(b, a), (NEAR - a[2]) / (b[2] - a[2]))));
      }
      return out.length >= 3 ? out.map(scr) : null;
    };
    const path = (pts: [number, number][]) => { ctx.beginPath(); pts.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); ctx.closePath(); };
    const fillPoly = (pts: V3[], col: string) => { const sp = polyC(pts.map(c2)); if (!sp) return; ctx.fillStyle = col; path(sp); ctx.fill(); };

    // ---- sky and ground, split at the horizon (the camera never rolls)
    const fh = norm(cross(UP, r));
    const hc = c2(add(C, mul(fh, 1e5)));
    const hy = hc[2] > 1 ? clamp(h / 2 - focal * hc[1] / hc[2], -h, 3 * h) : (f[2] > 0 ? 3 * h : -h);
    const sky = ctx.createLinearGradient(0, Math.min(hy - h, 0), 0, Math.max(1, hy));
    sky.addColorStop(0, COL.skyTop); sky.addColorStop(0.85, COL.skyLow); sky.addColorStop(1, COL.haze);
    ctx.fillStyle = sky; ctx.fillRect(0, 0, w, h);
    if (hy < h) {
      const gr = ctx.createLinearGradient(0, hy, 0, Math.min(h, hy + h * 0.5) + 1);
      gr.addColorStop(0, COL.grassFar); gr.addColorStop(1, COL.grass);
      ctx.fillStyle = gr; ctx.fillRect(0, Math.max(0, hy), w, h - Math.max(0, hy));
    }

    // ---- mown strip, grid, circuit, pad
    fillPoly([[-45, -4, 0], [45, -4, 0], [45, 4, 0], [-45, 4, 0]], COL.strip);
    ctx.strokeStyle = COL.grid; ctx.lineWidth = 1; ctx.beginPath();
    const gx = Math.round(C[0] / 10) * 10, gy = Math.round(C[1] / 10) * 10;
    for (let x = gx - 150; x <= gx + 150; x += 10) line([x, gy - 150, 0], [x, gy + 150, 0]);
    for (let y = gy - 150; y <= gy + 150; y += 10) line([gx - 150, y, 0], [gx + 150, y, 0]);
    ctx.stroke();
    ctx.strokeStyle = COL.circuit; ctx.globalAlpha = st.task === Task.Circuit ? 0.9 : 0.35; ctx.lineWidth = 1.5; ctx.setLineDash([7, 7]); ctx.beginPath();
    for (let i = 0; i < CIRCUIT.length; i++) { const a = CIRCUIT[i], b = CIRCUIT[(i + 1) % CIRCUIT.length]; line([a[0], a[1], 0.03], [b[0], b[1], 0.03]); }
    ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
    const ring = (R: number, z: number): V3[] => Array.from({ length: 28 }, (_, i) => { const a = i / 28 * Math.PI * 2; return [R * Math.cos(a), R * Math.sin(a), z] as V3; });
    fillPoly(ring(1.6, 0.01), COL.pad);
    ctx.strokeStyle = COL.padRing; ctx.lineWidth = 2; ctx.beginPath();
    const rp = ring(1.25, 0.015); for (let i = 0; i < rp.length; i++) line(rp[i], rp[(i + 1) % rp.length]);
    line([-0.6, 0, 0.02], [0.6, 0, 0.02]); line([0, -0.6, 0.02], [0, 0.6, 0.02]);
    ctx.stroke();

    // ---- windsock
    const W0 = st.windAt4m, ws = Math.hypot(W0[0], W0[1]);
    const sockBase: V3 = [7, -7, 4];
    ctx.strokeStyle = COL.pole; ctx.lineWidth = 1.5; ctx.beginPath(); line([7, -7, 0], sockBase); ctx.stroke();
    const sd = ws > 0.3 ? [W0[0] / ws, W0[1] / ws] : [0.2, 0.1];
    const lift = clamp(ws / 4, 0.08, 1), Ls = 1.4;
    const tip: V3 = [sockBase[0] + sd[0] * Ls * lift, sockBase[1] + sd[1] * Ls * lift, sockBase[2] - Ls * (1 - lift) * 0.9];
    fillPoly([[sockBase[0], sockBase[1], sockBase[2] + 0.18], [sockBase[0], sockBase[1], sockBase[2] - 0.18], tip], COL.sock);

    // ---- trees, split by depth around the aircraft
    const planeDepth = c2(P)[2];
    const drawTree = ([x, y]: [number, number]) => {
      const c = c2([x, y, 5]); if (c[2] < 1) return;
      ctx.strokeStyle = COL.trunk; ctx.lineWidth = Math.max(1, focal * 0.35 / c[2]); ctx.beginPath(); line([x, y, 0], [x, y, 4]); ctx.stroke();
      const s = scr(c); ctx.fillStyle = COL.tree; ctx.beginPath(); ctx.arc(s[0], s[1], focal * 2.4 / c[2], 0, 7); ctx.fill();
    };
    const trees = TREES.map(t => ({ t, d: c2([t[0], t[1], 3])[2] })).sort((a, b) => b.d - a.d);
    trees.filter(o => o.d >= planeDepth).forEach(o => drawTree(o.t));

    // ---- smoke trail
    if (this.trail.length > 1) {
      ctx.strokeStyle = COL.smoke; ctx.lineWidth = 2; ctx.beginPath();
      for (let i = 1; i < this.trail.length; i++) line(this.trail[i - 1], this.trail[i]);
      ctx.stroke();
    }

    // ---- aircraft shadow and body (scaled up when far in the pilot view so it stays readable)
    const polys = planeGeom(st);
    const scale = this.view === 'pilot' ? Math.max(1, len(sub(P, C)) / 70) : 1;
    const toW = (b: V3) => add(P, qrot(q, mul(b, scale)));
    ctx.fillStyle = COL.shadow;
    for (const pg of polys) {
      if (pg.prop) continue;
      const sp = polyC(pg.pts.map(b => { const wp = toW(b); return c2([wp[0], wp[1], 0.02]); }));
      if (sp) { path(sp); ctx.fill(); }
    }
    const faces: { cs: V3[]; depth: number; col: string; prop?: boolean }[] = [];
    for (const pg of polys) {
      const wp = pg.pts.map(toW), cs = wp.map(c2);
      if (cs.some(c => c[2] <= NEAR)) continue;
      const cen = mul(wp.reduce((a, b) => add(a, b), [0, 0, 0] as V3), 1 / wp.length);
      const facing = dot(qrot(q, pg.n), sub(C, cen)) > 0;
      faces.push({ cs, depth: cs.reduce((s, c) => s + c[2], 0) / cs.length - (pg.prop ? 0.02 : 0), col: facing ? pg.t : pg.b, prop: pg.prop });
    }
    faces.sort((a, b) => b.depth - a.depth);
    for (const fc of faces) {
      path(fc.cs.map(scr)); ctx.fillStyle = fc.col; ctx.fill();
      if (!fc.prop) { ctx.strokeStyle = COL.edge; ctx.lineWidth = 0.8; ctx.stroke(); }
    }
    trees.filter(o => o.d < planeDepth).forEach(o => drawTree(o.t));

    // ---- real-plane mode: the flight controller's own position estimate, and its error
    if (st.real) {
      const n = st.navP, pn = proj(n), pt = proj(P);
      if (pn) {
        ctx.strokeStyle = COL.nav; ctx.lineWidth = 1.5;
        if (pt) { ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(pt[0], pt[1]); ctx.lineTo(pn[0], pn[1]); ctx.stroke(); ctx.setLineDash([]); }
        ctx.beginPath(); ctx.arc(pn[0], pn[1], 6, 0, 7); ctx.moveTo(pn[0] - 10, pn[1]); ctx.lineTo(pn[0] + 10, pn[1]); ctx.moveTo(pn[0], pn[1] - 10); ctx.lineTo(pn[0], pn[1] + 10); ctx.stroke();
      }
    }

    // ---- off-screen arrow so the aircraft is never lost
    const ps = proj(P);
    if (!ps || ps[0] < 0 || ps[0] > w || ps[1] < 0 || ps[1] > h) {
      const c = c2(P); let ax = c[0], ay = -c[1]; if (c[2] < 0) { ax = -ax; ay = -ay; }
      const a = Math.atan2(ay, ax), R0 = Math.min(w, h) * 0.32;
      ctx.save(); ctx.translate(w / 2 + Math.cos(a) * R0, h / 2 + Math.sin(a) * R0); ctx.rotate(a); ctx.fillStyle = COL.padRing;
      ctx.beginPath(); ctx.moveTo(14, 0); ctx.lineTo(-6, -8); ctx.lineTo(-6, 8); ctx.closePath(); ctx.fill(); ctx.restore();
    }
  }
}
