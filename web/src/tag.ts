// The AprilTag-style marker on the pad and the tail camera that looks for it.
// Must match core/src/sensors.rs (CAM_*, TAGS): a 40 cm tag with an 8 cm tag nested in its centre.
import type { Quat, V3 } from './vec';
import { qrot, sub } from './vec';

export const CAM = { pos: [-0.70, 0, 0] as V3, w: 640, h: 480, f: 457 };

// 8 × 8 cells of 5 cm: black border, 6 × 6 code bits, and the centre 2 × 2 cells left for the small tag
const BITS = [
  '101101',
  '010011',
  '11..01',
  '01..10',
  '100111',
  '110010',
];

/** Flat polygons on the ground (x, y in m), painted in order: quiet zone, tag, code bits, nested tag. */
export function tagPolys(): { pts: [number, number][]; col: string }[] {
  const sq = (cx: number, cy: number, h: number): [number, number][] => [[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]];
  const out = [{ pts: sq(0, 0, 0.25), col: '#f4f4ef' }, { pts: sq(0, 0, 0.2), col: '#141414' }];
  const c = 0.05;
  for (let r = 0; r < 6; r++) for (let k = 0; k < 6; k++) {
    if (BITS[r][k] !== '1') continue;
    out.push({ pts: sq(-0.15 + (k + 0.5) * c, 0.15 - (r + 0.5) * c, c / 2), col: '#f4f4ef' });
  }
  // nested 8 cm tag in a white 10 cm window
  out.push({ pts: sq(0, 0, 0.05), col: '#f4f4ef' }, { pts: sq(0, 0, 0.04), col: '#141414' });
  for (const [x, y] of [[-0.01, 0.01], [0.01, -0.01], [0.01, 0.01]]) out.push({ pts: sq(x, y, 0.01), col: '#f4f4ef' });
  return out;
}

/** World point → tail-camera pixel (u right, v down) and depth, for the aircraft pose (p, q). */
export function camProject(p: V3, q: Quat, x: V3): [number, number, number] {
  const cam = qrot(q, CAM.pos); const d = sub(x, [p[0] + cam[0], p[1] + cam[1], p[2] + cam[2]]);
  const qc: Quat = [q[0], -q[1], -q[2], -q[3]];
  const b = qrot(qc, d);
  const k: V3 = [b[1], -b[2], -b[0]];
  return [CAM.w / 2 + CAM.f * k[0] / k[2], CAM.h / 2 + CAM.f * k[1] / k[2], k[2]];
}

const POLYS = tagPolys();

/** Draw what the tail camera sees: grass, a 1 m grid, the pad and the tags, plus the detector's box. */
export function drawCam(cv: HTMLCanvasElement, p: V3, q: Quat, lock: boolean, px: V3, tagId: number): void {
  const ctx = cv.getContext('2d')!;
  const dpr = Math.min(devicePixelRatio || 1, 2), W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== W * dpr) { cv.width = W * dpr; cv.height = H * dpr; }
  ctx.setTransform(dpr * W / CAM.w, 0, 0, dpr * H / CAM.h, 0, 0);
  ctx.fillStyle = '#4f7a3a'; ctx.fillRect(0, 0, CAM.w, CAM.h);
  const P = (x: number, y: number) => camProject(p, q, [x, y, 0]);
  // ground grid, 1 m, cut into short pieces so lines behind the camera drop out
  ctx.strokeStyle = 'rgba(255,255,255,0.16)'; ctx.lineWidth = 2; ctx.beginPath();
  const cx = Math.round(p[0]), cy = Math.round(p[1]);
  for (let i = -12; i <= 12; i++) for (let j = -12; j < 12; j++) {
    for (const [a, b] of [[P(cx + i, cy + j), P(cx + i, cy + j + 1)], [P(cx + j, cy + i), P(cx + j + 1, cy + i)]]) {
      if (a[2] > 0.05 && b[2] > 0.05) { ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); }
    }
  }
  ctx.stroke();
  const poly = (pts: [number, number][], col: string) => {
    const s = pts.map(([x, y]) => P(x, y));
    if (s.some(v => v[2] < 0.03)) return;
    ctx.fillStyle = col; ctx.beginPath(); s.forEach((v, i) => i ? ctx.lineTo(v[0], v[1]) : ctx.moveTo(v[0], v[1])); ctx.closePath(); ctx.fill();
  };
  // the pad
  poly(Array.from({ length: 28 }, (_, i) => [1.6 * Math.cos(i / 28 * 6.283), 1.6 * Math.sin(i / 28 * 6.283)] as [number, number]), '#d9d4c4');
  for (const t of POLYS) poly(t.pts, t.col);
  // the detector's box (from the last frame it decoded, so it lags a little)
  if (lock) {
    const h = px[2] / 2 + 6;
    ctx.strokeStyle = '#72e392'; ctx.lineWidth = 5; ctx.strokeRect(px[0] - h, px[1] - h, 2 * h, 2 * h);
    ctx.fillStyle = '#72e392'; ctx.font = '600 34px ui-monospace, monospace';
    ctx.fillText(tagId === 0 ? 'TAG 40' : 'TAG 8', Math.min(CAM.w - 130, px[0] - h), Math.max(34, px[1] - h - 10));
  }
  // centre mark
  ctx.strokeStyle = 'rgba(255,255,255,0.7)'; ctx.lineWidth = 3; ctx.beginPath();
  ctx.moveTo(CAM.w / 2 - 18, CAM.h / 2); ctx.lineTo(CAM.w / 2 + 18, CAM.h / 2); ctx.moveTo(CAM.w / 2, CAM.h / 2 - 18); ctx.lineTo(CAM.w / 2, CAM.h / 2 + 18); ctx.stroke();
}
