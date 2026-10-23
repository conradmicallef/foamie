// Small vector helpers for the renderer. World: x east, y north, z up. Body: x nose, y left wing, z canopy.
export type V3 = [number, number, number];
export type Quat = [number, number, number, number];
export const add = (a: V3, b: V3): V3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a: V3, b: V3): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const mul = (a: V3, k: number): V3 => [a[0] * k, a[1] * k, a[2] * k];
export const dot = (a: V3, b: V3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a: V3, b: V3): V3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const len = (a: V3): number => Math.hypot(a[0], a[1], a[2]);
export const norm = (a: V3): V3 => { const l = len(a) || 1; return mul(a, 1 / l); };
export const clamp = (x: number, a: number, b: number): number => Math.max(a, Math.min(b, x));
/** Unit horizontal direction plus horizontal length: [x, y, 0, length]. */
export const horiz = (a: V3): [number, number, number, number] => {
  const l = Math.hypot(a[0], a[1]);
  return l > 1e-6 ? [a[0] / l, a[1] / l, 0, l] : [0, 0, 0, 0];
};
export function qrot(q: Quat, v: V3): V3 {
  const u: V3 = [q[1], q[2], q[3]];
  const t = mul(cross(u, v), 2);
  return add(add(v, mul(t, q[0])), cross(u, t));
}
export const D2R = Math.PI / 180;
