declare module '*.wasm' { const bytes: Uint8Array; export default bytes; }
declare module '*fallback.js' {
  export const memory: { buffer: ArrayBuffer };
  export function fz_init(seed: number): void;
}
