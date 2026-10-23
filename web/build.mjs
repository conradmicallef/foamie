// Build: Rust core → WebAssembly → bundle TypeScript → one self-contained dist/index.html.
//   node build.mjs            full build (needs cargo and the wasm32-unknown-unknown target)
//   node build.mjs --web-only reuse gen/ (wasm already built)
import { execSync } from 'node:child_process';
import { mkdirSync, readFileSync, writeFileSync, existsSync } from 'node:fs';
import { build } from 'esbuild';

const sh = (cmd, env = {}) => execSync(cmd, { stdio: 'inherit', env: { ...process.env, ...env } });
const webOnly = process.argv.includes('--web-only');
mkdirSync('gen', { recursive: true });
mkdirSync('dist', { recursive: true });

if (!webOnly) {
  const target = '../core/target/wasm32-unknown-unknown/release/foamie_core.wasm';
  // MVP target: widest browser support
  const flags = { RUSTFLAGS: '-C target-cpu=mvp' };
  const installed = execSync('rustup target list --installed 2>/dev/null || true').toString().includes('wasm32-unknown-unknown');
  if (installed) {
    sh('cargo build --release --target wasm32-unknown-unknown --manifest-path ../core/Cargo.toml', flags);
  } else {
    // no prebuilt wasm32 std: build `core` from rust-src instead (the crate is no_std)
    console.log('wasm32 target not installed; building core from source (-Zbuild-std)');
    sh('cargo build --release --target wasm32-unknown-unknown --manifest-path ../core/Cargo.toml -Zbuild-std=core,compiler_builtins -Zbuild-std-features=compiler-builtins-mem',
      { ...flags, RUSTC_BOOTSTRAP: '1' });
  }
  sh(`npx wasm-opt -O3 ${target} -o gen/foamie_core.wasm`);
}
if (!existsSync('gen/foamie_core.wasm')) throw new Error('gen/foamie_core.wasm missing: run a full build first');

const out = await build({
  entryPoints: ['src/main.ts'], bundle: true, write: false, format: 'iife', minify: true,
  target: 'es2020', loader: { '.wasm': 'binary' }, legalComments: 'none',
});
const js = out.outputFiles[0].text.replace(/<\/script/gi, '<\\/script');
const html = readFileSync('src/index.html', 'utf8').replace('<script>/*BUNDLE*/</script>', () => `<script>${js}</script>`);
// standalone page (GitHub Pages): title, fonts and styles go in <head>, plus the base rules an artifact host would add
const cut = html.indexOf('</style>') + '</style>'.length;
const page = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<style>body{margin:0}[hidden]{display:none!important}</style>
${html.slice(0, cut).replace(/<meta name="claude:[^>]*>\n?/, '')}
</head>
<body>
${html.slice(cut).trim()}
</body>
</html>
`;
writeFileSync('dist/index.html', page);
// the same page without the document wrapper, for hosts that add their own (the claude.ai artifact)
writeFileSync('dist/embed.html', html);
console.log(`dist/index.html  ${(page.length / 1024).toFixed(0)} KB`);
