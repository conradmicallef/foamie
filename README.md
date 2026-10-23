# Foamie

**[▶ View in browser](https://conradmicallef.github.io/foamie/)** — runs on a phone; use two thumbs.

A mobile flight simulator for a high-powered 3D foam plane that takes off and lands vertically on X-shaped tail legs (a tail-sitter). The flight core — physics, sensors, state estimation, flight modes, autopilot and crash recovery — is written in Rust and compiled to WebAssembly. The front end — 3D view, touch sticks, instruments and messages — is TypeScript.

## What you can do

- **Fly it yourself** with ArduPilot-style modes:
  - **Manual** – sticks move the surfaces directly.
  - **FBWA** – stick sets bank (±50°) and pitch angle; wings level hands-off; from a hover it pitches over into normal flight.
  - **Acro** – stick sets roll/pitch/yaw rates, no limits. For aerobatics.
  - **QStabilize** – self-levels nose-up; stick tilts; raw throttle.
  - **QHover** – QStabilize plus height hold (left stick = climb rate).
  - **QLoiter** – position hold over the ground; stick moves the hover.
- **Autopilot buttons:** **Takeoff** (vertical climb, pitch-over, joins the circuit), **Circuit** (racetrack round the pad at 20 m), **Land** (straight in, pull vertical to brake, descend onto the legs). Moving a stick takes over.
- **Aerobatics (Show button):** the full show — loop, Immelmann, axial roll, split-S, four-point roll, hammerhead, Cuban eight and a prop-hang torque roll — or any single figure. Each figure is a short program of pulls, rolls and pivots: a reference attitude turns at fixed body rates and the quaternion attitude controller follows it, so the geometry holds inverted and vertical too. The autopilot climbs to the figure's safe height first, recovery stays armed, and it rejoins the circuit afterwards (`core/src/aerobatics.rs`).
- **Recovery:** predicts ground impact up to 10 s ahead along the curved flight path and pulls out automatically when the time left is shorter than this airframe needs to stop the sink.
- **Weathervane:** in QHover/QLoiter the plane turns its wing edge-on to the wind when you leave the yaw stick centred (like ArduPilot's `Q_WVANE_GAIN`); broadside, the wing catches too much wind to hold position.
- **Real plane:** flies the same software on simulated sensors instead of perfect knowledge — see below.

Stick convention is a real transmitter's (mode 2): right stick forward = nose down in the wing-borne modes, and tilt toward the belly in the hover modes. On a computer: W/S throttle, A/D yaw, arrow keys pitch and roll.

## Real-plane mode

| Part | What is simulated |
|---|---|
| IMU (gyro + accelerometer) | white noise, bias random walk, motor vibration growing with throttle |
| Compass | interference from motor current |
| Barometer | 25 Hz, noise, slow drift, prop-wash pressure error |
| GPS | 5 Hz, 0.16 s latency, wandering ~1 m position error |
| Tail lidar | looks down in the hover, 12 m range, reports minimum range when compressed |
| Wingtip pitot | forward airspeed only, unusable below ~3 m/s |
| Airframe | ±5 % thrust, ±8 % surface effectiveness, CG offset, warped wing, slower servos and motor |
| Battery | 3S 1000 mAh, voltage sag under load, thrust ∝ voltage² |

The flight controller sees only an estimate built from those sensors (`core/src/estimator.rs`): a DCM-style attitude filter with **velocity-aided tilt correction** (GPS velocity change vs. integrated accelerometer — without it, hover manoeuvres fool the gravity reference by up to 25°), dead reckoning corrected by delayed GPS, and baro/lidar height. A land detector and a tipped-over motor cut stop the hover throttle from fighting the ground. The cyan cross in the view is where the controller thinks it is.

### Results (`cargo run --release --example bench [real]`)

| | Ideal | Real plane |
|---|---|---|
| Autoland: tail landings | 18 / 18 | 53 / 54 |
| Autoland: mean time from pressing Land | 19.2 s | 25.6 s |
| Autoland: mean distance from pad centre | 0.16 m | 1.0 m (GPS-limited) |
| QLoiter in 2–4 m/s gusts: mean drift | 0.33 m | 1.1 m |
| Recovery: full-down dive, inverted motor-off | saved | saved |
| Recovery false alarms: split-S, FBWA cruise, hover landing | none | none |
| Aerobatic show: 8 figures, no recovery needed | ✓ | ✓ |

For centimetre landings on a real airframe you would add RTK GPS, optical flow, or a precision-landing beacon.

## Layout

```
core/                 Rust flight core (no_std, compiles to wasm32)
  src/model.rs        airframe, flat-plate aero ±180°, prop wash, gyroscopics, battery, wind, ground contact
  src/sensors.rs      sensor models for real-plane mode
  src/estimator.rs    attitude / velocity / position / height estimation
  src/flight.rs       rate & attitude loops, the six modes, guidance, impact prediction
  src/sim.rs          the simulation loop, autopilot tasks, touchdown scoring, events
  src/lib.rs          C-ABI exports and the state buffer read by the front end
  tests/scenarios.rs  flight scenarios that must keep working
  src/aerobatics.rs   aerobatic figures and the show sequence
  examples/bench.rs   the benchmark above; examples/show.rs each figure; examples/diag.rs estimator accuracy
web/                  TypeScript front end
  src/core.ts         loads the core (WebAssembly, or its wasm2js build if a page forbids WebAssembly)
  src/render.ts       3D view on a 2D canvas
  src/main.ts         sticks, buttons, instruments, messages
  build.mjs           builds everything into one self-contained web/dist/index.html
```

## Build

```sh
cd core && cargo test --release            # flight scenarios
rustup target add wasm32-unknown-unknown
cd web && npm ci && npm run build          # → web/dist/index.html (≈230 KB, no server needed)
```

`build.mjs` compiles the core for the baseline (MVP) WebAssembly target and also translates it to plain JavaScript with binaryen's `wasm2js`; the page tries WebAssembly first and falls back to the JS build, so it also runs where WebAssembly is blocked. Without a prebuilt `wasm32` standard library it builds `core` from `rust-src` (`-Zbuild-std`).

Pushing to `main` runs `.github/workflows/pages.yml`, which tests the core, builds the page and publishes it to the `gh-pages` branch. In the repository's **Settings → Pages**, set the source to the `gh-pages` branch.

## Licence

GPL-3.0-or-later — see [LICENSE](LICENSE).
