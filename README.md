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
- **Aerobatics (Show button):** a 12-figure show — loop, axial roll, knife-edge pass, Immelmann, four-point roll, rolling circle, split-S, hammerhead, harrier, Cuban eight, waterfall and a prop-hang torque roll — or any single figure (`core/src/aerobatics.rs`).
  - **Flown on a path:** each figure is real geometry — lines, circles, rolls placed on lines, a horizontal rolling circle, a stall-turn pivot (quarter-rolled so it stays in the figure plane), harrier slow flight, a tumble, a prop hang. The guidance works out the acceleration needed to stay on it (curvature, cross-track error, gravity) and points the nose so lift and fuselage side force make it, so knife-edge gets top rudder and inverted gets push automatically.
  - **Centre stage, into wind:** every figure is centred on the show centre 46 m in front of the pilot and entered into the wind.
  - **Flying the wind:** each figure drifts with the along-line mean wind, so the plane flies a clean shape relative to the air; the planned path is walked in time with that drift and the drifted shape is centred. In 4 m/s gusts (5 seeds) this took ideal-mode figures completed from 35/40 to 40/40 with no recoveries.
  - **Repositioning:** before each figure the plane flies out past the entry gate, turns back onto the show line, and only starts when it crosses the gate on line, height, heading and speed; otherwise it goes round again.
  - **Safety:** recovery stays armed. It trusts a figure while the plane is on its planned path, and if it does step in, the show carries on with the next figure.
- **Recovery:** predicts ground impact up to 10 s ahead along the curved flight path and pulls out automatically when the time left is shorter than this airframe needs to stop the sink.
- **Weathervane:** in QHover/QLoiter the plane turns its wing edge-on to the wind when you leave the yaw stick centred (like ArduPilot's `Q_WVANE_GAIN`); broadside, the wing catches too much wind to hold position. In real-plane mode the wind comes from the EKF's hover drag-fusion estimate (like `EK3_DRAG_*`).
- **Real plane:** flies the same software on simulated sensors instead of perfect knowledge, and lands on an AprilTag with a tail camera — see below.

Stick convention is a real transmitter's (mode 2): right stick forward = nose down in the wing-borne modes, and tilt toward the belly in the hover modes. On a computer: W/S throttle, A/D yaw, arrow keys pitch and roll.

## Real-plane mode

| Part | What is simulated |
|---|---|
| IMU (gyro + accelerometer) | white noise, bias random walk, motor vibration growing with throttle |
| Compass | interference from motor current |
| Barometer | 25 Hz, noise, slow drift, prop-wash pressure error |
| GPS | 5 Hz, 0.16 s latency, wandering ~1 m position error |
| Tail camera (**Tail cam** button) | looks out of the tail — straight down in the hover. 20 Hz, 640 × 480, 70° field of view, 80 ms latency, 0.7 px corner noise, a slightly crooked mount, frames lost to motion blur above 2.5 rad/s and 4 % at random. Decodes the AprilTag on the pad: a 40 cm tag (to about 9 m) with an 8 cm tag nested in its centre for the last metre. No lidar. |
| Wingtip pitot | forward airspeed only, unusable below ~3 m/s |
| Airframe | ±5 % thrust, ±8 % surface effectiveness, CG offset, warped wing, slower servos and motor |
| Battery | 3S 1000 mAh, voltage sag under load, thrust ∝ voltage² |

The flight controller sees only an estimate built from those sensors, by an 18-state **error-state EKF** like ArduPilot's (`core/src/ekf.rs`): position, velocity, attitude, gyro bias, accelerometer bias and the GPS's slowly wandering position offset. It predicts on the IMU; fuses delayed GPS position and velocity against the state at the time of the fix, plus barometer and compass (with compass-motor compensation, like `COMPASS_MOT`); applies zero-velocity and ground-height updates while standing on the legs; and gates every measurement (χ²) so glitches are rejected. Because GPS velocity and the accelerometer are tied through the attitude, tilt stays observable during manoeuvres without trusting the accelerometer as a gravity reference. A land detector (estimated height, or motor near idle while the IMU feels 1 g and no rotation) and a tipped-over motor cut stop the hover throttle from fighting the ground. The cyan cross in the view is where the controller thinks it is.

**Precision landing.** Each tag detection says where the pad is relative to the aircraft, to a few centimetres, delayed by the camera's latency. The EKF fuses it against the state when the frame was taken, as a position measurement, taking its own attitude as known — the way ArduPilot's precision landing does. Letting the tag also correct the attitude looked attractive but, with a 9 m lever arm, made the filter overconfident: in a long hover the tilt estimate crept 10–20° off. Because the GPS offset is a state of its own, every sighting teaches the EKF the GPS error, so GPS stays accurate for a while after the tag drops out of view. Below about 10 cm the tag is too close to read; height is then held by the IMU and barometer until the feet touch. While the tail points down, a small picture-in-picture shows what the camera sees, with the detector's box.

Estimate error vs truth (`cargo run --release --example est_stats [old]`, hover in wind + circuit + Cuban eight):

| | Earlier complementary filter + lidar | EKF + lidar | EKF + tail camera, no lidar |
|---|---|---|---|
| Attitude RMS / worst | 2.8° / 13.7° | 0.9° / 2.7° | **0.9° / 2.8°** |
| Velocity RMS | 0.23 m/s | 0.14 m/s | **0.13 m/s** |
| Position RMS | 0.90 m | 0.89 m (the GPS's own wander) | **0.42 m** (0.1–0.2 m while the tag is in view) |
| Height RMS | 0.14 m | 0.15 m | 0.27 m (no lidar; tag and baro) |

### Results (`cargo run --release --example bench [real]`)

`bench real nocam` turns the tail camera off.

| | Ideal | Real plane, tail cam | Real plane, GPS only | (before: GPS + lidar) |
|---|---|---|---|---|
| Autoland: tail landings | 18 / 18 | 54 / 54 | 54 / 54 | 54 / 54 |
| Autoland: mean time from pressing Land | 17.0 s | 17.6 s | 18.4 s | 17.6 s |
| Autoland: mean distance from pad centre | 0.20 m | **0.20 m** | 0.47 m | 0.84 m |
| QLoiter in 2–4 m/s gusts: mean drift | 0.22 m | 0.71 m | 0.65 m | 0.73 m |
| Recovery: full-down dive, inverted motor-off | saved | saved | saved | saved |
| Recovery false alarms: split-S, FBWA cruise, hover landing | none | none | none | none |
| Aerobatic show in calm air: 12 figures, no recovery, centred within | 2 m | 3 m | | |

With the camera the real plane lands as accurately as the ideal model with perfect knowledge; what is left is the landing controller's own scatter. Modelling the GPS offset as its own EKF state helped even without the camera (0.84 → 0.47 m). QLoiter drift is measured from where the plane started holding; with the tag in view it is held over the true spot, so corrections as the tag pins down the GPS error count as drift here.

Aerobatics (`cargo run --release --example show [real] [wind]`, `show_stats [real]`): path error about 1–5.6 m RMS per figure in calm air, every figure entered into wind. In 4 m/s gusts the ideal-mode show completes with no recoveries; real-plane mode occasionally abandons a figure that strays off its path low down and carries on with the next.

On a real airframe the camera side is an OpenMV H7 or similar running AprilTag detection and sending the tag's position to the flight controller over a serial link (ArduPilot: `PLND_TYPE` 1 with MAVLink `LANDING_TARGET`); the tag is printed on the landing mat.

## Flying real ArduPilot on the foamie (SITL)

`sitl/` is an ArduPilot SITL **JSON physics backend**: real ArduPlane firmware flies the same airframe, aero, prop-wash and ground-contact model as the browser sim. ArduPilot sends servo PWM over UDP each physics frame (lock-step); `foamie-sitl` steps `foamie_core::model::Truth` and replies with IMU, position, velocity and attitude, converting the core's ENU/FLU frames to ArduPilot's NED/FRD. ArduPilot then adds its own simulated GPS, baro, compass and airspeed and runs its own EKF3.

`sitl/foamie.parm` sets ArduPlane up as a single-motor tail-sitter (`Q_ENABLE 1`, `Q_FRAME_CLASS 10`, `Q_TAILSIT_ENABLE 1`, `Q_M_THST_HOVER 0.55`); outputs are ArduPlane's defaults — SERVO1 aileron, SERVO2 elevator, SERVO3 throttle, SERVO4 rudder.

```sh
git clone https://github.com/ArduPilot/ardupilot ~/ardupilot && cd ~/ardupilot
git submodule update --init --recursive
./waf configure --board sitl && ./waf plane
pip install pymavlink
cd foamie/sitl && ./run.sh --stage all     # WIND=4 for gusts, SPEEDUP=4 by default
```

`run.sh` starts the bridge and `arduplane --model JSON:127.0.0.1 --defaults foamie.parm`, then `fly.py` flies over MAVLink (TCP 5760 — a ground station such as Mission Planner or QGroundControl can connect there instead). With stock ArduPilot gains (master, October 2026):

| Flight | Result |
|---|---|
| QLOITER climb to 10 m, 20 s hands-off hover, QLAND, calm | held within 4 cm; landed 0.11 m from the take-off point; disarmed itself |
| Same in 4 m/s gusts | about 2 m RMS wander (up to 5 m) — gains not tuned for this airframe; the browser sim's own controller with weathervaning holds 0.2–0.8 m |
| AUTO: VTOL takeoff to 15 m, transition, 160 × 80 m fixed-wing circuit at 25 m and 14 m/s, VTOL landing | completed hands-off in 132 s; landed 0.24 m from home; balloons about 7 m during the back-transition, as tail-sitters do |

## Layout

```
core/                 Rust flight core (no_std, compiles to wasm32)
  src/model.rs        airframe, flat-plate aero ±180°, prop wash, gyroscopics, battery, wind, ground contact
  src/sensors.rs      sensor models for real-plane mode, including the tail camera and the AprilTag
  src/ekf.rs          18-state error-state EKF with AprilTag precision landing (real-plane mode)
  src/estimator.rs    the navigation state type; the earlier complementary filter, kept for comparison
  src/flight.rs       rate & attitude loops, the six modes, guidance, impact prediction
  src/sim.rs          the simulation loop, autopilot tasks, touchdown scoring, events
  src/lib.rs          C-ABI exports and the state buffer read by the front end
  src/aerobatics.rs   path-tracked aerobatic figures, positioning and the show sequence
  tests/scenarios.rs  flight scenarios that must keep working
  examples/bench.rs   the benchmark above; examples/show.rs each figure; examples/est_stats.rs estimator accuracy
sitl/                 ArduPilot SITL JSON backend on the same physics, tail-sitter params, test flight
web/                  TypeScript front end
  src/core.ts         loads the WebAssembly core and reads its state and events
  src/render.ts       3D view on a 2D canvas
  src/tag.ts          the AprilTag on the pad and the tail camera's picture-in-picture
  src/main.ts         sticks, buttons, instruments, messages
  build.mjs           builds everything into one self-contained web/dist/index.html
```

## Build

```sh
cd core && cargo test --release            # flight scenarios
rustup target add wasm32-unknown-unknown
cd web && npm ci && npm run build          # → web/dist/index.html (≈170 KB, no server needed)
```

`build.mjs` compiles the core for the baseline (MVP) WebAssembly target, optimises it with binaryen's `wasm-opt` and embeds it in the page. Without a prebuilt `wasm32` standard library it builds `core` from `rust-src` (`-Zbuild-std`).

Pushing to `main` runs `.github/workflows/pages.yml`, which tests the core, builds the page and publishes it to the `gh-pages` branch. In the repository's **Settings → Pages**, set the source to the `gh-pages` branch.

## Licence

GPL-3.0-or-later — see [LICENSE](LICENSE).
