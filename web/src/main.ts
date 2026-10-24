// Front end: touch sticks, buttons, instrument overlay and messages. All flight logic runs in the Rust core.
import { drawCam } from './tag';
import { Cmd, Core, Ev, Mode, Opt, State, Task, type Event } from './core';
import { Renderer } from './render';
import { add, clamp, D2R, qrot, type V3 } from './vec';

const $ = (id: string) => document.getElementById(id)!;
const cl1 = (x: number) => clamp(x, -1, 1);

const MODE_NAME: Record<Mode, string> = { [Mode.Manual]: 'Manual', [Mode.Fbwa]: 'FBWA', [Mode.Acro]: 'Acro', [Mode.QStab]: 'QStabilize', [Mode.QHover]: 'QHover', [Mode.QLoiter]: 'QLoiter' };
const MODE_ATTR: Record<string, Mode> = { MANUAL: Mode.Manual, FBWA: Mode.Fbwa, ACRO: Mode.Acro, QSTAB: Mode.QStab, QHOVER: Mode.QHover, QLOITER: Mode.QLoiter };
const TASK_NAME: Record<number, string> = { [Task.Takeoff]: 'takeoff', [Task.Circuit]: 'circuit', [Task.Land]: 'land', [Task.Recover]: 'recovery', [Task.Show]: 'show' };
const FIG: Record<number, [string, string]> = {
  1: ['Loop', 'A true 18 m circle, tracked against the ground'],
  2: ['Axial roll', 'One roll on a straight line, top rudder through knife-edge'],
  3: ['4-point roll', 'Roll in four 90° steps with a crisp pause at each'],
  4: ['Immelmann', 'Half loop up, half roll on top: reverses and climbs'],
  5: ['Split-S', 'Half roll inverted, half loop down: reverses and descends'],
  6: ['Hammerhead', 'Straight up until nearly stopped, rudder pivot, straight down, pull out'],
  7: ['Cuban eight', 'Five-eighths loop, half roll on the 45° down-line, and again'],
  8: ['Torque roll', 'Up into a prop hang, spinning on the prop, then pitch out'],
  9: ['Knife-edge pass', 'Quarter roll, a long line on the wingtip held up with top rudder, quarter roll back'],
  10: ['Rolling circle', 'A full horizontal circle away from you while rolling twice'],
  11: ['Harrier', 'Slow and nose-high on the prop, about 4.5 m/s, height held on throttle'],
  12: ['Waterfall', 'Slow into a harrier, tumble nose-down through 360° on full power, fly away'],
};
const PHASE_TXT: Record<number, string> = {
  1: 'climbing vertically to 8 m', 2: 'pitching over to wing-borne flight', 3: 'flying the circuit', 4: 'straight in to the pad',
  5: 'pulling up to vertical', 6: 'pulling vertical and braking onto the pad', 7: 'descending onto the legs',
  8: 'pulling out of the dive', 9: 'nose up, climbing on thrust', 10: 'safe; centre the sticks to take over', 11: '',
};
const KIND = ['Landing legs', 'Nose', 'Canopy', 'Belly', 'Wingtip'];
const HINTS: Record<string, string> = {
  [Mode.Manual]: 'Manual: sticks drive the control surfaces directly, no gyro. Right stick forward is nose down, as on a real transmitter. Left stick is throttle (stays put) and rudder.',
  [Mode.Fbwa]: 'FBWA: right stick sets bank (±50°) and pitch angle (forward = nose down); let go and it levels the wings. From a hover it pitches over along the belly.',
  [Mode.Acro]: 'Acro: sticks set roll, pitch and yaw rates with no angle limits (forward = nose down); centred holds the attitude. For aerobatics.',
  [Mode.QStab]: 'QStabilize: self-levels to nose-up. Right stick tilts it (forward = toward the belly), left stick is raw throttle, sideways spins.',
  [Mode.QHover]: 'QHover: like QStabilize, but the left stick sets climb or sink rate from centre, so height holds hands-off. Weathervanes edge-on to the wind.',
  [Mode.QLoiter]: 'QLoiter: right stick moves the hover over the ground, centred holds the spot. Left stick climbs or sinks from centre. Motor cuts when the legs touch.',
  takeoff: 'Auto takeoff: vertical climb to 8 m, pitch over along the belly, then join the circuit.',
  circuit: 'Circuit: anticlockwise racetrack round the pad at 20 m and 13 m/s. Move any stick to take over.',
  land: 'Auto land: straight in at 9 m, pull vertical at the braking point, brake onto the pad, descend fast then 0.45 m/s for the last metre.',
  show: 'Aerobatics: each figure is a real path (lines, circles, rolls on lines) centred on the show centre in front of you and entered into the wind. Before each one the plane flies out, turns back and lines up on the entry gate. Recovery stays armed. Move a stick to take over.',
  recovery: 'Recovery: ground impact was predicted sooner than the plane can stop the sink. Pulling out at full power; control returns once it is safe and the sticks are centred.',
  real: 'Real plane: the autopilot now flies on simulated sensors (noisy, delayed GPS, drifting baro, vibrating IMU, pitot, and a tail camera that looks for the AprilTag on the pad) through an EKF. The cyan cross is where it thinks it is. This airframe also has small build errors and a 3S 1000 mAh battery.',
  cam: 'Tail cam: in the hover the camera looks straight down past the legs. When it decodes the 40 cm tag on the pad (or the 8 cm one nested in it, for the last metre) the EKF learns the GPS error and lands on the tag. Switch it off to land on GPS alone.',
};

// ---- sticks
interface Stick { x: number; y: number; active: boolean; id: number | null; el: HTMLElement }
const sticks: Record<'L' | 'R', Stick> = {
  L: { x: 0, y: 1, active: false, id: null, el: $('stL') },
  R: { x: 0, y: 0, active: false, id: null, el: $('stR') },
};
function paintSticks() {
  for (const k of ['L', 'R'] as const) {
    const s = sticks[k], r = s.el.getBoundingClientRect().width * 0.39;
    (s.el.querySelector('.knob') as HTMLElement).style.transform = `translate(${s.x * r}px, ${s.y * r}px)`;
  }
}
function bindStick(key: 'L' | 'R') {
  const s = sticks[key], el = s.el;
  const upd = (e: PointerEvent) => {
    const r = el.getBoundingClientRect(), rad = r.width * 0.39;
    s.x = cl1((e.clientX - (r.left + r.width / 2)) / rad);
    s.y = cl1((e.clientY - (r.top + r.height / 2)) / rad);
    paintSticks();
  };
  el.addEventListener('pointerdown', e => { e.preventDefault(); s.active = true; s.id = e.pointerId; el.setPointerCapture(e.pointerId); upd(e); wake(); });
  el.addEventListener('pointermove', e => { if (s.active && e.pointerId === s.id) upd(e); });
  const end = (e: PointerEvent) => {
    if (e.pointerId !== s.id) return;
    s.active = false; s.id = null; s.x = 0;
    if (key === 'R') s.y = 0; // the left stick keeps its throttle position
    paintSticks();
  };
  el.addEventListener('pointerup', end); el.addEventListener('pointercancel', end);
}
bindStick('L'); bindStick('R');

// keyboard: W/S throttle, A/D yaw, arrows pitch (up = nose down, like the stick) and roll
const keys: Record<string, boolean> = {};
addEventListener('keydown', e => { keys[e.key.toLowerCase()] = true; if (e.key.startsWith('Arrow')) e.preventDefault(); });
addEventListener('keyup', e => { keys[e.key.toLowerCase()] = false; });
function applyKeys(ft: number) {
  const L = sticks.L, R = sticks.R;
  let any = false;
  if (keys['w']) { L.y = cl1(L.y - ft * 0.9); any = true; }
  if (keys['s']) { L.y = cl1(L.y + ft * 0.9); any = true; }
  if (!L.active && (keys['a'] || keys['d'] || L.x !== 0)) { L.x = keys['a'] ? -1 : keys['d'] ? 1 : 0; any = true; }
  if (!R.active && (keys['arrowleft'] || keys['arrowright'] || keys['arrowup'] || keys['arrowdown'] || R.x !== 0 || R.y !== 0)) {
    R.x = keys['arrowleft'] ? -1 : keys['arrowright'] ? 1 : 0;
    R.y = keys['arrowup'] ? -1 : keys['arrowdown'] ? 1 : 0;
    any = true;
  }
  if (any) paintSticks();
}

let wakeLock: { release(): Promise<void>; addEventListener(t: string, f: () => void): void } | null = null;
function wake() {
  const nav = navigator as Navigator & { wakeLock?: { request(t: string): Promise<typeof wakeLock> } };
  if (wakeLock || !nav.wakeLock) return;
  nav.wakeLock.request('screen').then(l => { wakeLock = l; l?.addEventListener('release', () => { wakeLock = null; }); }).catch(() => {});
}

// ---- messages
let toastTimer = 0, hintTimer = 0;
function toast(h: string, p: string, cls = '', ms = 2000) {
  const el = $('toast'); el.hidden = false; el.className = 'panel ' + cls;
  $('toastH').textContent = h; $('toastP').textContent = p;
  clearTimeout(toastTimer); if (ms) toastTimer = window.setTimeout(() => { el.hidden = true; }, ms);
}
function showHint(key: string) {
  const el = $('hint');
  el.textContent = HINTS[key] ?? '';
  el.classList.remove('fade'); clearTimeout(hintTimer);
  hintTimer = window.setTimeout(() => el.classList.add('fade'), 8000);
}

function onEvent(e: Event) {
  switch (e.code) {
    case Ev.TakeoffDone: toast('Transition complete', 'Flying the circuit. Move a stick to take over.', 'ok', 2600); showHint('circuit'); break;
    case Ev.YouHaveControl: toast('You have control', `${MODE_NAME[e.a as Mode]} mode`, '', 1400); break;
    case Ev.TailLanding: toast(`Tail landing · ${e.a.toFixed(0)}`, `sink ${e.b.toFixed(2)} m/s · drift ${e.c.toFixed(2)} m/s · ${e.d.toFixed(1)} m from pad`, 'ok', 4200); break;
    case Ev.HardTail: toast('Hard tail landing', `sink ${e.a.toFixed(1)} m/s · tilt ${e.b.toFixed(0)}°`, '', 4200); break;
    case Ev.Belly: toast('Belly landing', `touchdown ${e.a.toFixed(1)} m/s`, '', 4200); break;
    case Ev.Touched: toast('Touched down', `${KIND[e.a] ?? 'Airframe'} contact at ${e.b.toFixed(1)} m/s`, '', 4200); break;
    case Ev.Crash: toast('Crashed', `${KIND[e.a] ?? 'Airframe'} hit at ${e.b.toFixed(1)} m/s. Tap Reset.`, 'bad', 0); break;
    case Ev.Recovery: toast('Recovery', `Ground impact predicted in ${e.a.toFixed(1)} s at ${e.b.toFixed(1)} m/s`, 'bad', 2200); showHint('recovery'); break;
    case Ev.PulledOut: toast('Pulled out', 'Centre the sticks to take control back', 'ok', 1800); break;
    case Ev.Recovered: toast('Recovered', `${MODE_NAME[e.a as Mode]} engaged · ${e.b.toFixed(1)} m above ground`, 'ok', 2000); break;
    case Ev.StickLy: if (!sticks.L.active) { sticks.L.y = e.a; paintSticks(); } break;
    case Ev.ModeChanged: showHint(String(e.a)); break;
    case Ev.BatteryLow: toast('Battery low', `${e.a.toFixed(1)} V under load. Land now: hovering takes a lot of power.`, 'bad', 4000); break;
    case Ev.FigureStarted: { const [n, d] = FIG[e.a] ?? ['Figure', '']; toast(e.c > 1 ? `${n} · ${e.b}/${e.c}` : n, d, '', 2600); break; }
    case Ev.FigureDone: toast(`${FIG[e.a]?.[0] ?? 'Figure'} done`, `on the path within ${e.b.toFixed(1)} m RMS · worst ${e.c.toFixed(1)} m`, 'ok', 2400); break;
    case Ev.ShowDone: toast('Show complete', `last figure within ${e.b.toFixed(1)} m RMS · rejoining the circuit at ${e.d.toFixed(0)} m`, 'ok', 3200); break;
    case Ev.BatteryEmpty: toast('Battery empty', 'Thrust is fading. It will not hover much longer.', 'bad', 4000); break;
  }
}

// ---- instrument overlay
function setTile(id: string, key: string, val: string, unit: string, cls = '') {
  const el = $(id); el.className = 'tile' + (cls ? ' ' + cls : '');
  (el.querySelector('.k') as HTMLElement).textContent = key;
  (el.querySelector('.v') as HTMLElement).innerHTML = val + (unit ? `<small>${unit}</small>` : '');
}
let hudT = 0;
function hud(st: State, now: number) {
  ($('thrFill') as HTMLElement).style.height = (st.thr * 100).toFixed(0) + '%';
  if (now - hudT < 100) return; hudT = now;
  const X = qrot(st.q, [1, 0, 0]);
  const nose = Math.asin(clamp(X[2], -1, 1)) / D2R;
  const tw = st.thrustToWeight;
  setTile('mAlt', 'Alt', st.p[2].toFixed(1), 'm');
  setTile('mAir', 'Airspeed', st.airspeed.toFixed(1), 'm/s');
  setTile('mVs', 'V/S', (st.v[2] >= 0 ? '+' : '') + st.v[2].toFixed(1), 'm/s', st.v[2] < -1.5 && st.p[2] < 4 ? 'warn' : '');
  setTile('mNose', 'Nose', nose.toFixed(0), '°', nose > 80 ? 'hot' : '');
  setTile('mThr', 'Throttle', (st.thr * 100).toFixed(0), '%');
  setTile('mTw', 'Thrust/W', tw.toFixed(2), '', tw > 0.95 && tw < 1.05 ? 'hot' : '');
  if (st.real) {
    // real-plane mode: show the estimator's position error and the battery instead of AoA and wash
    setTile('mAoa', 'Est. error', st.estErrH.toFixed(1), 'm', st.estErrH > 2 ? 'warn' : '');
    const cell = st.battVolts / 3;
    setTile('mWash', 'Battery', st.battVolts.toFixed(1), `V ${(st.battSoc * 100).toFixed(0)}%`, cell < 3.55 ? 'warn' : '');
  } else {
    setTile('mAoa', 'AoA', (st.aoa / D2R).toFixed(0), '°', Math.abs(st.aoa) > 0.3 && st.airspeed > 2 ? 'warn' : '');
    setTile('mWash', 'Prop wash', st.washSpeed.toFixed(1), 'm/s');
  }
  let txt: string, k: string;
  if (st.crashed) { txt = 'Crashed'; k = 'bad'; }
  else if (st.task === Task.Show) {
    const name = FIG[st.figure]?.[0] ?? 'figure', of = st.showLength > 1 ? ` (${st.figureIndex + 1}/${st.showLength})` : '';
    txt = st.showStage === 2 ? `Aerobatics · ${name}${of}` : `Aerobatics · positioning for ${name}${of}, into wind`;
    k = 'auto';
  }
  else if (st.task !== Task.None) { txt = `Auto ${TASK_NAME[st.task]} · ${PHASE_TXT[st.phase] ?? ''}`; k = 'auto'; }
  else if (st.onGround && !st.airborne) { txt = `${MODE_NAME[st.mode]} · on the legs${Math.hypot(st.p[0], st.p[1]) < 1.5 ? ', on the pad' : ''}`; k = 'ok'; }
  else { txt = `${MODE_NAME[st.mode]} · ${X[2] > 0.8 ? 'hovering' : 'wing-borne'}`; k = ''; }
  const wl = [0, 2, 4][st.windLevel];
  if (st.real && st.camOn && st.tagLock) txt += ' · tag locked';
  if (wl) txt += ` · wind ${wl} m/s from E`;
  $('statusTxt').textContent = txt; ($('status') as HTMLElement).dataset.k = k;

  const wn = $('warn');
  const showW = st.airborne && !st.crashed && st.impactT < 10 && st.impactV > 1.8 && !st.deliberateLanding;
  wn.hidden = !showW;
  if (showW) {
    wn.className = st.impactT < st.timeToRecover + 1.5 ? '' : 'caution';
    $('warnTxt').textContent = `GROUND IN ${st.impactT.toFixed(1)} s · ${st.impactV.toFixed(0)} m/s${st.recoveryOn ? '' : ' · recovery off'}`;
  }

  // buttons and stick labels follow the core
  for (const b of document.querySelectorAll<HTMLElement>('[data-mode]')) b.setAttribute('aria-pressed', String(st.task === Task.None && MODE_ATTR[b.dataset.mode!] === st.mode));
  $('btTakeoff').setAttribute('aria-pressed', String(st.task === Task.Takeoff));
  $('btCircuit').setAttribute('aria-pressed', String(st.task === Task.Circuit));
  $('btLand').setAttribute('aria-pressed', String(st.task === Task.Land));
  $('btShow').setAttribute('aria-pressed', String(st.task === Task.Show));
  $('btRecov').setAttribute('aria-pressed', String(st.recoveryOn));
  $('btVane').setAttribute('aria-pressed', String(st.weathervaneOn));
  $('btReal').setAttribute('aria-pressed', String(st.real));
  ($('btCam') as HTMLButtonElement).hidden = !st.real;
  $('btCam').setAttribute('aria-pressed', String(st.camOn));
  const L: Record<Mode, string> = { [Mode.Manual]: 'Throttle · Rudder', [Mode.Fbwa]: 'Throttle · Rudder', [Mode.Acro]: 'Throttle · Yaw', [Mode.QStab]: 'Throttle · Spin', [Mode.QHover]: 'Climb · Spin', [Mode.QLoiter]: 'Climb · Spin' };
  const R: Record<Mode, string> = { [Mode.Manual]: 'Elevator · Aileron', [Mode.Fbwa]: 'Pitch · Bank', [Mode.Acro]: 'Pitch · Roll rate', [Mode.QStab]: 'Tilt', [Mode.QHover]: 'Tilt', [Mode.QLoiter]: 'Move' };
  $('lblL').textContent = st.task !== Task.None ? 'Move to take over' : L[st.mode];
  $('lblR').textContent = st.task !== Task.None ? 'Move to take over' : R[st.mode];
}

// ---- start
async function main() {
  const core = await Core.load(Math.floor(Math.random() * 1e6));
  $('engine').textContent = 'Rust · WebAssembly';
  const renderer = new Renderer($('cv') as HTMLCanvasElement);
  let smoke = false, windLvl = 0, real = false, trailT = 0;

  for (const b of document.querySelectorAll<HTMLElement>('[data-mode]')) b.addEventListener('click', () => core.setMode(MODE_ATTR[b.dataset.mode!]));
  $('btTakeoff').addEventListener('click', () => { core.cmd(Cmd.Takeoff); showHint('takeoff'); });
  $('btCircuit').addEventListener('click', () => { core.cmd(Cmd.Circuit); showHint('circuit'); });
  $('btLand').addEventListener('click', () => { core.cmd(Cmd.Land); showHint('land'); });
  $('btView').addEventListener('click', () => { renderer.view = renderer.view === 'pilot' ? 'chase' : 'pilot'; $('btView').textContent = 'View: ' + (renderer.view === 'pilot' ? 'Pilot' : 'Chase'); });
  $('btWind').addEventListener('click', () => { windLvl = (windLvl + 1) % 3; core.opt(Opt.Wind, windLvl); $('btWind').textContent = 'Wind ' + [0, 2, 4][windLvl]; });
  $('btSmoke').addEventListener('click', () => { smoke = !smoke; $('btSmoke').setAttribute('aria-pressed', String(smoke)); if (!smoke) renderer.trail = []; });
  $('btRecov').addEventListener('click', () => {
    const on = !core.state().recoveryOn; core.opt(Opt.Recovery, on);
    toast(on ? 'Recovery armed' : 'Recovery off', on ? 'Pulls out automatically when ground impact is imminent' : 'Impact warnings still show', '', 1600);
  });
  $('btVane').addEventListener('click', () => {
    const on = !core.state().weathervaneOn; core.opt(Opt.Weathervane, on);
    toast(on ? 'Weathervane on' : 'Weathervane off', on ? 'QHover and QLoiter turn the wing edge-on to the wind when the yaw stick is centred' : 'Heading stays where you leave it; broadside to the wind it will drift', '', 2000);
  });
  $('btCam').addEventListener('click', () => {
    const on = !core.state().camOn; core.opt(Opt.Camera, on);
    if (on) showHint('cam');
    toast(on ? 'Tail cam on' : 'Tail cam off', on ? 'Precision landing on the AprilTag' : 'Landing on GPS alone', '', 1800);
  });
  $('btReal').addEventListener('click', () => {
    real = !real; core.opt(Opt.RealPlane, real); renderer.trail = [];
    if (real) { showHint('real'); toast('Real plane', 'Sensors, estimator, build errors and battery. Back on the pad.', '', 2600); }
    else toast('Ideal model', 'Perfect state knowledge. Back on the pad.', '', 2000);
  });
  const pick = $('showPick');
  const setPick = (open: boolean) => { pick.hidden = !open; $('btShow').setAttribute('aria-expanded', String(open)); };
  $('btShow').addEventListener('click', () => setPick(pick.hidden));
  $('pickClose').addEventListener('click', () => setPick(false));
  for (const b of pick.querySelectorAll<HTMLElement>('[data-fig]')) b.addEventListener('click', () => {
    core.figure(Number(b.dataset.fig)); setPick(false); showHint('show');
  });
  $('btReset').addEventListener('click', () => { core.reset(); renderer.trail = []; $('toast').hidden = true; });
  addEventListener('resize', paintSticks);

  showHint(String(Mode.QLoiter));
  let last = performance.now();
  const frame = (now: number) => {
    const ft = Math.min(0.05, (now - last) / 1000); last = now;
    applyKeys(ft);
    core.sticks(sticks.R.x, sticks.R.y, sticks.L.x, sticks.L.y, sticks.L.active);
    core.advance(ft);
    for (const e of core.events()) onEvent(e);
    const st = core.state();
    if (smoke && !st.crashed) {
      trailT += ft;
      if (trailT > 0.04) { trailT = 0; renderer.trail.push(add(st.p, qrot(st.q, [-0.6, 0, 0])) as V3); if (renderer.trail.length > 700) renderer.trail.shift(); }
    }
    renderer.draw(st, ft);
    // tail camera picture-in-picture while the tail points at the ground
    const camShow = st.real && st.camOn && !st.crashed && qrot(st.q, [1, 0, 0])[2] > 0.5;
    const cw = $('camWrap'); cw.hidden = !camShow;
    if (camShow) {
      drawCam($('camv') as HTMLCanvasElement, st.p, st.q, st.tagLock, st.tagPx, st.tagId);
      cw.classList.toggle('lock', st.tagLock);
      $('camCap').textContent = st.tagLock ? `Tag locked · ${st.tagId === 0 ? '40' : '8'} cm` : 'Tail cam · searching';
    }
    hud(st, now);
    requestAnimationFrame(frame);
  };
  paintSticks();
  requestAnimationFrame(frame);
}
main().catch(err => { $('statusTxt').textContent = 'The flight core failed to start: ' + String(err); });
