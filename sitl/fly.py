#!/usr/bin/env python3
"""Test flight of ArduPlane on the foamie physics: QLOITER takeoff and hover, then an AUTO mission
(VTOL takeoff, a fixed-wing circuit, VTOL landing). Prints what happens and a summary.

  python3 fly.py [--connect tcp:127.0.0.1:5760] [--stage hover|auto|all]
"""
import argparse, math, sys, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument('--connect', default='tcp:127.0.0.1:5760')
ap.add_argument('--stage', default='all')
ap.add_argument('--param', action='append', default=[], help='NAME=VALUE set before flying')
args = ap.parse_args()

m = mavutil.mavlink_connection(args.connect, source_system=255)
m.wait_heartbeat(timeout=60)
print('heartbeat from system', m.target_system)
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

state = dict(t=0.0, x=0.0, y=0.0, z=0.0, vz=0.0, roll=0, pitch=0, yaw=0, mode='', armed=False, fix=0, air=0.0)
texts = []

def pump(dt=0.0):
    end = time.time() + dt
    while True:
        msg = m.recv_match(blocking=dt > 0, timeout=max(0.01, end - time.time()))
        if msg is not None:
            k = msg.get_type()
            if k == 'LOCAL_POSITION_NED':
                state.update(x=msg.x, y=msg.y, z=-msg.z, vz=-msg.vz)
            elif k == 'ATTITUDE':
                state.update(roll=math.degrees(msg.roll), pitch=math.degrees(msg.pitch), yaw=math.degrees(msg.yaw))
            elif k == 'SYSTEM_TIME':
                state['t'] = msg.time_boot_ms / 1000.0
            elif k == 'HEARTBEAT' and msg.get_srcSystem() == m.target_system:
                state['mode'] = mavutil.mode_string_v10(msg)
                state['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif k == 'GPS_RAW_INT':
                state['fix'] = msg.fix_type
            elif k == 'VFR_HUD':
                state['air'] = msg.airspeed
            elif k == 'STATUSTEXT':
                texts.append(msg.text); print('  AP:', msg.text)
        if time.time() >= end and (msg is None or dt == 0):
            break

def wait_sim(sec, every=1.0, fn=None):
    t0 = state['t']; nxt = t0
    while state['t'] - t0 < sec:
        pump(0.05)
        if fn and state['t'] >= nxt:
            nxt += every; fn()

def show():
    s = state
    print(f"  t={s['t']:6.1f} {s['mode']:<9} pos N{s['x']:6.1f} E{s['y']:6.1f} alt{s['z']:6.1f} vz{s['vz']:5.1f} "
          f"att r{s['roll']:6.1f} p{s['pitch']:6.1f} y{s['yaw']:6.1f} air{s['air']:5.1f}")

def rc(**ch):
    vals = [65535] * 8
    for k, v in ch.items(): vals[int(k[2:]) - 1] = v
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *vals)

def mode(name):
    m.set_mode(m.mode_mapping()[name])
    for _ in range(50):
        pump(0.1)
        if state['mode'] == name: return
    print('mode change failed', name)

def arm():
    for _ in range(5):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
        pump(1.0)
        if state['armed']: return True
    return False

print('waiting for GPS and EKF ...')
while state['fix'] < 3 or state['t'] < 25:
    pump(0.5)
print('GPS fix', state['fix'])
for pv in args.param:
    k, v = pv.split('=')
    m.param_set_send(k, float(v)); pump(0.3)
    print('set', k, v)

def keepalive():
    rc(ch1=1500, ch2=1500, ch3=state.get('thr', 1500), ch4=1500)

if args.stage in ('hover', 'all'):
    mode('QLOITER')
    state['thr'] = 1500
    keepalive()
    print('arming:', arm())
    print('== climb in QLOITER')
    state['thr'] = 1800
    t0 = state['t']
    while state['z'] < 10 and state['t'] - t0 < 30:
        keepalive(); pump(0.2)
    show()
    state['thr'] = 1500
    print('== hover 20 s, hands off')
    xs, ys, zs = [], [], []
    def rec():
        keepalive(); show(); xs.append(state['x']); ys.append(state['y']); zs.append(state['z'])
    wait_sim(20, 1.0, rec)
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    drift = max(math.hypot(x - xs[0], y - ys[0]) for x, y in zip(xs, ys))
    rms = math.sqrt(sum((x - cx) ** 2 + (y - cy) ** 2 for x, y in zip(xs, ys)) / len(xs))
    print(f'hover: rms {rms:.2f} m; alt {min(zs):.1f}..{max(zs):.1f} m, max drift {drift:.2f} m from start')
    print('== QLAND')
    mode('QLAND')
    wait_sim(40, 2.0, lambda: (keepalive(), show()))
    show()
    print('armed after QLAND:', state['armed'])

def upload(items):
    """items: list of (cmd, lat, lon, alt, p1); item 0 is home."""
    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    sent = set()
    while len(sent) < len(items):
        msg = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=5)
        if msg is None: print('mission upload timed out'); return False
        if msg.get_type() == 'MISSION_ACK': break
        i = msg.seq; cmd, lat, lon, alt, p1 = items[i]
        m.mav.mission_item_int_send(m.target_system, m.target_component, i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1, p1, 0, 0, 0,
            int(lat * 1e7), int(lon * 1e7), alt)
        sent.add(i)
    ack = m.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    print('mission uploaded', len(items), 'items, ack', ack.type if ack else None)
    return True

if args.stage in ('auto', 'all'):
    home = (35.8989, 14.4897)
    def off(n, e):
        return home[0] + n / 111320.0, home[1] + e / (111320.0 * math.cos(math.radians(home[0])))
    C = mavutil.mavlink
    pts = [(80, -40), (80, 40), (-60, 40), (-60, -40)]
    items = [(C.MAV_CMD_NAV_WAYPOINT, *home, 0, 0), (C.MAV_CMD_NAV_VTOL_TAKEOFF, *home, 15, 0)]
    items += [(C.MAV_CMD_NAV_WAYPOINT, *off(n, e), 25, 0) for n, e in pts]
    items += [(C.MAV_CMD_NAV_WAYPOINT, *off(80, -40), 25, 0), (C.MAV_CMD_NAV_VTOL_LAND, *home, 0, 0)]
    upload(items)
    pump(1)
    mode('AUTO')
    state['thr'] = 1000
    keepalive()
    print('arming:', arm())
    print('== AUTO: VTOL takeoff, fixed-wing circuit, VTOL land')
    t0 = state['t']; maxair = 0
    seq = [-1]
    while state['t'] - t0 < 300:
        pump(0.1)
        msg = m.recv_match(type='MISSION_CURRENT', blocking=False)
        if msg and msg.seq != seq[0]:
            seq[0] = msg.seq; print(f'  -> mission item {msg.seq}'); show()
        maxair = max(maxair, state['air'])
        if int(state['t'] * 2) % 6 == 0 and abs(state['t'] * 2 - round(state['t'] * 2)) < 0.05: show()
        if state['t'] - t0 > 20 and not state['armed']:
            break
    show()
    print(f"auto: landed {math.hypot(state['x'], state['y']):.2f} m from home, max airspeed {maxair:.1f} m/s, "
          f"armed={state['armed']}, {state['t'] - t0:.0f} s")
