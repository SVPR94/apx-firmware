#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
SITL test: AUTO mode path-following quality.

Flies takeoff -> 90deg corner -> 45deg corners (one diagonal leg) -> stop -> LAND
and measures, against the uploaded polyline:
  - max / mean horizontal distance from the planned path (straight-line tracking)
  - max horizontal speed vs WPNAV_SPEED (speed limiting)
  - at each corner: overshoot to the outside of the turn, and how close the
    vehicle passes the waypoint (fly-through corners are cut by design, by
    at most the waypoint radius)
  - overshoot past the final stopping waypoint
  - altitude error during cruise
  - with --sensor-log: the vehicle's actual horizontal accel/jerk and
    roll/pitch rates in each corner, from 400 Hz simulator truth

Usage: test_auto_path_tracking.py [--speed CM_S] [--accel CM_S2] [--no-limits]
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] pymavlink is required. Install with: pip install pymavlink")
    sys.exit(1)

WORKSPACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EARTH_R = 6378137.0
LAT0 = 18.5204
LON0 = 73.8567
ALT = 10.0

# Waypoints as (north_m, east_m) - home is (0, 0)
MISSIONS = {
    # 90deg corner, diagonal leg, 45deg corners, stop, LAND
    "corners": [(0.0, 0.0), (50.0, 0.0), (50.0, 50.0), (0.0, 100.0), (0.0, 130.0)],
    # lawnmower with 5 m cross legs, DO_CHANGE_SPEED mid-mission, then RTL
    "survey": [(0.0, 0.0), (30.0, 0.0), (30.0, 5.0), (0.0, 5.0), (0.0, 10.0), (30.0, 10.0)],
    # long straight legs for steady-state tracking (cross-track oscillation)
    "long": [(0.0, 0.0), (120.0, 0.0), (120.0, 10.0), (0.0, 10.0)],
}
PATH_NE = MISSIONS["corners"]
SURVEY_SPEED_MS = 3.0
WP_RADIUS_M = 2.0  # mission item param2 (acceptance radius) of every waypoint


def ne_to_latlon(n, e):
    lat = LAT0 + math.degrees(n / EARTH_R)
    lon = LON0 + math.degrees(e / (EARTH_R * math.cos(math.radians(LAT0))))
    return lat, lon


def latlon_to_ne(lat, lon):
    n = math.radians(lat - LAT0) * EARTH_R
    e = math.radians(lon - LON0) * EARTH_R * math.cos(math.radians(LAT0))
    return n, e


def dist_to_segment(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    t = 0.0 if seg_len_sq == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def dist_to_path(p):
    return min(dist_to_segment(p, PATH_NE[i], PATH_NE[i + 1]) for i in range(len(PATH_NE) - 1))


def overshoot_past(p, a, b):
    """Distance of p beyond vertex b along the a->b direction (0 if not past)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    return max(0.0, (p[0] - b[0]) * ux + (p[1] - b[1]) * uy)


def corner_geometry(i):
    """Unit vectors in/out of vertex i and the turn sign (+1 left, -1 right)."""
    a, v, b = PATH_NE[i - 1], PATH_NE[i], PATH_NE[i + 1]
    u1 = ((v[0] - a[0]) / math.dist(a, v), (v[1] - a[1]) / math.dist(a, v))
    u2 = ((b[0] - v[0]) / math.dist(v, b), (b[1] - v[1]) / math.dist(v, b))
    cross = u1[0] * u2[1] - u1[1] * u2[0]
    return u1, u2, (1.0 if cross >= 0 else -1.0)


def outside_overshoot(p, i):
    """How far p lies on the outside of the turn at vertex i (0 if inside)."""
    v = PATH_NE[i]
    u1, u2, sign = corner_geometry(i)
    d = (p[0] - v[0], p[1] - v[1])
    s1 = u1[0] * d[1] - u1[1] * d[0]
    s2 = u2[0] * d[1] - u2[1] * d[0]
    return max(0.0, -sign * s1, -sign * s2)


def nearest_leg(p):
    return min(range(len(PATH_NE) - 1), key=lambda k: dist_to_segment(p, PATH_NE[k], PATH_NE[k + 1]))


def corner_smoothness(path):
    """Per corner, from the SITL sensor log's truth rows: peak horizontal
    accel and jerk and peak roll/pitch rate within 12 m of the vertex, at
    cruise altitude."""
    t, n, e, d, vn, ve, p, q = [], [], [], [], [], [], [], []
    with open(path) as f:
        for line in f:
            f_ = line.split(",")
            if len(f_) < 20 or f_[1] != "T":
                continue
            t.append(int(f_[0]) * 1e-6)
            n.append(float(f_[3]))
            e.append(float(f_[4]))
            d.append(float(f_[5]))
            vn.append(float(f_[6]))
            ve.append(float(f_[7]))
            p.append(float(f_[13]))
            q.append(float(f_[14]))
    if len(t) < 100:
        return []
    # accel by central difference over +-25 ms, 0.1 s moving average, then
    # jerk the same way: the controller's response, not integration noise
    k = 10

    def deriv(xs):
        out = [0.0] * len(xs)
        for j in range(k, len(xs) - k):
            dt = t[j + k] - t[j - k]
            out[j] = (xs[j + k] - xs[j - k]) / dt if dt > 0 else 0.0
        return out

    def smooth(xs, w=40):
        out, acc = [0.0] * len(xs), 0.0
        for j, x in enumerate(xs):
            acc += x - (xs[j - w] if j >= w else 0.0)
            out[j] = acc / min(j + 1, w)
        return out

    an, ae = smooth(deriv(vn)), smooth(deriv(ve))
    jn, je = smooth(deriv(an)), smooth(deriv(ae))
    results = []
    for i in range(1, len(PATH_NE) - 1):
        v = PATH_NE[i]
        # rows on one of this corner's two legs (nearest leg; other survey
        # rows may be closer than 12 m) at cruise altitude (not RTL)
        idx = [j for j in range(len(t)) if math.hypot(n[j] - v[0], e[j] - v[1]) < 12.0
               and abs(-d[j] - ALT) < 2.0 and nearest_leg((n[j], e[j])) in (i - 1, i)]
        if not idx:
            continue
        results.append((i,
                        max(math.hypot(an[j], ae[j]) for j in idx),
                        max(math.hypot(jn[j], je[j]) for j in idx),
                        math.degrees(max(max(abs(p[j]), abs(q[j])) for j in idx)),
                        min(math.hypot(vn[j], ve[j]) for j in idx)))
    return results


def build_mission(name, mission_speed=0.0):
    """Returns (items, seq of the final LAND/RTL item). Item: (cmd, p2, lat, lon, alt)."""
    items = []
    items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, LAT0, LON0, 0.0))
    items.append((mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, LAT0, LON0, ALT))
    if mission_speed > 0:
        items.append((mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, mission_speed, 0.0, 0.0, 0.0))
    for i, (n, e) in enumerate(PATH_NE[1:]):
        if name == "survey" and i == 2:
            items.append((mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, SURVEY_SPEED_MS, 0.0, 0.0, 0.0))
        lat, lon = ne_to_latlon(n, e)
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, WP_RADIUS_M, lat, lon, ALT))
    if name == "survey":
        items.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0, 0.0))
    else:
        lat, lon = ne_to_latlon(*PATH_NE[-1])
        items.append((mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0, lat, lon, 0.0))
    return items, len(items) - 1


def set_param(mav, name, value):
    for _ in range(3):
        mav.mav.param_set_send(mav.target_system, mav.target_component, name.encode(), value,
                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id.rstrip('\x00') == name:
            print(f"[TEST] {name} = {msg.param_value}")
            return True
    print(f"[WARN] no PARAM_VALUE ack for {name}")
    return False


def upload_mission(mav, items):
    mav.mav.mission_count_send(mav.target_system, mav.target_component, len(items),
                               mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    for _ in range(len(items)):
        req = mav.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
        if not req:
            return False
        cmd, p2, lat, lon, alt = items[req.seq][:5]
        # optional 6th field: explicit param1 (e.g. DO_SPRAYER on/off);
        # DO_CHANGE_SPEED: param1 speed type 1 (ground speed), param2 speed
        if len(items[req.seq]) > 5:
            p1 = items[req.seq][5]
        else:
            p1 = 1.0 if cmd == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED else 0.0
        mav.mav.mission_item_int_send(
            mav.target_system, mav.target_component, req.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, cmd, 0, 1,
            p1, p2, 0, 0, int(lat * 1e7), int(lon * 1e7), alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    ack = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    return bool(ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED)


def run(args):
    sim_cmd = [sys.executable, os.path.join(WORKSPACE, "sitl/sim_vehicle.py"),
               "--vehicle", args.vehicle, "--sim", args.sim, "--port", "9003",
               # keep a running APX Flight Manager (UDP 14550) from attaching
               "--gcs-port", "14650"]
    if args.sensor_log:
        sim_cmd += ["--sensor-log", args.sensor_log]
    if args.sim == "gazebo":
        sim_cmd.append("--headless")
    sim_log = open(os.path.join(args.log_dir, "sim_output.log"), "w")
    sim_proc = subprocess.Popen(sim_cmd, stdout=sim_log, stderr=subprocess.STDOUT, text=True)
    time.sleep(8 if args.sim == "gazebo" else 4)
    try:
        mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190)
        mav.wait_heartbeat(timeout=15)
        print("[TEST] heartbeat received")

        speed_cms = args.speed
        if speed_cms > 0:
            set_param(mav, "WPNAV_SPEED", speed_cms)
        else:
            mav.mav.param_request_read_send(mav.target_system, mav.target_component, b"WPNAV_SPEED", -1)
            msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            speed_cms = msg.param_value if msg else 1000.0
        if args.accel > 0:
            set_param(mav, "WPNAV_ACCEL", args.accel)
        for kv in args.param:
            name, value = kv.split("=", 1)
            set_param(mav, name, float(value))
        speed_limit = speed_cms / 100.0

        items, land_seq = build_mission(args.mission, args.mission_speed)
        # seq of the item each path vertex is flown to (PATH_NE[0] is takeoff)
        vertex_seq = [1] + [i for i, it in enumerate(items) if it[0] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT and i > 0]
        if not upload_mission(mav, items):
            print("[FAIL] mission upload")
            return False
        print("[TEST] mission uploaded")

        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1)
        mav.mav.set_mode_send(mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3)

        start = time.time()
        last_print = 0.0
        seq = 0
        samples = []      # (t, n, e, alt, speed, seq)
        landing = False
        landed_t = None
        live_sent_t = None
        max_roll = 0.0
        max_tilt_at = (0.0, 0)
        yaw_deg = 0.0
        while time.time() - start < args.timeout:
            msg = mav.recv_match(type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT', 'ATTITUDE'],
                                 blocking=True, timeout=0.5)
            if not msg:
                continue
            if msg.get_type() == 'MISSION_CURRENT':
                seq = max(seq, msg.seq)
                if args.live_speed > 0 and live_sent_t is None and seq >= vertex_seq[2]:
                    mav.mav.command_long_send(mav.target_system, mav.target_component,
                                              mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                                              1, args.live_speed, -1, 0, 0, 0, 0)
                    live_sent_t = time.time() - start
                    print(f"  [TEST] sent DO_CHANGE_SPEED {args.live_speed} m/s (command)")
                if seq >= land_seq:
                    landing = True
                continue
            if msg.get_type() == 'ATTITUDE':
                yaw_deg = math.degrees(msg.yaw)
                tilt = max(abs(math.degrees(msg.roll)), abs(math.degrees(msg.pitch)))
                if tilt > max_roll:
                    max_roll = tilt
                    max_tilt_at = (time.time() - start, seq)
                continue
            n, e = latlon_to_ne(msg.lat / 1e7, msg.lon / 1e7)
            alt = msg.relative_alt / 1000.0
            spd = math.hypot(msg.vx, msg.vy) / 100.0
            t = time.time() - start
            samples.append((t, n, e, alt, spd, seq, yaw_deg))
            if t - last_print > 2.0:
                last_print = t
                print(f"  t={t:5.1f} seq={seq} N={n:7.2f} E={e:7.2f} alt={alt:5.2f} spd={spd:5.2f} "
                      f"path_err={dist_to_path((n, e)):5.2f}")
            # the hex's landing gear rests at ~0.6 m relative altitude
            if landing and alt < 1.0 and spd < 0.3 and abs(msg.vz) < 10:
                landed_t = landed_t or t
                if t - landed_t > 2.0:
                    break

        with open(os.path.join(args.log_dir, "track.csv"), "w") as f:
            f.write("t,n,e,alt,speed,seq,yaw\n")
            for s in samples:
                f.write(",".join(f"{v:.3f}" for v in s) + "\n")

        cruise = [s for s in samples if vertex_seq[2] <= s[5] < land_seq and s[3] > ALT - 2.0]
        # steady-state cross-track on long legs: samples > 15 m from any vertex
        steady = [dist_to_path((s[1], s[2])) for s in cruise
                  if min(math.hypot(s[1] - v[0], s[2] - v[1]) for v in PATH_NE) > 15.0]
        if not cruise:
            print("[FAIL] vehicle never reached cruise")
            return False
        path_err = [dist_to_path((s[1], s[2])) for s in cruise]
        speeds = [s[4] for s in cruise]
        alt_err = [abs(s[3] - ALT) for s in cruise]
        # Deviation from the planned polyline within 10 m of each corner:
        # overshooting a turn shows up as a large distance from both legs.
        corner_os = []
        for i in range(1, len(PATH_NE) - 1):
            v = PATH_NE[i]
            corner_os.append(max([dist_to_path((s[1], s[2])) for s in cruise
                                  if math.hypot(s[1] - v[0], s[2] - v[1]) < 10.0] or [0.0]))
        # Fly-through corners are cut on purpose; what must not happen is
        # drifting to the outside of the turn or cutting wider than the
        # waypoint radius.
        # From the first leg on: the mission moves past a fly-through
        # waypoint only at the corner's exit, so "cruise" (which starts at
        # the second waypoint) would miss the first corner.
        at_alt = [s for s in samples if vertex_seq[1] <= s[5] < land_seq and s[3] > ALT - 2.0]
        corner_out = []
        corner_pass = []
        for i in range(1, len(PATH_NE) - 1):
            # only while flying this corner's own two legs (survey rows are
            # closer together than the 10 m window)
            near = [s for s in at_alt if math.hypot(s[1] - PATH_NE[i][0], s[2] - PATH_NE[i][1]) < 10.0
                    and vertex_seq[i] <= s[5] <= vertex_seq[i + 1]]
            corner_out.append(max([outside_overshoot((s[1], s[2]), i) for s in near] or [0.0]))
            corner_pass.append(min([math.hypot(s[1] - PATH_NE[i][0], s[2] - PATH_NE[i][1]) for s in near]
                                   or [float("inf")]))
        final_os = max([overshoot_past((s[1], s[2]), PATH_NE[-2], PATH_NE[-1]) for s in samples
                        if s[5] > vertex_seq[-1]] or [0.0])
        land_n, land_e = samples[-1][1], samples[-1][2]
        land_target = (0.0, 0.0) if args.mission == "survey" else PATH_NE[-1]
        land_err = math.hypot(land_n - land_target[0], land_e - land_target[1])
        max_alt = max(s[3] for s in samples)

        print("=" * 70)
        print(f" WPNAV_SPEED limit      : {speed_limit:.2f} m/s")
        print(f" max speed              : {max(speeds):.2f} m/s")
        print(f" path error max / mean  : {max(path_err):.2f} / {sum(path_err) / len(path_err):.2f} m")
        if steady:
            print(f" mid-leg path error     : max {max(steady):.2f} m, mean {sum(steady) / len(steady):.2f} m")
        print(f" corner deviation (m)   : {', '.join(f'{c:.2f}' for c in corner_os)} (from either leg)")
        print(f" corner outside (m)     : {', '.join(f'{c:.2f}' for c in corner_out)}")
        print(f" corner pass dist (m)   : {', '.join(f'{c:.2f}' for c in corner_pass)} (WP radius {WP_RADIUS_M:.1f})")
        print(f" final WP overshoot     : {final_os:.2f} m")
        print(f" landing position error : {land_err:.2f} m")
        print(f" max |alt err| cruise   : {max(alt_err):.2f} m")
        print(f" max altitude           : {max_alt:.2f} m")
        print(f" max tilt               : {max_roll:.1f} deg (t={max_tilt_at[0]:.1f}s seq={max_tilt_at[1]})")
        print(f" mission time           : {samples[-1][0]:.1f} s (landed={landed_t is not None})")
        if args.sensor_log and os.path.exists(args.sensor_log):
            for i, acc, jerk, rate, vmin in corner_smoothness(args.sensor_log):
                print(f" corner {i} (truth)       : peak accel {acc:.2f} m/s^2, jerk {jerk:.2f} m/s^3, "
                      f"roll/pitch rate {rate:.1f} deg/s, min speed {vmin:.2f} m/s")
        print("=" * 70)

        # cruise speed well away from corners, before / after a live change
        def mid_leg_speeds(cond):
            legs = [s for s in samples if vertex_seq[1] <= s[5] < land_seq and s[3] > ALT - 2.0]
            return [s[4] for s in legs if cond(s) and
                    min(math.hypot(s[1] - v[0], s[2] - v[1]) for v in PATH_NE) > 12.0]
        expected = args.mission_speed if args.mission_speed > 0 else speed_limit
        before = mid_leg_speeds(lambda s: live_sent_t is None or s[0] < live_sent_t)
        after = mid_leg_speeds(lambda s: live_sent_t is not None and s[0] > live_sent_t + 6.0)
        if before:
            print(f" mid-leg cruise speed   : {max(before):.2f} m/s (expected {expected:.1f})")
        if after:
            print(f" cruise after live cmd  : {max(after):.2f} m/s (expected {args.live_speed:.1f})")

        if args.no_limits:
            return True
        ok = True
        top_speed = max([speed_limit, args.mission_speed, args.live_speed])
        checks = [
            (max(speeds) <= top_speed * 1.10 + 0.2, "speed within the commanded speed"),
            (max(path_err) <= 2.0, "max path error <= 2.0 m"),
            (max(corner_out) <= 0.5, "no corner overshoot to the outside (<= 0.5 m)"),
            (max(corner_pass) <= WP_RADIUS_M + 0.5, "corners pass within WP radius + 0.5 m"),
            (final_os <= 1.0, "final waypoint overshoot <= 1.0 m"),
            (landed_t is not None, "landed"),
            (land_err <= 1.0, "landed within 1.0 m of target"),
        ]
        if args.mission_speed > 0 and before:
            checks.append((abs(max(before) - args.mission_speed) <= 0.1 * args.mission_speed,
                           "mission DO_CHANGE_SPEED cruise reached"))
        if args.live_speed > 0:
            checks.append((bool(after) and abs(max(after) - args.live_speed) <= 0.1 * args.live_speed,
                           "live DO_CHANGE_SPEED command applied"))
        if args.mission == "survey":
            # Cruise speed away from corners (corner exits show a brief
            # catch-up transient, reported separately).
            late = [s for s in samples if vertex_seq[3] < s[5] < land_seq]
            mid_leg = [s[4] for s in late
                       if min(math.hypot(s[1] - v[0], s[2] - v[1]) for v in PATH_NE) > 6.0]
            print(f" speed after DO_CHANGE  : cruise {max(mid_leg or [0.0]):.2f} m/s, "
                  f"peak {max([s[4] for s in late] or [0.0]):.2f} m/s (requested {SURVEY_SPEED_MS:.1f})")
            checks.append((max(mid_leg or [0.0]) <= SURVEY_SPEED_MS * 1.1, "DO_CHANGE_SPEED respected"))
            checks.append((max_alt >= 14.0, "RTL climbed to RTL_ALT"))
        for passed, label in checks:
            print(f" [{'PASS' if passed else 'FAIL'}] {label}")
            ok = ok and passed
        return ok
    finally:
        if sim_proc.poll() is None:
            sim_proc.send_signal(signal.SIGINT)
            try:
                sim_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sim_proc.kill()
        sim_log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default="hex_x", choices=["quad_x", "hex_x", "octo_x"])
    parser.add_argument("--mission", default="corners", choices=sorted(MISSIONS))
    parser.add_argument("--sim", default="builtin", choices=["builtin", "gazebo"])
    parser.add_argument("--param", action="append", default=[], help="NAME=VALUE to set before flight")
    parser.add_argument("--mission-speed", type=float, default=0.0, help="DO_CHANGE_SPEED item (m/s) at mission start")
    parser.add_argument("--live-speed", type=float, default=0.0, help="DO_CHANGE_SPEED command (m/s) sent on leg 2")
    parser.add_argument("--speed", type=float, default=0.0, help="WPNAV_SPEED cm/s to set (0 = keep)")
    parser.add_argument("--accel", type=float, default=0.0, help="WPNAV_ACCEL cm/s/s to set (0 = keep)")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--log-dir", default="/tmp")
    parser.add_argument("--no-limits", action="store_true", help="report metrics only, no pass/fail")
    parser.add_argument("--sensor-log", default=None,
                        help="CSV path for the SITL sensor log; enables per-corner accel/jerk/rate analysis")
    cli_args = parser.parse_args()
    PATH_NE = MISSIONS[cli_args.mission]
    sys.exit(0 if run(cli_args) else 1)
