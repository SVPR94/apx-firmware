#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Shared helpers for the APX / apx flight-test tools.

Works against any MAVLink copter that uses APX Copter mode numbers (APX SITL,
apx SITL, real vehicles). Everything is relative to the position the
vehicle is at when a test starts, so the same mission can be flown at any
home location.

Main pieces:
  Vehicle           connection, params, mission upload, modes, arm/takeoff
  fly_mission()     run one AUTO mission and record a trajectory
  analyze()         waypoint-transition metrics (speed dips, stops, jerk,
                    cross-track error, overshoot) + a single score
  MISSIONS          built-in test patterns (north/east metres)
"""

import csv
import json
import math
import time

from pymavlink import mavutil

EARTH_R = 6378137.0

# APX Copter custom-mode numbers (APX uses the same numbering).
MODE_AUTO = 3
MODE_GUIDED = 4
MODE_LOITER = 5
MODE_RTL = 6
MODE_LAND = 9

# apx 4.7+ renamed the WPNAV_* params (cm, cm/s) to WP_* (m, m/s).
# Tools always take the APX / classic names; Vehicle translates when needed.
PARAM_ALIASES = {
    "WPNAV_SPEED": ("WP_SPD", 0.01),
    "WPNAV_ACCEL": ("WP_ACC", 0.01),
    "WPNAV_RADIUS": ("WP_RADIUS_M", 0.01),
    "WPNAV_JERK": ("WP_JERK", 1.0),
    "WPNAV_SPEED_UP": ("WP_SPD_UP", 0.01),
    "WPNAV_SPEED_DN": ("WP_SPD_DN", 0.01),
}

# Built-in missions: list of (north_m, east_m) waypoints flown after takeoff,
# starting from the vehicle's position. The first point is the start.
MISSIONS = {
    # square, 90 deg corners
    "square": [(0, 0), (40, 0), (40, 40), (0, 40), (0, 0)],
    # mixed 45/90/135 deg corners and legs of different lengths
    "corners": [(0, 0), (50, 0), (50, 50), (0, 100), (0, 130), (30, 160)],
    # gentle turns (20-30 deg): should be flown at full speed
    "gentle": [(0, 0), (40, 0), (78, 14), (115, 22), (150, 40), (185, 50)],
    # straight line split into many waypoints: must never slow down
    "straight": [(0, 0), (20, 0), (40, 0), (60, 0), (80, 0), (100, 0), (120, 0)],
    # lawnmower survey with 8 m row spacing
    "survey": [(0, 0), (40, 0), (40, 8), (0, 8), (0, 16), (40, 16), (40, 24), (0, 24)],
    # zig-zag, ~60 deg corners
    "zigzag": [(0, 0), (30, 17), (60, 0), (90, 17), (120, 0), (150, 17)],
}


# --- geometry -----------------------------------------------------------------

def ne_offset(lat, lon, north, east):
    """Lat/lon of a point north/east metres from (lat, lon)."""
    dlat = math.degrees(north / EARTH_R)
    dlon = math.degrees(east / (EARTH_R * math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


def latlon_to_ne(lat0, lon0, lat, lon):
    n = math.radians(lat - lat0) * EARTH_R
    e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    return n, e


def dist_point_segment(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def turn_angle_deg(a, b, c):
    """Heading change at b when flying a -> b -> c."""
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    d = math.degrees(h2 - h1)
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return abs(d)


def load_mission(spec):
    """Waypoint list (north, east) of a mission spec (see load_plan)."""
    return load_plan(spec)["wps"]


CMD_WAYPOINT = 16
CMD_RTL = 20
CMD_LAND = 21
CMD_TAKEOFF = 22
CMD_CONDITION_YAW = 115
CMD_DO_CHANGE_SPEED = 178
POSITIONAL = {16, 17, 18, 19, 21, 31, 82}


def orbit_yaw_plan(radius=28.0, points=16, speed=2.0, alt=20.0):
    """Orbit with a CONDITION_YAW after every waypoint (heading per waypoint,
    as the APX Flight Manager / GCS orbit-with-heading missions do)."""
    items = [dict(cmd=CMD_DO_CHANGE_SPEED, p1=1, p2=speed)]
    wps = [(0.0, 0.0)]
    for k in range(points + 1):
        a = 2 * math.pi * k / points
        n, e = radius * math.cos(a), radius * math.sin(a)
        items.append(dict(cmd=CMD_WAYPOINT, n=n, e=e, alt=alt))
        wps.append((n, e))
        # heading for the next leg, like the GCS: tangent to the circle
        items.append(dict(cmd=CMD_CONDITION_YAW, p1=(math.degrees(a) + 90.0 + 360.0 / points / 2) % 360))
    items.pop()
    return dict(wps=wps, items=items, alt=alt)


def _items_from_raw(raw, home):
    """raw: [(seq, cmd, frame, p1, p2, p3, p4, lat, lon, z)], home (lat, lon, alt_msl)."""
    items, wps = [], [(0.0, 0.0)]
    dropped = []
    for seq, cmd, frame, p1, p2, p3, p4, lat, lon, z in sorted(raw):
        if seq == 0:
            continue
        if cmd in (CMD_RTL, CMD_LAND):
            dropped.append(cmd)  # tests end hovering at the last waypoint
            continue
        it = dict(cmd=cmd, p1=p1, p2=p2, p3=p3, p4=p4)
        alt = z - home[2] if frame in (0, 5) and z != 0 else z  # GLOBAL frames are AMSL
        if cmd in POSITIONAL and (lat != 0 or lon != 0):
            n, e = latlon_to_ne(home[0], home[1], lat, lon)
            it.update(n=n, e=e, alt=alt)
            if cmd == CMD_WAYPOINT:
                wps.append((n, e))
        elif cmd == CMD_TAKEOFF:
            it.update(alt=alt)
        items.append(it)
    # start altitude: the mission's TAKEOFF altitude, else its first waypoint's
    nav_alts = [it["alt"] for it in items if it["cmd"] in (CMD_TAKEOFF, CMD_WAYPOINT)]
    if dropped:
        print(f"[flighttest] note: dropped {len(dropped)} RTL/LAND item(s) - the test ends at the last waypoint")
    return dict(wps=wps, items=items, alt=nav_alts[0] if nav_alts else None)


def _plan_from_tlog(path):
    m = mavutil.mavlink_connection(path)
    sets, cur = [], {}
    while True:
        msg = m.recv_match(type=["MISSION_ITEM_INT", "MISSION_ITEM"])
        if msg is None:
            break
        if getattr(msg, "mission_type", 0) != 0:
            continue
        if msg.seq == 0 and cur:
            sets.append(cur)
            cur = {}
        scale = 1e7 if msg.get_type() == "MISSION_ITEM_INT" else 1.0
        cur[msg.seq] = (msg.seq, msg.command, msg.frame, msg.param1, msg.param2, msg.param3, msg.param4,
                        msg.x / scale, msg.y / scale, msg.z)
    if cur:
        sets.append(cur)
    full = [st for st in sets if len(st) > 1 and len(st) == max(st) + 1]
    if not full:
        raise ValueError(f"no complete mission found in {path}")
    st = full[-1]
    h = st[0]
    return _items_from_raw(st.values(), (h[7], h[8], h[9]))


def _plan_from_qgc_plan(path):
    with open(path) as f:
        d = json.load(f)
    mis = d["mission"]
    home = mis.get("plannedHomePosition", [0, 0, 0])
    raw = []

    def add(it):
        p = it["params"] + [0] * 7
        raw.append((len(raw) + 1, it["command"], it.get("frame", 3), *(x or 0 for x in p[:4]),
                    p[4] or 0, p[5] or 0, p[6] or 0))

    for it in mis["items"]:
        if it.get("type") == "SimpleItem":
            add(it)
        else:  # complex items (survey, corridor): their generated simple items
            sub = it.get("TransectStyleComplexItem", {}).get("Items", [])
            for x in sub:
                if x.get("type") == "SimpleItem":
                    add(x)
    return _items_from_raw(raw, tuple(home))


def _plan_from_wpl(path):
    raw = []
    home = None
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 12 or not parts[0].isdigit():
                continue
            seq, frame, cmd = int(parts[0]), int(parts[2]), int(parts[3])
            p = [float(x) for x in parts[4:11]]
            if seq == 0:
                home = (p[4], p[5], p[6])
            raw.append((seq, cmd, frame, p[0], p[1], p[2], p[3], p[4], p[5], p[6]))
    return _items_from_raw(raw, home)


def load_plan(spec):
    """Mission spec -> {wps: [(n, e)], items: [...] or None, alt}.

    spec: a built-in name (MISSIONS, or 'orbit_yaw'), a JSON file
    ([[n, e], ...] or a list of item dicts), a GCS .plan, a .waypoints
    (GCS WPL 110) file, or a .tlog (the last mission uploaded/downloaded in it).
    Positions in files are made relative to their home, so a mission planned
    anywhere can be flown at any SITL location.
    """
    if spec == "orbit_yaw":
        return orbit_yaw_plan()
    if spec in MISSIONS:
        return dict(wps=[tuple(map(float, p)) for p in MISSIONS[spec]], items=None, alt=None)
    low = spec.lower()
    if low.endswith(".tlog"):
        return _plan_from_tlog(spec)
    if low.endswith(".plan"):
        return _plan_from_qgc_plan(spec)
    if low.endswith(".waypoints") or low.endswith(".txt"):
        return _plan_from_wpl(spec)
    with open(spec) as f:
        data = json.load(f)
    if data and isinstance(data[0], dict):
        wps = [(0.0, 0.0)] + [(it["n"], it["e"]) for it in data if it.get("cmd") == CMD_WAYPOINT]
        return dict(wps=wps, items=data, alt=None)
    return dict(wps=[(float(p[0]), float(p[1])) for p in data], items=None, alt=None)


# --- vehicle --------------------------------------------------------------------

class Vehicle:
    def __init__(self, connect, verbose=True, source_system=245, firmware=None):
        self.verbose = verbose
        self.last = {}
        self.log(f"connecting to {connect} ...")
        self.m = mavutil.mavlink_connection(connect, source_system=source_system, autoreconnect=True)
        self.wait_heartbeat()
        self.firmware = firmware or self.detect_firmware()
        self.log(f"heartbeat from sys {self.m.target_system} - firmware: {self.firmware}")
        self.request_streams(20)

    def detect_firmware(self):
        """APX also reports MAV_AUTOPILOT_APXMEGA; tell them apart by the
        AUTOPILOT_VERSION major number (APX 0.x, apx 4.x)."""
        for _ in range(3):
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                         mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                                         0, 0, 0, 0, 0, 0)
            msg = self.wait_msg("AUTOPILOT_VERSION", 2)
            if msg is not None:
                return "apx" if (msg.flight_sw_version >> 24) >= 1 else "apx"
        return "unknown"

    def log(self, msg):
        if self.verbose:
            print(f"[flighttest] {msg}", flush=True)

    def wait_heartbeat(self, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb is None:
                continue
            # skip GCS / onboard-computer heartbeats
            if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID or hb.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            self.m.target_system = hb.get_srcSystem()
            self.m.target_component = hb.get_srcComponent()
            self.autopilot = hb.autopilot
            return hb
        raise TimeoutError("no autopilot heartbeat")

    def request_streams(self, hz):
        interval = int(1e6 / hz)
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                       mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED):
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                         msg_id, interval, 0, 0, 0, 0, 0)
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, hz, 1)

    def pump(self, timeout=0.05):
        """Read pending messages, keep the latest of each type. Returns them."""
        out = []
        msg = self.m.recv_match(blocking=True, timeout=timeout)
        while msg is not None:
            if msg.get_srcSystem() == self.m.target_system:
                self.last[msg.get_type()] = msg
                out.append(msg)
            msg = self.m.recv_match(blocking=False)
        return out

    def wait_msg(self, mtype, timeout=10, cond=None):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for msg in self.pump(0.1):
                if msg.get_type() == mtype and (cond is None or cond(msg)):
                    return msg
        return None

    def position(self, timeout=30):
        msg = self.wait_msg("GLOBAL_POSITION_INT", timeout,
                            cond=lambda m: m.lat != 0 or m.lon != 0)
        if msg is None:
            raise TimeoutError("no GLOBAL_POSITION_INT")
        return msg

    # params ---------------------------------------------------------------------
    def _alias(self, name):
        """(real_name, scale) for this vehicle - classic name if it exists."""
        if name not in PARAM_ALIASES or self.firmware == "apx":
            return name, 1.0
        cache = self.__dict__.setdefault("_alias_cache", {})
        if name not in cache:
            new, scale = PARAM_ALIASES[name]
            cache[name] = (name, 1.0) if self._get_raw(name, retries=1) is not None else (new, scale)
        return cache[name]

    def get_param(self, name, timeout=3, retries=3):
        real, scale = self._alias(name)
        v = self._get_raw(real, timeout, retries)
        return None if v is None else v / scale

    def set_param(self, name, value, timeout=3, retries=3):
        real, scale = self._alias(name)
        return self._set_raw(real, value * scale, timeout, retries)

    def _get_raw(self, name, timeout=3, retries=3):
        for _ in range(retries):
            self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component,
                                               name.encode(), -1)
            msg = self.wait_msg("PARAM_VALUE", timeout, cond=lambda m: m.param_id == name)
            if msg is not None:
                return msg.param_value
        return None

    def _set_raw(self, name, value, timeout=3, retries=3):
        for _ in range(retries):
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(),
                                      float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            msg = self.wait_msg("PARAM_VALUE", timeout, cond=lambda m: m.param_id == name)
            if msg is not None and abs(msg.param_value - float(value)) <= 1e-3 * max(1.0, abs(value)):
                return True
        self.log(f"WARNING: could not set {name}={value}")
        return False

    def set_params(self, params):
        for k, v in params.items():
            self.set_param(k, v)

    # commands ---------------------------------------------------------------------
    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, timeout=3, retries=3):
        for _ in range(retries):
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0,
                                         p1, p2, p3, p4, p5, p6, p7)
            ack = self.wait_msg("COMMAND_ACK", timeout, cond=lambda m: m.command == cmd)
            if ack is not None:
                return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        return False

    def custom_mode(self):
        hb = self.last.get("HEARTBEAT")
        return hb.custom_mode if hb else None

    def armed(self):
        hb = self.last.get("HEARTBEAT")
        return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def set_mode(self, mode, timeout=10):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode,
                                         0, 0, 0, 0, 0)
            if self.wait_msg("HEARTBEAT", 2, cond=lambda m: m.custom_mode == mode and
                             m.get_srcComponent() == self.m.target_component) is not None:
                return True
        raise RuntimeError(f"mode {mode} not accepted")

    def arm(self, timeout=90):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, retries=1)
            if self.wait_msg("HEARTBEAT", 2, cond=lambda m: m.base_mode &
                             mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) is not None:
                return True
            time.sleep(1)  # EKF / prearm checks still settling
        raise RuntimeError("arming refused")

    def takeoff(self, alt, timeout=60):
        self.set_mode(MODE_GUIDED)
        self.arm()
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt)
        t0 = time.time()
        while time.time() - t0 < timeout:
            pos = self.wait_msg("GLOBAL_POSITION_INT", 1)
            if pos and pos.relative_alt / 1000.0 >= alt * 0.95:
                return True
        raise RuntimeError("takeoff did not reach altitude")

    def goto(self, lat, lon, alt, timeout=90, tol=1.0):
        """GUIDED position target; wait until there and settled."""
        mask = 0b0000111111111000
        t0 = time.time()
        settled_since = None
        while time.time() - t0 < timeout:
            self.m.mav.set_position_target_global_int_send(
                0, self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
                int(lat * 1e7), int(lon * 1e7), alt, 0, 0, 0, 0, 0, 0, 0, 0)
            pos = self.wait_msg("GLOBAL_POSITION_INT", 1)
            if pos is None:
                continue
            n, e = latlon_to_ne(lat, lon, pos.lat / 1e7, pos.lon / 1e7)
            speed = math.hypot(pos.vx, pos.vy) / 100.0
            ok = math.hypot(n, e) < tol and abs(pos.relative_alt / 1000.0 - alt) < tol and speed < 0.3
            if ok:
                settled_since = settled_since or time.time()
                if time.time() - settled_since > 2.0:
                    return True
            else:
                settled_since = None
        self.log("WARNING: goto timed out")
        return False

    # mission ------------------------------------------------------------------------
    def upload_mission(self, items, timeout=20):
        """items: list of dicts {cmd, lat, lon, alt, p1..p4}. Item 0 is home."""
        mav = self.m.mav
        mav.mission_clear_all_send(self.m.target_system, self.m.target_component)
        self.wait_msg("MISSION_ACK", 2)
        mav.mission_count_send(self.m.target_system, self.m.target_component, len(items))
        t0 = time.time()
        sent = set()
        while time.time() - t0 < timeout:
            for msg in self.pump(0.2):
                t = msg.get_type()
                if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                    it = items[msg.seq]
                    mav.mission_item_int_send(
                        self.m.target_system, self.m.target_component, msg.seq,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, it["cmd"],
                        0, 1, it.get("p1", 0), it.get("p2", 0), it.get("p3", 0), it.get("p4", 0),
                        int(it["lat"] * 1e7), int(it["lon"] * 1e7), it["alt"])
                    sent.add(msg.seq)
                elif t == "MISSION_ACK":
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED and len(sent) == len(items):
                        return True
                    if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        raise RuntimeError(f"mission rejected: {msg.type}")
        raise TimeoutError("mission upload timed out")


# --- flying a test -----------------------------------------------------------------

def build_items(origin_lat, origin_lon, wps_ne, alt, radius, takeoff=True):
    items = [dict(cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat=origin_lat, lon=origin_lon, alt=0)]
    if takeoff:
        items.append(dict(cmd=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, lat=0, lon=0, alt=alt))
    for n, e in wps_ne[1:]:
        lat, lon = ne_offset(origin_lat, origin_lon, n, e)
        items.append(dict(cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat=lat, lon=lon, alt=alt,
                          p2=radius))
    return items


def build_plan_items(lat0, lon0, plan, alt, radius):
    """Mission items for a plan with explicit items (positions from origin)."""
    items = [dict(cmd=CMD_WAYPOINT, lat=lat0, lon=lon0, alt=0)]
    if not any(it["cmd"] == CMD_TAKEOFF for it in plan["items"]):
        items.append(dict(cmd=CMD_TAKEOFF, lat=0, lon=0, alt=alt))
    for it in plan["items"]:
        out = dict(it)
        if "n" in it:
            out["lat"], out["lon"] = ne_offset(lat0, lon0, it["n"], it["e"])
        else:
            out["lat"] = out["lon"] = 0
        out["alt"] = it.get("alt", alt) if it.get("alt") is not None else (alt if "n" in it else 0)
        if it["cmd"] == CMD_WAYPOINT and not it.get("p2"):
            out["p2"] = radius
        items.append(out)
    return items


def fly_mission(veh, wps_ne, alt=10.0, radius=2.0, params=None, timeout=300, origin=None):
    """Fly a mission in AUTO and return a run dict.

    wps_ne: list of (north, east) waypoints from the origin, or a plan from
    load_plan() (keeps DO_/CONDITION_ items, altitudes and radii).
    If the vehicle is on the ground it takes off in GUIDED first. Before AUTO
    it is parked over the first waypoint so every run starts the same way.
    """
    plan = wps_ne if isinstance(wps_ne, dict) else dict(wps=list(wps_ne), items=None, alt=None)
    wps_ne = plan["wps"]
    if params:
        veh.set_params(params)
    pos = veh.position()
    veh.pump()
    if origin is None:
        origin = (pos.lat / 1e7, pos.lon / 1e7)
    lat0, lon0 = origin
    if not veh.armed() or pos.relative_alt / 1000.0 < 1.0:
        veh.log(f"taking off to {alt} m")
        veh.takeoff(alt)
    else:
        veh.set_mode(MODE_GUIDED)
    start = ne_offset(lat0, lon0, *wps_ne[0])
    veh.log("moving to start point")
    veh.goto(start[0], start[1], alt)

    # Mission starts at the first waypoint after the start point; TAKEOFF is
    # kept (already at altitude: completes at once) so ground starts work too.
    if plan["items"] is None:
        items = build_items(start[0], start[1], [(0, 0)] + [(n - wps_ne[0][0], e - wps_ne[0][1])
                                                            for n, e in wps_ne[1:]], alt, radius)
    else:
        items = build_plan_items(lat0, lon0, plan, alt, radius)
    veh.upload_mission(items)
    if veh.firmware == "apx":
        veh.m.mav.mission_set_current_send(veh.m.target_system, veh.m.target_component, 1)
    veh.log(f"AUTO: {len(wps_ne) - 1} legs, {len(items)} items")
    veh.set_mode(MODE_AUTO)
    if veh.firmware == "apx":
        # AUTO entered while armed in the air starts the mission; MISSION_START is harmless.
        veh.command(mavutil.mavlink.MAV_CMD_MISSION_START, retries=1)

    samples = []
    reached = []
    t_start = time.time()
    last_seq = max(i for i, it in enumerate(items) if it["cmd"] in POSITIONAL or i == 0)
    done_at = None
    boot0 = None
    while time.time() - t_start < timeout:
        for msg in veh.pump(0.05):
            t = msg.get_type()
            now = time.time() - t_start
            if t == "GLOBAL_POSITION_INT":
                n, e = latlon_to_ne(lat0, lon0, msg.lat / 1e7, msg.lon / 1e7)
                att = veh.last.get("ATTITUDE")
                # vehicle clock, not arrival time: TCP delivers in bursts
                if boot0 is None:
                    boot0 = msg.time_boot_ms / 1000.0 - now
                tb = msg.time_boot_ms / 1000.0 - boot0
                if samples and tb <= samples[-1]["t"]:
                    continue
                samples.append(dict(t=tb, n=n, e=e, alt=msg.relative_alt / 1000.0,
                                    vn=msg.vx / 100.0, ve=msg.vy / 100.0, vd=msg.vz / 100.0,
                                    roll=math.degrees(att.roll) if att else 0.0,
                                    pitch=math.degrees(att.pitch) if att else 0.0,
                                    yaw=math.degrees(att.yaw) if att else 0.0,
                                    yawrate=math.degrees(att.yawspeed) if att else 0.0))
            elif t == "MISSION_ITEM_REACHED":
                if not reached or reached[-1][0] != msg.seq:
                    reached.append((msg.seq, now))
                    if msg.seq >= last_seq and done_at is None:
                        done_at = now
        if done_at is not None and time.time() - t_start > done_at + 4.0:
            break
        if samples and done_at is None and len(samples) > 50:
            # fallback end: stopped at the final waypoint
            s = samples[-1]
            fn, fe = wps_ne[-1]
            if math.hypot(s["n"] - fn, s["e"] - fe) < radius and math.hypot(s["vn"], s["ve"]) < 0.2:
                done_at = s["t"]
    else:
        veh.log("WARNING: mission timed out")
    veh.set_mode(MODE_GUIDED)
    return dict(firmware=veh.firmware, wps=wps_ne, alt=alt, radius=radius, params=params or {},
                samples=samples, reached=reached, done_at=done_at)


# --- analysis ------------------------------------------------------------------------

def _smooth(xs, k=2):
    out = []
    for i in range(len(xs)):
        lo, hi = max(0, i - k), min(len(xs), i + k + 1)
        out.append(sum(xs[lo:hi]) / (hi - lo))
    return out


def resample(samples, hz=10.0):
    """Linear interpolation onto a uniform clock so APX (10 Hz) and apx
    (20 Hz) telemetry give comparable accel/jerk numbers."""
    if len(samples) < 2:
        return samples
    out = []
    keys = [k for k in samples[0] if k != "t"]
    t = samples[0]["t"]
    j = 0
    while t <= samples[-1]["t"]:
        while samples[j + 1]["t"] < t:
            j += 1
        a, b = samples[j], samples[j + 1]
        f = (t - a["t"]) / max(b["t"] - a["t"], 1e-6)
        x = {k: a[k] + f * (b[k] - a[k]) for k in keys}
        # yaw wraps: interpolate the short way
        dy = (b["yaw"] - a["yaw"] + 180.0) % 360.0 - 180.0
        x["yaw"] = a["yaw"] + f * dy
        x["t"] = t
        out.append(x)
        t += 1.0 / hz
    return out


def analyze(run, cruise=None):
    """Metrics for one run. cruise: commanded WPNAV speed (m/s) if known."""
    wps = run["wps"]
    s = resample(run["samples"])
    if len(s) < 10:
        return dict(error="no samples")
    # Mission window: from leaving the start point to arriving at the end.
    t0 = next((x["t"] for x in s if math.hypot(x["vn"], x["ve"]) > 0.3), s[0]["t"])
    t_end = run["done_at"] if run["done_at"] else s[-1]["t"]
    seg = [x for x in s if t0 <= x["t"] <= t_end]
    if len(seg) < 10:
        return dict(error="mission did not run")
    spd = [math.hypot(x["vn"], x["ve"]) for x in seg]
    tt = [x["t"] for x in seg]
    vmax = max(spd)
    # cruise: the WPNAV speed, unless the mission flew slower (DO_CHANGE_SPEED)
    p95 = sorted(spd)[int(0.95 * (len(spd) - 1))]
    cruise = min(cruise, p95) if cruise else p95
    if cruise < 0.2:
        return dict(error="vehicle did not move in AUTO")

    # accel / jerk from smoothed velocity
    vn = _smooth([x["vn"] for x in seg])
    ve = _smooth([x["ve"] for x in seg])
    acc, jerk = [], []
    for i in range(1, len(seg)):
        dt = max(tt[i] - tt[i - 1], 1e-3)
        acc.append(math.hypot(vn[i] - vn[i - 1], ve[i] - ve[i - 1]) / dt)
    acc_s = _smooth(acc)
    for i in range(1, len(acc_s)):
        dt = max(tt[i + 1] - tt[i], 1e-3)
        jerk.append(abs(acc_s[i] - acc_s[i - 1]) / dt)

    # cross-track error to the polyline
    xte = [min(dist_point_segment((x["n"], x["e"]), wps[i], wps[i + 1]) for i in range(len(wps) - 1))
           for x in seg]

    # per-waypoint transitions (interior waypoints only)
    corners = []
    k_prev = 0
    for i in range(1, len(wps) - 1):
        wp = wps[i]
        d = [math.hypot(x["n"] - wp[0], x["e"] - wp[1]) for x in seg]
        # Waypoints are passed in order: search forward from the previous one
        # (closed loops revisit the same spot). First approach within reach,
        # then its local minimum.
        legs = min(math.hypot(wp[0] - wps[i - 1][0], wp[1] - wps[i - 1][1]),
                   math.hypot(wps[i + 1][0] - wp[0], wps[i + 1][1] - wp[1]))
        near = max(1.5 * run["radius"], min(5.0, 0.5 * legs))
        k = next((j for j in range(k_prev, len(d)) if d[j] < near), None)
        if k is None:
            k = min(range(k_prev, len(d)), key=d.__getitem__)
        while k + 1 < len(d) and d[k + 1] <= d[k]:
            k += 1
        k_prev = k
        # window: within max(8 m, 3*radius) of the waypoint
        win = max(8.0, 3.0 * run["radius"])
        lo = k
        while lo > 0 and d[lo - 1] < win:
            lo -= 1
        hi = k
        while hi < len(d) - 1 and d[hi + 1] < win:
            hi += 1
        vwin = spd[lo:hi + 1]
        vmin = min(vwin)
        corners.append(dict(wp=i, turn_deg=round(turn_angle_deg(wps[i - 1], wp, wps[i + 1]), 1),
                            pass_dist=round(d[k], 2), v_at_wp=round(spd[k], 2), v_min=round(vmin, 2),
                            dip=round(1.0 - vmin / cruise, 3),
                            stopped=vmin < 0.5, t_window=round(tt[hi] - tt[lo], 2)))

    # "stops": samples slower than 0.5 m/s away from start/end
    stops = 0
    in_stop = False
    for x, v in zip(seg, spd):
        far = math.hypot(x["n"] - wps[0][0], x["e"] - wps[0][1]) > 3 and \
            math.hypot(x["n"] - wps[-1][0], x["e"] - wps[-1][1]) > 3
        if far and v < 0.5 and not in_stop:
            stops += 1
        in_stop = far and v < 0.5

    path_len = sum(math.hypot(wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1])
                   for i in range(len(wps) - 1))
    fin = seg[-1]
    # overshoot beyond the final waypoint along the last leg
    a, b = wps[-2], wps[-1]
    L = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
    ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
    # only after passing the last corner (closed loops start at the end point)
    tail = s[next((j for j, x in enumerate(s) if x["t"] >= seg[k_prev]["t"]), 0):] if corners else s
    overshoot = max(0.0, max((x["n"] - b[0]) * ux + (x["e"] - b[1]) * uy for x in tail))
    m = dict(
        firmware=run["firmware"],
        mission_time_s=round(t_end - t0, 2),
        path_len_m=round(path_len, 1),
        mean_speed=round(path_len / max(t_end - t0, 1e-3), 2),
        max_speed=round(vmax, 2),
        stops=stops,
        mean_corner_dip=round(sum(c["dip"] for c in corners) / len(corners), 3) if corners else 0.0,
        worst_corner_dip=round(max((c["dip"] for c in corners), default=0.0), 3),
        max_accel=round(max(acc_s), 2),
        rms_jerk=round(math.sqrt(sum(j * j for j in jerk) / len(jerk)), 2),
        max_jerk=round(max(jerk), 2),
        max_xte=round(max(xte), 2),
        mean_xte=round(sum(xte) / len(xte), 2),
        max_tilt=round(max(math.hypot(x["roll"], x["pitch"]) for x in seg), 1),
        max_yawrate=round(max(abs((seg[i]["yaw"] - seg[i - 1]["yaw"] + 180.0) % 360.0 - 180.0) /
                              max(seg[i]["t"] - seg[i - 1]["t"], 1e-3) for i in range(1, len(seg))), 1),
        final_overshoot_m=round(overshoot, 2),
        final_err_m=round(math.hypot(fin["n"] - b[0], fin["e"] - b[1]), 2),
        corners=corners,
    )
    m["score"] = score(m)
    return m


def score(m, w=None):
    """Lower is better. Balances speed, smooth corners, jerk and tracking."""
    w = w or dict(time=1.0, dip=40.0, stop=25.0, jerk=2.0, xte=4.0, overshoot=10.0)
    return round(w["time"] * m["mission_time_s"] * (10.0 / max(m["path_len_m"], 1.0)) * 10.0
                 + w["dip"] * m["mean_corner_dip"] + w["stop"] * m["stops"]
                 + w["jerk"] * m["rms_jerk"] + w["xte"] * max(0.0, m["max_xte"] - 1.0)
                 + w["overshoot"] * m["final_overshoot_m"], 2)


# --- output -----------------------------------------------------------------------------

def save_csv(run, path):
    keys = ["t", "n", "e", "alt", "vn", "ve", "vd", "roll", "pitch", "yaw", "yawrate"]
    with open(path, "w", newline="") as f:
        f.write("# " + json.dumps(dict(firmware=run["firmware"], wps=run["wps"], alt=run["alt"],
                                       radius=run["radius"], params=run["params"],
                                       reached=run["reached"], done_at=run["done_at"])) + "\n")
        wr = csv.writer(f)
        wr.writerow(keys)
        for x in run["samples"]:
            wr.writerow([f"{x[k]:.4f}" for k in keys])


def load_csv(path):
    with open(path) as f:
        meta = json.loads(f.readline()[2:])
        rd = csv.DictReader(f)
        samples = [{k: float(v) for k, v in row.items()} for row in rd]
    meta["samples"] = samples
    meta["wps"] = [tuple(p) for p in meta["wps"]]
    return meta


def print_report(m, label=""):
    if "error" in m:
        print(f"{label}: {m['error']}")
        return
    print(f"\n=== {label or m['firmware']} ===")
    print(f"  mission time {m['mission_time_s']} s  path {m['path_len_m']} m  mean {m['mean_speed']} m/s"
          f"  max {m['max_speed']} m/s")
    print(f"  stops {m['stops']}  corner dip mean {m['mean_corner_dip']:.0%} worst {m['worst_corner_dip']:.0%}")
    print(f"  accel max {m['max_accel']} m/s2  jerk rms {m['rms_jerk']} max {m['max_jerk']} m/s3")
    print(f"  xte max {m['max_xte']} mean {m['mean_xte']} m  tilt max {m['max_tilt']} deg"
          f"  yawrate max {m['max_yawrate']} deg/s")
    print(f"  final overshoot {m['final_overshoot_m']} m  final err {m['final_err_m']} m  SCORE {m['score']}")
    print("  wp  turn   pass   v@wp  vmin   dip  stop")
    for c in m["corners"]:
        print(f"  {c['wp']:>2} {c['turn_deg']:>5.0f}° {c['pass_dist']:>6.2f} {c['v_at_wp']:>5.2f} "
              f"{c['v_min']:>5.2f} {c['dip']:>5.0%}  {'STOP' if c['stopped'] else ''}")


def plot_runs(runs, labels, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed - no plot")
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    wps = runs[0]["wps"]
    ax1.plot([p[1] for p in wps], [p[0] for p in wps], "k--o", lw=1, label="mission")
    for run, lab in zip(runs, labels):
        s = run["samples"]
        ax1.plot([x["e"] for x in s], [x["n"] for x in s], lw=1.5, label=lab)
        t0 = next((x["t"] for x in s if math.hypot(x["vn"], x["ve"]) > 0.3), 0)
        ax2.plot([x["t"] - t0 for x in s], [math.hypot(x["vn"], x["ve"]) for x in s], lw=1.5, label=lab)
    ax1.set_aspect("equal")
    ax1.set_xlabel("east (m)")
    ax1.set_ylabel("north (m)")
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax2.set_xlabel("time since start (s)")
    ax2.set_ylabel("horizontal speed (m/s)")
    ax2.set_xlim(left=0)
    ax2.legend()
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"plot: {path}")
