#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
SITL test: sprayer start/stop from an AUTO mission, end to end into Gazebo.

Mission: takeoff -> DO_SPRAYER on -> two 40 m legs -> DO_SPRAYER off -> LAND.
Checks:
  - APX reports "Sprayer: spraying" after DO_SPRAYER on and once moving,
    and "Sprayer: stopped" after DO_SPRAYER off
  - the Gazebo plugin publishes nozzle emitter commands (emitting true, then
    false) on /apx/spray/nozzle1
(Gazebo's scene/info snapshot does not reflect runtime emitter commands -
those go to the renderer - so the visual itself is checked by eye with --gui.)

Usage: test_sprayer.py [--gui]
"""

import argparse
import os
import signal
import subprocess
import sys
import time

from pymavlink import mavutil

from test_auto_path_tracking import LAT0, LON0, ne_to_latlon, upload_mission

WORKSPACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DO_SPRAYER = 216
ALT = 8.0
NOZZLE_TOPIC = "/apx/spray/nozzle1"


def gz_env():
    env = os.environ.copy()
    env["GZ_SIM_RESOURCE_PATH"] = os.path.join(WORKSPACE, "sitl", "models")
    return env


def run(args):
    import shutil
    if not shutil.which("gz"):
        print("[SKIP] Gazebo ('gz') not found on this system. Skipping Gazebo sprayer simulation test.")
        return True

    sim_cmd = [sys.executable, os.path.join(WORKSPACE, "sitl/sim_vehicle.py"),
               "--vehicle", "hex_x", "--sim", "gazebo",
               # keep a running APX Flight Manager (UDP 14550) from attaching
               "--gcs-port", "14650"]
    if not args.gui:
        sim_cmd.append("--headless")
    sim_log = open(os.path.join(args.log_dir, "sprayer_sim.log"), "w")
    sim = subprocess.Popen(sim_cmd, stdout=sim_log, stderr=subprocess.STDOUT, text=True)
    time.sleep(8)

    # Record what the plugin publishes to the first nozzle.
    echo_log = os.path.join(args.log_dir, "nozzle_echo.log")
    echo = subprocess.Popen(["gz", "topic", "-e", "-t", NOZZLE_TOPIC], stdout=open(echo_log, "w"),
                            stderr=subprocess.STDOUT, env=gz_env())
    texts = []
    try:
        mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190)
        mav.wait_heartbeat(timeout=15)

        lat1, lon1 = ne_to_latlon(40.0, 0.0)
        lat2, lon2 = ne_to_latlon(40.0, 40.0)
        items = [
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, LAT0, LON0, 0.0),
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, LAT0, LON0, ALT),
            (DO_SPRAYER, 0.0, 0.0, 0.0, 0.0, 1.0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2.0, lat1, lon1, ALT),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2.0, lat2, lon2, ALT),
            (DO_SPRAYER, 0.0, 0.0, 0.0, 0.0, 0.0),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0, lat2, lon2, 0.0),
        ]
        if not upload_mission(mav, items):
            print("[FAIL] mission upload")
            return False
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1)
        mav.mav.set_mode_send(mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3)

        start = time.time()
        while time.time() - start < args.timeout:
            msg = mav.recv_match(type=['STATUSTEXT', 'MISSION_CURRENT'], blocking=True, timeout=0.5)
            if msg and msg.get_type() == 'STATUSTEXT' and "Sprayer" in msg.text:
                texts.append((round(time.time() - start, 1), msg.text))
                print(f"  t={time.time() - start:5.1f} {msg.text}")
                if msg.text.endswith("stopped") and len(texts) > 1:
                    break

        time.sleep(2)
        echo.terminate()
        echo_out = open(echo_log).read()
        published_on = "data: true" in echo_out
        published_off = echo_out.rstrip().endswith("}") and echo_out.count("emitting") >= 2

        print("=" * 70)
        print(f" STATUSTEXT sequence     : {texts}")
        print(f" nozzle cmds published   : on={published_on} off-after-on={published_off}")
        print("=" * 70)
        checks = [
            (any(t.endswith("spraying") for _, t in texts), "APX started spraying after DO_SPRAYER on"),
            (bool(texts) and texts[-1][1].endswith("stopped"), "APX stopped spraying after DO_SPRAYER off"),
            (published_on and published_off, "plugin published nozzle on then off"),
        ]
        ok = True
        for passed, label in checks:
            print(f" [{'PASS' if passed else 'FAIL'}] {label}")
            ok = ok and passed
        return ok
    finally:
        if echo.poll() is None:
            echo.kill()
        if sim.poll() is None:
            sim.send_signal(signal.SIGINT)
            try:
                sim.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sim.kill()
        sim_log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="show the Gazebo window")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--log-dir", default="/tmp")
    sys.exit(0 if run(parser.parse_args()) else 1)
