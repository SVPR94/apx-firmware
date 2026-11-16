#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
SITL test: LOITER landing, stick disarm and DISARM_DELAY.

Sticks come from MANUAL_CONTROL (GCS virtual joystick), streamed at 20 Hz.
  1. arm (throttle down + yaw right) in LOITER, take off, hover
  2. disarm gesture (throttle down + yaw left) in the air -> must be ignored
  3. land with throttle at zero -> must stay armed (motors idle)
  4. disarm gesture on the ground -> must disarm
  5. re-arm on the ground, leave throttle at zero -> auto-disarm after
     DISARM_DELAY (set to --delay for the test)
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

WORKSPACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOITER = 5


class Sticks:
    """Streams MANUAL_CONTROL; throttle z 0..1000, yaw r -1000..1000."""

    def __init__(self, mav):
        self.mav = mav
        self.z = 0
        self.r = 0
        self.running = True
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def set(self, z, r=0):
        with self.lock:
            self.z, self.r = z, r

    def _loop(self):
        while self.running:
            with self.lock:
                z, r = self.z, self.r
            self.mav.mav.manual_control_send(self.mav.target_system, 0, 0, z, r, 0)
            time.sleep(0.05)


def run(args):
    sim_log = open(os.path.join(args.log_dir, "loiter_sim.log"), "w")
    sim = subprocess.Popen([sys.executable, os.path.join(WORKSPACE, "sitl/sim_vehicle.py"),
                            "--vehicle", args.vehicle, "--sim", "builtin",
                            # keep a running APX Flight Manager (UDP 14550) from attaching:
                            # its virtual joystick would override these sticks
                            "--gcs-port", "14650"],
                           stdout=sim_log, stderr=subprocess.STDOUT, text=True)
    time.sleep(4)
    state = {"armed": False, "alt": 0.0, "vz": 0.0}
    mav = None
    try:
        mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190)
        mav.wait_heartbeat(timeout=15)
        mav.mav.param_set_send(mav.target_system, mav.target_component, b"DISARM_DELAY", args.delay,
                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.5)
        mav.mav.set_mode_send(mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LOITER)
        sticks = Sticks(mav)

        def pump(seconds):
            end = time.time() + seconds
            while time.time() < end:
                m = mav.recv_match(type=['HEARTBEAT', 'GLOBAL_POSITION_INT'], blocking=True, timeout=0.3)
                if not m:
                    continue
                if m.get_type() == 'HEARTBEAT' and m.get_srcSystem() == mav.target_system:
                    state["armed"] = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif m.get_type() == 'GLOBAL_POSITION_INT':
                    state["alt"] = m.relative_alt / 1000.0
                    state["vz"] = m.vz / 100.0

        results = []

        def check(cond, label):
            results.append((cond, label))
            print(f"  [{'PASS' if cond else 'FAIL'}] {label} (armed={state['armed']} alt={state['alt']:.2f})")

        # 1. arm by stick, take off, hover
        sticks.set(0, 1000)
        pump(2.5)
        sticks.set(0, 0)
        pump(1.0)
        check(state["armed"], "armed by stick gesture in LOITER")
        sticks.set(850)
        pump(5.0)
        sticks.set(500)
        pump(2.0)
        check(state["alt"] > 3.0, "took off with throttle up")

        # 2. disarm gesture in the air must be ignored
        sticks.set(0, -1000)
        pump(2.5)
        sticks.set(500, 0)
        pump(1.0)
        check(state["armed"], "disarm gesture ignored while airborne")

        # 3. land with throttle at zero; must stay armed afterwards
        sticks.set(0, 0)
        t0 = time.time()
        while time.time() - t0 < 40 and not (state["alt"] < 1.0 and abs(state["vz"]) < 0.1):
            pump(0.5)
        pump(5.0)
        check(state["armed"], "still armed 5 s after touchdown (motors idle)")

        # 4. disarm gesture on the ground
        sticks.set(0, -1000)
        pump(2.5)
        sticks.set(0, 0)
        pump(1.0)
        check(not state["armed"], "disarmed by stick gesture on the ground")

        # 5. re-arm on the ground, throttle stays at zero -> DISARM_DELAY
        sticks.set(0, 1000)
        pump(2.5)
        sticks.set(0, 0)
        armed_at = time.time()
        pump(0.5)
        check(state["armed"], "re-armed on the ground")
        while time.time() - armed_at < args.delay + 10 and state["armed"]:
            pump(0.5)
        elapsed = time.time() - armed_at
        check(not state["armed"] and args.delay - 1.5 <= elapsed <= args.delay + 3.0,
              f"auto-disarmed after DISARM_DELAY={args.delay:.0f}s at zero throttle (took {elapsed:.1f}s)")
        sticks.running = False
        return all(c for c, _ in results)
    finally:
        if sim.poll() is None:
            sim.send_signal(signal.SIGINT)
            try:
                sim.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sim.kill()
        sim_log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default="hex_x", choices=["quad_x", "hex_x", "octo_x"])
    parser.add_argument("--delay", type=float, default=10.0, help="DISARM_DELAY to test with (s)")
    parser.add_argument("--log-dir", default="/tmp")
    sys.exit(0 if run(parser.parse_args()) else 1)
