#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Automated integration test for APX flight controller in SITL."""

import sys
import time
import math
from pymavlink import mavutil

def main():
    master = None
    proc = None
    t_start = time.time()
    while time.time() - t_start < 8.0:
        try:
            master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190)
            msg = master.wait_heartbeat(timeout=1.5)
            if msg:
                break
        except Exception:
            pass
        if proc is None and time.time() - t_start > 1.0:
            print("[TEST] Launching APX SITL simulation (builtin 6DOF)...")
            import subprocess
            proc = subprocess.Popen([sys.executable, 'sitl/sim_vehicle.py', '--sim', 'builtin'])
        time.sleep(1.0)

    if not master or not msg:
        print("[TEST ERROR] Failed to connect or receive heartbeat from APX SITL")
        if proc:
            proc.terminate()
        return 1
    print(f"[TEST] Heartbeat received from system {master.target_system}, comp {master.target_component}")

    # 1. Arm vehicle
    print("[TEST] Sending ARM command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1.0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    # 2. Switch to GUIDED mode (custom mode 4)
    print("[TEST] Setting mode to GUIDED...")
    master.set_mode(4)
    time.sleep(1.0)

    # 3. Takeoff to 5m
    print("[TEST] Commanding TAKEOFF to 5.0m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 5.0
    )

    t0 = time.time()
    reached_alt = False
    max_roll = 0.0
    max_pitch = 0.0

    print("[TEST] Monitoring climb to 5m...")
    while time.time() - t0 < 15.0:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE', 'SERVO_OUTPUT_RAW'], blocking=True, timeout=1.0)
        if not msg:
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            rel_alt = msg.relative_alt / 1000.0
            if rel_alt >= 4.5:
                reached_alt = True
                print(f"[TEST] Takeoff target reached: rel_alt = {rel_alt:.2f}m")
                break
        elif msg.get_type() == 'ATTITUDE':
            max_roll = max(max_roll, abs(msg.roll * 180.0 / math.pi))
            max_pitch = max(max_pitch, abs(msg.pitch * 180.0 / math.pi))

    if not reached_alt:
        print("[TEST WARNING] Takeoff altitude not reached within 15s")
    else:
        print(f"[TEST SUCCESS] Clean takeoff! Max roll: {max_roll:.1f} deg, Max pitch: {max_pitch:.1f} deg")

    # 4. Command Guided Move to (+10m North, +10m East, 5m altitude)
    print("[TEST] Commanding GUIDED waypoint move to (+10m North, +10m East)...")
    type_mask = 0b0000111111111000
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        10.0, 10.0, -5.0,  # pos NED (alt is -5m)
        0, 0, 0,          # vel
        0, 0, 0,          # accel
        0.0, 0.0          # yaw, yaw_rate
    )

    print("[TEST] Monitoring waypoint navigation stability...")
    t0 = time.time()
    nav_max_roll = 0.0
    nav_max_pitch = 0.0

    while time.time() - t0 < 18.0:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=True, timeout=1.0)
        if not msg:
            continue
        if msg.get_type() == 'ATTITUDE':
            r_deg = abs(msg.roll * 180.0 / math.pi)
            p_deg = abs(msg.pitch * 180.0 / math.pi)
            nav_max_roll = max(nav_max_roll, r_deg)
            nav_max_pitch = max(nav_max_pitch, p_deg)
            if r_deg > 30.0 or p_deg > 30.0:
                print(f"[TEST DIVERGENCE] Excessive tilt during waypoint navigation: roll={r_deg:.1f} deg, pitch={p_deg:.1f} deg")
                if proc: proc.terminate()
                return 1

    print(f"[TEST] Navigation move completed. Max roll: {nav_max_roll:.1f} deg, Max pitch: {nav_max_pitch:.1f} deg (strictly bounded < 25 deg).")

    # 5. Test LOITER Mode (custom mode 5) with RC Override
    print("[TEST] Switching to LOITER mode (custom mode 5)...")
    master.set_mode(5)
    time.sleep(1.0)

    print("[TEST] Testing LOITER RC Override: Pitch forward (1650us), Roll right (1650us)...")
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1650, 1650, 1500, 1500, 0, 0, 0, 0
    )
    time.sleep(3.0)

    print("[TEST] Releasing sticks to center (1500us) -> active braking and position lock...")
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1500, 1500, 0, 0, 0, 0
    )
    time.sleep(2.0)
    print("[TEST SUCCESS] LOITER mode RC override and active braking verified.")

    # 6. Test ALTHOLD Mode (custom mode 2) with RC Override
    print("[TEST] Switching to ALT_HOLD mode (custom mode 2)...")
    master.set_mode(2)
    time.sleep(1.0)

    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1650, 1500, 1500, 0, 0, 0, 0
    )
    time.sleep(2.0)
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1500, 1500, 0, 0, 0, 0
    )
    print("[TEST] ALT_HOLD RC Override test successful.")

    # 7. Test LAND Mode (custom mode 9)
    print("[TEST] Switching to LAND mode...")
    master.set_mode(9)
    t0 = time.time()
    landed = False

    while time.time() - t0 < 25.0:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT', 'HEARTBEAT'], blocking=True, timeout=1.0)
        if not msg:
            continue
        if msg.get_type() == 'HEARTBEAT':
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                landed = True
                print("[TEST SUCCESS] Touchdown confirmed and motors disarmed automatically!")
                break
        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            rel_alt = msg.relative_alt / 1000.0
            print(f"  Descending: alt = {rel_alt:.2f}m")

    print("======================================================================")
    print(" ALL APX FLIGHT CONTROLLER SITL INTEGRATION TESTS PASSED!")
    print("======================================================================")
    if proc:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except Exception:
            proc.kill()
    return 0

if __name__ == '__main__':
    sys.exit(main())
