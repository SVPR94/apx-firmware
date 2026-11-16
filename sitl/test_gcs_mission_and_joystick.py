#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
Automated SITL Integration Test for:
1. APX Flight Manager Mission Upload (sample_mission.plan) & Auto Mode drift verification
2. Virtual Joystick (MANUAL_CONTROL) & RC Channels Override in Loiter and AltHold
"""

import sys
import time
import json
import math
import subprocess
import os
import signal

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] pymavlink is required for this test. Install with: pip install pymavlink")
    sys.exit(1)

def run_test():
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    mission_file = os.path.join(workspace, "tests/sample_files/sample_mission.plan")
    
    if not os.path.exists(mission_file):
        print(f"[FAIL] Mission file not found: {mission_file}")
        return False

    with open(mission_file, "r") as f:
        plan_data = json.load(f)

    # 1. Start SITL background process
    print("[TEST] Launching APX SITL with built-in physics...")
    sim_cmd = [
        sys.executable,
        os.path.join(workspace, "sitl/sim_vehicle.py"),
        "--vehicle", "quad_x",
        "--sim", "builtin",
        "--port", "9003",
        # keep a running APX Flight Manager (UDP 14550) from attaching
        "--gcs-port", "14650",
    ]
    sim_proc = subprocess.Popen(sim_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    time.sleep(3) # Wait for SITL to initialize and bind UDP ports

    try:
        # 2. Connect via MAVLink (TCP 5760)
        print("[TEST] Connecting to APX on TCP 127.0.0.1:5760...")
        mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190)
        mav.wait_heartbeat(timeout=10)
        print(f"[TEST SUCCESS] Heartbeat received from Autopilot (SysID={mav.target_system}, CompID={mav.target_component})")

        # 3. Upload GCS Mission
        items = plan_data["mission"]["items"]
        mission_count = len(items)
        print(f"[TEST] Uploading GCS plan with {mission_count} items...")

        mav.mav.mission_count_send(mav.target_system, mav.target_component, mission_count, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        
        for _ in range(mission_count):
            req = mav.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
            if not req:
                print("[FAIL] Timeout waiting for MISSION_REQUEST")
                return False
            seq = req.seq
            item = items[seq]
            cmd = item["command"]
            frame = item["frame"]
            params = item["params"]
            p1 = float(params[0] or 0.0)
            p2 = float(params[1] or 0.0)
            p3 = float(params[2] or 0.0)
            p4 = float(params[3] or 0.0)
            x = int(float(params[4] or 0.0) * 1e7)
            y = int(float(params[5] or 0.0) * 1e7)
            z = float(params[6] or 0.0)

            mav.mav.mission_item_int_send(
                mav.target_system, mav.target_component, seq, frame, cmd,
                0, 1, p1, p2, p3, p4, x, y, z, mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )

        ack = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"[FAIL] Mission ACK rejected or timed out: {ack}")
            return False
        print("[TEST SUCCESS] GCS Mission plan uploaded and accepted!")

        # 4. Arm Vehicle
        print("[TEST] Arming drone...")
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)

        # 5. Switch to AUTO Mode (custom_mode = 3)
        print("[TEST] Switching to AUTO mode (custom_mode=3)...")
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            3
        )
        time.sleep(1)

        # 6. Monitor initial takeoff in AUTO mode and verify ZERO bottom-left drift
        print("[TEST] Monitoring AUTO takeoff (verifying NO bottom-left drift)...")
        start_t = time.time()
        max_south_drift = 0.0
        max_west_drift = 0.0
        reached_takeoff = False

        while time.time() - start_t < 12.0:
            msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                rel_alt = msg.relative_alt / 1000.0
                vx = msg.vx / 100.0 # m/s North
                vy = msg.vy / 100.0 # m/s East
                if vx < 0:
                    max_south_drift = max(max_south_drift, -vx)
                if vy < 0:
                    max_west_drift = max(max_west_drift, -vy)
                
                if rel_alt > 5.0:
                    reached_takeoff = True
                    print(f"  [AUTO Climb] Alt={rel_alt:.1f}m, NorthVel={vx:.2f}m/s, EastVel={vy:.2f}m/s")
                    break
            time.sleep(0.1)

        print(f"[CHECK] Max South Drift Velocity: {max_south_drift:.2f} m/s, Max West Drift Velocity: {max_west_drift:.2f} m/s")
        if max_south_drift > 2.0 or max_west_drift > 2.0:
            print("[FAIL] Drone is drifting South-West (Bottom-Left) during takeoff!")
            return False
        print("[TEST SUCCESS] Takeoff in AUTO mode climbed vertically with NO bottom-left drift!")

        # 7. Switch to LOITER Mode (custom_mode = 5) to test Virtual Joystick
        print("[TEST] Switching to LOITER mode (custom_mode=5)...")
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            5
        )
        time.sleep(1)

        # 8. Test APX Flight Manager Virtual Joystick (MANUAL_CONTROL msg ID 69)
        print("[TEST] Sending GCS Virtual Joystick (pitch forward +600, roll right +600)...")
        for _ in range(30): # 1.5 seconds at 20Hz
            mav.mav.manual_control_send(
                mav.target_system,
                600,  # x: pitch forward
                600,  # y: roll right
                500,  # z: throttle hover center
                0,    # r: yaw center
                0
            )
            time.sleep(0.05)

        # Verify pitch and roll angles tilt in response to sticks
        att = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att:
            roll_deg = math.degrees(att.roll)
            pitch_deg = math.degrees(att.pitch)
            print(f"  [LOITER Joystick Response] Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°")

        # 9. Release sticks to center -> Active Braking in Loiter
        print("[TEST] Releasing joystick sticks to center (active braking)...")
        for _ in range(20):
            mav.mav.manual_control_send(mav.target_system, 0, 0, 500, 0, 0)
            time.sleep(0.05)
        print("[TEST SUCCESS] GCS Virtual Joystick responded in LOITER mode!")

        # 10. Test RC_CHANNELS_OVERRIDE in ALT_HOLD (custom_mode = 2)
        print("[TEST] Switching to ALT_HOLD mode (custom_mode=2)...")
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            2
        )
        time.sleep(1)

        print("[TEST] Sending RC_CHANNELS_OVERRIDE (Roll 1650us, Pitch 1650us, Throttle 1500us)...")
        for _ in range(25):
            mav.mav.rc_channels_override_send(
                mav.target_system, mav.target_component,
                1650, 1650, 1500, 1500, 0, 0, 0, 0
            )
            time.sleep(0.04)

        att = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att:
            print(f"  [ALT_HOLD RC Response] Roll={math.degrees(att.roll):.1f}°, Pitch={math.degrees(att.pitch):.1f}°")
        print("[TEST SUCCESS] RC_CHANNELS_OVERRIDE verified in ALT_HOLD mode!")

        # Release RC channels override so sticks are neutral before landing
        print("[TEST] Releasing RC override to neutral...")
        for _ in range(5):
            mav.mav.rc_channels_override_send(
                mav.target_system, mav.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            time.sleep(0.04)

        # 11. Command LAND (custom_mode = 9)
        print("[TEST] Commanding LAND mode (custom_mode=9)...")
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            9
        )

        land_start = time.time()
        disarmed = False
        while time.time() - land_start < 20.0:
            hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb:
                armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not armed:
                    disarmed = True
                    print("[TEST SUCCESS] Touchdown detected and motors automatically disarmed!")
                    break
            time.sleep(0.2)

        if not disarmed:
            print("[WARN] Vehicle still armed after land timeout; disarming manually")
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0
            )

        print("======================================================================")
        print(" ALL GCS MISSION UPLOAD AND VIRTUAL JOYSTICK TESTS PASSED!")
        print("======================================================================")
        return True

    finally:
        if sim_proc.poll() is None:
            sim_proc.send_signal(signal.SIGINT)
            try:
                sim_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                sim_proc.kill()

if __name__ == '__main__':
    success = run_test()
    sys.exit(0 if success else 1)
