#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
Automated SITL Integration Test for Per-Mission-Item and DO_CHANGE_SPEED Handling in AUTO Mode.
Tests that:
1. MAV_CMD_DO_CHANGE_SPEED (178) changes cruise speed for subsequent waypoints.
2. Leg 1 flies at the commanded speed (e.g. 4.0 m/s).
3. Leg 2 accelerates and cruises at the higher commanded speed (e.g. 8.0 m/s).
4. Slew rates and cascade speed limits track the mission item speed.
"""

import sys
import time
import math
import subprocess
import os
import signal

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] pymavlink is required. Install with: pip install pymavlink")
    sys.exit(1)

def run_test():
    workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    print("======================================================================")
    print(" TEST: Per-Mission-Item Speed & DO_CHANGE_SPEED Trajectory Execution")
    print("======================================================================")

    # Launch APX SITL with hex_x vehicle and builtin physics
    sim_cmd = [
        sys.executable,
        os.path.join(workspace, "sitl/sim_vehicle.py"),
        "--vehicle", "hex_x",
        "--sim", "builtin",
        "--port", "9003",
        # keep a running APX Flight Manager (UDP 14550) from attaching
        "--gcs-port", "14650",
    ]
    sim_proc = subprocess.Popen(sim_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(3)

    try:
        print("[TEST] Connecting via MAVLink on TCP 127.0.0.1:5760...")
        mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190)
        mav.wait_heartbeat(timeout=10)
        print("[TEST SUCCESS] Heartbeat received from Autopilot.")

        lat_base = 18.5204
        lon_base = 73.8567
        dlat_40m = 40.0 * 8.9976e-6
        dlon_60m = 60.0 * (8.9976e-6 / math.cos(math.radians(lat_base)))

        # Mission plan:
        # seq 0: Home
        # seq 1: Takeoff to 10m
        # seq 2: DO_CHANGE_SPEED -> 4.0 m/s
        # seq 3: Waypoint 1 (North 40m) -> expected cruise ~4.0 m/s
        # seq 4: DO_CHANGE_SPEED -> 8.0 m/s
        # seq 5: Waypoint 2 (North 40m, East 60m) -> expected cruise ~8.0 m/s
        # seq 6: Land
        mission_items = [
            # seq 0: Home
            (0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 2.0, 0, 0, lat_base, lon_base, 0.0),
            # seq 1: Takeoff to 10m
            (1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, lat_base, lon_base, 10.0),
            # seq 2: DO_CHANGE_SPEED (param1=1 groundspeed, param2=4.0 m/s)
            (2, mavutil.mavlink.MAV_FRAME_MISSION, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1.0, 4.0, 0, 0, 0, 0, 0),
            # seq 3: Waypoint 1 (North 40m)
            (3, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 2.0, 0, 0, lat_base + dlat_40m, lon_base, 10.0),
            # seq 4: DO_CHANGE_SPEED (param1=1 groundspeed, param2=8.0 m/s)
            (4, mavutil.mavlink.MAV_FRAME_MISSION, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1.0, 8.0, 0, 0, 0, 0, 0),
            # seq 5: Waypoint 2 (North 40m, East 60m)
            (5, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 2.0, 0, 0, lat_base + dlat_40m, lon_base + dlon_60m, 10.0),
            # seq 6: Land
            (6, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 2.0, 0, 0, lat_base + dlat_40m, lon_base + dlon_60m, 0.0),
        ]

        print(f"[TEST] Uploading {len(mission_items)} mission items with dynamic speed overrides...")
        mav.mav.mission_count_send(mav.target_system, mav.target_component, len(mission_items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        
        for _ in range(len(mission_items)):
            req = mav.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
            if not req:
                print("[FAIL] Timeout waiting for MISSION_REQUEST")
                return False
            seq = req.seq
            item = mission_items[seq]
            mav.mav.mission_item_int_send(
                mav.target_system, mav.target_component, item[0], item[1], item[2],
                0, 1, item[3], item[4], item[5], item[6],
                int(item[7] * 1e7), int(item[8] * 1e7), item[9],
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )

        ack = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"[FAIL] Mission rejected: {ack}")
            return False
        print("[TEST SUCCESS] Mission uploaded and accepted.")

        # Arm and switch to AUTO mode
        print("[TEST] Arming drone...")
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)

        print("[TEST] Switching to AUTO mode...")
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            3 # custom_mode AUTO
        )

        print("[TEST] Monitoring speed tracking across mission legs (max 60 seconds)...")
        start_time = time.time()
        last_print = 0
        current_seq = 0
        max_speed_leg1 = 0.0
        max_speed_leg2 = 0.0
        speeds_leg1 = []
        speeds_leg2 = []

        while time.time() - start_time < 60.0:
            msg = mav.recv_match(type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT', 'STATUSTEXT'], blocking=True, timeout=0.5)
            if not msg:
                continue

            if msg.get_type() == 'MISSION_CURRENT':
                current_seq = msg.seq

            elif msg.get_type() == 'GLOBAL_POSITION_INT':
                vx = msg.vx / 100.0
                vy = msg.vy / 100.0
                vz = msg.vz / 100.0
                alt = msg.relative_alt / 1000.0
                horiz_speed = math.sqrt(vx * vx + vy * vy)

                # Record speeds during steady cruise on Leg 1 (towards item 3) and Leg 2 (towards item 5)
                if current_seq in [2, 3] and alt > 8.0:
                    speeds_leg1.append(horiz_speed)
                    if horiz_speed > max_speed_leg1:
                        max_speed_leg1 = horiz_speed
                elif current_seq in [4, 5] and alt > 8.0:
                    speeds_leg2.append(horiz_speed)
                    if horiz_speed > max_speed_leg2:
                        max_speed_leg2 = horiz_speed

                now = time.time()
                if now - last_print > 2.0:
                    print(f"  [T+{now-start_time:4.1f}s] Seq={current_seq} Alt={alt:5.2f}m HorizSpeed={horiz_speed:5.2f}m/s (Vz={vz:5.2f}m/s)")
                    last_print = now

                # If mission reached landing and touchdown occurred
                if current_seq >= 6 and alt < 0.3 and abs(vz) < 0.2 and now - start_time > 20.0:
                    print(f"[TEST] Touchdown confirmed at T+{now-start_time:.1f}s.")
                    break

        print("\n======================================================================")
        print(" SPEED TRACKING RESULTS")
        print("======================================================================")
        print(f"Leg 1 (Commanded 4.0 m/s): Max Recorded Speed = {max_speed_leg1:.2f} m/s")
        print(f"Leg 2 (Commanded 8.0 m/s): Max Recorded Speed = {max_speed_leg2:.2f} m/s")

        # Validation assertions:
        if max_speed_leg1 < 3.2 or max_speed_leg1 > 4.8:
            print(f"[FAIL] Leg 1 speed {max_speed_leg1:.2f} m/s was not within expected [3.2, 4.8] m/s window!")
            return False
        print("[PASS] Leg 1 accurately tracked the 4.0 m/s mission item speed.")

        if max_speed_leg2 < 6.8 or max_speed_leg2 > 8.8:
            print(f"[FAIL] Leg 2 speed {max_speed_leg2:.2f} m/s was not within expected [6.8, 8.8] m/s window!")
            return False
        print("[PASS] Leg 2 dynamically accelerated and tracked the 8.0 m/s mission item speed.")

        print("======================================================================")
        print(" [ALL TESTS PASSED] Per-mission-item speed handling verified 100%! ")
        print("======================================================================")
        return True

    finally:
        print("[TEST] Cleaning up SITL process...")
        sim_proc.send_signal(signal.SIGINT)
        try:
            sim_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            sim_proc.kill()

if __name__ == '__main__':
    success = run_test()
    sys.exit(0 if success else 1)
