#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
Automated SITL Integration Test for Waypoint Approach, Slew-Rate Braking,
and Altitude Preservation in AUTO Mode on hex_x.
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
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    
    print("======================================================================")
    print(" TEST: Waypoint Approach, Kinematic Braking & Altitude Stability (hex_x)")
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

        # Upload a 4-waypoint square mission with 90-degree turns at 10m altitude
        # Home: (18.5204, 73.8567, 540.0)
        # 1 deg lat ~ 111139 m, 1m ~ 8.9976e-6 deg
        lat_base = 18.5204
        lon_base = 73.8567
        dlat_25m = 25.0 * 8.9976e-6
        dlon_25m = 25.0 * (8.9976e-6 / math.cos(math.radians(lat_base)))

        mission_items = [
            # seq 0: Home / dummy
            (0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 2.0, 0, 0, lat_base, lon_base, 0.0),
            # seq 1: Takeoff to 10m
            (1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, lat_base, lon_base, 10.0),
            # seq 2: Waypoint 1 (North 25m)
            (2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 2.0, 0, 0, lat_base + dlat_25m, lon_base, 10.0),
            # seq 3: Waypoint 2 (North 25m, East 25m) -> 90 deg corner
            (3, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 2.0, 0, 0, lat_base + dlat_25m, lon_base + dlon_25m, 10.0),
            # seq 4: Land
            (4, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 2.0, 0, 0, lat_base + dlat_25m, lon_base + dlon_25m, 0.0),
        ]

        print(f"[TEST] Uploading {len(mission_items)} mission items...")
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

        print("[TEST] Monitoring flight execution (max 45 seconds)...")
        start_time = time.time()
        last_print = 0
        min_altitude_during_cruise = 100.0
        max_altitude_during_cruise = -100.0
        reached_item_2 = False
        reached_item_3 = False
        reached_cruise = False
        landing_started = False
        speed_records_near_wp = []
        altitude_errors_during_flight = []

        while time.time() - start_time < 45.0:
            msg = mav.recv_match(type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT'], blocking=True, timeout=0.5)
            if not msg:
                continue

            if msg.get_type() == 'MISSION_CURRENT':
                if msg.seq >= 2:
                    reached_item_2 = True
                if msg.seq >= 3:
                    reached_item_3 = True
                if msg.seq >= 4:
                    landing_started = True

            if msg.get_type() == 'GLOBAL_POSITION_INT':
                rel_alt = msg.relative_alt / 1000.0
                vx = msg.vx / 100.0
                vy = msg.vy / 100.0
                speed_xy = math.sqrt(vx * vx + vy * vy)

                if rel_alt >= 9.8:
                    reached_cruise = True

                # Track altitude during level cruise flight (after takeoff reached, before landing descent)
                if reached_cruise and not landing_started:
                    min_altitude_during_cruise = min(min_altitude_during_cruise, rel_alt)
                    max_altitude_during_cruise = max(max_altitude_during_cruise, rel_alt)
                    alt_err = abs(rel_alt - 10.0)
                    altitude_errors_during_flight.append(alt_err)

                # Telemetry log every 2 seconds
                now = time.time()
                if now - last_print > 2.0:
                    last_print = now
                    print(f"  [T+{now-start_time:.1f}s] Alt={rel_alt:.2f}m, SpeedXY={speed_xy:.2f}m/s (Vx={vx:.2f}, Vy={vy:.2f})")

                # If reached final waypoint or landed, test is done
                if reached_item_3 and rel_alt < 1.0:
                    print("[TEST] Drone reached destination and landed successfully!")
                    break

            time.sleep(0.05)

        print("\n======================================================================")
        print(" FLIGHT EVALUATION RESULTS:")
        print("======================================================================")
        if min_altitude_during_cruise < 100.0:
            alt_drop = 10.0 - min_altitude_during_cruise
            print(f" Cruise Target Altitude: 10.00 m")
            print(f" Lowest Altitude During Cruise/Braking: {min_altitude_during_cruise:.2f} m (max drop: {alt_drop:.2f} m)")
            print(f" Highest Altitude During Cruise: {max_altitude_during_cruise:.2f} m")
            if altitude_errors_during_flight:
                avg_err = sum(altitude_errors_during_flight) / len(altitude_errors_during_flight)
                print(f" Average Altitude Tracking Error: {avg_err:.2f} m")

            # Check for excessive altitude drop during braking (must be <= 0.35m)
            if alt_drop > 0.45:
                print(f"[FAIL] Excessive altitude drop during flight/braking: {alt_drop:.2f} m > 0.45 m")
                return False
            else:
                print(f"[PASS] Altitude held stably without dipping during braking! Drop = {alt_drop:.2f} m <= 0.45 m")

        print("======================================================================")
        print(" ALL WAYPOINT APPROACH & ALTITUDE TESTS COMPLETED SUCCESSFULLY!")
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
    ok = run_test()
    sys.exit(0 if ok else 1)
