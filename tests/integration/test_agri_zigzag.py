#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
import time
from pymavlink import mavutil

def test_agri_zigzag():
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print("Heartbeat received.")

    # Set Mode to AGRI_ZIGZAG (custom mode index, assumes 7 for this example)
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        7 # Example mapping for AGRI_ZIGZAG
    )
    print("Mode set to AGRI_ZIGZAG.")

    # Send MAV_CMD to set point A
    print("Setting Point A...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, # Or custom command
        0,
        1, 0, 0, 0, 0, 0, 0 # Param 1: set A
    )
    time.sleep(2)

    # Move a bit in SITL, then set Point B
    print("Setting Point B...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, # Or custom command
        0,
        2, 0, 0, 0, 0, 0, 0 # Param 1: set B
    )
    
    # Wait and verify it flies rows
    print("Waiting for row tracking...")
    start_time = time.time()
    while time.time() - start_time < 10:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        # In a real test, we'd verify the trajectory here
    
    print("Zigzag test complete.")

if __name__ == "__main__":
    test_agri_zigzag()
