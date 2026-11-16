#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""APX MAVLink Console Monitor."""
import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("[WARN] pymavlink not installed. Run: pip3 install pymavlink")

def main():
    parser = argparse.ArgumentParser(description="APX MAVLink Console Monitor")
    parser.add_argument("--udp", default="127.0.0.1:14550", help="UDP endpoint ip:port")
    parser.add_argument("--tcp", default=None, help="TCP endpoint ip:port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate for serial")
    parser.add_argument("command", nargs="*", help="Optional param command e.g. param get <NAME>")
    args = parser.parse_args()

    conn_str = f"udp:{args.udp}" if not args.tcp else f"tcp:{args.tcp}"
    print(f"[APX Console] Connecting to {conn_str}...")

    try:
        mav = mavutil.mavlink_connection(conn_str)
        mav.wait_heartbeat(timeout=5)
        print(f"[APX Console] Heartbeat received from system {mav.target_system} comp {mav.target_component}")
    except Exception as e:
        print(f"[APX Console] Connection failed: {e}")
        return

    if args.command:
        cmd = args.command
        if cmd[0] == "param" and len(cmd) >= 3 and cmd[1] == "get":
            mav.param_fetch_one(cmd[2])
            print(f"Requested param: {cmd[2]}")
        elif cmd[0] == "param" and len(cmd) >= 4 and cmd[1] == "set":
            val = float(cmd[3])
            mav.param_set_send(cmd[2], val)
            print(f"Sent param set: {cmd[2]} = {val}")
        return

    print("[APX Console] Listening to telemetry stream (Ctrl+C to exit)...")
    while True:
        try:
            msg = mav.recv_match(blocking=True, timeout=1.0)
            if msg:
                mtype = msg.get_type()
                if mtype in ["ATTITUDE", "GLOBAL_POSITION_INT", "SYS_STATUS", "STATUSTEXT"]:
                    print(f"[{mtype}] {msg.to_dict()}")
        except KeyboardInterrupt:
            print("
[APX Console] Exiting.")
            break

if __name__ == "__main__":
    main()
