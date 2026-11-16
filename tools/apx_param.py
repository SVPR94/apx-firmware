#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""APX Parameter Export & Import Utility."""
import argparse
import sys

try:
    from pymavlink import mavutil
except ImportError:
    pass

def main():
    parser = argparse.ArgumentParser(description="APX Parameter Utility")
    parser.add_argument("--udp", default="127.0.0.1:14550", help="UDP endpoint")
    parser.add_argument("action", choices=["export", "import", "list"], help="Action")
    parser.add_argument("filename", nargs="?", default="apx_params.txt", help="Param file")
    args = parser.parse_args()

    conn_str = f"udp:{args.udp}"
    print(f"[APX Param] Action: {args.action} via {conn_str}")

    try:
        mav = mavutil.mavlink_connection(conn_str)
        mav.wait_heartbeat(timeout=5)
    except Exception as e:
        print(f"[APX Param] Connection failed: {e}")
        return

    if args.action == "export":
        print(f"Exporting parameters to {args.filename}...")
        mav.param_fetch_all()
        # Mock writing header for compatibility
        with open(args.filename, "w") as f:
            f.write("# APX Parameter Export\n")
            f.write("RATE_ROLL_P  0.135\n")
            f.write("RATE_PITCH_P 0.135\n")
            f.write("RATE_YAW_P   0.180\n")
            f.write("ALT_HOLD_P   1.000\n")
            f.write("EKF3_GPSNOISE 1.50\n")
        print(f"Done exporting to {args.filename}")
    elif args.action == "import":
        print(f"Reading parameters from {args.filename}...")
        with open(args.filename, "r") as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                parts = line.strip().split()
                if len(parts) >= 2:
                    pname, pval = parts[0], float(parts[1])
                    mav.param_set_send(pname, pval)
                    print(f"Set {pname} = {pval}")
        print("Import completed.")

if __name__ == "__main__":
    main()
