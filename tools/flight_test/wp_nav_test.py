#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Waypoint-to-waypoint navigation test for APX and apx (SITL or real).

Flies a mission in AUTO, records the trajectory and reports how smoothly the
vehicle passes each waypoint: speed dip, full stops, accel/jerk, cross-track
error, final overshoot, plus one overall score (lower = better).

Examples
  # APX SITL started with:  python3 sitl/sim_vehicle.py --vehicle hex_x -I 1
  ./wp_nav_test.py --connect tcp:127.0.0.1:5770 --mission corners --label apx

  # apx SITL:  ./Tools/autotest/sim_vehicle.py -v APX Copter --no-mavproxy
  ./wp_nav_test.py --connect tcp:127.0.0.1:5760 --mission corners --label apx

  # same params on both, then compare the saved runs in one plot/table
  ./wp_nav_test.py --connect tcp:127.0.0.1:5770 --param WPNAV_SPEED=800 --mission square
  ./wp_nav_test.py --compare runs/apx_corners.csv runs/APX_corners.csv

Missions: built-in names (see --list) or a JSON file [[north_m, east_m], ...].
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flightlib as fl  # noqa: E402


def parse_params(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        out[k.strip()] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5770", help="MAVLink connection string")
    ap.add_argument("--mission", default="corners",
                    help="built-in name (--list), JSON, GCS .plan, .waypoints, or a .tlog to replay "
                         "the mission recorded in it")
    ap.add_argument("--alt", type=float, default=10.0, help="mission altitude (m, relative)")
    ap.add_argument("--radius", type=float, default=2.0, help="waypoint acceptance radius (m)")
    ap.add_argument("--speed", type=float, default=None, help="WPNAV_SPEED in m/s (sets the param)")
    ap.add_argument("--param", action="append", help="NAME=VALUE to set before flying (repeatable)")
    ap.add_argument("--repeat", type=int, default=1, help="fly the mission N times")
    ap.add_argument("--label", default=None, help="name for the output files")
    ap.add_argument("--out-dir", default="runs", help="where CSV/JSON/PNG are written")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--firmware", choices=["apx", "apx"], default=None,
                    help="override firmware auto-detection")
    ap.add_argument("--land", action="store_true", help="LAND when finished")
    ap.add_argument("--compare", nargs="+", metavar="CSV", help="only analyse/plot saved runs")
    ap.add_argument("--list", action="store_true", help="list built-in missions")
    args = ap.parse_args()

    if args.list:
        for k, v in fl.MISSIONS.items():
            print(f"{k:10s} {v}")
        print("orbit_yaw  16-point orbit, CONDITION_YAW after every waypoint, 2 m/s")
        return 0

    os.makedirs(args.out_dir, exist_ok=True)

    if args.compare:
        runs, labels, mets = [], [], []
        for p in args.compare:
            run = fl.load_csv(p)
            lab = os.path.splitext(os.path.basename(p))[0]
            cruise = run["params"].get("WPNAV_SPEED")
            m = fl.analyze(run, cruise / 100.0 if cruise else None)
            fl.print_report(m, lab)
            runs.append(run)
            labels.append(lab)
            mets.append(m)
        print("\n%-28s %8s %6s %6s %6s %7s %6s %6s" % ("run", "time_s", "stops", "dip", "jerk",
                                                      "xte_max", "ovr_m", "score"))
        for lab, m in zip(labels, mets):
            if "error" not in m:
                print("%-28s %8.1f %6d %5.0f%% %6.2f %7.2f %6.2f %6.1f" % (
                    lab[:28], m["mission_time_s"], m["stops"], 100 * m["mean_corner_dip"], m["rms_jerk"],
                    m["max_xte"], m["final_overshoot_m"], m["score"]))
        fl.plot_runs(runs, labels, os.path.join(args.out_dir, "compare.png"))
        return 0

    params = parse_params(args.param)
    if args.speed:
        params["WPNAV_SPEED"] = args.speed * 100.0
    plan = fl.load_plan(args.mission)
    if plan["alt"] and args.alt == ap.get_default("alt"):
        args.alt = plan["alt"]
    veh = fl.Vehicle(args.connect, firmware=args.firmware)
    cruise = params.get("WPNAV_SPEED") or veh.get_param("WPNAV_SPEED")
    cruise_ms = cruise / 100.0 if cruise else None
    if cruise and "WPNAV_SPEED" not in params:
        params["WPNAV_SPEED"] = cruise  # recorded with the run
    label = args.label or veh.firmware
    mission_name = os.path.splitext(os.path.basename(args.mission))[0]
    origin = None
    for i in range(args.repeat):
        run = fl.fly_mission(veh, plan, alt=args.alt, radius=args.radius, params=params,
                             timeout=args.timeout, origin=origin)
        origin = None
        suffix = f"_{i + 1}" if args.repeat > 1 else ""
        base = os.path.join(args.out_dir, f"{label}_{mission_name}{suffix}")
        fl.save_csv(run, base + ".csv")
        m = fl.analyze(run, cruise_ms)
        with open(base + ".json", "w") as f:
            json.dump(m, f, indent=1)
        fl.print_report(m, os.path.basename(base))
        fl.plot_runs([run], [label], base + ".png")
        print(f"saved: {base}.csv / .json / .png")
    if args.land:
        veh.set_mode(fl.MODE_LAND)
    return 0


if __name__ == "__main__":
    sys.exit(main())
