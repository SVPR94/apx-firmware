#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Fly a set of navigation test missions and print one summary table.

Use it as a regression check after a firmware change, or to compare APX
with apx on identical missions.

Examples
  # all built-in missions at 5 m/s on APX (instance 1), save as 'apx_new'
  ./nav_suite.py --connect tcp:127.0.0.1:5770 --label apx_new --speed 5

  # same on apx, then the table for both
  ./nav_suite.py --connect tcp:127.0.0.1:5762 --label apx --speed 5
  ./nav_suite.py --table apx_new apx

  # replay a real mission from a GCS log as part of the suite
  ./nav_suite.py --connect tcp:127.0.0.1:5770 --label apx --missions corners,~/flight.tlog

Runs are written to --out-dir as <label>_<mission>.csv/.json/.png, the same
files wp_nav_test.py writes, so either tool can re-analyse them.
"""

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flightlib as fl  # noqa: E402

DEFAULT_MISSIONS = "corners,square,survey,zigzag,gentle,straight,orbit_yaw"
COLS = [("time_s", "mission_time_s", "%7.1f"), ("stops", "stops", "%5d"), ("dip", "mean_corner_dip", "%5.0f%%"),
        ("jerk", "rms_jerk", "%5.2f"), ("xte", "max_xte", "%5.2f"), ("ovr", "final_overshoot_m", "%5.2f"),
        ("score", "score", "%7.1f")]


def fmt(m, key, f):
    v = m.get(key)
    if v is None:
        return "-"
    return f % (100 * v) if "%%" in f else f % v


def table(labels, out_dir):
    rows = {}
    for lab in labels:
        for p in sorted(glob.glob(os.path.join(out_dir, f"{lab}_*.json"))):
            mission = os.path.basename(p)[len(lab) + 1:-5]
            with open(p) as f:
                rows.setdefault(mission, {})[lab] = json.load(f)
    head = "%-12s %-14s" % ("mission", "run") + " ".join("%7s" % c[0] for c in COLS)
    print(head)
    print("-" * len(head))
    totals = {lab: 0.0 for lab in labels}
    for mission in sorted(rows):
        for lab in labels:
            m = rows[mission].get(lab)
            if not m or "error" in m:
                continue
            totals[lab] += m["score"]
            print("%-12s %-14s" % (mission[:12], lab[:14]) + " ".join("%7s" % fmt(m, k, f) for _, k, f in COLS))
        print()
    print("total score (lower is better): " + ", ".join(f"{lab} {v:.1f}" for lab, v in totals.items()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5770")
    ap.add_argument("--label", default=None, help="run label (default: firmware name)")
    ap.add_argument("--missions", default=DEFAULT_MISSIONS, help="comma-separated mission specs")
    ap.add_argument("--speed", type=float, default=None, help="WPNAV_SPEED m/s for every mission")
    ap.add_argument("--alt", type=float, default=10.0)
    ap.add_argument("--radius", type=float, default=2.0)
    ap.add_argument("--param", action="append", help="NAME=VALUE (repeatable)")
    ap.add_argument("--firmware", choices=["apx", "apx"], default=None)
    ap.add_argument("--out-dir", default="runs")
    ap.add_argument("--table", nargs="+", metavar="LABEL", help="only print the table for saved labels")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    if args.table:
        table(args.table, args.out_dir)
        return 0

    params = {}
    for it in args.param or []:
        k, v = it.split("=", 1)
        params[k.strip()] = float(v)
    if args.speed:
        params["WPNAV_SPEED"] = args.speed * 100.0
    veh = fl.Vehicle(args.connect, firmware=args.firmware)
    label = args.label or veh.firmware
    cruise = params.get("WPNAV_SPEED") or veh.get_param("WPNAV_SPEED")
    for spec in args.missions.split(","):
        spec = os.path.expanduser(spec.strip())
        plan = fl.load_plan(spec)
        name = os.path.splitext(os.path.basename(spec))[0]
        alt = plan["alt"] or args.alt
        try:
            run = fl.fly_mission(veh, plan, alt=alt, radius=args.radius, params=params)
        except Exception as e:  # keep going with the other missions
            print(f"[suite] {name}: FAILED - {e}")
            continue
        base = os.path.join(args.out_dir, f"{label}_{name}")
        fl.save_csv(run, base + ".csv")
        m = fl.analyze(run, cruise / 100.0 if cruise else None)
        with open(base + ".json", "w") as f:
            json.dump(m, f, indent=1)
        fl.plot_runs([run], [label], base + ".png")
        print(f"[suite] {name}: time {m.get('mission_time_s')} s, stops {m.get('stops')}, score {m.get('score')}")
    table([label], args.out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
