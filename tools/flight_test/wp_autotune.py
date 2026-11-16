#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Auto-tune waypoint navigation parameters by flying test missions.

Coordinate descent: for each parameter in turn, try it a step up and a step
down, fly the evaluation mission(s), keep the change if the score improves;
after a full round with no improvement the step is halved. Every trial is
logged so the search can be inspected or resumed by hand.

The score (flightlib.score, lower = better) rewards mission time and
penalises corner speed dips, full stops, jerk, cross-track error and final
overshoot. Trials that break the safety limits (--max-xte, --max-overshoot,
--max-tilt) are rejected whatever their score.

APX re-reads these params every time AUTO is entered, so no reboot is needed
between trials. Run it against SITL - never a real vehicle in the air
without a pilot ready to take over.

Examples
  # tune the trajectory + position-loop params on APX SITL (instance 1)
  ./wp_autotune.py --connect tcp:127.0.0.1:5770 --missions square,survey --speed 6

  # only the trajectory limits, 3 rounds, and leave the best values set
  ./wp_autotune.py --connect tcp:127.0.0.1:5770 --space WPNAV_ACCEL=100:500 \\
        --space WPNAV_JERK=1:15 --rounds 3 --apply

  # weight smoothness more than speed
  ./wp_autotune.py --connect tcp:127.0.0.1:5770 --weights time=0.5,jerk=6,dip=40
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flightlib as fl  # noqa: E402

# name: (min, max, relative step) - APX names/units (cm, cm/s for WPNAV_*)
DEFAULT_SPACE = {
    "WPNAV_ACCEL": (100.0, 500.0, 0.25),
    "WPNAV_JERK": (1.0, 15.0, 0.3),
    "PSC_POSXY_P": (0.5, 2.0, 0.2),
    "PSC_VELXY_P": (0.8, 5.0, 0.25),
    "PSC_VELXY_I": (0.2, 3.0, 0.3),
    "PSC_VELXY_D": (0.0, 1.5, 0.3),
}


def parse_space(items):
    if not items:
        return dict(DEFAULT_SPACE)
    out = {}
    for it in items:
        name, rng = it.split("=", 1)
        parts = [float(x) for x in rng.split(":")]
        lo, hi = parts[0], parts[1]
        step = parts[2] if len(parts) > 2 else 0.25
        out[name.strip()] = (lo, hi, step)
    return out


def parse_weights(s):
    w = dict(time=1.0, dip=40.0, stop=25.0, jerk=2.0, xte=4.0, overshoot=10.0)
    if s:
        for it in s.split(","):
            k, v = it.split("=")
            w[k.strip()] = float(v)
    return w


class Evaluator:
    def __init__(self, veh, plans, args, weights, log_path):
        self.veh = veh
        self.plans = plans
        self.args = args
        self.w = weights
        self.log_path = log_path
        self.n = 0

    def __call__(self, params):
        self.n += 1
        total = 0.0
        details = []
        ok = True
        for name, plan in self.plans:
            fixed = dict(params)
            if self.args.speed:
                fixed["WPNAV_SPEED"] = self.args.speed * 100.0
            run = fl.fly_mission(self.veh, plan, alt=plan["alt"] or self.args.alt, radius=self.args.radius,
                                 params=fixed, timeout=self.args.timeout)
            m = fl.analyze(run, self.args.speed)
            if "error" in m:
                ok = False
                details.append(dict(mission=name, error=m["error"]))
                continue
            s = fl.score(m, self.w)
            reasons = []
            if m["max_xte"] > self.args.max_xte:
                reasons.append(f"xte {m['max_xte']}")
            if m["final_overshoot_m"] > self.args.max_overshoot:
                reasons.append(f"overshoot {m['final_overshoot_m']}")
            if m["max_tilt"] > self.args.max_tilt:
                reasons.append(f"tilt {m['max_tilt']}")
            if reasons:
                ok = False
            total += s
            details.append(dict(mission=name, score=s, time=m["mission_time_s"], stops=m["stops"],
                                dip=m["mean_corner_dip"], jerk=m["rms_jerk"], xte=m["max_xte"],
                                overshoot=m["final_overshoot_m"], tilt=m["max_tilt"], rejected=reasons))
        result = dict(trial=self.n, params=params, score=round(total, 2) if ok else None, ok=ok,
                      details=details, t=time.strftime("%H:%M:%S"))
        with open(self.log_path, "a") as f:
            f.write(json.dumps(result) + "\n")
        pstr = " ".join(f"{k}={v:g}" for k, v in params.items())
        print(f"[tune] #{self.n:03d} {'%.1f' % total if ok else 'REJECTED':>9}  {pstr}"
              + ("" if ok else "  " + "; ".join(r for d in details for r in d.get("rejected", []))), flush=True)
        return total if ok else float("inf")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5770")
    ap.add_argument("--missions", default="square,survey", help="comma-separated mission specs (see wp_nav_test)")
    ap.add_argument("--speed", type=float, default=6.0, help="WPNAV_SPEED (m/s) held fixed while tuning")
    ap.add_argument("--alt", type=float, default=10.0)
    ap.add_argument("--radius", type=float, default=2.0)
    ap.add_argument("--space", action="append", help="NAME=min:max[:rel_step] (repeatable; default: nav set)")
    ap.add_argument("--rounds", type=int, default=4, help="max coordinate-descent rounds")
    ap.add_argument("--min-step", type=float, default=0.05, help="stop when the relative step is below this")
    ap.add_argument("--weights", default=None, help="score weights, e.g. time=1,dip=40,stop=25,jerk=2,xte=4")
    ap.add_argument("--max-xte", type=float, default=3.0, help="reject trials with more cross-track error (m)")
    ap.add_argument("--max-overshoot", type=float, default=1.5, help="reject trials overshooting the end (m)")
    ap.add_argument("--max-tilt", type=float, default=35.0, help="reject trials leaning more (deg)")
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--apply", action="store_true", help="leave the best values set on the vehicle")
    ap.add_argument("--firmware", choices=["apx", "apx"], default=None)
    ap.add_argument("--out-dir", default="runs")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    log_path = os.path.join(args.out_dir, time.strftime("autotune_%Y%m%d_%H%M%S.jsonl"))
    space = parse_space(args.space)
    weights = parse_weights(args.weights)
    plans = []
    for spec in args.missions.split(","):
        spec = os.path.expanduser(spec.strip())
        plans.append((os.path.splitext(os.path.basename(spec))[0], fl.load_plan(spec)))

    veh = fl.Vehicle(args.connect, firmware=args.firmware)
    start = {}
    for name, (lo, hi, _) in space.items():
        v = veh.get_param(name)
        if v is None:
            print(f"[tune] {name} not found on the vehicle - skipped")
            continue
        start[name] = min(max(v, lo), hi)
    if not start:
        print("[tune] nothing to tune")
        return 1
    space = {k: v for k, v in space.items() if k in start}
    print(f"[tune] log: {log_path}")
    print(f"[tune] start: {start}")

    evaluate = Evaluator(veh, plans, args, weights, log_path)
    best = dict(start)
    best_score = start_score = evaluate(best)
    steps = {k: space[k][2] for k in space}

    for rnd in range(1, args.rounds + 1):
        improved = False
        for name in space:
            lo, hi, _ = space[name]
            for direction in (+1, -1):
                cur = best[name]
                cand_v = cur * (1.0 + direction * steps[name]) if cur != 0 else direction * steps[name]
                cand_v = round(min(max(cand_v, lo), hi), 4)
                if abs(cand_v - cur) < 1e-6:
                    continue
                cand = dict(best)
                cand[name] = cand_v
                s = evaluate(cand)
                if s < best_score:
                    best, best_score, improved = cand, s, True
                    break  # keep going in this direction next round
        print(f"[tune] round {rnd}: best {best_score:.1f}  {best}")
        if not improved:
            steps = {k: v * 0.5 for k, v in steps.items()}
            if max(steps.values()) < args.min_step:
                break

    print("\n[tune] ==== result ====")
    print(f"[tune] start score {start_score:.1f}")
    for k in space:
        print(f"  {k:14s} {start[k]:>9g} -> {best[k]:>9g}")
    print(f"[tune] best score {best_score:.1f} after {evaluate.n} trials (log {log_path})")
    if args.apply:
        veh.set_params(best)
        print("[tune] best values applied")
    else:
        veh.set_params(start)
        print("[tune] original values restored (use --apply to keep the best)")
    with open(os.path.join(args.out_dir, "autotune_best.json"), "w") as f:
        json.dump(dict(best=best, start=start, score=best_score, missions=args.missions,
                       speed=args.speed), f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
