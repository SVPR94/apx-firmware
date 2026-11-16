# Flight-test tools (APX and apx)

Scripts that fly a mission in AUTO over MAVLink, record the trajectory and
measure how smoothly the vehicle goes from waypoint to waypoint. They work
with APX SITL, apx SITL and real vehicles (APX Copter mode numbers),
so the same mission can be compared on both firmwares.

| Script | What it does |
| --- | --- |
| `wp_nav_test.py` | Fly one mission, report per-waypoint metrics, save CSV/JSON/PNG. `--compare` overlays saved runs. |
| `nav_suite.py` | Fly a list of missions and print one summary table (regression check, APX vs apx). |
| `wp_autotune.py` | Tune `WPNAV_*` / `PSC_*` params by coordinate descent, flying the missions for every trial. |
| `flightlib.py` | Shared library: connection, params, mission upload, flying, analysis, plots. |

Requirements: `pip install pymavlink matplotlib` (matplotlib only for plots).

## Running APX and apx SITL side by side

Both firmwares listen on TCP 5760 by default. Start APX as instance 1, which
shifts every port by 10, the same convention as apx's `-I`:

```bash
# APX instance 0: TCP 5760-5763, GCS UDP 14550, physics UDP 9003
cd apx
python3 sitl/sim_vehicle.py --vehicle hex_x --sim gazebo -I 0

# APX instance 1: TCP 5770-5773, GCS UDP 14560, physics UDP 9013
cd apx
python3 sitl/sim_vehicle.py --vehicle hex_x --sim gazebo -I 1
```

`-I 0` (the default) keeps the old ports (TCP 5760-5763, UDP 14550). The
launcher refuses to start if its TCP ports are already in use.
`--build-dir DIR` (or `$APX_BUILD_DIR`) runs a build other than `build/sitl`.

Which port to give the tools:

* APX: `tcp:127.0.0.1:5770` (any of 5770-5773; every port is a full link).
* apx with MAVProxy: `tcp:127.0.0.1:5762` (or `udp:127.0.0.1:14550`).
* apx with `--no-mavproxy`: 5762 or 5763. 5760 (SERIAL0) took only one
  client in our runs.

## Examples

```bash
cd tools/flight_test

# one mission on each firmware at the same speed, then compare
./wp_nav_test.py --connect tcp:127.0.0.1:5770 --mission corners --speed 8 --label apx
./wp_nav_test.py --connect tcp:127.0.0.1:5762 --mission corners --speed 8 --label apx
./wp_nav_test.py --compare runs/apx_corners.csv runs/APX_corners.csv

# replay the mission recorded in a APX Flight Manager log (or a .plan / .waypoints file)
./wp_nav_test.py --connect tcp:127.0.0.1:5770 \
    --mission "$HOME/Documents/APX Flight Manager/Telemetry/2026-11-05 14-49-55.tlog" --label apx_orbit

# whole suite, then a side-by-side table
./nav_suite.py --connect tcp:127.0.0.1:5770 --label apx --speed 5
./nav_suite.py --connect tcp:127.0.0.1:5762 --label apx --speed 5
./nav_suite.py --table apx apx

# auto-tune on SITL (restores the original values unless --apply)
./wp_autotune.py --connect tcp:127.0.0.1:5770 --missions square,survey --speed 6
```

`./wp_nav_test.py --list` shows the built-in missions: `square`, `corners`,
`gentle`, `straight`, `survey`, `zigzag`, and `orbit_yaw`, an orbit with a
`CONDITION_YAW` after every waypoint, as the Flight Manager orbit generates.

Missions are flown relative to wherever the vehicle is, so files planned at
another location (an apx log from Canberra, say) fly fine at the APX
SITL home. RTL/LAND items are dropped from replayed missions, so every test
ends hovering at the last waypoint. Param names are the APX/classic ones
(`WPNAV_SPEED` in cm/s, ...). On apx 4.7+, which renamed them to
`WP_SPD` (m/s) and so on, the tools translate names and units automatically.

## What the report means

```
  wp  turn   pass   v@wp  vmin   dip  stop
   2    24°   0.07  1.92  0.11   98%  STOP
```

* **dip**: speed lost around the waypoint, `1 - vmin / cruise`. Where cruise
  is the WPNAV speed, or the speed actually flown if the mission's
  DO_CHANGE_SPEED is slower. APX-standard smooth corners: small dip for
  gentle turns, larger only for sharp ones.
* **stop**: speed fell below 0.5 m/s at a fly-through waypoint.
* **pass**: closest distance to the waypoint. Fly-through corners cut it by
  up to the acceptance radius.
* **jerk rms / max**: from velocity telemetry resampled to 10 Hz, so APX
  (10 Hz) and apx (20 Hz) give comparable numbers.
* **xte**: distance from the planned polyline (includes the deliberate cut
  at corners).
* **final overshoot**: how far past the last waypoint the vehicle went.
* **SCORE**: one number for ranking runs and tuning, lower is better.
  It combines time per metre, dips, stops, jerk, cross-track error and
  overshoot; the weights are in `flightlib.score()` and in `wp_autotune.py --weights`.

Every run is also saved as `<label>_<mission>.csv` (t, n, e, alt,
velocities, attitude). The first line is a JSON header with the mission
and params, so runs can be re-analysed later without flying again.

## Auto-tune notes

* APX re-reads the navigation and position-controller params every time
  AUTO is entered, so trials need no reboot. Each trial parks the vehicle at
  the start point in GUIDED, sets the params, then flies the missions.
* Trials that exceed `--max-xte`, `--max-overshoot` or `--max-tilt` are
  rejected whatever their score.
* Every trial is appended to `runs/autotune_*.jsonl`, and the best set is
  written to `runs/autotune_best.json`.
* Tune in SITL first. On a real vehicle, keep a pilot ready to take over,
  and start with narrow ranges.
