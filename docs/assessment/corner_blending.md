# AUTO Corner Blending (Curvature-Continuous Turns)

**Request:** when flying autonomously, blend waypoint corners geometrically
so the position controller gets smooth target states, then verify that
turns are smooth.

**Implementation:** APX's own original implementation of curvature-continuous
geometric corner blending designed from kinematic principles.

## Problem with the previous corners

AUTO flew each fly-through waypoint as a straight leg, then a circular arc,
then a straight leg. That arc is the problem:

- **Curvature step:** a straight line has curvature 0 and the arc has 1/R,
  so at the arc entry and exit the commanded lateral acceleration (and so
  the lean angle) stepped within one 400 Hz tick.
- **Along-track acceleration:** it was also reset to 0 on entering the arc
  and at the start of the next leg.
- **Corner speed:** came partly from a rule of thumb, cruise × cos(θ/2).

## New corner shape: spiral-arc-spiral blend

Code: `modules/modes/path_tracker.{hpp,cpp}`; `WPNAV_BLEND=1` is the
default.

The corner is a transition curve, as used in road and rail design. Its
curvature rises linearly from 0 to κ over an entry spiral, holds κ on a
circular arc, then falls linearly back to 0 over an exit spiral.

At the corner speed v:

- lateral acceleration v²κ(s) changes continuously;
- on the spirals it grows at a constant rate v³κ/L_s;
- there is no step anywhere, either in curvature or in along-track
  acceleration.

**Sizing.** The planner picks the highest corner speed v (up to the leg's
cruise speed, and no faster than could still stop on the next leg) that
meets all of these:

| Constraint | Limit |
|---|---|
| Lateral acceleration v²κ | ≤ `WPNAV_ACCEL` |
| Lateral jerk v³κ/L_s | ≤ `WPNAV_JERK` |
| Closest approach to the waypoint (curve apex) | ≤ the waypoint radius: mission item acceptance radius, else `WPNAV_RADIUS` |
| Length used on each leg | ≤ 45 % of that leg, so two corners on one leg still leave a straight part |

At a given v, the first two constraints fix κ and the spiral length L_s.
If the spirals alone already turn more than the corner angle, the curve
becomes two spirals meeting at a lower peak curvature. The speed is found
by bisection, once per waypoint, with no heap allocation.

**Evaluation.** Heading along the curve has a closed form. Position doesn't
(it needs Fresnel integrals), so it's integrated with Simpson's rule into
a 64-point table and read back by cubic Hermite interpolation with the
exact tangents.

**Reference states.** Each tick the reference provides:

- position on the curve;
- velocity v·T;
- acceleration a·T + v²κ·N (T = tangent, N = inward normal).

Along-track speed and acceleration carry through the corner and into the
next leg under the existing jerk-limited (S-curve) speed shaping.

**Rules for when a corner is blended:**

- turns under 8° are flown straight through;
- turns over 150° stop at the waypoint;
- if no blend fits (very short legs), the vehicle stops at the waypoint.

`WPNAV_BLEND=0` restores the circular arc for comparison.

**Example sizes** (accel 2.5 m/s², jerk 5 m/s³, radius 2 m, long legs):

| Turn | Corner speed (arc → blend) | Notes |
|---|---|---|
| 90° | 2.24 → 3.44 m/s | Minimum turn radius 4.7 m; uses 5.6 m of each leg |
| 45° | 3.5 → 7.7 m/s | Limited by the 2 m apex |

## Verification

### Unit tests

`tests/unit/test_path_tracker.cpp`: 6 new tests. The 2 arc-specific tests
now set `blend_corners = false`. Full suite: 73/73 pass.

The new tests check the reference generator directly:

- lateral accel ≤ `WPNAV_ACCEL` and lateral jerk ≤ `WPNAV_JERK` through
  corners and short survey rows;
- no reference jump at waypoint handovers;
- the curve passes within the waypoint radius and is flown through, not
  stopped;
- corner speed is 1.4× or more of the arc's at 90°;
- wider turns keep more speed;
- the arc's lateral jerk exceeds 20× the limit, documenting the step the
  blend removes.

Total jerk is checked on moving ticks only. At the final stop, the existing
stop logic zeroes a small residual deceleration (about 0.3 m/s²) in one
tick. That behaviour predates this change and is left as is.

### SITL A/B flights

Same mission, `WPNAV_BLEND=0` vs `1`, measured from 400 Hz simulator truth
(sensor log) within 12 m of each corner.

The corners mission is a 90° corner then two 45° corners at 5 m/s, 2 m
waypoint radius. Each cell gives arc → blend:

| Airframe | Peak jerk (m/s³) | Peak roll/pitch rate (°/s) | Outside drift (m) | 90° min speed (m/s) | 45° min speed (m/s) | Mission time (s) |
|---|---|---|---|---|---|---|
| Built-in quad | 11.5–12.1 → 5.8–5.9 | 75 → 37–38 | 0.29 → 0.12 | 1.8 → 3.0 | 3.1 → 4.5 | 79.5 → 77.3 |
| Built-in hex | 11.2–11.4 → 6.1–6.3 | 70–76 → 40–41 | 0.29 → 0.12 | 1.8 → 3.0 | 3.0 → 4.5 | 79.8 → 77.5 |
| Built-in octo | 11.2–11.4 → 6.4–6.5 | 68–77 → 41–44 | 0.29 → 0.11 | 1.8 → 3.0 | 3.0 → 4.5 | 79.6 → 77.2 |
| Gazebo `apx_hex` (14.5 kg) | 12.2–12.6 → 5.7–6.2 | 70–72 → 33–34 | 0.32 → 0.14 | 1.7 → 2.9 | 3.0 → 4.4 | 75.8 → 73.4 |
| Built-in hex at 8 m/s | 11.3–11.6 → 6.2–6.8 | 70–76 → 40–41 | 0.28 → 0.08 | 1.5 → 2.7 | 2.7 → 7.2 | 68.0 → 63.1 |

Closest approach to the waypoint with the blend:

- 1.76–1.78 m at 90° and 0.63–0.70 m at 45°, at 5 m/s;
- 1.82–1.88 m at 8 m/s.

These are within the 2 m design limit.

**Survey (5 m rows, 3 m/s).** The blend is smoother: peak jerk
10.5–11.9 → 6.6–7.4 m/s³, roll/pitch rate 69–79 → 40°/s. It is slightly
slower: corner minimum speed 1.4 → 1.1 m/s, mission time 89.4 → 90.9 s. A
5 m row leaves at most 2.25 m per corner for the curvature ramps.

**Plot.** [img/corner_blend_comparison.png](img/corner_blend_comparison.png)
shows the hex's 90° corner (path, speed, acceleration, jerk, tilt rate) and
peak jerk for every airframe.

### SITL test changes

**Corner checks in `sitl/test_auto_path_tracking.py`.** The old check
"corner deviation ≤ 1.5 m" measured distance from either leg near the
corner. It was meant to catch overshoot but can't tell overshoot from an
intended cut: at 45° a 2 m apex is 1.85 m from the legs. It is replaced by:

- **Outside drift:** distance on the outside of the turn, ≤ 0.5 m;
- **Cut:** closest approach to the waypoint, ≤ waypoint radius + 0.5 m.

The old metric is still printed. Samples are attributed to a corner only
while flying its own two legs. Before that fix, the 5 m-spaced survey rows
and the first corner (passed before the mission moves on) were scored
wrongly.

**New `--sensor-log` option.** It adds the per-corner truth analysis above.

**Separate GCS port.** The braking, mission-speed, GCS and sprayer tests
now use `--gcs-port 14650`, like the other SITL tests, so a running
APX Flight Manager can't attach to them.

### Regression

All pass with the blend as default:

- corners, survey, long legs, live `DO_CHANGE_SPEED`;
- waypoint braking, per-item mission speed, GCS mission and joystick;
- loiter landing, sprayer (Gazebo).

STM32H7 MinSizeRel builds compile. The images are unchanged, because the
flight modes are not linked into the H7 image yet.

## Known limits

- The corner is flown at constant speed. Braking ends where the curve
  starts, rather than overlapping the entry spiral, so on short survey rows
  the blend is slightly slower than the arc.
- The stop at a final waypoint still ends with a small one-tick
  acceleration step (stop logic, pre-existing).
- `WPNAV_ACCEL` bounds the reference's lateral acceleration. The vehicle's
  measured peak is about 3.0 m/s² because the controller lags the
  reference.
