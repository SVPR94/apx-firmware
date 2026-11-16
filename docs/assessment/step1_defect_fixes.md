# Step 1 — Verified Defect Fixes and Build Environment

Follows `phase_a_firmware_assessment.md` §11 step 1. Each fix targets a defect
confirmed in the code; each has a test or a SITL check. All results below were
produced on 2026-10-14 against base commit `0d3970e` plus these changes.

## Build environment

| Change | Files |
|---|---|
| One-time Ubuntu setup: apt deps, Python deps, pinned Arm GNU Toolchain 15.2.rel1, permanent PATH block in `~/.bashrc`/`~/.profile` (idempotent; `--no-apt`, `--with-gazebo`, `--toolchain-only`) | `tools/setup_ubuntu_env.sh`, `requirements.txt` |
| Missing ARM compiler now stops configure with a message pointing to the setup script; any local toolchain version or `$APX_ARM_TOOLCHAIN_BIN` is found | `platforms/stm32h7/toolchain-arm-none-eabi.cmake` |
| `APX_BUILD_SITL` defaults OFF for `APX_BOARD=stm32h7` (the Linux-socket SITL bridge was compiled for ARM and failed a plain configure) | `CMakeLists.txt` |
| Docs | `docs/build.md`, `docs/troubleshooting.md`, `platforms/stm32h7/README.md` |

STM32H7 builds (MinSizeRel, GCC 15.2.1): `apx_h743_v1` 5 092 B flash / 17 KB RAM;
`apx_h743_pro` and `custom_h7_fc` 4 944 B flash / 24 KB DTCM. This is the
synthetic test loop, not a flight stack (see the Phase A report §2).

## Defect fixes

| # | Defect | Fix | Evidence |
|---|---|---|---|
| 1 | Built-in SITL "hex X" had a hexa-plus layout (motor 1 on the nose) that the HEX_X mixer doesn't drive; every built-in hex takeoff flipped (also on unmodified `0d3970e`) | Physics hex/octo written as the real X airframes; `octo_x` gets an octo model instead of the quad | Unit test cross-checks every sim airframe against the mixer tables; GUIDED takeoff: hex 3/3, octo, quad all hold 5 m at 0° tilt (was 175–179° flip) |
| 2 | Mixer read a null table for QUAD_PLUS / HEX_FLAT / OCTO_COAX / OCTO_PLUS | Frames without a table report 0 motors; `mix()` leaves outputs untouched | `MotorMixerTest.FramesWithoutTableLeaveOutputsUntouched` |
| 3 | PID derivative kick (D = kd·e/dt) on the first tick after every reset/mode entry | Derivative is 0 until a previous sample exists | `PidControllerTest.NoDerivativeKickOnFirstTickAfterReset`; `DerivativeFilter` test rewritten (it relied on the kick) |
| 4 | "Butterworth" LPF had Q = √2; notch coefficients didn't form a notch; harmonic notch reset its state on every retune | RBJ Butterworth α = sin ω / √2; pole-zero notch with unity DC gain and Nyquist guard; bumpless retune | `ButterworthLPFTest` (−3 dB at cutoff, no passband peak, unity DC), `NotchFilterTest` (>30 dB at centre), `HarmonicNotchTest` |
| 5 | Bare-metal `now_us()` jumped backwards every 32-bit DWT wrap (~67 s) | 64-bit wrap extension; µs conversion that can't overflow | Builds for all STM32H7 boards; needs hardware for a runtime test |
| 6 | All MAVLink links shared one parser channel | UDP on channel 0; each TCP client its own channel, released and reset on disconnect | Two TCP links each sent 100 concurrent PARAM_REQUEST_READ: 100/100 answered on both |
| 7 | HEARTBEAT always `MAV_TYPE_QUADROTOR` | Type from the frame (QUAD/HEXA/OCTOROTOR) | SITL hex reports type 13 (HEXAROTOR) |
| 8 | Param change callbacks ran with the ParamManager mutex held (deadlock if a callback reads a param) | Value set under the lock; callbacks run after release on a copied list | `ParamManagerTest.CallbackMayReadParameters` (bounded by a 2 s timeout) |
| 9 | `ModeRTL` ignored `RTL_ALT` (always 15 m) | Read in the constructor (cm → m) | Code change; covered by AUTO/RTL SITL survey test |
| 10 | MAVLink disarm accepted in flight | Refused unless landed (or STABILIZE); force code param2 = 21196 still disarms. Shared `ModeManager::disarm_allowed()` (atomic ground timer, read from the MAVLink thread) | SITL: in-flight disarm → DENIED; forced → ACCEPTED |

## Regression results (after all changes)

| Test | Result |
|---|---|
| C++ unit tests | 47/47 pass (35 existing + 12 new) |
| AUTO corners, built-in hex | path error max 0.28 m, tilt 21.2°, all checks pass |
| AUTO survey + DO_CHANGE_SPEED + RTL, built-in hex | path error max 0.40 m, all checks pass |
| AUTO corners, built-in octo (first octo flight) | path error max 0.28 m, landed on target |
| AUTO corners, Gazebo hex | path error max 0.26 m, tilt 18.3°, landed |
| LOITER landing / stick disarm / DISARM_DELAY | 7/7 pass |
| Waypoint approach & braking; GCS mission + joystick | pass |
| Sprayer (Gazebo) | 3/3 pass |

All of these still fly on simulator truth (no sensor noise or estimator);
they show the fixes introduce no regression, not real-vehicle performance.

## Not addressed in this step

Integrators reset on every mode switch; land/crash detector thresholds (fixed
hover estimate, home-relative altitude); unconnected failsafes and arming
checks; parameter persistence; logging. These are in later roadmap steps.
