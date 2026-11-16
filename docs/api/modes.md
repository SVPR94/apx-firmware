# Flight Modes API Reference

Namespace: `apx::modes`

## Summary

This page documents all public classes, data structures, and enumerations for `Flight Modes`.

| Symbol | Kind | Declared In | Description |
|---|---|---|---|
| [`ControlOutput`](#controloutput) | `struct` | `modules/modes/vehicle_state.hpp` | What a FlightMode produces each tick. thrust/roll_cmd/pitch_cmd/yaw_cmd feed str |
| [`CrashDetector`](#crashdetector) | `class` | `modules/modes/crash_detector.hpp` | Mode-independent safety net, checked every tick by ModeManager regardless of whi |
| [`FlightMode`](#flightmode) | `class` | `modules/modes/mode_base.hpp` | Class definition. |
| [`FlightModeId`](#flightmodeid) | `enum` | `modules/modes/mode_base.hpp` | Custom mode enum for APX flight modes (see mode_manager.cpp's to_custom_mode_num |
| [`GuidedTarget`](#guidedtarget) | `struct` | `modules/modes/vehicle_state.hpp` | A GCS-commanded "go here" target (SET_POSITION_TARGET_LOCAL_NED/ GLOBAL_INT, or  |
| [`LandDetector`](#landdetector) | `class` | `modules/modes/land_detector.hpp` | Shared by ModeLand and ModeRTL's final descent leg. Three independent, physicall |
| [`LandPhase`](#landphase) | `enum` | `modules/modes/mode_land.hpp` | Enum definition. |
| [`LandStage`](#landstage) | `enum` | `modules/modes/mode_auto.hpp` | LAND / RTL items are flown in stages: (RTL only) climb to RTL_ALT in place, fly  |
| [`ModeAgriSpray`](#modeagrispray) | `class` | `modules/modes/mode_agri_spray.hpp` | AGRI_SPRAY: pump/relay control and spray-area path logic aren't implemented yet  |
| [`ModeAgriZigZag`](#modeagrizigzag) | `class` | `modules/modes/mode_agri_zigzag.hpp` | AGRI_ZIGZAG: the point-A/point-B row-following pattern isn't implemented yet (ne |
| [`ModeAltHold`](#modealthold) | `class` | `modules/modes/mode_althold.hpp` | ALT_HOLD: unlike POS_HOLD/LOITER, roll/pitch/yaw are controlled by the pilot's s |
| [`ModeAuto`](#modeauto) | `class` | `modules/modes/mode_auto.hpp` | AUTO: real waypoint sequencing over whatever mission MavlinkHandler most recentl |
| [`ModeBrake`](#modebrake) | `class` | `modules/modes/mode_brake.hpp` | Class definition. |
| [`ModeGuided`](#modeguided) | `class` | `modules/modes/mode_guided.hpp` | GUIDED: the mode MAV_CMD_NAV_TAKEOFF, SET_POSITION_TARGET_LOCAL_NED/ GLOBAL_INT  |
| [`ModeLand`](#modeland) | `class` | `modules/modes/mode_land.hpp` | LAND: safely brakes horizontal speed along line of travel first, then descends v |
| [`ModeLoiter`](#modeloiter) | `class` | `modules/modes/mode_loiter.hpp` | Class definition. |
| [`ModeManager`](#modemanager) | `class` | `modules/modes/mode_manager.hpp` | Single place mode switching/transition logic lives: adding a new mode means (1)  |
| [`ModePosHold`](#modeposhold) | `class` | `modules/modes/mode_poshold.hpp` | POS_HOLD: holds the 3D position/heading captured at entry using the same Navigat |
| [`ModeRTL`](#modertl) | `class` | `modules/modes/mode_rtl.hpp` | RTL: climb to m_rtl_altitude (if not already above it) at the current position,  |
| [`ModeStabilize`](#modestabilize) | `class` | `modules/modes/mode_stabilize.hpp` | STABILIZE: pilot controls attitude (roll/pitch tilt), yaw rate, and collective t |
| [`NavigationCascade`](#navigationcascade) | `class` | `modules/modes/navigation_cascade.hpp` | Shared Position -> Attitude -> Rate -> mixer-command cascade used by every mode  |
| [`PathSegment`](#pathsegment) | `struct` | `modules/modes/path_tracker.hpp` | Struct definition. |
| [`PathTracker`](#pathtracker) | `class` | `modules/modes/path_tracker.hpp` | Class definition. |
| [`PathTrackerLimits`](#pathtrackerlimits) | `struct` | `modules/modes/path_tracker.hpp` | Struct definition. |
| [`PathTrackerOutput`](#pathtrackeroutput) | `struct` | `modules/modes/path_tracker.hpp` | Struct definition. |
| [`Phase`](#phase) | `enum` | `modules/modes/path_tracker.hpp` | Enum definition. |
| [`Result`](#result) | `enum` | `modules/modes/stick_arm_detector.hpp` | Enum definition. |
| [`RtlState`](#rtlstate) | `enum` | `modules/modes/mode_rtl.hpp` | Enum definition. |
| [`SprayState`](#spraystate) | `enum` | `modules/modes/mode_agri_spray.hpp` | Enum definition. |
| [`Sprayer`](#sprayer) | `class` | `modules/modes/sprayer.hpp` | Class definition. |
| [`StickArmDetector`](#stickarmdetector) | `class` | `modules/modes/stick_arm_detector.hpp` | RC-stick arm/disarm gesture recognition: throttle held at minimum + yaw stick he |
| [`VehicleState`](#vehiclestate) | `struct` | `modules/modes/vehicle_state.hpp` | Snapshot of everything a FlightMode needs to make a decision each tick. Position |
| [`Waypoint`](#waypoint) | `struct` | `modules/modes/mode_auto.hpp` | One mission item, already resolved to NED-relative-to-home by the caller (Mavlin |
| [`ZigZagState`](#zigzagstate) | `enum` | `modules/modes/mode_agri_zigzag.hpp` | Enum definition. |

---

## ControlOutput
**Kind:** `struct`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/vehicle_state.hpp`  

> What a FlightMode produces each tick. thrust/roll_cmd/pitch_cmd/yaw_cmd feed straight into MotorMixer::mix() (same 0..1 / -1..1 convention it already used for the thrust-only case). request_disarm is how the land detector in ModeLand tells the caller to cut power once touchdown is confirmed - see mode_manager.cpp.

---

## CrashDetector
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/crash_detector.hpp`  

> Mode-independent safety net, checked every tick by ModeManager regardless of which FlightMode is active (see mode_manager.cpp) - unlike LandDetector, which only ModeLand/ModeRTL's final leg care about, a crash can happen in any mode (a diverging guided move, a physics anomaly, anything that gets the vehicle badly inverted while still armed and trying to fly). Two independent signals, each sustained for kConfirmSeconds to reject a single noisy sample: - attitude far enough past any legitimate flight envelope (well beyond NavigationCascade's own safety tilt clamp - see navigation_cascade.cpp's kSafetyTiltRad) that self-leveling isn't recovering it - a real, sustained ground impact while armed: near-zero altitude reached at a high descent rate outside of LAND's own controlled, detected touchdown (ModeLand/ModeRTL already have LandDetector for the normal case - this is for hitting the ground any *other* way)

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `bool` | `update(float roll_rad, float pitch_rad, float rel_alt_m, float climb_rate_ms, bool armed, float dt_s)` | reset()). |
| `void` | `reset()` | Member function. |

---

## FlightMode
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_base.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `void` | `on_enter(const VehicleState& state)` | hold-position mode can capture "here" without a separate call. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |
| `FlightModeId` | `id()` | Member function. |
| `std::string_view` | `get_name()` | Member function. |

---

## FlightModeId
**Kind:** `enum`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_base.hpp`  

> Custom mode enum for APX flight modes (see mode_manager.cpp's to_custom_mode_number()/from_custom_mode_number()) so standard GCS mode selection and MAV_CMD_DO_SET_MODE work reliably on the GCS side. GUIDED is the mode MAV_CMD_NAV_TAKEOFF/SET_POSITION_TARGET_*/MAV_CMD_DO_REPOSITION act on; takeoff is a sub-behavior inside ModeGuided (see mode_guided.hpp).

### Enumerators
| Value | Description |
|---|---|
| `STABILIZE` | Enumerator value |
| `ALT_HOLD` | Enumerator value |
| `AUTO` | Enumerator value |
| `GUIDED` | Enumerator value |
| `LOITER` | Enumerator value |
| `LAND` | Enumerator value |
| `POS_HOLD` | Enumerator value |
| `BRAKE` | Enumerator value |
| `AGRI_ZIGZAG` | Enumerator value |

---

## GuidedTarget
**Kind:** `struct`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/vehicle_state.hpp`  

> A GCS-commanded "go here" target (SET_POSITION_TARGET_LOCAL_NED/ GLOBAL_INT, or MAV_CMD_DO_REPOSITION), already resolved to NED-relative- to-home by the caller so modes never need to know about MAVLink frames.

---

## LandDetector
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/land_detector.hpp`  

> Shared by ModeLand and ModeRTL's final descent leg. Three independent, physically-grounded signals (not a timeout): resting altitude, near-zero climb rate, and thrust well below what hover would need (the ground - not the rotors - is supporting the vehicle's weight). All three must hold continuously for kConfirmSeconds before touchdown is declared, the same way a real landing detector debounces against a single noisy sample instead of triggering on one.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `bool` | `update(float rel_alt_m, float climb_rate_ms, float thrust, float dt_s)` | reset()). |
| `void` | `reset()` | Member function. |

---

## LandPhase
**Kind:** `enum`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_land.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `Braking` | Enumerator value |

---

## LandStage
**Kind:** `enum`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_auto.hpp`  

> LAND / RTL items are flown in stages: (RTL only) climb to RTL_ALT in place, fly level to the landing point as a stopping leg, then descend.

### Enumerators
| Value | Description |
|---|---|
| `Climb` | Enumerator value |
| `Approach` | Enumerator value |
| `Descend` | Enumerator value |

---

## ModeAgriSpray
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_agri_spray.hpp`  
**Inherits from:** `FlightMode`  

> AGRI_SPRAY: pump/relay control and spray-area path logic aren't implemented yet (needs the flow sensor/pump driver + a defined spray pattern). Until then this is a safe placeholder: hold the position it was entered at, same as ModeAuto, rather than doing nothing.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeAgriSpray()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |

---

## ModeAgriZigZag
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_agri_zigzag.hpp`  
**Inherits from:** `FlightMode`  

> AGRI_ZIGZAG: the point-A/point-B row-following pattern isn't implemented yet (needs a way for the GCS/RC to mark A and B, then row-spacing geometry on top of the same NavigationCascade GUIDED uses). Until then this is a safe placeholder: hold the position it was entered at.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeAgriZigZag()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |

---

## ModeAltHold
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_althold.hpp`  
**Inherits from:** `FlightMode`  

> ALT_HOLD: unlike POS_HOLD/LOITER, roll/pitch/yaw are controlled by the pilot's sticks while altitude is maintained automatically - it does NOT actively correct horizontal drift. Pilot roll/pitch/yaw/throttle come from VehicleState.rc_* (see mavlink_handler.cpp's MANUAL_CONTROL/ RC_CHANNELS_OVERRIDE handling for how those get populated).

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeAltHold()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |

---

## ModeAuto
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_auto.hpp`  
**Inherits from:** `FlightMode`  

> AUTO: real waypoint sequencing over whatever mission MavlinkHandler most recently uploaded (see mode_manager.hpp's load_mission()/m_mission - ModeAuto instances are recreated on every mode entry, same as every other mode, so the mission itself has to live above that, in ModeManager, and get handed to a fresh ModeAuto each time AUTO is entered). Falls back to holding the entry position/altitude - same as before - only when there's no mission loaded, so selecting AUTO with an empty mission is still the safe no-op it always was rather than dropping the vehicle.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeAuto()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |
| `void` | `load_mission(const std::vector<Waypoint>& mission)` | Member function. |
| `bool` | `set_speed_xy(float speed_ms)` | MAV_CMD_DO_CHANGE_SPEED received as a command while AUTO is active. |

---

## ModeBrake
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_brake.hpp`  
**Inherits from:** `FlightMode`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeBrake()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |

---

## ModeGuided
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_guided.hpp`  
**Inherits from:** `FlightMode`  

> GUIDED: the mode MAV_CMD_NAV_TAKEOFF, SET_POSITION_TARGET_LOCAL_NED/ GLOBAL_INT and MAV_CMD_DO_REPOSITION all act on - there is no separate "TAKEOFF" custom_mode; takeoff is a sub-behavior here (request_takeoff() sets the altitude target and lets the navigation cascade climb to it). Behavior: always holds m_target_pos_ned via NavigationCascade. On entry the target is "stay right here" (current position at time of entry, or arm-time position if entered while disarmed on the ground). A GCS target (set_guided_target) or a takeoff request (request_takeoff) just changes what "here" means - the control loop doesn't otherwise care which one moved it.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeGuided()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |
| `void` | `set_guided_target(const GuidedTarget& target)` | Member function. |
| `void` | `request_takeoff(float target_rel_alt_m)` | Member function. |

---

## ModeLand
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_land.hpp`  
**Inherits from:** `FlightMode`  

> LAND: safely brakes horizontal speed along line of travel first, then descends vertically while holding the stopped position, using LandDetector to disarm on touchdown.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeLand()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |
| `` | `landed()` | Member function. |

---

## ModeLoiter
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_loiter.hpp`  
**Inherits from:** `FlightMode`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeLoiter()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |

---

## ModeManager
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_manager.hpp`  

> Single place mode switching/transition logic lives: adding a new mode means (1) a FlightMode subclass, (2) one line in create_mode()'s switch, (3) if it should be GCS-selectable, one line in the custom-mode tables below. Nothing else in the codebase needs to change.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeManager()` | Member function. |
| `are` | `met(arming state, GPS/position // availability vs. what the target mode requires)` | Member function. |
| `bool` | `request_mode(FlightModeId id, const VehicleState& state)` | already active (no-op, returns true). |
| `bool` | `request_takeoff(float target_rel_alt_m, const VehicleState& state)` | asks the active mode to climb. |
| `bool` | `request_land(const VehicleState& state)` | MAV_CMD_NAV_LAND: switches to LAND mode. |
| `bool` | `set_guided_target(const GuidedTarget& target)` | in any other mode) - returns false if ignored for that reason. |
| `active` | `mode(AUTO only // today)` | Member function. |
| `` | `set_speed_xy(speed_ms)` | Member function. |
| `int` | `auto_mission_current_seq()` | MISSION_CURRENT/MISSION_ITEM_REACHED. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |
| `` | `id()` | Member function. |
| `` | `get_name()` | Member function. |
| `` | `can_arm()` | Member function. |
| `bool` | `stick_disarm_allowed()` | STABILIZE, where throttle is manual. |
| `uint32_t` | `to_custom_mode_number(FlightModeId id)` | Standard custom mode numbering for seamless GCS interoperability. |
| `std::optional<FlightModeId>` | `from_custom_mode_number(uint32_t custom_mode)` | Member function. |

---

## ModePosHold
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_poshold.hpp`  
**Inherits from:** `FlightMode`  

> POS_HOLD: holds the 3D position/heading captured at entry using the same NavigationCascade GUIDED uses, but does not accept an external GCS target - maintains stationary lock at current coordinates.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModePosHold(FlightModeId represented_as = FlightModeId::POS_HOLD)` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |
| `std::string_view` | `get_name()` | Member function. |

---

## ModeRTL
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_rtl.hpp`  
**Inherits from:** `FlightMode`  

> RTL: climb to m_rtl_altitude (if not already above it) at the current position, navigate to home (NED origin) at that altitude, then hand off to the same descent+LandDetector logic ModeLand uses. Position/velocity in VehicleState are already NED-relative-to-home (see apx_main.cpp/apx_gazebo_plugin.cpp), so "home" is simply (0,0).

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeRTL()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |

---

## ModeStabilize
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_stabilize.hpp`  
**Inherits from:** `FlightMode`  

> STABILIZE: pilot controls attitude (roll/pitch tilt), yaw rate, and collective thrust.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ModeStabilize()` | Member function. |
| `void` | `on_enter(const VehicleState& state)` | Member function. |
| `void` | `on_exit()` | Member function. |
| `ControlOutput` | `update(const VehicleState& state)` | Member function. |

---

## NavigationCascade
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/navigation_cascade.hpp`  

> Shared Position -> Attitude -> Rate -> mixer-command cascade used by every mode that needs to hold or navigate to a 3D point (ModeGuided, ModePosHold, ModeLoiter, ModeLand). This is the one place that pipeline lives - a new position-aware mode only has to decide *what* target to feed in each tick (see mode_manager.hpp/.cpp for how modes themselves are registered/switched), not re-derive how a target becomes a mixer command.

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `NavigationCascade()` | Member function. |
| `ControlOutput` | `compute(const float target_pos_ned[3], bool has_yaw, float target_yaw, const VehicleState& state, bool use_forced_climb_rate = false, float forced_climb_rate_ms = 0.0f, const float vel_ff_ne[2] = nullptr, const float accel_ff_ne[2] = nullptr)` | already-shaped trajectory and is not slew-limited again. |
| `void` | `reset()` | Member function. |
| `` | `set_max_horizontal_speed(speed_ms)` | Member function. |
| `` | `set_max_vertical_speed(up_ms, down_ms)` | Member function. |

---

## PathSegment
**Kind:** `struct`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/path_tracker.hpp`  

---

## PathTracker
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/path_tracker.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `PathTracker()` | Member function. |
| `void` | `reset()` | Member function. |
| `void` | `set_segment(const PathSegment& seg, const VehicleState& state)` | Member function. |
| `PathTrackerOutput` | `update(const VehicleState& state)` | Member function. |
| `in` | `flight(DO_CHANGE_SPEED)` | Member function. |
| `void` | `set_cruise_speed(float speed_ms)` | the reference ramps to it with the normal accel/jerk limits. |
| `float` | `sqrt_controller(float error, float p, float limit, float dt)` | one step. |

---

## PathTrackerLimits
**Kind:** `struct`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/path_tracker.hpp`  

---

## PathTrackerOutput
**Kind:** `struct`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/path_tracker.hpp`  

---

## Phase
**Kind:** `enum`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/path_tracker.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `Straight` | Enumerator value |
| `Corner` | Enumerator value |
| `Done` | Enumerator value |

---

## Result
**Kind:** `enum`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/stick_arm_detector.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `NONE` | Enumerator value |
| `ARM` | Enumerator value |
| `DISARM` | Enumerator value |

---

## RtlState
**Kind:** `enum`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_rtl.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `CLIMBING` | Enumerator value |
| `RETURNING` | Enumerator value |
| `DESCENDING` | Enumerator value |

---

## SprayState
**Kind:** `enum`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_agri_spray.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `IDLE` | Enumerator value |
| `SPRAYING` | Enumerator value |
| `PAUSED` | Enumerator value |
| `DONE` | Enumerator value |

---

## Sprayer
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/sprayer.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `Sprayer&` | `instance()` | Member function. |
| `void` | `load_params()` | Reads SPRAY_* parameters (call after ParamManager::init_defaults()). |
| `bool` | `run(bool on)` | sprayer is disabled (SPRAY_ENABLE=0). |
| `void` | `set_pump_pwm_manual(uint16_t pwm)` | Direct PWM control via MAV_CMD_DO_SET_SERVO (1100 = full off, 1900 = full on) |
| `void` | `set_spinner_pwm_manual(uint16_t pwm)` | Member function. |
| `void` | `clear_manual_control()` | Member function. |
| `void` | `set_flow_rate(float rate_0_to_1)` | Flow rate fraction (0.0 = full off, 1.0 = full on) |
| `void` | `update(const VehicleState& state)` | Call every control tick. |
| `uint16_t` | `pump_pwm()` | Member function. |
| `uint16_t` | `spinner_pwm()` | Member function. |

---

## StickArmDetector
**Kind:** `class`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/stick_arm_detector.hpp`  

> RC-stick arm/disarm gesture recognition: throttle held at minimum + yaw stick held full right for a sustained hold arms; throttle at minimum + yaw full left disarms. Ticked every frame from apx_main.cpp (see that file) alongside ModeManager::update() - this class only *recognizes* the gesture, it doesn't own the armed flag itself (that's MavlinkHandler::set_armed(), same authority MAV_CMD_COMPONENT_ARM_DISARM already uses) or decide whether arming should actually be allowed right now (that's ModeManager::can_arm(), which the caller should also check before acting on an ARM result).

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `Result` | `update(bool armed, bool rc_valid, float rc_throttle, float rc_yaw, float dt_s)` | DISARM the one tick the hold duration completes; NONE otherwise. |
| `void` | `reset()` | Member function. |

---

## VehicleState
**Kind:** `struct`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/vehicle_state.hpp`  

> Snapshot of everything a FlightMode needs to make a decision each tick. Position/velocity are NED, relative to the home position established at boot (see apx_main.cpp --home-lat/--home-lon/--home-alt). Attitude is both euler (for logging/simple checks) and quaternion (for the attitude controller, which needs it to avoid gimbal-lock issues that naive euler-error math has).

---

## Waypoint
**Kind:** `struct`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_auto.hpp`  

> One mission item, already resolved to NED-relative-to-home by the caller (MavlinkHandler - see its handle_mission_item_int()) so this module never needs to know about MAVLink frames/lat-lon, the same pattern GuidedTarget already uses for SET_POSITION_TARGET_*.

---

## ZigZagState
**Kind:** `enum`  
**Namespace:** `apx::modes`  
**Header:** `modules/modes/mode_agri_zigzag.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `SET_A` | Enumerator value |
| `SET_B` | Enumerator value |
| `FLYING_ROW` | Enumerator value |
| `TURNING` | Enumerator value |
| `COMPLETE` | Enumerator value |

---
