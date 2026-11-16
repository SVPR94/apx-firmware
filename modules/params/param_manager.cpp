// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "param_manager.hpp"
#include <cstring>
#include <cstdio>
#include <algorithm> // std::clamp - previously relied on another header pulling this in transitively
#include <iostream> // For temporary logging, should be replaced with HAL log

namespace apx::gcs {

ParamManager& ParamManager::get_instance() {
    static ParamManager instance;
    return instance;
}

void ParamManager::register_param(const ParamDefinition& def) {
    apx::kernel::LockGuard lock(m_mutex);
    std::string name_str(def.name);
    m_definitions[name_str] = def;
    // Set default value if not loaded from storage
    if (m_values.find(name_str) == m_values.end()) {
        m_values[name_str] = def.default_value;
    }
}

// Setters update the value under m_mutex, then notify after releasing it:
// callbacks may read parameters (get_*), which would self-deadlock on the
// non-recursive hosted mutex if they ran with the lock held.
bool ParamManager::set_float(const std::string& name, float val) {
    ParamValue pv;
    {
        apx::kernel::LockGuard lock(m_mutex);
        auto it = m_definitions.find(name);
        if (it == m_definitions.end() || it->second.type != ParamType::FLOAT) {
            return false;
        }
        pv = ParamValue(std::clamp(val, it->second.min_value.f, it->second.max_value.f));
        m_values[name] = pv;
    }
    notify_callbacks(name, pv);
    return true;
}

bool ParamManager::set_int(const std::string& name, int32_t val) {
    ParamValue pv;
    {
        apx::kernel::LockGuard lock(m_mutex);
        auto it = m_definitions.find(name);
        if (it == m_definitions.end() || it->second.type != ParamType::INT32) {
            return false;
        }
        pv = ParamValue(std::clamp(val, it->second.min_value.i, it->second.max_value.i));
        m_values[name] = pv;
    }
    notify_callbacks(name, pv);
    return true;
}

bool ParamManager::set_bool(const std::string& name, bool val) {
    ParamValue pv(val);
    {
        apx::kernel::LockGuard lock(m_mutex);
        auto it = m_definitions.find(name);
        if (it == m_definitions.end() || it->second.type != ParamType::BOOL) {
            return false;
        }
        m_values[name] = pv;
    }
    notify_callbacks(name, pv);
    return true;
}

std::optional<float> ParamManager::get_float(const std::string& name) const {
    apx::kernel::LockGuard lock(m_mutex);
    auto it = m_values.find(name);
    if (it != m_values.end() && it->second.type == ParamType::FLOAT) {
        return it->second.f;
    }
    return std::nullopt;
}

std::optional<int32_t> ParamManager::get_int(const std::string& name) const {
    apx::kernel::LockGuard lock(m_mutex);
    auto it = m_values.find(name);
    if (it != m_values.end() && it->second.type == ParamType::INT32) {
        return it->second.i;
    }
    return std::nullopt;
}

float ParamManager::get_float_or(const std::string& name, float fallback) const {
    auto v = get_float(name);
    return v.has_value() ? *v : fallback;
}

int32_t ParamManager::get_int_or(const std::string& name, int32_t fallback) const {
    auto v = get_int(name);
    return v.has_value() ? *v : fallback;
}

std::optional<bool> ParamManager::get_bool(const std::string& name) const {
    apx::kernel::LockGuard lock(m_mutex);
    auto it = m_values.find(name);
    if (it != m_values.end() && it->second.type == ParamType::BOOL) {
        return it->second.b;
    }
    return std::nullopt;
}

std::optional<ParamValue> ParamManager::get_param(const std::string& name) const {
    apx::kernel::LockGuard lock(m_mutex);
    auto it = m_values.find(name);
    if (it != m_values.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<std::string> ParamManager::get_all_param_names() const {
    apx::kernel::LockGuard lock(m_mutex);
    std::vector<std::string> names;
    names.reserve(m_definitions.size());
    for (const auto& [name, _] : m_definitions) {
        names.push_back(name);
    }
    return names;
}

const ParamDefinition* ParamManager::get_param_def(const std::string& name) const {
    apx::kernel::LockGuard lock(m_mutex);
    auto it = m_definitions.find(name);
    if (it != m_definitions.end()) {
        return &it->second;
    }
    return nullptr;
}

void ParamManager::add_change_callback(ParamChangeCallback cb) {
    apx::kernel::LockGuard lock(m_mutex);
    m_callbacks.push_back(cb);
}

// Must be called without m_mutex held (see set_float()). The callback list
// is copied under the lock so add_change_callback() can't race the loop.
void ParamManager::notify_callbacks(const std::string& name, const ParamValue& val) {
    std::vector<ParamChangeCallback> callbacks;
    {
        apx::kernel::LockGuard lock(m_mutex);
        if (m_callbacks.empty()) {
            return;
        }
        callbacks = m_callbacks;
    }
    for (const auto& cb : callbacks) {
        cb(name, val);
    }
}

bool ParamManager::load_from_storage() {
    // TODO: Implement actual HAL storage loading
    return true;
}

bool ParamManager::save_to_storage() {
    // TODO: Implement actual HAL storage saving
    return true;
}

void ParamManager::init_defaults() {
    auto add_f = [this](const char* name, float def, float min, float max, const char* desc) {
        ParamDefinition d{};
        std::strncpy(d.name, name, 16);
        d.name[16] = '\0';
        d.type = ParamType::FLOAT;
        d.default_value = ParamValue(def);
        d.min_value = ParamValue(min);
        d.max_value = ParamValue(max);
        d.description = desc;
        register_param(d);
    };

    auto add_i = [this](const char* name, int32_t def, int32_t min, int32_t max, const char* desc) {
        ParamDefinition d{};
        std::strncpy(d.name, name, 16);
        d.name[16] = '\0';
        d.type = ParamType::INT32;
        d.default_value = ParamValue(def);
        d.min_value = ParamValue(min);
        d.max_value = ParamValue(max);
        d.description = desc;
        register_param(d);
    };

    // Core System Parameters
    add_i("SYSID_THISMAV",   1, 1, 255, "System ID of this vehicle");
    add_i("SYSID_MYGCS",   255, 1, 255, "System ID of primary GCS");
    add_i("FRAME_CLASS",     1, 0, 15,  "Vehicle frame class: 1=Quad");
    add_i("FRAME_TYPE",      1, 0, 15,  "Vehicle frame type: 1=X");
    add_i("ARMING_CHECK",    1, 0, 1,   "Arming checks enabled");
    // DISARM_DELAY (s): auto-disarm after this long landed with
    // throttle at zero (LOITER). 0 disables. APX default 30 s.
    add_f("DISARM_DELAY",   30.0f, 0.0f, 127.0f, "Auto-disarm delay after landing s (0=off)");
    add_i("FS_THR_ENABLE",   1, 0, 3,   "Failsafe throttle action");
    add_i("FS_GCS_ENABLE",   1, 0, 3,   "Failsafe GCS action");

    // Safety Geofence Parameters (standard names & units)
    add_i("FENCE_ENABLE",    0, 0, 1,    "Enable safety geofence (0=disabled, 1=enabled)");
    add_i("FENCE_TYPE",      5, 1, 7,    "Fence type bitmask: 1=Alt max, 2=Circle, 4=Polygon, 5=Alt+Polygon");
    add_i("FENCE_ACTION",    1, 0, 4,    "Fence breach action: 0=Report only, 1=RTL, 2=Land, 4=Brake");
    add_f("FENCE_ALT_MAX", 100.0f, 0.0f, 1000.0f, "Fence maximum altitude m (0=unlimited)");
    add_f("FENCE_MARGIN",    2.0f, 0.0f, 50.0f,   "Fence breach margin m");

    // Battery Parameters
    add_i("BATT_MONITOR",    4, 0, 10,  "Battery monitor type");
    add_f("BATT_CAPACITY", 5000.0f, 100.0f, 100000.0f, "Battery capacity mAh");
    add_f("BATT_LOW_VOLT", 10.5f, 5.0f, 50.0f, "Battery low voltage threshold");
    add_f("BATT_CRT_VOLT", 10.0f, 5.0f, 50.0f, "Battery critical voltage threshold");

    // Flight & Navigation Parameters
    add_f("PILOT_THR_FILT",  0.0f, 0.0f, 100.0f, "Pilot throttle filter cutoff Hz");
    // AUTO mission navigation (WPNAV_* names/units). PathTracker
    // shapes a jerk-limited reference trajectory from these: WPNAV_SPEED is
    // the cruise ceiling (also caps GUIDED), WPNAV_ACCEL the along-track and
    // cornering accel, WPNAV_JERK how fast accel ramps in/out, and
    // WPNAV_RADIUS both the stop-waypoint arrival radius and how far a
    // fly-through corner may pass from the waypoint (a mission item's own
    // param2 overrides it).
    // WPNAV_SPEED default 500 (5 m/s): 1000 gave 10 m/s missions that
    // overshot corners and the final waypoint by >10 m in SITL.
    add_f("WPNAV_SPEED",   500.0f, 20.0f, 2000.0f, "Waypoint horizontal speed cm/s");
    add_f("WPNAV_RADIUS",  200.0f, 10.0f, 1000.0f, "Waypoint radius / corner cut cm");
    add_f("WPNAV_SPEED_UP", 250.0f, 10.0f, 1000.0f, "Waypoint climb speed cm/s");
    add_f("WPNAV_SPEED_DN", 150.0f, 10.0f, 500.0f, "Waypoint descent speed cm/s");
    add_f("WPNAV_JERK",      5.0f, 1.0f, 20.0f, "Waypoint horizontal jerk m/s/s/s");
    // Fly-through corner shape. 1: spiral-arc-spiral blend whose curvature
    // ramps in and out, so lateral accel builds at no more than WPNAV_JERK
    // and the path passes within WPNAV_RADIUS of the waypoint. 0: the older
    // circular arc (lateral accel steps in at the arc entry).
    add_i("WPNAV_BLEND", 1, 0, 1, "Corner shape: 0=Circular arc,1=Curvature-continuous blend");
    // SITL only: fly and report height/climb rate from the simulator's
    // truth (1) instead of the vertical estimator fed by the simulated IMU
    // and baro (0, like real hardware).
    add_i("SIM_TRUTH_ALT", 0, 0, 1, "SITL height source: 0=Estimator (baro+IMU),1=Simulator truth");
    add_i("WP_YAW_BEHAVIOR", 2, 0, 4,
          "Auto yaw: 0=Constant heading,1=Face next WP,2=Face next WP except RTL,3=Along course,4=Fixed/Condition yaw");
    add_f("AUTO_YAW_HEAD", -1.0f, -1.0f, 360.0f, "Auto mode fixed yaw heading deg (-1=entry heading)");
    add_f("RTL_ALT",      1500.0f, 0.0f, 10000.0f, "RTL return altitude cm");
    // AUTO LAND/RTL descent: LAND_SPEED_HIGH (0 = use
    // WPNAV_SPEED_DN) above LAND_ALT_LOW, LAND_SPEED below it.
    add_f("LAND_SPEED",      50.0f, 30.0f, 200.0f, "Final landing descent speed cm/s");
    add_f("LAND_SPEED_HIGH",  0.0f, 0.0f, 500.0f, "Landing descent speed above LAND_ALT_LOW cm/s");
    add_f("LAND_ALT_LOW",  1000.0f, 100.0f, 10000.0f, "Altitude to slow to LAND_SPEED cm");
    add_f("LAND_REPOS_SPEED", 150.0f, 20.0f, 500.0f, "Landing horizontal reposition max speed cm/s");

    // Crop sprayer (SPRAY_* names/units) - see modules/modes/
    // sprayer.hpp. Started/stopped by MAV_CMD_DO_SPRAYER or DO_SET_SERVO.
    // SPRAY_PUMP_CH/SPRAY_SPIN_CH are APX's servo output mapping (1-16).
    // Servos 1-8 are dedicated to motors; 9-12 are auxiliary outputs (pump=9, spinner=10).
    add_i("SPRAY_ENABLE",      1, 0, 1, "Sprayer enable");
    add_f("SPRAY_PUMP_RATE", 10.0f, 0.0f, 100.0f, "Pump % per 1 m/s ground speed");
    add_f("SPRAY_PUMP_MIN",   0.0f, 0.0f, 100.0f, "Minimum pump % while spraying");
    add_f("SPRAY_SPEED_MIN", 100.0f, 0.0f, 1000.0f, "Min ground speed to spray cm/s");
    add_i("SPRAY_SPINNER",  1300, 1000, 2000, "Spinner PWM while spraying");
    add_i("SPRAY_PUMP_CH",     9, 1, 16, "Servo output for the pump (aux ch 9)");
    add_i("SPRAY_SPIN_CH",    10, 1, 16, "Servo output for the spinner (aux ch 10)");
    add_i("SPRAY_PWM_MIN",  1100, 800, 2200, "Sprayer pump min PWM us (full off)");
    add_i("SPRAY_PWM_MAX",  1900, 800, 2200, "Sprayer pump max PWM us (full on)");
    add_f("BRAKE_ACCEL",   250.0f, 50.0f, 1000.0f, "Brake mode horizontal deceleration cm/s/s");
    // LOIT_SPEED - deliberately its own parameter, not
    // WPNAV_SPEED, even though APX's ModeLoiter clamps it to never exceed
    // WPNAV_SPEED (see mode_loiter.cpp's constructor) -
    // keeps pilot-stick loiter speed independently tunable from auto-
    // mission speed too. Default (180 cm/s) is conservative and
    // verified live at this value after stability testing (see mode_loiter.hpp).
    add_f("LOIT_SPEED",    180.0f, 20.0f, 2000.0f, "Pilot-commanded loiter speed cm/s");

    // Motor Output: MOT_SPIN_ARM - once armed, motors
    // idle-spin at this thrust floor instead of sitting at true zero, providing
    // visible/audible "I am armed" feedback to the pilot. Applied in
    // apx_main.cpp, not per-mode, so it's consistent regardless of which
    // FlightMode is active. Default is 0.10 (10%).
    add_f("MOT_SPIN_ARM", 0.10f, 0.0f, 0.30f, "Idle motor output fraction while armed");

    // MOT_THST_HOVER: PositionController's altitude
    // climb-rate PID output is a CORRECTION around this baseline, not the
    // whole thrust command (see position_controller.cpp) - without it,
    // the PID had to build 100% of hover-equivalent thrust out of P+I+D
    // from a cold start every time, which is the structural reason behind
    // the "jerky takeoff" class of issue. Starts at 0.5 (unbiased baseline)
    // and self-corrects via slow in-flight learning (see
    // PositionController::update_altitude()).
    add_f("MOT_THST_HOVER", 0.5f, 0.1f, 0.9f, "Learned hover thrust fraction");

    // Rate Controller PID Parameters
    add_f("ATC_RAT_RLL_P", 0.135f, 0.01f, 1.0f, "Roll rate PID P gain");
    add_f("ATC_RAT_RLL_I", 0.135f, 0.01f, 1.0f, "Roll rate PID I gain");
    add_f("ATC_RAT_RLL_D", 0.0036f, 0.0001f, 0.1f, "Roll rate PID D gain");
    add_f("ATC_RAT_PIT_P", 0.135f, 0.01f, 1.0f, "Pitch rate PID P gain");
    add_f("ATC_RAT_PIT_I", 0.135f, 0.01f, 1.0f, "Pitch rate PID I gain");
    add_f("ATC_RAT_PIT_D", 0.0036f, 0.0001f, 0.1f, "Pitch rate PID D gain");
    // Yaw rate loop: 0.6/0.06 (was 0.18/0.018). Yaw is turned only by rotor
    // reaction torque (~2 % of thrust), so on the heavy hex the 0.18 loop
    // closed at ~1-1.3 rad/s - slower than the 4.5/s yaw angle loop above
    // it - and heading rang at ~0.3 Hz (damping ~0.25) after every turn:
    // 16-25 deg peak-to-peak on straight AUTO legs in Gazebo, 1.2-1.6 deg
    // with 0.6/0.06. Tuned on the SITL airframes; verify on hardware.
    add_f("ATC_RAT_YAW_P", 0.600f, 0.01f, 2.0f, "Yaw rate PID P gain");
    add_f("ATC_RAT_YAW_I", 0.060f, 0.001f, 0.5f, "Yaw rate PID I gain");
    add_f("ATC_RAT_YAW_D", 0.0000f, 0.0f, 0.1f, "Yaw rate PID D gain");

    // Attitude (angle) P gains - NavigationCascade/ModeStabilize/ModeAltHold's
    // AttitudeController::set_proportional_gain() input.
    add_f("ATC_ANG_RLL_P", 4.5f, 1.0f, 12.0f, "Roll angle P gain");
    add_f("ATC_ANG_PIT_P", 4.5f, 1.0f, 12.0f, "Pitch angle P gain");
    add_f("ATC_ANG_YAW_P", 4.5f, 1.0f, 12.0f, "Yaw angle P gain");

    // Max commandable body rate - AttitudeController::set_rate_limits()
    // input. Defaults are APX's verified-safe ceiling (1.57/1.57/0.78 rad/s =
    // 90/90/45 deg/s - see attitude_controller.hpp).
    add_f("ATC_RATE_R_MAX", 90.0f, 10.0f, 360.0f, "Max roll rate deg/s");
    add_f("ATC_RATE_P_MAX", 90.0f, 10.0f, 360.0f, "Max pitch rate deg/s");
    add_f("ATC_RATE_Y_MAX", 45.0f, 5.0f, 180.0f, "Max yaw rate deg/s");

    // Angular-ACCELERATION limit on the commanded rate itself (how fast
    // desired_rate is allowed to change, not its magnitude) -
    // AttitudeController::set_accel_limits() input. Default is disabled (0).
    add_f("ATC_ACCEL_R_MAX", 0.0f, 0.0f, 3000.0f, "Max roll accel deg/s/s (0=unlimited)");
    add_f("ATC_ACCEL_P_MAX", 0.0f, 0.0f, 3000.0f, "Max pitch accel deg/s/s (0=unlimited)");
    add_f("ATC_ACCEL_Y_MAX", 0.0f, 0.0f, 3000.0f, "Max yaw accel deg/s/s (0=unlimited)");

    // Max lean angle - NavigationCascade's kMaxTiltRad, ANGLE_MAX
    // (centidegrees). APX defaults to 2500 (25deg): live SITL testing
    // found rate-loop gains verified stable up to roughly this ceiling on
    // APX's airframe model (see navigation_cascade.cpp).
    add_i("ANGLE_MAX", 2500, 1000, 4500, "Max lean angle centidegrees");

    // Position/velocity loop speed & accel ceilings -
    // WPNAV_SPEED (cm/s) and WPNAV_ACCEL (cm/s^2), and PILOT_SPEED_UP/DN
    // (cm/s) for pilot-commanded climb/descent in ALT_HOLD/LOITER.
    add_f("WPNAV_ACCEL",   250.0f, 50.0f, 1000.0f, "Horizontal accel limit cm/s/s");
    add_f("PILOT_SPEED_UP", 250.0f, 50.0f, 500.0f, "Max pilot climb speed cm/s");
    add_f("PILOT_SPEED_DN", 150.0f, 50.0f, 500.0f, "Max pilot descent speed cm/s");

    // Position controller gains - PositionController's setters.
    add_f("PSC_POSXY_P",   1.0f, 0.1f, 3.0f, "Horizontal position P gain");
    add_f("PSC_VELXY_P",   2.0f, 0.1f, 6.0f, "Horizontal velocity P gain");
    add_f("PSC_VELXY_I",   1.0f, 0.0f, 3.0f, "Horizontal velocity I gain");
    add_f("PSC_VELXY_D",   0.5f, 0.0f, 2.0f, "Horizontal velocity D gain");
    add_f("PSC_POSZ_P",    1.0f, 0.1f, 3.0f, "Altitude position P gain");
    add_f("PSC_VELZ_P",    1.5f, 0.1f, 6.0f, "Climb-rate P gain (drives thrust)");
    add_f("PSC_VELZ_I",    0.2f, 0.0f, 2.0f, "Climb-rate I gain");
    add_f("PSC_VELZ_D",    0.05f, 0.0f, 0.5f, "Climb-rate D gain");

    // Estimator & Simulation Parameters
    add_i("EK3_ENABLE",      1, 0, 1,   "Enable EKF3 estimator");
    add_f("SIM_SPEEDUP",   1.0f, 0.1f, 50.0f, "Simulation speedup factor");

    // --- RC input mapping & calibration ---
    // RCMAP_* (which physical RC channel is roll/pitch/yaw/throttle) -
    // defaults: roll=ch1, pitch=ch2, throttle=ch3, yaw=ch4.
    add_i("RCMAP_ROLL",     1, 1, 8, "RC channel for roll");
    add_i("RCMAP_PITCH",    2, 1, 8, "RC channel for pitch");
    add_i("RCMAP_THROTTLE", 3, 1, 8, "RC channel for throttle");
    add_i("RCMAP_YAW",      4, 1, 8, "RC channel for yaw");

    // Per-channel RC calibration (PWM microseconds) -
    // RC1_MIN/MAX/TRIM .. RC8_MIN/MAX/TRIM. Channels 1-4 default to the
    // roll/pitch/throttle/yaw RCMAP_* assignment above; 5-8 as spare/aux.
    for (int ch = 1; ch <= 8; ++ch) {
        char name[17];
        std::snprintf(name, sizeof(name), "RC%d_MIN", ch);
        add_i(name, 1100, 800, 2200, "RC channel min PWM us");
        std::snprintf(name, sizeof(name), "RC%d_MAX", ch);
        add_i(name, 1900, 800, 2200, "RC channel max PWM us");
        std::snprintf(name, sizeof(name), "RC%d_TRIM", ch);
        add_i(name, 1500, 800, 2200, "RC channel trim/center PWM us");
        std::snprintf(name, sizeof(name), "RC%d_REV", ch);
        add_i(name, 1, -1, 1, "RC channel reverse (-1=reversed)");
        std::snprintf(name, sizeof(name), "RC%d_DZ", ch);
        add_i(name, 30, 0, 200, "RC channel deadzone PWM us");
    }

    // --- Compass calibration & mapping (up to 3 compasses) ---
    // Device IDs (0 = not detected/uncalibrated), per-axis hard-iron offsets
    // (mGauss), and magnetic declination (radians).
    add_i("COMPASS_DEV_ID",  0, 0, 0x7FFFFFFF, "Compass 1 device ID");
    add_i("COMPASS_DEV_ID2", 0, 0, 0x7FFFFFFF, "Compass 2 device ID");
    add_i("COMPASS_DEV_ID3", 0, 0, 0x7FFFFFFF, "Compass 3 device ID");
    add_f("COMPASS_OFS_X",  0.0f, -2000.0f, 2000.0f, "Compass 1 offset X mGauss");
    add_f("COMPASS_OFS_Y",  0.0f, -2000.0f, 2000.0f, "Compass 1 offset Y mGauss");
    add_f("COMPASS_OFS_Z",  0.0f, -2000.0f, 2000.0f, "Compass 1 offset Z mGauss");
    add_f("COMPASS2_OFS_X", 0.0f, -2000.0f, 2000.0f, "Compass 2 offset X mGauss");
    add_f("COMPASS2_OFS_Y", 0.0f, -2000.0f, 2000.0f, "Compass 2 offset Y mGauss");
    add_f("COMPASS2_OFS_Z", 0.0f, -2000.0f, 2000.0f, "Compass 2 offset Z mGauss");
    add_f("COMPASS3_OFS_X", 0.0f, -2000.0f, 2000.0f, "Compass 3 offset X mGauss");
    add_f("COMPASS3_OFS_Y", 0.0f, -2000.0f, 2000.0f, "Compass 3 offset Y mGauss");
    add_f("COMPASS3_OFS_Z", 0.0f, -2000.0f, 2000.0f, "Compass 3 offset Z mGauss");
    add_f("COMPASS_DEC",    0.0f, -3.15f, 3.15f, "Magnetic declination radians");

    // --- IMU (INS) accelerometer calibration ---
    // INS_ACCSCAL_{X,Y,Z} (per-axis scale factor, 1.0 = uncalibrated/identity)
    // and INS_ACCOFFS_{X,Y,Z} (per-axis bias, m/s/s).
    add_f("INS_ACCSCAL_X", 1.0f, 0.8f, 1.2f, "Accel 1 scale factor X");
    add_f("INS_ACCSCAL_Y", 1.0f, 0.8f, 1.2f, "Accel 1 scale factor Y");
    add_f("INS_ACCSCAL_Z", 1.0f, 0.8f, 1.2f, "Accel 1 scale factor Z");
    add_f("INS_ACCOFFS_X", 0.0f, -3.5f, 3.5f, "Accel 1 offset X m/s/s");
    add_f("INS_ACCOFFS_Y", 0.0f, -3.5f, 3.5f, "Accel 1 offset Y m/s/s");
    add_f("INS_ACCOFFS_Z", 0.0f, -3.5f, 3.5f, "Accel 1 offset Z m/s/s");

    // --- Flight mode channel bank ---
    // Custom flight mode numbering FLTMODE1..6 (see ModeManager::to_custom_mode_number());
    // factory default is STABILIZE (0) for all six banks until the pilot configures them.
    add_i("FLTMODE1", 0, 0, 101, "Flight mode 1 (custom mode number)");
    add_i("FLTMODE2", 0, 0, 101, "Flight mode 2 (custom mode number)");
    add_i("FLTMODE3", 0, 0, 101, "Flight mode 3 (custom mode number)");
    add_i("FLTMODE4", 0, 0, 101, "Flight mode 4 (custom mode number)");
    add_i("FLTMODE5", 0, 0, 101, "Flight mode 5 (custom mode number)");
    add_i("FLTMODE6", 0, 0, 101, "Flight mode 6 (custom mode number)");

    // --- Servo channel output functions ---
    // Channels 1-8: Multi-rotor motors (33=Motor1..40=Motor8)
    // Channels 9-12: Auxiliary outputs (9: Sprayer pump 22, 10: Sprayer spinner 23, 11-12: Aux/Disabled 0)
    for (int ch = 1; ch <= 8; ++ch) {
        char name[17];
        std::snprintf(name, sizeof(name), "SERVO%d_FUNCTION", ch);
        add_i(name, 32 + ch, 0, 120, "Servo output function (33=Motor1..40=Motor8)");
    }
    add_i("SERVO9_FUNCTION", 22, 0, 120, "Servo output function (22=Sprayer pump)");
    add_i("SERVO10_FUNCTION", 23, 0, 120, "Servo output function (23=Sprayer spinner)");
    add_i("SERVO11_FUNCTION", 0, 0, 120, "Servo output function (0=Disabled/Aux)");
    add_i("SERVO12_FUNCTION", 0, 0, 120, "Servo output function (0=Disabled/Aux)");

    // --- Multi-directional Radar & Terrain-Following Parameters ---
    add_i("RADAR_ENABLE",       1, 0, 1,    "Master radar enable (0=Disable, 1=Enable)");
    add_f("RADAR_RATE",        20.0f, 1.0f, 100.0f, "Radar telemetry broadcast rate Hz");
    add_i("RADAR_TIMEOUT_MS", 500, 50, 5000, "Radar sensor timeout in milliseconds");

    add_i("TERRAIN_ENABLE",     1, 0, 1,    "Terrain following enable (0=Disable, 1=Enable)");
    add_f("TERRAIN_ALT_TGT",    2.0f, 0.5f, 50.0f, "Target terrain clearance altitude in meters");
    add_f("TERRAIN_MIN_M",      2.0f, 0.2f, 15.0f,  "Min altimeter reading range for terrain effect (m)");
    add_f("TERRAIN_MAX_M",     30.0f, 5.0f, 100.0f, "Max altimeter reading range for terrain effect (m)");
    add_i("TERRAIN_SRC",        0, 0, 1,    "Terrain reference source (0=Downward radar, 1=Relative)");
    add_f("TERRAIN_MAX_TILT",  35.0f, 10.0f, 60.0f, "Max vehicle tilt for terrain radar gating (deg)");

    // 10 Radar Channels (1:Down, 2:Front, 3:FR, 4:Right, 5:RR, 6:Rear, 7:RL, 8:Left, 9:FL, 10:Top)
    static const uint8_t kParamOrientations[10] = { 25, 0, 1, 2, 3, 4, 5, 6, 7, 24 };
    static const char* kParamNames[10] = {
        "Down", "Front", "Front-Right", "Right", "Rear-Right", "Rear", "Rear-Left", "Left", "Front-Left", "Top"
    };
    for (int r = 1; r <= 10; ++r) {
        char name[17];
        char desc[64];
        std::snprintf(name, sizeof(name), "RADAR%d_ENABLE", r);
        std::snprintf(desc, sizeof(desc), "Radar %s enable (0/1)", kParamNames[r - 1]);
        add_i(name, 1, 0, 1, desc);

        std::snprintf(name, sizeof(name), "RADAR%d_ORIENT", r);
        std::snprintf(desc, sizeof(desc), "Radar %s orientation enum", kParamNames[r - 1]);
        add_i(name, kParamOrientations[r - 1], 0, 25, desc);

        std::snprintf(name, sizeof(name), "RADAR%d_MIN_CM", r);
        std::snprintf(desc, sizeof(desc), "Radar %s min valid distance cm", kParamNames[r - 1]);
        add_f(name, (r == 1) ? 200.0f : 20.0f, 2.0f, 500.0f, desc);

        std::snprintf(name, sizeof(name), "RADAR%d_MAX_CM", r);
        std::snprintf(desc, sizeof(desc), "Radar %s max valid distance cm", kParamNames[r - 1]);
        add_f(name, (r == 1) ? 3000.0f : 4000.0f, 100.0f, 10000.0f, desc);

        std::snprintf(name, sizeof(name), "RADAR%d_POS_X", r);
        add_f(name, 0.0f, -5.0f, 5.0f, "Radar mount offset X m");
        std::snprintf(name, sizeof(name), "RADAR%d_POS_Y", r);
        add_f(name, 0.0f, -5.0f, 5.0f, "Radar mount offset Y m");
        std::snprintf(name, sizeof(name), "RADAR%d_POS_Z", r);
        add_f(name, 0.0f, -5.0f, 5.0f, "Radar mount offset Z m");
    }
}

} // namespace apx::gcs
