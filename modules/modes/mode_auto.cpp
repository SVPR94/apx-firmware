// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_auto.hpp"
#include "param_manager.hpp"
#include "sprayer.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace apx::modes {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravityMs2 = 9.81f;
constexpr float kDegToRad = 0.017453293f;
constexpr float kStickDeadband = 0.05f;
// RTL climbs in place first only when this far below RTL_ALT.
constexpr float kRtlClimbMarginM = 0.5f;
// Position-controller accel limit as a multiple of WPNAV_ACCEL.
constexpr float kAccelHeadroom = 1.5f;
// Hard ceiling for a DO_CHANGE_SPEED request (WPNAV_SPEED's own maximum).
constexpr float kMaxSpeedMs = 20.0f;
// DO_CHANGE_SPEED speed value meaning "return to the default speed".
constexpr float kSpeedResetValue = -2.0f;

float wrap_pi(float angle) {
    while (angle > kPi) angle -= 2.0f * kPi;
    while (angle < -kPi) angle += 2.0f * kPi;
    return angle;
}

bool is_nav_item(uint16_t command) {
    return command == ModeAuto::kCmdNavWaypoint || command == ModeAuto::kCmdNavTakeoff ||
           command == ModeAuto::kCmdNavLand || command == ModeAuto::kCmdNavReturnToLaunch;
}

bool is_condition_item(uint16_t command) {
    return command == ModeAuto::kCmdConditionYaw || command == ModeAuto::kCmdConditionChangeAlt;
}

bool is_recognized(uint16_t command) {
    return is_nav_item(command);
}

bool is_landing(uint16_t command) {
    return command == ModeAuto::kCmdNavLand || command == ModeAuto::kCmdNavReturnToLaunch;
}
} // namespace

ModeAuto::ModeAuto() {
    auto& params = apx::gcs::ParamManager::get_instance();
    m_wpnav_speed_ms = params.get_float_or("WPNAV_SPEED", 500.0f) / 100.0f;
    m_wpnav_radius_m = params.get_float_or("WPNAV_RADIUS", 200.0f) / 100.0f;
    m_wp_yaw_behavior = params.get_int_or("WP_YAW_BEHAVIOR", 2);
    m_rtl_alt_m = params.get_float_or("RTL_ALT", 1500.0f) / 100.0f;
    m_speed_up_ms = params.get_float_or("WPNAV_SPEED_UP", 250.0f) / 100.0f;
    m_speed_dn_ms = params.get_float_or("WPNAV_SPEED_DN", 150.0f) / 100.0f;
    m_land_speed_ms = params.get_float_or("LAND_SPEED", 50.0f) / 100.0f;
    m_land_speed_high_ms = params.get_float_or("LAND_SPEED_HIGH", 0.0f) / 100.0f;
    m_land_alt_low_m = params.get_float_or("LAND_ALT_LOW", 1000.0f) / 100.0f;
    m_max_reposition_speed_ms = params.get_float_or("LAND_REPOS_SPEED", 150.0f) / 100.0f;
    m_max_yaw_rate_rads = params.get_float_or("ATC_RATE_Y_MAX", 45.0f) * kDegToRad;

    m_cascade.set_max_vertical_speed(m_speed_up_ms, m_speed_dn_ms);
    // The trajectory only plans to use WPNAV_ACCEL; the position controller
    // gets headroom above that so it can still correct tracking errors, but
    // not the full lean-angle limit (demanding ANGLE_MAX while braking into
    // a corner overshot it by ~2deg in SITL).
    const float wpnav_accel_ms2 = params.get_float_or("WPNAV_ACCEL", 250.0f) / 100.0f;
    m_cascade.set_max_horizontal_accel(std::min(kGravityMs2 * std::tan(m_cascade.get_max_tilt_rad()),
                                                kAccelHeadroom * wpnav_accel_ms2));
}

void ModeAuto::on_enter(const VehicleState& state) {
    m_current_wp_index = 0;
    m_mission_complete = m_mission.empty();
    m_hold_pos_ned[0] = state.pos_ned[0];
    m_hold_pos_ned[1] = state.pos_ned[1];
    m_hold_pos_ned[2] = state.pos_ned[2];
    m_hold_yaw = state.yaw;
    m_cascade.reset();
    m_land_detector.reset();
    m_path_tracker.reset();
    m_land_stage = LandStage::Climb;
    m_leg_start_ned[0] = state.pos_ned[0];
    m_leg_start_ned[1] = state.pos_ned[1];
    m_leg_start_ned[2] = state.pos_ned[2];
    m_leg_initialized = false;
    m_condition_active = false;

    auto& params = apx::gcs::ParamManager::get_instance();
    m_wp_yaw_behavior = params.get_int_or("WP_YAW_BEHAVIOR", 2);
    float auto_yaw_head_deg = params.get_float_or("AUTO_YAW_HEAD", -1.0f);
    if (m_wp_yaw_behavior == 0 || m_wp_yaw_behavior == 4) {
        m_yaw_locked = true;
        if (auto_yaw_head_deg >= 0.0f && auto_yaw_head_deg <= 360.0f) {
            m_locked_yaw = wrap_pi(auto_yaw_head_deg * kDegToRad);
        } else {
            m_locked_yaw = state.yaw;
        }
        m_hold_yaw = m_locked_yaw;
        m_path_tracker.set_held_yaw(m_locked_yaw);
    } else {
        m_yaw_locked = false;
        m_locked_yaw = state.yaw;
    }
    m_cond_current_yaw = m_cond_target_yaw = m_last_cmd_yaw = m_yaw_locked ? m_locked_yaw : state.yaw;
    m_cond_yaw_dir = 0;

    if (m_mission.empty()) {
        std::printf("[APX Mode] Entering AUTO (no mission loaded - holding position)\n");
    } else {
        std::printf("[APX Mode] Entering AUTO (%zu mission items, WPNAV_SPEED %.1f m/s, yaw_locked=%d)\n",
                   m_mission.size(), (double)m_wpnav_speed_ms, m_yaw_locked ? 1 : 0);
    }
}

void ModeAuto::on_exit() {
    m_cascade.reset();
    m_path_tracker.reset();
    m_leg_initialized = false;
    m_condition_active = false;
}

size_t ModeAuto::next_nav_index(size_t from) const {
    for (size_t i = from + 1; i < m_mission.size(); ++i) {
        if (is_recognized(m_mission[i].command)) {
            return i;
        }
    }
    return m_mission.size();
}

float ModeAuto::cruise_speed_ms() const {
    return (m_requested_speed_ms > 0.0f) ? std::min(m_requested_speed_ms, kMaxSpeedMs) : m_wpnav_speed_ms;
}

void ModeAuto::apply_speed_request(float speed_ms) {
    if (speed_ms > 0.0f) {
        m_requested_speed_ms = std::min(speed_ms, kMaxSpeedMs);
    } else if (speed_ms == kSpeedResetValue) {
        m_requested_speed_ms = -1.0f;
    } else {
        return; // -1 (or other non-positive): no change, per MAVLink
    }
    const float cruise = cruise_speed_ms();
    m_cascade.set_max_horizontal_speed(std::max(cruise, m_wpnav_speed_ms));
    std::printf("[APX Mode] AUTO: speed set to %.1f m/s\n", (double)cruise);
}

bool ModeAuto::set_speed_xy(float speed_ms) {
    apply_speed_request(speed_ms);
    if (m_leg_initialized) {
        m_path_tracker.set_cruise_speed(cruise_speed_ms());
    }
    return true;
}

float ModeAuto::descent_speed_ms(float alt_m) const {
    if (alt_m > m_land_alt_low_m) {
        return (m_land_speed_high_ms > 0.0f) ? m_land_speed_high_ms : m_speed_dn_ms;
    }
    return m_land_speed_ms;
}

void ModeAuto::setup_current_leg(const VehicleState& state) {
    if (m_mission_complete || m_current_wp_index >= m_mission.size()) {
        return;
    }
    const auto& curr = m_mission[m_current_wp_index];
    PathSegment seg;
    for (int i = 0; i < 3; ++i) {
        seg.start[i] = m_leg_start_ned[i];
        seg.dest[i] = curr.pos_ned[i];
    }
    seg.accept_radius_m = (curr.accept_radius_m > 0.0f) ? curr.accept_radius_m : m_wpnav_radius_m;
    seg.cruise_speed_ms = cruise_speed_ms();

    // WP_YAW_BEHAVIOR: 0 never change / constant heading, 1 face next waypoint,
    // 2 face next waypoint except RTL, 3 face along GPS course, 4 fixed / condition locked.
    const bool lock_heading = m_yaw_locked || m_wp_yaw_behavior == 0 || m_wp_yaw_behavior == 4;
    seg.face_along_track = !lock_heading &&
                           !(m_wp_yaw_behavior == 2 && curr.command == kCmdNavReturnToLaunch);
    if (lock_heading) {
        m_path_tracker.set_held_yaw(m_locked_yaw);
    }
    seg.has_next = false;

    if (curr.command == kCmdNavTakeoff) {
        // Climb vertically at current location for takeoff.
        seg.dest[0] = seg.start[0];
        seg.dest[1] = seg.start[1];
        seg.face_along_track = false;
        if (lock_heading) {
            m_path_tracker.set_held_yaw(m_locked_yaw);
        }
    } else if (is_landing(curr.command)) {
        if (lock_heading) {
            m_hold_yaw = m_locked_yaw;
            m_path_tracker.set_held_yaw(m_locked_yaw);
        }
        if (curr.command == kCmdNavReturnToLaunch) {
            // RTL item: fly home (state.home_ned); its own altitude field is
            // usually 0, so the return altitude comes from RTL_ALT instead.
            const float rtl_z = std::min(seg.start[2], -m_rtl_alt_m);
            if (m_land_stage == LandStage::Climb && seg.start[2] > rtl_z + kRtlClimbMarginM) {
                seg.dest[0] = seg.start[0];
                seg.dest[1] = seg.start[1];
                seg.dest[2] = rtl_z;
            } else {
                m_land_stage = LandStage::Approach;
                seg.dest[0] = state.home_ned[0];
                seg.dest[1] = state.home_ned[1];
                seg.dest[2] = rtl_z;
            }
        } else {
            // Fly level to the landing point first; descend only once there.
            m_land_stage = LandStage::Approach;
            seg.dest[2] = seg.start[2];
        }
    } else {
        // Fly through this waypoint (corner arc) only when the next
        // navigable item is another plain waypoint; stop before takeoff/
        // land/RTL items.
        const size_t next = next_nav_index(m_current_wp_index);
        if (next < m_mission.size() && m_mission[next].command == kCmdNavWaypoint) {
            seg.has_next = true;
            for (int i = 0; i < 3; ++i) {
                seg.next[i] = m_mission[next].pos_ned[i];
            }
        }
    }

    m_path_tracker.set_segment(seg, state);
    if (lock_heading) {
        m_path_tracker.set_held_yaw(m_locked_yaw);
    }
    m_leg_initialized = true;
}

void ModeAuto::advance_to_next_item(const VehicleState& state) {
    if (m_current_wp_index < m_mission.size() && is_nav_item(m_mission[m_current_wp_index].command)) {
        const float* dest = m_path_tracker.segment().dest;
        m_leg_start_ned[0] = dest[0];
        m_leg_start_ned[1] = dest[1];
        m_leg_start_ned[2] = dest[2];
    } else {
        m_leg_start_ned[0] = state.pos_ned[0];
        m_leg_start_ned[1] = state.pos_ned[1];
        m_leg_start_ned[2] = state.pos_ned[2];
    }
    m_current_wp_index++;
    m_land_detector.reset();
    m_land_stage = LandStage::Climb;
    m_condition_active = false;
    if (m_current_wp_index >= m_mission.size()) {
        m_mission_complete = true;
        m_hold_pos_ned[0] = state.pos_ned[0];
        m_hold_pos_ned[1] = state.pos_ned[1];
        m_hold_pos_ned[2] = state.pos_ned[2];
        m_hold_yaw = m_yaw_locked ? m_locked_yaw : state.yaw;
        std::printf("[APX Mode] AUTO: mission complete - holding position (yaw=%.1f deg)\n",
                   (double)(m_hold_yaw * 57.2958f));
    } else {
        std::printf("[APX Mode] AUTO: reached item %zu, proceeding to item %zu\n",
                   m_current_wp_index - 1, m_current_wp_index);
        m_leg_initialized = false;
    }
}

void ModeAuto::start_condition_yaw(const Waypoint& item, float base_yaw) {
    const float speed_degs = (item.param2 > 0.0f) ? item.param2 : 30.0f;
    m_cond_yaw_speed_rads = speed_degs * kDegToRad;
    const float angle_deg = item.param1;
    const bool is_relative = (item.param4 > 0.5f);
    int dir = (item.param3 < -0.1f) ? -1 : ((item.param3 > 0.1f) ? 1 : 0);
    if (is_relative) {
        m_cond_target_yaw = wrap_pi(base_yaw + ((dir == -1) ? -angle_deg : angle_deg) * kDegToRad);
    } else {
        m_cond_target_yaw = wrap_pi(angle_deg * kDegToRad);
    }
    if (dir == 0) {
        dir = (wrap_pi(m_cond_target_yaw - base_yaw) >= 0.0f) ? 1 : -1;
    }
    m_cond_yaw_dir = dir;
    // Continue from the heading being commanded now (no step).
    m_cond_current_yaw = base_yaw;
    m_yaw_locked = true;
    m_locked_yaw = m_cond_target_yaw;
    m_hold_yaw = m_cond_target_yaw;
    m_path_tracker.set_held_yaw(m_cond_target_yaw);
}

void ModeAuto::slew_condition_yaw(float dt) {
    // Rate limit with a deceleration profile so the heading eases into the
    // target: omega = min(speed, sqrt(2*alpha*err), 2*err), alpha 1.5 rad/s^2.
    const float diff = wrap_pi(m_cond_target_yaw - m_cond_current_yaw);
    const float abs_diff = std::fabs(diff);
    if (abs_diff < 1e-4f) {
        m_cond_current_yaw = m_cond_target_yaw;
        return;
    }
    constexpr float kYawDecel = 1.5f;
    const float omega_decel = std::sqrt(2.0f * kYawDecel * abs_diff);
    const float omega_linear = 2.0f * abs_diff;
    const float allowed_rate = std::min(m_cond_yaw_speed_rads, std::max(0.08f, std::min(omega_decel, omega_linear)));
    const float step = allowed_rate * dt;

    // Within ~20 deg always take the short way, so an overshoot can never
    // turn into a full extra revolution in the requested direction.
    if (abs_diff <= 0.35f || m_cond_yaw_dir == 0) {
        m_cond_current_yaw = (step >= abs_diff) ? m_cond_target_yaw
                                                : wrap_pi(m_cond_current_yaw + std::copysign(step, diff));
    } else if (m_cond_yaw_dir == 1) { // CW
        m_cond_current_yaw = (diff > 0.0f && step >= diff) ? m_cond_target_yaw
                                                           : wrap_pi(m_cond_current_yaw + step);
    } else { // CCW
        m_cond_current_yaw = (diff < 0.0f && -step <= diff) ? m_cond_target_yaw
                                                            : wrap_pi(m_cond_current_yaw - step);
    }
}

void ModeAuto::process_instant_items(const VehicleState& state) {
    // DO_* items run in zero time. CONDITION_YAW followed by a nav item is
    // not blocking either (apx semantics: it runs alongside the next
    // leg); holding position to turn in place stopped the vehicle dead at
    // every waypoint of e.g. an orbit mission with a heading per waypoint.
    while (!m_mission_complete && m_current_wp_index < m_mission.size()) {
        const auto& skipped = m_mission[m_current_wp_index];
        if (is_nav_item(skipped.command)) {
            break;
        }
        if (skipped.command == kCmdConditionYaw) {
            if (m_condition_active || next_nav_index(m_current_wp_index) >= m_mission.size()) {
                break; // nothing to fly meanwhile: turn in place (blocking, see update())
            }
            start_condition_yaw(skipped, m_last_cmd_yaw);
            std::printf("[APX Mode] AUTO: CONDITION_YAW -> %.1f deg at %.1f deg/s while flying on\n",
                        (double)(m_cond_target_yaw * 57.2958f), (double)(m_cond_yaw_speed_rads * 57.2958f));
        } else if (is_condition_item(skipped.command)) {
            break; // CONDITION_CHANGE_ALT stays blocking
        } else if (skipped.command == kCmdDoChangeSpeed) {
            int speed_type = static_cast<int>(skipped.param1);
            float speed = (skipped.param2 > 0.0f) ? skipped.param2 : skipped.accept_radius_m;
            if (speed_type == 2 && speed > 0.0f) {
                // Vertical climb rate
                m_speed_up_ms = speed;
                m_cascade.set_max_vertical_speed(m_speed_up_ms, m_speed_dn_ms);
                std::printf("[APX Mode] AUTO: climb speed set to %.1f m/s\n", (double)speed);
            } else if (speed_type == 3 && speed > 0.0f) {
                // Vertical descent rate
                m_speed_dn_ms = speed;
                m_cascade.set_max_vertical_speed(m_speed_up_ms, m_speed_dn_ms);
                std::printf("[APX Mode] AUTO: descent speed set to %.1f m/s\n", (double)speed);
            } else {
                apply_speed_request(speed);
            }
        } else if (skipped.command == kCmdDoChangeAltitude) {
            float new_alt_m = (skipped.param1 > 0.0f) ? skipped.param1 : -skipped.pos_ned[2];
            float target_z = -new_alt_m;
            std::printf("[APX Mode] AUTO: executing DO_CHANGE_ALTITUDE to %.1f m\n", (double)new_alt_m);
            m_leg_start_ned[2] = target_z;
            m_hold_pos_ned[2] = target_z;
        } else if (skipped.command == kCmdDoSprayer) {
            Sprayer::instance().run(skipped.param1 > 0.5f);
        } else if (skipped.command == kCmdDoSetServo) {
            int channel = static_cast<int>(skipped.param1);
            float raw_pwm = (skipped.param2 > 0.0f) ? skipped.param2 : skipped.accept_radius_m;
            uint16_t pwm = static_cast<uint16_t>(std::clamp(raw_pwm, 800.0f, 2200.0f));
            std::printf("[APX Mode] AUTO: executing DO_SET_SERVO ch=%d pwm=%u\n", channel, pwm);
            if (channel == 9 || channel == Sprayer::instance().pump_channel() + 1) {
                Sprayer::instance().set_pump_pwm_manual(pwm);
            } else if (channel == 10 || channel == Sprayer::instance().spinner_channel() + 1) {
                Sprayer::instance().set_spinner_pwm_manual(pwm);
            }
        } else {
            std::printf("[APX Mode] AUTO: skipping unsupported item %zu (command %u)\n",
                       m_current_wp_index, (unsigned)skipped.command);
        }
        m_current_wp_index++;
    }
}

ControlOutput ModeAuto::update(const VehicleState& state) {
    if (!state.armed) {
        m_cascade.reset();
        m_land_detector.reset();
        m_path_tracker.reset();
        m_land_stage = LandStage::Climb;
        m_leg_initialized = false;
        m_condition_active = false;
        m_hold_pos_ned[0] = state.pos_ned[0];
        m_hold_pos_ned[1] = state.pos_ned[1];
        m_hold_pos_ned[2] = state.pos_ned[2];
        m_leg_start_ned[0] = state.pos_ned[0];
        m_leg_start_ned[1] = state.pos_ned[1];
        m_leg_start_ned[2] = state.pos_ned[2];
        m_cond_current_yaw = m_last_cmd_yaw = m_yaw_locked ? m_locked_yaw : state.yaw;
        return ControlOutput{};
    }

    process_instant_items(state);
    if (!m_mission_complete && m_current_wp_index >= m_mission.size()) {
        m_mission_complete = true;
        m_hold_pos_ned[0] = state.pos_ned[0];
        m_hold_pos_ned[1] = state.pos_ned[1];
        m_hold_pos_ned[2] = state.pos_ned[2];
        m_hold_yaw = m_yaw_locked ? m_locked_yaw : state.yaw;
    }

    const float dt_yaw = std::clamp(state.dt_s, 0.001f, 0.05f);
    if (m_mission_complete) {
        float hold_yaw = m_hold_yaw;
        if (m_yaw_locked) {
            slew_condition_yaw(dt_yaw);
            hold_yaw = m_cond_current_yaw;
        }
        m_last_cmd_yaw = hold_yaw;
        return m_cascade.compute(m_hold_pos_ned, true, hold_yaw, state);
    }

    const Waypoint& item = m_mission[m_current_wp_index];

    // Handling Blocking Condition Commands:
    if (item.command == kCmdConditionYaw) {
        const float dt = std::clamp(state.dt_s, 0.001f, 0.05f);
        if (!m_condition_active) {
            // Only reached when no nav item follows (see process_instant_items):
            // turn in place at the current position.
            m_condition_active = true;
            m_hold_pos_ned[0] = state.pos_ned[0];
            m_hold_pos_ned[1] = state.pos_ned[1];
            m_hold_pos_ned[2] = state.pos_ned[2];
            start_condition_yaw(item, state.yaw);
            m_cond_current_yaw = m_last_cmd_yaw;
            std::printf("[APX Mode] AUTO: CONDITION_YAW start -> target=%.1f deg, speed=%.1f deg/s, dir=%d\n",
                        (double)(m_cond_target_yaw * 57.2958f), (double)(m_cond_yaw_speed_rads * 57.2958f),
                        m_cond_yaw_dir);
        }
        slew_condition_yaw(dt);

        constexpr float kYawToleranceRad = 0.052f; // ~3 deg (tight, exact heading stop)
        float actual_err = std::fabs(wrap_pi(state.yaw - m_cond_target_yaw));
        float slew_err = std::fabs(wrap_pi(m_cond_current_yaw - m_cond_target_yaw));
        bool gyro_quiet = std::fabs(state.gyro_body[2]) < 0.15f; // vehicle rotation settled

        if (actual_err <= kYawToleranceRad && slew_err <= 0.01f && gyro_quiet) {
            std::printf("[APX Mode] AUTO: CONDITION_YAW complete -> locked at %.1f deg\n",
                        (double)(m_cond_target_yaw * 57.2958f));
            m_condition_active = false;
            m_yaw_locked = true;
            m_locked_yaw = m_cond_target_yaw;
            m_hold_yaw = m_cond_target_yaw;
            m_path_tracker.set_held_yaw(m_cond_target_yaw);

            advance_to_next_item(state);
            process_instant_items(state);
            if (!m_mission_complete && m_current_wp_index < m_mission.size() &&
                is_nav_item(m_mission[m_current_wp_index].command)) {
                setup_current_leg(state);
            }
        }

        m_last_cmd_yaw = m_cond_current_yaw;
        return m_cascade.compute(m_hold_pos_ned, /*has_yaw=*/true, m_cond_current_yaw, state);
    } else if (item.command == kCmdConditionChangeAlt) {
        if (!m_condition_active) {
            m_condition_active = true;
            m_hold_pos_ned[0] = state.pos_ned[0];
            m_hold_pos_ned[1] = state.pos_ned[1];
            float target_alt_m = (item.param2 > 0.0f) ? item.param2 : -item.pos_ned[2];
            m_hold_pos_ned[2] = -target_alt_m;
            std::printf("[APX Mode] AUTO: CONDITION_CHANGE_ALT start -> target alt=%.1f m\n", (double)target_alt_m);
        }

        float target_z = m_hold_pos_ned[2];
        float alt_err = target_z - state.pos_ned[2];
        float speed_req = (item.param1 > 0.0f) ? item.param1 : (alt_err < 0.0f ? m_speed_up_ms : m_speed_dn_ms);
        float forced_climb_rate = (alt_err < 0.0f) ? speed_req : -speed_req;

        if (std::fabs(state.pos_ned[2] - target_z) <= 0.5f) {
            std::printf("[APX Mode] AUTO: CONDITION_CHANGE_ALT complete -> reached alt=%.1f m\n", -(double)target_z);
            m_condition_active = false;
            advance_to_next_item(state);
            process_instant_items(state);
            if (!m_mission_complete && m_current_wp_index < m_mission.size() &&
                is_nav_item(m_mission[m_current_wp_index].command)) {
                setup_current_leg(state);
            }
        }

        float yaw_target = m_yaw_locked ? m_locked_yaw : m_hold_yaw;
        return m_cascade.compute(m_hold_pos_ned, /*has_yaw=*/true, yaw_target, state,
                                 /*use_forced_climb_rate=*/true, forced_climb_rate);
    }

    if (!m_leg_initialized) {
        setup_current_leg(state);
    }

    if (is_landing(item.command) && m_land_stage == LandStage::Descend) {
        if (m_land_detector.landed()) {
            ControlOutput out;
            out.request_disarm = true;
            return out;
        }

        const float dt = std::clamp(state.dt_s, 0.001f, 0.05f);
        // Pilot manual repositioning during auto landing descent
        if (state.rc_valid) {
            float v_fwd = 0.0f;
            float v_right = 0.0f;

            if (std::abs(state.rc_pitch) > kStickDeadband) {
                v_fwd = state.rc_pitch * m_max_reposition_speed_ms;
            }
            if (std::abs(state.rc_roll) > kStickDeadband) {
                v_right = state.rc_roll * m_max_reposition_speed_ms;
            }

            if (v_fwd != 0.0f || v_right != 0.0f) {
                float cy = std::cos(m_hold_yaw);
                float sy = std::sin(m_hold_yaw);
                float vel_n = v_fwd * cy - v_right * sy;
                float vel_e = v_fwd * sy + v_right * cy;

                m_hold_pos_ned[0] += vel_n * dt;
                m_hold_pos_ned[1] += vel_e * dt;
            }

            if (std::abs(state.rc_yaw) > kStickDeadband) {
                m_hold_yaw += state.rc_yaw * m_max_yaw_rate_rads * dt;
                m_hold_yaw = wrap_pi(m_hold_yaw);
                if (m_yaw_locked) {
                    m_locked_yaw = m_hold_yaw;
                }
            }
        }

        const float current_rel_alt = -state.pos_ned[2];
        float target[3] = {m_hold_pos_ned[0], m_hold_pos_ned[1], state.pos_ned[2]};
        ControlOutput out = m_cascade.compute(target, true, m_hold_yaw, state,
                                              /*use_forced_climb_rate=*/true,
                                              -descent_speed_ms(current_rel_alt));
        float climb_rate = -state.vel_ned[2];
        if (m_land_detector.update(current_rel_alt, climb_rate, out.thrust, state.dt_s)) {
            std::printf("[APX Mode] AUTO: LAND item touchdown confirmed - stopping motors\n");
            ControlOutput stopped;
            stopped.request_disarm = true;
            advance_to_next_item(state);
            return stopped;
        }
        return out;
    }

    PathTrackerOutput ref = m_path_tracker.update(state);
    float desired_yaw = ref.yaw;
    if (m_yaw_locked) {
        // Locked heading (CONDITION_YAW / WP_YAW_BEHAVIOR 0): slewed, so a
        // new CONDITION_YAW target turns the nose smoothly in flight.
        m_cond_target_yaw = m_locked_yaw;
        slew_condition_yaw(dt_yaw);
        desired_yaw = m_cond_current_yaw;
    } else {
        m_cond_current_yaw = ref.yaw;
    }
    m_last_cmd_yaw = desired_yaw;
    ControlOutput out = m_cascade.compute(ref.pos_ned, true, desired_yaw, state,
                                          false, 0.0f, ref.vel_ff_ne, ref.accel_ff_ne);

    if (ref.reached) {
        if (is_landing(item.command)) {
            const float* dest = m_path_tracker.segment().dest;
            m_leg_start_ned[0] = dest[0];
            m_leg_start_ned[1] = dest[1];
            m_leg_start_ned[2] = dest[2];
            if (m_land_stage == LandStage::Climb) {
                m_land_stage = LandStage::Approach;
                setup_current_leg(state);
            } else {
                m_land_stage = LandStage::Descend;
                m_hold_pos_ned[0] = dest[0];
                m_hold_pos_ned[1] = dest[1];
                m_hold_pos_ned[2] = dest[2];
                m_hold_yaw = m_yaw_locked ? m_locked_yaw : ref.yaw;
                std::printf("[APX Mode] AUTO: over landing point - descending (hold yaw=%.1f deg)\n",
                            (double)(m_hold_yaw * 57.2958f));
            }
        } else {
            advance_to_next_item(state);
            process_instant_items(state);
            if (!m_mission_complete && m_current_wp_index < m_mission.size() &&
                is_nav_item(m_mission[m_current_wp_index].command)) {
                setup_current_leg(state);
            }
        }
    }
    return out;
}

void ModeAuto::load_mission(const std::vector<Waypoint>& mission) {
    m_mission = mission;
    m_current_wp_index = 0;
    m_mission_complete = m_mission.empty();
    m_leg_initialized = false;
    m_condition_active = false;
}

} // namespace apx::modes
