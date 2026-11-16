// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "command_handler.hpp"
#include "mavlink_handler.hpp"
#include "sprayer.hpp"
#include "fence.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>

namespace apx::gcs {

CommandHandler::CommandHandler(MavlinkHandler* mavlink)
    : m_mavlink(mavlink) {
}

void CommandHandler::handle_command(const mavlink_message_t& msg) {
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);
        process_command_long(cmd);
    } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        mavlink_command_int_t cmd;
        mavlink_msg_command_int_decode(&msg, &cmd);
        process_command_int(cmd);
    }
}

void CommandHandler::process_command_long(const mavlink_command_long_t& cmd) {
    uint8_t result = execute_command(cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4,
                                     static_cast<double>(cmd.param5), static_cast<double>(cmd.param6), cmd.param7);
    if (m_mavlink) {
        m_mavlink->send_command_ack(cmd.command, result);
    }
}

void CommandHandler::process_command_int(const mavlink_command_int_t& cmd) {
    // For command_int, param5/6 are lat/lon in 1e7 degrees
    double param5 = static_cast<double>(cmd.x) * 1e-7;
    double param6 = static_cast<double>(cmd.y) * 1e-7;
    float param7 = cmd.z;
    uint8_t result = execute_command(cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4, param5, param6, param7);
    if (m_mavlink) {
        m_mavlink->send_command_ack(cmd.command, result);
    }
}

namespace {
// MAV_CMD_DO_SPRAYER command ID.
constexpr uint16_t kMavCmdDoSprayer = 216;
} // namespace

uint8_t CommandHandler::execute_command(uint16_t command, float param1, float param2, float param3, float param4, double param5, double param6, float param7) {
    switch (command) {
        case kMavCmdDoSprayer:
            // param1: 1 = start spraying, 0 = stop.
            return apx::modes::Sprayer::instance().run(param1 > 0.5f) ? MAV_RESULT_ACCEPTED
                                                                      : MAV_RESULT_DENIED;
        case MAV_CMD_NAV_TAKEOFF: {
            // param7 = target relative altitude (m). Refuse if not armed,
            // rather than silently ACKing and doing nothing.
            if (!m_mavlink || !m_mavlink->get_armed() || !m_mavlink->get_mode_manager()) {
                return MAV_RESULT_DENIED;
            }
            auto state = m_mavlink->get_last_known_vehicle_state();
            m_mavlink->get_mode_manager()->request_takeoff(param7 > 0.0f ? param7 : 2.0f, state);
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_NAV_LAND: {
            // Switch to LAND mode (see ModeManager::request_land()), the mode
            // itself handles the actual descent + touchdown detection.
            if (!m_mavlink || !m_mavlink->get_mode_manager()) {
                return MAV_RESULT_DENIED;
            }
            auto state = m_mavlink->get_last_known_vehicle_state();
            m_mavlink->get_mode_manager()->request_land(state);
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_DO_REPOSITION: {
            // Guided "go to this location" from a GCS map click -
            // requires GUIDED already active; ModeManager
            // enforces the same (set_guided_target() no-ops with a log
            // if not GUIDED, rather than silently switching mode for you).
            if (!m_mavlink || !m_mavlink->get_mode_manager()) {
                return MAV_RESULT_DENIED;
            }
            apx::modes::GuidedTarget target;
            // param5/param6 = lat/lon (already converted to degrees by
            // process_command_int, or passed as degrees directly for
            // COMMAND_LONG per the MAV_CMD_DO_REPOSITION spec), param7 = alt.
            constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
            double dlat = (static_cast<double>(param5) - m_mavlink->home_lat()) * kDegToRad;
            double dlon = (static_cast<double>(param6) - m_mavlink->home_lon()) * kDegToRad;
            constexpr double kEarthRadiusM = 6378137.0;
            target.pos_ned[0] = static_cast<float>(dlat * kEarthRadiusM);
            target.pos_ned[1] = static_cast<float>(dlon * kEarthRadiusM * std::cos(m_mavlink->home_lat() * kDegToRad));
            target.pos_ned[2] = -(param7 - m_mavlink->home_alt_msl()); // treat as MSL, matches DO_REPOSITION's Latitude/Longitude/Altitude fields
            bool yaw_given = !std::isnan(param4);
            target.has_yaw = yaw_given;
            target.yaw = yaw_given ? param4 : 0.0f;
            if (!m_mavlink->get_mode_manager()->set_guided_target(target)) {
                return MAV_RESULT_DENIED;
            }
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_DO_SET_MODE: {
            // param1 = base_mode bitmask, param2 = custom_mode number.
            if (!m_mavlink || !m_mavlink->get_mode_manager()) {
                return MAV_RESULT_DENIED;
            }
            auto mode_id = apx::modes::ModeManager::from_custom_mode_number(static_cast<uint32_t>(param2));
            if (!mode_id) {
                return MAV_RESULT_UNSUPPORTED;
            }
            auto state = m_mavlink->get_last_known_vehicle_state();
            m_mavlink->get_mode_manager()->request_mode(*mode_id, state);
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_DO_CHANGE_SPEED: {
            // param1 speed type (0 airspeed, 1 ground speed, 2 climb,
            // 3 descent), param2 speed m/s (-1 no change, -2 default).
            // Only horizontal speed is supported; a multicopter treats
            // airspeed and ground speed alike.
            if (!m_mavlink || !m_mavlink->get_mode_manager()) {
                return MAV_RESULT_DENIED;
            }
            if (param1 > 1.5f) {
                return MAV_RESULT_UNSUPPORTED;
            }
            if (!m_mavlink->get_mode_manager()->set_speed_xy(param2)) {
                return MAV_RESULT_TEMPORARILY_REJECTED;
            }
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_MISSION_START: {
            // GCS's "Start Mission" action sends this -
            // requires armed + AUTO already active and just resumes
            // mission execution from wherever it is.
            // Switching to AUTO here if not already active (rather than failing)
            // matches what a GCS pressing this button expects.
            if (!m_mavlink || !m_mavlink->get_armed() || !m_mavlink->get_mode_manager()) {
                return MAV_RESULT_FAILED;
            }
            auto state = m_mavlink->get_last_known_vehicle_state();
            m_mavlink->get_mode_manager()->request_mode(apx::modes::FlightModeId::AUTO, state);
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            // TODO: run real arming checks (ARMING_CHECK param, sensor health,
            // GPS fix, etc.) before accepting an arm request.
            if (!m_mavlink) {
                return MAV_RESULT_FAILED;
            }
            bool want_armed = param1 > 0.5f;
            if (want_armed && m_mavlink->arming_block_reason() != nullptr) {
                m_mavlink->send_statustext(MAV_SEVERITY_CRITICAL, m_mavlink->arming_block_reason());
                return MAV_RESULT_DENIED;
            }
            // FlightMode::can_arm() (e.g. a mode that requires GPS/position
            // and refuses to arm without it) existed but was never actually
            // checked by anything - this is the same gate the RC stick-arm
            // gesture uses (see StickArmDetector's call site in
            // apx_main.cpp), so both arming paths respect it consistently.
            if (want_armed && m_mavlink->get_mode_manager() &&
                !m_mavlink->get_mode_manager()->can_arm()) {
                return MAV_RESULT_FAILED;
            }
            // Pre-arm geofence validation check
            constexpr float kForceMagic = 21196.0f;
            const bool forced = std::fabs(param2 - kForceMagic) < 0.5f;
            if (want_armed && !forced) {
                std::string fence_msg;
                auto last_state = m_mavlink->get_last_known_vehicle_state();
                if (!apx::safety::FenceManager::instance().pre_arm_check(-last_state.pos_ned[2],
                                                                         last_state.pos_ned[0],
                                                                         last_state.pos_ned[1],
                                                                         fence_msg)) {
                    m_mavlink->send_statustext(MAV_SEVERITY_CRITICAL, fence_msg.c_str());
                    return MAV_RESULT_DENIED;
                }
            }
            if (!want_armed && !forced && m_mavlink->get_armed() && m_mavlink->get_mode_manager() &&
                !m_mavlink->get_mode_manager()->disarm_allowed()) {
                m_mavlink->send_statustext(MAV_SEVERITY_WARNING, "Disarm refused: not landed (force to override)");
                return MAV_RESULT_DENIED;
            }
            m_mavlink->set_armed(want_armed);
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_DO_SET_SERVO: {
            // param1: Servo instance number (1-16)
            // param2: PWM value in microseconds (typically 800-2200, 1100=off, 1900=full on)
            int channel = static_cast<int>(param1);
            if (channel < 1 || channel > 16) {
                return MAV_RESULT_DENIED;
            }
            uint16_t pwm = static_cast<uint16_t>(std::clamp(param2, 800.0f, 2200.0f));
            if (m_mavlink) {
                m_mavlink->set_aux_servo_pwm(static_cast<uint8_t>(channel), pwm);
            }
            if (channel == 9 || channel == apx::modes::Sprayer::instance().pump_channel() + 1) {
                apx::modes::Sprayer::instance().set_pump_pwm_manual(pwm);
            } else if (channel == 10 || channel == apx::modes::Sprayer::instance().spinner_channel() + 1) {
                apx::modes::Sprayer::instance().set_spinner_pwm_manual(pwm);
            }
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_DO_SET_HOME: {
            // param1 = 1: home = the vehicle's current position.
            // param1 = 0: home = param5/6/7 (lat, lon in degrees; altitude MSL).
            // A non-finite altitude keeps the current home altitude: a ground
            // station placing home on a map does not know the ground height.
            // RTL and mission RTL items read the new home on their next tick,
            // also in flight. A home set here is kept across arming/landing.
            double lat = param5;
            double lon = param6;
            float alt = param7;
            if (param1 > 0.5f) {
                const auto s = m_mavlink->get_vehicle_telemetry_state();
                if (s.lat == 0.0 && s.lon == 0.0) {
                    m_mavlink->send_statustext(MAV_SEVERITY_WARNING, "Set home failed: no position");
                    return MAV_RESULT_FAILED;
                }
                lat = s.lat;
                lon = s.lon;
                alt = s.alt_msl;
            }
            const bool valid = std::isfinite(lat) && std::isfinite(lon) && std::fabs(lat) <= 90.0 &&
                               std::fabs(lon) <= 180.0 && !(lat == 0.0 && lon == 0.0);
            if (!valid) {
                m_mavlink->send_statustext(MAV_SEVERITY_WARNING, "Set home denied: invalid position");
                return MAV_RESULT_DENIED;
            }
            if (!std::isfinite(alt)) {
                alt = m_mavlink->home_alt_msl();
            }
            m_mavlink->set_home_position(lat, lon, alt, true);
            char text[50];
            std::snprintf(text, sizeof(text), "Home set: %.6f, %.6f", lat, lon);
            m_mavlink->send_statustext(MAV_SEVERITY_INFO, text);
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_DO_MOTOR_TEST: {
            // param1: Motor instance number (1 to 8)
            // param2: Throttle type (0 = percent, 1 = PWM)
            // param3: Throttle value
            // param4: Timeout in seconds (0 = stop immediately)
            // param5: Motor count
            // param6: Motor test order
            if (m_mavlink->get_armed()) {
                m_mavlink->send_statustext(MAV_SEVERITY_WARNING, "Motor test denied: disarm vehicle first");
                return MAV_RESULT_TEMPORARILY_REJECTED;
            }
            uint8_t motor_idx = static_cast<uint8_t>(param1);
            uint8_t throttle_type = static_cast<uint8_t>(param2);
            float throttle_val = param3;
            float timeout_sec = param4;
            if (motor_idx < 1 || motor_idx > 8) {
                return MAV_RESULT_DENIED;
            }
            if (!m_mavlink->start_motor_test(motor_idx, throttle_type, throttle_val, timeout_sec)) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_GET_HOME_POSITION: {
            m_mavlink->send_home_position();
            return MAV_RESULT_ACCEPTED;
        }
        case MAV_CMD_DO_PARACHUTE:
            // TODO: trigger parachute
            return MAV_RESULT_ACCEPTED;
        case MAV_CMD_PREFLIGHT_CALIBRATION:
            // TODO: start calibration
            return MAV_RESULT_ACCEPTED;
        default:
            return MAV_RESULT_UNSUPPORTED;
    }
}

} // namespace apx::gcs
