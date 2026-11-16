// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mode_manager.hpp"
#include <cmath>
#include "mode_stabilize.hpp"
#include "mode_althold.hpp"
#include "mode_guided.hpp"
#include "mode_poshold.hpp"
#include "mode_loiter.hpp"
#include "mode_rtl.hpp"
#include "mode_land.hpp"
#include "mode_auto.hpp"
#include "mode_agri_spray.hpp"
#include "mode_agri_zigzag.hpp"
#include "mode_brake.hpp"
#include "sprayer.hpp"
#include "param_manager.hpp"
#include <cstdio>

namespace apx::modes {

std::unique_ptr<FlightMode> ModeManager::create_mode(FlightModeId id) {
    switch (id) {
        case FlightModeId::STABILIZE:   return std::make_unique<ModeStabilize>();
        case FlightModeId::ALT_HOLD:    return std::make_unique<ModeAltHold>();
        case FlightModeId::AUTO: {
            auto mode = std::make_unique<ModeAuto>();
            mode->load_mission(m_mission);
            return mode;
        }
        case FlightModeId::GUIDED:      return std::make_unique<ModeGuided>();
        case FlightModeId::LOITER:      return std::make_unique<ModeLoiter>();
        case FlightModeId::RTL:         return std::make_unique<ModeRTL>();
        case FlightModeId::LAND:        return std::make_unique<ModeLand>();
        case FlightModeId::POS_HOLD:    return std::make_unique<ModeLoiter>();
        case FlightModeId::BRAKE:       return std::make_unique<ModeBrake>();
        case FlightModeId::AGRI_SPRAY:  return std::make_unique<ModeAgriSpray>();
        case FlightModeId::AGRI_ZIGZAG: return std::make_unique<ModeAgriZigZag>();
    }
    return std::make_unique<ModeStabilize>(); // unreachable for a valid enum value; safe fallback
}

ModeManager::ModeManager() {
    Sprayer::instance().load_params();
    apx::gcs::ParamManager::get_instance().add_change_callback(
        [](const std::string& name, const apx::gcs::ParamValue&) {
            if (name.rfind("SPRAY_", 0) == 0) {
                Sprayer::instance().mark_params_dirty();
            }
        });
    // STABILIZE is the safe default - powering up in a manual-assist
    // mode rather than something that could move on arm.
    m_current = create_mode(FlightModeId::STABILIZE);
    VehicleState boot_state; // armed=false, everything else zeroed
    m_current->on_enter(boot_state);
}

bool ModeManager::request_mode(FlightModeId id, const VehicleState& state) {
    if (m_current && m_current->id() == id) {
        return true; // already there
    }
    // TODO: real GPS-fix/EKF-health gating for is_gps_required()/
    // is_position_required() modes - SITL doesn't model losing GPS fix
    // yet, so that check would always pass trivially either way.
    auto next = create_mode(id);
    if (m_current) {
        m_current->on_exit();
    }
    m_current = std::move(next);
    m_current->on_enter(state);
    return true;
}

bool ModeManager::request_takeoff(float target_rel_alt_m, const VehicleState& state) {
    if (m_current->id() != FlightModeId::GUIDED) {
        request_mode(FlightModeId::GUIDED, state);
    }
    m_current->request_takeoff(target_rel_alt_m);
    return true;
}

bool ModeManager::request_land(const VehicleState& state) {
    return request_mode(FlightModeId::LAND, state);
}

bool ModeManager::set_guided_target(const GuidedTarget& target) {
    if (m_current->id() != FlightModeId::GUIDED) {
        std::printf("[APX Mode] Ignoring guided target - not in GUIDED (current: %s)\n",
                   m_current->get_name().data());
        return false;
    }
    m_current->set_guided_target(target);
    return true;
}

ControlOutput ModeManager::update(const VehicleState& state) {
    float rel_alt = -state.pos_ned[2];
    float climb_rate = -state.vel_ned[2];
    bool crashed = m_crash_detector.update(state.roll, state.pitch, rel_alt, climb_rate, state.armed, state.dt_s);

    if (crashed) {
        // Emergency stop, independent of whatever the active mode would
        // otherwise output - see crash_detector.hpp. Printed once per
        // triggering (m_crash_logged), not every tick.
        if (!m_crash_logged) {
            std::printf("[APX SAFETY] Crash detected (roll=%.1fdeg pitch=%.1fdeg alt=%.2fm climb=%.2fm/s) "
                       "- emergency stop\n",
                       (double)(state.roll * 57.2958f), (double)(state.pitch * 57.2958f),
                       (double)rel_alt, (double)climb_rate);
        }
        m_crash_logged = true;
        ControlOutput out;
        out.request_disarm = true;
        out.crash_detected = true;
        return out;
    }
    m_crash_logged = false;

    ControlOutput out = m_current->update(state);
    Sprayer::instance().update(state);

    // Same thresholds as LandDetector (resting height incl. the hex's tall
    // landing gear, ~zero climb rate, thrust well below the ~0.7 hover).
    constexpr float kGroundAltM = 1.5f;
    constexpr float kGroundClimbMs = 0.1f;
    constexpr float kGroundThrust = 0.42f;
    const bool looks_landed = !state.armed ||
                              (rel_alt < kGroundAltM && std::fabs(climb_rate) < kGroundClimbMs &&
                               out.thrust < kGroundThrust);
    m_ground_time_s.store(looks_landed ? m_ground_time_s.load() + state.dt_s : 0.0f);
    return out;
}

bool ModeManager::disarm_allowed() const {
    constexpr float kGroundConfirmS = 0.5f;
    return m_current->id() == FlightModeId::STABILIZE || m_ground_time_s.load() >= kGroundConfirmS;
}

int ModeManager::auto_mission_current_seq() const {
    if (m_current->id() != FlightModeId::AUTO) return -1;
    // Only FlightMode itself is guaranteed here (create_mode() returns
    // that base pointer) - AUTO is always really a ModeAuto (see
    // create_mode() above), so this cast is safe.
    return static_cast<const ModeAuto*>(m_current.get())->current_seq();
}

uint32_t ModeManager::to_custom_mode_number(FlightModeId id) {
    switch (id) {
        case FlightModeId::STABILIZE:   return 0;
        case FlightModeId::ALT_HOLD:    return 2;
        case FlightModeId::AUTO:        return 3;
        case FlightModeId::GUIDED:      return 4;
        case FlightModeId::LOITER:      return 5;
        case FlightModeId::RTL:         return 6;
        case FlightModeId::LAND:        return 9;
        case FlightModeId::POS_HOLD:    return 16;
        case FlightModeId::BRAKE:       return 17; // Brake
        case FlightModeId::AGRI_SPRAY:  return 100; // APX Agri Spray
        case FlightModeId::AGRI_ZIGZAG: return 101; // APX Agri ZigZag
    }
    return 0;
}

std::optional<FlightModeId> ModeManager::from_custom_mode_number(uint32_t custom_mode) {
    switch (custom_mode) {
        case 0:   return FlightModeId::STABILIZE;
        case 2:   return FlightModeId::ALT_HOLD;
        case 3:   return FlightModeId::AUTO;
        case 4:   return FlightModeId::GUIDED;
        case 5:   return FlightModeId::LOITER;
        case 6:   return FlightModeId::RTL;
        case 9:   return FlightModeId::LAND;
        case 16:  return FlightModeId::POS_HOLD;
        case 17:  return FlightModeId::BRAKE;
        case 100: return FlightModeId::AGRI_SPRAY;
        case 101: return FlightModeId::AGRI_ZIGZAG;
        default:  return std::nullopt;
    }
}

} // namespace apx::modes
