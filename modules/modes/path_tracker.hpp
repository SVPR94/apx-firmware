// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// controller tracks a trajectory it can physically follow, instead of
// chasing a setpoint placed far ahead of the vehicle.
//
// Algorithms (kinematic trajectory reference generator with jerk-limited
// input shaping):
//  - along-track speed: square-root braking curve v = sqrt(v_exit^2 + 2*a*d)
//    with a linear zone near the target, accel shaped by a second sqrt
//    controller and jerk-limited (S-curve-like accel/decel ramps)
//  - corners (WPNAV_BLEND=1, default): a spiral-arc-spiral blend, the
//    track-transition curve of road/rail design. Curvature rises linearly
//    from 0 (the straight leg) to 1/R, holds, and falls back to 0, so at
//    the planned corner speed v_c the lateral accel v^2*kappa grows at a
//    bounded rate: no lean-angle step anywhere. v_c is the highest speed
//    for which lateral accel <= WPNAV_ACCEL, lateral jerk <= WPNAV_JERK,
//    the curve stays within the waypoint radius of the waypoint and fits on
//    both legs.
//  - corners (WPNAV_BLEND=0): circular arc tangent to both legs, cutting
//    the corner by the waypoint's acceptance radius; corner speed from the
//    lateral accel limit v_c = sqrt(a_lat * R), R = d_tangent / tan(angle/2).
//    Curvature, and so lateral accel, steps at the arc ends.
//  - leash: if the vehicle falls behind the reference, the reference's time
//    is slowed (scaled) so it never runs away from the vehicle

#pragma once

#include "vehicle_state.hpp"

namespace apx::modes {

struct PathSegment {
    float start[3]{0.0f, 0.0f, 0.0f};  // NED start of leg
    float dest[3]{0.0f, 0.0f, 0.0f};   // NED destination of leg
    bool has_next{false};              // false: stop at dest
    float next[3]{0.0f, 0.0f, 0.0f};   // NED waypoint after dest (corner planning)
    float cruise_speed_ms{5.0f};       // horizontal speed ceiling for this leg
    float accept_radius_m{2.0f};       // arrival radius (stop) / corner cut distance (fly-through)
    bool face_along_track{true};       // false: hold heading for the leg
};

struct PathTrackerLimits {
    float accel_ms2{2.5f};         // WPNAV_ACCEL
    float jerk_ms3{5.0f};          // WPNAV_JERK
    float speed_up_ms{2.5f};       // WPNAV_SPEED_UP
    float speed_down_ms{1.5f};     // WPNAV_SPEED_DN
    float yaw_rate_rads{0.785f};   // ATC_RATE_Y_MAX
    float yaw_accel_rads2{1.05f};  // ATC_ACCEL_Y_MAX (0 there = APX default 60 deg/s/s)
    bool blend_corners{true};      // WPNAV_BLEND
};

struct PathTrackerOutput {
    float pos_ned[3]{0.0f, 0.0f, 0.0f};
    float vel_ff_ne[2]{0.0f, 0.0f};
    float accel_ff_ne[2]{0.0f, 0.0f};
    float yaw{0.0f};
    bool reached{false};
};

class PathTracker {
public:
    PathTracker();
    explicit PathTracker(const PathTrackerLimits& limits) : m_lim(limits) {}

    void reset();
    void set_segment(const PathSegment& seg, const VehicleState& state);
    PathTrackerOutput update(const VehicleState& state);

    // Change the current leg's cruise ceiling in flight (DO_CHANGE_SPEED);
    // the reference ramps to it with the normal accel/jerk limits.
    void set_cruise_speed(float speed_ms);

    // Explicitly lock or set the held heading for segments where face_along_track is false.
    void set_held_yaw(float yaw) {
        m_held_yaw = yaw;
        m_has_held_yaw = true;
        m_yaw_rate = 0.0f;
    }
    float held_yaw() const { return m_held_yaw; }

    const PathSegment& segment() const { return m_seg; }
    float reference_speed() const { return m_v; }

    // 1D square-root controller: linear (gain p) near zero error, then
    // sqrt(2*limit*|error|) so the implied second derivative never exceeds
    // limit. Output is also bounded to |error|/dt so it cannot overshoot in
    // one step.
    static float sqrt_controller(float error, float p, float limit, float dt);

private:
    enum class Phase { Straight, Corner, Done };

    void plan_corner(float v_leg);
    bool plan_blend(float v_max, float theta, float d_max, float apex_max);
    void advance_blend(float dt);
    void update_leg_speed();
    void advance_straight(float dt);
    void advance_corner(float dt);
    void reference_point(float pos_ne[2], float vel_ne[2], float accel_ne[2], float& track_yaw) const;

    PathTrackerLimits m_lim;
    PathSegment m_seg;
    bool m_initialized{false};
    Phase m_phase{Phase::Done};

    // Leg geometry
    float m_u[2]{1.0f, 0.0f};  // leg unit vector (N,E)
    float m_leg_len{0.0f};
    float m_s_begin{0.0f};     // along-track position the reference entered this leg at
    float m_s_end{0.0f};       // end of the straight part (corner entry or dest)
    float m_leg_speed{0.0f};   // cruise for this leg after vertical-speed limiting

    // Corner arc at dest (tangent distance 0 => sharp pass-through)
    bool m_has_corner{false};
    float m_corner_speed{0.0f};
    float m_corner_speed_limit{0.0f}; // geometric limit, before the leg cruise cap
    float m_corner_tangent{0.0f};
    float m_corner_radius{0.0f};
    float m_corner_angle{0.0f};
    float m_corner_center[2]{0.0f, 0.0f};
    float m_corner_normal[2]{0.0f, 0.0f}; // unit vector from arc entry toward the arc centre

    // Spiral-arc-spiral blend, in a frame with x along this leg and y
    // toward the inside of the turn, starting at the blend entry point.
    struct Blend {
        static constexpr int kSamples = 64;
        bool active{false};
        float kappa{0.0f};     // curvature of the middle arc (1/m)
        float spiral_len{0.0f};
        float arc_len{0.0f};
        float length{0.0f};    // total curve length
        float entry[2]{0.0f, 0.0f};
        float x[kSamples + 1]{};
        float y[kSamples + 1]{};

        float heading(float s) const;    // tangent angle from the leg direction
        float curvature(float s) const;
        void position(float s, float& px, float& py) const;
        // Fills x/y (when store) and returns the end point and midpoint.
        void integrate(bool store, float end[2], float mid[2]);
        void shape(float v, float accel, float jerk, float theta);
    };
    Blend m_blend;
    float m_blend_s{0.0f};     // distance travelled along the blend

    // Reference state
    float m_s{0.0f};           // along-track position on the straight part
    float m_v{0.0f};           // reference speed
    float m_a{0.0f};           // along-track accel (jerk-limited)
    float m_phi{0.0f};         // angle travelled on the corner arc
    float m_time_scale{1.0f};  // leash time scaling, 0..1
    float m_accel_ff_ne[2]{0.0f, 0.0f}; // jerk-limited accel feedforward

    // Continuity into the next leg after a fly-through corner
    bool m_continue_valid{false};
    float m_continue_s{0.0f};
    float m_continue_v{0.0f};
    float m_continue_a{0.0f};
    float m_continue_start[3]{0.0f, 0.0f, 0.0f};
    float m_continue_dest[3]{0.0f, 0.0f, 0.0f};

    float m_held_yaw{0.0f};
    float m_yaw_rate{0.0f};    // shaped heading rate (rad/s)
    bool m_has_held_yaw{false};
};

} // namespace apx::modes
