// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "path_tracker.hpp"
#include "param_manager.hpp"
#include <algorithm>
#include <cmath>

namespace apx::modes {

namespace {
constexpr float kPi = 3.14159265f;
constexpr float kDegToRad = 0.017453293f;
// Below this turn angle consecutive legs are treated as one straight line
// (no arc, no slow-down).
constexpr float kStraightAngleRad = 8.0f * kDegToRad;
// Above this turn angle (near U-turn) the waypoint becomes a full stop:
// the arc radius would be ~0 and the corner speed ~0 anyway.
constexpr float kMaxCornerAngleRad = 150.0f * kDegToRad;
// Shortest corner cut worth flying as an arc; smaller means stop instead.
constexpr float kMinCornerTangentM = 0.1f;
// Horizontal legs shorter than this are flown as a pure vertical move.
constexpr float kMinLegLenM = 0.2f;
// Leash: the reference's time is slowed once the vehicle is further than
// this from it, and frozen at twice this distance.
constexpr float kLeashM = 2.0f;
// Fraction of WPNAV_ACCEL assumed available to stop on the leg after a
// corner (leaves room for jerk ramps and the next corner).
constexpr float kNextLegBrakeRatio = 0.5f;
// Arc speed floor so the reference can never stall mid-corner.
constexpr float kMinCornerSpeedMs = 0.2f;
// Stopping legs: reference counts as arrived this close to dest.
constexpr float kRefArrivedM = 0.05f;
constexpr float kArriveAltM = 1.0f;
// Auto-heading shaping: heading accel used when ATC_ACCEL_Y_MAX is 0, and
// the linear-zone gain of the heading sqrt controller (rad/s per rad).
constexpr float kDefaultYawAccelDeg = 60.0f;
constexpr float kYawShapeP = 2.0f;
// Blend corners: each may use at most this share of either leg, so two
// corners on one leg always leave a straight part between them.
constexpr float kBlendLegShare = 0.45f;
// Slowest corner speed worth planning a blend for; below it, stop.
constexpr float kMinBlendSpeedMs = 0.5f;
constexpr int kBlendSearchIters = 20;

float wrap_pi(float angle) {
    while (angle > kPi) {
        angle -= 2.0f * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0f * kPi;
    }
    return angle;
}

bool same_point(const float a[3], const float b[3]) {
    return std::fabs(a[0] - b[0]) < 0.01f && std::fabs(a[1] - b[1]) < 0.01f &&
           std::fabs(a[2] - b[2]) < 0.01f;
}
} // namespace

PathTracker::PathTracker() {
    auto& params = apx::gcs::ParamManager::get_instance();
    m_lim.accel_ms2 = params.get_float_or("WPNAV_ACCEL", 250.0f) / 100.0f;
    m_lim.jerk_ms3 = params.get_float_or("WPNAV_JERK", 5.0f);
    m_lim.speed_up_ms = params.get_float_or("WPNAV_SPEED_UP", 250.0f) / 100.0f;
    m_lim.speed_down_ms = params.get_float_or("WPNAV_SPEED_DN", 150.0f) / 100.0f;
    m_lim.yaw_rate_rads = params.get_float_or("ATC_RATE_Y_MAX", 45.0f) * kDegToRad;
    const float yaw_accel_deg = params.get_float_or("ATC_ACCEL_Y_MAX", 0.0f);
    m_lim.yaw_accel_rads2 = ((yaw_accel_deg > 0.0f) ? yaw_accel_deg : kDefaultYawAccelDeg) * kDegToRad;
    m_lim.blend_corners = params.get_int_or("WPNAV_BLEND", 1) != 0;
}

// --- Spiral-arc-spiral blend ------------------------------------------------
// Curvature profile over arc length s (L = 2*spiral_len + arc_len):
//   0 -> kappa linearly over the entry spiral, kappa on the arc, kappa -> 0
//   over the exit spiral. Heading is its integral (closed form); position
//   has no closed form (Fresnel integrals), so it is integrated with
//   Simpson's rule into a table and read back by Hermite interpolation.

void PathTracker::Blend::shape(float v, float accel, float jerk, float theta) {
    // At speed v: lateral accel v^2*kappa <= accel, and on the spirals
    // lateral jerk v^3*kappa/spiral_len <= jerk.
    float k = accel / (v * v);
    float ls = accel * v / jerk;
    float lc = 0.0f;
    if (k * ls >= theta) {
        // The spirals alone turn more than theta at full accel: two spirals
        // meeting at a lower peak curvature, still at the jerk limit.
        k = std::sqrt(theta * jerk / (v * v * v));
        ls = v * v * v * k / jerk;
    } else {
        lc = (theta - k * ls) / k;
    }
    kappa = k;
    spiral_len = ls;
    arc_len = lc;
    length = 2.0f * ls + lc;
}

float PathTracker::Blend::heading(float s) const {
    s = std::clamp(s, 0.0f, length);
    if (s < spiral_len) {
        return kappa * s * s / (2.0f * spiral_len);
    }
    if (s < spiral_len + arc_len) {
        return kappa * (0.5f * spiral_len + (s - spiral_len));
    }
    const float r = length - s;
    return kappa * (spiral_len + arc_len) - kappa * r * r / (2.0f * spiral_len);
}

float PathTracker::Blend::curvature(float s) const {
    s = std::clamp(s, 0.0f, length);
    if (s < spiral_len) {
        return kappa * s / spiral_len;
    }
    if (s < spiral_len + arc_len) {
        return kappa;
    }
    return kappa * (length - s) / spiral_len;
}

void PathTracker::Blend::integrate(bool store, float end[2], float mid[2]) {
    const float h = length / static_cast<float>(kSamples);
    float px = 0.0f;
    float py = 0.0f;
    float ha = 0.0f; // heading at the start of the step
    if (store) {
        x[0] = y[0] = 0.0f;
    }
    for (int k = 0; k < kSamples; ++k) {
        const float s0 = static_cast<float>(k) * h;
        const float hm = heading(s0 + 0.5f * h);
        const float hb = heading(s0 + h);
        px += h * (std::cos(ha) + 4.0f * std::cos(hm) + std::cos(hb)) / 6.0f;
        py += h * (std::sin(ha) + 4.0f * std::sin(hm) + std::sin(hb)) / 6.0f;
        ha = hb;
        if (store) {
            x[k + 1] = px;
            y[k + 1] = py;
        }
        if (k + 1 == kSamples / 2) {
            mid[0] = px;
            mid[1] = py;
        }
    }
    end[0] = px;
    end[1] = py;
}

void PathTracker::Blend::position(float s, float& px, float& py) const {
    s = std::clamp(s, 0.0f, length);
    const float h = length / static_cast<float>(kSamples);
    int k = static_cast<int>(s / h);
    k = std::clamp(k, 0, kSamples - 1);
    const float t = (s - static_cast<float>(k) * h) / h;
    // Cubic Hermite between samples k and k+1 with the exact unit tangents.
    const float h00 = (1.0f + 2.0f * t) * (1.0f - t) * (1.0f - t);
    const float h10 = t * (1.0f - t) * (1.0f - t);
    const float h01 = t * t * (3.0f - 2.0f * t);
    const float h11 = t * t * (t - 1.0f);
    const float a0 = heading(static_cast<float>(k) * h);
    const float a1 = heading(static_cast<float>(k + 1) * h);
    px = h00 * x[k] + h10 * h * std::cos(a0) + h01 * x[k + 1] + h11 * h * std::cos(a1);
    py = h00 * y[k] + h10 * h * std::sin(a0) + h01 * y[k + 1] + h11 * h * std::sin(a1);
}

bool PathTracker::plan_blend(float v_max, float theta, float d_max, float apex_max) {
    const float accel = std::max(m_lim.accel_ms2, 0.1f);
    const float jerk = std::max(m_lim.jerk_ms3, 0.1f);
    // Symmetric curve from (0,0) heading 0: the waypoint is at (d, 0) and the
    // exit point at (d, 0) + d * (cos theta, sin theta).
    const float bx = 1.0f + std::cos(theta);
    const float by = std::sin(theta);
    float d = 0.0f;
    auto fits = [&](float v) {
        m_blend.shape(v, accel, jerk, theta);
        float end[2];
        float mid[2];
        m_blend.integrate(false, end, mid);
        d = (end[0] * bx + end[1] * by) / (bx * bx + by * by);
        const float apex = std::hypot(mid[0] - d, mid[1]);
        return d <= d_max && apex <= apex_max;
    };

    float v = v_max;
    if (!fits(v)) {
        float lo = std::min(kMinBlendSpeedMs, v_max);
        if (!fits(lo)) {
            return false;
        }
        float hi = v_max;
        for (int i = 0; i < kBlendSearchIters; ++i) {
            const float m = 0.5f * (lo + hi);
            (fits(m) ? lo : hi) = m;
        }
        v = lo;
    }
    fits(v); // leave shape and d for the chosen speed
    float end[2];
    float mid[2];
    m_blend.integrate(true, end, mid);
    m_blend.active = true;
    m_blend.entry[0] = m_seg.dest[0] - d * m_u[0];
    m_blend.entry[1] = m_seg.dest[1] - d * m_u[1];
    m_corner_tangent = d;
    m_corner_speed_limit = v;
    return true;
}

void PathTracker::reset() {
    m_initialized = false;
    m_phase = Phase::Done;
    m_s = m_v = m_a = m_phi = 0.0f;
    m_time_scale = 1.0f;
    m_accel_ff_ne[0] = m_accel_ff_ne[1] = 0.0f;
    m_continue_valid = false;
    m_continue_a = 0.0f;
    m_blend.active = false;
    m_blend_s = 0.0f;
    m_has_held_yaw = false;
    m_yaw_rate = 0.0f;
}

float PathTracker::sqrt_controller(float error, float p, float limit, float dt) {
    float out;
    const float abs_err = std::fabs(error);
    const float sign = (error >= 0.0f) ? 1.0f : -1.0f;
    if (limit <= 0.0f) {
        out = p * error;
    } else if (p <= 0.0f) {
        out = sign * std::sqrt(2.0f * limit * abs_err);
    } else {
        const float linear_dist = limit / (p * p);
        if (abs_err > linear_dist) {
            out = sign * std::sqrt(2.0f * limit * (abs_err - 0.5f * linear_dist));
        } else {
            out = p * error;
        }
    }
    if (dt > 0.0f) {
        out = std::clamp(out, -abs_err / dt, abs_err / dt);
    }
    return out;
}

void PathTracker::set_segment(const PathSegment& seg, const VehicleState& state) {
    m_seg = seg;
    m_initialized = true;
    m_time_scale = 1.0f;
    m_phi = 0.0f;
    m_has_corner = false;
    m_corner_tangent = 0.0f;
    m_blend.active = false;
    m_blend_s = 0.0f;

    if (!m_has_held_yaw) {
        m_held_yaw = state.yaw;
        m_has_held_yaw = true;
    }

    const float dx = seg.dest[0] - seg.start[0];
    const float dy = seg.dest[1] - seg.start[1];
    m_leg_len = std::sqrt(dx * dx + dy * dy);
    if (m_leg_len < kMinLegLenM) {
        m_phase = Phase::Straight;
        m_s = m_s_begin = m_s_end = 0.0f;
        m_v = m_a = 0.0f;
        m_accel_ff_ne[0] = m_accel_ff_ne[1] = 0.0f;
        m_continue_valid = false;
        return;
    }
    m_u[0] = dx / m_leg_len;
    m_u[1] = dy / m_leg_len;

    update_leg_speed();

    const bool continuing = m_continue_valid && same_point(seg.start, m_continue_start) &&
                            same_point(seg.dest, m_continue_dest);
    if (continuing) {
        // Previous leg ended on a corner arc whose exit lies on this leg:
        // pick up exactly where the reference left off (position and speed
        // continuous, no setpoint jump at the waypoint).
        m_s_begin = std::min(m_continue_s, m_leg_len);
        m_v = std::min(m_continue_v, m_leg_speed);
    } else {
        // Fresh leg (mode entry, after a stop): start the reference at the
        // vehicle's projection on the line with its along-track speed.
        const float px = state.pos_ned[0] - seg.start[0];
        const float py = state.pos_ned[1] - seg.start[1];
        m_s_begin = std::clamp(px * m_u[0] + py * m_u[1], 0.0f, m_leg_len);
        const float v_along = state.vel_ned[0] * m_u[0] + state.vel_ned[1] * m_u[1];
        m_v = std::clamp(v_along, 0.0f, m_leg_speed);
        m_accel_ff_ne[0] = m_accel_ff_ne[1] = 0.0f;
    }
    m_continue_valid = false;
    m_s = m_s_begin;
    // Along-track accel carries over from a blended corner (always 0 after
    // an arc corner or a stop), so accel stays continuous at the handover.
    m_a = continuing ? m_continue_a : 0.0f;

    plan_corner(m_leg_speed);
    m_s_end = std::max(m_s_begin, m_leg_len - m_corner_tangent);
    m_phase = Phase::Straight;
}

void PathTracker::update_leg_speed() {
    // Limit horizontal speed so the climb/descent rate the leg implies stays
    // within WPNAV_SPEED_UP/DN - otherwise the vehicle arrives horizontally
    // with altitude still lagging.
    m_leg_speed = std::max(0.1f, m_seg.cruise_speed_ms);
    const float dz = m_seg.dest[2] - m_seg.start[2];
    if (std::fabs(dz) > 0.01f && m_leg_len > 0.0f) {
        const float v_z_max = (dz < 0.0f) ? m_lim.speed_up_ms : m_lim.speed_down_ms;
        m_leg_speed = std::max(0.1f, std::min(m_leg_speed, v_z_max * m_leg_len / std::fabs(dz)));
    }

    // Short-distance leg optimization (e.g. 2-5m row transitions in survey patterns):
    // Kinematically cap speed on short legs so vehicle does not jerk or swing
    // by accelerating up to full cruise speed when it cannot smoothly stop/turn on such a short span.
    if (m_leg_len < 12.0f && m_leg_len > 0.0f) {
        const float a_brake_plan = 0.75f * std::max(m_lim.accel_ms2, 0.5f);
        const float v_short_cap = std::max(1.0f, std::sqrt(a_brake_plan * m_leg_len));
        m_leg_speed = std::min(m_leg_speed, v_short_cap);
    }
}

void PathTracker::set_cruise_speed(float speed_ms) {
    if (!m_initialized || speed_ms <= 0.0f) {
        return;
    }
    m_seg.cruise_speed_ms = speed_ms;
    update_leg_speed();
    if (m_has_corner) {
        m_corner_speed = std::min(m_leg_speed, m_corner_speed_limit);
    }
}

void PathTracker::plan_corner(float v_leg) {
    if (!m_seg.has_next) {
        return;
    }
    const float l2n = m_seg.next[0] - m_seg.dest[0];
    const float l2e = m_seg.next[1] - m_seg.dest[1];
    const float len2 = std::sqrt(l2n * l2n + l2e * l2e);
    if (len2 < kMinLegLenM) {
        return; // next item is (nearly) vertical from here - stop first
    }
    const float u2[2] = {l2n / len2, l2e / len2};
    const float dot = std::clamp(m_u[0] * u2[0] + m_u[1] * u2[1], -1.0f, 1.0f);
    const float theta = std::acos(dot);
    if (theta > kMaxCornerAngleRad) {
        return;
    }

    // Never enter the next leg faster than could still stop on it.
    const float v_next_cap = std::sqrt(2.0f * kNextLegBrakeRatio * m_lim.accel_ms2 * len2);

    // Straight line or small turn (theta <= kStraightAngleRad):
    // Do not slow down! Keep full cruise speed.
    if (theta < kStraightAngleRad) {
        m_has_corner = true;
        m_corner_angle = 0.0f;
        m_corner_radius = 0.0f;
        m_corner_tangent = 0.0f;
        m_corner_speed_limit = v_next_cap;
        m_corner_speed = std::min(v_leg, v_next_cap);
        return;
    }

    // Turn direction: positive cross product (N x E) is a right turn.
    const float cross = m_u[0] * u2[1] - m_u[1] * u2[0];
    const float turn_sign = (cross >= 0.0f) ? 1.0f : -1.0f;
    m_corner_normal[0] = -turn_sign * m_u[1];
    m_corner_normal[1] = turn_sign * m_u[0];

    if (m_lim.blend_corners) {
        const float d_max = kBlendLegShare * std::min(m_leg_len - m_s_begin, len2);
        if (plan_blend(std::min(v_leg, v_next_cap), theta, d_max, m_seg.accept_radius_m)) {
            m_has_corner = true;
            m_corner_angle = theta;
            m_corner_radius = 1.0f / m_blend.kappa;
            m_corner_speed = std::max(kMinCornerSpeedMs, std::min(v_leg, m_corner_speed_limit));
            return;
        }
        // Blend could not fit inside the apex/distance limits. Fall through to circular
        // arc cornering below instead of aborting and stopping dead at the waypoint!
    }

    // Survey / General Cornering Optimization:
    // Cap corner tangent to min of accept_radius_m and 0.40 of leg length to guarantee at least 20%
    // straight section. This completely eliminates jerks on short-distance survey row transitions.
    const float tangent = std::min({m_seg.accept_radius_m, 0.40f * (m_leg_len - m_s_begin), 0.40f * len2});
    if (tangent < kMinCornerTangentM) {
        return;
    }
    const float radius = tangent / std::tan(0.5f * theta);
    const float v_lat = std::sqrt(m_lim.accel_ms2 * radius);

    // Proportional turn speed scaling: scales speed smoothly by cos(theta / 2)
    // so small turns maintain speed while sharper turns reduce speed in proportion to turn angle.
    const float v_prop = v_leg * std::cos(0.5f * theta);

    m_has_corner = true;
    m_corner_angle = theta;
    m_corner_radius = radius;
    m_corner_tangent = tangent;
    m_corner_speed_limit = std::min(v_lat, v_next_cap);
    m_corner_speed = std::max(kMinCornerSpeedMs, std::min({v_leg, v_prop, m_corner_speed_limit}));

    const float entry_n = m_seg.dest[0] - tangent * m_u[0];
    const float entry_e = m_seg.dest[1] - tangent * m_u[1];
    m_corner_center[0] = entry_n + radius * m_corner_normal[0];
    m_corner_center[1] = entry_e + radius * m_corner_normal[1];
}

void PathTracker::advance_straight(float dt) {
    const float accel = std::max(m_lim.accel_ms2, 0.1f);
    const float jerk = std::max(m_lim.jerk_ms3, 0.1f);
    const float dist = std::max(0.0f, m_s_end - m_s);
    const float v_exit = m_has_corner ? m_corner_speed : 0.0f;

    // For stopping legs (v_exit == 0), physical multicopters lag when pitching back to brake.
    // Use conservative braking acceleration a_plan = 0.75 * accel so deceleration begins
    // earlier and vehicle does not overshoot or blow through waypoints.
    const float a_plan = (v_exit < 0.1f) ? (0.75f * accel) : accel;

    // Speed allowed by the braking curve toward the end of the straight.
    // Gains follow from the jerk/accel limits so the shaped accel profile
    // stays stable (accel ramps take accel/jerk seconds).
    const float kp_v = 0.5f * jerk / a_plan;
    const float kp_a = jerk / a_plan;
    const float v_sqrt = sqrt_controller(dist, kp_v, a_plan, 0.0f);
    const float v_target = std::min(m_leg_speed, std::sqrt(v_exit * v_exit + v_sqrt * v_sqrt));
    // No speed cap tied to heading alignment: a multirotor's thrust vector
    // does not depend on yaw (the cascade rotates the accel demand into the
    // commanded heading), and capping speed until the nose caught up held
    // the vehicle at ~1/3 speed for seconds after every sharp corner -
    // apx does not do this either.

    float a_target = sqrt_controller(v_target - m_v, kp_a, jerk, dt);
    a_target = std::clamp(a_target, -accel, accel);
    m_a += std::clamp(a_target - m_a, -jerk * dt, jerk * dt);
    m_v += m_a * dt;

    // Hard ceiling: never faster than full-accel braking could still bring
    // down to v_exit by the end of the straight.
    const float v_hard = std::sqrt(v_exit * v_exit + 2.0f * a_plan * dist);
    if (m_v > v_hard) {
        m_v = v_hard;
    }
    // Cap at cruise only while speeding up; after a live speed reduction
    // the reference is above cruise and decelerates smoothly instead.
    if (m_v > m_leg_speed && m_a > 0.0f) {
        m_v = m_leg_speed;
        m_a = 0.0f;
    }
    if (m_v < 0.0f) {
        m_v = 0.0f;
        m_a = std::max(m_a, 0.0f);
    }

    m_s += m_v * dt;
    if (m_s >= m_s_end && m_blend.active) {
        // Into the blend with the distance overrun, speed and accel intact.
        m_blend_s = m_s - m_s_end;
        m_s = m_s_end;
        m_phase = Phase::Corner;
    } else if (m_s >= m_s_end) {
        m_s = m_s_end;
        if (m_has_corner) {
            m_phase = Phase::Corner;
            m_phi = 0.0f;
            m_a = 0.0f;
        } else {
            m_v = m_a = 0.0f;
            m_phase = Phase::Done;
        }
    } else if (!m_has_corner && dist < kRefArrivedM && m_v < 0.1f) {
        m_s = m_s_end;
        m_v = m_a = 0.0f;
        m_phase = Phase::Done;
    }
}

void PathTracker::advance_blend(float dt) {
    // Same jerk-limited along-track shaping as the straight, holding the
    // planned corner speed (the curve was sized for it).
    const float accel = std::max(m_lim.accel_ms2, 0.1f);
    const float jerk = std::max(m_lim.jerk_ms3, 0.1f);
    float a_target = sqrt_controller(m_corner_speed - m_v, jerk / accel, jerk, dt);
    a_target = std::clamp(a_target, -accel, accel);
    m_a += std::clamp(a_target - m_a, -jerk * dt, jerk * dt);
    m_v += m_a * dt;
    if (m_v > m_corner_speed && m_a > 0.0f) {
        m_v = m_corner_speed;
        m_a = 0.0f;
    }
    if (m_v < kMinCornerSpeedMs) {
        m_v = kMinCornerSpeedMs;
        m_a = std::max(m_a, 0.0f);
    }
    m_blend_s += m_v * dt;
    if (m_blend_s >= m_blend.length) {
        m_phase = Phase::Done;
        m_continue_valid = true;
        // The overrun past the curve end is dropped (this tick stops at the
        // exit point): carrying it into the next leg as well as that leg's
        // own first step would advance the reference twice in one tick.
        m_continue_s = m_corner_tangent;
        m_continue_v = m_v;
        m_continue_a = m_a;
        m_blend_s = m_blend.length;
        for (int i = 0; i < 3; ++i) {
            m_continue_start[i] = m_seg.dest[i];
            m_continue_dest[i] = m_seg.next[i];
        }
    }
}

void PathTracker::advance_corner(float dt) {
    if (m_blend.active) {
        advance_blend(dt);
        return;
    }
    bool finished = m_corner_radius < 1e-3f || m_corner_angle < 1e-3f;
    if (!finished) {
        const float dv = std::clamp(m_corner_speed - m_v, -m_lim.accel_ms2 * dt, m_lim.accel_ms2 * dt);
        m_v = std::max(m_v + dv, kMinCornerSpeedMs);
        m_phi += m_v * dt / m_corner_radius;
        if (m_phi >= m_corner_angle) {
            m_phi = m_corner_angle;
            finished = true;
        }
    }
    if (finished) {
        m_phase = Phase::Done;
        m_continue_valid = true;
        m_continue_s = m_corner_tangent;
        m_continue_v = m_v;
        m_continue_a = 0.0f;
        for (int i = 0; i < 3; ++i) {
            m_continue_start[i] = m_seg.dest[i];
            m_continue_dest[i] = m_seg.next[i];
        }
    }
}

void PathTracker::reference_point(float pos_ne[2], float vel_ne[2], float accel_ne[2],
                                  float& track_yaw) const {
    if (m_blend.active && (m_phase == Phase::Corner || m_phase == Phase::Done)) {
        float lx;
        float ly;
        m_blend.position(m_blend_s, lx, ly);
        const float psi = m_blend.heading(m_blend_s);
        const float kappa = m_blend.curvature(m_blend_s);
        const float* u = m_u;
        const float* n = m_corner_normal; // toward the inside of the turn
        const float c = std::cos(psi);
        const float s = std::sin(psi);
        const float t_n = u[0] * c + n[0] * s;
        const float t_e = u[1] * c + n[1] * s;
        const float in_n = -u[0] * s + n[0] * c;
        const float in_e = -u[1] * s + n[1] * c;
        pos_ne[0] = m_blend.entry[0] + lx * u[0] + ly * n[0];
        pos_ne[1] = m_blend.entry[1] + lx * u[1] + ly * n[1];
        vel_ne[0] = m_v * t_n;
        vel_ne[1] = m_v * t_e;
        const float a_lat = m_v * m_v * kappa;
        accel_ne[0] = m_a * t_n + a_lat * in_n;
        accel_ne[1] = m_a * t_e + a_lat * in_e;
        track_yaw = std::atan2(t_e, t_n);
        return;
    }
    const bool on_arc = (m_phase == Phase::Corner || (m_phase == Phase::Done && m_has_corner)) &&
                        m_corner_radius > 1e-3f;
    if (!on_arc) {
        pos_ne[0] = m_seg.start[0] + m_s * m_u[0];
        pos_ne[1] = m_seg.start[1] + m_s * m_u[1];
        vel_ne[0] = m_v * m_u[0];
        vel_ne[1] = m_v * m_u[1];
        accel_ne[0] = m_a * m_u[0];
        accel_ne[1] = m_a * m_u[1];
        track_yaw = std::atan2(m_u[1], m_u[0]);
        return;
    }
    // Arc parametrised by turned angle phi from the entry point:
    //   p(phi)   = C + R * (-n cos(phi) + u sin(phi))
    //   tangent  = u cos(phi) + n sin(phi)
    //   centripetal accel = v^2/R * (n cos(phi) - u sin(phi))
    const float c = std::cos(m_phi);
    const float s = std::sin(m_phi);
    const float* n = m_corner_normal;
    const float r = m_corner_radius;
    pos_ne[0] = m_corner_center[0] + r * (-n[0] * c + m_u[0] * s);
    pos_ne[1] = m_corner_center[1] + r * (-n[1] * c + m_u[1] * s);
    const float t_n = m_u[0] * c + n[0] * s;
    const float t_e = m_u[1] * c + n[1] * s;
    vel_ne[0] = m_v * t_n;
    vel_ne[1] = m_v * t_e;
    const float a_c = m_v * m_v / r;
    accel_ne[0] = a_c * (n[0] * c - m_u[0] * s);
    accel_ne[1] = a_c * (n[1] * c - m_u[1] * s);
    track_yaw = std::atan2(t_e, t_n);
}

PathTrackerOutput PathTracker::update(const VehicleState& state) {
    PathTrackerOutput out;
    out.pos_ned[0] = state.pos_ned[0];
    out.pos_ned[1] = state.pos_ned[1];
    out.pos_ned[2] = state.pos_ned[2];
    out.yaw = state.yaw;
    if (!m_initialized) {
        return out;
    }
    const float dt = std::clamp(state.dt_s, 0.001f, 0.05f);

    // Pure vertical move (takeoff, altitude change in place).
    if (m_leg_len < kMinLegLenM) {
        out.pos_ned[0] = m_seg.dest[0];
        out.pos_ned[1] = m_seg.dest[1];
        out.pos_ned[2] = m_seg.dest[2];
        out.yaw = m_held_yaw;
        out.reached = std::fabs(state.pos_ned[2] - m_seg.dest[2]) <= kArriveAltM;
        return out;
    }

    // Leash: slow the reference's clock while the vehicle lags behind it.
    float ref_ne[2], ref_vel[2], ref_acc[2], track_yaw;
    reference_point(ref_ne, ref_vel, ref_acc, track_yaw);
    const float en = ref_ne[0] - state.pos_ned[0];
    const float ee = ref_ne[1] - state.pos_ned[1];
    const float err = std::sqrt(en * en + ee * ee);
    const float leash = std::max(kLeashM, m_leg_speed * 0.8f);
    m_time_scale = (err <= leash) ? 1.0f : std::clamp(1.0f - (err - leash) / leash, 0.15f, 1.0f);
    const float dt_ref = dt * m_time_scale;

    if (m_phase == Phase::Straight) {
        advance_straight(dt_ref);
    } else if (m_phase == Phase::Corner) {
        advance_corner(dt_ref);
    }

    reference_point(ref_ne, ref_vel, ref_acc, track_yaw);
    const float scale_sq = m_time_scale * m_time_scale;
    m_accel_ff_ne[0] = ref_acc[0] * scale_sq;
    m_accel_ff_ne[1] = ref_acc[1] * scale_sq;

    out.pos_ned[0] = ref_ne[0];
    out.pos_ned[1] = ref_ne[1];
    if (m_phase == Phase::Straight && m_s_end - m_s_begin > 0.01f) {
        const float frac = std::clamp((m_s - m_s_begin) / (m_s_end - m_s_begin), 0.0f, 1.0f);
        out.pos_ned[2] = m_seg.start[2] + frac * (m_seg.dest[2] - m_seg.start[2]);
    } else {
        out.pos_ned[2] = m_seg.dest[2];
    }
    out.vel_ff_ne[0] = ref_vel[0] * m_time_scale;
    out.vel_ff_ne[1] = ref_vel[1] * m_time_scale;
    out.accel_ff_ne[0] = m_accel_ff_ne[0];
    out.accel_ff_ne[1] = m_accel_ff_ne[1];

    // Heading: rate- and accel-limited so the nose decelerates into the
    // new heading instead of stopping dead at full yaw rate, which the
    // vehicle then overshoots.
    if (m_seg.face_along_track) {
        const float yaw_accel = std::max(m_lim.yaw_accel_rads2, 0.1f);
        float rate_target = sqrt_controller(wrap_pi(track_yaw - m_held_yaw), kYawShapeP, yaw_accel, dt);
        rate_target = std::clamp(rate_target, -m_lim.yaw_rate_rads, m_lim.yaw_rate_rads);
        m_yaw_rate += std::clamp(rate_target - m_yaw_rate, -yaw_accel * dt, yaw_accel * dt);
        m_held_yaw = wrap_pi(m_held_yaw + m_yaw_rate * dt);
    } else {
        m_yaw_rate = 0.0f;
    }
    out.yaw = m_held_yaw;

    if (m_phase == Phase::Done) {
        if (m_has_corner) {
            out.reached = true;
        } else {
            const float dn = m_seg.dest[0] - state.pos_ned[0];
            const float de = m_seg.dest[1] - state.pos_ned[1];
            const float dz = std::fabs(m_seg.dest[2] - state.pos_ned[2]);
            const float speed_xy = std::sqrt(state.vel_ned[0] * state.vel_ned[0] +
                                             state.vel_ned[1] * state.vel_ned[1]);
            // Stopping waypoint arrival check:
            // Must be within distance tolerance AND horizontal speed must be controlled/stopped
            // so high-speed vehicles do not declare reached while blowing through the waypoint!
            const bool pos_reached = std::sqrt(dn * dn + de * de) <= m_seg.accept_radius_m && dz <= kArriveAltM;
            const bool speed_settled = speed_xy <= std::max(0.8f, 0.25f * m_leg_speed);
            out.reached = pos_reached && speed_settled;
        }
    }
    return out;
}

} // namespace apx::modes
