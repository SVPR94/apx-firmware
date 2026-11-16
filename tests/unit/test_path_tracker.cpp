// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gtest/gtest.h>
#include "path_tracker.hpp"

#include <cmath>
#include <vector>

using apx::modes::PathSegment;
using apx::modes::PathTracker;
using apx::modes::PathTrackerLimits;
using apx::modes::PathTrackerOutput;
using apx::modes::VehicleState;

namespace {

constexpr float kDt = 0.01f;

struct Pt {
    float n, e;
};

float dist_to_segment(Pt p, Pt a, Pt b) {
    float dx = b.n - a.n, dy = b.e - a.e;
    float len_sq = dx * dx + dy * dy;
    float t = len_sq > 0.0f ? ((p.n - a.n) * dx + (p.e - a.e) * dy) / len_sq : 0.0f;
    t = std::fmax(0.0f, std::fmin(1.0f, t));
    return std::hypot(p.n - (a.n + t * dx), p.e - (a.e + t * dy));
}

// Flies a polyline with a vehicle that follows the reference exactly, so
// only the reference generator itself is under test.
struct FlightLog {
    std::vector<PathTrackerOutput> ticks;
    float max_speed{0.0f};
    float max_step{0.0f};       // largest reference position jump in one tick
    float max_path_err{0.0f};   // largest reference distance from the polyline
    bool finished{false};
};

FlightLog fly(const std::vector<Pt>& wps, float cruise, float radius, PathTrackerLimits lim = {}) {
    PathTracker tracker(lim);
    VehicleState state;
    state.armed = true;
    state.dt_s = kDt;
    state.pos_ned[0] = wps[0].n;
    state.pos_ned[1] = wps[0].e;
    state.pos_ned[2] = -10.0f;

    auto make_seg = [&](size_t i) {
        PathSegment seg;
        seg.start[0] = wps[i - 1].n;
        seg.start[1] = wps[i - 1].e;
        seg.start[2] = -10.0f;
        seg.dest[0] = wps[i].n;
        seg.dest[1] = wps[i].e;
        seg.dest[2] = -10.0f;
        seg.has_next = i + 1 < wps.size();
        if (seg.has_next) {
            seg.next[0] = wps[i + 1].n;
            seg.next[1] = wps[i + 1].e;
            seg.next[2] = -10.0f;
        }
        seg.cruise_speed_ms = cruise;
        seg.accept_radius_m = radius;
        return seg;
    };

    FlightLog log;
    size_t idx = 1;
    tracker.set_segment(make_seg(idx), state);
    float prev[2] = {wps[0].n, wps[0].e};
    for (int tick = 0; tick < 200000; ++tick) {
        PathTrackerOutput out = tracker.update(state);
        log.ticks.push_back(out);
        log.max_speed = std::fmax(log.max_speed, std::hypot(out.vel_ff_ne[0], out.vel_ff_ne[1]));
        log.max_step = std::fmax(log.max_step, std::hypot(out.pos_ned[0] - prev[0], out.pos_ned[1] - prev[1]));
        float err = 1e9f;
        for (size_t i = 1; i < wps.size(); ++i) {
            err = std::fmin(err, dist_to_segment({out.pos_ned[0], out.pos_ned[1]}, wps[i - 1], wps[i]));
        }
        log.max_path_err = std::fmax(log.max_path_err, err);
        prev[0] = out.pos_ned[0];
        prev[1] = out.pos_ned[1];

        state.pos_ned[0] = out.pos_ned[0];
        state.pos_ned[1] = out.pos_ned[1];
        state.vel_ned[0] = out.vel_ff_ne[0];
        state.vel_ned[1] = out.vel_ff_ne[1];

        if (out.reached) {
            if (++idx >= wps.size()) {
                log.finished = true;
                break;
            }
            tracker.set_segment(make_seg(idx), state);
        }
    }
    return log;
}

} // namespace

TEST(PathTrackerTest, SqrtControllerLinearThenSqrt) {
    // p = 1, limit = 2 -> linear zone up to limit / p^2 = 2
    EXPECT_FLOAT_EQ(PathTracker::sqrt_controller(1.0f, 1.0f, 2.0f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(PathTracker::sqrt_controller(8.0f, 1.0f, 2.0f, 0.0f), std::sqrt(2.0f * 2.0f * 7.0f));
    EXPECT_FLOAT_EQ(PathTracker::sqrt_controller(-8.0f, 1.0f, 2.0f, 0.0f), -std::sqrt(2.0f * 2.0f * 7.0f));
    // dt bound: cannot step past zero error in one tick (p * e = 1.0 > e / dt)
    EXPECT_FLOAT_EQ(PathTracker::sqrt_controller(0.001f, 1000.0f, 0.0f, 0.01f), 0.1f);
}

TEST(PathTrackerTest, StopLegNeverExceedsCruiseOrOvershoots) {
    FlightLog log = fly({{0, 0}, {40, 0}}, 5.0f, 2.0f);
    ASSERT_TRUE(log.finished);
    EXPECT_LE(log.max_speed, 5.0f + 1e-3f);
    float max_n = 0.0f;
    for (const auto& t : log.ticks) {
        max_n = std::fmax(max_n, t.pos_ned[0]);
    }
    EXPECT_LE(max_n, 40.0f + 1e-3f);
    EXPECT_NEAR(log.ticks.back().pos_ned[0], 40.0f, 0.06f);
    EXPECT_LE(log.max_path_err, 1e-3f);
}

TEST(PathTrackerTest, ShortLegPeakLimitedByBraking) {
    PathTrackerLimits lim;
    lim.accel_ms2 = 2.5f;
    FlightLog log = fly({{0, 0}, {4, 0}}, 10.0f, 1.0f, lim);
    ASSERT_TRUE(log.finished);
    // accelerate + brake over 4 m at 2.5 m/s^2 can peak at most sqrt(a * L)
    EXPECT_LE(log.max_speed, std::sqrt(2.5f * 4.0f) + 0.05f);
}

TEST(PathTrackerTest, CornerIsContinuousAndInsideTheTurn) {
    const float radius = 2.0f;
    PathTrackerLimits arc;
    arc.blend_corners = false;
    FlightLog log = fly({{0, 0}, {30, 0}, {30, 30}, {0, 30}}, 5.0f, radius, arc);
    ASSERT_TRUE(log.finished);
    // no setpoint jump at the waypoint handovers: at most one tick of travel
    EXPECT_LE(log.max_step, 5.0f * kDt + 1e-3f);
    // 90deg arc tangent 2 m from the vertex: R = 2, apex sits R*(sqrt2-1)
    // inside the corner, so the reference never leaves the polyline by more
    EXPECT_LE(log.max_path_err, radius * (std::sqrt(2.0f) - 1.0f) + 0.01f);
}

TEST(PathTrackerTest, CornerSpeedRespectsLateralAccel) {
    PathTrackerLimits lim;
    lim.accel_ms2 = 2.5f;
    lim.blend_corners = false;
    FlightLog log = fly({{0, 0}, {30, 0}, {30, 30}}, 8.0f, 2.0f, lim);
    ASSERT_TRUE(log.finished);
    // speed when passing closest to the corner vertex
    float best = 1e9f, v_at_corner = 0.0f;
    for (const auto& t : log.ticks) {
        float d = std::hypot(t.pos_ned[0] - 30.0f, t.pos_ned[1]);
        if (d < best) {
            best = d;
            v_at_corner = std::hypot(t.vel_ff_ne[0], t.vel_ff_ne[1]);
        }
    }
    EXPECT_LE(v_at_corner, std::sqrt(2.5f * 2.0f) + 0.05f);
    EXPECT_GT(v_at_corner, 1.0f); // flown through, not stopped
}

TEST(PathTrackerTest, CollinearWaypointKeepsCruiseSpeed) {
    FlightLog log = fly({{0, 0}, {30, 0}, {60, 0}}, 5.0f, 2.0f);
    ASSERT_TRUE(log.finished);
    for (const auto& t : log.ticks) {
        if (std::fabs(t.pos_ned[0] - 30.0f) < 0.1f) {
            EXPECT_NEAR(std::hypot(t.vel_ff_ne[0], t.vel_ff_ne[1]), 5.0f, 0.05f);
        }
    }
}

TEST(PathTrackerTest, UTurnStopsAtWaypoint) {
    FlightLog log = fly({{0, 0}, {30, 0}, {0, 0.5f}}, 5.0f, 2.0f);
    ASSERT_TRUE(log.finished);
    float max_n = 0.0f;
    for (const auto& t : log.ticks) {
        max_n = std::fmax(max_n, t.pos_ned[0]);
    }
    EXPECT_LE(max_n, 30.0f + 1e-3f);
}

// --- Curvature-continuous corner blend (WPNAV_BLEND = 1) -------------------

namespace {

struct Smoothness {
    float max_accel{0.0f};      // |reference accel|
    float max_lat_accel{0.0f};  // accel normal to the reference velocity
    float max_jerk{0.0f};       // |d accel / dt|
    float max_lat_jerk{0.0f};   // d(lateral accel)/dt
};

Smoothness smoothness(const FlightLog& log) {
    Smoothness r;
    float prev_a[2] = {0.0f, 0.0f};
    float prev_lat = 0.0f;
    bool first = true;
    for (const auto& t : log.ticks) {
        const float vn = t.vel_ff_ne[0];
        const float ve = t.vel_ff_ne[1];
        const float v = std::hypot(vn, ve);
        const float an = t.accel_ff_ne[0];
        const float ae = t.accel_ff_ne[1];
        const float lat = v > 0.3f ? (vn * ae - ve * an) / v : 0.0f; // signed
        r.max_accel = std::fmax(r.max_accel, std::hypot(an, ae));
        r.max_lat_accel = std::fmax(r.max_lat_accel, std::fabs(lat));
        // Moving ticks only: the final stop zeroes a small residual accel
        // in one tick (stop logic, not corner shaping).
        if (!first && v > 0.3f) {
            r.max_jerk = std::fmax(r.max_jerk, std::hypot(an - prev_a[0], ae - prev_a[1]) / kDt);
            r.max_lat_jerk = std::fmax(r.max_lat_jerk, std::fabs(lat - prev_lat) / kDt);
        }
        prev_a[0] = an;
        prev_a[1] = ae;
        prev_lat = lat;
        first = false;
    }
    return r;
}

float speed_nearest(const FlightLog& log, Pt vertex, float* closest = nullptr) {
    float best = 1e9f;
    float v = 0.0f;
    for (const auto& t : log.ticks) {
        const float d = std::hypot(t.pos_ned[0] - vertex.n, t.pos_ned[1] - vertex.e);
        if (d < best) {
            best = d;
            v = std::hypot(t.vel_ff_ne[0], t.vel_ff_ne[1]);
        }
    }
    if (closest != nullptr) {
        *closest = best;
    }
    return v;
}

const std::vector<Pt> kSquare = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};

} // namespace

TEST(PathTrackerBlendTest, AccelAndJerkStayWithinLimitsThroughCorners) {
    PathTrackerLimits lim; // accel 2.5, jerk 5
    FlightLog log = fly(kSquare, 5.0f, 2.0f, lim);
    ASSERT_TRUE(log.finished);
    const Smoothness s = smoothness(log);
    // Lateral accel builds along the entry spiral: never above the limit,
    // never faster than the jerk limit.
    EXPECT_LE(s.max_lat_accel, lim.accel_ms2 * 1.02f);
    EXPECT_LE(s.max_lat_jerk, lim.jerk_ms3 * 1.1f);
    // Along-track and lateral jerk are each limited; their vector sum too.
    EXPECT_LE(s.max_jerk, std::sqrt(2.0f) * lim.jerk_ms3 * 1.1f);
    // No reference jump at the waypoint handovers.
    EXPECT_LE(log.max_step, 5.0f * kDt + 1e-3f);
}

TEST(PathTrackerBlendTest, ArcCornerStepsLateralAccelForComparison) {
    // The circular arc this replaces: lateral accel switches on in one tick.
    PathTrackerLimits arc;
    arc.blend_corners = false;
    FlightLog log = fly(kSquare, 5.0f, 2.0f, arc);
    ASSERT_TRUE(log.finished);
    EXPECT_GT(smoothness(log).max_lat_jerk, 20.0f * arc.jerk_ms3);
}

TEST(PathTrackerBlendTest, PassesWithinWaypointRadiusAndStaysOnLegs) {
    const float radius = 2.0f;
    FlightLog log = fly(kSquare, 5.0f, radius);
    ASSERT_TRUE(log.finished);
    float closest = 0.0f;
    const float v = speed_nearest(log, {40, 0}, &closest);
    EXPECT_LE(closest, radius + 0.01f);
    EXPECT_GT(v, 1.0f); // flown through, not stopped
    // The apex is the furthest point from both legs: radius * cos(45 deg).
    EXPECT_LE(log.max_path_err, radius * std::cos(0.25f * 3.14159265f) + 0.01f);
}

TEST(PathTrackerBlendTest, FasterThanArcAtTheSameRadius) {
    PathTrackerLimits arc;
    arc.blend_corners = false;
    const float v_arc = speed_nearest(fly(kSquare, 8.0f, 2.0f, arc), {40, 0});
    const float v_blend = speed_nearest(fly(kSquare, 8.0f, 2.0f), {40, 0});
    // 90 deg, accel 2.5, jerk 5, apex within 2 m: arc ~2.2 m/s, blend ~3.4 m/s.
    EXPECT_GT(v_blend, 1.4f * v_arc);
    EXPECT_LE(v_blend, 8.0f);
}

TEST(PathTrackerBlendTest, WideTurnKeepsMoreSpeed) {
    const float v90 = speed_nearest(fly({{0, 0}, {40, 0}, {40, 40}}, 8.0f, 2.0f), {40, 0});
    const float v30 = speed_nearest(fly({{0, 0}, {40, 0}, {74.6f, 20.0f}}, 8.0f, 2.0f), {40, 0});
    EXPECT_GT(v30, v90 + 2.0f);
}

TEST(PathTrackerBlendTest, ShortSurveyRowsStaySmooth) {
    PathTrackerLimits lim;
    const std::vector<Pt> survey = {{0, 0}, {30, 0}, {30, 5}, {0, 5}, {0, 10}, {30, 10}};
    FlightLog log = fly(survey, 5.0f, 2.0f, lim);
    ASSERT_TRUE(log.finished);
    const Smoothness s = smoothness(log);
    EXPECT_LE(s.max_lat_accel, lim.accel_ms2 * 1.02f);
    EXPECT_LE(s.max_lat_jerk, lim.jerk_ms3 * 1.1f);
    EXPECT_LE(s.max_jerk, std::sqrt(2.0f) * lim.jerk_ms3 * 1.1f);
    EXPECT_LE(log.max_step, 5.0f * kDt + 1e-3f);
    EXPECT_LE(log.max_path_err, 2.0f);
}
