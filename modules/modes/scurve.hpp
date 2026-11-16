// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX — snap-limited S-curve trajectory along a straight 3D leg.
//
// A leg is travelled rest-to-rest along one profile s(t):
//
//   accel phase              cruise        decel phase
//   J+ ramp, J+, J+ ramp-out, hold a, J- ramp, J-, J- ramp-out | v_peak | mirror
//
// Jerk itself never steps: every jerk change is a half-cosine ramp of
// duration tj, so snap is bounded by J*pi/(2*tj). Peak speed, accel and jerk
// are the largest values the leg direction allows under separate horizontal
// and vertical limits. Short legs lower the peak speed (and accel) until the
// two phases fit.
//
// Consecutive legs are joined by running the next leg's acceleration phase
// on top of this leg's deceleration phase (see PathTracker): two rest-to-rest
// legs superimposed give a smooth, jerk-limited corner and, on collinear
// legs, an unchanged cruise speed.
#pragma once

#include <cstdint>

namespace apx::modes {

struct ScurveLimits {
    float speed_xy{10.0f};   // m/s horizontal
    float speed_up{2.5f};    // m/s climb
    float speed_down{1.5f};  // m/s descent
    float accel_xy{2.5f};    // m/s^2 horizontal
    float accel_z{1.0f};     // m/s^2 vertical
    float jerk{1.0f};        // m/s^3 along the leg
    float jerk_time{0.2f};   // s, half-cosine jerk ramp (sets the snap limit)
};

// 1D rest-to-rest profile over a distance.
class ScurveProfile {
public:
    void clear();
    // Returns false (and leaves an empty profile) for non-positive inputs.
    bool plan(float length, float v_max, float a_max, float j_max, float tj);

    bool valid() const { return m_n > 0; }
    float length() const { return m_length; }
    float duration() const { return m_duration; }
    float peak_speed() const { return m_v_peak; }
    float accel_end_time() const { return m_t_accel_end; }   // end of the accel phase
    float decel_start_time() const { return m_t_decel_start; }

    // Position, speed and accel along the profile at time t (clamped to [0, duration]).
    void eval(float t, float& s, float& v, float& a) const;

private:
    enum class Kind : uint8_t { JerkRise, JerkConst, JerkFall };
    struct Seg {
        Kind kind;
        float t0;   // segment start time
        float dur;
        float j;    // jerk level (signed) the segment ramps to / holds / ramps from
        float s0, v0, a0;
    };
    static constexpr int kMaxSegs = 15;

    void add(Kind kind, float dur, float j);
    static void step(const Seg& g, float tau, float tj, float& s, float& v, float& a);

    Seg m_seg[kMaxSegs]{};
    int m_n{0};
    float m_tj{0.2f};
    float m_length{0.0f};
    float m_duration{0.0f};
    float m_v_peak{0.0f};
    float m_t_accel_end{0.0f};
    float m_t_decel_start{0.0f};
};

// Straight 3D leg (NED) travelled with an ScurveProfile.
class ScurveLeg {
public:
    void clear();
    bool plan(const float origin[3], const float dest[3], const ScurveLimits& lim);

    bool valid() const { return m_profile.valid(); }
    const float* origin() const { return m_origin; }
    const float* dest() const { return m_dest; }
    const float* track() const { return m_track; }   // dest - origin
    float length() const { return m_profile.length(); }
    float duration() const { return m_profile.duration(); }
    float accel_end_time() const { return m_profile.accel_end_time(); }
    float decel_start_time() const { return m_profile.decel_start_time(); }
    float speed_limit() const { return m_v_max; }    // projected speed ceiling of this leg
    float accel_z_limit() const { return m_accel_z; }

    // Displacement from origin, velocity and accel at leg time t.
    void sample(float t, float disp[3], float vel[3], float acc[3]) const;

private:
    float m_origin[3]{};
    float m_dest[3]{};
    float m_track[3]{};
    float m_unit[3]{};
    float m_v_max{0.0f};
    float m_accel_z{0.0f};
    ScurveProfile m_profile;
};

// Largest magnitude along a direction (horizontal length h, vertical
// component dz, NED: dz < 0 climbs) whose horizontal part stays within
// max_xy and vertical part within max_up / max_down.
float scurve_direction_limit(float h, float dz, float max_xy, float max_up, float max_down);

} // namespace apx::modes
