// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX — snap-limited S-curve trajectory along a straight 3D leg.
#include "scurve.hpp"

#include <algorithm>
#include <cmath>

namespace apx::modes {

namespace {
constexpr float kPi = 3.14159265358979323846f;

// One acceleration phase: speed 0 -> vp (or vp -> 0 mirrored).
struct Phase {
    float a{0.0f};    // accel reached
    float j{0.0f};    // jerk level of the ramps
    float tcj{0.0f};  // constant-jerk time inside each ramp
    float t4{0.0f};   // constant-accel time
    float T{0.0f};    // phase duration
};

// Accel ramp to level a and back: duration and velocity gained.
float ramp_time(float a, float jm, float tj) {
    return (a >= jm * tj) ? (tj + a / jm) : 2.0f * tj;
}

Phase accel_phase(float vp, float am, float jm, float tj) {
    Phase p;
    if (!(vp > 0.0f)) return p;
    float a = am;
    if (am * ramp_time(am, jm, tj) > vp) {
        // a * ramp_time(a) = vp has no room for an accel hold
        const float a1 = 0.5f * jm * (-tj + std::sqrt(tj * tj + 4.0f * vp / jm));
        a = (a1 >= jm * tj) ? a1 : vp / (2.0f * tj);
    }
    p.a = a;
    if (a >= jm * tj) {
        p.j = jm;
        p.tcj = a / jm - tj;
    } else {
        p.j = a / tj;
        p.tcj = 0.0f;
    }
    const float tr = 2.0f * tj + p.tcj;
    p.t4 = std::max(0.0f, (vp - a * tr) / a);
    p.T = 2.0f * tr + p.t4;
    return p;
}

// Distance of a rest-to-rest profile peaking at vp with no cruise:
// each symmetric phase covers vp*T/2.
float phases_distance(float vp, float am, float jm, float tj) {
    return vp * accel_phase(vp, am, jm, tj).T;
}
} // namespace

float scurve_direction_limit(float h, float dz, float max_xy, float max_up, float max_down) {
    const float len = std::sqrt(h * h + dz * dz);
    if (!(len > 0.0f)) return 0.0f;
    const float uh = h / len;
    const float uz = std::fabs(dz) / len;
    const float max_z = (dz < 0.0f) ? max_up : max_down;
    if (uh < 1.0e-6f) return max_z;
    if (uz < 1.0e-6f) return max_xy;
    return std::min(max_xy / uh, max_z / uz);
}

// ─── ScurveProfile ────────────────────────────────────────────────────────────

void ScurveProfile::clear() {
    m_n = 0;
    m_length = m_duration = m_v_peak = 0.0f;
    m_t_accel_end = m_t_decel_start = 0.0f;
}

void ScurveProfile::add(Kind kind, float dur, float j) {
    if (!(dur > 1.0e-6f) || m_n >= kMaxSegs) return;
    Seg& g = m_seg[m_n];
    g.kind = kind;
    g.dur = dur;
    g.j = j;
    if (m_n == 0) {
        g.t0 = 0.0f;
        g.s0 = g.v0 = g.a0 = 0.0f;
    } else {
        const Seg& p = m_seg[m_n - 1];
        g.t0 = p.t0 + p.dur;
        step(p, p.dur, m_tj, g.s0, g.v0, g.a0);
    }
    ++m_n;
}

// State after tau seconds into segment g. Half-cosine jerk ramps:
//   rise: j(tau) = J/2 (1 - cos(w tau)),  fall: j(tau) = J/2 (1 + cos(w tau)),  w = pi/tj
void ScurveProfile::step(const Seg& g, float tau, float tj, float& s, float& v, float& a) {
    const float J = g.j;
    const float t2 = tau * tau;
    const float t3 = t2 * tau;
    float da, dv, ds;
    if (g.kind == Kind::JerkConst) {
        da = J * tau;
        dv = 0.5f * J * t2;
        ds = J * t3 / 6.0f;
    } else {
        const float w = kPi / tj;
        const float sw = std::sin(w * tau) / w;
        const float cw = (std::cos(w * tau) - 1.0f) / (w * w);
        const float sgn = (g.kind == Kind::JerkRise) ? -1.0f : 1.0f;
        da = 0.5f * J * (tau + sgn * sw);
        dv = 0.5f * J * (0.5f * t2 + sgn * cw);
        ds = 0.5f * J * (t3 / 6.0f + sgn * (sw - tau) / (w * w));
    }
    a = g.a0 + da;
    v = g.v0 + g.a0 * tau + dv;
    s = g.s0 + g.v0 * tau + 0.5f * g.a0 * t2 + ds;
}

bool ScurveProfile::plan(float length, float v_max, float a_max, float j_max, float tj) {
    clear();
    if (!(length > 1.0e-4f) || !(v_max > 0.0f) || !(a_max > 0.0f) || !(j_max > 0.0f) || !(tj > 0.0f)) {
        return false;
    }
    m_tj = tj;

    // Peak speed: v_max if both phases fit, else the speed whose phases
    // exactly cover the leg (distance grows monotonically with speed).
    float vp = v_max;
    float cruise = 0.0f;
    const float d_max = phases_distance(v_max, a_max, j_max, tj);
    if (d_max <= length) {
        cruise = (length - d_max) / v_max;
    } else {
        float lo = 0.0f, hi = v_max;
        for (int i = 0; i < 40; ++i) {
            const float mid = 0.5f * (lo + hi);
            (phases_distance(mid, a_max, j_max, tj) > length ? hi : lo) = mid;
        }
        vp = lo;
    }
    const Phase p = accel_phase(vp, a_max, j_max, tj);
    if (!(p.T > 0.0f)) return false;

    // accel phase
    add(Kind::JerkRise, tj, p.j);
    add(Kind::JerkConst, p.tcj, p.j);
    add(Kind::JerkFall, tj, p.j);
    add(Kind::JerkConst, p.t4, 0.0f);
    add(Kind::JerkRise, tj, -p.j);
    add(Kind::JerkConst, p.tcj, -p.j);
    add(Kind::JerkFall, tj, -p.j);
    m_t_accel_end = p.T;
    // cruise
    add(Kind::JerkConst, cruise, 0.0f);
    m_t_decel_start = p.T + cruise;
    // decel phase
    add(Kind::JerkRise, tj, -p.j);
    add(Kind::JerkConst, p.tcj, -p.j);
    add(Kind::JerkFall, tj, -p.j);
    add(Kind::JerkConst, p.t4, 0.0f);
    add(Kind::JerkRise, tj, p.j);
    add(Kind::JerkConst, p.tcj, p.j);
    add(Kind::JerkFall, tj, p.j);

    const Seg& last = m_seg[m_n - 1];
    m_duration = last.t0 + last.dur;
    m_length = length;
    m_v_peak = vp;
    return true;
}

void ScurveProfile::eval(float t, float& s, float& v, float& a) const {
    if (m_n == 0 || t <= 0.0f) {
        s = v = a = 0.0f;
        return;
    }
    if (t >= m_duration) {
        s = m_length;
        v = a = 0.0f;
        return;
    }
    int i = m_n - 1;
    while (i > 0 && m_seg[i].t0 > t) --i;
    step(m_seg[i], t - m_seg[i].t0, m_tj, s, v, a);
    s = std::clamp(s, 0.0f, m_length);
}

// ─── ScurveLeg ────────────────────────────────────────────────────────────────

void ScurveLeg::clear() {
    m_profile.clear();
    for (int i = 0; i < 3; ++i) {
        m_track[i] = m_unit[i] = 0.0f;
    }
    m_v_max = m_accel_z = 0.0f;
}

bool ScurveLeg::plan(const float origin[3], const float dest[3], const ScurveLimits& lim) {
    clear();
    for (int i = 0; i < 3; ++i) {
        m_origin[i] = origin[i];
        m_dest[i] = dest[i];
        m_track[i] = dest[i] - origin[i];
    }
    const float h = std::sqrt(m_track[0] * m_track[0] + m_track[1] * m_track[1]);
    const float len = std::sqrt(h * h + m_track[2] * m_track[2]);
    if (!(len > 1.0e-3f)) return false;
    for (int i = 0; i < 3; ++i) m_unit[i] = m_track[i] / len;
    m_v_max = scurve_direction_limit(h, m_track[2], lim.speed_xy, lim.speed_up, lim.speed_down);
    const float a_max = scurve_direction_limit(h, m_track[2], lim.accel_xy, lim.accel_z, lim.accel_z);
    m_accel_z = lim.accel_z;
    return m_profile.plan(len, m_v_max, a_max, lim.jerk, lim.jerk_time);
}

void ScurveLeg::sample(float t, float disp[3], float vel[3], float acc[3]) const {
    float s, v, a;
    m_profile.eval(t, s, v, a);
    for (int i = 0; i < 3; ++i) {
        disp[i] = m_unit[i] * s;
        vel[i] = m_unit[i] * v;
        acc[i] = m_unit[i] * a;
    }
}

} // namespace apx::modes
