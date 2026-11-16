// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "attitude_controller.hpp"
#include <cmath>

namespace apx::control {

AttitudeController::AttitudeController() {}

void AttitudeController::update(const float q_desired[4], const float q_actual[4], float dt_s, float desired_rate_rads[3]) {
    // q_err = conj(q_actual) * q_desired - the BODY-FRAME rotation needed
    // to take the vehicle from its actual to its desired attitude, which
    // is what desired_rate_rads must be in (it's compared directly
    // against state.gyro_body, a body-frame measurement, by the caller's
    // rate controller). The previous formula here, q_desired*conj(q_actual),
    // is the WORLD-frame error instead - for a SMALL error the two are
    // nearly indistinguishable (why every small-angle case tested this
    // session - hover, gentle waypoint flying - worked fine), but they
    // diverge sharply once yaw has rotated substantially: confirmed live
    // via telemetry and hand-verified numerically (see the investigation
    // that found this) that with ~100+ deg of accumulated yaw rotation
    // and a large simultaneous tilt error, the world-frame formula
    // computed a roll-rate correction with the WRONG SIGN - one that
    // drove roll further from level instead of back toward it - which is
    // exactly why the tilt safety fail-safe (see NavigationCascade's
    // m_safety_tilt_rad) never actually recovered once triggered: it
    // correctly zeroed the target, but the wrong-frame error computation
    // fed that (correct) target through a correction that made the real
    // divergence worse, not better.
    float qa[4] = {q_actual[0], -q_actual[1], -q_actual[2], -q_actual[3]};
    float qe[4];
    qe[0] = qa[0]*q_desired[0] - qa[1]*q_desired[1] - qa[2]*q_desired[2] - qa[3]*q_desired[3];
    qe[1] = qa[0]*q_desired[1] + qa[1]*q_desired[0] + qa[2]*q_desired[3] - qa[3]*q_desired[2];
    qe[2] = qa[0]*q_desired[2] - qa[1]*q_desired[3] + qa[2]*q_desired[0] + qa[3]*q_desired[1];
    qe[3] = qa[0]*q_desired[3] + qa[1]*q_desired[2] - qa[2]*q_desired[1] + qa[3]*q_desired[0];
    
    if (qe[0] < 0.0f) {
        qe[0] = -qe[0]; qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3];
    }

    // Swing-twist decomposition about body Z, NOT a single combined
    // axis-angle: qe is split into a pure-yaw "twist" (rotation about body
    // Z only) and the remaining pure-tilt "swing" (roll/pitch only, zero Z
    // component by construction), corrected completely independently.
    //
    // A single combined axis-angle (the previous version of this
    // function) reads qe's x/y/z components off directly as "roll rate,
    // pitch rate, yaw rate" - which only approximates independent
    // per-axis corrections for SMALL errors. Live SITL testing found that
    // once a real disturbance produced a LARGE simultaneous tilt error
    // *and* a large yaw error together (confirmed via telemetry: ~45deg
    // roll, ~33deg pitch, and a ~30deg yaw error all at once after a sharp
    // waypoint-corner turn), the combined rotation's axis stopped
    // corresponding to "reduce roll error, reduce pitch error" - the
    // vehicle's actual roll/pitch continued diverging *further* for
    // several seconds even after NavigationCascade's safety fail-safe
    // correctly zeroed the roll/pitch target, eventually tumbling. We
    // avoid this by controlling thrust-vector tilt and heading as two
    // independent corrections rather than one combined quaternion error
    // - this is that swing-twist decoupling.
    float twist_norm_sq = qe[0]*qe[0] + qe[3]*qe[3];
    float qt[4]; // twist: pure rotation about body Z
    if (twist_norm_sq > 1e-9f) {
        float inv_norm = 1.0f / std::sqrt(twist_norm_sq);
        qt[0] = qe[0] * inv_norm; qt[1] = 0.0f; qt[2] = 0.0f; qt[3] = qe[3] * inv_norm;
    } else {
        qt[0] = 1.0f; qt[1] = 0.0f; qt[2] = 0.0f; qt[3] = 0.0f;
    }

    // swing = qe * conj(qt) - the remaining tilt-only rotation, guaranteed
    // by the swing-twist identity to have zero component along Z.
    float qt_conj[4] = {qt[0], -qt[1], -qt[2], -qt[3]};
    float qs[4];
    qs[0] = qe[0]*qt_conj[0] - qe[1]*qt_conj[1] - qe[2]*qt_conj[2] - qe[3]*qt_conj[3];
    qs[1] = qe[0]*qt_conj[1] + qe[1]*qt_conj[0] + qe[2]*qt_conj[3] - qe[3]*qt_conj[2];
    qs[2] = qe[0]*qt_conj[2] - qe[1]*qt_conj[3] + qe[2]*qt_conj[0] + qe[3]*qt_conj[1];
    qs[3] = qe[0]*qt_conj[3] + qe[1]*qt_conj[2] - qe[2]*qt_conj[1] + qe[3]*qt_conj[0];
    if (qs[0] < 0.0f) { qs[0] = -qs[0]; qs[1] = -qs[1]; qs[2] = -qs[2]; qs[3] = -qs[3]; }

    // atan2(|vec|, w), not acos(w): see this function's original comment -
    // acos's derivative blows up as w->1, exactly for the small errors
    // this controller spends nearly all its time correcting.
    float sin_half_s = std::sqrt(qs[1]*qs[1] + qs[2]*qs[2] + qs[3]*qs[3]);
    float tilt_angle = 2.0f * std::atan2(sin_half_s, qs[0]);
    float tilt_axis[3] = {0, 0, 0};
    if (sin_half_s > 1e-6f) {
        tilt_axis[0] = qs[1] / sin_half_s;
        tilt_axis[1] = qs[2] / sin_half_s;
    }
    float yaw_angle = 2.0f * std::atan2(qt[3], qt[0]);

    float raw_rate[3];
    raw_rate[0] = clamp(m_p_gain[0] * tilt_angle * tilt_axis[0], -m_max_rate_rp, m_max_rate_rp);
    raw_rate[1] = clamp(m_p_gain[1] * tilt_angle * tilt_axis[1], -m_max_rate_rp, m_max_rate_rp);
    raw_rate[2] = clamp(m_p_gain[2] * yaw_angle, -m_max_rate_y, m_max_rate_y);

    // Angular-acceleration limit (ATC_ACCEL_{R,P,Y}_MAX-equivalent): smooths
    // a large, sudden STEP-UP in demanded rate (e.g. a waypoint corner
    // suddenly asking for a much bigger lean in the same direction the
    // vehicle is already leaning) instead of slamming the rate loop with
    // an effectively-infinite step every tick.
    //
    // Live SITL testing found this must NEVER slow down a DECREASE or a
    // sign reversal of the demanded rate: raw_rate[i] is recomputed fresh
    // from the actual angle error every tick, so once the vehicle
    // overshoots past its target, raw_rate correctly and instantly flips
    // sign to command recovery the other way. Slew-limiting that flip
    // (the original, buggy version of this code did) forces the rate
    // loop to keep applying the WRONG-direction command for
    // |2*max_rate / max_accel| seconds after the overshoot - at the
    // defaults (ATC_ACCEL_R_MAX=300deg/s^2, max rate 90deg/s) that's
    // ~0.6s of actively driving the tilt further past the target, which
    // is exactly what was turning a recoverable waypoint-corner
    // disturbance into a full tumble (confirmed via DBG2 telemetry:
    // desired_rate pinned at +max_rate for 50+ consecutive ticks while
    // actual roll swung from -58 deg through 0 to +63 deg and beyond).
    // So: only rate-limit an INCREASE in magnitude with the same sign as
    // the previous command: that is the "smoothing a step" case. Any
    // decrease or reversal - i.e. every case where the controller is
    // trying to arrest or undo overshoot - passes through unslewed.
    for (int i = 0; i < 3; ++i) {
        float max_accel = (i < 2) ? m_max_accel_rp : m_max_accel_y;
        bool same_sign = m_have_prev_rate &&
            ((raw_rate[i] >= 0.0f) == (m_prev_desired_rate[i] >= 0.0f));
        bool increasing = same_sign &&
            (std::fabs(raw_rate[i]) > std::fabs(m_prev_desired_rate[i]));
        if (max_accel <= 0.0f || dt_s <= 0.0f || !increasing) {
            desired_rate_rads[i] = raw_rate[i];
        } else {
            float max_step = max_accel * dt_s;
            float delta = clamp(raw_rate[i] - m_prev_desired_rate[i], -max_step, max_step);
            desired_rate_rads[i] = m_prev_desired_rate[i] + delta;
        }
        m_prev_desired_rate[i] = desired_rate_rads[i];
    }
    m_have_prev_rate = true;
}

void AttitudeController::set_proportional_gain(float p_roll, float p_pitch, float p_yaw) {
    m_p_gain[0] = p_roll; m_p_gain[1] = p_pitch; m_p_gain[2] = p_yaw;
}

void AttitudeController::set_rate_limits(float max_roll_pitch, float max_yaw) {
    m_max_rate_rp = max_roll_pitch;
    m_max_rate_y = max_yaw;
}

void AttitudeController::set_accel_limits(float max_accel_rp_rads2, float max_accel_y_rads2) {
    m_max_accel_rp = max_accel_rp_rads2;
    m_max_accel_y = max_accel_y_rads2;
}

void AttitudeController::reset() {
    m_prev_desired_rate[0] = m_prev_desired_rate[1] = m_prev_desired_rate[2] = 0.0f;
    m_have_prev_rate = false;
}

float AttitudeController::clamp(float val, float min, float max) const {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

} // namespace apx::control
