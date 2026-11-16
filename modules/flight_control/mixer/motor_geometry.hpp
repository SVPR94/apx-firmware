// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

namespace apx::control {

enum class FrameType {
    QUAD_X,
    QUAD_PLUS,
    HEX_X,
    HEX_FLAT,
    OCTO_X,
    OCTO_COAX,
    OCTO_PLUS
};

struct MotorConfig {
    float roll_factor;
    float pitch_factor;
    float yaw_factor;
    int index;
};

// roll_factor/pitch_factor: response of each motor to a mixer roll/pitch
// command (FRD body frame, +Y=right, +X=forward - matches
// modules/modes/navigation_cascade.cpp's accel->tilt convention and the
// MAVLink ATTITUDE roll/pitch this project reports). Previously these two
// columns were transposed here (documented in the old per-motor comments
// as "+roll" for both front motors instead of both right motors) - it
// went unnoticed because thrust was the only mixer input ever exercised
// until modules/modes started commanding roll/pitch, at which point it
// showed up as the vehicle tumbling instead of holding/navigating (roll
// commands were actually being applied as a front/rear pitch differential
// and vice versa - a real positive-feedback loop, not just "faces the
// wrong way"). QUAD_X was re-derived from first principles (torque = r x
// F at each motor's actual sitl/models/apx_quad/model.sdf position) and
// confirmed by a live SITL run that a guided target now converges instead
// of diverging. HEX_X/OCTO_X got the same fix by the same reasoning but
// aren't backed by a real model/live test yet - re-verify before relying
// on them.
constexpr MotorConfig QUAD_X_MOTORS[4] = {
    {-1.0f,  1.0f,  1.0f, 0}, // FR
    { 1.0f, -1.0f,  1.0f, 1}, // BL
    { 1.0f,  1.0f, -1.0f, 2}, // FL
    {-1.0f, -1.0f, -1.0f, 3}  // BR
};

// yaw_factor for idx1/idx3 was flipped from what their actual geometric
// position (roll_factor/pitch_factor, unchanged below) requires: sorting
// all 6 motors by their real position angle around the hex (atan2(roll,
// pitch), the same derivation that fixed QUAD_X - see this file's header
// comment) showed CCW,CCW and CW,CW adjacent pairs at idx5/idx1 and
// idx3/idx4 instead of strict CW/CCW alternation all the way around the
// ring. A hex mixer needs that strict alternation for yaw authority to
// actually cancel reaction torque symmetrically between neighbors -
// confirmed by the same position-sorted check, now alternating CW/CCW at
// every neighboring pair after this fix. Matches this file's own header
// warning that HEX_X wasn't live-verified yet.
constexpr MotorConfig HEX_X_MOTORS[6] = {
    {-0.5f,   0.866f,  1.0f, 0},
    { 0.5f,  -0.866f, -1.0f, 1},
    { 0.5f,   0.866f, -1.0f, 2},
    {-0.5f,  -0.866f,  1.0f, 3},
    {-1.0f,   0.0f,   -1.0f, 4},
    { 1.0f,   0.0f,    1.0f, 5}
};

constexpr MotorConfig OCTO_X_MOTORS[8] = {
    {-0.924f,  0.383f,  1.0f, 0},
    {-0.383f,  0.924f, -1.0f, 1},
    { 0.383f,  0.924f,  1.0f, 2},
    { 0.924f,  0.383f, -1.0f, 3},
    { 0.924f, -0.383f,  1.0f, 4},
    { 0.383f, -0.924f, -1.0f, 5},
    {-0.383f, -0.924f,  1.0f, 6},
    {-0.924f, -0.383f, -1.0f, 7}
};

} // namespace apx::control
