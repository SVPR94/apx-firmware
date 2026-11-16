// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <cstdint>
#include <cmath>

namespace apx::sitl {

struct MultirotorParams {
    float mass_kg{2.0f};
    float inertia[9]{            // 3x3 inertia tensor kg.m^2
        0.02f, 0.0f, 0.0f,
        0.0f, 0.02f, 0.0f,
        0.0f, 0.0f, 0.04f
    };
    float kT{1.5e-5f};           // Thrust coefficient N/(rad/s)^2
    float kQ{3e-7f};             // Drag coefficient Nm/(rad/s)^2
    float tau_motor{0.02f};      // Motor time constant (s)
    float max_rpm{8000.0f};      // Max motor RPM
    int   num_motors{4};
    float motor_pos[8][3]{       // Motor positions (m) body frame
        { 0.15f,  0.15f, 0.0f},  // Motor 0: FR
        {-0.15f, -0.15f, 0.0f},  // Motor 1: RL
        { 0.15f, -0.15f, 0.0f},  // Motor 2: FL
        {-0.15f,  0.15f, 0.0f},  // Motor 3: RR
    };
    float motor_dir[8]{1,-1,-1,1,0,0,0,0}; // +1=CW, -1=CCW (for yaw torque)
    // Body drag: F = -0.5 * rho * drag_area * |v_air| * v_air, with v_air the
    // velocity relative to the wind. drag_area is Cd*A (m^2); 0 = no drag.
    float drag_area_m2{0.0f};
    float air_density{1.225f};

    static MultirotorParams make_quad_x(float mass, float arm_m);
    static MultirotorParams make_hex_x(float mass, float arm_m);
    static MultirotorParams make_octo_x(float mass, float arm_m);
};

struct PhysicsState {
    float position[3]{};       // NED (m)
    float velocity[3]{};       // NED (m/s)
    float quaternion[4]{1,0,0,0}; // attitude (w,x,y,z)
    float angular_rate[3]{};   // body frame (rad/s)
    float motor_omega[8]{};    // current motor angular velocity (rad/s)
    float accel_body[3]{};     // specific force body frame (m/s^2, includes gravity!)
    float time_s{0.0f};
};

class MultirotorDynamics {
public:
    explicit MultirotorDynamics(const MultirotorParams& params = MultirotorParams{});
    void reset();
    void set_params(const MultirotorParams& p) { m_params = p; }

    // Set motor throttle commands [0,1] for each motor
    void set_motor_throttle(int motor, float throttle_0_1);

    // Air velocity (NED, m/s) acting on the body drag from now on.
    void set_wind_ned(const float wind_ned[3]);
    // Speed of the vehicle relative to the air.
    float get_airspeed() const;

    // Step physics by dt_s using RK4
    void step(float dt_s);

    const PhysicsState& get_state() const { return m_state; }
    float get_altitude_agl() const { return -m_state.position[2]; }

private:
    struct Derivative {
        float dpos[3], dvel[3], dquat[4], domega[3], dmotor_omega[8];
    };

    Derivative compute_derivative(const PhysicsState& s, float dt_s) const;
    PhysicsState integrate(const PhysicsState& s, const Derivative& k, float h) const;

    void quat_deriv(const float q[4], const float omega[3], float dq[4]) const;
    void quat_to_dcm(const float q[4], float R[9]) const;
    void cross3(const float a[3], const float b[3], float out[3]) const;
    // On the ground with less thrust than weight: friction holds it in place.
    bool resting_on_ground(const PhysicsState& s) const;

    MultirotorParams m_params;
    PhysicsState     m_state;
    float m_throttle_cmd[8]{};
    float m_wind_ned[3]{};
};

} // namespace apx::sitl
