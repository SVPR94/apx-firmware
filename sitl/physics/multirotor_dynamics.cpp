// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "multirotor_dynamics.hpp"
#include <cstring>
#include <algorithm>

namespace apx::sitl {

static constexpr float kG = 9.80665f, kPi = 3.14159265f;

MultirotorParams MultirotorParams::make_quad_x(float mass, float arm_m) {
    MultirotorParams p;
    p.mass_kg = mass;
    // Inertia: 2*mass*arm^2/3 for cross shape
    float Ixy = mass*arm_m*arm_m/2.0f;
    float Iz  = mass*arm_m*arm_m;
    p.inertia[0]=Ixy; p.inertia[4]=Ixy; p.inertia[8]=Iz;
    p.num_motors = 4;
    float a = arm_m/std::sqrt(2.0f);
    float mp[4][3]={{a,a,0},{-a,-a,0},{a,-a,0},{-a,a,0}};
    for(int i=0;i<4;i++) for(int j=0;j<3;j++) p.motor_pos[i][j]=mp[i][j];
    p.motor_dir[0]=1; p.motor_dir[1]=1; p.motor_dir[2]=-1; p.motor_dir[3]=-1;
    return p;
}

// Motor layout convention (FRD body): motor i at (x, y) with reaction-torque
// sign motor_dir[i]. The firmware mixer drives such a motor with
// pitch_factor = x/arm, roll_factor = -y/arm, yaw_factor = motor_dir - see
// tests/unit/test_sim_airframe_geometry.cpp, which checks every built-in
// airframe against the mixer tables. The geometry is written out here as the
// physical airframe rather than derived from the mixer, so a wrong mixer
// table can't silently fly in simulation.
static void set_motors(MultirotorParams& p, int n, const float layout[][3], float arm_m) {
    p.num_motors = n;
    for (int i = 0; i < n; ++i) {
        p.motor_pos[i][0] = layout[i][0] * arm_m;
        p.motor_pos[i][1] = layout[i][1] * arm_m;
        p.motor_pos[i][2] = 0.0f;
        p.motor_dir[i] = layout[i][2];
    }
}

// Hexa-X: no arm on the nose; motors at +-30, +-90, +-150 deg.
// Motor order (outputs 1-6): front-right, rear-left, front-left, rear-right,
// right, left. (This used to place motor 1 on the nose at 60-deg steps - a
// hexa-plus airframe - which the HEX_X mixer doesn't drive correctly: the
// hex flew while symmetric and flipped once any asymmetry built up.)
MultirotorParams MultirotorParams::make_hex_x(float mass, float arm_m) {
    MultirotorParams p = make_quad_x(mass, arm_m);
    static constexpr float kC = 0.8660254f;  // cos 30
    static constexpr float kS = 0.5f;        // sin 30
    static constexpr float layout[6][3] = {
        { kC,  kS,  1.0f},   // 1 front-right   30 deg
        {-kC, -kS, -1.0f},   // 2 rear-left    210 deg
        { kC, -kS, -1.0f},   // 3 front-left   330 deg
        {-kC,  kS,  1.0f},   // 4 rear-right   150 deg
        {0.0f, 1.0f, -1.0f}, // 5 right         90 deg
        {0.0f, -1.0f, 1.0f}, // 6 left         270 deg
    };
    set_motors(p, 6, layout, arm_m);
    // Ring of 6 point masses: Ixx = Iyy = m r^2 / 2, Izz = m r^2 (same
    // approximation as make_quad_x).
    p.inertia[0] = p.inertia[4] = mass * arm_m * arm_m / 2.0f;
    p.inertia[8] = mass * arm_m * arm_m;
    return p;
}

// Octa-X: motors at 22.5 + k*45 deg, alternating spin around the ring.
MultirotorParams MultirotorParams::make_octo_x(float mass, float arm_m) {
    MultirotorParams p = make_quad_x(mass, arm_m);
    static constexpr float kA = 0.9238795f;  // cos 22.5
    static constexpr float kB = 0.3826834f;  // sin 22.5
    static constexpr float layout[8][3] = {
        { kB,  kA,  1.0f},   // 1   67.5 deg
        { kA,  kB, -1.0f},   // 2   22.5 deg
        { kA, -kB,  1.0f},   // 3  337.5 deg
        { kB, -kA, -1.0f},   // 4  292.5 deg
        {-kB, -kA,  1.0f},   // 5  247.5 deg
        {-kA, -kB, -1.0f},   // 6  202.5 deg
        {-kA,  kB,  1.0f},   // 7  157.5 deg
        {-kB,  kA, -1.0f},   // 8  112.5 deg
    };
    set_motors(p, 8, layout, arm_m);
    p.inertia[0] = p.inertia[4] = mass * arm_m * arm_m / 2.0f;
    p.inertia[8] = mass * arm_m * arm_m;
    return p;
}

MultirotorDynamics::MultirotorDynamics(const MultirotorParams& p) : m_params(p) { reset(); }

void MultirotorDynamics::reset() {
    std::memset(&m_state, 0, sizeof(m_state));
    m_state.quaternion[0]=1.0f;
    std::memset(m_throttle_cmd, 0, sizeof(m_throttle_cmd));
}

void MultirotorDynamics::set_motor_throttle(int m, float t) {
    if(m>=0 && m<8) m_throttle_cmd[m]=std::max(0.0f,std::min(1.0f,t));
}

void MultirotorDynamics::set_wind_ned(const float wind_ned[3]) {
    for (int i = 0; i < 3; ++i) m_wind_ned[i] = wind_ned[i];
}

float MultirotorDynamics::get_airspeed() const {
    float sq = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const float v = m_state.velocity[i] - m_wind_ned[i];
        sq += v * v;
    }
    return std::sqrt(sq);
}

bool MultirotorDynamics::resting_on_ground(const PhysicsState& s) const {
    if (s.position[2] < 0.0f) return false;
    float thrust = 0.0f;
    for (int i = 0; i < m_params.num_motors; ++i) {
        thrust += m_params.kT * s.motor_omega[i] * s.motor_omega[i];
    }
    return thrust < m_params.mass_kg * kG;
}

void MultirotorDynamics::quat_to_dcm(const float q[4], float R[9]) const {
    float q0=q[0],q1=q[1],q2=q[2],q3=q[3];
    R[0]=1-2*(q2*q2+q3*q3); R[1]=2*(q1*q2-q0*q3);   R[2]=2*(q1*q3+q0*q2);
    R[3]=2*(q1*q2+q0*q3);   R[4]=1-2*(q1*q1+q3*q3); R[5]=2*(q2*q3-q0*q1);
    R[6]=2*(q1*q3-q0*q2);   R[7]=2*(q2*q3+q0*q1);   R[8]=1-2*(q1*q1+q2*q2);
}

void MultirotorDynamics::quat_deriv(const float q[4], const float w[3], float dq[4]) const {
    dq[0]=0.5f*(-q[1]*w[0]-q[2]*w[1]-q[3]*w[2]);
    dq[1]=0.5f*( q[0]*w[0]+q[2]*w[2]-q[3]*w[1]);
    dq[2]=0.5f*( q[0]*w[1]-q[1]*w[2]+q[3]*w[0]);
    dq[3]=0.5f*( q[0]*w[2]+q[1]*w[1]-q[2]*w[0]);
}

void MultirotorDynamics::cross3(const float a[3], const float b[3], float out[3]) const {
    out[0]=a[1]*b[2]-a[2]*b[1]; out[1]=a[2]*b[0]-a[0]*b[2]; out[2]=a[0]*b[1]-a[1]*b[0];
}

MultirotorDynamics::Derivative MultirotorDynamics::compute_derivative(
        const PhysicsState& s, float dt_s) const {
    Derivative d{};

    // Motor dynamics: first-order lag
    float max_omega = m_params.max_rpm * kPi / 30.0f; // RPM to rad/s
    for(int i=0;i<m_params.num_motors;i++) {
        float omega_cmd = std::sqrt(m_throttle_cmd[i]) * max_omega;
        d.dmotor_omega[i] = (omega_cmd - s.motor_omega[i]) / m_params.tau_motor;
    }

    // Thrust and torque
    float thrust_total = 0.0f;
    float torque_body[3]{};
    float R[9]; quat_to_dcm(s.quaternion, R);

    for(int i=0;i<m_params.num_motors;i++) {
        float omega = s.motor_omega[i];
        float T = m_params.kT * omega * omega;
        float Q = m_params.kQ * omega * omega * m_params.motor_dir[i];
        thrust_total += T;

        // Moment arm: r x F (thrust is -Z body)
        float r[3]={m_params.motor_pos[i][0],m_params.motor_pos[i][1],m_params.motor_pos[i][2]};
        float F_body[3]={0,0,-T};
        float M[3]; cross3(r, F_body, M);
        for(int j=0;j<3;j++) torque_body[j]+=M[j];
        torque_body[2] += Q; // reaction torque
    }

    // Position derivative = velocity
    for(int i=0;i<3;i++) d.dpos[i]=s.velocity[i];

    // Acceleration NED: gravity + thrust in body-Z rotated to NED
    // Thrust direction is body -Z: T_NED = R * [0,0,-T]
    float T_ned[3]={-R[2]*thrust_total, -R[5]*thrust_total, -R[8]*thrust_total};
    d.dvel[0] = T_ned[0] / m_params.mass_kg;
    d.dvel[1] = T_ned[1] / m_params.mass_kg;
    d.dvel[2] = T_ned[2] / m_params.mass_kg + kG;  // +g (NED down)

    if (m_params.drag_area_m2 > 0.0f) {
        float v_air[3];
        float sq = 0.0f;
        for (int i = 0; i < 3; ++i) {
            v_air[i] = s.velocity[i] - m_wind_ned[i];
            sq += v_air[i] * v_air[i];
        }
        const float k = 0.5f * m_params.air_density * m_params.drag_area_m2 * std::sqrt(sq) / m_params.mass_kg;
        for (int i = 0; i < 3; ++i) d.dvel[i] -= k * v_air[i];
    }

    // Ground constraint
    if (s.position[2] >= 0.0f) {
        if (d.dvel[2] > 0.0f) d.dvel[2] = 0.0f;
        if (d.dpos[2] > 0.0f) d.dpos[2] = 0.0f;
    }
    if (resting_on_ground(s)) {
        d.dvel[0] = d.dvel[1] = 0.0f;
        d.dpos[0] = d.dpos[1] = 0.0f;
    }

    // Quaternion derivative
    quat_deriv(s.quaternion, s.angular_rate, d.dquat);

    // Angular acceleration: I*alpha = torque - omega x (I*omega)
    // Simplified: use diagonal inertia
    float Ixx=m_params.inertia[0], Iyy=m_params.inertia[4], Izz=m_params.inertia[8];
    float wx=s.angular_rate[0], wy=s.angular_rate[1], wz=s.angular_rate[2];
    d.domega[0] = (torque_body[0] - (Izz-Iyy)*wy*wz) / Ixx;
    d.domega[1] = (torque_body[1] - (Ixx-Izz)*wz*wx) / Iyy;
    d.domega[2] = (torque_body[2] - (Iyy-Ixx)*wx*wy) / Izz;

    (void)dt_s;
    return d;
}

PhysicsState MultirotorDynamics::integrate(const PhysicsState& s,
                                            const Derivative& k, float h) const {
    PhysicsState out = s;
    for(int i=0;i<3;i++) {
        out.position[i]     += k.dpos[i]*h;
        out.velocity[i]     += k.dvel[i]*h;
        out.angular_rate[i] += k.domega[i]*h;
        out.quaternion[i]   += k.dquat[i]*h;
    }
    out.quaternion[3] += k.dquat[3]*h;
    for(int i=0;i<8;i++) out.motor_omega[i] += k.dmotor_omega[i]*h;

    // Normalize quaternion
    float n=0;
    for(int i=0;i<4;i++) n+=out.quaternion[i]*out.quaternion[i];
    n=std::sqrt(n); if(n>1e-6f) for(int i=0;i<4;i++) out.quaternion[i]/=n;

    // Ground constraint
    if (out.position[2] >= 0.0f) {
        out.position[2] = 0.0f;
        if (out.velocity[2] > 0.0f) out.velocity[2] = 0.0f;
    }

    return out;
}

void MultirotorDynamics::step(float dt_s) {
    // RK4 integration
    Derivative k1=compute_derivative(m_state, 0);
    PhysicsState s2=integrate(m_state,k1,dt_s/2);
    Derivative k2=compute_derivative(s2, dt_s/2);
    PhysicsState s3=integrate(m_state,k2,dt_s/2);
    Derivative k3=compute_derivative(s3, dt_s/2);
    PhysicsState s4=integrate(m_state,k3,dt_s);
    Derivative k4=compute_derivative(s4, dt_s);

    // Combine: x_new = x + dt/6*(k1+2k2+2k3+k4)
    Derivative k_combined;
    for(int i=0;i<3;i++) {
        k_combined.dpos[i]   =(k1.dpos[i]+2*k2.dpos[i]+2*k3.dpos[i]+k4.dpos[i])/6;
        k_combined.dvel[i]   =(k1.dvel[i]+2*k2.dvel[i]+2*k3.dvel[i]+k4.dvel[i])/6;
        k_combined.dquat[i]  =(k1.dquat[i]+2*k2.dquat[i]+2*k3.dquat[i]+k4.dquat[i])/6;
        k_combined.domega[i] =(k1.domega[i]+2*k2.domega[i]+2*k3.domega[i]+k4.domega[i])/6;
    }
    k_combined.dquat[3]=(k1.dquat[3]+2*k2.dquat[3]+2*k3.dquat[3]+k4.dquat[3])/6;
    for(int i=0;i<8;i++)
        k_combined.dmotor_omega[i]=(k1.dmotor_omega[i]+2*k2.dmotor_omega[i]+2*k3.dmotor_omega[i]+k4.dmotor_omega[i])/6;

    m_state = integrate(m_state, k_combined, dt_s);
    if (m_state.position[2] >= 0.0f) {
        m_state.position[2] = 0.0f;
        if (m_state.velocity[2] > 0.0f) m_state.velocity[2] = 0.0f;
        if (resting_on_ground(m_state)) m_state.velocity[0] = m_state.velocity[1] = 0.0f;
        m_state.angular_rate[0] *= 0.8f;
        m_state.angular_rate[1] *= 0.8f;
        m_state.angular_rate[2] *= 0.8f;
    }
    m_state.time_s += dt_s;

    // Calculate accelerometer specific force in body frame:
    // a_spec = R^T * (a_inertial - g_ned)
    float R[9];
    quat_to_dcm(m_state.quaternion, R);
    float a_inertial_ned[3] = {k_combined.dvel[0], k_combined.dvel[1], k_combined.dvel[2]};
    float a_spec_ned[3] = {
        a_inertial_ned[0],
        a_inertial_ned[1],
        a_inertial_ned[2] - kG
    };
    // Body frame is R^T * a_spec_ned
    m_state.accel_body[0] = R[0]*a_spec_ned[0] + R[3]*a_spec_ned[1] + R[6]*a_spec_ned[2];
    m_state.accel_body[1] = R[1]*a_spec_ned[0] + R[4]*a_spec_ned[1] + R[7]*a_spec_ned[2];
    m_state.accel_body[2] = R[2]*a_spec_ned[0] + R[5]*a_spec_ned[1] + R[8]*a_spec_ned[2];
}

} // namespace apx::sitl
