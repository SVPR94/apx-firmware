// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <chrono>

#include <cstdint>
#include <thread>
#include <atomic>
#include <mutex>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace apx {
namespace sitl {

struct SitlPhysicsState {
    uint64_t timestamp_us;
    double position_ned[3];
    double velocity_ned[3];
    double accel_body[3];
    double gyro_body[3];
    double euler_angles[3];
    double quaternion[4];
    double airspeed_ms;
    double baro_pressure_pa;
    double baro_temp_c;
    double gps_lat;
    double gps_lon;
    double gps_alt;
    uint8_t gps_fix;
    double wind_vel_ned[3];
    // Multi-directional radar & terrain-following simulation (10 channels)
    // [0]=Down, [1]=Front, [2]=Front-Right, [3]=Right, [4]=Rear-Right,
    // [5]=Rear, [6]=Rear-Left, [7]=Left, [8]=Front-Left, [9]=Top
    float radar_distances[10];
    uint16_t radar_valid_mask;
} __attribute__((packed));

struct SitlActuatorOutput {
    uint64_t timestamp_us;
    float motor_throttle[8];
    uint16_t servo_pwm[16];
} __attribute__((packed));

class SitlServer {
public:
    SitlServer();
    ~SitlServer();

    bool init(uint16_t udp_port = 9003, uint16_t tcp_port = 9002);
    void start();
    void stop();

    bool get_latest_physics_state(SitlPhysicsState& state);
    void send_actuator_output(const SitlActuatorOutput& output);

private:
    void udp_listen_loop();
    
    int m_udp_socket{-1};
    uint16_t m_udp_port;
    
    std::thread m_listen_thread;
    std::atomic<bool> m_running{false};

    std::mutex m_state_mutex;
    SitlPhysicsState m_latest_state{};
    
    struct sockaddr_in m_client_addr{};
    bool m_client_connected{false};
    // Physics peer latch: packets from any other sender are dropped while
    // the latched one is alive, so a second simulator on the same port
    // (e.g. an orphaned builtin runner next to Gazebo) cannot interleave
    // its vehicle with ours. Re-latches after kPeerTimeoutMs of silence.
    std::chrono::steady_clock::time_point m_last_peer_rx{};
    std::chrono::steady_clock::time_point m_last_foreign_warn{};
    uint64_t m_foreign_dropped{0};
};

} // namespace sitl
} // namespace apx
