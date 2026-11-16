// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "multirotor_dynamics.hpp"
#include "wind_model.hpp"
#include "../sitl_bridge/sitl_server.hpp"

namespace {
volatile bool g_running = true;
void sig_handler(int) { g_running = false; }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    uint16_t target_port = 9003;
    uint16_t listen_port = 9004;
    double home_lat = 18.5204;
    double home_lon = 73.8567;
    float home_alt = 540.0f;
    std::string vehicle = "quad_x";
    uint32_t seed = 1;
    apx::sitl::WindConfig wind_cfg{};

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--port=", 0) == 0) target_port = static_cast<uint16_t>(std::atoi(arg.substr(7).c_str()));
        else if (arg == "--port" && i + 1 < argc) target_port = static_cast<uint16_t>(std::atoi(argv[++i]));
        else if (arg.rfind("--listen-port=", 0) == 0) listen_port = static_cast<uint16_t>(std::atoi(arg.substr(14).c_str()));
        else if (arg == "--listen-port" && i + 1 < argc) listen_port = static_cast<uint16_t>(std::atoi(argv[++i]));
        else if (arg.rfind("--vehicle=", 0) == 0) vehicle = arg.substr(10);
        else if (arg == "--vehicle" && i + 1 < argc) vehicle = argv[++i];
        else if (arg.rfind("--lat=", 0) == 0) home_lat = std::atof(arg.substr(6).c_str());
        else if (arg == "--lat" && i + 1 < argc) home_lat = std::atof(argv[++i]);
        else if (arg.rfind("--lon=", 0) == 0) home_lon = std::atof(arg.substr(6).c_str());
        else if (arg == "--lon" && i + 1 < argc) home_lon = std::atof(argv[++i]);
        else if (arg.rfind("--alt=", 0) == 0) home_alt = static_cast<float>(std::atof(arg.substr(6).c_str()));
        else if (arg == "--alt" && i + 1 < argc) home_alt = static_cast<float>(std::atof(argv[++i]));
        else if (arg.rfind("--seed=", 0) == 0) seed = static_cast<uint32_t>(std::strtoul(arg.substr(7).c_str(), nullptr, 10));
        else if (arg == "--seed" && i + 1 < argc) seed = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
        else if (arg.rfind("--wind=", 0) == 0) {
            if (!apx::sitl::parse_wind_spec(arg.substr(7), wind_cfg)) {
                std::fprintf(stderr, "[APX Sim Physics] Bad --wind '%s' (expected SPEED,FROM_DEG[,GUST_SIGMA])\n",
                             arg.substr(7).c_str());
                return 2;
            }
        } else if (arg == "--wind" && i + 1 < argc) {
            std::string val = argv[++i];
            if (!apx::sitl::parse_wind_spec(val, wind_cfg)) {
                std::fprintf(stderr, "[APX Sim Physics] Bad --wind '%s' (expected SPEED,FROM_DEG[,GUST_SIGMA])\n",
                             val.c_str());
                return 2;
            }
        }
    }

    std::printf("[APX Sim Physics] Starting 6DOF RK4 multirotor engine for '%s'\n", vehicle.c_str());
    std::printf("[APX Sim Physics] Target APX SITL port: UDP %u, Local listener: UDP %u\n", target_port, listen_port);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("[APX Sim Physics] Failed to create socket");
        return 1;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    // No SO_REUSEPORT: with it, a second simulator instance silently shared
    // the listen port and the kernel handed each APX's motor commands to the
    // other instance's physics (the vehicle then climbed away at full
    // thrust). On a clash the ephemeral-port fallback below is used instead.

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(listen_port);
    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        // Fallback to ephemeral port if listen_port is taken
        local_addr.sin_port = 0;
        bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr));
    }

    // Set non-blocking
    fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in target_addr{};
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(target_port);
    inet_pton(AF_INET, "127.0.0.1", &target_addr.sin_addr);

    apx::sitl::MultirotorParams params;
    // drag_area_m2 (Cd * frontal area) values are estimates for these
    // generic airframes, not measurements.
    if (vehicle == "hex_x") {
        params = apx::sitl::MultirotorParams::make_hex_x(2.8f, 0.28f);
        params.drag_area_m2 = 0.05f;
    } else if (vehicle == "octo_x") {
        params = apx::sitl::MultirotorParams::make_octo_x(3.8f, 0.33f);
        params.drag_area_m2 = 0.07f;
    } else {
        params = apx::sitl::MultirotorParams::make_quad_x(2.0f, 0.225f);
        params.drag_area_m2 = 0.04f;
    }
    // Quad rotor parameters: T:W ~ 2.2:1 at max throttle
    // At hover (~0.48 throttle): T ~ weight = 2.0 * 9.81 = 19.62 N
    // max thrust = 19.62 / (4 * 0.48) ~ 10.2 N per motor
    params.max_rpm = 8500.0f;
    float max_omega = params.max_rpm * 3.14159f / 30.0f;
    params.kT = 10.2f / (max_omega * max_omega);
    params.kQ = 0.02f * params.kT;

    apx::sitl::MultirotorDynamics dynamics(params);
    apx::sitl::WindModel wind;
    wind.init(wind_cfg, seed);
    std::printf("[APX Sim Physics] Wind: %.1f m/s from %.0f deg, gust sigma %.1f m/s (seed %u)\n",
                wind_cfg.speed_ms, wind_cfg.from_deg, wind_cfg.gust_sigma_ms, seed);

    constexpr double kEarthRadiusM = 6378137.0;
    constexpr float dt_s = 0.0025f; // 400 Hz physics step

    auto start_time = std::chrono::steady_clock::now();
    const auto step_period = std::chrono::microseconds(2500);
    auto next_step = start_time + step_period;

    while (g_running) {
        auto step_start = std::chrono::steady_clock::now();

        // 1. Receive actuator commands from APX. Drain the socket and keep
        // only the newest packet: reading one per step let a queue build up
        // whenever APX ran even slightly faster than this loop, adding
        // seconds of actuator latency until the vehicle flipped.
        apx::sitl::SitlActuatorOutput act{};
        sockaddr_in from_addr{};
        socklen_t from_len = sizeof(from_addr);
        bool have_act = false;
        for (;;) {
            apx::sitl::SitlActuatorOutput pkt{};
            ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0, (struct sockaddr*)&from_addr, &from_len);
            if (n < 0) {
                break; // EAGAIN: queue empty (socket is non-blocking)
            }
            if (n == sizeof(pkt)) {
                act = pkt;
                have_act = true;
            }
        }
        if (have_act) {
            for (int i = 0; i < params.num_motors; ++i) {
                dynamics.set_motor_throttle(i, act.motor_throttle[i]);
            }
        }

        // 2. Step 6DOF RK4 physics
        dynamics.set_wind_ned(wind.update(dt_s));
        dynamics.step(dt_s);
        const auto& s = dynamics.get_state();

        // 3. Pack physics state
        apx::sitl::SitlPhysicsState phys{};
        phys.timestamp_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(step_start - start_time).count()
        );

        for (int i = 0; i < 3; ++i) {
            phys.position_ned[i] = s.position[i];
            phys.velocity_ned[i] = s.velocity[i];
            phys.gyro_body[i] = s.angular_rate[i];
            phys.accel_body[i] = s.accel_body[i];
        }

        for (int i = 0; i < 4; ++i) {
            phys.quaternion[i] = s.quaternion[i];
        }

        // Compute Euler RPY from quaternion (ZYX)
        float q0 = s.quaternion[0], q1 = s.quaternion[1], q2 = s.quaternion[2], q3 = s.quaternion[3];
        phys.euler_angles[0] = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        phys.euler_angles[1] = std::asin(std::clamp(sinp, -1.0f, 1.0f));
        phys.euler_angles[2] = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

        phys.airspeed_ms = dynamics.get_airspeed();

        phys.gps_lat = home_lat + (s.position[0] / kEarthRadiusM) * (180.0 / M_PI);
        phys.gps_lon = home_lon + (s.position[1] / (kEarthRadiusM * std::cos(home_lat * M_PI / 180.0))) * (180.0 / M_PI);
        phys.gps_alt = home_alt - s.position[2];
        phys.gps_fix = 3;
        phys.baro_pressure_pa = 101325.0 * std::pow(1.0 - (phys.gps_alt / 44330.0), 5.255);
        phys.baro_temp_c = 15.0 - 0.0065 * phys.gps_alt;
        const float* w = wind.ned();
        phys.wind_vel_ned[0] = w[0];
        phys.wind_vel_ned[1] = w[1];
        phys.wind_vel_ned[2] = w[2];

        // Multi-radar simulation (10 channels)
        // 0: Down (Z=+1 in FRD), 1: Front (X=+1), 2: Front-Right, 3: Right (Y=+1),
        // 4: Rear-Right, 5: Rear (X=-1), 6: Rear-Left, 7: Left (Y=-1),
        // 8: Front-Left, 9: Top (Z=-1 in FRD)
        static const float kRadarDirsFrd[10][3] = {
            { 0.0f,  0.0f,  1.0f}, // Down
            { 1.0f,  0.0f,  0.0f}, // Front
            { 0.70710678f,  0.70710678f,  0.0f}, // Front-Right
            { 0.0f,  1.0f,  0.0f}, // Right
            {-0.70710678f,  0.70710678f,  0.0f}, // Rear-Right
            {-1.0f,  0.0f,  0.0f}, // Rear
            {-0.70710678f, -0.70710678f,  0.0f}, // Rear-Left
            { 0.0f, -1.0f,  0.0f}, // Left
            { 0.70710678f, -0.70710678f,  0.0f}, // Front-Left
            { 0.0f,  0.0f, -1.0f}  // Top
        };

        phys.radar_valid_mask = 0;
        const float pos_z = static_cast<float>(s.position[2]); // NED Z (down is positive, ground is 0)
        for (int r = 0; r < 10; ++r) {
            const float bx = kRadarDirsFrd[r][0];
            const float by = kRadarDirsFrd[r][1];
            const float bz = kRadarDirsFrd[r][2];
            // Rotation matrix 3rd row (Z-component in NED):
            // R31 = 2*(q1*q3 - q0*q2), R32 = 2*(q2*q3 + q0*q1), R33 = 1 - 2*(q1*q1 + q2*q2)
            const float r31 = 2.0f * (q1 * q3 - q0 * q2);
            const float r32 = 2.0f * (q2 * q3 + q0 * q1);
            const float r33 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
            const float dir_z = r31 * bx + r32 * by + r33 * bz;

            // Ground is at NED Z = 0. When airborne, pos_z < 0.
            // pos_z + dist * dir_z = 0  ==>  dist = -pos_z / dir_z.
            if (dir_z > 1e-4f) {
                float dist = (pos_z < 0.0f) ? (-pos_z / dir_z) : 0.05f;
                if (dist >= 0.02f && dist <= 40.0f) {
                    phys.radar_distances[r] = dist;
                    phys.radar_valid_mask |= (1 << r);
                } else {
                    phys.radar_distances[r] = 40.0f;
                }
            } else {
                phys.radar_distances[r] = 40.0f;
            }
        }

        // 4. Send physics state to APX SITL
        sendto(sock, &phys, sizeof(phys), 0, (struct sockaddr*)&target_addr, sizeof(target_addr));

        // Hold 400 Hz against an absolute deadline (sleep overshoot used to
        // make this loop run ~2-4% slow); resync after a long stall.
        const auto now = std::chrono::steady_clock::now();
        if (next_step > now) {
            std::this_thread::sleep_until(next_step);
        } else if (now - next_step > 4 * step_period) {
            next_step = now;
        }
        next_step += step_period;
    }

    close(sock);
    std::printf("[APX Sim Physics] Simulator cleanly stopped.\n");
    return 0;
}
