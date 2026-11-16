// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "mavlink_handler.hpp"
#include "param_manager.hpp"
#include "fence.hpp"
#include "radar_manager.hpp"
#include "apx/kernel/timer.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>
#if !defined(APX_EMBEDDED)
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#endif
#include <unistd.h>
#include <errno.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace apx::gcs {

namespace {
// HEARTBEAT autopilot type 3: ground stations use it to pick the parameter
// and flight-mode schema APX follows (MAVLink MAV_AUTOPILOT enum, value 3).
constexpr uint8_t kHeartbeatAutopilotType = 3;
} // namespace

MavlinkHandler::MavlinkHandler(uint8_t sysid, uint8_t compid)
    : m_sysid(sysid), m_compid(compid) {
    m_command_handler = std::make_unique<CommandHandler>(this);
    m_start_time_ms = get_time_ms();
}

MavlinkHandler::~MavlinkHandler() {
    stop_network();
}

void MavlinkHandler::init() {
    ParamManager::get_instance().init_defaults();
}

uint32_t MavlinkHandler::get_time_ms() {
#if defined(APX_EMBEDDED)
    // No usable std::chrono clock without a hosted OS (newlib's
    // gettimeofday stub always fails, freezing every periodic stream).
    return static_cast<uint32_t>(apx::kernel::now_ms());
#else
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
#endif
}

#if !defined(APX_EMBEDDED)
bool MavlinkHandler::start_network(uint16_t udp_port, const std::vector<uint16_t>& tcp_ports) {
    if (m_network_running) {
        return true;
    }

    m_udp_port = udp_port;
    m_tcp_ports = tcp_ports;

    // 1. Setup UDP socket
    m_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udp_sock >= 0) {
        int opt = 1;
        setsockopt(m_udp_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
        setsockopt(m_udp_sock, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif
        setsockopt(m_udp_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
        fcntl(m_udp_sock, F_SETFL, fcntl(m_udp_sock, F_GETFL, 0) | O_NONBLOCK);

        uint16_t local_bind_port = (m_udp_port == 14550) ? 14551 : static_cast<uint16_t>(m_udp_port + 1);
        sockaddr_in udp_bind{};
        udp_bind.sin_family = AF_INET;
        udp_bind.sin_addr.s_addr = INADDR_ANY;
        udp_bind.sin_port = htons(local_bind_port);

        if (bind(m_udp_sock, (struct sockaddr*)&udp_bind, sizeof(udp_bind)) == 0) {
            std::printf("[APX Telemetry] Bound local UDP socket on 0.0.0.0:%u (streaming to 127.0.0.1:%u)\n",
                        local_bind_port, m_udp_port);
        } else {
            std::printf("[APX Telemetry] Warning: UDP bind on port %u returned: %s\n",
                        local_bind_port, strerror(errno));
        }

        m_udp_target_addr.sin_family = AF_INET;
        m_udp_target_addr.sin_port = htons(m_udp_port);
        inet_pton(AF_INET, "127.0.0.1", &m_udp_target_addr.sin_addr);
    } else {
        std::fprintf(stderr, "[APX Telemetry] Failed to create UDP socket: %s\n", strerror(errno));
    }

    // 2. Setup TCP server sockets for each requested port (e.g. 5760, 5761, 5762, 5763)
    for (uint16_t port : m_tcp_ports) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            std::fprintf(stderr, "[APX Telemetry] Failed to create TCP socket for port %u: %s\n", port, strerror(errno));
            continue;
        }

        int opt = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
        setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

        sockaddr_in sin{};
        sin.sin_family = AF_INET;
        sin.sin_addr.s_addr = INADDR_ANY;
        sin.sin_port = htons(port);

        if (bind(fd, (struct sockaddr*)&sin, sizeof(sin)) == 0 && listen(fd, 5) == 0) {
            m_tcp_server_fds.push_back(fd);
            std::printf("[APX Telemetry] TCP server listening on 0.0.0.0:%u\n", port);
        } else {
            std::fprintf(stderr, "[APX Telemetry] Failed to bind/listen TCP port %u: %s\n", port, strerror(errno));
            close(fd);
        }
    }

    // 3. Start background network thread
    m_network_running = true;
    m_network_thread = std::thread(&MavlinkHandler::network_io_loop, this);

    std::printf("[APX Telemetry] Telemetry service active on UDP:%u and %zu TCP endpoints.\n",
                m_udp_port, m_tcp_server_fds.size());
    return true;
}

void MavlinkHandler::stop_network() {
    if (!m_network_running) return;
    m_network_running = false;

    if (m_network_thread.joinable()) {
        m_network_thread.join();
    }

    LockGuardType lock(m_network_mutex);
    for (int fd : m_client_fds) {
        close(fd);
    }
    m_client_fds.clear();

    for (int fd : m_tcp_server_fds) {
        close(fd);
    }
    m_tcp_server_fds.clear();

    if (m_udp_sock >= 0) {
        close(m_udp_sock);
        m_udp_sock = -1;
    }
}

#endif // !APX_EMBEDDED (start/stop_network)

void MavlinkHandler::set_vehicle_state(float roll, float pitch, float yaw,
                                       double lat, double lon, float alt, float rel_alt,
                                       float vx, float vy, float vz,
                                       float battery_v, float battery_a, int8_t battery_remaining,
                                       bool armed, uint8_t base_mode, uint32_t custom_mode) {
    LockGuardType lock(m_state_mutex);
    m_state.roll = roll;
    m_state.pitch = pitch;
    m_state.yaw = yaw;
    m_state.lat = lat;
    m_state.lon = lon;
    m_state.alt_msl = alt;
    m_state.rel_alt = rel_alt;
    m_state.vx = vx;
    m_state.vy = vy;
    m_state.vz = vz;
    m_state.groundspeed = std::sqrt(vx * vx + vy * vy);
    m_state.airspeed = m_state.groundspeed;
    m_state.battery_voltage = battery_v;
    m_state.battery_current = battery_a;
    m_state.battery_remaining = battery_remaining;
    m_state.armed = armed;
    m_state.base_mode = base_mode | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
    m_state.custom_mode = custom_mode;
    m_state.system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
}

void MavlinkHandler::set_armed(bool armed) {
    bool was_armed = false;
    double cur_lat = 0.0;
    double cur_lon = 0.0;
    float cur_alt = 0.0f;
    {
        LockGuardType lock(m_state_mutex);
        was_armed = m_state.armed;
        m_state.armed = armed;
        m_state.base_mode = (m_state.base_mode & ~static_cast<uint8_t>(MAV_MODE_FLAG_SAFETY_ARMED))
                           | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
        m_state.system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
        cur_lat = m_state.lat;
        cur_lon = m_state.lon;
        cur_alt = m_state.alt_msl;
    }

    // Home is where the vehicle takes off: re-set it on arming to the
    // current location (RTL and relative altitudes use it) unless explicitly locked by GCS.
    if (!was_armed && armed) {
        if (!m_home_locked) {
            if (cur_lat != 0.0 || cur_lon != 0.0) {
                set_home_position(cur_lat, cur_lon, cur_alt, false);
                send_statustext(MAV_SEVERITY_INFO, "Home set to arming location");
            } else if (is_home_set()) {
                LockGuardType lock(m_vehicle_state_mutex);
                constexpr double DEG_PER_M_LAT = 1.0 / 111319.5;
                double cur_home_lat = home_lat();
                double cur_home_lon = home_lon();
                float cur_home_alt = home_alt_msl();
                double deg_per_m_lon = DEG_PER_M_LAT / std::cos(cur_home_lat * 0.017453292519943295);
                double new_lat = cur_home_lat + m_last_known_vehicle_state.pos_ned[0] * DEG_PER_M_LAT;
                double new_lon = cur_home_lon + m_last_known_vehicle_state.pos_ned[1] * deg_per_m_lon;
                float new_alt = cur_home_alt - m_last_known_vehicle_state.pos_ned[2];
                set_home_position(new_lat, new_lon, new_alt, false);
                send_statustext(MAV_SEVERITY_INFO, "Home set to arming location");
            }
        }
    } else if (was_armed && !armed) {
        // Reset home upon landing to touchdown location unless explicitly locked by GCS
        if (!m_home_locked) {
            if (cur_lat != 0.0 || cur_lon != 0.0) {
                set_home_position(cur_lat, cur_lon, cur_alt, false);
                send_statustext(MAV_SEVERITY_INFO, "Home reset to landing location");
            } else if (is_home_set()) {
                LockGuardType lock(m_vehicle_state_mutex);
                constexpr double DEG_PER_M_LAT = 1.0 / 111319.5;
                double cur_home_lat = home_lat();
                double cur_home_lon = home_lon();
                float cur_home_alt = home_alt_msl();
                double deg_per_m_lon = DEG_PER_M_LAT / std::cos(cur_home_lat * 0.017453292519943295);
                double new_lat = cur_home_lat + m_last_known_vehicle_state.pos_ned[0] * DEG_PER_M_LAT;
                double new_lon = cur_home_lon + m_last_known_vehicle_state.pos_ned[1] * deg_per_m_lon;
                float new_alt = cur_home_alt - m_last_known_vehicle_state.pos_ned[2];
                set_home_position(new_lat, new_lon, new_alt, false);
                send_statustext(MAV_SEVERITY_INFO, "Home reset to landing location");
            }
        }
    }
}

VehicleTelemetryState MavlinkHandler::get_vehicle_telemetry_state() const {
    LockGuardType lock(m_state_mutex);
    return m_state;
}

void MavlinkHandler::set_custom_mode(uint32_t custom_mode) {
    LockGuardType lock(m_state_mutex);
    m_state.custom_mode = custom_mode;
    m_state.base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
}

void MavlinkHandler::set_motor_outputs(const float outputs[8]) {
    LockGuardType lock(m_state_mutex);
    for (int i = 0; i < 8; ++i) {
        float t = std::clamp(outputs[i], 0.0f, 1.0f);
        uint16_t pwm = static_cast<uint16_t>(1000.0f + t * 1000.0f);
        m_state.motor_pwm[i] = pwm;
        m_state.servo_pwm[i] = pwm;
    }
}

void MavlinkHandler::set_raw_sensor_telemetry(const RawSensorTelemetry& raw) {
    LockGuardType lock(m_state_mutex);
    m_raw_sensors = raw;
    m_have_raw_sensors = true;
}

void MavlinkHandler::set_servo_outputs(const uint16_t pwm[16]) {
    LockGuardType lock(m_state_mutex);
    for (int i = 0; i < 16; ++i) {
        m_state.servo_pwm[i] = pwm[i];
    }
}

void MavlinkHandler::set_aux_servo_pwm(uint8_t channel_1_based, uint16_t pwm) {
    LockGuardType lock(m_state_mutex);
    if (channel_1_based >= 1 && channel_1_based <= 16) {
        m_state.servo_pwm[channel_1_based - 1] = pwm;
    }
}

uint16_t MavlinkHandler::get_aux_servo_pwm(uint8_t channel_1_based) const {
    LockGuardType lock(m_state_mutex);
    if (channel_1_based >= 1 && channel_1_based <= 16) {
        return m_state.servo_pwm[channel_1_based - 1];
    }
    return 1500;
}

bool MavlinkHandler::get_armed() const {
    LockGuardType lock(m_state_mutex);
    return m_state.armed;
}

uint32_t MavlinkHandler::get_custom_mode() const {
    LockGuardType lock(m_state_mutex);
    return m_state.custom_mode;
}

void MavlinkHandler::set_home_position(double lat, double lon, float alt_msl, bool locked) {
    {
        LockGuardType lock(m_state_mutex);
        m_home_lat = lat;
        m_home_lon = lon;
        m_home_alt_msl = alt_msl;
        m_home_set = true;
        if (locked) {
            m_home_locked = true;
        }
        if (!m_origin_set) {
            m_origin_lat = lat;
            m_origin_lon = lon;
            m_origin_alt_msl = alt_msl;
            m_origin_set = true;
        }
    }
    send_home_position();
}

void MavlinkHandler::global_to_ned(double lat, double lon, float rel_alt_m, float ned[3]) const {
    LockGuardType lock(m_state_mutex);
    constexpr double kEarthRadiusM = 6378137.0;
    const double dlat = (lat - m_origin_lat) * (M_PI / 180.0);
    const double dlon = (lon - m_origin_lon) * (M_PI / 180.0);
    ned[0] = static_cast<float>(dlat * kEarthRadiusM);
    ned[1] = static_cast<float>(dlon * kEarthRadiusM * std::cos(m_origin_lat * M_PI / 180.0));
    ned[2] = -(rel_alt_m + (m_home_alt_msl - m_origin_alt_msl));
}

void MavlinkHandler::home_ned(float ned[3]) const {
    double lat, lon;
    {
        LockGuardType lock(m_state_mutex);
        lat = m_home_lat;
        lon = m_home_lon;
    }
    global_to_ned(lat, lon, 0.0f, ned);
}

void MavlinkHandler::send_home_position() {
    double h_lat = 0.0;
    double h_lon = 0.0;
    float h_alt = 0.0f;
    float h_ned[3] = {0.0f, 0.0f, 0.0f};
    {
        LockGuardType lock(m_state_mutex);
        if (!m_home_set) return;
        h_lat = m_home_lat;
        h_lon = m_home_lon;
        h_alt = m_home_alt_msl;
    }
    home_ned(h_ned);
    mavlink_message_t msg;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    uint64_t time_usec = static_cast<uint64_t>(get_time_ms()) * 1000ULL;
    mavlink_msg_home_position_pack(
        m_sysid, m_compid, &msg,
        static_cast<int32_t>(h_lat * 1e7),
        static_cast<int32_t>(h_lon * 1e7),
        static_cast<int32_t>(h_alt * 1000.0f),
        h_ned[0], h_ned[1], h_ned[2],
        q,
        0.0f, 0.0f, 0.0f,
        time_usec
    );
    send_message(msg);
}

void MavlinkHandler::set_last_known_vehicle_state(const apx::modes::VehicleState& state) {
    {
        LockGuardType lock(m_vehicle_state_mutex);
        m_last_known_vehicle_state = state;
    }
    // ATTITUDE body rates
    LockGuardType lock(m_state_mutex);
    m_state.rollspeed = state.gyro_body[0];
    m_state.pitchspeed = state.gyro_body[1];
    m_state.yawspeed = state.gyro_body[2];
}

apx::modes::VehicleState MavlinkHandler::get_last_known_vehicle_state() const {
    LockGuardType lock(m_vehicle_state_mutex);
    return m_last_known_vehicle_state;
}

bool MavlinkHandler::start_motor_test(uint8_t motor_index, uint8_t throttle_type, float throttle_val, float timeout_sec) {
    if (motor_index < 1 || motor_index > 8) {
        return false;
    }
    float throttle = 0.0f;
    if (throttle_type == 0) { // MOTOR_TEST_THROTTLE_PERCENT (0..100)
        throttle = std::clamp(throttle_val / 100.0f, 0.0f, 1.0f);
    } else if (throttle_type == 1) { // MOTOR_TEST_THROTTLE_PWM (1000..2000)
        throttle = std::clamp((throttle_val - 1000.0f) / 1000.0f, 0.0f, 1.0f);
    } else {
        throttle = std::clamp(throttle_val / 100.0f, 0.0f, 1.0f);
    }

    if (timeout_sec <= 0.0f || throttle <= 0.0f) {
        stop_motor_test();
        return true;
    }

    uint32_t now = get_time_ms();
    uint32_t duration_ms = static_cast<uint32_t>(std::clamp(timeout_sec, 0.1f, 10.0f) * 1000.0f);
    {
        LockGuardType lock(m_motor_test_mutex);
        m_motor_test.motor_index = motor_index;
        m_motor_test.throttle = throttle;
        m_motor_test.end_time_ms = now + duration_ms;
        m_motor_test.active = true;
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Motor %u test: %d%% for %.1fs", motor_index, static_cast<int>(throttle * 100.0f), timeout_sec);
    send_statustext(MAV_SEVERITY_INFO, buf);
    return true;
}

void MavlinkHandler::stop_motor_test() {
    bool was_active = false;
    {
        LockGuardType lock(m_motor_test_mutex);
        was_active = m_motor_test.active;
        m_motor_test.active = false;
        m_motor_test.throttle = 0.0f;
        m_motor_test.motor_index = 0;
    }
    if (was_active) {
        send_statustext(MAV_SEVERITY_INFO, "Motor test stopped");
    }
}

bool MavlinkHandler::is_motor_test_active() const {
    LockGuardType lock(m_motor_test_mutex);
    return m_motor_test.active;
}

uint8_t MavlinkHandler::get_motor_test_index() const {
    LockGuardType lock(m_motor_test_mutex);
    return m_motor_test.motor_index;
}

float MavlinkHandler::get_motor_test_throttle() const {
    LockGuardType lock(m_motor_test_mutex);
    return m_motor_test.throttle;
}

#if !defined(APX_EMBEDDED) // hosted socket I/O thread

void MavlinkHandler::network_io_loop() {
    while (m_network_running) {
        std::vector<struct pollfd> poll_fds;

        // Poll UDP socket
        if (m_udp_sock >= 0) {
            poll_fds.push_back({m_udp_sock, POLLIN, 0});
        }

        // Poll TCP listening sockets
        for (int sfd : m_tcp_server_fds) {
            poll_fds.push_back({sfd, POLLIN, 0});
        }

        // Poll connected TCP clients
        {
            LockGuardType lock(m_network_mutex);
            for (int cfd : m_client_fds) {
                poll_fds.push_back({cfd, POLLIN, 0});
            }
        }

        int ret = poll(poll_fds.data(), poll_fds.size(), 10); // 10ms timeout
        if (ret > 0) {
            size_t idx = 0;

            // Check UDP
            if (m_udp_sock >= 0) {
                if (poll_fds[idx].revents & POLLIN) {
                    uint8_t rx_buf[2048];
                    sockaddr_in client_addr{};
                    socklen_t client_len = sizeof(client_addr);
                    ssize_t n = recvfrom(m_udp_sock, rx_buf, sizeof(rx_buf), 0,
                                         (struct sockaddr*)&client_addr, &client_len);
                    if (n > 0) {
                        char ip_str[INET_ADDRSTRLEN] = {0};
                        inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
                        uint16_t remote_port = ntohs(client_addr.sin_port);
                        std::string rem_str = std::string(ip_str) + ":" + std::to_string(remote_port);

                        // NOTE: this lock must NOT still be held when we call
                        // process_incoming_byte() below. Handling an incoming
                        // message (e.g. PARAM_REQUEST_LIST) can synchronously
                        // send a reply via send_message()->write_bytes(),
                        // which itself locks m_network_mutex. Holding the
                        // lock across that call self-deadlocks this thread
                        // (std::mutex is not recursive) the moment any GCS
                        // request needs a response - which looked exactly
                        // like "GCS connects but nothing else ever works".
                        // Some GCS clients (confirmed with APX Flight Manager's
                        // UDP auto-discovery) send their heartbeat FROM the
                        // same port they're listening on, 14550 - meaning
                        // the "remote" we just saw traffic from is
                        // bit-for-bit the same address write_bytes() ALREADY
                        // broadcasts to unconditionally as the hardcoded
                        // default target (m_udp_target_addr). Registering
                        // that address into m_udp_remotes too meant every
                        // outgoing message got sent to it TWICE - once via
                        // the unconditional default-target write, once via
                        // the "all registered remotes" loop - which GCS's
                        // message log showed as perfectly duplicated
                        // notices (confirmed live: identical text AND
                        // millisecond timestamp, back to back).
                        bool is_default_target =
                            client_addr.sin_addr.s_addr == m_udp_target_addr.sin_addr.s_addr &&
                            client_addr.sin_port == m_udp_target_addr.sin_port;

                        bool found = false;
                        bool newly_registered = false;
                        {
                            LockGuardType lock(m_network_mutex);
                            for (auto& ep : m_active_endpoints) {
                                if (ep.fd == -1 && ep.remote_addr == rem_str) {
                                    ep.last_seen_ms = get_time_ms();
                                    found = true;
                                    break;
                                }
                            }
                            if (!found && client_len > 0) {
                                GcsEndpointInfo ep{};
                                ep.fd = -1;
                                ep.interface_name = "UDP " + std::to_string(m_udp_port);
                                ep.remote_addr = rem_str;
                                ep.port = m_udp_port;
                                ep.last_seen_ms = get_time_ms();
                                ep.is_active = true;
                                m_active_endpoints.push_back(ep);
                                if (!is_default_target) {
                                    m_udp_remotes.push_back(client_addr);
                                }
                                newly_registered = true;
                            }
                        } // lock released before any reply is sent

                        if (newly_registered) {
                            std::printf("[GCS CONNECTED] Ground Control Station packet detected on UDP %u from %s\n",
                                        m_udp_port, rem_str.c_str());
                            // No send_statustext() here anymore - see the
                            // comment on the TCP accept path below for why
                            // broadcasting this to every connected client
                            // was the actual complaint, not just the
                            // double-send bug fixed earlier.
                        }

                        for (ssize_t i = 0; i < n; ++i) {
                            process_incoming_byte(rx_buf[i], kUdpChannel);
                        }
                    }
                }
                idx++;
            }

            // Check TCP server listening sockets (accept new connections)
            for (size_t s = 0; s < m_tcp_server_fds.size(); ++s) {
                if (poll_fds[idx].revents & POLLIN) {
                    sockaddr_in client_addr{};
                    socklen_t client_len = sizeof(client_addr);
                    int new_client = accept(m_tcp_server_fds[s], (struct sockaddr*)&client_addr, &client_len);
                    if (new_client >= 0) {
                        fcntl(new_client, F_SETFL, fcntl(new_client, F_GETFL, 0) | O_NONBLOCK);
                        char client_ip[INET_ADDRSTRLEN] = {0};
                        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
                        uint16_t client_port = ntohs(client_addr.sin_port);
                        uint16_t srv_port = (s < m_tcp_ports.size()) ? m_tcp_ports[s] : 5760;

                        GcsEndpointInfo ep{};
                        ep.fd = new_client;
                        ep.interface_name = "TCP " + std::to_string(srv_port);
                        ep.remote_addr = std::string(client_ip) + ":" + std::to_string(client_port);
                        ep.port = srv_port;
                        ep.last_seen_ms = get_time_ms();
                        ep.is_active = true;

                        {
                            LockGuardType lock(m_network_mutex);
                            m_client_fds.push_back(new_client);
                            m_active_endpoints.push_back(ep);
                        }

                        std::printf("[GCS CONNECTED] Ground Control Station connected on %s from %s (client fd %d)\n",
                                    ep.interface_name.c_str(), ep.remote_addr.c_str(), new_client);

                        // Greet client immediately
                        send_heartbeat();
                        send_autopilot_version();
                        // Deliberately no send_statustext("GCS connected...")
                        // broadcast here (write_bytes() sends every message
                        // to ALL currently connected endpoints, not just
                        // this one) - confirmed live: sim_vehicle.py's own
                        // cosmetic startup port-verification banner
                        // (verify_telemetry_ports() in sitl/sim_vehicle.py)
                        // opens and immediately closes a real TCP connection
                        // to each of 5760-5763 just to print "[OK] OPEN &
                        // LISTENING" - every one of those was a real accept()
                        // from this server's point of view, so every one
                        // broadcast a connect/disconnect notice to every
                        // *other* already-connected client (e.g. GCS on
                        // UDP), flooding its message log with churn about
                        // ports it never used and a client (the port-check
                        // probe) it never knew existed. This connect/
                        // disconnect bookkeeping is genuinely only useful
                        // server-side (the printf above, already visible in
                        // this process's own console) - a real GCS operator
                        // has no use for being told some *other* channel's
                        // client came and went, probe or not.
                    }
                }
                idx++;
            }

            // Check connected clients
            std::vector<int> dead_clients;
            // fd -> received bytes; processed after m_network_mutex is
            // released (process_incoming_byte() can trigger a reply that
            // locks m_network_mutex again - see the UDP branch above for
            // why that must never happen while we're still holding it).
            std::vector<std::pair<int, std::vector<uint8_t>>> pending_rx;
            struct DeadInfo { int fd; std::string if_name; std::string r_addr; };
            std::vector<DeadInfo> dead_info;
            {
                LockGuardType lock(m_network_mutex);
                for (size_t c = 0; c < m_client_fds.size(); ++c) {
                    if (idx < poll_fds.size()) {
                        if (poll_fds[idx].revents & (POLLIN | POLLERR | POLLHUP)) {
                            uint8_t rx_buf[1024];
                            ssize_t n = recv(m_client_fds[c], rx_buf, sizeof(rx_buf), 0);
                            if (n > 0) {
                                for (auto& ep : m_active_endpoints) {
                                    if (ep.fd == m_client_fds[c]) {
                                        ep.last_seen_ms = get_time_ms();
                                        break;
                                    }
                                }
                                pending_rx.emplace_back(m_client_fds[c],
                                                        std::vector<uint8_t>(rx_buf, rx_buf + n));
                            } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                                dead_clients.push_back(m_client_fds[c]);
                            }
                        }
                        idx++;
                    }
                }

                // Remove disconnected clients
                for (int dead_fd : dead_clients) {
                    close(dead_fd);
                    m_client_fds.erase(std::remove(m_client_fds.begin(), m_client_fds.end(), dead_fd),
                                       m_client_fds.end());
                    std::string if_name = "TCP";
                    std::string r_addr = "";
                    for (auto it = m_active_endpoints.begin(); it != m_active_endpoints.end(); ) {
                        if (it->fd == dead_fd) {
                            if_name = it->interface_name;
                            r_addr = it->remote_addr;
                            it = m_active_endpoints.erase(it);
                        } else {
                            ++it;
                        }
                    }
                    dead_info.push_back({dead_fd, if_name, r_addr});
                }
            } // lock released before we process bytes or send replies

            for (auto& di : dead_info) {
                release_tcp_channel(di.fd);
            }
            for (auto& [fd, bytes] : pending_rx) {
                const uint8_t channel = channel_for_tcp_fd(fd);
                for (uint8_t b : bytes) {
                    process_incoming_byte(b, channel);
                }
            }

            for (auto& di : dead_info) {
                std::printf("[GCS DISCONNECTED] Ground Control Station disconnected from %s (%s, fd %d)\n",
                            di.if_name.c_str(), di.r_addr.c_str(), di.fd);
                // No send_statustext() broadcast here either - see the
                // comment on the TCP accept path above.
            }
        }

        // Periodic telemetry streams
        update();
    }
}

#endif // !APX_EMBEDDED (network_io_loop)

void MavlinkHandler::process_incoming_byte(uint8_t byte, uint8_t channel) {
    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_parse_char(channel, byte, &msg, &status)) {
        handle_message(msg);
    }
}

uint8_t MavlinkHandler::channel_for_tcp_fd(int fd) {
    if (!m_tcp_channels_init) {
        for (int& owner : m_tcp_channel_fd) {
            owner = -1;
        }
        m_tcp_channels_init = true;
    }
    int free_slot = -1;
    for (int ch = 1; ch < kSerialChannel; ++ch) {
        if (m_tcp_channel_fd[ch] == fd) {
            return static_cast<uint8_t>(ch);
        }
        if (free_slot < 0 && m_tcp_channel_fd[ch] < 0) {
            free_slot = ch;
        }
    }
    if (free_slot < 0) {
        // More TCP clients than parser channels: share the last TCP one
        // (the pre-fix behaviour, now limited to the overflow case).
        return static_cast<uint8_t>(kSerialChannel - 1);
    }
    m_tcp_channel_fd[free_slot] = fd;
    mavlink_reset_channel_status(static_cast<uint8_t>(free_slot));
    return static_cast<uint8_t>(free_slot);
}

void MavlinkHandler::release_tcp_channel(int fd) {
    if (!m_tcp_channels_init) {
        return;
    }
    for (int ch = 1; ch < kSerialChannel; ++ch) {
        if (m_tcp_channel_fd[ch] == fd) {
            m_tcp_channel_fd[ch] = -1;
            mavlink_reset_channel_status(static_cast<uint8_t>(ch));
        }
    }
}

size_t MavlinkHandler::read_bytes(uint8_t*, size_t) {
    return 0;
}

void MavlinkHandler::set_sensor_status(uint32_t present, uint32_t enabled, uint32_t healthy) {
    m_sensors_present.store(present);
    m_sensors_enabled.store(enabled);
    m_sensors_health.store(healthy);
}

void MavlinkHandler::set_estimates_valid(bool attitude, bool position) {
    m_attitude_valid.store(attitude);
    m_position_valid.store(position);
}

void MavlinkHandler::set_byte_writer(ByteWriter writer, void* ctx) {
    m_byte_writer_ctx = ctx;
    m_byte_writer = writer;
}

void MavlinkHandler::receive_bytes(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        process_incoming_byte(data[i], kSerialChannel);
    }
}

#if !defined(APX_EMBEDDED) // hosted socket write

void MavlinkHandler::write_bytes(const uint8_t* buf, size_t len) {
    if (!buf || len == 0) return;

    if (m_byte_writer != nullptr) {
        m_byte_writer(buf, len, m_byte_writer_ctx);
    }

    // 1. Send to default local UDP broadcast target (127.0.0.1:14550)
    if (m_udp_sock >= 0) {
        sendto(m_udp_sock, buf, len, 0,
               (struct sockaddr*)&m_udp_target_addr, sizeof(m_udp_target_addr));

        // Also send to all registered remote UDP clients
        for (const auto& rem : m_udp_remotes) {
            sendto(m_udp_sock, buf, len, 0, (struct sockaddr*)&rem, sizeof(rem));
        }
    }

    // 2. Send to all connected TCP clients across ports 5760-5763
    LockGuardType lock(m_network_mutex);
    for (int cfd : m_client_fds) {
        send(cfd, buf, len, MSG_NOSIGNAL);
    }
}
#else
bool MavlinkHandler::start_network(uint16_t, const std::vector<uint16_t>&) {
    m_network_running = true;
    return true;
}

void MavlinkHandler::stop_network() {
    m_network_running = false;
}

void MavlinkHandler::network_io_loop() {}

void MavlinkHandler::write_bytes(const uint8_t* buf, size_t len) {
    if (buf != nullptr && len > 0 && m_byte_writer != nullptr) {
        m_byte_writer(buf, len, m_byte_writer_ctx);
    }
}
#endif

void MavlinkHandler::update() {
    uint32_t now = get_time_ms();
    uint32_t time_boot_ms = now - m_start_time_ms;

    // Check motor test timeout
    {
        LockGuardType lock(m_motor_test_mutex);
        if (m_motor_test.active && now >= m_motor_test.end_time_ms) {
            m_motor_test.active = false;
            m_motor_test.throttle = 0.0f;
            m_motor_test.motor_index = 0;
        }
    }

    VehicleTelemetryState state;
    {
        LockGuardType lock(m_state_mutex);
        state = m_state;
    }

    // 1 Hz Heartbeat
    if (now - m_last_heartbeat_ms >= 1000) {
        send_heartbeat();
        if (state.battery_voltage >= 0.0f) { // < 0: no battery monitor
            send_battery_status(state.battery_voltage, state.battery_current, state.battery_remaining);
        }
        m_last_heartbeat_ms = now;
    }

    // 2 Hz System Status
    if (now - m_last_sys_status_ms >= 500) {
        send_sys_status();
        m_last_sys_status_ms = now;
    }

    RawSensorTelemetry raw;
    bool have_raw = false;
    {
        LockGuardType lock(m_state_mutex);
        have_raw = m_have_raw_sensors;
        if (have_raw) {
            raw = m_raw_sensors;
        }
    }

    // 10 Hz HIGHRES_IMU per sensor lane (id = lane index).
    if (have_raw && now - m_last_highres_imu_ms >= 100) {
        // accel, gyro, mag, abs_pressure, pressure_alt, temperature
        constexpr uint16_t kFields = 0x01FF | (1U << 9) | (1U << 11) | (1U << 12);
        for (int i = 0; i < RawSensorTelemetry::kMaxImu; ++i) {
            const auto& imu = raw.imu[i];
            if (!imu.present) {
                continue;
            }
            mavlink_message_t msg;
            mavlink_msg_highres_imu_pack(m_sysid, m_compid, &msg, imu.time_us,
                                         imu.accel[0], imu.accel[1], imu.accel[2],
                                         imu.gyro[0], imu.gyro[1], imu.gyro[2],
                                         imu.mag[0], imu.mag[1], imu.mag[2],
                                         imu.pressure_hpa, 0.0f, imu.pressure_alt_m, imu.temp_c,
                                         kFields, static_cast<uint8_t>(i));
            send_message(msg);
        }
        m_last_highres_imu_ms = now;
    }

    // 5 Hz GPS Raw Int
    if (have_raw && now - m_last_gps_raw_ms >= 200) {
        // From the simulated receivers. Receiver 1 -> GPS_RAW_INT,
        // receiver 2 -> GPS2_RAW; no receiver -> GPS_RAW_INT "no GPS".
        auto pack_gps = [&](int i, mavlink_message_t& msg) {
            const auto& g = raw.gps[i];
            const float speed = std::sqrt(g.vel_ned[0] * g.vel_ned[0] + g.vel_ned[1] * g.vel_ned[1]);
            const uint16_t cog = speed > 0.3f
                ? static_cast<uint16_t>(std::fmod(std::atan2(g.vel_ned[1], g.vel_ned[0]) * 180.0f / static_cast<float>(M_PI) + 360.0f, 360.0f) * 100.0f)
                : UINT16_MAX;
            const uint8_t fix = g.fix_3d ? GPS_FIX_TYPE_3D_FIX : GPS_FIX_TYPE_NO_FIX;
            const auto lat = static_cast<int32_t>(g.lat_deg * 1e7);
            const auto lon = static_cast<int32_t>(g.lon_deg * 1e7);
            const auto alt = static_cast<int32_t>(g.alt_msl_m * 1000.0f);
            const auto eph = static_cast<uint16_t>(std::min(g.hdop * 100.0f, 65534.0f));
            const auto epv = static_cast<uint16_t>(std::min(g.vdop * 100.0f, 65534.0f));
            const auto vel = static_cast<uint16_t>(speed * 100.0f);
            if (i == 0) {
                mavlink_msg_gps_raw_int_pack(m_sysid, m_compid, &msg, g.time_us, fix, lat, lon, alt, eph, epv,
                                             vel, cog, g.satellites, 0, 0, 0, 0, 0, 0);
            } else {
                mavlink_msg_gps2_raw_pack(m_sysid, m_compid, &msg, g.time_us, fix, lat, lon, alt, eph, epv,
                                          vel, cog, g.satellites, 0, 0, 0, 0, 0, 0, 0, 0);
            }
        };
        mavlink_message_t msg;
        if (raw.gps_count <= 0) {
            mavlink_msg_gps_raw_int_pack(m_sysid, m_compid, &msg, static_cast<uint64_t>(now) * 1000ULL,
                                         GPS_FIX_TYPE_NO_GPS, 0, 0, 0, UINT16_MAX, UINT16_MAX, 0, UINT16_MAX,
                                         0, 0, 0, 0, 0, 0, 0);
            send_message(msg);
        }
        for (int i = 0; i < std::min(raw.gps_count, 2); ++i) {
            if (raw.gps[i].present) {
                pack_gps(i, msg);
                send_message(msg);
            }
        }
        m_last_gps_raw_ms = now;
    } else if (!have_raw && now - m_last_gps_raw_ms >= 200) {
        uint64_t time_usec = static_cast<uint64_t>(now) * 1000ULL;
        send_gps_raw_int(time_usec, 3 /* 3D FIX */,
                         static_cast<int32_t>(state.lat * 1e7),
                         static_cast<int32_t>(state.lon * 1e7),
                         static_cast<int32_t>(state.alt_msl * 1000.0f),
                         100, 100,
                         static_cast<uint16_t>(state.groundspeed * 100.0f),
                         // state.yaw is signed radians in [-pi,pi] (see
                         // mode_loiter.cpp's wrap convention) - clamp(...,
                         // 0, 360) on the raw degrees value floors every
                         // negative yaw (i.e. every heading from 180 to
                         // 360 degrees, half the compass) to 0 instead of
                         // wrapping it, showing a wildly wrong
                         // course-over-ground on any GCS reading this
                         // field for headings west of due south. fmod(...+
                         // 360, 360) is the correct wrap (same formula
                         // GLOBAL_POSITION_INT's hdg field below already
                         // uses correctly).
                         static_cast<uint16_t>(std::fmod(state.yaw * 180.0f / static_cast<float>(M_PI) + 360.0f, 360.0f) * 100.0f),
                         14);
        m_last_gps_raw_ms = now;
    }

    // 10 Hz Global Position & VFR HUD (only with a position estimate)
    if (m_position_valid.load() && now - m_last_position_ms >= 100) {
        uint16_t hdg = static_cast<uint16_t>(std::fmod(state.yaw * 180.0f / static_cast<float>(M_PI) + 360.0f, 360.0f) * 100.0f);
        send_global_position_int(time_boot_ms,
                                 static_cast<int32_t>(state.lat * 1e7),
                                 static_cast<int32_t>(state.lon * 1e7),
                                 static_cast<int32_t>(state.alt_msl * 1000.0f),
                                 static_cast<int32_t>(state.rel_alt * 1000.0f),
                                 static_cast<int16_t>(state.vx * 100.0f),
                                 static_cast<int16_t>(state.vy * 100.0f),
                                 static_cast<int16_t>(state.vz * 100.0f),
                                 hdg);
        send_vfr_hud(state.airspeed, state.groundspeed, static_cast<int16_t>(hdg / 100),
                     state.throttle_pct, state.alt_msl, -state.vz);
        m_last_position_ms = now;
    }

    // 20 Hz Attitude (only with an attitude estimate)
    if (m_attitude_valid.load() && now - m_last_attitude_ms >= 50) {
        send_attitude(time_boot_ms, state.roll, state.pitch, state.yaw,
                      state.rollspeed, state.pitchspeed, state.yawspeed);
        m_last_attitude_ms = now;
    }

    // 10 Hz Servo/motor outputs - lets a GCS (or a raw MAVLink log) confirm
    // whether motors are actually being commanded, independent of whether
    // they're visibly spinning in the simulator.
    if (now - m_last_servo_output_ms >= 100) {
        send_servo_output_raw();
        m_last_servo_output_ms = now;
    }

    // 4 Hz Wind Estimation (MAVLink message 168 WIND)
    if (now - m_last_wind_ms >= 250) {
        send_wind(state.wind_direction, state.wind_speed, state.wind_speed_z);
        m_last_wind_ms = now;
    }

    // 10 Hz RC_CHANNELS - send_rc_channels() already existed, fully
    // implemented, but was never actually called from anywhere: GCS's
    // Radio Setup page has nothing to show or calibrate against without
    // a live RC_CHANNELS/RC_CHANNELS_RAW stream, which is most of why it
    // reports "RC not calibrated" regardless of what RCn_MIN/MAX/TRIM
    // parameters exist. m_rc_channels only stores normalized -1..1/0..1
    // values (handle_manual_control()/handle_rc_channels_override() both
    // convert incoming PWM to normalized immediately and don't keep the
    // raw figures), so this reconstructs PWM from the same RCn_MIN/MAX/
    // TRIM calibration parameters a real receiver/calibration would use -
    // channels 1-4 in the fixed roll/pitch/throttle/yaw order this
    // codebase already assumes elsewhere (RCMAP_* isn't actually honored
    // yet - see param_manager.cpp's registration comment), channels 5-8
    // at their trim/center value since nothing drives them.
    if (now - m_last_rc_channels_ms >= 100) {
        RcChannels rc = get_rc_channels();
        auto& pm = apx::gcs::ParamManager::get_instance();
        auto bipolar_pwm = [&](int ch, float v) -> uint16_t {
            int32_t lo = pm.get_int_or(("RC" + std::to_string(ch) + "_MIN").c_str(), 1100);
            int32_t hi = pm.get_int_or(("RC" + std::to_string(ch) + "_MAX").c_str(), 1900);
            int32_t trim = pm.get_int_or(("RC" + std::to_string(ch) + "_TRIM").c_str(), 1500);
            float pwm = static_cast<float>(trim) + v * static_cast<float>(v >= 0.0f ? (hi - trim) : (trim - lo));
            return static_cast<uint16_t>(std::clamp(pwm, 800.0f, 2200.0f));
        };
        auto unipolar_pwm = [&](int ch, float v) -> uint16_t {
            int32_t lo = pm.get_int_or(("RC" + std::to_string(ch) + "_MIN").c_str(), 1100);
            int32_t hi = pm.get_int_or(("RC" + std::to_string(ch) + "_MAX").c_str(), 1900);
            float pwm = static_cast<float>(lo) + v * static_cast<float>(hi - lo);
            return static_cast<uint16_t>(std::clamp(pwm, 800.0f, 2200.0f));
        };
        uint16_t trim5 = static_cast<uint16_t>(pm.get_int_or("RC5_TRIM", 1500));
        uint16_t trim6 = static_cast<uint16_t>(pm.get_int_or("RC6_TRIM", 1500));
        uint16_t trim7 = static_cast<uint16_t>(pm.get_int_or("RC7_TRIM", 1500));
        uint16_t trim8 = static_cast<uint16_t>(pm.get_int_or("RC8_TRIM", 1500));
        send_rc_channels(time_boot_ms, 0,
                         bipolar_pwm(1, rc.roll), bipolar_pwm(2, rc.pitch),
                         unipolar_pwm(3, rc.throttle), bipolar_pwm(4, rc.yaw),
                         trim5, trim6, trim7, trim8,
                         rc.valid ? 100 : 255 /* 255 = RSSI unknown, matches MAVLink convention */);
        m_last_rc_channels_ms = now;
    }

    // 2 Hz FENCE_STATUS (MAVLink message 162) - reports safety geofence breach status & action
    if (now - m_last_fence_status_ms >= 500) {
        auto& fence = apx::safety::FenceManager::instance();
        mavlink_message_t fence_msg;
        mavlink_msg_fence_status_pack(m_sysid, m_compid, &fence_msg,
                                      fence.is_breached() ? 1 : 0,
                                      fence.breach_count(),
                                      static_cast<uint8_t>(fence.current_breach()),
                                      fence.last_breach_time_ms(),
                                      static_cast<uint8_t>(fence.get_action()));
        send_message(fence_msg);
        m_last_fence_status_ms = now;
    }

    // 1 Hz HOME_POSITION (MAVLink message 242) - streams vehicle home location
    if (m_home_set && now - m_last_home_position_ms >= 1000) {
        send_home_position();
        m_last_home_position_ms = now;
    }

    // 20 Hz Radar & Terrain-Following Telemetry (DISTANCE_SENSOR #132 & OBSTACLE_DISTANCE #330)
    if (now - m_last_radar_ms >= 50) {
        send_distance_sensors();
        send_obstacle_distance();
        m_last_radar_ms = now;
    }

    // MISSION_CURRENT / MISSION_ITEM_REACHED - only on an actual change,
    // not on a fixed rate (nothing to report most ticks). This is how GCS
    // knows which waypoint AUTO is flying toward and draws mission
    // progress; previously nothing here ever sent either message, so a
    // GCS had no way to see a mission actually being followed even once
    // ModeAuto could fly one.
    if (m_mode_manager) {
        int seq = m_mode_manager->auto_mission_current_seq();
        if (seq != m_last_sent_mission_seq) {
            // Any change away from a real, previously-reported item means
            // that item's leg is done - including the final item (seq
            // going to -1/mission-complete), not just "moved on to a
            // numerically later one".
            if (m_last_sent_mission_seq >= 0) {
                mavlink_message_t reached_msg;
                mavlink_msg_mission_item_reached_pack(m_sysid, m_compid, &reached_msg,
                                                      static_cast<uint16_t>(m_last_sent_mission_seq));
                send_message(reached_msg);
            }
            if (seq >= 0) {
                uint16_t total;
                {
                    LockGuardType lock(m_mission_mutex);
                    total = static_cast<uint16_t>(m_mission_items.size());
                }
                mavlink_message_t cur_msg;
                mavlink_msg_mission_current_pack(m_sysid, m_compid, &cur_msg,
                                                 static_cast<uint16_t>(seq), total, 0, 0);
                send_message(cur_msg);
            }
            m_last_sent_mission_seq = seq;
        }
    }
}

void MavlinkHandler::send_message(const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write_bytes(buf, len);
}

void MavlinkHandler::send_heartbeat() {
    VehicleTelemetryState state;
    {
        LockGuardType lock(m_state_mutex);
        state = m_state;
    }

    mavlink_message_t msg;
    // Custom mode dialect (not GENERIC): APX Flight Manager and standard GCS
    // select vehicle plugins and telemetry interpretation based on this field.
    // Standard multicopter custom_mode numbers (0=STABILIZE, 4=GUIDED, 3=AUTO, etc.)
    // map correctly in GCS UI when using this autopilot dialect identifier.
    mavlink_msg_heartbeat_pack(m_sysid, m_compid, &msg,
                               m_vehicle_type.load(),
                               kHeartbeatAutopilotType,
                               state.base_mode,
                               state.custom_mode,
                               state.system_status);
    send_message(msg);
}

void MavlinkHandler::send_attitude(uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(m_sysid, m_compid, &msg, time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
    send_message(msg);
}

void MavlinkHandler::send_global_position_int(uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg) {
    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(m_sysid, m_compid, &msg, time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg);
    send_message(msg);
}

void MavlinkHandler::send_gps_raw_int(uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible) {
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack(m_sysid, m_compid, &msg, time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, 0, 0, 0, 0, 0, 0);
    send_message(msg);
}

void MavlinkHandler::send_vfr_hud(float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb) {
    mavlink_message_t msg;
    mavlink_msg_vfr_hud_pack(m_sysid, m_compid, &msg, airspeed, groundspeed, heading, throttle, alt, climb);
    send_message(msg);
}

void MavlinkHandler::send_wind(float direction_deg, float speed_ms, float speed_z_ms) {
    mavlink_message_t msg;
    mavlink_msg_wind_pack(m_sysid, m_compid, &msg, direction_deg, speed_ms, speed_z_ms);
    send_message(msg);
}

void MavlinkHandler::set_wind(float direction_deg, float speed_ms, float speed_z_ms) {
    LockGuardType lock(m_state_mutex);
    m_state.wind_direction = direction_deg;
    m_state.wind_speed = speed_ms;
    m_state.wind_speed_z = speed_z_ms;
}

void MavlinkHandler::send_sys_status() {
    mavlink_message_t msg;
    // 3D Gyro, Accel, Mag, Baro, GPS, Batt, RC Receiver (bit 16 -
    // MAV_SYS_STATUS_SENSOR_RC_RECEIVER) - added alongside wiring up the
    // actual RC_CHANNELS stream (see update()'s periodic send section);
    // leaving this bit unset while RC data IS flowing was a separate,
    // smaller contributor to a GCS showing RC as not present/calibrated.
    // The set comes from set_sensor_status() (default: all of these, as
    // simulated in SITL).
    const uint32_t sensors_present = m_sensors_present.load();
    const uint32_t sensors_enabled = m_sensors_enabled.load();
    const uint32_t sensors_health = m_sensors_health.load();

    VehicleTelemetryState state;
    {
        LockGuardType lock(m_state_mutex);
        state = m_state;
    }

    // Negative = no battery monitor: report "unknown" as MAVLink defines it.
    const bool batt_known = state.battery_voltage >= 0.0f;
    uint16_t voltage_mv = batt_known ? static_cast<uint16_t>(state.battery_voltage * 1000.0f) : UINT16_MAX;
    int16_t current_ca = batt_known ? static_cast<int16_t>(state.battery_current * 100.0f) : -1;

    mavlink_msg_sys_status_pack(m_sysid, m_compid, &msg,
                                sensors_present, sensors_enabled, sensors_health,
                                250 /* 25% load */, voltage_mv, current_ca,
                                state.battery_remaining, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    send_message(msg);
}

void MavlinkHandler::send_battery_status(float voltage_v, float current_a, int8_t battery_remaining) {
    mavlink_message_t msg;
    // mV / cA: the volts were previously truncated to whole volts first.
    uint16_t voltages[10] = {static_cast<uint16_t>(voltage_v * 1000.0f), 0};
    mavlink_msg_battery_status_pack(m_sysid, m_compid, &msg,
                                    0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
                                    2500, voltages, static_cast<int16_t>(current_a * 100.0f), -1, -1,
                                    battery_remaining, 0, MAV_BATTERY_CHARGE_STATE_OK,
                                    voltages, 0, 0);
    send_message(msg);
}

void MavlinkHandler::send_rc_channels(uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi) {
    mavlink_message_t msg;
    mavlink_msg_rc_channels_pack(m_sysid, m_compid, &msg, time_boot_ms, 8,
                                 chan1_raw, chan2_raw, chan3_raw, chan4_raw,
                                 chan5_raw, chan6_raw, chan7_raw, chan8_raw,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, rssi);
    send_message(msg);
}

void MavlinkHandler::send_statustext(MAV_SEVERITY severity, const std::string& text) {
    mavlink_message_t msg;
    char text_buf[50] = {0};
    std::strncpy(text_buf, text.c_str(), 49);
    mavlink_msg_statustext_pack(m_sysid, m_compid, &msg, severity, text_buf, 0, 0);
    send_message(msg);
}

void MavlinkHandler::send_command_ack(uint16_t command, uint8_t result) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(m_sysid, m_compid, &msg, command, result, 0, 0, 0, 0);
    send_message(msg);
}

void MavlinkHandler::send_autopilot_version() {
    mavlink_message_t msg;
    uint8_t custom_ver[8] = {0, 1, 0, 0, 0, 0, 0, 0};
    uint8_t uid2[18] = {0};
    uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
    mavlink_msg_autopilot_version_pack(m_sysid, m_compid, &msg,
                                       capabilities,
                                       0x00010000, 0x00010000, 0x00010000, 1,
                                       custom_ver, custom_ver, custom_ver,
                                       0, 0, 0, uid2);
    send_message(msg);
}

void MavlinkHandler::send_servo_output_raw() {
    VehicleTelemetryState state;
    {
        LockGuardType lock(m_state_mutex);
        state = m_state;
    }

    mavlink_message_t msg;
    mavlink_msg_servo_output_raw_pack(m_sysid, m_compid, &msg,
                                      get_time_ms() * 1000, 0,
                                      state.servo_pwm[0], state.servo_pwm[1],
                                      state.servo_pwm[2], state.servo_pwm[3],
                                      state.servo_pwm[4], state.servo_pwm[5],
                                      state.servo_pwm[6], state.servo_pwm[7],
                                      state.servo_pwm[8], state.servo_pwm[9],
                                      state.servo_pwm[10], state.servo_pwm[11],
                                      state.servo_pwm[12], state.servo_pwm[13],
                                      state.servo_pwm[14], state.servo_pwm[15]);
    send_message(msg);
}

void MavlinkHandler::send_distance_sensors() {
    auto& radar = apx::radar::RadarManager::get_instance();
    if (!radar.is_radar_system_enabled()) return;

    const uint32_t time_boot_ms = get_time_ms();

    for (uint8_t i = 0; i < apx::radar::kNumRadarSensors; ++i) {
        auto cfg = radar.get_sensor_config(i);
        if (!cfg.enabled) continue;
        auto reading = radar.get_sensor_reading(i);
        if (!reading.healthy) continue;

        uint16_t current_cm = static_cast<uint16_t>(std::clamp(std::round(reading.distance_m * 100.0f), 0.0f, 65535.0f));
        uint16_t min_cm = static_cast<uint16_t>(std::clamp(std::round(cfg.min_distance_m * 100.0f), 0.0f, 65535.0f));
        uint16_t max_cm = static_cast<uint16_t>(std::clamp(std::round(cfg.max_distance_m * 100.0f), 0.0f, 65535.0f));

        mavlink_message_t msg;
        float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        mavlink_msg_distance_sensor_pack(
            m_sysid, m_compid, &msg,
            time_boot_ms,
            min_cm,
            max_cm,
            reading.valid ? current_cm : max_cm,
            MAV_DISTANCE_SENSOR_RADAR,
            i,
            cfg.orientation,
            255,   // unknown covariance
            0.35f, // ~20 deg horizontal FOV
            0.35f, // ~20 deg vertical FOV
            q,
            reading.signal_quality
        );
        send_message(msg);
    }
}

void MavlinkHandler::send_obstacle_distance() {
    auto& radar = apx::radar::RadarManager::get_instance();
    if (!radar.is_radar_system_enabled()) return;

    uint16_t distances_cm[72];
    float min_m = 0.2f, max_m = 40.0f;
    radar.get_obstacle_distances_cm(distances_cm, min_m, max_m);

    mavlink_message_t msg;
    mavlink_msg_obstacle_distance_pack(
        m_sysid, m_compid, &msg,
        static_cast<uint64_t>(get_time_ms()) * 1000ULL,
        MAV_DISTANCE_SENSOR_RADAR,
        distances_cm,
        5, // increment deg
        static_cast<uint16_t>(min_m * 100.0f),
        static_cast<uint16_t>(max_m * 100.0f),
        0.0f,
        0.0f,
        MAV_FRAME_BODY_FRD
    );
    send_message(msg);
}

void MavlinkHandler::handle_message(const mavlink_message_t& msg) {
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        if (hb.type == MAV_TYPE_GCS || msg.sysid == 255) {
            static uint32_t last_gcs_hb_log = 0;
            uint32_t now = get_time_ms();
            if (now - last_gcs_hb_log > 5000) {
                std::printf("[GCS HEARTBEAT] Active link verified with GCS (SysID: %u, CompID: %u, Type: %u)\n",
                            msg.sysid, msg.compid, hb.type);
                last_gcs_hb_log = now;
            }
        }
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            handle_param_request_list(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            handle_param_set(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            handle_param_request_read(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
        case MAVLINK_MSG_ID_COMMAND_INT:
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);
                if (cmd.command == 520 /* MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES */ ||
                    (cmd.command == 512 /* MAV_CMD_REQUEST_MESSAGE */ && static_cast<uint16_t>(cmd.param1) == 148 /* AUTOPILOT_VERSION */)) {
                    send_autopilot_version();
                    send_command_ack(cmd.command, MAV_RESULT_ACCEPTED);
                    break;
                }
            }
            if (m_command_handler) {
                m_command_handler->handle_command(msg);
            }
            break;
        case MAVLINK_MSG_ID_SET_MODE:
            handle_set_mode(msg);
            break;
        case MAVLINK_MSG_ID_SET_HOME_POSITION: {
            mavlink_set_home_position_t shp;
            mavlink_msg_set_home_position_decode(&msg, &shp);
            if (shp.target_system == m_sysid || shp.target_system == 0) {
                double lat = static_cast<double>(shp.latitude) * 1e-7;
                double lon = static_cast<double>(shp.longitude) * 1e-7;
                float alt_msl = static_cast<float>(shp.altitude) * 1e-3f;
                set_home_position(lat, lon, alt_msl, true);
                send_statustext(MAV_SEVERITY_INFO, "Home position set from GCS");
            }
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            handle_mission_request_list(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
            handle_mission_count(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        case MAVLINK_MSG_ID_MISSION_ITEM:
            handle_mission_item_int(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST:
            handle_mission_request_int(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            handle_mission_clear_all(msg);
            break;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
            handle_set_position_target_local_ned(msg);
            break;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
            handle_set_position_target_global_int(msg);
            break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            handle_manual_control(msg);
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handle_rc_channels_override(msg);
            break;
        default:
            break;
    }
}

void MavlinkHandler::handle_param_request_list(const mavlink_message_t& msg) {
    mavlink_param_request_list_t req;
    mavlink_msg_param_request_list_decode(&msg, &req);
    
    if (req.target_system != m_sysid && req.target_system != 0) return;

    auto names = ParamManager::get_instance().get_all_param_names();
    for (const auto& name : names) {
        send_param_value(name);
    }
}

void MavlinkHandler::handle_param_set(const mavlink_message_t& msg) {
    mavlink_param_set_t req;
    mavlink_msg_param_set_decode(&msg, &req);

    if (req.target_system != m_sysid && req.target_system != 0) return;
    
    char name[17];
    std::strncpy(name, req.param_id, 16);
    name[16] = '\0';
    
    std::string name_str(name);
    auto def = ParamManager::get_instance().get_param_def(name_str);
    
    if (def) {
        if (def->type == ParamType::FLOAT) {
            ParamManager::get_instance().set_float(name_str, req.param_value);
        } else if (def->type == ParamType::INT32 || def->type == ParamType::BOOL) {
            ParamManager::get_instance().set_int(name_str, static_cast<int32_t>(req.param_value));
        }
        send_param_value(name_str);
    }
}

void MavlinkHandler::handle_param_request_read(const mavlink_message_t& msg) {
    mavlink_param_request_read_t req;
    mavlink_msg_param_request_read_decode(&msg, &req);

    if (req.target_system != m_sysid && req.target_system != 0) return;

    if (req.param_index >= 0) {
        auto names = ParamManager::get_instance().get_all_param_names();
        if (static_cast<size_t>(req.param_index) < names.size()) {
            send_param_value(names[req.param_index]);
        }
    } else {
        char name[17];
        std::strncpy(name, req.param_id, 16);
        name[16] = '\0';
        send_param_value(std::string(name));
    }
}

void MavlinkHandler::send_param_value(const std::string& param_name) {
    auto p = ParamManager::get_instance().get_param(param_name);
    auto pdef = ParamManager::get_instance().get_param_def(param_name);
    if (!p || !pdef) return;
    
    float val = 0.0f;
    uint8_t mtype = MAV_PARAM_TYPE_REAL32;
    
    if (p->type == ParamType::FLOAT) {
        val = p->f;
        mtype = MAV_PARAM_TYPE_REAL32;
    } else {
        val = static_cast<float>(p->i);
        mtype = MAV_PARAM_TYPE_INT32;
    }
    
    auto names = ParamManager::get_instance().get_all_param_names();
    uint16_t index = 0xFFFF;
    for (size_t i = 0; i < names.size(); ++i) {
        if (names[i] == param_name) {
            index = static_cast<uint16_t>(i);
            break;
        }
    }

    mavlink_message_t out_msg;
    char param_id[17] = {0};
    std::strncpy(param_id, param_name.c_str(), 16);
    mavlink_msg_param_value_pack(m_sysid, m_compid, &out_msg, param_id, val, mtype,
                                 static_cast<uint16_t>(names.size()), index);
    send_message(out_msg);
}

void MavlinkHandler::mission_request_next_item(uint8_t target_sysid, uint8_t target_compid, uint16_t seq) {
    mavlink_message_t req_msg;
    mavlink_msg_mission_request_int_pack(m_sysid, m_compid, &req_msg,
                                         target_sysid, target_compid, seq, MAV_MISSION_TYPE_MISSION);
    send_message(req_msg);
}

void MavlinkHandler::handle_set_mode(const mavlink_message_t& msg) {
    mavlink_set_mode_t req;
    mavlink_msg_set_mode_decode(&msg, &req);
    if (req.target_system != m_sysid && req.target_system != 0) return;
    if (!m_mode_manager) return;

    // Accept standard MAVLink SET_MODE message alongside COMMAND_LONG DO_SET_MODE
    // for compatibility with GCS and telemetry tools.
    auto mode_id = apx::modes::ModeManager::from_custom_mode_number(req.custom_mode);
    if (!mode_id) return;
    auto state = get_last_known_vehicle_state();
    m_mode_manager->request_mode(*mode_id, state);
}

void MavlinkHandler::handle_mission_request_list(const mavlink_message_t& msg) {
    mavlink_mission_request_list_t req;
    mavlink_msg_mission_request_list_decode(&msg, &req);
    if (req.target_system != m_sysid && req.target_system != 0) return;

    uint16_t count = 0;
    {
        LockGuardType lock(m_mission_mutex);
        count = static_cast<uint16_t>(m_mission_items.size());
    }

    mavlink_message_t reply;
    mavlink_msg_mission_count_pack(m_sysid, m_compid, &reply, msg.sysid, msg.compid, count, MAV_MISSION_TYPE_MISSION);
    send_message(reply);
}

void MavlinkHandler::handle_mission_count(const mavlink_message_t& msg) {
    mavlink_mission_count_t req;
    mavlink_msg_mission_count_decode(&msg, &req);
    if (req.target_system != m_sysid && req.target_system != 0) return;

    m_mission_upload_type = req.mission_type;

    if (req.count == 0) {
        if (req.mission_type == MAV_MISSION_TYPE_FENCE) {
            apx::safety::FenceManager::instance().clear_polygon();
            mavlink_message_t reply;
            mavlink_msg_mission_ack_pack(m_sysid, m_compid, &reply, msg.sysid, msg.compid,
                                         MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_FENCE);
            send_message(reply);
            return;
        }

        // GCS is uploading an empty mission - just clear ours and ack.
        {
            LockGuardType lock(m_mission_mutex);
            m_mission_items.clear();
        }
        m_mission_upload_active = false;
        if (m_mode_manager) {
            m_mode_manager->load_mission({});
        }

        mavlink_message_t reply;
        mavlink_msg_mission_ack_pack(m_sysid, m_compid, &reply, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        send_message(reply);
        return;
    }

    m_mission_upload_active = true;
    m_mission_upload_expected = req.count;
    m_mission_upload_gcs_sysid = msg.sysid;
    m_mission_upload_gcs_compid = msg.compid;
    m_mission_upload_buffer.clear();
    m_mission_upload_buffer.reserve(req.count);

    // Kick off the transfer by requesting item 0.
    mission_request_next_item(msg.sysid, msg.compid, 0);
}

void MavlinkHandler::handle_mission_item_int(const mavlink_message_t& msg) {
    if (!m_mission_upload_active) {
        // We never requested this item (no upload in progress) - ignore.
        return;
    }

    MissionItem item{};
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        mavlink_mission_item_int_t mi;
        mavlink_msg_mission_item_int_decode(&msg, &mi);
        if (mi.target_system != m_sysid && mi.target_system != 0) return;
        item.seq = mi.seq;
        item.frame = mi.frame;
        item.command = mi.command;
        item.current = mi.current;
        item.autocontinue = mi.autocontinue;
        item.param1 = mi.param1;
        item.param2 = mi.param2;
        item.param3 = mi.param3;
        item.param4 = mi.param4;
        item.x = mi.x;
        item.y = mi.y;
        item.z = mi.z;
        item.mission_type = mi.mission_type;
    } else {
        // Legacy MISSION_ITEM: x/y are plain float lat/lon degrees,
        // convert to the *1e7 fixed point form used internally/by _INT.
        mavlink_mission_item_t mi;
        mavlink_msg_mission_item_decode(&msg, &mi);
        if (mi.target_system != m_sysid && mi.target_system != 0) return;
        item.seq = mi.seq;
        item.frame = mi.frame;
        item.command = mi.command;
        item.current = mi.current;
        item.autocontinue = mi.autocontinue;
        item.param1 = mi.param1;
        item.param2 = mi.param2;
        item.param3 = mi.param3;
        item.param4 = mi.param4;
        item.x = static_cast<int32_t>(mi.x * 1e7f);
        item.y = static_cast<int32_t>(mi.y * 1e7f);
        item.z = mi.z;
        item.mission_type = mi.mission_type;
    }

    if (item.seq != m_mission_upload_buffer.size()) {
        // Out of sequence - re-request the item we actually need next.
        mission_request_next_item(m_mission_upload_gcs_sysid, m_mission_upload_gcs_compid,
                                  static_cast<uint16_t>(m_mission_upload_buffer.size()));
        return;
    }

    m_mission_upload_buffer.push_back(item);

    if (m_mission_upload_buffer.size() < m_mission_upload_expected) {
        mission_request_next_item(m_mission_upload_gcs_sysid, m_mission_upload_gcs_compid,
                                  static_cast<uint16_t>(m_mission_upload_buffer.size()));
        return;
    }

    // All items received - commit and ack based on mission type.
    if (m_mission_upload_type == MAV_MISSION_TYPE_FENCE) {
        std::vector<apx::safety::FencePoint> fence_pts;
        for (const auto& mi : m_mission_upload_buffer) {
            double lat = static_cast<double>(mi.x) * 1e-7;
            double lon = static_cast<double>(mi.y) * 1e-7;
            float ned[3];
            global_to_ned(lat, lon, 0.0f, ned);
            fence_pts.push_back({ned[0], ned[1]});
        }
        apx::safety::FenceManager::instance().set_polygon_points(fence_pts);
        m_mission_upload_active = false;
        m_mission_upload_buffer.clear();

        mavlink_message_t reply;
        mavlink_msg_mission_ack_pack(m_sysid, m_compid, &reply,
                                     m_mission_upload_gcs_sysid, m_mission_upload_gcs_compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_FENCE);
        send_message(reply);
        send_statustext(MAV_SEVERITY_NOTICE, "Safety fence upload complete");
        return;
    }

    {
        LockGuardType lock(m_mission_mutex);
        m_mission_items = m_mission_upload_buffer;
    }
    m_mission_upload_active = false;
    m_mission_upload_buffer.clear();

    // Hand the mission to ModeManager so AUTO actually flies it (see
    // mode_manager.hpp's load_mission()) - previously nothing here ever
    // did this: an uploaded mission was stored for later MISSION_REQUEST_*
    // download only, never reached ModeAuto, which itself had no waypoint
    // sequencing logic either (both halves of "upload and follow a
    // mission" were missing).
    if (m_mode_manager) {
        m_mode_manager->load_mission(convert_mission_to_waypoints());
    }

    mavlink_message_t reply;
    mavlink_msg_mission_ack_pack(m_sysid, m_compid, &reply,
                                 m_mission_upload_gcs_sysid, m_mission_upload_gcs_compid,
                                 MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    send_message(reply);
    send_statustext(MAV_SEVERITY_NOTICE, "Mission upload complete");
}

std::vector<apx::modes::Waypoint> MavlinkHandler::convert_mission_to_waypoints() const {
    constexpr double kEarthRadiusM = 6378137.0; // matches handle_set_position_target_global_int()

    std::vector<MissionItem> items;
    {
        LockGuardType lock(m_mission_mutex);
        items = m_mission_items;
    }

    std::vector<apx::modes::Waypoint> waypoints;
    waypoints.reserve(items.size());
    for (const MissionItem& item : items) {
        // Skip item 0 ONLY if it's a planned home waypoint (MAV_CMD_NAV_WAYPOINT at home)
        if (item.seq == 0 && item.command == MAV_CMD_NAV_WAYPOINT) continue;

        apx::modes::Waypoint wp;
        wp.seq = item.seq;
        wp.command = item.command;

        // If coordinates are 0 (e.g. TAKEOFF or RTL with (0,0)), or non-spatial commands,
        // do NOT map to Null Island (-2000km South-West)
        if ((item.x == 0 && item.y == 0) || item.command == MAV_CMD_NAV_TAKEOFF ||
            item.command == MAV_CMD_NAV_RETURN_TO_LAUNCH ||
            item.command == 113 || item.command == 115 || item.command == 178 ||
            item.command == 183 || item.command == 186 || item.command == 216) {
            wp.pos_ned[0] = 0.0f;
            wp.pos_ned[1] = 0.0f;
            wp.has_location = false;
        } else {
            float ned[3];
            global_to_ned(item.x / 1e7, item.y / 1e7, 0.0f, ned);
            wp.pos_ned[0] = ned[0];
            wp.pos_ned[1] = ned[1];
        }

        float rel_alt_m;
        switch (item.frame) {
            case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                rel_alt_m = item.z;
                break;
            case MAV_FRAME_GLOBAL:
            case MAV_FRAME_GLOBAL_INT:
            default:
                rel_alt_m = (item.z != 0.0f) ? (item.z - m_home_alt_msl) : 0.0f;
                break;
        }
        if (item.command == 113 && rel_alt_m == 0.0f && item.param2 != 0.0f) {
            rel_alt_m = item.param2;
        } else if (item.command == 186 && rel_alt_m == 0.0f && item.param1 != 0.0f) {
            rel_alt_m = item.param1;
        }
        wp.pos_ned[2] = -rel_alt_m;

        // NAV_WAYPOINT's param2 is acceptance radius (m); 0 means "use the
        // vehicle's default" per the MAVLink spec, which
        // Waypoint::accept_radius_m's own 0-means-default already matches.
        wp.accept_radius_m = item.param2;
        wp.param1 = item.param1;
        wp.param2 = item.param2;
        wp.param3 = item.param3;
        wp.param4 = item.param4;

        waypoints.push_back(wp);
    }
    return waypoints;
}

void MavlinkHandler::handle_mission_request_int(const mavlink_message_t& msg) {
    uint16_t seq = 0;
    uint8_t target_system = 0;
    uint8_t target_component = 0;
    uint8_t mission_type = MAV_MISSION_TYPE_MISSION;

    if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        mavlink_mission_request_int_t req;
        mavlink_msg_mission_request_int_decode(&msg, &req);
        seq = req.seq;
        target_system = req.target_system;
        target_component = req.target_component;
        mission_type = req.mission_type;
    } else {
        mavlink_mission_request_t req;
        mavlink_msg_mission_request_decode(&msg, &req);
        seq = req.seq;
        target_system = req.target_system;
        target_component = req.target_component;
        mission_type = req.mission_type;
    }
    if (target_system != m_sysid && target_system != 0) return;

    LockGuardType lock(m_mission_mutex);
    if (seq >= m_mission_items.size()) {
        mavlink_message_t reply;
        mavlink_msg_mission_ack_pack(m_sysid, m_compid, &reply, msg.sysid, msg.compid,
                                     MAV_MISSION_INVALID_SEQUENCE, mission_type);
        send_message(reply);
        return;
    }

    const MissionItem& item = m_mission_items[seq];
    mavlink_message_t reply;
    mavlink_msg_mission_item_int_pack(m_sysid, m_compid, &reply,
                                      msg.sysid, msg.compid, item.seq, item.frame, item.command,
                                      item.current, item.autocontinue,
                                      item.param1, item.param2, item.param3, item.param4,
                                      item.x, item.y, item.z, item.mission_type);
    send_message(reply);
}

void MavlinkHandler::handle_mission_clear_all(const mavlink_message_t& msg) {
    mavlink_mission_clear_all_t req;
    mavlink_msg_mission_clear_all_decode(&msg, &req);
    if (req.target_system != m_sysid && req.target_system != 0) return;

    {
        LockGuardType lock(m_mission_mutex);
        m_mission_items.clear();
    }
    m_mission_upload_active = false;
    m_mission_upload_buffer.clear();
    if (m_mode_manager) {
        m_mode_manager->load_mission({});
    }

    mavlink_message_t reply;
    mavlink_msg_mission_ack_pack(m_sysid, m_compid, &reply, msg.sysid, msg.compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    send_message(reply);
}

void MavlinkHandler::handle_set_position_target_local_ned(const mavlink_message_t& msg) {
    mavlink_set_position_target_local_ned_t req;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &req);
    if (req.target_system != m_sysid && req.target_system != 0) return;
    if (!m_mode_manager) return;

    // Only the common "absolute NED position relative to home" case is
    // handled (what GCS/a mission-item-style guided click sends). Offset
    // frames (LOCAL_OFFSET_NED/BODY_NED/BODY_OFFSET_NED) and velocity/
    // acceleration-setpoint streaming (companion-computer offboard
    // control) aren't - see class-level notes; those need "current
    // position" and "continuous streaming setpoint" handling this doesn't
    // have yet.
    if (req.coordinate_frame != MAV_FRAME_LOCAL_NED) {
        send_statustext(MAV_SEVERITY_WARNING, "Guided: only LOCAL_NED frame supported");
        return;
    }
    bool pos_ignored = (req.type_mask & (POSITION_TARGET_TYPEMASK_X_IGNORE |
                                         POSITION_TARGET_TYPEMASK_Y_IGNORE |
                                         POSITION_TARGET_TYPEMASK_Z_IGNORE)) != 0;
    if (pos_ignored) {
        send_statustext(MAV_SEVERITY_WARNING, "Guided: velocity/accel-only setpoints not supported");
        return;
    }

    apx::modes::GuidedTarget target;
    target.pos_ned[0] = req.x;
    target.pos_ned[1] = req.y;
    target.pos_ned[2] = req.z;
    target.has_yaw = !(req.type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE);
    target.yaw = req.yaw;
    m_mode_manager->set_guided_target(target);
}

void MavlinkHandler::handle_set_position_target_global_int(const mavlink_message_t& msg) {
    mavlink_set_position_target_global_int_t req;
    mavlink_msg_set_position_target_global_int_decode(&msg, &req);
    if (req.target_system != m_sysid && req.target_system != 0) return;
    if (!m_mode_manager) return;

    bool pos_ignored = (req.type_mask & (POSITION_TARGET_TYPEMASK_X_IGNORE |
                                         POSITION_TARGET_TYPEMASK_Y_IGNORE |
                                         POSITION_TARGET_TYPEMASK_Z_IGNORE)) != 0;
    if (pos_ignored) {
        send_statustext(MAV_SEVERITY_WARNING, "Guided: velocity/accel-only setpoints not supported");
        return;
    }

    // Flat-earth projection from home - same approximation
    // apx_gazebo_plugin.cpp uses for the reverse (NED -> lat/lon)
    // conversion, kept consistent so a round trip lines up.

    float rel_alt_m;
    switch (req.coordinate_frame) {
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            rel_alt_m = req.alt;
            break;
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_INT:
        default:
            rel_alt_m = req.alt - m_home_alt_msl; // treat as MSL
            break;
    }

    apx::modes::GuidedTarget target;
    global_to_ned(req.lat_int / 1e7, req.lon_int / 1e7, rel_alt_m, target.pos_ned);
    target.has_yaw = !(req.type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE);
    target.yaw = req.yaw;
    m_mode_manager->set_guided_target(target);
}

#if !defined(APX_EMBEDDED)
bool MavlinkHandler::has_active_gcs() const {
    LockGuardType lock(const_cast<MutexType&>(m_network_mutex));
    return !m_active_endpoints.empty();
}

size_t MavlinkHandler::get_active_gcs_count() const {
    LockGuardType lock(const_cast<MutexType&>(m_network_mutex));
    return m_active_endpoints.size();
}

std::vector<MavlinkHandler::GcsEndpointInfo> MavlinkHandler::get_active_gcs_endpoints() const {
    LockGuardType lock(const_cast<MutexType&>(m_network_mutex));
    return m_active_endpoints;
}

std::string MavlinkHandler::get_active_gcs_summary() const {
    LockGuardType lock(const_cast<MutexType&>(m_network_mutex));
    if (m_active_endpoints.empty()) {
        return "None";
    }
    std::string summary;
    for (size_t i = 0; i < m_active_endpoints.size(); ++i) {
        if (i > 0) summary += ", ";
        summary += m_active_endpoints[i].interface_name + " [" + m_active_endpoints[i].remote_addr + "]";
    }
    return summary;
}
#else
bool MavlinkHandler::has_active_gcs() const { return false; }
size_t MavlinkHandler::get_active_gcs_count() const { return 0; }
std::vector<MavlinkHandler::GcsEndpointInfo> MavlinkHandler::get_active_gcs_endpoints() const { return {}; }
std::string MavlinkHandler::get_active_gcs_summary() const { return "None"; }
#endif

namespace {
float scale_rc_bipolar(int ch, uint16_t pwm_raw) {
    char name[24];
    std::snprintf(name, sizeof(name), "RC%d_MIN", ch);
    int32_t c_min = ParamManager::get_instance().get_int_or(name, 1000);
    std::snprintf(name, sizeof(name), "RC%d_MAX", ch);
    int32_t c_max = ParamManager::get_instance().get_int_or(name, 2000);
    std::snprintf(name, sizeof(name), "RC%d_TRIM", ch);
    int32_t c_trim = ParamManager::get_instance().get_int_or(name, 1500);
    std::snprintf(name, sizeof(name), "RC%d_REV", ch);
    int32_t c_rev = ParamManager::get_instance().get_int_or(name, 1);
    std::snprintf(name, sizeof(name), "RC%d_DZ", ch);
    int32_t c_dz = ParamManager::get_instance().get_int_or(name, 30);

    float val = 0.0f;
    int32_t pwm = static_cast<int32_t>(pwm_raw);
    if (pwm > c_trim + c_dz) {
        float span = static_cast<float>(c_max - (c_trim + c_dz));
        if (span > 1.0f) {
            val = static_cast<float>(pwm - (c_trim + c_dz)) / span;
        }
    } else if (pwm < c_trim - c_dz) {
        float span = static_cast<float>((c_trim - c_dz) - c_min);
        if (span > 1.0f) {
            val = static_cast<float>(pwm - (c_trim - c_dz)) / span;
        }
    }
    val = std::clamp(val, -1.0f, 1.0f);
    if (c_rev < 0) {
        val = -val;
    }
    return val;
}

float scale_rc_unipolar(int ch, uint16_t pwm_raw) {
    char name[24];
    std::snprintf(name, sizeof(name), "RC%d_MIN", ch);
    int32_t c_min = ParamManager::get_instance().get_int_or(name, 1000);
    std::snprintf(name, sizeof(name), "RC%d_MAX", ch);
    int32_t c_max = ParamManager::get_instance().get_int_or(name, 2000);
    std::snprintf(name, sizeof(name), "RC%d_REV", ch);
    int32_t c_rev = ParamManager::get_instance().get_int_or(name, 1);
    std::snprintf(name, sizeof(name), "RC%d_DZ", ch);
    int32_t c_dz = ParamManager::get_instance().get_int_or(name, 30);

    int32_t pwm = static_cast<int32_t>(pwm_raw);
    float val = 0.0f;
    if (pwm > c_min + c_dz) {
        float span = static_cast<float>(c_max - (c_min + c_dz));
        if (span > 1.0f) {
            val = static_cast<float>(pwm - (c_min + c_dz)) / span;
        }
    }
    val = std::clamp(val, 0.0f, 1.0f);
    if (c_rev < 0) {
        val = 1.0f - val;
    }
    return val;
}
} // namespace

void MavlinkHandler::handle_manual_control(const mavlink_message_t& msg) {
    mavlink_manual_control_t mc;
    mavlink_msg_manual_control_decode(&msg, &mc);
    if (mc.target != 0 && mc.target != m_sysid) return;

    int32_t rev_roll  = ParamManager::get_instance().get_int_or("RC1_REV", 1);
    int32_t rev_pitch = ParamManager::get_instance().get_int_or("RC2_REV", 1);
    int32_t rev_thr   = ParamManager::get_instance().get_int_or("RC3_REV", 1);
    int32_t rev_yaw   = ParamManager::get_instance().get_int_or("RC4_REV", 1);

    LockGuardType lock(m_rc_mutex);
    // mc.x: pitch, +1000 = forward (pitch down/forward)
    // mc.y: roll, +1000 = right
    // mc.r: yaw, +1000 = clockwise
    // mc.z: throttle, 0..1000 (or -1000..1000 depending on stick configuration)
    m_rc_channels.pitch = (rev_pitch < 0 ? -1.0f : 1.0f) * std::clamp(static_cast<float>(mc.x) / 1000.0f, -1.0f, 1.0f);
    m_rc_channels.roll  = (rev_roll < 0 ? -1.0f : 1.0f) * std::clamp(static_cast<float>(mc.y) / 1000.0f, -1.0f, 1.0f);
    m_rc_channels.yaw   = (rev_yaw < 0 ? -1.0f : 1.0f) * std::clamp(static_cast<float>(mc.r) / 1000.0f, -1.0f, 1.0f);

    float z = static_cast<float>(mc.z);
    float thr = 0.0f;
    if (z < 0.0f) {
        thr = std::clamp((z + 1000.0f) / 2000.0f, 0.0f, 1.0f);
    } else {
        thr = std::clamp(z / 1000.0f, 0.0f, 1.0f);
    }
    if (rev_thr < 0) {
        thr = 1.0f - thr;
    }
    m_rc_channels.throttle = thr;
    m_rc_channels.valid = true;
    m_last_rc_update_us = static_cast<uint64_t>(get_time_ms()) * 1000ULL;
}

void MavlinkHandler::handle_rc_channels_override(const mavlink_message_t& msg) {
    mavlink_rc_channels_override_t rc;
    mavlink_msg_rc_channels_override_decode(&msg, &rc);
    if (rc.target_system != 0 && rc.target_system != m_sysid) return;

    LockGuardType lock(m_rc_mutex);
    bool any_override = false;
    if (rc.chan1_raw > 0 && rc.chan1_raw != UINT16_MAX) {
        m_rc_channels.roll = scale_rc_bipolar(1, rc.chan1_raw);
        any_override = true;
    } else {
        m_rc_channels.roll = 0.0f;
    }
    if (rc.chan2_raw > 0 && rc.chan2_raw != UINT16_MAX) {
        m_rc_channels.pitch = scale_rc_bipolar(2, rc.chan2_raw);
        any_override = true;
    } else {
        m_rc_channels.pitch = 0.0f;
    }
    if (rc.chan3_raw > 0 && rc.chan3_raw != UINT16_MAX) {
        m_rc_channels.throttle = scale_rc_unipolar(3, rc.chan3_raw);
        any_override = true;
    } else {
        m_rc_channels.throttle = 0.0f;
    }
    if (rc.chan4_raw > 0 && rc.chan4_raw != UINT16_MAX) {
        m_rc_channels.yaw = scale_rc_bipolar(4, rc.chan4_raw);
        any_override = true;
    } else {
        m_rc_channels.yaw = 0.0f;
    }
    m_rc_channels.valid = any_override;
    m_last_rc_update_us = static_cast<uint64_t>(get_time_ms()) * 1000ULL;
}

MavlinkHandler::RcChannels MavlinkHandler::get_rc_channels() const {
    LockGuardType lock(m_rc_mutex);
    uint64_t now_us = static_cast<uint64_t>(const_cast<MavlinkHandler*>(this)->get_time_ms()) * 1000ULL;
    // 1-second failsafe timeout if GCS stops streaming stick inputs
    if (now_us > m_last_rc_update_us + 1000000ULL) {
        RcChannels neutral;
        neutral.throttle = 0.5f;
        neutral.valid = false;
        return neutral;
    }
    return m_rc_channels;
}

} // namespace apx::gcs
