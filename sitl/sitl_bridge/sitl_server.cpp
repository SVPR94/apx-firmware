// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "sitl_server.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>

namespace apx {
namespace sitl {

SitlServer::SitlServer() {}

SitlServer::~SitlServer() {
    stop();
}

bool SitlServer::init(uint16_t udp_port, uint16_t tcp_port) {
    m_udp_port = udp_port;
    
    m_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udp_socket < 0) {
        std::cerr << "Failed to create UDP socket" << std::endl;
        return false;
    }

    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(m_udp_port);

    if (bind(m_udp_socket, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind UDP socket to port " << m_udp_port << std::endl;
        close(m_udp_socket);
        return false;
    }

    // Non-blocking + polled with a timeout (see udp_listen_loop) so stop()
    // can actually terminate this thread. A plain blocking recvfrom() here
    // hangs forever until the first Gazebo physics packet arrives; closing
    // the fd from another thread (as stop() used to rely on) does not
    // reliably wake a thread blocked in a syscall on that fd on Linux, so
    // every shutdown before Gazebo connects would hang and leave the
    // process stuck holding UDP 9003 (and the MAVLink ports) - the next
    // `sim_vehicle.py` launch would then silently fail to bind them, which
    // looks identical to "no response from the flight controller".
    int flags = fcntl(m_udp_socket, F_GETFL, 0);
    fcntl(m_udp_socket, F_SETFL, flags | O_NONBLOCK);

    return true;
}

void SitlServer::start() {
    if (m_running) return;
    m_running = true;
    m_listen_thread = std::thread(&SitlServer::udp_listen_loop, this);
}

void SitlServer::stop() {
    if (!m_running) return;
    m_running = false;
    if (m_udp_socket >= 0) {
        close(m_udp_socket);
        m_udp_socket = -1;
    }
    if (m_listen_thread.joinable()) {
        m_listen_thread.join();
    }
}

bool SitlServer::get_latest_physics_state(SitlPhysicsState& state) {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    state = m_latest_state;
    return m_client_connected;
}

void SitlServer::send_actuator_output(const SitlActuatorOutput& output) {
    if (m_udp_socket < 0 || !m_client_connected) return;
    
    sendto(m_udp_socket, &output, sizeof(output), 0, 
           (const struct sockaddr *)&m_client_addr, sizeof(m_client_addr));
}

void SitlServer::udp_listen_loop() {
    while (m_running) {
        struct pollfd pfd{m_udp_socket, POLLIN, 0};
        // Short timeout so m_running is re-checked promptly on stop(),
        // instead of blocking indefinitely waiting for Gazebo.
        int pret = poll(&pfd, 1, 100);
        if (pret <= 0 || !(pfd.revents & POLLIN)) {
            continue;
        }

        SitlPhysicsState state{};
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        ssize_t n = recvfrom(m_udp_socket, &state, sizeof(state), 0,
                             (struct sockaddr *)&client_addr, &client_len);

        if (n == sizeof(SitlPhysicsState) || n == 225) {
            if (n == 225) {
                for (int i = 0; i < 10; ++i) state.radar_distances[i] = 0.0f;
                state.radar_valid_mask = 0;
            }
            constexpr auto kPeerTimeout = std::chrono::milliseconds(1000);
            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(m_state_mutex);
            const bool same_peer = m_client_connected &&
                                   client_addr.sin_addr.s_addr == m_client_addr.sin_addr.s_addr &&
                                   client_addr.sin_port == m_client_addr.sin_port;
            if (m_client_connected && !same_peer && now - m_last_peer_rx < kPeerTimeout) {
                ++m_foreign_dropped;
                if (now - m_last_foreign_warn > std::chrono::seconds(5)) {
                    m_last_foreign_warn = now;
                    char ip[INET_ADDRSTRLEN] = {0};
                    inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip));
                    std::cerr << "[APX SITL] WARNING: a second simulator is sending physics to this port from "
                              << ip << ":" << ntohs(client_addr.sin_port) << " (" << m_foreign_dropped
                              << " packets ignored). Another SITL/physics runner is still running - stop it."
                              << std::endl;
                }
                continue;
            }
            if (m_client_connected && !same_peer) {
                std::cerr << "[APX SITL] Physics peer changed (previous one silent); latching new sender."
                          << std::endl;
            }
            m_latest_state = state;
            m_client_addr = client_addr;
            m_client_connected = true;
            m_last_peer_rx = now;
        } else if (n > 0) {
            std::cerr << "Received wrong size struct: " << n << std::endl;
        }
    }
}

} // namespace sitl
} // namespace apx
