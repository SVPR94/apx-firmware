// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <mavlink.h>
#include <cstdint>

namespace apx::gcs {

class MavlinkHandler;

class CommandHandler {
public:
    explicit CommandHandler(MavlinkHandler* mavlink);
    ~CommandHandler() = default;

    void handle_command(const mavlink_message_t& msg);

private:
    MavlinkHandler* m_mavlink;

    void process_command_long(const mavlink_command_long_t& cmd);
    void process_command_int(const mavlink_command_int_t& cmd);
    
    uint8_t execute_command(uint16_t command, float param1, float param2, float param3, float param4, double param5, double param6, float param7);
};

} // namespace apx::gcs
