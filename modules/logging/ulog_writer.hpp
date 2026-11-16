// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <fstream>

namespace apx::logging {

#pragma pack(push, 1)
struct ULogHeader {
    uint8_t magic[8] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35};
    uint64_t timestamp = 0;
};
#pragma pack(pop)

class ULogWriter {
public:
    ULogWriter();
    ~ULogWriter();

    bool open(const std::string& filename);
    void close();
    
    void write_format(const std::string& name, const std::string& format_str);
    void write_data(uint16_t msg_id, const uint8_t* data, size_t length);
    void update();

    // Data logging endpoints
    void log_imu(uint64_t timestamp, float accel[3], float gyro[3]);
    void log_baro(uint64_t timestamp, float pressure, float temp);
    void log_gps(uint64_t timestamp, int32_t lat, int32_t lon, int32_t alt);
    void log_attitude(uint64_t timestamp, float q[4]);
    void log_battery(uint64_t timestamp, float voltage, float current);

private:
    std::ofstream m_file;
    std::mutex m_mutex;
    bool m_is_open = false;

    std::vector<uint8_t> m_ring_buffer;
    size_t m_head = 0;
    size_t m_tail = 0;

    void write_to_buffer(const uint8_t* data, size_t len);
    void flush_buffer();
};

} // namespace apx::logging
