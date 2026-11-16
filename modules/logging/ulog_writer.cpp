// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "ulog_writer.hpp"
#include <cstring>
#include <iostream>
#include <chrono>

namespace apx::logging {

ULogWriter::ULogWriter() {
    m_ring_buffer.resize(32768); // 32KB ring buffer
}

ULogWriter::~ULogWriter() {
    close();
}

bool ULogWriter::open(const std::string& filename) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_file.open(filename, std::ios::binary | std::ios::trunc);
    if (!m_file.is_open()) {
        return false;
    }
    m_is_open = true;

    ULogHeader hdr;
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    hdr.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    m_file.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));

    // Register basic formats
    write_format("sensor_imu", "uint64_t timestamp;float[3] accel;float[3] gyro;");
    write_format("sensor_baro", "uint64_t timestamp;float pressure;float temperature;");
    write_format("vehicle_gps", "uint64_t timestamp;int32_t lat;int32_t lon;int32_t alt;");
    write_format("vehicle_attitude", "uint64_t timestamp;float[4] q;");
    write_format("battery_status", "uint64_t timestamp;float voltage_v;float current_a;");

    return true;
}

void ULogWriter::close() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_is_open) {
        flush_buffer();
        m_file.close();
        m_is_open = false;
    }
}

void ULogWriter::write_format(const std::string& name, const std::string& format_str) {
    if (!m_is_open) return;
    
    std::string full_str = name + ":" + format_str;
    uint16_t msg_size = full_str.length();
    
    uint8_t header[3] = {static_cast<uint8_t>(msg_size & 0xFF), static_cast<uint8_t>(msg_size >> 8), 'F'};
    
    std::lock_guard<std::mutex> lock(m_mutex);
    write_to_buffer(header, 3);
    write_to_buffer(reinterpret_cast<const uint8_t*>(full_str.c_str()), msg_size);
}

void ULogWriter::write_data(uint16_t msg_id, const uint8_t* data, size_t length) {
    if (!m_is_open) return;
    
    uint16_t msg_size = length + 2;
    uint8_t header[3] = {static_cast<uint8_t>(msg_size & 0xFF), static_cast<uint8_t>(msg_size >> 8), 'D'};
    uint8_t id_buf[2] = {static_cast<uint8_t>(msg_id & 0xFF), static_cast<uint8_t>(msg_id >> 8)};
    
    std::lock_guard<std::mutex> lock(m_mutex);
    write_to_buffer(header, 3);
    write_to_buffer(id_buf, 2);
    write_to_buffer(data, length);
}

void ULogWriter::write_to_buffer(const uint8_t* data, size_t len) {
    size_t capacity = m_ring_buffer.size();
    size_t available = capacity - (m_head - m_tail);
    if (len > available) {
        // Buffer overflow, drop data
        return;
    }

    for (size_t i = 0; i < len; ++i) {
        m_ring_buffer[m_head % capacity] = data[i];
        m_head++;
    }
}

void ULogWriter::flush_buffer() {
    size_t capacity = m_ring_buffer.size();
    while (m_tail < m_head) {
        size_t chunk = std::min(m_head - m_tail, capacity - (m_tail % capacity));
        m_file.write(reinterpret_cast<const char*>(&m_ring_buffer[m_tail % capacity]), chunk);
        m_tail += chunk;
    }
}

void ULogWriter::update() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_is_open) {
        flush_buffer();
    }
}

// Stub log functions. Assume message IDs: IMU=0, BARO=1, GPS=2, ATT=3, BATT=4
void ULogWriter::log_imu(uint64_t timestamp, float accel[3], float gyro[3]) {
    struct { uint64_t t; float a[3]; float g[3]; } __attribute__((packed)) d = {timestamp, {accel[0], accel[1], accel[2]}, {gyro[0], gyro[1], gyro[2]}};
    write_data(0, reinterpret_cast<const uint8_t*>(&d), sizeof(d));
}

void ULogWriter::log_baro(uint64_t timestamp, float pressure, float temp) {
    struct { uint64_t t; float p; float t2; } __attribute__((packed)) d = {timestamp, pressure, temp};
    write_data(1, reinterpret_cast<const uint8_t*>(&d), sizeof(d));
}

void ULogWriter::log_gps(uint64_t timestamp, int32_t lat, int32_t lon, int32_t alt) {
    struct { uint64_t t; int32_t lat; int32_t lon; int32_t alt; } __attribute__((packed)) d = {timestamp, lat, lon, alt};
    write_data(2, reinterpret_cast<const uint8_t*>(&d), sizeof(d));
}

void ULogWriter::log_attitude(uint64_t timestamp, float q[4]) {
    struct { uint64_t t; float q[4]; } __attribute__((packed)) d = {timestamp, {q[0], q[1], q[2], q[3]}};
    write_data(3, reinterpret_cast<const uint8_t*>(&d), sizeof(d));
}

void ULogWriter::log_battery(uint64_t timestamp, float voltage, float current) {
    struct { uint64_t t; float v; float c; } __attribute__((packed)) d = {timestamp, voltage, current};
    write_data(4, reinterpret_cast<const uint8_t*>(&d), sizeof(d));
}

} // namespace apx::logging
