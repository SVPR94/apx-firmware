// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX driver: IO coprocessor (IO MCU) - main-MCU side of the register protocol.

#include "io_coprocessor.hpp"

#include <cstring>

namespace apx::drivers {

namespace iocp {

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = static_cast<uint8_t>((crc & 0x80u) ? ((crc << 1) ^ 0x07u) : (crc << 1));
        }
    }
    return crc;
}

size_t encode(uint8_t* out, Code code, uint8_t page, uint8_t offset, const uint16_t* regs, uint8_t count) {
    if (count > kMaxRegs) count = static_cast<uint8_t>(kMaxRegs);
    out[0] = static_cast<uint8_t>((count & 0x3Fu) | (static_cast<uint8_t>(code) << 6));
    out[1] = 0;
    out[2] = page;
    out[3] = offset;
    for (uint8_t i = 0; i < count; ++i) {
        const uint16_t v = (regs != nullptr && code == kWrite) ? regs[i] : 0;
        out[kHeader + 2 * i] = static_cast<uint8_t>(v);
        out[kHeader + 2 * i + 1] = static_cast<uint8_t>(v >> 8);
    }
    const size_t len = kHeader + 2u * count;
    out[1] = crc8(out, len);
    return len;
}

} // namespace iocp

namespace {

constexpr uint32_t kTimeoutMs = 10;
constexpr uint32_t kProbeIntervalMs = 500;
constexpr uint32_t kRcIntervalMs = 20;       // 50 Hz
constexpr uint32_t kStatusIntervalMs = 50;   // 20 Hz
constexpr uint32_t kOutputIntervalMs = 5;    // 200 Hz
constexpr uint32_t kLostAfterFailures = 20;

uint16_t le16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t le32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// Byte offsets inside the register pages (little-endian, natural alignment).
namespace status_off {
constexpr size_t kTimestampMs = 8;
constexpr size_t kServoRailMv = 12;
constexpr size_t kErrors = 16;
constexpr size_t kSafetyOff = 32;
} // namespace status_off
namespace rc_off {
constexpr size_t kCount = 0;
constexpr size_t kFlags = 1;     // bit 0 failsafe, bit 1 valid
constexpr size_t kProtocol = 2;
constexpr size_t kPwm = 4;       // 16 x u16
constexpr size_t kRssi = 36;
} // namespace rc_off

} // namespace

const char* IoCoprocessor::state_name(State s) {
    switch (s) {
    case State::Off: return "off";
    case State::Probing: return "probing";
    case State::Ready: return "ready";
    case State::Unsupported: return "unsupported protocol";
    case State::Lost: return "not responding";
    }
    return "?";
}

void IoCoprocessor::begin(apx::hal::IHalUart* link) {
    m_link = link;
    m_state = link != nullptr ? State::Probing : State::Off;
    m_job = Job::None;
}

void IoCoprocessor::set_outputs(const uint16_t* pwm_us, uint8_t count) {
    m_out_count = count > 8 ? 8 : count;
    std::memcpy(m_out, pwm_us, m_out_count * sizeof(uint16_t));
}

void IoCoprocessor::start(Job job, iocp::Code code, uint8_t page, uint8_t count, const uint16_t* regs) {
    uint8_t junk[64];
    while (m_link->read(junk, sizeof(junk)) > 0) {
        // drop stale bytes from an earlier, timed-out reply
    }
    uint8_t pkt[iocp::kHeader + 2 * iocp::kMaxRegs];
    const size_t len = iocp::encode(pkt, code, page, 0, regs, count);
    if (m_link->write(pkt, len) != static_cast<int>(len)) {
        ++m_failures;
        return;
    }
    m_job = job;
    m_expect_regs = code == iocp::kRead ? count : 0;
    m_job_start_ms = m_now_ms;
    m_rx_len = 0;
}

void IoCoprocessor::finish(bool ok) {
    m_job = Job::None;
    if (ok) {
        ++m_ok;
        m_consecutive_failures = 0;
        return;
    }
    ++m_failures;
    if (++m_consecutive_failures >= kLostAfterFailures && m_state == State::Ready) {
        m_state = State::Lost;
        m_rc.valid = false;
    }
}

void IoCoprocessor::handle_reply() {
    // Validate: complete packet, CRC, success code, enough registers.
    const uint8_t count = m_rx[0] & 0x3Fu;
    const uint8_t code = m_rx[0] >> 6;
    const uint8_t got_crc = m_rx[1];
    m_rx[1] = 0;
    const bool crc_ok = iocp::crc8(m_rx, iocp::kHeader + 2u * count) == got_crc;
    if (!crc_ok || code != iocp::kSuccess || count < m_expect_regs) {
        finish(false);
        return;
    }
    const uint8_t* r = m_rx + iocp::kHeader;
    switch (m_job) {
    case Job::Config:
        m_version = le16(r);
        m_version2 = le16(r + 2);
        m_mcu_id = le32(r + 4);
        m_state = m_version == iocp::kProtocolVersion ? State::Ready : State::Unsupported;
        break;
    case Job::Status:
        m_status.io_time_ms = le32(r + status_off::kTimestampMs);
        m_status.servo_rail_mv = le16(r + status_off::kServoRailMv);
        m_status.io_errors = le32(r + status_off::kErrors);
        m_status.safety_off = r[status_off::kSafetyOff] != 0;
        break;
    case Job::Rc: {
        m_rc.channels = r[rc_off::kCount] > 16 ? 16 : r[rc_off::kCount];
        m_rc.failsafe = (r[rc_off::kFlags] & 0x01u) != 0;
        m_rc.valid = (r[rc_off::kFlags] & 0x02u) != 0 && !m_rc.failsafe;
        m_rc.protocol = r[rc_off::kProtocol];
        for (int i = 0; i < 16; ++i) {
            m_rc.pwm[i] = le16(r + rc_off::kPwm + 2 * i);
        }
        m_rc.rssi = static_cast<int16_t>(le16(r + rc_off::kRssi));
        m_rc.updated_ms = m_now_ms;
        break;
    }
    default:
        break;
    }
    finish(true);
}

void IoCoprocessor::update(uint32_t now_ms) {
    m_now_ms = now_ms;
    if (m_link == nullptr || m_state == State::Off) {
        return;
    }

    if (m_job != Job::None) {
        // Collect the reply: header first (gives the length), then registers.
        const int n = m_link->read(m_rx + m_rx_len, sizeof(m_rx) - m_rx_len);
        if (n > 0) {
            m_rx_len += static_cast<size_t>(n);
        }
        if (m_rx_len >= iocp::kHeader) {
            const size_t want = iocp::kHeader + 2u * (m_rx[0] & 0x3Fu);
            if (m_rx_len >= want) {
                handle_reply();
                return;
            }
        }
        if (now_ms - m_job_start_ms > kTimeoutMs) {
            finish(false);
        }
        return;
    }

    switch (m_state) {
    case State::Probing:
    case State::Lost:
    case State::Unsupported:
        if (now_ms >= m_next_probe_ms) {
            m_next_probe_ms = now_ms + kProbeIntervalMs;
            start(Job::Config, iocp::kRead, iocp::kPageConfig, iocp::kConfigRegs);
        }
        break;
    case State::Ready:
        if (m_outputs_enabled && m_out_count > 0 && now_ms - m_last_out_ms >= kOutputIntervalMs) {
            m_last_out_ms = now_ms;
            start(Job::Outputs, iocp::kWrite, iocp::kPageDirectPwm, m_out_count, m_out);
        } else if (now_ms - m_last_rc_ms >= kRcIntervalMs) {
            m_last_rc_ms = now_ms;
            start(Job::Rc, iocp::kRead, iocp::kPageRawRcInput, iocp::kRcRegs);
        } else if (now_ms - m_last_status_ms >= kStatusIntervalMs) {
            m_last_status_ms = now_ms;
            start(Job::Status, iocp::kRead, iocp::kPageStatus, iocp::kStatusRegs);
        }
        break;
    case State::Off:
        break;
    }
}

} // namespace apx::drivers
