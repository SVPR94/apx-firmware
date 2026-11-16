#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX driver: IO coprocessor (IO MCU).
//
// Some flight controllers (Cube Orange/Orange+, Pixhawk 6X/6C, Durandal,
// SIYI N7, ...) route their MAIN PWM outputs, RC input and safety switch to a
// small separate MCU. The main MCU talks to it over a 1.5 Mbaud UART with a
// register protocol: each transaction reads or writes up to 22 16-bit
// registers in a page. This driver implements the main-MCU side, compatible
// with the IO firmware these boards ship with (protocol version 4).
//
// Boards without an IO MCU simply have no IO link (apx::board::iomcu_link()
// returns nullptr) and never construct or start this driver.
//
// Non-blocking: update() is called from the flight loop every tick; at most
// one transaction is in flight, so the loop never waits on the UART.

#include <cstddef>
#include <cstdint>

#include "apx/hal/hal_uart.hpp"

namespace apx::drivers {

namespace iocp {
constexpr uint32_t kBaud = 1500000;
constexpr size_t kMaxRegs = 22;
constexpr size_t kHeader = 4;  // count/code, crc, page, offset

// first byte: bits 0-5 register count, bits 6-7 code
enum Code : uint8_t { kRead = 0, kWrite = 1 };
enum Reply : uint8_t { kSuccess = 0, kCorrupt = 1, kError = 2 };

enum Page : uint8_t {
    kPageConfig = 0,
    kPageStatus = 1,
    kPageServos = 3,
    kPageRawRcInput = 4,
    kPageSetup = 50,
    kPageDirectPwm = 54,
    kPageFailsafePwm = 55,
};

constexpr uint16_t kProtocolVersion = 4;

// Register counts of the pages read by this driver
constexpr uint8_t kConfigRegs = 6;   // version, version2, mcu id (2), cpu id (2)
constexpr uint8_t kStatusRegs = 22;
constexpr uint8_t kRcRegs = 19;

// CRC-8, polynomial 0x07, initial value 0, computed with the crc byte zeroed.
uint8_t crc8(const uint8_t* data, size_t len);

// Build a request into out (>= kHeader + 2 * kMaxRegs bytes); returns its length.
// Reads carry zeroed registers so the request has the size of the reply.
size_t encode(uint8_t* out, Code code, uint8_t page, uint8_t offset, const uint16_t* regs, uint8_t count);
} // namespace iocp

struct IoStatus {
    bool safety_off{false};     // safety switch pressed: outputs may be enabled
    uint16_t servo_rail_mv{0};  // servo rail voltage
    uint32_t io_time_ms{0};     // IO MCU uptime
    uint32_t io_errors{0};      // protocol errors counted by the IO MCU
};

struct IoRcInput {
    uint8_t channels{0};
    bool failsafe{true};
    bool valid{false};
    uint8_t protocol{0};
    uint16_t pwm[16]{};
    int16_t rssi{0};
    uint32_t updated_ms{0};
};

class IoCoprocessor {
public:
    enum class State : uint8_t {
        Off,          // no IO link on this board
        Probing,      // waiting for the IO MCU to answer
        Ready,        // talking, protocol supported
        Unsupported,  // answers, but with another protocol version
        Lost,         // was ready, stopped answering (re-probing)
    };

    void begin(apx::hal::IHalUart* link);
    void update(uint32_t now_ms);

    State state() const { return m_state; }
    static const char* state_name(State s);
    uint16_t protocol_version() const { return m_version; }
    uint16_t protocol_version2() const { return m_version2; }
    uint32_t mcu_id() const { return m_mcu_id; }
    const IoStatus& status() const { return m_status; }
    const IoRcInput& rc() const { return m_rc; }
    uint32_t transactions() const { return m_ok; }
    uint32_t failures() const { return m_failures; }

    // Main outputs. Not written until enable_outputs(true): during bring-up
    // the IO MCU keeps its own disarmed/failsafe output values.
    void set_outputs(const uint16_t* pwm_us, uint8_t count);
    void enable_outputs(bool enable) { m_outputs_enabled = enable; }

private:
    enum class Job : uint8_t { None, Config, Status, Rc, Outputs };

    void start(Job job, iocp::Code code, uint8_t page, uint8_t count, const uint16_t* regs = nullptr);
    void finish(bool ok);
    void handle_reply();

    apx::hal::IHalUart* m_link{nullptr};
    State m_state{State::Off};
    Job m_job{Job::None};
    uint8_t m_expect_regs{0};
    uint32_t m_job_start_ms{0};
    uint32_t m_now_ms{0};
    uint8_t m_rx[iocp::kHeader + 2 * iocp::kMaxRegs];
    size_t m_rx_len{0};

    uint32_t m_next_probe_ms{0};
    uint32_t m_last_status_ms{0};
    uint32_t m_last_rc_ms{0};
    uint32_t m_last_out_ms{0};
    uint32_t m_consecutive_failures{0};
    uint32_t m_ok{0};
    uint32_t m_failures{0};

    uint16_t m_version{0};
    uint16_t m_version2{0};
    uint32_t m_mcu_id{0};
    IoStatus m_status;
    IoRcInput m_rc;

    uint16_t m_out[8]{};
    uint8_t m_out_count{0};
    bool m_outputs_enabled{false};
};

} // namespace apx::drivers
