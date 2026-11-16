// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// Host test for drivers/iomcu (IO coprocessor driver) against a simulated IO MCU.
// Run: tests/iomcu/run_tests.sh

#include <cstdio>
#include <cstring>
#include <vector>

#include "io_coprocessor.hpp"

using apx::drivers::IoCoprocessor;
namespace iocp = apx::drivers::iocp;

static int g_fail = 0;
#define CHECK(name, cond) do { bool ok_ = (cond); std::printf("  %s  %s\n", ok_ ? "PASS" : "FAIL", name); if (!ok_) ++g_fail; } while (0)

// Simulated IO MCU: answers register reads/writes like the real one.
class SimIo final : public apx::hal::IHalUart {
public:
    uint16_t version = 4;
    bool silent = false;
    bool corrupt = false;
    bool safety_off = true;
    uint16_t rc[16] = {1500, 1510, 1000, 1520, 1100, 1900, 0};
    std::vector<uint8_t> rx;              // bytes for the driver to read
    std::vector<std::vector<uint8_t>> requests;

    bool init(uint32_t) override { return true; }
    void deinit() override {}
    int available() const override { return static_cast<int>(rx.size()); }
    void flush() override {}
    void set_rx_callback(RxCallback) override {}
    int read(uint8_t* buf, size_t max) override {
        const size_t n = rx.size() < max ? rx.size() : max;
        std::memcpy(buf, rx.data(), n);
        rx.erase(rx.begin(), rx.begin() + static_cast<long>(n));
        return static_cast<int>(n);
    }
    int write(const uint8_t* buf, size_t len) override {
        requests.emplace_back(buf, buf + len);
        if (silent) return static_cast<int>(len);
        const uint8_t count = buf[0] & 0x3F, code = buf[0] >> 6, page = buf[2];
        std::vector<uint8_t> chk(buf, buf + len);
        chk[1] = 0;
        if (iocp::crc8(chk.data(), chk.size()) != buf[1]) return static_cast<int>(len);
        uint8_t regs[44] = {0};
        if (code == 0 && page == 0) {           // config
            regs[0] = version & 0xFF; regs[1] = version >> 8; regs[2] = 10;
            regs[4] = 0x10; regs[5] = 0x04; regs[6] = 0x41; regs[7] = 0x20;  // mcu id 0x20410410
        } else if (code == 0 && page == 1) {    // status
            regs[8] = 0xE8; regs[9] = 0x03;     // 1000 ms
            regs[12] = 0x88; regs[13] = 0x13;   // 5000 mV
            regs[32] = safety_off ? 1 : 0;
        } else if (code == 0 && page == 4) {    // rc input
            regs[0] = 6; regs[1] = 0x02; regs[2] = 3;
            for (int i = 0; i < 16; ++i) { regs[4 + 2 * i] = rc[i] & 0xFF; regs[5 + 2 * i] = rc[i] >> 8; }
            regs[36] = 200;
        }
        const uint8_t reply_count = code == 0 ? count : 0;
        std::vector<uint8_t> pkt = {reply_count, 0, page, buf[3]};   // code bits 6-7 = 0 (success)
        pkt.insert(pkt.end(), regs, regs + 2 * reply_count);
        pkt[1] = iocp::crc8(pkt.data(), pkt.size());
        if (corrupt) pkt[1] ^= 0x5A;
        rx.insert(rx.end(), pkt.begin(), pkt.end());
        return static_cast<int>(len);
    }
};

static void run(IoCoprocessor& io, uint32_t& t, uint32_t ms) {
    for (uint32_t end = t + ms; t < end; t += 1) io.update(t);
}

int main() {
    std::printf("wire format:\n");
    const char* check = "123456789";
    CHECK("CRC-8 (poly 0x07) check value 0xF4", iocp::crc8(reinterpret_cast<const uint8_t*>(check), 9) == 0xF4);
    struct Hdr { uint8_t count : 6; uint8_t code : 2; } h{};
    h.count = 5; h.code = 1;
    uint8_t hb; std::memcpy(&hb, &h, 1);
    uint8_t pkt[48];
    const uint16_t regs[2] = {0x1234, 0xABCD};
    size_t n = iocp::encode(pkt, iocp::kWrite, 54, 0, regs, 2);
    CHECK("header byte = count | code << 6 (matches C bitfield layout)", pkt[0] == 0x42 && hb == 0x45);
    CHECK("write packet: 4 + 2*count bytes, little-endian registers",
          n == 8 && pkt[2] == 54 && pkt[4] == 0x34 && pkt[5] == 0x12 && pkt[6] == 0xCD && pkt[7] == 0xAB);
    n = iocp::encode(pkt, iocp::kRead, 4, 0, nullptr, 19);
    CHECK("read request has the reply size, zeroed registers", n == 42 && pkt[0] == 19 && pkt[10] == 0);

    std::printf("driver:\n");
    SimIo sim;
    IoCoprocessor io;
    uint32_t t = 1;
    io.begin(nullptr);
    CHECK("no IO link -> Off (boards without an IO MCU)", io.state() == IoCoprocessor::State::Off);
    io.begin(&sim);
    run(io, t, 5);
    CHECK("probe -> Ready, protocol 4", io.state() == IoCoprocessor::State::Ready && io.protocol_version() == 4);
    CHECK("mcu id parsed", io.mcu_id() == 0x20410410u && io.protocol_version2() == 10);
    run(io, t, 200);
    CHECK("RC input parsed", io.rc().valid && io.rc().channels == 6 && io.rc().pwm[0] == 1500 &&
                             io.rc().pwm[5] == 1900 && io.rc().protocol == 3 && io.rc().rssi == 200);
    CHECK("status parsed (safety off, servo rail 5.0 V)", io.status().safety_off &&
                                                          io.status().servo_rail_mv == 5000 && io.status().io_time_ms == 1000);
    bool wrote_outputs = false;
    for (auto& r : sim.requests) wrote_outputs |= ((r[0] >> 6) == 1 && r[2] == 54);
    CHECK("no output writes until enabled (bring-up safety)", !wrote_outputs);
    const uint16_t out[4] = {1100, 1200, 1300, 1400};
    io.set_outputs(out, 4);
    io.enable_outputs(true);
    sim.requests.clear();
    run(io, t, 20);
    wrote_outputs = false;
    for (auto& r : sim.requests) wrote_outputs |= ((r[0] >> 6) == 1 && r[2] == 54 && (r[0] & 0x3F) == 4 && r[4] == (1100 & 0xFF));
    CHECK("outputs written to the direct PWM page once enabled", wrote_outputs);
    io.enable_outputs(false);

    sim.corrupt = true;
    const uint32_t f0 = io.failures();
    run(io, t, 100);
    CHECK("corrupt replies rejected (CRC)", io.failures() > f0);
    sim.corrupt = false;
    sim.silent = true;
    run(io, t, 1000);
    CHECK("silent IO MCU -> Lost, RC invalid", io.state() == IoCoprocessor::State::Lost && !io.rc().valid);
    sim.silent = false;
    run(io, t, 1000);
    CHECK("recovers to Ready when it answers again", io.state() == IoCoprocessor::State::Ready);

    SimIo other;
    other.version = 7;
    IoCoprocessor io2;
    io2.begin(&other);
    uint32_t t2 = 1;
    run(io2, t2, 10);
    CHECK("other protocol version -> Unsupported (no RC/status polling)", io2.state() == IoCoprocessor::State::Unsupported);

    std::printf(g_fail ? "FAILED (%d)\n" : "ALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
