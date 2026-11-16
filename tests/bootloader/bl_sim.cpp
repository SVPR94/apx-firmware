// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// Host simulator for the APX bootloader protocol core.
//
// Runs bootloader/src/protocol.cpp against simulated flash on a pseudo-
// terminal, so a real ground-station uploader can talk to it:
//
//   bl_sim --board-id 1063 --app-size 917504 --image out.bin
//   -> prints the PTY path, serves commands, writes the final flash image
//      to out.bin on REBOOT and exits 0 (exit 2 on timeout).
//
// POSIX only (Linux/macOS); used by tests/bootloader/test_bootloader.py.

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "apx/bootloader/protocol.hpp"

using apx::bootloader::Identity;
using apx::bootloader::Port;
using apx::bootloader::Protocol;

namespace {

class SimPort final : public Port {
public:
    SimPort(int fd, uint32_t app_size, size_t unit) : m_fd(fd), m_flash(app_size, 0xA5), m_unit(unit) {}

    size_t read(uint8_t* buf, size_t max_len) override {
        const ssize_t n = ::read(m_fd, buf, max_len);
        return n > 0 ? static_cast<size_t>(n) : 0;
    }
    void write(const uint8_t* buf, size_t len) override {
        while (len > 0) {
            const ssize_t n = ::write(m_fd, buf, len);
            if (n <= 0) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
            buf += n;
            len -= static_cast<size_t>(n);
        }
    }
    uint32_t millis() override {
        using namespace std::chrono;
        return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
    }
    void idle() override { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }

    bool erase_app() override {
        std::fill(m_flash.begin(), m_flash.end(), 0xFF);
        ++erases;
        return true;
    }
    bool program(uint32_t offset, const uint8_t* data, size_t len) override {
        if (offset % m_unit || len % m_unit || offset + len > m_flash.size()) {
            std::fprintf(stderr, "sim: misaligned program @%u len %zu\n", offset, len);
            return false;
        }
        for (size_t i = 0; i < len; ++i) {
            if (m_flash[offset + i] != 0xFF) { // real flash cannot be rewritten without erase
                std::fprintf(stderr, "sim: program over non-erased byte @%zu\n", offset + i);
                return false;
            }
        }
        std::memcpy(&m_flash[offset], data, len);
        if (offset == 0) first_unit_written_last = true;
        else if (first_unit_written_last) first_unit_written_last = false;
        return true;
    }
    void read_app(uint32_t offset, uint8_t* data, size_t len) override {
        std::memcpy(data, &m_flash[offset], len);
    }
    size_t program_unit() const override { return m_unit; }
    bool serial_word(uint32_t offset, uint32_t& word) override {
        if (offset >= 12) return false;
        word = 0x41505800u + offset;
        return true;
    }

    const std::vector<uint8_t>& flash() const { return m_flash; }
    int erases = 0;
    bool first_unit_written_last = false;

private:
    int m_fd;
    std::vector<uint8_t> m_flash;
    size_t m_unit;
};

} // namespace

int main(int argc, char** argv) {
    uint32_t board_id = 1063, app_size = 917504, timeout_ms = 60000;
    size_t unit = 32;
    std::string image = "sim_flash.bin";
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string a = argv[i];
        if (a == "--board-id") board_id = std::strtoul(argv[i + 1], nullptr, 0);
        else if (a == "--app-size") app_size = std::strtoul(argv[i + 1], nullptr, 0);
        else if (a == "--unit") unit = std::strtoul(argv[i + 1], nullptr, 0);
        else if (a == "--image") image = argv[i + 1];
        else if (a == "--timeout-ms") timeout_ms = std::strtoul(argv[i + 1], nullptr, 0);
    }
    int master = -1, slave = -1;
    char name[128];
    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    cfmakeraw(&tio);
    if (openpty(&master, &slave, name, &tio, nullptr) != 0) {
        std::perror("openpty");
        return 1;
    }
    fcntl(master, F_SETFL, fcntl(master, F_GETFL) | O_NONBLOCK);
    std::printf("%s\n", name);
    std::fflush(stdout);

    SimPort port(master, app_size, unit);
    const Identity id{board_id, 0, app_size, 0x10016450u, "SIMULATED,X", "APX bootloader (host simulator)"};
    Protocol proto(port, id);
    const auto result = proto.run(timeout_ms, timeout_ms);
    FILE* f = std::fopen(image.c_str(), "wb");
    std::fwrite(port.flash().data(), 1, port.flash().size(), f);
    std::fclose(f);
    std::fprintf(stderr, "sim: result=%s erases=%d first_unit_written_last=%d\n",
                 result == Protocol::Result::Boot ? "boot" : "timeout", port.erases,
                 port.first_unit_written_last ? 1 : 0);
    close(slave);
    return result == Protocol::Result::Boot ? 0 : 2;
}
