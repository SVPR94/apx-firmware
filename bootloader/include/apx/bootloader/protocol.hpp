#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX bootloader - serial upload protocol (revision 5).
//
// Speaks the bootloader protocol that ground stations use to flash .apj
// firmware files (APX Flight Manager and standard ground control stations and the common command-
// line uploader). See bootloader/README.md for the command reference.
//
// Every command ends with kEoc. Every reply is the command's payload (if
// any) followed by kInSync and a status byte (kOk / kFailed / kInvalid).

#include <cstddef>
#include <cstdint>

#include "apx/bootloader/port.hpp"

namespace apx::bootloader {

namespace proto {
constexpr uint8_t kInSync = 0x12;
constexpr uint8_t kEoc = 0x20;
constexpr uint8_t kOk = 0x10;
constexpr uint8_t kFailed = 0x11;
constexpr uint8_t kInvalid = 0x13;

constexpr uint8_t kNop = 0x00;
constexpr uint8_t kGetSync = 0x21;
constexpr uint8_t kGetDevice = 0x22;
constexpr uint8_t kChipErase = 0x23;
constexpr uint8_t kChipVerify = 0x24;
constexpr uint8_t kProgMulti = 0x27;
constexpr uint8_t kReadMulti = 0x28;
constexpr uint8_t kGetCrc = 0x29;
constexpr uint8_t kGetOtp = 0x2a;
constexpr uint8_t kGetSn = 0x2b;
constexpr uint8_t kGetChip = 0x2c;
constexpr uint8_t kSetBootDelay = 0x2d;
constexpr uint8_t kGetChipDes = 0x2e;
constexpr uint8_t kGetSoftware = 0x2f;
constexpr uint8_t kReboot = 0x30;
constexpr uint8_t kSetBaud = 0x33;
constexpr uint8_t kExtfErase = 0x34;
constexpr uint8_t kExtfProgMulti = 0x35;
constexpr uint8_t kExtfReadMulti = 0x36;
constexpr uint8_t kExtfGetCrc = 0x37;
constexpr uint8_t kChipFullErase = 0x40;

// GET_DEVICE parameters
constexpr uint8_t kInfoProtocolRev = 1;
constexpr uint8_t kInfoBoardId = 2;
constexpr uint8_t kInfoBoardRev = 3;
constexpr uint8_t kInfoAppSize = 4;
constexpr uint8_t kInfoExtFlashSize = 6;

constexpr uint32_t kProtocolRevision = 5;
constexpr size_t kMaxProgramUnit = 64;
} // namespace proto

// CRC used by GET_CRC: CRC-32 (reflected polynomial 0xEDB88320), initial
// value 0, no final inversion. Chain calls by passing the previous result.
uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc = 0);

class Protocol {
public:
    enum class Result {
        Boot,     // host sent REBOOT: start the application
        Timeout,  // no (more) host activity within the timeout
    };

    Protocol(Port& port, const Identity& identity);

    // Serve commands until REBOOT or a timeout.
    //   first_contact_ms  give up if no valid command arrives this long (0 = wait forever)
    //   idle_ms           after contact, give up after this long without a command (0 = never)
    Result run(uint32_t first_contact_ms, uint32_t idle_ms);

    // True once any valid command was received.
    bool host_seen() const { return m_host_seen; }

private:
    bool handle(uint8_t cmd, bool& boot);
    bool recv(uint8_t& byte, uint32_t timeout_ms);
    bool recv_n(uint8_t* buf, size_t n);
    bool recv_word(uint32_t& word);
    bool eoc();
    void status(uint8_t code);
    void word_reply(uint32_t value);
    void text_reply(const char* text);

    void stage(const uint8_t* data, size_t len, bool& ok);
    bool write_unit();
    bool finish_programming();
    void read_image(uint32_t offset, uint8_t* out, size_t len);
    uint32_t image_crc();

    Port& m_port;
    Identity m_id;
    size_t m_unit;
    bool m_host_seen{false};

    bool m_erased{false};
    uint32_t m_prog_addr{0};
    uint8_t m_pending[proto::kMaxProgramUnit];
    size_t m_pending_fill{0};
    uint32_t m_pending_addr{0};
    // The first program unit (initial stack pointer + reset vector) is held
    // back and written last, at REBOOT, so an interrupted upload leaves an
    // image the bootloader recognises as invalid and never starts.
    uint8_t m_first[proto::kMaxProgramUnit];
    bool m_first_held{false};
    uint32_t m_read_addr{0};
};

} // namespace apx::bootloader
