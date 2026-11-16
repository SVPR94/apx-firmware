// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX bootloader - serial upload protocol core (portable, no OS headers).

#include "apx/bootloader/protocol.hpp"

#include <cstring>

namespace apx::bootloader {

using namespace proto;

namespace {

constexpr uint32_t kByteTimeoutMs = 500;  // between bytes of one command
constexpr uint32_t kPollMs = 10;          // command wait granularity

struct CrcTable {
    uint32_t v[256];
    constexpr CrcTable() : v() {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            v[i] = c;
        }
    }
};
constexpr CrcTable kCrcTable;

void put_le32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

} // namespace

uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc) {
    for (size_t i = 0; i < len; ++i) {
        crc = kCrcTable.v[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc;
}

Protocol::Protocol(Port& port, const Identity& identity)
    : m_port(port), m_id(identity), m_unit(port.program_unit()) {
    if (m_unit < 4 || m_unit > kMaxProgramUnit) {
        m_unit = 4; // misconfigured port: fail safe (programming will be refused)
    }
}

// ---------------------------------------------------------------- I/O helpers

bool Protocol::recv(uint8_t& byte, uint32_t timeout_ms) {
    const uint32_t start = m_port.millis();
    while (true) {
        if (m_port.read(&byte, 1) == 1) {
            return true;
        }
        if (m_port.millis() - start >= timeout_ms) {
            return false;
        }
        m_port.idle();
    }
}

bool Protocol::recv_n(uint8_t* buf, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        if (!recv(buf[i], kByteTimeoutMs)) {
            return false;
        }
    }
    return true;
}

bool Protocol::recv_word(uint32_t& word) {
    uint8_t b[4];
    if (!recv_n(b, 4)) {
        return false;
    }
    word = static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
           (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
    return true;
}

bool Protocol::eoc() {
    uint8_t b;
    return recv(b, kByteTimeoutMs) && b == kEoc;
}

void Protocol::status(uint8_t code) {
    const uint8_t r[2] = {kInSync, code};
    m_port.write(r, sizeof(r));
}

void Protocol::word_reply(uint32_t value) {
    uint8_t r[6];
    put_le32(r, value);
    r[4] = kInSync;
    r[5] = kOk;
    m_port.write(r, sizeof(r));
}

void Protocol::text_reply(const char* text) {
    const uint32_t len = static_cast<uint32_t>(std::strlen(text));
    uint8_t hdr[4];
    put_le32(hdr, len);
    m_port.write(hdr, sizeof(hdr));
    m_port.write(reinterpret_cast<const uint8_t*>(text), len);
    status(kOk);
}

// ---------------------------------------------------------------- programming

bool Protocol::write_unit() {
    bool ok = true;
    if (m_pending_addr == 0) {
        std::memcpy(m_first, m_pending, m_unit); // hold back until REBOOT
        m_first_held = true;
    } else {
        ok = m_port.program(m_pending_addr, m_pending, m_unit);
    }
    m_pending_fill = 0;
    return ok;
}

void Protocol::stage(const uint8_t* data, size_t len, bool& ok) {
    for (size_t i = 0; i < len; ++i) {
        if (m_pending_fill == 0) {
            m_pending_addr = m_prog_addr;
        }
        m_pending[m_pending_fill++] = data[i];
        ++m_prog_addr;
        if (m_pending_fill == m_unit) {
            ok &= write_unit();
        }
    }
}

bool Protocol::finish_programming() {
    bool ok = true;
    if (m_pending_fill > 0) { // last partial unit, padded like erased flash
        std::memset(m_pending + m_pending_fill, 0xFF, m_unit - m_pending_fill);
        ok &= write_unit();
    }
    if (m_first_held) {
        ok &= m_port.program(0, m_first, m_unit);
        m_first_held = false;
    }
    return ok;
}

// The image as it will be once programming finishes: flash contents with
// the held-back first unit and the pending partial unit overlaid.
void Protocol::read_image(uint32_t offset, uint8_t* out, size_t len) {
    m_port.read_app(offset, out, len);
    auto overlay = [&](uint32_t base, const uint8_t* src, size_t n) {
        const uint32_t lo = offset > base ? offset : base;
        const uint32_t hi_a = offset + static_cast<uint32_t>(len);
        const uint32_t hi_b = base + static_cast<uint32_t>(n);
        const uint32_t hi = hi_a < hi_b ? hi_a : hi_b;
        if (lo < hi) {
            std::memcpy(out + (lo - offset), src + (lo - base), hi - lo);
        }
    };
    if (m_first_held) {
        overlay(0, m_first, m_unit);
    }
    if (m_pending_fill > 0) {
        overlay(m_pending_addr, m_pending, m_pending_fill);
    }
}

uint32_t Protocol::image_crc() {
    uint8_t buf[256];
    uint32_t crc = 0;
    for (uint32_t off = 0; off < m_id.app_size; off += sizeof(buf)) {
        const size_t n = (m_id.app_size - off) < sizeof(buf) ? (m_id.app_size - off) : sizeof(buf);
        read_image(off, buf, n);
        crc = crc32(buf, n, crc);
    }
    return crc;
}

// ---------------------------------------------------------------- commands

bool Protocol::handle(uint8_t cmd, bool& boot) {
    switch (cmd) {
    case kGetSync:
        if (!eoc()) break;
        status(kOk);
        return true;

    case kGetDevice: {
        uint8_t param;
        if (!recv(param, kByteTimeoutMs) || !eoc()) break;
        switch (param) {
        case kInfoProtocolRev: word_reply(kProtocolRevision); break;
        case kInfoBoardId: word_reply(m_id.board_id); break;
        case kInfoBoardRev: word_reply(m_id.board_revision); break;
        case kInfoAppSize: word_reply(m_id.app_size); break;
        case kInfoExtFlashSize: word_reply(0); break; // no external flash
        default: status(kInvalid); break;
        }
        return true;
    }

    case kChipErase:
    case kChipFullErase:
        if (!eoc()) break;
        m_first_held = false;
        m_pending_fill = 0;
        m_prog_addr = 0;
        m_erased = m_port.erase_app();
        status(m_erased ? kOk : kFailed);
        return true;

    case kProgMulti: {
        uint8_t len;
        uint8_t data[255];
        if (!recv(len, kByteTimeoutMs) || !recv_n(data, len) || !eoc()) break;
        if (len == 0 || (len % 4) != 0) {
            status(kInvalid);
        } else if (!m_erased || m_prog_addr + len > m_id.app_size) {
            status(kFailed);
        } else {
            bool ok = true;
            stage(data, len, ok);
            status(ok ? kOk : kFailed);
        }
        return true;
    }

    case kGetCrc:
        if (!eoc()) break;
        word_reply(image_crc());
        return true;

    case kChipVerify: // older revisions: restart READ_MULTI at the image start
        if (!eoc()) break;
        m_read_addr = 0;
        status(kOk);
        return true;

    case kReadMulti: {
        uint8_t len;
        if (!recv(len, kByteTimeoutMs) || !eoc()) break;
        if (m_read_addr + len > m_id.app_size) {
            status(kInvalid);
            return true;
        }
        uint8_t data[255];
        read_image(m_read_addr, data, len);
        m_read_addr += len;
        m_port.write(data, len);
        status(kOk);
        return true;
    }

    case kGetOtp: { // no OTP area: report it blank
        uint32_t offset;
        if (!recv_word(offset) || !eoc()) break;
        word_reply(0xFFFFFFFFu);
        return true;
    }

    case kGetSn: {
        uint32_t offset;
        uint32_t word = 0;
        if (!recv_word(offset) || !eoc()) break;
        if (!m_port.serial_word(offset, word)) {
            word = 0xFFFFFFFFu;
        }
        word_reply(word);
        return true;
    }

    case kGetChip:
        if (!eoc()) break;
        word_reply(m_id.chip_id);
        return true;

    case kGetChipDes:
        if (!eoc()) break;
        text_reply(m_id.chip_description);
        return true;

    case kGetSoftware:
        if (!eoc()) break;
        text_reply(m_id.software);
        return true;

    case kSetBootDelay: { // accepted; the boot delay is fixed at build time
        uint8_t delay;
        if (!recv(delay, kByteTimeoutMs) || !eoc()) break;
        status(kOk);
        return true;
    }

    case kSetBaud: {
        uint32_t baud;
        if (!recv_word(baud) || !eoc()) break;
        status(kOk);
        m_port.flush();
        if (!m_port.set_baud(baud)) {
            // Unsupported (USB CDC ignores baud rates): the link keeps working.
        }
        return true;
    }

    case kReboot:
        if (!eoc()) break;
        if (!finish_programming()) {
            status(kFailed);
            return true;
        }
        status(kOk);
        m_port.flush();
        boot = true;
        return true;

    case kExtfErase:
    case kExtfProgMulti:
    case kExtfReadMulti:
    case kExtfGetCrc:
        status(kInvalid); // no external flash on this board
        return true;

    default:
        return false; // unknown byte (noise, other protocols): ignore silently
    }

    // A known command with a missing or wrong terminator.
    status(kInvalid);
    return true;
}

Protocol::Result Protocol::run(uint32_t first_contact_ms, uint32_t idle_ms) {
    const uint32_t start = m_port.millis();
    uint32_t last = start;
    while (true) {
        uint8_t cmd;
        if (recv(cmd, kPollMs)) {
            bool boot = false;
            if (handle(cmd, boot)) {
                m_host_seen = true;
                last = m_port.millis();
            }
            if (boot) {
                return Result::Boot;
            }
            continue;
        }
        const uint32_t now = m_port.millis();
        if (!m_host_seen && first_contact_ms != 0 && now - start >= first_contact_ms) {
            return Result::Timeout;
        }
        if (m_host_seen && idle_ms != 0 && now - last >= idle_ms) {
            return Result::Timeout;
        }
    }
}

} // namespace apx::bootloader
