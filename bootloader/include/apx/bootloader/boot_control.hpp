#pragma once
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
// APX boot control - one small RAM block shared by the application and the
// bootloader that survives a warm reset (it lives outside both images' RAM;
// on Zephyr boards: the apx_boot_control devicetree node at the end of DTCM,
// which the core never caches).
//
//   application -> bootloader   "stay in the bootloader" (MAVLink reboot-to-
//                               bootloader request from a ground station)
//   bootloader  -> bootloader   "start the application" after a reset, so the
//                               application begins with every peripheral in
//                               its reset state
//
// RAM content is random after power-on; the check word rejects it.

#include <cstdint>

namespace apx::bootloader {

enum class BootRequest : uint32_t {
    None = 0,
    StayInBootloader = 0x5354424Cu,  // "LBTS"
    StartApplication = 0x50414D4Au,  // "JMAP"
};

struct BootControl {
    uint32_t magic;
    uint32_t request;
    uint32_t check;
    uint32_t reserved;
};

constexpr uint32_t kBootControlMagic = 0x42585041u; // "APXB"

inline void boot_control_set(volatile BootControl* bc, BootRequest request) {
    const uint32_t r = static_cast<uint32_t>(request);
    bc->magic = kBootControlMagic;
    bc->request = r;
    bc->check = ~(kBootControlMagic ^ r);
    bc->reserved = 0;
}

// Returns the pending request and clears it (a request is used once).
inline BootRequest boot_control_take(volatile BootControl* bc) {
    const uint32_t r = bc->request;
    const bool valid = bc->magic == kBootControlMagic && bc->check == ~(kBootControlMagic ^ r);
    bc->magic = 0;
    bc->request = 0;
    bc->check = 0;
    if (!valid) {
        return BootRequest::None;
    }
    return static_cast<BootRequest>(r);
}

} // namespace apx::bootloader
