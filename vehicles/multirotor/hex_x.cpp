// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "../vehicle_registry.hpp"
#include <cstdio>

namespace apx {

class HexX : public VehicleBase {
public:
    const char* name() const override { return "hex_x"; }
    void configure() override {
        // 6 motors at 0, 60, 120, 180, 240, 300 degrees
        // Alternating CW/CCW spin direction
        printf("Configured: Hex X frame, 6 motors\n");
    }
};

APX_REGISTER_VEHICLE("hex_x", HexX);

} // namespace apx
