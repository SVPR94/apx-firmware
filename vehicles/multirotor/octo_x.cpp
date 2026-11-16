// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "../vehicle_registry.hpp"
#include <cstdio>

namespace apx {

class OctoX : public VehicleBase {
public:
    const char* name() const override { return "octo_x"; }
    void configure() override {
        // 8 motors at 0, 45, 90, 135, 180, 225, 270, 315 degrees
        printf("Configured: Octo X frame, 8 motors\n");
    }
};

APX_REGISTER_VEHICLE("octo_x", OctoX);

} // namespace apx
