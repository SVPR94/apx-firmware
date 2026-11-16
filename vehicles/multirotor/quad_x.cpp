// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include "../vehicle_registry.hpp"
#include <cstdio>

namespace apx {

class QuadX : public VehicleBase {
public:
    const char* name() const override { return "quad_x"; }
    void configure() override {
        // Motor assignment for Quad X:
        // Motor 0: Front-Right (+roll, -pitch, +yaw) CW
        // Motor 1: Rear-Left   (-roll, +pitch, +yaw) CW
        // Motor 2: Front-Left  (+roll, +pitch, -yaw) CCW
        // Motor 3: Rear-Right  (-roll, -pitch, -yaw) CCW
        printf("Configured: Quad X frame, 4 motors\n");
    }
};

APX_REGISTER_VEHICLE("quad_x", QuadX);

} // namespace apx
