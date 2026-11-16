// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include "../../include/apx/drivers/driver_base.hpp"
#include <cstdint>

namespace apx::hal {
    struct CanFrame {
        uint32_t id;
        uint8_t data[8];
        uint8_t dlc;
    };
    class IHalCan {
    public:
        virtual ~IHalCan() = default;
        virtual int read(CanFrame* frame) = 0;
        virtual int write(const CanFrame& frame) = 0;
    };
}

namespace apx::drivers {

struct ObstacleData {
    float distance_m[10];
    float angle_deg[10];
    bool valid[10];
    uint64_t timestamp_us;
};

class CanRadarDriver : public DriverBase {
public:
    CanRadarDriver(apx::hal::IHalCan& can, uint32_t base_can_id = 0x200);
    bool init() override;
    void update() override;
    
    ObstacleData get_data() const;

private:
    apx::hal::IHalCan& m_can;
    uint32_t m_base_can_id;
    ObstacleData m_data{};
};

} // namespace apx::drivers
