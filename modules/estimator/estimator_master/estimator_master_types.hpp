// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <array>
#include <string_view>

namespace apx::estimator {

enum class EstimatorLaneId : uint8_t {
    LANE_EKF3 = 0,
    LANE_UKF = 1,
    LANE_INVARIANT = 2,
    COUNT = 3
};

inline std::string_view lane_name(EstimatorLaneId lane) {
    switch (lane) {
        case EstimatorLaneId::LANE_EKF3:      return "EKF3 (24-State)";
        case EstimatorLaneId::LANE_UKF:       return "UKF (Non-Linear)";
        case EstimatorLaneId::LANE_INVARIANT: return "Fast Invariant CF";
        default:                              return "Unknown";
    }
}

enum class LaneHealthStatus : uint8_t {
    OPTIMAL = 0,     // NIS < 2.0, covariance well conditioned
    ACCEPTABLE = 1,  // NIS < 5.0, nominal operation
    DEGRADED = 2,    // NIS 5.0-9.0 or minor sensor contradiction
    DIVERGING = 3,   // High innovation, rapidly growing covariance
    FAULTED = 4      // NaN/Inf, divergence or timeout
};

struct LaneDiagnostics {
    EstimatorLaneId lane{EstimatorLaneId::LANE_EKF3};
    LaneHealthStatus health{LaneHealthStatus::OPTIMAL};
    float nis{0.0f};
    float trace_p{0.0f};
    float residual_drift{0.0f};
    float consensus_discrepancy{0.0f}; // distance from consensus
    float fusion_weight{0.333f};
    bool  is_primary{false};
};

struct FusedNavSolution {
    float quat[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float euler_rpy[3]{0.0f, 0.0f, 0.0f};
    float vel_ned[3]{0.0f, 0.0f, 0.0f};
    float pos_ned[3]{0.0f, 0.0f, 0.0f};
    float gyro_bias[3]{0.0f, 0.0f, 0.0f};
    float accel_bias[3]{0.0f, 0.0f, 0.0f};

    float att_var[3]{1e-4f, 1e-4f, 1e-4f};
    float vel_var[3]{1e-3f, 1e-3f, 1e-3f};
    float pos_var[3]{1e-2f, 1e-2f, 1e-2f};

    EstimatorLaneId primary_lane{EstimatorLaneId::LANE_EKF3};
    std::array<float, 3> lane_weights{0.5f, 0.4f, 0.1f};
    bool valid{false};
    uint64_t timestamp_us{0};
};

} // namespace apx::estimator
