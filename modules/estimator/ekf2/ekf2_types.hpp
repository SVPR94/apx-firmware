// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <array>
#include <cstdint>
namespace apx::estimator {

constexpr int EKF_STATE_SIZE = 16;
constexpr int EKF_COV_SIZE  = EKF_STATE_SIZE;

// State: [q0,q1,q2,q3, vx,vy,vz, px,py,pz, bg_x,bg_y,bg_z, ba_x,ba_y,ba_z]
enum StateIdx { Q0=0,Q1,Q2,Q3, VX,VY,VZ, PX,PY,PZ, BGX,BGY,BGZ, BAX,BAY,BAZ };

struct EKF2State {
    std::array<float, EKF_STATE_SIZE> x{}; // state vector
    std::array<float, EKF_COV_SIZE * EKF_COV_SIZE> P{}; // covariance (16x16 diagonal init)
};

enum class MeasurementType { GPS_POS, GPS_VEL, BARO_ALT, MAG_HEADING, FLOW_VEL };

struct GpsMeasurement { float lat_deg, lon_deg, alt_m; };
struct BaroMeasurement { float altitude_m; };
struct MagMeasurement  { float heading_rad; };

} // namespace apx::estimator
