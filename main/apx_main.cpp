// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <csignal>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <unistd.h>
#include <apx/kernel/timer.hpp>

#include "mavlink_handler.hpp"
#include "param_manager.hpp"
#include "mixer/motor_mixer.hpp"
#include "mode_manager.hpp"
#include "stick_arm_detector.hpp"
#include "sprayer.hpp"
#include "fence.hpp"
#include "radar_manager.hpp"

#ifdef APX_SITL
#include "sitl_bridge/sitl_server.hpp"
#include "sensors/sim_sensors.hpp"
#include "sensors/sim_sensor_log.hpp"
#include "altitude_estimator.hpp"
#endif

namespace {
volatile bool g_running = true;

apx::control::FrameType frame_type_for_vehicle(const std::string& vehicle_type) {
    if (vehicle_type.rfind("hex", 0) == 0 || vehicle_type == "hexa") return apx::control::FrameType::HEX_X;
    if (vehicle_type.rfind("octo", 0) == 0 || vehicle_type == "octa") return apx::control::FrameType::OCTO_X;
    return apx::control::FrameType::QUAD_X;
}
}

static void signal_handler(int) {
    g_running = false;
}

static std::vector<uint16_t> parse_port_list(const std::string& str) {
    std::vector<uint16_t> ports;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) {
            int p = std::atoi(item.c_str());
            if (p > 0 && p < 65536) {
                ports.push_back(static_cast<uint16_t>(p));
            }
        }
    }
    return ports;
}

int main(int argc, char* argv[]) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::printf("======================================================================\n");
    std::printf(" APX Flight Controller v0.1.0\n");
#ifdef APX_SITL
    std::printf(" Running in SITL (Simulation) Mode\n");
#endif
    std::printf("======================================================================\n");

    // Default configuration
    std::string vehicle_type = "quad_x";
    uint16_t sitl_port = 9003;
    uint16_t gcs_udp_port = 14550;
    std::vector<uint16_t> tcp_ports = {5760, 5761, 5762, 5763};
    double home_lat = 18.5204;
    double home_lon = 73.8567;
    float home_alt = 540.0f;
#ifdef APX_SITL
    // Simulated sensor suite (see sitl/sensors/sim_sensors.hpp).
    int sensor_lanes = 4;     // IMU+baro+mag ICs, 1..4
    int gps_count = 1;        // GNSS receivers, 0..4
    uint32_t sim_seed = 1;
    apx::sitl::SensorProfile sensor_profile = apx::sitl::SensorProfile::REALISTIC;
    std::string sensor_log_path;
#endif

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--vehicle=", 0) == 0) {
            vehicle_type = arg.substr(10);
        } else if (arg.rfind("--sitl-port=", 0) == 0) {
            sitl_port = static_cast<uint16_t>(std::atoi(arg.substr(12).c_str()));
        } else if (arg.rfind("--gcs-udp=", 0) == 0) {
            gcs_udp_port = static_cast<uint16_t>(std::atoi(arg.substr(10).c_str()));
        } else if (arg.rfind("--gcs-tcp=", 0) == 0) {
            uint16_t primary_tcp = static_cast<uint16_t>(std::atoi(arg.substr(10).c_str()));
            if (primary_tcp > 0) {
                tcp_ports = {primary_tcp, static_cast<uint16_t>(primary_tcp + 1),
                             static_cast<uint16_t>(primary_tcp + 2), static_cast<uint16_t>(primary_tcp + 3)};
            }
        } else if (arg.rfind("--tcp-ports=", 0) == 0) {
            auto custom_ports = parse_port_list(arg.substr(12));
            if (!custom_ports.empty()) {
                tcp_ports = custom_ports;
            }
        } else if (arg.rfind("--home-lat=", 0) == 0) {
            home_lat = std::atof(arg.substr(11).c_str());
        } else if (arg.rfind("--home-lon=", 0) == 0) {
            home_lon = std::atof(arg.substr(11).c_str());
        } else if (arg.rfind("--home-alt=", 0) == 0) {
            home_alt = static_cast<float>(std::atof(arg.substr(11).c_str()));
#ifdef APX_SITL
        } else if (arg.rfind("--sensor-lanes=", 0) == 0) {
            sensor_lanes = std::atoi(arg.substr(15).c_str());
            if (sensor_lanes < 1 || sensor_lanes > apx::sitl::SimSensorSuite::kMaxImu) {
                std::fprintf(stderr, "[APX Error] --sensor-lanes must be 1..%d\n", apx::sitl::SimSensorSuite::kMaxImu);
                return 2;
            }
        } else if (arg.rfind("--gps-count=", 0) == 0) {
            gps_count = std::atoi(arg.substr(12).c_str());
            if (gps_count < 0 || gps_count > apx::sitl::SimSensorSuite::kMaxGps) {
                std::fprintf(stderr, "[APX Error] --gps-count must be 0..%d\n", apx::sitl::SimSensorSuite::kMaxGps);
                return 2;
            }
        } else if (arg.rfind("--sim-seed=", 0) == 0) {
            sim_seed = static_cast<uint32_t>(std::strtoul(arg.substr(11).c_str(), nullptr, 10));
        } else if (arg.rfind("--sensor-profile=", 0) == 0) {
            if (!apx::sitl::parse_sensor_profile(arg.substr(17).c_str(), sensor_profile)) {
                std::fprintf(stderr, "[APX Error] --sensor-profile must be ideal, realistic or harsh\n");
                return 2;
            }
        } else if (arg.rfind("--sensor-log=", 0) == 0) {
            sensor_log_path = arg.substr(13);
#endif
        } else if (arg == "--radar") {
            apx::gcs::ParamManager::get_instance().set_int("RADAR_ENABLE", 1);
            apx::gcs::ParamManager::get_instance().set_int("TERRAIN_ENABLE", 1);
        }
    }

    std::printf("[APX Init] Vehicle frame : %s\n", vehicle_type.c_str());
    std::printf("[APX Init] Home location : %.4f N, %.4f E, %.1fm MSL\n", home_lat, home_lon, home_alt);
    std::printf("[APX Init] GCS UDP port  : %u\n", gcs_udp_port);
    std::printf("[APX Init] GCS TCP ports : ");
    for (size_t i = 0; i < tcp_ports.size(); ++i) {
        std::printf("%u%s", tcp_ports[i], (i + 1 < tcp_ports.size()) ? ", " : "\n");
    }

    // 1. Initialize Parameter System
    apx::gcs::ParamManager::get_instance().init_defaults();

    // 2. Initialize MAVLink Telemetry Handler & Network Streams
    apx::gcs::MavlinkHandler mavlink(1, 1);
    mavlink.init();
    mavlink.set_home_position(home_lat, home_lon, home_alt);

    // Seed initial state with home position
    mavlink.set_vehicle_state(
        0.0f, 0.0f, 0.0f,       // roll, pitch, yaw
        home_lat, home_lon, home_alt, 0.0f, // lat, lon, alt_msl, rel_alt
        0.0f, 0.0f, 0.0f,       // vx, vy, vz
        12.6f, 2.5f, 98,        // voltage, current, pct
        false,                  // armed
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0                       // STABILIZE
    );

    if (!mavlink.start_network(gcs_udp_port, tcp_ports)) {
        std::fprintf(stderr, "[APX Error] Failed to start telemetry network services.\n");
    }

    apx::control::MotorMixer motor_mixer;
    const apx::control::FrameType frame_type = frame_type_for_vehicle(vehicle_type);

    const int frame_class_val = (frame_type == apx::control::FrameType::HEX_X) ? 2
                              : (frame_type == apx::control::FrameType::OCTO_X) ? 3 : 1;
    apx::gcs::ParamManager::get_instance().set_int("FRAME_CLASS", frame_class_val);

    // Owns mode switching/transition logic and the Position->Attitude->Rate
    // cascade each position-aware mode uses (see modules/modes/). Boots
    // into STABILIZE, powering up in a manual mode rather than something
    // that could move on arm.
    apx::modes::ModeManager mode_manager;
    mavlink.set_mode_manager(&mode_manager);
    mavlink.set_home_position(home_lat, home_lon, home_alt);
    mavlink.set_vehicle_type(frame_type == apx::control::FrameType::HEX_X    ? MAV_TYPE_HEXAROTOR
                             : frame_type == apx::control::FrameType::OCTO_X ? MAV_TYPE_OCTOROTOR
                                                                             : MAV_TYPE_QUADROTOR);
    apx::modes::StickArmDetector stick_arm_detector;
    apx::radar::RadarManager::get_instance().init();

    // Persists across ticks - built from SITL physics when it arrives, but
    // mode_manager.update() runs every tick regardless (e.g. armed/dt_s
    // must always be current even on a tick with no fresh physics sample).
    apx::modes::VehicleState vstate;
    mavlink.home_ned(vstate.home_ned);

    // 3. Initialize SITL Physics Bridge if enabled
#ifdef APX_SITL
    apx::sitl::SitlServer sitl_server;
    if (sitl_server.init(sitl_port)) {
        sitl_server.start();
        std::printf("[APX SITL] SITL physics bridge connected on UDP port %u\n", sitl_port);
    } else {
        std::fprintf(stderr, "[APX SITL] Warning: Could not bind SITL port %u\n", sitl_port);
    }
#endif

#ifdef APX_SITL
    apx::sitl::SimSensorConfig sensor_cfg = apx::sitl::make_sensor_config(sensor_profile);
    sensor_cfg.imu_count = sensor_lanes;
    sensor_cfg.gps_count = gps_count;
    sensor_cfg.seed = sim_seed;
    sensor_cfg.home_lat_deg = home_lat;
    sensor_cfg.home_lon_deg = home_lon;
    sensor_cfg.home_alt_m = home_alt;
    apx::sitl::SimSensorSuite sim_sensors;
    sim_sensors.init(sensor_cfg);
    std::printf("[APX SITL] Sensors: %d IMU/baro/mag lane(s), %d GPS, profile '%s', seed %u\n",
                sim_sensors.imu_count(), sim_sensors.gps_count(),
                apx::sitl::sensor_profile_name(sensor_profile), sim_seed);
    apx::sitl::SimSensorLog sensor_log;
    if (!sensor_log_path.empty()) {
        if (sensor_log.open(sensor_log_path.c_str())) {
            std::printf("[APX SITL] Sensor log: %s\n", sensor_log_path.c_str());
        } else {
            std::fprintf(stderr, "[APX SITL] Warning: cannot open sensor log '%s'\n", sensor_log_path.c_str());
        }
    }
    const int sim_motor_count = frame_type == apx::control::FrameType::HEX_X    ? 6
                              : frame_type == apx::control::FrameType::OCTO_X ? 8
                                                                              : 4;
    float sim_motor_throttle[8] = {0};
    uint64_t last_truth_us = UINT64_MAX;
    apx::estimator::AltitudeEstimator alt_estimator;
    uint32_t est_imu_seq[apx::sitl::SimSensorSuite::kMaxImu] = {0};
    uint32_t est_baro_seq[apx::sitl::SimSensorSuite::kMaxImu] = {0};
    bool est_was_armed = false;
    // MSL altitude of the estimator's zero (the boot spot, then the last
    // arming spot). Kept apart from home: a GCS may move home anywhere,
    // which must not shift the vehicle's reported altitude.
    float est_zero_alt_msl = home_alt;
#endif

    std::printf("[APX Flight Controller] System ready. Entering control loop (400 Hz)...\n");

    // Flight Controller main loop (400 Hz):
    // the ATC_RAT_* rate-loop gains in modules/modes/navigation_cascade.cpp
    // assume a fast inner rate loop to ensure sufficient phase margin.
    //
    // Fixed-rate scheduling against an absolute deadline. The loop used to
    // do its work and then sleep a further 2.5 ms, so the real period was
    // 2.5 ms + work time while every controller still integrated with a
    // hard-coded dt of 2.5 ms: the waypoint reference, yaw slews and PID
    // integrators all ran ~5-8% slow (AUTO flew 4.7 m/s for WPNAV_SPEED 5).
    // Sleeping to an absolute deadline makes the average period exact.
    constexpr uint64_t kLoopPeriodUs = 2500;
    uint64_t next_tick_us = apx::kernel::now_us() + kLoopPeriodUs;
    while (g_running) {
        // Keep home location in sync with telemetry / GCS updates
        if (mavlink.is_home_set()) {
            home_lat = mavlink.home_lat();
            home_lon = mavlink.home_lon();
            home_alt = mavlink.home_alt_msl();
            mavlink.home_ned(vstate.home_ned);
        }

        // Heartbeat's custom_mode must reflect whatever ModeManager is
        // actually in *every* tick, not just ticks with fresh physics -
        // e.g. the very first DO_SET_MODE before any physics has arrived.
        mavlink.set_custom_mode(apx::modes::ModeManager::to_custom_mode_number(mode_manager.current_id()));

#ifdef APX_SITL
        apx::sitl::SitlPhysicsState phys;
        if (sitl_server.get_latest_physics_state(phys)) {
            float roll = static_cast<float>(phys.euler_angles[0]);
            float pitch = static_cast<float>(phys.euler_angles[1]);
            float yaw = static_cast<float>(phys.euler_angles[2]);
            double lat = phys.gps_lat;
            double lon = phys.gps_lon;
            float alt = static_cast<float>(phys.gps_alt);
            float rel_alt = static_cast<float>(phys.gps_alt - home_alt);
            float vx = static_cast<float>(phys.velocity_ned[0]);
            float vy = static_cast<float>(phys.velocity_ned[1]);
            float vz = static_cast<float>(phys.velocity_ned[2]);

            for (int i = 0; i < 3; ++i) {
                vstate.pos_ned[i] = static_cast<float>(phys.position_ned[i]);
                vstate.vel_ned[i] = static_cast<float>(phys.velocity_ned[i]);
                vstate.gyro_body[i] = static_cast<float>(phys.gyro_body[i]);
            }
            vstate.roll = roll;
            vstate.pitch = pitch;
            vstate.yaw = yaw;
            for (int i = 0; i < 4; ++i) {
                vstate.quat[i] = static_cast<float>(phys.quaternion[i]);
            }

            // Feed simulated radar data to RadarManager
            apx::radar::RadarManager::get_instance().feed_sitl_radar(
                phys.radar_distances, phys.radar_valid_mask, phys.timestamp_us);

            // Wind telemetry (MAVLink message 168 WIND: direction from, speed, speed_z)
            const float wn = static_cast<float>(phys.wind_vel_ned[0]);
            const float we = static_cast<float>(phys.wind_vel_ned[1]);
            const float wz = static_cast<float>(phys.wind_vel_ned[2]);
            const float wind_speed = std::sqrt(wn * wn + we * we);
            float wind_dir = std::atan2(-we, -wn) * 180.0f / static_cast<float>(M_PI);
            if (wind_dir < 0.0f) wind_dir += 360.0f;
            mavlink.set_wind(wind_dir, wind_speed, wz);

            // Simulated sensors. Height and climb rate come from the vertical
            // estimator fed by them (SIM_TRUTH_ALT=0); everything else is
            // still simulator truth.
            if (phys.timestamp_us != last_truth_us) {
                last_truth_us = phys.timestamp_us;
                apx::sitl::SimTruth truth;
                truth.time_us = phys.timestamp_us;
                for (int i = 0; i < 3; ++i) {
                    truth.pos_ned[i] = phys.position_ned[i];
                    truth.vel_ned[i] = phys.velocity_ned[i];
                    truth.accel_body[i] = phys.accel_body[i];
                    truth.gyro_body[i] = phys.gyro_body[i];
                }
                for (int i = 0; i < 4; ++i) {
                    truth.quat[i] = phys.quaternion[i];
                }
                truth.airspeed_ms = phys.airspeed_ms;
                truth.num_motors = sim_motor_count;
                for (int i = 0; i < 8; ++i) {
                    truth.motor_throttle[i] = sim_motor_throttle[i];
                }
                sim_sensors.update(truth);

                // Vertical estimator: mean of the fresh IMU lanes predicts,
                // each fresh baro lane corrects.
                float accel_sum[3] = {0.0f, 0.0f, 0.0f};
                int accel_n = 0;
                uint64_t imu_t = 0;
                for (int i = 0; i < sim_sensors.imu_count(); ++i) {
                    if (sim_sensors.imu_seq(i) != est_imu_seq[i]) {
                        est_imu_seq[i] = sim_sensors.imu_seq(i);
                        const auto& d = sim_sensors.imu(i);
                        for (int a = 0; a < 3; ++a) {
                            accel_sum[a] += d.accel_xyz[a];
                        }
                        imu_t = d.timestamp_us;
                        ++accel_n;
                    }
                    if (sim_sensors.baro_seq(i) != est_baro_seq[i]) {
                        est_baro_seq[i] = sim_sensors.baro_seq(i);
                        alt_estimator.update_baro(i, sim_sensors.baro(i).altitude_m);
                    }
                }
                if (accel_n > 0) {
                    const float accel_mean[3] = {accel_sum[0] / accel_n, accel_sum[1] / accel_n,
                                                 accel_sum[2] / accel_n};
                    alt_estimator.predict_imu(imu_t, accel_mean, vstate.quat);
                }
                sensor_log.log_truth(truth);
                sensor_log.log_new_samples(truth.time_us, sim_sensors);

                apx::gcs::RawSensorTelemetry raw;
                for (int i = 0; i < sim_sensors.imu_count(); ++i) {
                    if (sim_sensors.imu_seq(i) == 0) {
                        continue;
                    }
                    auto& r = raw.imu[i];
                    const auto& imu = sim_sensors.imu(i);
                    const auto& baro = sim_sensors.baro(i);
                    const auto& mag = sim_sensors.mag(i);
                    r.present = true;
                    r.time_us = imu.timestamp_us;
                    for (int a = 0; a < 3; ++a) {
                        r.accel[a] = imu.accel_xyz[a];
                        r.gyro[a] = imu.gyro_xyz[a];
                        r.mag[a] = mag.mag_xyz[a];
                    }
                    r.pressure_hpa = baro.pressure_pa * 0.01f;
                    r.pressure_alt_m = baro.altitude_m;
                    r.temp_c = imu.temp_c;
                }
                raw.gps_count = sim_sensors.gps_count();
                for (int i = 0; i < sim_sensors.gps_count(); ++i) {
                    if (sim_sensors.gps_seq(i) == 0) {
                        continue;
                    }
                    auto& r = raw.gps[i];
                    const auto& g = sim_sensors.gps(i);
                    r.present = true;
                    r.time_us = g.timestamp_us;
                    r.fix_3d = g.valid && g.fix_type == apx::drivers::GpsFixType::FIX_3D;
                    r.lat_deg = g.lat_deg;
                    r.lon_deg = g.lon_deg;
                    r.alt_msl_m = g.alt_msl_m;
                    r.vel_ned[0] = g.vel_n_ms;
                    r.vel_ned[1] = g.vel_e_ms;
                    r.vel_ned[2] = g.vel_d_ms;
                    r.hdop = g.hdop;
                    r.vdop = g.vdop;
                    r.satellites = g.satellites;
                }
                mavlink.set_raw_sensor_telemetry(raw);
            }

            // Baro ground calibration on arming, as on hardware.
            const bool armed_now = mavlink.get_armed();
            if (armed_now && !est_was_armed) {
                if (alt_estimator.initialised()) {
                    est_zero_alt_msl += alt_estimator.height();
                }
                alt_estimator.rezero();
            }
            est_was_armed = armed_now;

            if (alt_estimator.initialised() &&
                apx::gcs::ParamManager::get_instance().get_int_or("SIM_TRUTH_ALT", 0) == 0) {
                const float h = alt_estimator.height();
                const float climb = alt_estimator.climb_rate();
                vstate.pos_ned[2] = -h;
                vstate.vel_ned[2] = -climb;
                rel_alt = h;
                alt = est_zero_alt_msl + h;
                vz = -climb;
            }

            // Preserve armed/mode state here: they are mutated independently
            // by CommandHandler (ARM/DISARM, DO_SET_MODE) via
            // mavlink.set_armed()/mode_manager.request_mode(). Previously
            // this call hardcoded armed=false and custom_mode=0 every
            // 10ms, which silently reverted any mode/arm change the
            // instant the next physics tick arrived - GCS would show
            // ACCEPTED for the command but the vehicle state never
            // actually changed.
            mavlink.set_vehicle_state(
                roll, pitch, yaw,
                lat, lon, alt, rel_alt,
                vx, vy, vz,
                12.6f, 2.5f, 95,
                mavlink.get_armed(),
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mavlink.get_custom_mode()
            );
        }
#endif

        vstate.armed = mavlink.get_armed();
        // The deadline scheduler below keeps the average period at exactly
        // kLoopPeriodUs, so the nominal dt is right on average. (A measured
        // per-tick dt was tried: its jitter, and catch-up ticks that see no
        // new sensor data, spiked the D terms and destabilised the vehicle.)
        vstate.dt_s = static_cast<float>(kLoopPeriodUs) * 1e-6f;

        auto rc = mavlink.get_rc_channels();
        vstate.rc_roll = rc.roll;
        vstate.rc_pitch = rc.pitch;
        vstate.rc_yaw = rc.yaw;
        vstate.rc_throttle = rc.throttle;
        vstate.rc_valid = rc.valid;

        // RC stick arm/disarm gesture (throttle-down + yaw-right to arm,
        // throttle-down + yaw-left to disarm, held ~1.5s). Uses
        // the same set_armed() authority MAV_CMD_COMPONENT_ARM_DISARM
        // already uses (see command_handler.cpp), and the same
        // ModeManager::can_arm() gate, so a mode that refuses to arm
        // (e.g. no GPS fix) is respected from either path.
        {
            using Result = apx::modes::StickArmDetector::Result;
            Result r = stick_arm_detector.update(vstate.armed, vstate.rc_valid,
                                                  vstate.rc_throttle, vstate.rc_yaw, vstate.dt_s);
            if (r == Result::ARM) {
                std::string fence_msg;
                if (!apx::safety::FenceManager::instance().pre_arm_check(-vstate.pos_ned[2],
                                                                         vstate.pos_ned[0],
                                                                         vstate.pos_ned[1],
                                                                         fence_msg)) {
                    mavlink.send_statustext(MAV_SEVERITY_CRITICAL, fence_msg.c_str());
                    std::printf("[APX Flight] RC arm gesture ignored - %s\n", fence_msg.c_str());
                } else if (mode_manager.can_arm()) {
                    mavlink.set_armed(true);
                    std::printf("[APX Flight] Armed via RC stick gesture\n");
                } else {
                    std::printf("[APX Flight] RC arm gesture ignored - current mode refuses to arm\n");
                }
            } else if (r == Result::DISARM) {
                if (mode_manager.disarm_allowed()) {
                    mavlink.set_armed(false);
                    std::printf("[APX Flight] Disarmed via RC stick gesture\n");
                } else {
                    std::printf("[APX Flight] RC disarm gesture ignored - not landed\n");
                }
            }
            vstate.armed = mavlink.get_armed();
        }

        // Periodic radar processing and terrain-following update
        apx::radar::RadarManager::get_instance().update(
            apx::kernel::now_us(), vstate.pos_ned, vstate.roll, vstate.pitch, vstate.yaw, vstate.dt_s);

        mavlink.set_last_known_vehicle_state(vstate);

        // Geofence periodic check when armed
        if (vstate.armed) {
            float rel_alt = -vstate.pos_ned[2];
            auto breach = apx::safety::FenceManager::instance().check(rel_alt, vstate.pos_ned[0], vstate.pos_ned[1], vstate.dt_s);
            if (breach != apx::safety::FenceBreachType::NONE) {
                auto action = apx::safety::FenceManager::instance().get_action();
                static apx::safety::FenceBreachType last_announced_breach = apx::safety::FenceBreachType::NONE;
                if (last_announced_breach != breach) {
                    last_announced_breach = breach;
                    const char* type_str = (breach == apx::safety::FenceBreachType::ALTITUDE) ? "Altitude" : "Polygon";
                    const char* act_str = (action == apx::safety::FenceAction::RTL) ? "RTL" :
                                          (action == apx::safety::FenceAction::LAND) ? "LAND" :
                                          (action == apx::safety::FenceAction::BRAKE) ? "BRAKE" : "REPORT";
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "Fence breach: %s - Action: %s", type_str, act_str);
                    mavlink.send_statustext(MAV_SEVERITY_CRITICAL, buf);
                }
                if (action == apx::safety::FenceAction::RTL) {
                    mode_manager.request_mode(apx::modes::FlightModeId::RTL, vstate);
                } else if (action == apx::safety::FenceAction::LAND) {
                    mode_manager.request_mode(apx::modes::FlightModeId::LAND, vstate);
                } else if (action == apx::safety::FenceAction::BRAKE) {
                    mode_manager.request_mode(apx::modes::FlightModeId::BRAKE, vstate);
                }
            }
        }

#ifdef APX_SITL
        // Drive the simulated motors from whatever ModeManager's active
        // mode decides this tick (idle-spin in STABILIZE, climb/hold/
        // navigate in GUIDED, descend+land-detect in LAND, etc. - see
        // modules/modes/). This is the single place mode output reaches
        // the mixer, matching the "handy to modify later" ask: a new mode
        // doesn't need any changes here, only in modules/modes/.
        {
            apx::modes::ControlOutput ctrl = mode_manager.update(vstate);
            if (ctrl.crash_detected && mavlink.get_armed()) {
                mavlink.send_statustext(MAV_SEVERITY_CRITICAL, "Crash detected - motors stopped");
            }
            if (ctrl.request_disarm && mavlink.get_armed()) {
                std::printf("[APX Flight] Mode requested disarm (e.g. landing complete)\n");
                mavlink.set_last_known_vehicle_state(vstate);
                mavlink.set_armed(false);
            }

            // MOT_SPIN_ARM: once armed, motors idle-spin at this floor
            // instead of sitting at true zero thrust, providing clear
            // indication that the vehicle is armed. Checked with the fresh
            // post-disarm armed state (mavlink.get_armed(), not the vstate snapshot
            // from the top of this tick) so a mode that just requested disarm above
            // (landing complete, crash stop) doesn't get its motors
            // re-spun by this floor in the same tick. A floor (max), never
            // reduces a mode's actual thrust demand once really flying.
            if (mavlink.get_armed()) {
                float spin_arm = apx::gcs::ParamManager::get_instance().get_float_or("MOT_SPIN_ARM", 0.10f);
                if (ctrl.thrust < spin_arm) {
                    ctrl.thrust = spin_arm;
                }
            }

            float outputs[8] = {0};
            motor_mixer.mix(ctrl.thrust, ctrl.roll_cmd, ctrl.pitch_cmd, ctrl.yaw_cmd, frame_type, outputs);

            // Active motor test override (bench testing single motor / all motors)
            if (mavlink.is_motor_test_active()) {
                uint8_t test_motor = mavlink.get_motor_test_index();
                float test_throttle = mavlink.get_motor_test_throttle();
                if (test_motor >= 1 && test_motor <= 8) {
                    outputs[test_motor - 1] = test_throttle;
                }
            }

            apx::sitl::SitlActuatorOutput actuator{};
            actuator.timestamp_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            for (int i = 0; i < 8; ++i) {
                actuator.motor_throttle[i] = outputs[i];
                actuator.servo_pwm[i] = static_cast<uint16_t>(1000.0f + std::clamp(outputs[i], 0.0f, 1.0f) * 1000.0f);
            }
            for (int i = 8; i < 16; ++i) {
                actuator.servo_pwm[i] = mavlink.get_aux_servo_pwm(static_cast<uint8_t>(i + 1));
            }
            // Sprayer pump/spinner on their servo outputs (the Gazebo
            // plugin turns these into the spray visual).
            {
                auto& sprayer = apx::modes::Sprayer::instance();
                int pump_ch = sprayer.pump_channel();
                int spin_ch = sprayer.spinner_channel();
                if (pump_ch >= 0 && pump_ch < 16) {
                    actuator.servo_pwm[pump_ch] = sprayer.pump_pwm();
                }
                if (spin_ch >= 0 && spin_ch < 16) {
                    actuator.servo_pwm[spin_ch] = sprayer.spinner_pwm();
                }
                if (sprayer.take_state_changed()) {
                    mavlink.send_statustext(MAV_SEVERITY_INFO,
                                            sprayer.spraying() ? "Sprayer: spraying" : "Sprayer: stopped");
                }
            }
            sitl_server.send_actuator_output(actuator);
            for (int i = 0; i < 8; ++i) {
                sim_motor_throttle[i] = outputs[i];
            }
            mavlink.set_motor_outputs(outputs);
            mavlink.set_servo_outputs(actuator.servo_pwm);
        }
#endif

        // Step vehicle control & telemetry tasks
        mavlink.update();

        // Monitor and log GCS connection state transitions
        static size_t last_gcs_count = 0;
        size_t current_gcs_count = mavlink.get_active_gcs_count();
        if (current_gcs_count != last_gcs_count) {
            if (current_gcs_count > last_gcs_count) {
                std::printf("[APX GCS] Active Ground Control Station detected: %s\n",
                            mavlink.get_active_gcs_summary().c_str());
            } else {
                std::printf("[APX GCS] Ground Control Station status update: %s\n",
                            mavlink.get_active_gcs_summary().c_str());
            }
            last_gcs_count = current_gcs_count;
        }

        // 2.5ms loop rate (400Hz) - see the loop-rate comment above. Sleep
        // to the next deadline; after a long overrun resync instead of
        // running a burst of back-to-back catch-up ticks.
        next_tick_us += kLoopPeriodUs;
        const uint64_t loop_now_us = apx::kernel::now_us();
        if (next_tick_us > loop_now_us) {
            apx::kernel::delay_us(static_cast<uint32_t>(next_tick_us - loop_now_us));
        } else if (loop_now_us - next_tick_us > 4 * kLoopPeriodUs) {
            next_tick_us = loop_now_us;
        }
    }

    std::printf("[APX Shutdown] Stopping services...\n");
    mavlink.stop_network();
#ifdef APX_SITL
    sitl_server.stop();
#endif
    std::printf("[APX Shutdown] Clean exit completed.\n");
    return 0;
}
