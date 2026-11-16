// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <cstdlib>

#include "../sitl_bridge/sitl_server.hpp"
#include "../physics/wind_model.hpp"

using namespace gz;
using namespace sim;

namespace apx {
namespace sitl {

// Purely visual/kinematic spin rate at full throttle - independent of the
// actual thrust model below, this just makes the propeller mesh spin.
constexpr double kMaxPropSpinRadS = 600.0;

// Thrust model: F = m_thrust_max_per_motor_n * throttle^2 (throttle in
// 0..1), applied along the propeller's own +Z (so tilting the vehicle
// redirects thrust, same as a real rotor) at the motor's position on
// base_link, plus a small reaction torque about that same axis (direction
// * drag coefficient * thrust) approximating rotor drag - the mechanism
// real yaw control eventually uses once a rate/attitude controller
// commands differential motor speeds. m_thrust_max_per_motor_n is a
// per-model SDF parameter (see Configure()), defaulting to 11.0 - chosen
// so the original quad model's hover sits at ~70% throttle for its
// ~2.2kg mass (T:W ~= 2:1, typical for a hobby quad):
//   4 motors * 11N = 44N max thrust; weight = 2.2kg * 9.81 = 21.58N;
//   hover throttle = sqrt(21.58 / 44) = 0.70.
constexpr double kYawDragCoeff = 0.02;

// Global lat/lon/alt that Gazebo world position (0,0,0) maps to. Must match
// apx_main.cpp's --home-lat/--home-lon/--home-alt defaults (18.5204,
// 73.8567, 540.0) - position_ned=0 here IS "home" as far as the firmware's
// rel_alt/GPS math is concerned, so if these disagree, everything computed
// from them silently disagrees too. They previously differed from the home_alt default;
// with a 540m home_alt default, that alone made a stationary vehicle report relative_alt=44m -
// enough to make MAV_CMD_NAV_TAKEOFF's altitude controller conclude it was
// already 41m above a 3m target and correctly command zero thrust forever.
// TODO: these are still compile-time constants, so a non-default
// --home-lat/--home-lon/--home-alt on the firmware side won't reach the
// plugin - needs a real handshake (e.g. an SDF param, or a "set origin"
// message over the actuator/physics UDP channel) to become truly dynamic.
constexpr double kOriginLat = 18.5204;
constexpr double kOriginLon = 73.8567;
constexpr double kOriginAlt = 540.0;
constexpr double kEarthRadiusM = 6378137.0;

static double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class ApxGazeboPlugin : public System,
                        public ISystemConfigure,
                        public ISystemPreUpdate,
                        public ISystemPostUpdate {
public:
    ApxGazeboPlugin() : m_udp_socket(-1) {}

    ~ApxGazeboPlugin() {
        if (m_udp_socket >= 0) close(m_udp_socket);
    }

    void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm, EventManager &_eventMgr) override {
        m_model = Model(_entity);
        if (!m_model.Valid(_ecm)) {
            std::cerr << "APX Gazebo Plugin: Entity is not a valid model." << std::endl;
            return;
        }

        if (_sdf && _sdf->HasElement("thrust_max_per_motor_n")) {
            m_thrust_max_per_motor_n = _sdf->Get<double>("thrust_max_per_motor_n");
        }
        if (_sdf && _sdf->HasElement("drag_area_m2")) {
            m_drag_area_m2 = _sdf->Get<double>("drag_area_m2");
        }

        // Wind comes from the environment so sim_vehicle.py can set it per
        // run without editing the world: APX_SIM_WIND="SPEED,FROM_DEG[,GUST]",
        // APX_SIM_SEED=N.
        WindConfig wind_cfg{};
        uint32_t seed = 1;
        if (const char* env_seed = std::getenv("APX_SIM_SEED")) {
            seed = static_cast<uint32_t>(std::strtoul(env_seed, nullptr, 10));
        }
        if (const char* env_wind = std::getenv("APX_SIM_WIND")) {
            if (!parse_wind_spec(env_wind, wind_cfg)) {
                std::cerr << "APX Gazebo Plugin: ignoring bad APX_SIM_WIND '" << env_wind << "'" << std::endl;
                wind_cfg = WindConfig{};
            }
        }
        m_wind.init(wind_cfg, seed);
        std::printf("APX Gazebo Plugin: drag area %.3f m^2, wind %.1f m/s from %.0f deg, gust sigma %.1f m/s (seed %u)\n",
                    m_drag_area_m2, wind_cfg.speed_ms, wind_cfg.from_deg, wind_cfg.gust_sigma_ms, seed);

        m_marker_pub = m_node.Advertise<msgs::Marker>("/marker");
        m_node.Subscribe("/lidar", &ApxGazeboPlugin::OnLidarScan, this);
        if (const char* env_radar = std::getenv("APX_SIM_RADAR")) {
            m_radar_sim_enabled = (std::strcmp(env_radar, "0") != 0);
        }
        if (_sdf && _sdf->HasElement("radar_enabled")) {
            m_radar_sim_enabled = _sdf->Get<bool>("radar_enabled");
        }
        std::printf("APX Gazebo Plugin: multi-radar simulation %s (visual markers on /marker)\n",
                    m_radar_sim_enabled ? "ENABLED" : "DISABLED");

        // Bind each <motor><joint>...</joint><direction>...</direction></motor>
        // block from the SDF to its joint Entity, in declaration order. That
        // order is what apx_main.cpp's MotorMixer output index refers to
        // (motor1 -> outputs[0], motor2 -> outputs[1], ...).
        if (_sdf) {
            sdf::ElementConstPtr motorElem = _sdf->FindElement("motor");
            int idx = 0;
            while (motorElem && idx < 8) {
                std::string jointName = motorElem->Get<std::string>("joint", "").first;
                std::string linkName = motorElem->Get<std::string>("link", "").first;
                int direction = motorElem->Get<int>("direction", 1).first;
                if (!jointName.empty()) {
                    Entity jointEntity = m_model.JointByName(_ecm, jointName);
                    if (jointEntity != kNullEntity) {
                        MotorBinding mb;
                        mb.joint = jointEntity;
                        mb.direction = direction;
                        mb.output_index = idx;

                        // Position of the motor relative to base_link, read
                        // from the link's own Pose component rather than
                        // hardcoded, so this stays correct if the SDF
                        // layout ever changes without touching this file.
                        if (!linkName.empty()) {
                            Entity linkEntity = m_model.LinkByName(_ecm, linkName);
                            if (linkEntity != kNullEntity) {
                                auto *poseComp = _ecm.Component<components::Pose>(linkEntity);
                                if (poseComp) {
                                    mb.local_offset = poseComp->Data().Pos();
                                }
                            }
                        }

                        m_motors.push_back(mb);
                    } else {
                        std::cerr << "APX Gazebo Plugin: motor " << idx
                                  << " references unknown joint '" << jointName << "'" << std::endl;
                    }
                }
                motorElem = motorElem->GetNextElement("motor");
                ++idx;
            }
        }
        std::printf("APX Gazebo Plugin: bound %zu motor joint(s) for actuator commands\n", m_motors.size());

        // Optional <spray> block: nozzle particle emitters driven by the
        // sprayer pump's servo output (see sitl/models/apx_hex/model.sdf).
        if (_sdf && _sdf->HasElement("spray")) {
            sdf::ElementConstPtr sprayElem = _sdf->FindElement("spray");
            m_spray_pump_channel = sprayElem->Get<int>("pump_channel", 1).first - 1;
            m_spray_max_rate = sprayElem->Get<double>("max_rate", 600.0).first;
            for (sdf::ElementConstPtr t = sprayElem->FindElement("topic"); t; t = t->GetNextElement("topic")) {
                m_spray_pubs.push_back(m_node.Advertise<msgs::ParticleEmitter>(t->Get<std::string>()));
            }
            std::printf("APX Gazebo Plugin: spray visual on %zu nozzle(s), pump on servo output %d\n",
                        m_spray_pubs.size(), m_spray_pump_channel + 1);
        }

        Entity baseLinkEntity = m_model.LinkByName(_ecm, "base_link");
        if (baseLinkEntity != kNullEntity) {
            m_base_link = Link(baseLinkEntity);
            // Velocities are NOT populated by default (perf trade-off in
            // gz-sim) - without this, WorldLinearVelocity()/
            // WorldAngularVelocity() always return nullopt.
            m_base_link.EnableVelocityChecks(_ecm, true);
            // Same for accelerations (the IMU model needs specific force).
            m_base_link.EnableAccelerationChecks(_ecm, true);
        } else {
            std::cerr << "APX Gazebo Plugin: model has no 'base_link' - "
                      << "position/attitude telemetry will stay at the origin." << std::endl;
        }

        m_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
        m_server_addr.sin_family = AF_INET;
        // Physics bridge port: sim_vehicle.py sets APX_SIM_PORT (9003 + 10*instance)
        // so several SITL instances can run side by side.
        int sim_port = 9003;
        if (const char* env_port = std::getenv("APX_SIM_PORT")) {
            const int p = std::atoi(env_port);
            if (p > 0 && p < 65536) {
                sim_port = p;
            }
        }
        m_server_addr.sin_port = htons(static_cast<uint16_t>(sim_port));
        inet_pton(AF_INET, "127.0.0.1", &m_server_addr.sin_addr);

        // Non-blocking socket for receiving actuator outputs
        struct timeval tv{0, 0};
        setsockopt(m_udp_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::cout << "APX Gazebo Plugin configured for model: " << m_model.Name(_ecm) << std::endl;
    }

    // Runs BEFORE the physics step, with a mutable EntityComponentManager -
    // this is where joint/actuator commands must be written. (PostUpdate
    // below only gets a const EntityComponentManager since it runs after
    // the physics step and is meant for reading state out, not writing
    // commands into it - that's why the previous version of this plugin
    // could receive motor_throttle but never actually spin anything.)
    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override {
        if (_info.paused) return;

        // Drain the actuator UDP socket - keep only the newest packet.
        SitlActuatorOutput output;
        while (true) {
            ssize_t n = recvfrom(m_udp_socket, &output, sizeof(output), MSG_DONTWAIT, nullptr, nullptr);
            if (n == sizeof(SitlActuatorOutput)) {
                m_last_output = output;
                m_have_output = true;
            } else {
                break;
            }
        }

        // Wind and body drag act whether or not the firmware is talking.
        const double dt_s = std::chrono::duration<double>(_info.dt).count();
        const double sim_time_s = std::chrono::duration<double>(_info.simTime).count();
        const float* wind_ned = m_wind.update(static_cast<float>(dt_s));
        m_wind_enu = math::Vector3d(wind_ned[1], wind_ned[0], -wind_ned[2]);

        update_windsock_and_trees(_ecm, sim_time_s);

        if (m_drag_area_m2 > 0.0) {
            auto velOpt = m_base_link.WorldLinearVelocity(_ecm);
            if (velOpt) {
                const math::Vector3d v_air = *velOpt - m_wind_enu;
                const math::Vector3d drag = -0.5 * kAirDensity * m_drag_area_m2 * v_air.Length() * v_air;
                m_base_link.AddWorldForce(_ecm, drag);
            }
        }

        if (!m_have_output) return;

        update_spray_visual();

        auto poseOpt = m_base_link.WorldPose(_ecm);

        for (const auto &motor : m_motors) {
            float throttle = 0.0f;
            if (motor.output_index >= 0 && motor.output_index < 8) {
                throttle = m_last_output.motor_throttle[motor.output_index];
            }
            throttle = static_cast<float>(std::clamp(static_cast<double>(throttle), 0.0, 1.0));

            // Visual/kinematic prop spin (unrelated to the thrust force
            // below - this is purely so the mesh actually rotates).
            double vel = static_cast<double>(motor.direction) * throttle * kMaxPropSpinRadS;
            auto *cmd = _ecm.Component<components::JointVelocityCmd>(motor.joint);
            if (cmd) {
                cmd->Data()[0] = vel;
            } else {
                _ecm.CreateComponent(motor.joint, components::JointVelocityCmd({vel}));
            }

            // Real thrust: force along the vehicle's own +Z (so a tilted
            // vehicle redirects thrust, same as a real rotor), applied at
            // this motor's position on base_link, plus a small reaction
            // torque about that same axis. This is what previously didn't
            // exist at all - motors could spin visually and the vehicle
            // would still never leave the ground.
            if (poseOpt) {
                double thrustN = m_thrust_max_per_motor_n * static_cast<double>(throttle) * static_cast<double>(throttle);
                math::Vector3d forceWorld = poseOpt->Rot().RotateVector(math::Vector3d(0, 0, thrustN));
                math::Vector3d torqueWorld = poseOpt->Rot().RotateVector(
                    math::Vector3d(0, 0, motor.direction * kYawDragCoeff * thrustN));
                m_base_link.AddWorldWrench(_ecm, forceWorld, torqueWorld, motor.local_offset);
            }
        }
    }

    void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override {
        if (_info.paused) return;

        SitlPhysicsState state{};
        state.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count();

        auto poseOpt = m_base_link.WorldPose(_ecm);
        auto linVelOpt = m_base_link.WorldLinearVelocity(_ecm);
        auto angVelOpt = m_base_link.WorldAngularVelocity(_ecm);
        auto linAccOpt = m_base_link.WorldLinearAcceleration(_ecm);

        if (poseOpt) {
            const math::Pose3d &pose = *poseOpt;

            // Gazebo's world frame here is treated as ENU (X=East, Y=North,
            // Z=Up) with an FLU body (X=forward, Y=left, Z=up) - the usual
            // convention for a robot spawned with identity orientation.
            // MAVLink navigation expects NED position and FRD body attitude.
            state.position_ned[0] = pose.Pos().Y();   // North
            state.position_ned[1] = pose.Pos().X();   // East
            state.position_ned[2] = -pose.Pos().Z();  // Down

            // Orientation: a true frame-rotation (not a per-component
            // euler flip, which has gimbal-lock edge cases) via quaternion
            // composition. ENU->NED and FLU->FRD are each a fixed 180-deg
            // rotation - about the (1,1,0)/sqrt(2) axis for the world
            // conversion, about body X for the body conversion. Verified
            // against a live SITL run (identity spawn -> yaw=+90deg, i.e.
            // NED "facing east", matching the old per-component formula
            // this replaces) rather than trusted on paper alone.
            static const math::Quaterniond kEnu2Ned(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
            static const math::Quaterniond kFlu2Frd(0.0, 1.0, 0.0, 0.0);
            math::Quaterniond q_ned_frd = kEnu2Ned * pose.Rot() * kFlu2Frd;
            state.quaternion[0] = q_ned_frd.W();
            state.quaternion[1] = q_ned_frd.X();
            state.quaternion[2] = q_ned_frd.Y();
            state.quaternion[3] = q_ned_frd.Z();

            math::Vector3d euler = q_ned_frd.Euler();
            state.euler_angles[0] = euler.X();
            state.euler_angles[1] = euler.Y();
            state.euler_angles[2] = wrap_pi(euler.Z());

            state.gps_lat = kOriginLat + (state.position_ned[0] / kEarthRadiusM) * (180.0 / M_PI);
            state.gps_lon = kOriginLon + (state.position_ned[1] /
                                          (kEarthRadiusM * std::cos(kOriginLat * M_PI / 180.0))) * (180.0 / M_PI);
            state.gps_alt = kOriginAlt - state.position_ned[2];

            if (angVelOpt) {
                // World-frame angular velocity -> body FLU -> body FRD
                // (same kFlu2Frd used for orientation above - a rotation
                // *rate* about an axis just flips sign with that axis, no
                // extra yaw offset needed unlike the absolute heading).
                math::Vector3d bodyRateFlu = pose.Rot().RotateVectorReverse(*angVelOpt);
                math::Vector3d bodyRateFrd = kFlu2Frd.RotateVector(bodyRateFlu);
                state.gyro_body[0] = bodyRateFrd.X();
                state.gyro_body[1] = bodyRateFrd.Y();
                state.gyro_body[2] = bodyRateFrd.Z();
            }

            if (linAccOpt) {
                // Specific force = acceleration - gravity (world ENU, gravity
                // -Z), then into body FLU -> FRD. At rest this reads -g on
                // body Z, as an accelerometer does.
                const math::Vector3d specificWorld = *linAccOpt + math::Vector3d(0.0, 0.0, kGravity);
                math::Vector3d bodyFlu = pose.Rot().RotateVectorReverse(specificWorld);
                math::Vector3d bodyFrd = kFlu2Frd.RotateVector(bodyFlu);
                state.accel_body[0] = bodyFrd.X();
                state.accel_body[1] = bodyFrd.Y();
                state.accel_body[2] = bodyFrd.Z();
            }
        } else {
            // No base_link / no WorldPose yet (e.g. very first update) -
            // report the origin rather than fabricating movement.
            state.gps_lat = kOriginLat;
            state.gps_lon = kOriginLon;
            state.gps_alt = kOriginAlt;
        }

        if (linVelOpt) {
            state.velocity_ned[0] = linVelOpt->Y();
            state.velocity_ned[1] = linVelOpt->X();
            state.velocity_ned[2] = -linVelOpt->Z();
            state.airspeed_ms = (*linVelOpt - m_wind_enu).Length();
        }

        const float* w = m_wind.ned();
        state.wind_vel_ned[0] = w[0];
        state.wind_vel_ned[1] = w[1];
        state.wind_vel_ned[2] = w[2];

        // ISA atmosphere, same formula as the built-in runner.
        state.baro_pressure_pa = 101325.0 * std::pow(1.0 - (state.gps_alt / 44330.0), 5.255);
        state.baro_temp_c = 15.0 - 0.0065 * state.gps_alt;
        state.gps_fix = 3; // 3D fix

        // Multi-radar ray casting & simulation (10 channels)
        // 0: Down, 1: Front, 2: Front-Right, 3: Right, 4: Rear-Right,
        // 5: Rear, 6: Rear-Left, 7: Left, 8: Front-Left, 9: Top
        static const math::Vector3d kRadarDirsFlu[10] = {
            math::Vector3d(0.0, 0.0, -1.0),                                // Down
            math::Vector3d(1.0, 0.0, 0.0),                                 // Front
            math::Vector3d(M_SQRT1_2, -M_SQRT1_2, 0.0),                   // Front-Right
            math::Vector3d(0.0, -1.0, 0.0),                                // Right
            math::Vector3d(-M_SQRT1_2, -M_SQRT1_2, 0.0),                  // Rear-Right
            math::Vector3d(-1.0, 0.0, 0.0),                                // Rear
            math::Vector3d(-M_SQRT1_2, M_SQRT1_2, 0.0),                   // Rear-Left
            math::Vector3d(0.0, 1.0, 0.0),                                 // Left
            math::Vector3d(M_SQRT1_2, M_SQRT1_2, 0.0),                    // Front-Left
            math::Vector3d(0.0, 0.0, 1.0)                                  // Top
        };

        state.radar_valid_mask = 0;
        constexpr double kMaxRadarRange = 40.0; // 40m
        constexpr double kMinRadarRange = 0.02;  // 2cm (allows valid detection even while resting on landing gear)

        const auto now_wall = std::chrono::steady_clock::now();
        const bool should_pub_marker = m_radar_sim_enabled && 
            (now_wall - m_last_marker_pub > std::chrono::milliseconds(50));
        if (should_pub_marker) {
            m_last_marker_pub = now_wall;
        }

        if (poseOpt) {
            const math::Pose3d &drone_pose = *poseOpt;
            const math::Vector3d drone_pos = drone_pose.Pos();
            const math::Quaterniond drone_rot = drone_pose.Rot();

            for (int r = 0; r < 10; ++r) {
                math::Vector3d dir_world = drone_rot.RotateVector(kRadarDirsFlu[r]);
                dir_world.Normalize();

                double hit_distance = kMaxRadarRange;
                bool hit = false;

                // For downward altimeter (r = 0): use GPU Lidar sensor reading if available
                const bool use_lidar = (r == 0) && m_has_lidar &&
                    (now_wall - m_last_lidar_time < std::chrono::milliseconds(500));
                if (use_lidar) {
                    hit_distance = m_lidar_distance;
                    hit = (hit_distance >= kMinRadarRange && hit_distance <= kMaxRadarRange);
                } else if (dir_world.Z() < -1e-4) {
                    // Raycast against ground plane (Z = 0)
                    double s = -drone_pos.Z() / dir_world.Z();
                    if (s > 0.0 && s <= kMaxRadarRange) {
                        hit_distance = s;
                        hit = true;
                    }
                }

                if (hit && hit_distance >= kMinRadarRange && hit_distance <= kMaxRadarRange) {
                    state.radar_distances[r] = static_cast<float>(hit_distance);
                    state.radar_valid_mask |= (1 << r);
                } else {
                    state.radar_distances[r] = static_cast<float>(kMaxRadarRange);
                }

                // Publish visual laser/radar beam and detection point to Gazebo
                if (should_pub_marker) {
                    const math::Vector3d hit_pt = drone_pos + dir_world * (hit ? hit_distance : std::min(hit_distance, 15.0));

                    // Laser/radar beam line
                    msgs::Marker beam_msg;
                    beam_msg.set_ns("apx_radar_beam");
                    beam_msg.set_id(r);
                    beam_msg.set_action(msgs::Marker_Action_ADD_MODIFY);
                    beam_msg.set_type(msgs::Marker_Type_LINE_STRIP);
                    
                    auto *p1 = beam_msg.add_point();
                    p1->set_x(drone_pos.X());
                    p1->set_y(drone_pos.Y());
                    p1->set_z(drone_pos.Z());

                    auto *p2 = beam_msg.add_point();
                    p2->set_x(hit_pt.X());
                    p2->set_y(hit_pt.Y());
                    p2->set_z(hit_pt.Z());

                    auto *mat = beam_msg.mutable_material();
                    auto *amb = mat->mutable_ambient();
                    auto *diff = mat->mutable_diffuse();
                    if (r == 0) {
                        // Downward terrain-following sensor: vibrant cyan
                        amb->set_r(0.0f); amb->set_g(0.95f); amb->set_b(1.0f); amb->set_a(0.90f);
                        diff->set_r(0.0f); diff->set_g(0.95f); diff->set_b(1.0f); diff->set_a(0.90f);
                    } else if (r == 9) {
                        // Top ceiling sensor: purple
                        amb->set_r(0.9f); amb->set_g(0.2f); amb->set_b(0.9f); amb->set_a(0.75f);
                        diff->set_r(0.9f); diff->set_g(0.2f); diff->set_b(0.9f); diff->set_a(0.75f);
                    } else {
                        // Perimeter horizontal sensors: bright green
                        amb->set_r(0.1f); amb->set_g(0.95f); amb->set_b(0.3f); amb->set_a(0.65f);
                        diff->set_r(0.1f); diff->set_g(0.95f); diff->set_b(0.3f); diff->set_a(0.65f);
                    }
                    m_marker_pub.Publish(beam_msg);

                    // For downward radar (channel 0): publish high-visibility 3D beam cylinder and ground footprint disk
                    if (r == 0 && hit_distance > 0.03) {
                        // 1. 3D cylindrical beam connecting drone to ground
                        msgs::Marker beam_3d;
                        beam_3d.set_ns("apx_radar_down_beam");
                        beam_3d.set_id(0);
                        beam_3d.set_action(msgs::Marker_Action_ADD_MODIFY);
                        beam_3d.set_type(msgs::Marker_Type_CYLINDER);

                        const math::Vector3d mid_pt = (drone_pos + hit_pt) * 0.5;
                        auto *bpose = beam_3d.mutable_pose();
                        bpose->mutable_position()->set_x(mid_pt.X());
                        bpose->mutable_position()->set_y(mid_pt.Y());
                        bpose->mutable_position()->set_z(mid_pt.Z());

                        math::Quaterniond beam_rot;
                        beam_rot.SetFrom2Axes(math::Vector3d::UnitZ, dir_world);
                        bpose->mutable_orientation()->set_w(beam_rot.W());
                        bpose->mutable_orientation()->set_x(beam_rot.X());
                        bpose->mutable_orientation()->set_y(beam_rot.Y());
                        bpose->mutable_orientation()->set_z(beam_rot.Z());

                        auto *bscale = beam_3d.mutable_scale();
                        bscale->set_x(0.06); // 6cm beam diameter
                        bscale->set_y(0.06);
                        bscale->set_z(hit_distance);

                        auto *bmat = beam_3d.mutable_material();
                        bmat->mutable_ambient()->set_r(0.0f);
                        bmat->mutable_ambient()->set_g(0.90f);
                        bmat->mutable_ambient()->set_b(1.0f);
                        bmat->mutable_ambient()->set_a(0.65f);
                        bmat->mutable_diffuse()->set_r(0.0f);
                        bmat->mutable_diffuse()->set_g(0.90f);
                        bmat->mutable_diffuse()->set_b(1.0f);
                        bmat->mutable_diffuse()->set_a(0.65f);
                        m_marker_pub.Publish(beam_3d);

                        // 2. High-visibility ground target footprint disk on the terrain surface
                        msgs::Marker target_disk;
                        target_disk.set_ns("apx_radar_target");
                        target_disk.set_id(0);
                        target_disk.set_action(msgs::Marker_Action_ADD_MODIFY);
                        target_disk.set_type(msgs::Marker_Type_CYLINDER);

                        auto *tpose = target_disk.mutable_pose();
                        tpose->mutable_position()->set_x(hit_pt.X());
                        tpose->mutable_position()->set_y(hit_pt.Y());
                        tpose->mutable_position()->set_z(hit_pt.Z() + 0.015);

                        auto *tscale = target_disk.mutable_scale();
                        tscale->set_x(0.70); // 70cm diameter ground target circle
                        tscale->set_y(0.70);
                        tscale->set_z(0.02); // 2cm disk height

                        auto *tmat = target_disk.mutable_material();
                        tmat->mutable_ambient()->set_r(0.0f);
                        tmat->mutable_ambient()->set_g(0.95f);
                        tmat->mutable_ambient()->set_b(1.0f);
                        tmat->mutable_ambient()->set_a(0.85f);
                        tmat->mutable_diffuse()->set_r(0.0f);
                        tmat->mutable_diffuse()->set_g(0.95f);
                        tmat->mutable_diffuse()->set_b(1.0f);
                        tmat->mutable_diffuse()->set_a(0.85f);
                        m_marker_pub.Publish(target_disk);
                    }

                    // Detection hit point sphere
                    if (hit) {
                        msgs::Marker hit_msg;
                        hit_msg.set_ns("apx_radar_hit");
                        hit_msg.set_id(r);
                        hit_msg.set_action(msgs::Marker_Action_ADD_MODIFY);
                        hit_msg.set_type(msgs::Marker_Type_SPHERE);
                        
                        const double sph_size = (r == 0) ? 0.25 : 0.16;
                        auto *scale = hit_msg.mutable_scale();
                        scale->set_x(sph_size); scale->set_y(sph_size); scale->set_z(sph_size);
                        
                        auto *hpose = hit_msg.mutable_pose();
                        hpose->mutable_position()->set_x(hit_pt.X());
                        hpose->mutable_position()->set_y(hit_pt.Y());
                        hpose->mutable_position()->set_z(hit_pt.Z() + (r == 0 ? 0.04 : 0.0));

                        auto *hmat = hit_msg.mutable_material();
                        auto *hamb = hmat->mutable_ambient();
                        auto *hdiff = hmat->mutable_diffuse();
                        if (r == 0) {
                            hamb->set_r(0.0f); hamb->set_g(0.95f); hamb->set_b(1.0f); hamb->set_a(0.95f);
                            hdiff->set_r(0.0f); hdiff->set_g(0.95f); hdiff->set_b(1.0f); hdiff->set_a(0.95f);
                        } else if (hit_distance < 2.0) {
                            hamb->set_r(1.0f); hamb->set_g(0.1f); hamb->set_b(0.1f); hamb->set_a(0.95f);
                            hdiff->set_r(1.0f); hdiff->set_g(0.1f); hdiff->set_b(0.1f); hdiff->set_a(0.95f);
                        } else if (hit_distance < 5.0) {
                            hamb->set_r(1.0f); hamb->set_g(0.55f); hamb->set_b(0.0f); hamb->set_a(0.9f);
                            hdiff->set_r(1.0f); hdiff->set_g(0.55f); hdiff->set_b(0.0f); hdiff->set_a(0.9f);
                        } else {
                            hamb->set_r(0.2f); hamb->set_g(0.9f); hamb->set_b(1.0f); hamb->set_a(0.85f);
                            hdiff->set_r(0.2f); hdiff->set_g(0.9f); hdiff->set_b(1.0f); hdiff->set_a(0.85f);
                        }
                        m_marker_pub.Publish(hit_msg);
                    }
                }
            }
        }

        sendto(m_udp_socket, &state, sizeof(state), 0,
               (const struct sockaddr*)&m_server_addr, sizeof(m_server_addr));
    }

private:
    // Pump servo PWM -> nozzle emitters: 1100 full off, 1900 full on.
    // Particle rate proportional to pump output. Published only when it changes.
    void update_spray_visual() {
        if (m_spray_pubs.empty() || m_spray_pump_channel < 0 || m_spray_pump_channel >= 16) {
            return;
        }
        const uint16_t pwm = m_last_output.servo_pwm[m_spray_pump_channel];
        // 1100 us = full off (0%), 1900 us = full on (100%)
        const double pump = (pwm <= 1100) ? 0.0 :
            std::clamp((static_cast<double>(pwm) - 1100.0) / (1900.0 - 1100.0), 0.0, 1.0);
        const bool emitting = pump > 0.01;
        const double rate = emitting ? m_spray_max_rate * pump : 0.0;
        if (emitting == m_spray_emitting && std::fabs(rate - m_spray_rate) < 0.02 * m_spray_max_rate) {
            return;
        }
        m_spray_emitting = emitting;
        m_spray_rate = rate;
        msgs::ParticleEmitter msg;
        msg.mutable_emitting()->set_data(emitting);
        msg.mutable_rate()->set_data(static_cast<float>(rate));
        for (auto &pub : m_spray_pubs) {
            pub.Publish(msg);
        }
        std::printf("[APX Gazebo Plugin] Spray visual: pump_pwm=%u, emitting=%s, rate=%.1f\n",
                    pwm, emitting ? "ON" : "OFF", rate);
    }

    struct MotorBinding {
        Entity joint{kNullEntity};
        int direction{1};
        int output_index{-1};
        math::Vector3d local_offset{0, 0, 0}; // relative to base_link
    };

    Model m_model;
    Link m_base_link{kNullEntity};
    int m_udp_socket;
    struct sockaddr_in m_server_addr;

    std::vector<MotorBinding> m_motors;
    SitlActuatorOutput m_last_output{};
    bool m_have_output{false};
    // Per-motor max thrust (N) - was a single compile-time constant
    // (kThrustMaxPerMotorN, still the default below) shared by every model
    // using this plugin, which was fine while only one ~2.2kg quad model
    // existed but can't be right for a drastically different airframe
    // (a 16-25kg hex needs roughly an order of magnitude more thrust per
    // motor just to hover). Now an optional <thrust_max_per_motor_n> SDF
    // plugin parameter, read in Configure() below, defaulting to the
    // original 11.0 so the existing quad model's behavior is unchanged.
    double m_thrust_max_per_motor_n{11.0};

    // Body drag (Cd * frontal area, m^2) against the wind; optional
    // <drag_area_m2> SDF parameter, 0 = no drag.
    static constexpr double kAirDensity = 1.225;
    static constexpr double kGravity = 9.80665;
    double m_drag_area_m2{0.0};
    WindModel m_wind;
    math::Vector3d m_wind_enu{0, 0, 0};

    transport::Node m_node;
    transport::Node::Publisher m_marker_pub;
    bool m_radar_sim_enabled{true};
    std::chrono::steady_clock::time_point m_last_marker_pub{};

    double m_lidar_distance{40.0};
    bool m_has_lidar{false};
    std::chrono::steady_clock::time_point m_last_lidar_time{};

    void OnLidarScan(const msgs::LaserScan &_msg) {
        double min_r = _msg.range_max();
        bool found = false;
        for (int i = 0; i < _msg.ranges_size(); ++i) {
            double r = _msg.ranges(i);
            if (r >= _msg.range_min() && r <= _msg.range_max() && !std::isinf(r) && !std::isnan(r)) {
                if (r < min_r) {
                    min_r = r;
                    found = true;
                }
            }
        }
        if (found) {
            m_lidar_distance = min_r;
            m_last_lidar_time = std::chrono::steady_clock::now();
            m_has_lidar = true;
        }
    }

    std::vector<transport::Node::Publisher> m_spray_pubs;
    int m_spray_pump_channel{-1};
    double m_spray_max_rate{600.0};
    bool m_spray_emitting{false};
    double m_spray_rate{0.0};

    struct TreeBinding {
        Entity model{kNullEntity};
        Entity yaw_joint{kNullEntity};
        Entity sway_joint{kNullEntity};
        int id{0};
    };

    void scan_world_visual_indicators(EntityComponentManager &_ecm) {
        std::unordered_map<Entity, TreeBinding> tree_by_model;
        m_windsock_yaw_joint = kNullEntity;
        m_windsock_pitch_joint = kNullEntity;
        m_trees.clear();

        _ecm.Each<components::Name, components::ParentEntity>([&](const Entity &entity,
                                                                  const components::Name *nameComp,
                                                                  const components::ParentEntity *parentComp) -> bool {
            if (!nameComp) return true;
            const std::string &name = nameComp->Data();
            if (name == "windsock_yaw_joint") {
                m_windsock_yaw_joint = entity;
            } else if (name == "windsock_pitch_joint") {
                m_windsock_pitch_joint = entity;
            } else if (name == "tree_yaw_joint") {
                if (parentComp) {
                    tree_by_model[parentComp->Data()].yaw_joint = entity;
                    tree_by_model[parentComp->Data()].model = parentComp->Data();
                }
            } else if (name == "tree_sway_joint") {
                if (parentComp) {
                    tree_by_model[parentComp->Data()].sway_joint = entity;
                    tree_by_model[parentComp->Data()].model = parentComp->Data();
                }
            }
            return true;
        });

        int id = 0;
        for (auto &kv : tree_by_model) {
            kv.second.id = id++;
            m_trees.push_back(kv.second);
        }

        m_scan_retry_count++;
        if (m_windsock_yaw_joint != kNullEntity || !m_trees.empty() || m_scan_retry_count > 60) {
            std::printf("[APX Gazebo Plugin] Visual wind indicators found: windsock (yaw=%lu, pitch=%lu), %zu tree(s)\n",
                        static_cast<unsigned long>(m_windsock_yaw_joint),
                        static_cast<unsigned long>(m_windsock_pitch_joint),
                        m_trees.size());
            m_scanned_visual_entities = true;
        }
    }

    void update_windsock_and_trees(EntityComponentManager &_ecm, double sim_time_s) {
        if (!m_scanned_visual_entities) {
            scan_world_visual_indicators(_ecm);
        }

        // 30 Hz is plenty for the eye. Every JointPositionReset is a one-time
        // ECM change, and SceneBroadcaster publishes the whole scene state to
        // the GUI on each one - at the 1 kHz physics rate that dragged the
        // sim to ~0.4x real time whenever the GUI was open. The motion is a
        // function of sim time, so its speed is unchanged.
        constexpr double kIndicatorPeriodS = 1.0 / 30.0;
        if (sim_time_s - m_last_indicator_update_s < kIndicatorPeriodS && sim_time_s >= m_last_indicator_update_s) {
            return;
        }
        const double dt_s = (m_last_indicator_update_s < 0.0) ? kIndicatorPeriodS :
                            std::clamp(sim_time_s - m_last_indicator_update_s, 0.001, 0.5);
        m_last_indicator_update_s = sim_time_s;

        const double vx = m_wind_enu.X();
        const double vy = m_wind_enu.Y();
        const double speed_h = std::hypot(vx, vy);

        // Instantaneous downwind direction (bearing wind is blowing towards in ENU)
        const double instantaneous_yaw = std::atan2(vy, vx);

        if (!m_indicator_initialized) {
            if (speed_h > 0.1) {
                m_prevailing_wind_yaw = instantaneous_yaw;
                m_windsock_yaw = instantaneous_yaw;
                m_indicator_initialized = true;
            }
        }

        // 1. Windsock update
        if (m_windsock_yaw_joint != kNullEntity || m_windsock_pitch_joint != kNullEntity) {
            // Mechanical response filter: windsock yaw tracks wind direction smoothly
            // when wind speed is above calm air threshold (> 0.25 m/s).
            if (speed_h > 0.25) {
                const double alpha_sock = 1.0 - std::exp(-dt_s / 1.0);
                m_windsock_yaw = wrap_pi(m_windsock_yaw + alpha_sock * wrap_pi(instantaneous_yaw - m_windsock_yaw));
            }

            // High-frequency fluttering only appears in brisk wind (> 1.2 m/s), scaling smoothly
            const double flutter_factor = std::clamp((speed_h - 1.2) / 5.0, 0.0, 1.0);
            const double yaw_flutter = 0.03 * flutter_factor * std::sin(5.5 * sim_time_s);
            const double pitch_flutter = 0.02 * flutter_factor * std::sin(11.0 * sim_time_s + 0.8);

            const double sock_yaw = wrap_pi(m_windsock_yaw + yaw_flutter);
            // Droop: ~0.95 rad in calm air (~54 deg down), lifts towards ~0 rad in high wind
            const double target_pitch = std::max(0.0, 0.95 * std::exp(-speed_h / 2.8) + pitch_flutter);
            const double alpha_pitch = 1.0 - std::exp(-dt_s / 0.8);
            m_windsock_pitch += alpha_pitch * (target_pitch - m_windsock_pitch);

            if (m_windsock_yaw_joint != kNullEntity) {
                set_joint_position(_ecm, m_windsock_yaw_joint, sock_yaw);
            }
            if (m_windsock_pitch_joint != kNullEntity) {
                set_joint_position(_ecm, m_windsock_pitch_joint, m_windsock_pitch);
            }
        }

        // 2. Prevailing wind tracking for trees
        // Trees do not weather-vane to rapid gust fluctuations or zero-crossings.
        // They only align with the sustained prevailing wind direction with a slow time constant (tau = 6.0s),
        // and only when wind speed is sustained above 0.6 m/s.
        if (speed_h > 0.6) {
            const double alpha_tree = 1.0 - std::exp(-dt_s / 6.0);
            m_prevailing_wind_yaw = wrap_pi(m_prevailing_wind_yaw + alpha_tree * wrap_pi(instantaneous_yaw - m_prevailing_wind_yaw));
        }

        // 3. Tree sway update
        for (const auto &tree : m_trees) {
            if (tree.yaw_joint != kNullEntity) {
                set_joint_position(_ecm, tree.yaw_joint, m_prevailing_wind_yaw);
            }
            if (tree.sway_joint != kNullEntity) {
                // Natural sway frequency for large mature trees (4.5m tall): ~0.18 - 0.28 Hz (3.5 - 5.5 s period)
                // Distinct frequency and phase per tree so they do not sway in rigid unison
                const double freq = 2.0 * M_PI * (0.20 + 0.02 * (tree.id % 5));
                const double phase = 1.25 * tree.id;

                // Mean aerodynamic deflection leaning downwind (proportional to dynamic pressure ~ v^2)
                const double mean_bend = std::min(0.18, 0.0025 * speed_h * speed_h);

                // Dynamic sway oscillation: strictly 0 in calm air, scaling smoothly with wind speed
                // In calm air (0 m/s): amp = 0 rad (stationary)
                // In light breeze (1.5 m/s): amp ≈ 0.0034 rad (~0.2 deg, gentle natural rustle)
                // In moderate wind (5.0 m/s): amp ≈ 0.029 rad (~1.7 deg)
                // In strong wind (10.0 m/s): amp ≈ 0.08 rad (~4.6 deg)
                const double amp = std::min(0.08, 0.0008 * speed_h + 0.0010 * speed_h * speed_h);
                const double sway = mean_bend + amp * std::sin(freq * sim_time_s + phase);

                set_joint_position(_ecm, tree.sway_joint, sway);
            }
        }
    }

    static void set_joint_position(EntityComponentManager &_ecm, Entity joint, double pos) {
        if (joint == kNullEntity) return;
        auto *reset = _ecm.Component<components::JointPositionReset>(joint);
        if (reset) {
            reset->Data() = {pos};
        } else {
            _ecm.CreateComponent(joint, components::JointPositionReset({pos}));
        }
    }

    Entity m_windsock_yaw_joint{kNullEntity};
    Entity m_windsock_pitch_joint{kNullEntity};
    std::vector<TreeBinding> m_trees;
    bool m_scanned_visual_entities{false};
    double m_last_indicator_update_s{-1.0e9};
    int m_scan_retry_count{0};

    // Filtered visual indicator states for smooth physical motion
    double m_windsock_yaw{0.0};
    double m_windsock_pitch{0.95};
    double m_prevailing_wind_yaw{0.0};
    bool m_indicator_initialized{false};
};

} // namespace sitl
} // namespace apx

GZ_ADD_PLUGIN(apx::sitl::ApxGazeboPlugin, gz::sim::System,
              apx::sitl::ApxGazeboPlugin::ISystemConfigure,
              apx::sitl::ApxGazeboPlugin::ISystemPreUpdate,
              apx::sitl::ApxGazeboPlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(apx::sitl::ApxGazeboPlugin, "apx_gazebo_plugin")
