// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#include <atomic>

#if defined(APX_EMBEDDED)
#include "apx/kernel/mutex.hpp"
namespace apx::gcs {
using MutexType = apx::kernel::Mutex;
using LockGuardType = apx::kernel::LockGuard;
}
#else
#include <mutex>
#include <thread>
#include <netinet/in.h>
namespace apx::gcs {
using MutexType = std::mutex;
using LockGuardType = std::lock_guard<std::mutex>;
}
#endif

// Requires standard MAVLink C library v2 in include path
#include <mavlink.h>
#include "command_handler.hpp"
#include "mode_manager.hpp"

namespace apx::gcs {

struct VehicleTelemetryState {
    float roll{0.0f};
    float pitch{0.0f};
    float yaw{0.0f};
    float rollspeed{0.0f};
    float pitchspeed{0.0f};
    float yawspeed{0.0f};
    double lat{18.5204};
    double lon{73.8567};
    float alt_msl{540.0f};
    float rel_alt{0.0f};
    float vx{0.0f};
    float vy{0.0f};
    float vz{0.0f};
    float airspeed{0.0f};
    float groundspeed{0.0f};
    uint16_t throttle_pct{0};
    float battery_voltage{12.6f};
    float battery_current{2.5f};
    int8_t battery_remaining{95};
    bool armed{false};
    uint8_t base_mode{MAV_MODE_FLAG_CUSTOM_MODE_ENABLED};
    uint32_t custom_mode{0}; // 0 = STABILIZE
    uint8_t system_status{MAV_STATE_STANDBY};
    uint16_t motor_pwm[8]{1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    uint16_t servo_pwm[16]{1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1100, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    float wind_direction{0.0f}; // deg (direction wind is coming from, 0..360)
    float wind_speed{0.0f};     // m/s in ground plane
    float wind_speed_z{0.0f};   // m/s vertical
};

// Raw sensor samples for telemetry, one entry per sensor instance:
// HIGHRES_IMU (id = lane) per IMU/baro/mag lane, GPS_RAW_INT and GPS2_RAW
// for the first two GNSS receivers (MAVLink has no message for a third or
// fourth). Without this, GPS_RAW_INT is built from the vehicle state.
struct RawSensorTelemetry {
    static constexpr int kMaxImu = 4;
    static constexpr int kMaxGps = 4;
    struct Imu {
        bool present{false};
        uint64_t time_us{0};
        float accel[3]{};        // m/s^2, FRD
        float gyro[3]{};         // rad/s
        float mag[3]{};          // gauss
        float pressure_hpa{0.0f};
        float pressure_alt_m{0.0f};
        float temp_c{0.0f};
    };
    struct Gps {
        bool present{false};
        uint64_t time_us{0};
        bool fix_3d{false};
        double lat_deg{0.0};
        double lon_deg{0.0};
        float alt_msl_m{0.0f};
        float vel_ned[3]{};
        float hdop{99.99f};
        float vdop{99.99f};
        uint8_t satellites{0};
    };
    int gps_count{0};
    Imu imu[kMaxImu]{};
    Gps gps[kMaxGps]{};
};

class MavlinkHandler {
public:
    MavlinkHandler(uint8_t sysid = 1, uint8_t compid = 1);
    ~MavlinkHandler();

    void init();
    bool start_network(uint16_t udp_port = 14550, const std::vector<uint16_t>& tcp_ports = {5760, 5761, 5762, 5763});
    void stop_network();

    void update(); // periodic update for state & telemetry

    void set_vehicle_state(float roll, float pitch, float yaw,
                           double lat, double lon, float alt, float rel_alt,
                           float vx, float vy, float vz,
                           float battery_v, float battery_a, int8_t battery_remaining,
                           bool armed, uint8_t base_mode, uint32_t custom_mode);

    void send_message(const mavlink_message_t& msg);

    // Serial byte transport (hardware USB CDC / UART): every outgoing frame
    // is passed to the writer; received bytes are parsed with
    // receive_bytes(). Works alongside the hosted UDP/TCP links.
    using ByteWriter = void (*)(const uint8_t* data, size_t len, void* ctx);

    // What the GCS is told about the vehicle itself. Defaults suit SITL
    // (every sensor simulated and healthy, estimates available); a
    // hardware build reports only what actually works, so GCS never shows
    // a healthy, located vehicle that has no working sensors.
    void set_sensor_status(uint32_t present, uint32_t enabled, uint32_t healthy);
    void set_estimates_valid(bool attitude, bool position);

    // Arming gate for builds that cannot fly yet (e.g. hardware without
    // sensors or motor outputs): ARM requests are refused with this reason.
    void set_arming_blocked(const char* reason) { m_arming_block_reason = reason; }
    const char* arming_block_reason() const { return m_arming_block_reason; }
    void set_byte_writer(ByteWriter writer, void* ctx);

    // Reboot requests (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1 1 = reboot,
    // 3 = reboot into the bootloader). Refused unless the platform enabled
    // them and the vehicle is disarmed; the main loop polls
    // take_reboot_request() and performs the reboot after the ACK is sent.
    enum class RebootRequest : uint8_t { None = 0, Reboot = 1, Bootloader = 2 };
    void set_reboot_supported(bool supported) { m_reboot_supported.store(supported); }
    bool reboot_supported() const { return m_reboot_supported.load(); }
    void request_reboot(RebootRequest request) { m_reboot_request.store(static_cast<uint8_t>(request)); }
    RebootRequest take_reboot_request() { return static_cast<RebootRequest>(m_reboot_request.exchange(0)); }
    void receive_bytes(const uint8_t* data, size_t len);

    // Convenience senders
    void send_heartbeat();
    void send_sys_status();
    void send_attitude(uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
    void send_global_position_int(uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg);
    void send_gps_raw_int(uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible);
    void send_battery_status(float voltage_v, float current_a, int8_t battery_remaining);
    void send_vfr_hud(float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb);
    void send_rc_channels(uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi);
    void send_statustext(MAV_SEVERITY severity, const std::string& text);
    void send_command_ack(uint16_t command, uint8_t result);
    void send_autopilot_version();
    void send_servo_output_raw();
    void send_wind(float direction_deg, float speed_ms, float speed_z_ms);
    void set_wind(float direction_deg, float speed_ms, float speed_z_ms);

    uint8_t get_sysid() const { return m_sysid; }
    uint8_t get_compid() const { return m_compid; }

    // Vehicle state mutators used by CommandHandler so ARM/mode changes
    // survive the next periodic telemetry update instead of being
    // overwritten by it.
    void set_armed(bool armed);
    void set_custom_mode(uint32_t custom_mode);
    // HEARTBEAT vehicle type (MAV_TYPE_QUADROTOR / HEXAROTOR / OCTOROTOR);
    // GCSs use it to pick the vehicle icon, motor-test layout and checks.
    void set_vehicle_type(MAV_TYPE type) { m_vehicle_type.store(type); }
    bool get_armed() const;
    uint32_t get_custom_mode() const;

    // apx_main.cpp owns the ModeManager (it's the one driving the
    // mixer/actuator each tick) and hands us a pointer so CommandHandler
    // and the SET_POSITION_TARGET_*/DO_REPOSITION handlers below can reach
    // it - mode switching, takeoff/land requests, and guided targets all
    // go through it rather than through ad hoc state here.
    void set_mode_manager(apx::modes::ModeManager* mgr) { m_mode_manager = mgr; }
    apx::modes::ModeManager* get_mode_manager() const { return m_mode_manager; }

    // Needed to convert SET_POSITION_TARGET_GLOBAL_INT's lat/lon/alt into
    // the NED-relative-to-home coordinates modes work in. Set once at
    // startup by apx_main.cpp from the same --home-lat/lon/alt values
    // seeded into set_vehicle_state().
    void set_home_position(double lat, double lon, float alt_msl, bool locked = false);
    void send_home_position();
    // Vehicle positions are NED from a fixed origin: the first home
    // position (boot). Home itself may move later (it is re-set on arming),
    // so every lat/lon <-> NED conversion goes through the origin, and
    // relative altitudes are measured from home.
    void global_to_ned(double lat, double lon, float rel_alt_m, float ned[3]) const;
    void home_ned(float ned[3]) const;
    double home_lat() const {
        LockGuardType lock(m_state_mutex);
        return m_home_lat;
    }
    double home_lon() const {
        LockGuardType lock(m_state_mutex);
        return m_home_lon;
    }
    float home_alt_msl() const {
        LockGuardType lock(m_state_mutex);
        return m_home_alt_msl;
    }
    bool is_home_set() const {
        LockGuardType lock(m_state_mutex);
        return m_home_set;
    }
    bool is_home_locked() const {
        LockGuardType lock(m_state_mutex);
        return m_home_locked;
    }
    void unlock_home() {
        LockGuardType lock(m_state_mutex);
        m_home_locked = false;
    }

    VehicleTelemetryState get_vehicle_telemetry_state() const;

    // Motor Test controls (MAV_CMD_DO_MOTOR_TEST)
    bool start_motor_test(uint8_t motor_index, uint8_t throttle_type, float throttle_val, float timeout_sec);
    void stop_motor_test();
    bool is_motor_test_active() const;
    uint8_t get_motor_test_index() const;
    float get_motor_test_throttle() const;

    // CommandHandler runs on the network thread, asynchronously from
    // apx_main.cpp's control loop - it needs *some* VehicleState to pass
    // to ModeManager::request_mode()/request_takeoff() (e.g. so a fresh
    // mode's on_enter() captures a sane hold position), but doesn't have
    // one of its own. apx_main.cpp refreshes this every tick (10ms); a
    // command reacting to state that's up to one tick stale is fine here.
    void set_last_known_vehicle_state(const apx::modes::VehicleState& state);
    apx::modes::VehicleState get_last_known_vehicle_state() const;

    // Motor/servo outputs as commanded by apx_main.cpp's mixer, in 0..1
    // throttle. Surfaced over MAVLink as SERVO_OUTPUT_RAW (1000-2000us PWM)
    // so a GCS can show whether motors are actually being driven, instead
    // of that only being visible (or not) inside the simulator.
    void set_motor_outputs(const float outputs[8]);
    void set_raw_sensor_telemetry(const RawSensorTelemetry& raw);
    void set_servo_outputs(const uint16_t pwm[16]);
    void set_aux_servo_pwm(uint8_t channel_1_based, uint16_t pwm);
    uint16_t get_aux_servo_pwm(uint8_t channel_1_based) const;

    // RC override / manual control inputs from APX Flight Manager or companion computer
    struct RcChannels {
        float roll{0.0f};      // -1.0 .. +1.0
        float pitch{0.0f};     // -1.0 .. +1.0
        float yaw{0.0f};       // -1.0 .. +1.0
        float throttle{0.5f};  // 0.0 .. 1.0 (0.5 = hover center)
        bool valid{false};
    };
    RcChannels get_rc_channels() const;

    // GCS connection status helpers
    bool has_active_gcs() const;
    size_t get_active_gcs_count() const;
    std::string get_active_gcs_summary() const;

private:
    std::atomic<MAV_TYPE> m_vehicle_type{MAV_TYPE_QUADROTOR};
    uint8_t m_sysid;
    uint8_t m_compid;
    // MAVLink parser state is per channel, so every link needs its own:
    // feeding two links' bytes into one channel corrupts frames whenever
    // their packets interleave. UDP uses kUdpChannel; each TCP client gets
    // a channel of its own (see channel_for_tcp_fd()).
    static constexpr uint8_t kUdpChannel = MAVLINK_COMM_0;
    static constexpr int kNumParseChannels = MAVLINK_COMM_NUM_BUFFERS;
    // Last channel: the serial byte stream (receive_bytes); TCP clients
    // use 1 .. kSerialChannel - 1.
    static constexpr uint8_t kSerialChannel = static_cast<uint8_t>(kNumParseChannels - 1);
    int m_tcp_channel_fd[kNumParseChannels]{};  // fd owning each channel, -1 = free; index 0 = UDP
    bool m_tcp_channels_init{false};
    uint8_t channel_for_tcp_fd(int fd);
    void release_tcp_channel(int fd);
    
    std::unique_ptr<CommandHandler> m_command_handler;

    uint32_t m_start_time_ms{0};
    uint32_t m_last_heartbeat_ms{0};
    uint32_t m_last_attitude_ms{0};
    uint32_t m_last_position_ms{0};
    uint32_t m_last_sys_status_ms{0};
    uint32_t m_last_gps_raw_ms{0};
    uint32_t m_last_vfr_hud_ms{0};
    uint32_t m_last_servo_output_ms{0};
    uint32_t m_last_rc_channels_ms{0};
    uint32_t m_last_highres_imu_ms{0};
    uint32_t m_last_wind_ms{0};
    uint32_t m_last_fence_status_ms{0};
    uint32_t m_last_home_position_ms{0};
    uint32_t m_last_radar_ms{0};

    void send_distance_sensors();
    void send_obstacle_distance();

    apx::modes::ModeManager* m_mode_manager{nullptr};
    double m_origin_lat{0.0};
    double m_origin_lon{0.0};
    float m_origin_alt_msl{0.0f};
    bool m_origin_set{false};
    double m_home_lat{0.0};
    double m_home_lon{0.0};
    float m_home_alt_msl{0.0f};
    bool m_home_set{false};
    bool m_home_locked{false};

    mutable MutexType m_vehicle_state_mutex;
    apx::modes::VehicleState m_last_known_vehicle_state{};

    mutable MutexType m_state_mutex;
    VehicleTelemetryState m_state{};
    RawSensorTelemetry m_raw_sensors{};   // guarded by m_state_mutex
    bool m_have_raw_sensors{false};

    struct MotorTestState {
        uint8_t motor_index{0}; // 1-based (0 = none)
        float throttle{0.0f};   // 0.0 to 1.0
        uint32_t end_time_ms{0};
        bool active{false};
    };
    mutable MutexType m_motor_test_mutex;
    MotorTestState m_motor_test{};

#if !defined(APX_EMBEDDED)
    // Network members
    int m_udp_sock{-1};
    uint16_t m_udp_port{14550};
    struct sockaddr_in m_udp_target_addr{};

    std::vector<int> m_tcp_server_fds;
    std::vector<uint16_t> m_tcp_ports;
#endif

public:
    struct GcsEndpointInfo {
        int fd{-1};
        std::string interface_name; // e.g. "TCP 5760" or "UDP 14550"
        std::string remote_addr;    // e.g. "127.0.0.1:52134"
        uint16_t port{0};
        uint8_t sysid{0};
        uint8_t compid{0};
        uint32_t last_seen_ms{0};
        bool is_active{true};
    };

    std::vector<GcsEndpointInfo> get_active_gcs_endpoints() const;

private:
#if !defined(APX_EMBEDDED)
    MutexType m_network_mutex;
    std::vector<int> m_client_fds;
    std::vector<GcsEndpointInfo> m_active_endpoints;
    std::vector<struct sockaddr_in> m_udp_remotes;

    std::thread m_network_thread;
    std::atomic<bool> m_network_running{false};

    void network_io_loop();
#else
    bool m_network_running{false};
    void network_io_loop();
#endif
    void process_incoming_byte(uint8_t byte, uint8_t channel);

    void handle_message(const mavlink_message_t& msg);
    void handle_param_request_list(const mavlink_message_t& msg);
    void handle_param_set(const mavlink_message_t& msg);
    void handle_param_request_read(const mavlink_message_t& msg);
    void handle_set_mode(const mavlink_message_t& msg);
    void handle_mission_request_list(const mavlink_message_t& msg);
    void handle_mission_count(const mavlink_message_t& msg);
    void handle_mission_item_int(const mavlink_message_t& msg);
    void handle_mission_request_int(const mavlink_message_t& msg);
    void handle_mission_clear_all(const mavlink_message_t& msg);
    void handle_set_position_target_local_ned(const mavlink_message_t& msg);
    void handle_set_position_target_global_int(const mavlink_message_t& msg);
    void handle_manual_control(const mavlink_message_t& msg);
    void handle_rc_channels_override(const mavlink_message_t& msg);

    mutable MutexType m_rc_mutex;
    RcChannels m_rc_channels;
    uint64_t m_last_rc_update_us{0};

    void send_param_value(const std::string& param_name);

    uint32_t get_time_ms();
    const char* m_arming_block_reason{nullptr}; // nullptr = arming allowed
    ByteWriter m_byte_writer{nullptr};
    // 3D gyro, accel, mag, baro, GPS, battery, RC receiver
    static constexpr uint32_t kSitlSensors = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | (1u << 4) |
                                             (1u << 5) | (1u << 16);
    std::atomic<uint32_t> m_sensors_present{kSitlSensors};
    std::atomic<bool> m_reboot_supported{false};
    std::atomic<uint8_t> m_reboot_request{0};
    std::atomic<uint32_t> m_sensors_enabled{kSitlSensors};
    std::atomic<uint32_t> m_sensors_health{kSitlSensors};
    std::atomic<bool> m_attitude_valid{true};
    std::atomic<bool> m_position_valid{true};
    void* m_byte_writer_ctx{nullptr};
    size_t read_bytes(uint8_t* buf, size_t max_len);
    void write_bytes(const uint8_t* buf, size_t len);

    // --- Mission storage -------------------------------------------------
    // Simplified upload/download state machine:
    //   Upload:   GCS -> MISSION_COUNT -> we request item 0..N-1 one at a
    //             time via MISSION_REQUEST_INT -> GCS replies with
    //             MISSION_ITEM_INT -> once all received we MISSION_ACK.
    //   Download: GCS -> MISSION_REQUEST_LIST -> we reply MISSION_COUNT
    //             (actual stored count) -> GCS requests each item via
    //             MISSION_REQUEST_INT -> we reply MISSION_ITEM_INT.
    struct MissionItem {
        uint16_t seq{0};
        uint8_t frame{0};
        uint16_t command{0};
        uint8_t current{0};
        uint8_t autocontinue{1};
        float param1{0}, param2{0}, param3{0}, param4{0};
        int32_t x{0};
        int32_t y{0};
        float z{0};
        uint8_t mission_type{0};
    };

    mutable MutexType m_mission_mutex;
    std::vector<MissionItem> m_mission_items;

    bool m_mission_upload_active{false};
    uint8_t m_mission_upload_type{0};
    uint16_t m_mission_upload_expected{0};
    uint8_t m_mission_upload_gcs_sysid{0};
    uint8_t m_mission_upload_gcs_compid{0};
    std::vector<MissionItem> m_mission_upload_buffer;

    void mission_request_next_item(uint8_t target_sysid, uint8_t target_compid, uint16_t seq);
    // Converts m_mission_items (raw MAVLink mission storage) to the
    // NED-relative-to-home Waypoint list ModeAuto actually flies - called
    // once a mission upload completes. Skips seq 0: by standard MAVLink/GCS
    // convention that item is a home-position marker, not something to
    // fly to.
    std::vector<apx::modes::Waypoint> convert_mission_to_waypoints() const;

    // Last MISSION_CURRENT seq sent, so update() only re-sends it (and
    // MISSION_ITEM_REACHED) on an actual change, not every tick.
    int m_last_sent_mission_seq{-1};
};

} // namespace apx::gcs
