# GCS & MAVLink API Reference

Namespace: `apx::gcs`

## Summary

This page documents all public classes, data structures, and enumerations for `GCS & MAVLink`.

| Symbol | Kind | Declared In | Description |
|---|---|---|---|
| [`CommandHandler`](#commandhandler) | `class` | `modules/gcs/command_handler.hpp` | Class definition. |
| [`MavlinkHandler`](#mavlinkhandler) | `class` | `modules/gcs/mavlink_handler.hpp` | Class definition. |
| [`VehicleTelemetryState`](#vehicletelemetrystate) | `struct` | `modules/gcs/mavlink_handler.hpp` | Struct definition. |
| [`param_union`](#param_union) | `struct` | `modules/gcs/mavlink/mavlink_types.h` | Struct definition. |
| [`param_union_extended`](#param_union_extended) | `struct` | `modules/gcs/mavlink/mavlink_types.h` | Struct definition. |

---

## CommandHandler
**Kind:** `class`  
**Namespace:** `apx::gcs`  
**Header:** `modules/gcs/command_handler.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `CommandHandler(MavlinkHandler* mavlink)` | Member function. |
| `void` | `handle_command(const mavlink_message_t& msg)` | Member function. |

---

## MavlinkHandler
**Kind:** `class`  
**Namespace:** `apx::gcs::apx::gcs::apx::gcs`  
**Header:** `modules/gcs/mavlink_handler.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `MavlinkHandler(uint8_t sysid = 1, uint8_t compid = 1)` | Member function. |
| `` | `~MavlinkHandler()` | Member function. |
| `void` | `init()` | Member function. |
| `bool` | `start_network(uint16_t udp_port = 14550, const std::vector<uint16_t>& tcp_ports = {5760, 5761, 5762, 5763})` | Member function. |
| `void` | `stop_network()` | Member function. |
| `void` | `update()` | Member function. |
| `void` | `set_vehicle_state(float roll, float pitch, float yaw, double lat, double lon, float alt, float rel_alt, float vx, float vy, float vz, float battery_v, float battery_a, int8_t battery_remaining, bool armed, uint8_t base_mode, uint32_t custom_mode)` | periodic update for state & telemetry |
| `void` | `send_message(const mavlink_message_t& msg)` | Member function. |
| `void` | `send_heartbeat()` | Convenience senders |
| `void` | `send_sys_status()` | Member function. |
| `void` | `send_attitude(uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)` | Member function. |
| `void` | `send_global_position_int(uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)` | Member function. |
| `void` | `send_gps_raw_int(uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)` | Member function. |
| `void` | `send_battery_status(uint16_t voltage_v, int16_t current_a, int8_t battery_remaining)` | Member function. |
| `void` | `send_vfr_hud(float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)` | Member function. |
| `void` | `send_rc_channels(uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)` | Member function. |
| `void` | `send_statustext(MAV_SEVERITY severity, const std::string& text)` | Member function. |
| `void` | `send_command_ack(uint16_t command, uint8_t result)` | Member function. |
| `void` | `send_autopilot_version()` | Member function. |
| `void` | `send_servo_output_raw()` | Member function. |
| `void` | `set_armed(bool armed)` | overwritten by it. |
| `void` | `set_custom_mode(uint32_t custom_mode)` | Member function. |
| `bool` | `get_armed()` | Member function. |
| `uint32_t` | `get_custom_mode()` | Member function. |
| `void` | `set_home_position(double lat, double lon, float alt_msl)` | seeded into set_vehicle_state(). |
| `every` | `tick(10ms)` | Member function. |
| `void` | `set_last_known_vehicle_state(const apx::modes::VehicleState& state)` | command reacting to state that's up to one tick stale is fine here. |
| `apx::modes::VehicleState` | `get_last_known_vehicle_state()` | Member function. |
| `void` | `set_motor_outputs(const float outputs[8])` | of that only being visible (or not) inside the simulator. |
| `void` | `set_servo_outputs(const uint16_t pwm[16])` | Member function. |
| `void` | `set_aux_servo_pwm(uint8_t channel_1_based, uint16_t pwm)` | Member function. |
| `uint16_t` | `get_aux_servo_pwm(uint8_t channel_1_based)` | Member function. |
| `RcChannels` | `get_rc_channels()` | Member function. |
| `bool` | `has_active_gcs()` | GCS connection status helpers |
| `size_t` | `get_active_gcs_count()` | Member function. |
| `std::string` | `get_active_gcs_summary()` | Member function. |
| `std::vector<GcsEndpointInfo>` | `get_active_gcs_endpoints()` | Member function. |

---

## VehicleTelemetryState
**Kind:** `struct`  
**Namespace:** `apx::gcs::apx::gcs::apx::gcs`  
**Header:** `modules/gcs/mavlink_handler.hpp`  

---

## param_union
**Kind:** `struct`  
**Namespace:** `mavlink`  
**Header:** `modules/gcs/mavlink/mavlink_types.h`  

---

## param_union_extended
**Kind:** `struct`  
**Namespace:** `mavlink`  
**Header:** `modules/gcs/mavlink/mavlink_types.h`  

---
