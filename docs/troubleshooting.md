# APX Troubleshooting Guide

This guide covers common build, configuration, simulation, and flight issues encountered in APX, along with step-by-step diagnostic procedures.

---

## 1. Build and Toolchain Issues

### 1.1 "ARM cross-compiler (arm-none-eabi-gcc) not found"
- **Cause:** CMake cannot locate the ARM Cortex-M7 cross compiler.
- **Resolution:**
  - Run the one-time setup: `tools/setup_ubuntu_env.sh` (or `--no-apt` without sudo), then open a new terminal.
  - Delete the failed build directory before configuring again: `rm -rf build/stm32h7 && cmake --preset stm32h7`.
  - To use your own toolchain instead, put its `bin/` on `PATH` or set `APX_ARM_TOOLCHAIN_BIN` to it.

### 1.2 "Generator: Ninja does not match generator used previously"
- **Cause:** Switching between `Unix Makefiles` and `Ninja` in an existing build directory.
- **Resolution:** Clean the build directory before re-configuring:
  ```bash
  rm -rf build/<preset> && cmake --preset <preset>
  ```

---

## 2. SITL Simulation Issues

### 2.1 "Could not bind SITL port 9003"
- **Cause:** A previous instance of `apx_sitl` or Gazebo is still running and occupying the UDP socket.
- **Resolution:** Terminate running simulation instances:
  ```bash
  pkill -9 apx_sitl || true
  pkill -9 -f "gz sim" || true
  ```

### 2.2 Gazebo Plugin Fails to Load (`apx_gazebo_plugin.so`)
- **Cause:** Gazebo cannot locate the shared library or GZ plugin path is unset.
- **Resolution:** Ensure `GZ_SIM_SYSTEM_PLUGIN_PATH` includes the build directory:
  ```bash
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/sitl/sitl/gazebo_plugin:$GZ_SIM_SYSTEM_PLUGIN_PATH
  ```

---

## 3. MAVLink & Ground Station Issues

### 3.1 APX Flight Manager Shows "Guided Not Supported" or Flight Modes Are Blank
- **Cause:** Heartbeat packets were sent with generic autopilot ID instead of multicopter dialect identifier.
- **Resolution:** APX broadcasts standard custom autopilot dialect identification in `mavlink_handler.cpp:send_heartbeat()`. This enables standard GCS and Ground Control Station mode tables (`0=STABILIZE`, `3=AUTO`, `4=GUIDED`, `17=BRAKE`).

### 3.2 Vehicle Refuses to Enter Guided or Auto Mode
- **Cause:** Missing 3D GPS fix or home position has not been acquired.
- **Resolution:**
  - In SITL, wait until the GPS simulator acquires satellites (`satellites >= 10`, `hdop < 1.5`).
  - Verify that home coordinates have been set via `mavlink.set_home_position(lat, lon, alt)`.

---

## 4. Flight Control & Actuator Issues

### 4.1 Sprayer Stays Stopped / Servo 9 Stays at 1100 PWM
- **Cause:** Disarm interlock is active, or `DO_SET_SERVO` target channel does not match configured pump channel.
- **Resolution:**
  - Verify parameter `SPRAY_PWM_MAX` is set to `1900` and `SPRAY_PWM_MIN` to `1100`.
  - Check that the vehicle is armed before spraying in manual mode. Disarming automatically drives the pump to 1100 us for operator safety.
  - In autonomous missions, verify the mission command is `MAV_CMD_DO_SET_SERVO` with param1 = `9` and param2 = `1900`.

### 4.2 Aggressive Yaw Kick During Waypoint Transitions
- **Cause:** Large yaw setpoint steps stealing motor headroom from pitch/roll stabilization.
- **Resolution:**
  - The APX motor mixer enforces `yaw_headroom = max(0.15, 1.0 - rp_spread)`, prioritizing attitude stability over yaw speed.
  - S-curve cornering in `path_tracker.hpp` computes cornering radius to stay within lateral acceleration bounds.
