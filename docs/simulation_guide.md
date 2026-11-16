# Simulation Guide (SITL)

Software In The Loop (SITL) allows you to run APX on your development machine and connect it to a physics simulator like Gazebo.

## Prerequisites

- Ubuntu 22.04 or later
- CMake 3.20+
- Gazebo Garden or Classic
- APX Flight Manager (GCS)

```bash
sudo apt update
sudo apt install cmake build-essential gz-garden
```

## Build SITL

Configure the build for the native Linux target:

```bash
mkdir build && cd build
cmake .. -DPRESET=sitl
make -j$(nproc)
```

## Running Simulation

1. Launch Gazebo with the drone world:
   ```bash
   gz sim worlds/quadcopter.sdf
   ```
2. Run the APX executable:
   ```bash
   ./apx_sitl
   ```

Simulated sensor lanes, GNSS receivers, sensor error profile, wind and
the sensor CSV log are set with `sitl/sim_vehicle.py` options; see
[Simulated Sensors and Wind](simulated_sensors.md).

## Connecting APX Flight Manager

APX listens on UDP port 14550 for GCS connections by default. Open APX Flight Manager, and it should automatically connect to the simulated drone. 

## TCP vs UDP Mode

For MAVLink communication:
- **UDP** (Default): Best for local simulation and real radio links. APX broadcasts on `127.0.0.1:14550`.
- **TCP**: Useful for debugging with step-by-step debuggers. Pass `--tcp 5760` to `apx_sitl`.

## Running Tests

Unit tests are built alongside the SITL executable.
```bash
make test
# or
ctest --output-on-failure
```
