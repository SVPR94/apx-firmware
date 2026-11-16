# Getting Started

Welcome to APX! This guide gets you flying quickly.

## Hardware Requirements
- A supported Flight Controller (e.g., APX H7 v1)
- Multirotor frame, ESCs, motors, and props
- GPS/Compass module
- RC Receiver
- LiPo Battery

## Software Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/myflight/apx.git
   cd apx
   ```
2. Build for your board:
   ```bash
   mkdir build && cd build
   cmake .. -DBOARD=apx_h7_v1
   make -j$(nproc)
   ```
3. Flash the firmware via USB or DFU.

## First Flight Checklist
1. **Calibration**: Complete Accel, Gyro, and Compass calibration in APX Flight Manager.
2. **RC Setup**: Bind your receiver and verify channel mapping in GCS.
3. **Motor Test**: Remove props. Use the GCS motor test tab to ensure motors spin in the correct direction.
4. **Arming Check**: Verify there are no pre-arm errors.
5. **Maiden**: Go to a safe, open field. Attach props, arm in STABILIZE mode, and carefully increase throttle to verify control.
