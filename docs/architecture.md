# APX Flight Controller Architecture

This document details the software architecture, execution flow, data pathways, and component interaction models of the APX autopilot platform.

---

## 1. System Overview

APX is an aerospace-grade, multi-platform autopilot written in modern C++17. The system is designed with strict separation between platform hardware drivers, estimation, vehicle guidance modes, inner-loop flight control, and ground communication.

```mermaid
flowchart TD
    subgraph Hardware_Or_SITL["Hardware & Simulation Layer"]
        Sensors["Sensors (IMU, Baro, Mag, GPS, Batt)"]
        Actuators["Motors, Servos & Spray Relays"]
        HAL["Hardware Abstraction Layer (apx::hal)"]
    end

    subgraph Core_Kernel["APX Microkernel & Bus"]
        uBus["Lock-Free Publish/Subscribe (uBus)"]
        Params["Persistent Parameter Manager"]
    end

    subgraph Estimation["State Estimation Engine"]
        SensMgr["Sensor Manager & Health Voter"]
        Prefilter["Dynamic Notch & LPF Prefilters"]
        EKF["Multi-Estimator Suite (EKF3, Mahony, UKF)"]
    end

    subgraph Guidance_Executive["Guidance & Executive"]
        ModeMgr["Flight Mode Manager"]
        Modes["Active Mode (Stabilize, PosHold, Auto, Brake, RTL, Land)"]
        MissionNav["Mission Navigator & Path Tracker"]
    end

    subgraph Flight_Control["Flight Control Inner Loops"]
        PosCtrl["Position & Altitude Controller"]
        AttCtrl["Quaternion Swing-Twist Attitude Controller"]
        RateCtrl["Multi-Axis Rate PID with D-Term LPF"]
        Mixer["Multirotor Geometry Mixer (Desaturation & Yaw Priority)"]
    end

    subgraph Communications["GCS & Telemetry"]
        MAVLink["MAVLink Telemetry Protocol Handler"]
        CmdHandler["MAVLink Command Dispatcher"]
        GCS["Ground Control Stations (GCS, Ground Control Station)"]
    end

    Sensors --> HAL
    HAL --> SensMgr
    SensMgr --> Prefilter
    Prefilter --> EKF
    EKF --> uBus
    uBus --> ModeMgr
    ModeMgr --> Modes
    Modes --> MissionNav
    MissionNav --> PosCtrl
    PosCtrl --> AttCtrl
    AttCtrl --> RateCtrl
    RateCtrl --> Mixer
    Mixer --> HAL
    HAL --> Actuators

    GCS <--> MAVLink
    MAVLink --> CmdHandler
    CmdHandler --> ModeMgr
    CmdHandler --> Params
    uBus --> MAVLink
```

---

## 2. Flight Control Cascade & Dynamics

The flight control cascade follows a four-tier nested feedback loop:

1. **Position & Velocity Loop:** Converts 3D position targets into horizontal velocity and vertical climb rates. Velocity demands are capped by `m_max_speed_ms` (`WPNAV_SPEED`) and converted into desired body tilt vectors.
2. **Attitude Control Loop (Swing-Twist Quaternion):** Decouples thrust-vector tilt (swing) from heading alignment (twist). This prevents large simultaneous yaw disturbances from destabilizing pitch/roll.
3. **Rate Control Loop:** Runs at 400 Hz. Computes torque setpoints using differentiated measurement derivatives \( \frac{d\omega}{dt} \) and dynamic low-pass filtering.
4. **Actuator Mixer:** Scales roll/pitch first, allocates remaining headroom to yaw authority, and performs whole-vector desaturation without asymmetric motor clipping.

```mermaid
flowchart LR
    PosErr["Target Position (NED)"] --> PosP["Position P Controller"]
    PosP --> VelCap["Velocity Cap (WPNAV_SPEED)"]
    VelCap --> VelPID["Velocity PI Controller"]
    VelPID --> AttTarget["Desired Tilt & Yaw Angle"]
    AttTarget --> SwingTwist["Swing-Twist Quaternion Decoupling"]
    SwingTwist --> RateCmd["Commanded Angular Rates"]
    RateCmd --> RateSlew["Rate Slew Limiter (ATC_ACCEL)"]
    RateSlew --> RatePID["Rate PID Controller (400 Hz)"]
    RatePID --> Torques["Roll, Pitch, Yaw Torques"]
    Torques --> Mixer["Multirotor Matrix Mixer"]
    Mixer --> PWM["Motor PWM / DShot"]
```

---

## 3. Flight Mode State Machine

The `ModeManager` validates pre-arm requirements, handles pilot mode transitions, and executes mode lifecycle callbacks (`on_enter`, `update`, `on_exit`).

```mermaid
stateDiagram-v2
    [*] --> BOOT: Power On
    BOOT --> DISARMED: Initialization Complete
    
    DISARMED --> STABILIZE: Pilot Arm Gesture / GCS Arm
    DISARMED --> ALT_HOLD: Arming (if Altitude Healthy)
    DISARMED --> POS_HOLD: Arming (if 3D Fix & Home Set)
    
    STABILIZE --> ALT_HOLD: Mode Switch
    ALT_HOLD --> POS_HOLD: Mode Switch (GPS Fix)
    POS_HOLD --> GUIDED: GCS Takeoff / Waypoint
    POS_HOLD --> AUTO: Mission Start Command
    
    GUIDED --> BRAKE: RC Pause / GCS Stop
    AUTO --> BRAKE: Emergency Pause
    BRAKE --> POS_HOLD: Pilot Stick Override
    
    AUTO --> RTL: Mission Complete / Failsafe Trigger
    POS_HOLD --> RTL: Pilot RTL Switch
    RTL --> LAND: Reached Home Loiter Alt
    LAND --> DISARMED: Land Detector Confirmed Touchdown
```

---

## 4. Multi-Estimator Fusion Architecture

APX incorporates a parallel estimator pipeline where multiple estimation algorithms execute simultaneously:

- **EKF3 (24-State Error-State Kalman Filter):** Full 3D navigation solution fusing IMU, GPS (including RTK), Barometer, and Magnetometer with online sensor bias learning.
- **Fast Geometric Observer (Mahony Filter):** Lightweight complementary filter providing backup attitude estimation when GPS or magnetometer degraded.
- **Unscented Kalman Filter (UKF):** High-dynamic turn tracker preventing Jacobian linearization singularity.
- **Sensor Voter & Health Arbiter:** Dynamically ranks sensor lanes (IMU1..4, Baro1..4) based on variance and innovation tests, switching streams seamlessly.

```mermaid
flowchart TD
    subgraph Sensors["Redundant Sensor Inputs"]
        IMU1["IMU 1 (ICM-42688)"]
        IMU2["IMU 2 (BMI088)"]
        Baro1["Baro 1 (MS5611)"]
        Baro2["Baro 2 (BMP388)"]
        GPS1["GPS 1 (u-blox F9P)"]
        Mag1["Mag 1 (IST8310)"]
    end

    subgraph Arbitration["Arbiter & Pre-filtering"]
        Voter["Sensor Voter (Pairwise Outlier Rejection)"]
        Harmonic["Harmonic Notch (ESC RPM Tracking)"]
        LPF["Butterworth Low-Pass Filters"]
    end

    subgraph Estimators["Parallel Estimator Lanes"]
        EKF3["Lane 1: 24-State EKF3"]
        Mahony["Lane 2: Geometric Mahony Filter"]
        UKF["Lane 3: 15-State UKF Observer"]
    end

    subgraph Fusion["Consensus & Health"]
        Consensus["Covariance Intersection Consensus Arbiter"]
        NavState["Fused Navigation State (Position, Velocity, Attitude)"]
    end

    IMU1 & IMU2 & Baro1 & Baro2 & GPS1 & Mag1 --> Voter
    Voter --> Harmonic --> LPF
    LPF --> EKF3 & Mahony & UKF
    EKF3 & Mahony & UKF --> Consensus
    Consensus --> NavState
```

---

## 5. MAVLink Telemetry and Command Dispatch

Ground Station communication is completely asynchronous:

- Incoming MAVLink packets arrive over UART or UDP sockets.
- The `MavlinkHandler` decodes packets, updates parameter registers, and delegates mission items to `MissionExecutor`.
- Heartbeat packets broadcast standard custom mode identifiers (`0=STABILIZE`, `3=AUTO`, `4=GUIDED`, `17=BRAKE`, etc.) with standard custom dialect identification for universal GCS and Ground Control Station support.
- Actuator telemetry (including motor outputs and spray pump PWM on servo channel 9) is streamed in real time.
