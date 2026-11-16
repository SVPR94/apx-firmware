# APX Flight Controller: Multi-Algorithmic Parallel Estimation Architecture

## 1. Multi-Algorithmic Parallel Estimation

The APX flight controller departs from traditional single-algorithm or homogenous lane approaches by employing a heterogeneous multi-algorithmic parallel estimation strategy. Three distinct estimators run concurrently, each exploiting different mathematical properties to guarantee robustness across all flight envelopes.

### 1.1 EKF3: Error-State Extended Kalman Filter (24-State)
A traditional but highly tuned 24-state error-state EKF.
- **States:** Attitude errors (3), Velocity (3), Position (3), Gyro bias (3), Accel bias (3), Earth Magnetic Field (3), Body Magnetic Field (3), Wind Velocity (2), Terrain Offset (1).
- **Mathematical Basis:** Linearization of the non-linear process and measurement models via Jacobians.
- **Strengths:** Computationally efficient, well-understood covariance propagation, handles standard flight dynamics well.
- **Weaknesses:** Vulnerable to linearization errors during highly dynamic maneuvers or large initial condition errors.

### 1.2 UKF: Unscented Kalman Filter (15-State)
A 15-state UKF utilizing a scaled unscented transform.
- **States:** Attitude (quaternion) (4), Velocity (3), Position (3), Gyro bias (3), Accel bias (3).
- **Mathematical Basis:** Propagates a deterministic set of sigma points through the non-linear equations, capturing posterior mean and covariance accurately to the 3rd order (Taylor series expansion) for Gaussian inputs.
- **Formulation:**
  - Sigma points: $\mathcal{X}_i = \bar{x} \pm \sqrt{(n+\lambda)P_x}$
  - Avoids Jacobian linearization, providing superior performance during high-rate turns and aggressive aerobatics.
- **Strengths:** Robust to high nonlinearities, no Jacobian computation required.
- **Weaknesses:** Higher computational burden than EKF.

### 1.3 Fast Geometric Invariant / Complementary Observer
A computationally lightweight, non-linear observer based on invariant EKF and Lyapunov stability principles.
- **States:** Attitude (SO(3)), Velocity, Position.
- **Mathematical Basis:** Exploits the geometric symmetries (Lie group structure) of the navigation equations.
  - $\dot{\hat{R}} = \hat{R}(\Omega - \hat{b}_g - k_P \omega_e)^\times$
  - Ensures global exponential convergence for attitude tracking.
- **Strengths:** Extremely fast, guaranteed stability bounds, unaffected by standard EKF divergence mechanisms. Provides a robust fallback during catastrophic sensor faults or EKF/UKF divergence.
- **Weaknesses:** Suboptimal noise rejection compared to EKF/UKF.

## 2. Algorithm Master & Estimator Arbiter

The Arbiter dynamically evaluates and ranks each estimator's health and consistency.

### 2.1 Decision-Making Metrics
- **Normalized Innovation Squared (NIS):** Measures the Mahalanobis distance of the measurement residual.
  - $\epsilon_\nu = \nu^T S^{-1} \nu$
  - Checked against a $\chi^2$ distribution threshold.
- **Covariance Trace ($Tr(P)$):** Measures the filter's self-reported uncertainty.
- **Residual Drift Rate:** Tracks the moving average of residuals to detect slow drift (e.g., GPS multipath).
- **Divergence Detection:** Identifies when state estimates violate physical bounds or kinematically impossible accelerations.

### 2.2 Dynamic Lane Ranking and Health State Machine
Each estimator is classified into a dynamic health state:
- **OPTIMAL:** NIS within bounds, stable $Tr(P)$, no divergence.
- **ACCEPTABLE:** Occasional NIS spikes, slightly elevated $Tr(P)$.
- **DEGRADED:** High NIS, growing $Tr(P)$, or missing minor sensor updates.
- **DIVERGING:** Continuous NIS rejection, kinematic inconsistencies.
- **FAULTED:** Math errors (NaN), missed deadlines, severe sensor contradictions.

## 3. Federated Multi-Algorithm Consensus Fusion

Unlike legacy autopilots that use a discrete lane-switching mechanism (often causing control shocks), APX employs an Information-Weighted / Covariance-Intersection consensus blend.

### 3.1 Covariance Intersection Blending
The fused state $x_{fused}$ and covariance $P_{fused}$ are computed dynamically:
$$ P_{fused}^{-1} = \sum_{i=1}^N \omega_i P_i^{-1} $$
$$ P_{fused}^{-1} x_{fused} = \sum_{i=1}^N \omega_i P_i^{-1} x_i $$

### 3.2 Weighting Strategy
The weights $\omega_i$ are determined dynamically based on the estimator's health state, NIS consistency, and covariance trace.
$$ \omega_i = f(NIS_i, Tr(P_i), HealthState_i) $$
- **Result:** This ensures a continuous, shock-free transition. As an estimator degrades, its weight smoothly approaches zero, isolating it from the fused output without step changes in the control loop.

## 4. Dynamic Sensor Crossbar / Live Sensor Rewiring

APX features a live flight dynamic sensor routing matrix, abstracting physical sensors from the estimators.

### 4.1 Sensor Health Monitoring
- **Vibration Spikes:** High-frequency noise metrics.
- **Clipping:** Accelerometer/Gyro saturation detection.
- **Thermal Drift:** Temperature-correlated bias tracking.
- **Magnetic Anomalies:** Field strength and inclination variance checks.

### 4.2 Dynamic Routing
Instead of hardcoding IMU1 to Lane 1, the crossbar dynamically maps the best physical sensors to the optimal estimator.
- **Matrix Mapping:** $M_{routing} : \{Sensors_{physical}\} \rightarrow \{Estimators\}$
- **Live Rewiring:** If IMU1 clips during a maneuver, the crossbar instantly routes IMU2's data to EKF3 without halting the filter. The covariance matrices are updated with the new sensor noise characteristics ($R$ matrix injection), maintaining filter continuity.

## 5. STM32H7 Embedded Feasibility & Resource Budget

The architecture is designed for the STM32H7 Cortex-M7 running at 480MHz.

### 5.1 Memory Footprint (Zero Malloc)
- All matrices and state arrays are statically allocated.
- EKF3 (24-state): ~15KB RAM (State, Covariance, Jacobians).
- UKF (15-state): ~25KB RAM (Sigma points, weights).
- Invariant Observer: ~2KB RAM.
- **Total Static RAM Budget:** < 50KB, well within TCM (Tightly Coupled Memory) limits for deterministic access.

### 5.2 FPU Requirements & Cycle Count
- Cortex-M7 features a double-precision hardware FPU (DP-FPU). All critical matrix operations (Cholesky decomposition, matrix inversion) use optimized DSP libraries (ARM CMSIS-DSP).
- **EKF3 Update:** ~4,000 cycles.
- **UKF Update:** ~15,000 cycles.
- **Invariant Observer:** ~500 cycles.
- **Arbiter & Consensus:** ~2,000 cycles.

### 5.3 Timing Budget (400Hz IMU / 100Hz Fusion)
- At 480MHz, 1ms (1000Hz) = 480,000 cycles.
- A 400Hz loop gives 1.2M cycles per frame.
- Total worst-case execution time (WCET) for the full architecture: ~25,000 cycles.
- **CPU Utilization:** ~2% for the entire estimation suite, leaving ample headroom for control, path planning, and IO processing.
