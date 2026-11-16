# APX Flight Controller Documentation Portal

Welcome to the technical documentation repository for the **APX Flight Controller**.

APX is an open-source, modular, aerospace-grade flight control firmware and simulation platform written in modern C++17. It supports autonomous multirotor operations, agricultural spraying payloads, multi-estimator state fusion, and bare-metal STM32H7 hardware execution.

---

## Documentation Index

### 1. Architecture & Design
- **[System Architecture](architecture.md):** Complete architectural overview, control cascade, multi-estimator design, and Mermaid workflow diagrams.
- **[Subsystem & Module Catalog](modules.md):** In-depth documentation of flight control, estimation, guidance, GCS, drivers, and safety modules.
- **[Core Interfaces](interfaces.md):** Pure virtual HAL interfaces (`IHalUart`, `IHalSpi`, `IHalPwm`), DriverBase, FlightMode, and uBus pub/sub topics.
- **[Codebase Structure & Conventions](code-structure.md):** Directory layout, namespace mapping, memory policies, and design conventions.

### 2. Development, Build & Hardware
- **[Build System & Toolchain Guide](build.md):** Build presets (`sitl`, `linux-release`, `stm32h7`), cross-compiling, and dependencies.
- **[Board Management & Hardware Definitions](board-management.md):** Defining and generating custom STM32 boards from `hwdef.dat` in one command.
- **[Testing & Quality Assurance Guide](testing.md):** Unified unit test suite (GoogleTest) and automated SITL flight verification scenarios.
- **[Troubleshooting Guide](troubleshooting.md):** Diagnosing common build, simulation, toolchain, and MAVLink telemetry issues.

### 3. Project Evolution & API Reference
- **[C++ API Reference](api/index.md):** Generated C++ API documentation across all subsystems, classes, structs, enums, and methods.
- **[Git History & Architectural Milestones](git-history.md):** Chronological log of major architectural changes and bug fixes.
- **[AI Project Context & Primer](project-context.md):** Compact, high-density reference optimized for AI coding assistants.

---

## Quick Start Commands

```bash
# 1. Build and Run Unit Tests
cmake --preset sitl && cmake --build build/sitl -j$(nproc)
./build/sitl/tests/unit/run_unit_tests

# 2. Build STM32H7 Firmware
cmake --preset stm32h7 && cmake --build build/stm32h7 -j$(nproc)

# 3. Generate Local Documentation Site
./scripts/generate-docs.sh
```
