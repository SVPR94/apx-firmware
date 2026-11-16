# APX C++ API Documentation Index

Welcome to the formal C++ API reference for the APX Flight Controller codebase.
This API reference is extracted directly from the C++17 source and header files,
indexed by architectural layer and namespace.

## Subsystem Reference Modules

| Layer / Subsystem | Primary Namespace | API Reference Link | Exported Symbols |
|---|---|---|:---:|
| **Flight Control** | `apx::control` | [Flight Control](control.md) | 7 |
| **State Estimation** | `apx::estimator` | [State Estimation](estimator.md) | 35 |
| **Flight Modes** | `apx::modes` | [Flight Modes](modes.md) | 34 |
| **GCS & MAVLink** | `apx::gcs` | [GCS & MAVLink](gcs.md) | 5 |
| **Hardware Abstraction Layer** | `apx::hal` | [Hardware Abstraction Layer](hal.md) | 13 |
| **Kernel Subsystem** | `apx::kernel` | [Kernel Subsystem](kernel.md) | 4 |
| **Device Drivers** | `apx::drivers` | [Device Drivers](drivers.md) | 48 |
| **Platform Support** | `apx::platform` | [Platform Support](platform.md) | 6 |

**Total Documented C++ Symbols:** `152`

## Standards & Conventions
- **C++ Standard:** C++17 (`std::array`, `std::atomic`, `constexpr`, `std::string_view`)
- **Memory Policy:** Zero heap allocation in flight loops; static sizing or stack-allocated frames
- **Concurrency:** Lock-free pub/sub bus (`apx::kernel::Topic`) and mutex-guarded GCS streams
- **Platform Portability:** Pure virtual interfaces in `apx::hal` implemented by POSIX, SITL, and bare-metal STM32H7
