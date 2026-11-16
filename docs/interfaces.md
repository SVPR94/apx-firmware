# APX Core Interfaces

This document describes the primary C++ interfaces used for modularity, abstraction, and communication throughout the APX codebase.

---

## 1. Hardware Abstraction Layer Interfaces (`apx::hal`)

Located in `hal/include/apx/hal/`. These interfaces decouple flight software from physical silicon or simulation environments.

### 1.1 `IHalUart`
Serial interface for GPS, telemetry, and RC receivers.
```cpp
namespace apx::hal {
class IHalUart {
public:
    virtual ~IHalUart() = default;
    virtual bool init(uint32_t baud_rate) = 0;
    virtual void deinit() = 0;
    virtual int write(const uint8_t* buffer, size_t length) = 0;
    virtual int read(uint8_t* buffer, size_t max_length) = 0;
    virtual int available() = 0;
    virtual void flush() = 0;
    virtual void set_rx_callback(std::function<void(const uint8_t*, size_t)> callback) = 0;
};
}
```

### 1.2 `IHalSpi`
Synchronous serial interface for IMU, Barometer, and external Flash.
```cpp
namespace apx::hal {
class IHalSpi {
public:
    virtual ~IHalSpi() = default;
    virtual bool init(uint32_t clock_hz, uint8_t mode = 3, uint8_t bits = 8) = 0;
    virtual bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) = 0;
    virtual void cs_select(uint32_t device_id) = 0;
    virtual void cs_deselect(uint32_t device_id) = 0;
};
}
```

### 1.3 `IHalPwm`
Timer-driven PWM generation for motors, servos, and pumps.
```cpp
namespace apx::hal {
class IHalPwm {
public:
    virtual ~IHalPwm() = default;
    virtual bool init(uint8_t channel, uint32_t freq_hz) = 0;
    virtual void set_duty(uint8_t channel, float duty_cycle_0_to_1) = 0;
    virtual void set_pulse_width_us(uint8_t channel, uint16_t pulse_us) = 0;
    virtual void enable(uint8_t channel) = 0;
    virtual void disable(uint8_t channel) = 0;
};
}
```

---

## 2. Driver Interface (`apx::drivers::DriverBase`)
 
Located in `drivers/include/apx/drivers/driver_base.hpp`. Base class for all physical sensor and actuator drivers.
 
```cpp
namespace apx::drivers {
enum class DriverStatus { UNINITIALIZED, INITIALIZING, RUNNING, DEGRADED, FAILED, DISABLED };

class DriverBase {
public:
    explicit DriverBase(std::string_view name);
    virtual ~DriverBase() = default;
    virtual bool init() = 0;
    virtual void deinit() {}
    virtual void update() {}
    virtual bool is_healthy() const;
    DriverStatus get_status() const;
    std::string_view get_name() const;
};
}
```

---

## 3. Flight Mode Interface (`apx::modes::FlightMode`)

Located in `modules/modes/mode_base.hpp`. Base class for all guidance and pilot modes.

```cpp
namespace apx::modes {
enum class FlightModeId : uint8_t {
    MANUAL = 0,
    STABILIZE = 1,
    ALT_HOLD = 2,
    POS_HOLD = 3,
    GUIDED = 4,
    AUTO = 5,
    RTL = 6,
    LAND = 7,
    BRAKE = 8,
    LOITER = 9
};

class FlightMode {
public:
    virtual ~FlightMode() = default;
    virtual void on_enter(const VehicleState& state) = 0;
    virtual ControlOutput update(const VehicleState& state) = 0;
    virtual void on_exit() = 0;
    virtual FlightModeId get_id() const = 0;
    virtual std::string_view get_name() const = 0;
    virtual bool can_arm() const { return true; }
    virtual bool is_gps_required() const { return false; }
};
}
```

---

## 4. Microkernel Message Bus (`apx::kernel::Topic`)

Located in `kernel/include/apx/kernel/ubus.hpp`. High-performance lock-free ring buffer connecting producers to multiple consumer threads without dynamic allocation.

```cpp
namespace apx::kernel {
template<typename T, size_t BufferSize = 16>
class Topic {
public:
    void publish(const T& sample);
    bool read(Subscription<T>& sub, T& output);
    bool read_latest(T& output);
};
}
```
