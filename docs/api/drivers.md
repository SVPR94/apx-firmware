# Device Drivers API Reference

Namespace: `apx::drivers`

## Summary

This page documents all public classes, data structures, and enumerations for `Device Drivers`.

| Symbol | Kind | Declared In | Description |
|---|---|---|---|
| [`AccelRange`](#accelrange) | `enum` | `drivers/imu/icm42688/icm42688.hpp` | Enum definition. |
| [`BaroData`](#barodata) | `struct` | `drivers/baro/ms5611/ms5611.hpp` | Struct definition. |
| [`BatteryData`](#batterydata) | `struct` | `drivers/battery/ina226/ina226.hpp` | Struct definition. |
| [`Bmi088`](#bmi088) | `class` | `drivers/imu/bmi088/bmi088.hpp` | Class definition. |
| [`Bmi270`](#bmi270) | `class` | `drivers/imu/bmi270/bmi270.hpp` | Class definition. |
| [`CanFrame`](#canframe) | `struct` | `drivers/rangefinder/can_radar/can_radar_driver.hpp` | Struct definition. |
| [`CanRadarDriver`](#canradardriver) | `class` | `drivers/rangefinder/can_radar/can_radar_driver.hpp` | Class definition. |
| [`CrsfDriver`](#crsfdriver) | `class` | `drivers/rc_input/crsf/crsf_driver.hpp` | Class definition. |
| [`DriverAutoRegistrar`](#driverautoregistrar) | `struct` | `drivers/include/apx/drivers/driver_registry.hpp` | Auto-registration helper |
| [`DriverBase`](#driverbase) | `class` | `drivers/include/apx/drivers/driver_base.hpp` | Class definition. |
| [`DriverRegistry`](#driverregistry) | `class` | `drivers/include/apx/drivers/driver_registry.hpp` | Class definition. |
| [`DriverStatus`](#driverstatus) | `enum` | `drivers/include/apx/drivers/driver_base.hpp` | Enum definition. |
| [`DshotCmd`](#dshotcmd) | `enum` | `drivers/esc/dshot/dshot_driver.hpp` | Enum definition. |
| [`DshotDriver`](#dshotdriver) | `class` | `drivers/esc/dshot/dshot_driver.hpp` | Class definition. |
| [`DshotSpeed`](#dshotspeed) | `enum` | `drivers/esc/dshot/dshot_driver.hpp` | Enum definition. |
| [`DshotTelemetry`](#dshottelemetry) | `struct` | `drivers/esc/dshot/dshot_driver.hpp` | Struct definition. |
| [`FlowData`](#flowdata) | `struct` | `drivers/flow/pmw3901/pmw3901.hpp` | Struct definition. |
| [`GpsData`](#gpsdata) | `struct` | `drivers/gps/ublox/ublox_driver.hpp` | Struct definition. |
| [`GpsDataRTK`](#gpsdatartk) | `struct` | `drivers/gps/ublox_f9p/ublox_f9p.hpp` | Struct definition. |
| [`GpsFixType`](#gpsfixtype) | `enum` | `drivers/gps/ublox/ublox_driver.hpp` | Enum definition. |
| [`GyroRange`](#gyrorange) | `enum` | `drivers/imu/icm42688/icm42688.hpp` | Enum definition. |
| [`IHalCan`](#ihalcan) | `class` | `drivers/rangefinder/can_radar/can_radar_driver.hpp` | Class definition. |
| [`IHalSpi`](#ihalspi) | `class` | `drivers/flow/pmw3901/pmw3901.hpp` | Class definition. |
| [`Icm20689`](#icm20689) | `class` | `drivers/imu/icm20689/icm20689.hpp` | Class definition. |
| [`Icm42605`](#icm42605) | `class` | `drivers/imu/icm42605/icm42605.hpp` | Class definition. |
| [`Icm42688`](#icm42688) | `class` | `drivers/imu/icm42688/icm42688.hpp` | Class definition. |
| [`ImuData`](#imudata) | `struct` | `drivers/imu/icm42688/icm42688.hpp` | Struct definition. |
| [`ImuOdr`](#imuodr) | `enum` | `drivers/imu/icm42688/icm42688.hpp` | Enum definition. |
| [`Ina226`](#ina226) | `class` | `drivers/battery/ina226/ina226.hpp` | Class definition. |
| [`Ist8310`](#ist8310) | `class` | `drivers/mag/ist8310/ist8310.hpp` | Class definition. |
| [`Lis3mdl`](#lis3mdl) | `class` | `drivers/mag/lis3mdl/lis3mdl.hpp` | Class definition. |
| [`MagData`](#magdata) | `struct` | `drivers/mag/ist8310/ist8310.hpp` | Struct definition. |
| [`Ms5611`](#ms5611) | `class` | `drivers/baro/ms5611/ms5611.hpp` | Class definition. |
| [`ObstacleData`](#obstacledata) | `struct` | `drivers/rangefinder/can_radar/can_radar_driver.hpp` | Struct definition. |
| [`ParseState`](#parsestate) | `enum` | `drivers/gps/ublox/ublox_driver.hpp` | Enum definition. |
| [`Pmw3901Driver`](#pmw3901driver) | `class` | `drivers/flow/pmw3901/pmw3901.hpp` | Class definition. |
| [`PumpDriver`](#pumpdriver) | `class` | `drivers/pump/pump_driver.hpp` | Class definition. |
| [`RcData`](#rcdata) | `struct` | `drivers/rc_input/sbus/sbus_driver.hpp` | Struct definition. |
| [`RelayDriver`](#relaydriver) | `class` | `drivers/relay/relay_driver.hpp` | Class definition. |
| [`Rm3100`](#rm3100) | `class` | `drivers/mag/rm3100/rm3100.hpp` | Class definition. |
| [`SbusDriver`](#sbusdriver) | `class` | `drivers/rc_input/sbus/sbus_driver.hpp` | Class definition. |
| [`ServoDriver`](#servodriver) | `class` | `drivers/servo/servo_driver.hpp` | Class definition. |
| [`State`](#state) | `enum` | `drivers/baro/ms5611/ms5611.hpp` | Enum definition. |
| [`UartMavlinkDriver`](#uartmavlinkdriver) | `class` | `drivers/telemetry/uart_mavlink/uart_mavlink_driver.hpp` | Class definition. |
| [`UbloxDriver`](#ubloxdriver) | `class` | `drivers/gps/ublox/ublox_driver.hpp` | Class definition. |
| [`UbloxF9P`](#ubloxf9p) | `class` | `drivers/gps/ublox_f9p/ublox_f9p.hpp` | Class definition. |

---

## AccelRange
**Kind:** `enum`  
**Namespace:** `apx::drivers::Icm42688Reg`  
**Header:** `drivers/imu/icm42688/icm42688.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `G2` | Enumerator value |
| `G4` | Enumerator value |
| `G8` | Enumerator value |
| `G16` | Enumerator value |

---

## BaroData
**Kind:** `struct`  
**Namespace:** `apx::drivers::Ms5611Cmd`  
**Header:** `drivers/baro/ms5611/ms5611.hpp`  

---

## BatteryData
**Kind:** `struct`  
**Namespace:** `apx::drivers::Ina226Reg`  
**Header:** `drivers/battery/ina226/ina226.hpp`  

---

## Bmi088
**Kind:** `class`  
**Namespace:** `apx::drivers::Bmi088AccReg::Bmi088GyrReg`  
**Header:** `drivers/imu/bmi088/bmi088.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Bmi088(apx::hal::IHalSpi& spi, uint8_t cs_acc, uint8_t cs_gyr)` | Two separate SPI CS lines: one for accel, one for gyro |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## Bmi270
**Kind:** `class`  
**Namespace:** `apx::drivers::Bmi270Reg`  
**Header:** `drivers/imu/bmi270/bmi270.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Bmi270(apx::hal::IHalSpi& spi, uint8_t cs_id)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## CanFrame
**Kind:** `struct`  
**Namespace:** `apx::hal::apx::drivers`  
**Header:** `drivers/rangefinder/can_radar/can_radar_driver.hpp`  

---

## CanRadarDriver
**Kind:** `class`  
**Namespace:** `apx::hal::apx::drivers`  
**Header:** `drivers/rangefinder/can_radar/can_radar_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `CanRadarDriver(apx::hal::IHalCan& can, uint32_t base_can_id = 0x200)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `ObstacleData` | `get_data()` | Member function. |

---

## CrsfDriver
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/rc_input/crsf/crsf_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `CrsfDriver(apx::hal::IHalUart& uart)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |

---

## DriverAutoRegistrar
**Kind:** `struct`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/include/apx/drivers/driver_registry.hpp`  

> Auto-registration helper

---

## DriverBase
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/include/apx/drivers/driver_base.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `bool` | `init()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## DriverRegistry
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/include/apx/drivers/driver_registry.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `move(factory)` | Member function. |
| `` | `find(name)` | Member function. |
| `` | `second()` | Member function. |
| `` | `push_back(name)` | Member function. |

---

## DriverStatus
**Kind:** `enum`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/include/apx/drivers/driver_base.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `UNINITIALIZED` | Enumerator value |
| `INITIALIZING` | Enumerator value |
| `RUNNING` | Enumerator value |
| `DEGRADED` | Enumerator value |
| `FAILED` | Enumerator value |
| `DISABLED` | Enumerator value |

---

## DshotCmd
**Kind:** `enum`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/esc/dshot/dshot_driver.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `MOTOR_STOP` | Enumerator value |
| `BEEP1` | Enumerator value |
| `BEEP2` | Enumerator value |
| `BEEP3` | Enumerator value |
| `BEEP4` | Enumerator value |
| `BEEP5` | Enumerator value |
| `ESC_INFO` | Enumerator value |
| `SPIN_DIR_1` | Enumerator value |
| `SPIN_DIR_2` | Enumerator value |
| `SAVE_SETTINGS` | Enumerator value |
| `SPIN_DIR_NORMAL` | Enumerator value |
| `SPIN_DIR_REVERSED` | Enumerator value |

---

## DshotDriver
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/esc/dshot/dshot_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `DshotDriver(DshotSpeed speed = DshotSpeed::DSHOT600)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `void` | `set_throttle(uint8_t channel, uint16_t value_0_2047)` | Member function. |
| `void` | `send_command(uint8_t channel, DshotCmd cmd, bool telemetry_req = false)` | Member function. |
| `void` | `arm_all()` | Member function. |

---

## DshotSpeed
**Kind:** `enum`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/esc/dshot/dshot_driver.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `DSHOT150` | Enumerator value |
| `DSHOT300` | Enumerator value |
| `DSHOT600` | Enumerator value |
| `DSHOT1200` | Enumerator value |

---

## DshotTelemetry
**Kind:** `struct`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/esc/dshot/dshot_driver.hpp`  

---

## FlowData
**Kind:** `struct`  
**Namespace:** `apx::hal::apx::drivers`  
**Header:** `drivers/flow/pmw3901/pmw3901.hpp`  

---

## GpsData
**Kind:** `struct`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/gps/ublox/ublox_driver.hpp`  

---

## GpsDataRTK
**Kind:** `struct`  
**Namespace:** `apx::drivers::F9PKey`  
**Header:** `drivers/gps/ublox_f9p/ublox_f9p.hpp`  
**Inherits from:** `GpsData`  

---

## GpsFixType
**Kind:** `enum`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/gps/ublox/ublox_driver.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `NO_FIX` | Enumerator value |
| `DEAD_RECK` | Enumerator value |
| `FIX_2D` | Enumerator value |
| `FIX_3D` | Enumerator value |
| `GNSS_DEAD_RECK` | Enumerator value |

---

## GyroRange
**Kind:** `enum`  
**Namespace:** `apx::drivers::Icm42688Reg`  
**Header:** `drivers/imu/icm42688/icm42688.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `DPS250` | Enumerator value |
| `DPS500` | Enumerator value |
| `DPS1000` | Enumerator value |
| `DPS2000` | Enumerator value |

---

## IHalCan
**Kind:** `class`  
**Namespace:** `apx::hal::apx::drivers`  
**Header:** `drivers/rangefinder/can_radar/can_radar_driver.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `int` | `read(CanFrame* frame)` | Member function. |
| `int` | `write(const CanFrame& frame)` | Member function. |

---

## IHalSpi
**Kind:** `class`  
**Namespace:** `apx::hal::apx::drivers`  
**Header:** `drivers/flow/pmw3901/pmw3901.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `uint8_t` | `transfer(uint8_t data)` | Member function. |
| `void` | `select()` | Member function. |
| `void` | `deselect()` | Member function. |

---

## Icm20689
**Kind:** `class`  
**Namespace:** `apx::drivers::Icm20689Reg`  
**Header:** `drivers/imu/icm20689/icm20689.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Icm20689(apx::hal::IHalSpi& spi, uint8_t cs_id)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## Icm42605
**Kind:** `class`  
**Namespace:** `apx::drivers::Icm42605Reg`  
**Header:** `drivers/imu/icm42605/icm42605.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Icm42605(apx::hal::IHalSpi& spi, uint8_t cs_id)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## Icm42688
**Kind:** `class`  
**Namespace:** `apx::drivers::Icm42688Reg`  
**Header:** `drivers/imu/icm42688/icm42688.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Icm42688(apx::hal::IHalSpi& spi, uint8_t cs_id, ImuOdr odr = ImuOdr::HZ_1000, AccelRange accel_range = AccelRange::G16, GyroRange gyro_range = GyroRange::DPS2000)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `deinit()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |
| `bool` | `self_test()` | Member function. |

---

## ImuData
**Kind:** `struct`  
**Namespace:** `apx::drivers::Icm42688Reg`  
**Header:** `drivers/imu/icm42688/icm42688.hpp`  

---

## ImuOdr
**Kind:** `enum`  
**Namespace:** `apx::drivers::Icm42688Reg`  
**Header:** `drivers/imu/icm42688/icm42688.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `HZ_1000` | Enumerator value |
| `HZ_500` | Enumerator value |
| `HZ_200` | Enumerator value |

---

## Ina226
**Kind:** `class`  
**Namespace:** `apx::drivers::Ina226Reg`  
**Header:** `drivers/battery/ina226/ina226.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Ina226(apx::hal::IHalI2c& i2c, uint8_t addr, float shunt_ohm = 0.001f, uint8_t cell_count = 6)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## Ist8310
**Kind:** `class`  
**Namespace:** `apx::drivers::Ist8310Reg`  
**Header:** `drivers/mag/ist8310/ist8310.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Ist8310(apx::hal::IHalI2c& i2c, uint8_t addr = Ist8310Reg::DEFAULT_ADDR)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## Lis3mdl
**Kind:** `class`  
**Namespace:** `apx::drivers::Lis3mdlReg`  
**Header:** `drivers/mag/lis3mdl/lis3mdl.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Lis3mdl(apx::hal::IHalI2c& i2c, uint8_t addr = Lis3mdlReg::DEFAULT_ADDR)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## MagData
**Kind:** `struct`  
**Namespace:** `apx::drivers::Ist8310Reg`  
**Header:** `drivers/mag/ist8310/ist8310.hpp`  

---

## Ms5611
**Kind:** `class`  
**Namespace:** `apx::drivers::Ms5611Cmd`  
**Header:** `drivers/baro/ms5611/ms5611.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Ms5611(apx::hal::IHalSpi& spi, uint8_t cs_id)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |

---

## ObstacleData
**Kind:** `struct`  
**Namespace:** `apx::hal::apx::drivers`  
**Header:** `drivers/rangefinder/can_radar/can_radar_driver.hpp`  

---

## ParseState
**Kind:** `enum`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/gps/ublox/ublox_driver.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `SYNC1` | Enumerator value |
| `SYNC2` | Enumerator value |
| `CLASS` | Enumerator value |
| `ID` | Enumerator value |
| `LEN1` | Enumerator value |
| `LEN2` | Enumerator value |
| `PAYLOAD` | Enumerator value |
| `CK_A` | Enumerator value |
| `CK_B` | Enumerator value |

---

## Pmw3901Driver
**Kind:** `class`  
**Namespace:** `apx::hal::apx::drivers`  
**Header:** `drivers/flow/pmw3901/pmw3901.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Pmw3901Driver(apx::hal::IHalSpi& spi)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `FlowData` | `get_data()` | Member function. |

---

## PumpDriver
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/pump/pump_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `PumpDriver(apx::hal::IHalPwm& pwm, uint8_t channel, uint32_t freq_hz = 50)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `void` | `set_flow_rate(float rate_0_to_1)` | Member function. |
| `void` | `enable()` | Member function. |
| `void` | `disable()` | Member function. |
| `void` | `update_volume(float dt_s, float area_m2, float speed_ms)` | Member function. |

---

## RcData
**Kind:** `struct`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/rc_input/sbus/sbus_driver.hpp`  

---

## RelayDriver
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/relay/relay_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `RelayDriver(apx::hal::IHalGpio& gpio, uint8_t pin, bool active_high = true)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `set_state(bool on)` | Member function. |
| `void` | `toggle()` | Member function. |

---

## Rm3100
**Kind:** `class`  
**Namespace:** `apx::drivers::Rm3100Reg`  
**Header:** `drivers/mag/rm3100/rm3100.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Rm3100(apx::hal::IHalSpi& spi, uint8_t cs_id, uint16_t cycle_count = 200)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |

---

## SbusDriver
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/rc_input/sbus/sbus_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `SbusDriver(apx::hal::IHalUart& uart)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |

---

## ServoDriver
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/servo/servo_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `ServoDriver(apx::hal::IHalPwm& pwm, uint8_t first_ch = 0, uint8_t count = 8)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `set_pwm_us(uint8_t ch, uint16_t us)` | Member function. |
| `void` | `set_normalized(uint8_t ch, float val_neg1_to_1)` | Member function. |
| `void` | `set_throttle(uint8_t ch, float throttle_0_to_1)` | Member function. |
| `void` | `disarm_all()` | Member function. |
| `void` | `arm()` | Member function. |
| `void` | `set_channel_config(uint8_t ch, const ChannelConfig& cfg)` | Member function. |
| `ChannelConfig&` | `get_channel_config(uint8_t ch)` | Member function. |

---

## State
**Kind:** `enum`  
**Namespace:** `apx::drivers::Ms5611Cmd`  
**Header:** `drivers/baro/ms5611/ms5611.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `IDLE` | Enumerator value |
| `WAIT_TEMP` | Enumerator value |
| `WAIT_PRESS` | Enumerator value |

---

## UartMavlinkDriver
**Kind:** `class`  
**Namespace:** `apx::hal::apx::drivers`  
**Header:** `drivers/telemetry/uart_mavlink/uart_mavlink_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `UartMavlinkDriver(apx::hal::IHalUart& uart, uint32_t baud = 57600)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `int` | `read_bytes(uint8_t* buf, int max_len)` | Member function. |
| `int` | `write_bytes(const uint8_t* buf, int len)` | Member function. |

---

## UbloxDriver
**Kind:** `class`  
**Namespace:** `apx::drivers`  
**Header:** `drivers/gps/ublox/ublox_driver.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `UbloxDriver(apx::hal::IHalUart& uart)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |

---

## UbloxF9P
**Kind:** `class`  
**Namespace:** `apx::drivers::F9PKey`  
**Header:** `drivers/gps/ublox_f9p/ublox_f9p.hpp`  
**Inherits from:** `DriverBase`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `UbloxF9P(apx::hal::IHalUart& uart, uint32_t baud = 460800)` | Member function. |
| `bool` | `init()` | Member function. |
| `void` | `update()` | Member function. |
| `bool` | `is_healthy()` | Member function. |
| `void` | `inject_rtcm(const uint8_t* data, size_t len)` | Forward RTCM corrections from base station |

---
