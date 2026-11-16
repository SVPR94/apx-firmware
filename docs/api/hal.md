# Hardware Abstraction Layer API Reference

Namespace: `apx::hal`

## Summary

This page documents all public classes, data structures, and enumerations for `Hardware Abstraction Layer`.

| Symbol | Kind | Declared In | Description |
|---|---|---|---|
| [`CanFrame`](#canframe) | `struct` | `hal/include/apx/hal/hal_can.hpp` | Struct definition. |
| [`Direction`](#direction) | `enum` | `hal/include/apx/hal/hal_gpio.hpp` | Enum definition. |
| [`Edge`](#edge) | `enum` | `hal/include/apx/hal/hal_gpio.hpp` | Enum definition. |
| [`IHalCan`](#ihalcan) | `class` | `hal/include/apx/hal/hal_can.hpp` | Class definition. |
| [`IHalGpio`](#ihalgpio) | `class` | `hal/include/apx/hal/hal_gpio.hpp` | Class definition. |
| [`IHalI2c`](#ihali2c) | `class` | `hal/include/apx/hal/hal_i2c.hpp` | Class definition. |
| [`IHalPwm`](#ihalpwm) | `class` | `hal/include/apx/hal/hal_pwm.hpp` | Class definition. |
| [`IHalSpi`](#ihalspi) | `class` | `hal/include/apx/hal/hal_spi.hpp` | Class definition. |
| [`IHalStorage`](#ihalstorage) | `class` | `hal/include/apx/hal/hal_storage.hpp` | Class definition. |
| [`IHalTimer`](#ihaltimer) | `class` | `hal/include/apx/hal/hal_timer.hpp` | Class definition. |
| [`IHalUart`](#ihaluart) | `class` | `hal/include/apx/hal/hal_uart.hpp` | Class definition. |
| [`Pull`](#pull) | `enum` | `hal/include/apx/hal/hal_gpio.hpp` | Enum definition. |
| [`SpiMode`](#spimode) | `enum` | `hal/include/apx/hal/hal_spi.hpp` | Enum definition. |

---

## CanFrame
**Kind:** `struct`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_can.hpp`  

---

## Direction
**Kind:** `enum`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_gpio.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `INPUT` | Enumerator value |
| `OUTPUT` | Enumerator value |

---

## Edge
**Kind:** `enum`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_gpio.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `RISING` | Enumerator value |
| `FALLING` | Enumerator value |
| `BOTH` | Enumerator value |

---

## IHalCan
**Kind:** `class`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_can.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `void` | `init(uint32_t baud_rate)` | Member function. |
| `bool` | `send(const CanFrame& frame)` | Member function. |
| `bool` | `recv(CanFrame& frame, uint32_t timeout_ms)` | Member function. |

---

## IHalGpio
**Kind:** `class`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_gpio.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `void` | `configure(uint32_t pin, Direction direction, Pull pull = Pull::NONE)` | Member function. |
| `void` | `write(uint32_t pin, bool value)` | Member function. |
| `bool` | `read(uint32_t pin)` | Member function. |
| `void` | `set_interrupt(uint32_t pin, Edge edge, InterruptCallback callback)` | Member function. |

---

## IHalI2c
**Kind:** `class`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_i2c.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `void` | `init(uint32_t clock_hz)` | Member function. |
| `bool` | `write_reg(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len)` | Member function. |
| `bool` | `read_reg(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len)` | Member function. |

---

## IHalPwm
**Kind:** `class`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_pwm.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `void` | `init(uint32_t channel, uint32_t freq_hz)` | Member function. |
| `void` | `set_duty(uint32_t channel, uint32_t duty_us)` | Member function. |
| `void` | `enable(uint32_t channel)` | Member function. |
| `void` | `disable(uint32_t channel)` | Member function. |

---

## IHalSpi
**Kind:** `class`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_spi.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `void` | `init(uint32_t clock_hz, SpiMode mode, uint8_t bits = 8)` | Member function. |
| `bool` | `transfer(const uint8_t* tx, uint8_t* rx, size_t len)` | Member function. |
| `void` | `cs_select(uint32_t id)` | Member function. |
| `void` | `cs_deselect(uint32_t id)` | Member function. |

---

## IHalStorage
**Kind:** `class`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_storage.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `bool` | `read(uint32_t offset, uint8_t* buf, size_t len)` | Member function. |
| `bool` | `write(uint32_t offset, const uint8_t* buf, size_t len)` | Member function. |
| `bool` | `erase(uint32_t offset, size_t len)` | Member function. |
| `void` | `sync()` | Member function. |

---

## IHalTimer
**Kind:** `class`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_timer.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `uint64_t` | `get_microseconds()` | Member function. |
| `uint64_t` | `get_milliseconds()` | Member function. |
| `void` | `delay_us(uint32_t us)` | Member function. |
| `void` | `delay_ms(uint32_t ms)` | Member function. |

---

## IHalUart
**Kind:** `class`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_uart.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `bool` | `init(uint32_t baud_rate)` | Member function. |
| `void` | `deinit()` | Member function. |
| `int` | `write(const uint8_t* buf, size_t len)` | Member function. |
| `int` | `read(uint8_t* buf, size_t max_len)` | Member function. |
| `int` | `available()` | Member function. |
| `void` | `flush()` | Member function. |
| `` | `init(baud_rate)` | Member function. |
| `void` | `set_rx_callback(RxCallback fn)` | Member function. |

---

## Pull
**Kind:** `enum`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_gpio.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `NONE` | Enumerator value |
| `UP` | Enumerator value |
| `DOWN` | Enumerator value |

---

## SpiMode
**Kind:** `enum`  
**Namespace:** `apx::hal`  
**Header:** `hal/include/apx/hal/hal_spi.hpp`  

### Enumerators
| Value | Description |
|---|---|
| `MODE_0` | Enumerator value |
| `CPHA` | Enumerator value |
| `CPHA` | Enumerator value |
| `CPHA` | Enumerator value |
| `CPHA` | Enumerator value |

---
