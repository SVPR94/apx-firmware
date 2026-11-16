# Kernel Subsystem API Reference

Namespace: `apx::kernel`

## Summary

This page documents all public classes, data structures, and enumerations for `Kernel Subsystem`.

| Symbol | Kind | Declared In | Description |
|---|---|---|---|
| [`LockGuard`](#lockguard) | `class` | `kernel/include/apx/kernel/mutex.hpp` | Class definition. |
| [`Mutex`](#mutex) | `class` | `kernel/include/apx/kernel/mutex.hpp` | Class definition. |
| [`Task`](#task) | `class` | `kernel/include/apx/kernel/task.hpp` | Class definition. |
| [`Topic`](#topic) | `class` | `kernel/include/apx/kernel/ubus.hpp` | Class definition. |

---

## LockGuard
**Kind:** `class`  
**Namespace:** `apx::kernel`  
**Header:** `kernel/include/apx/kernel/mutex.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `lock()` | Member function. |
| `` | `unlock()` | Member function. |

---

## Mutex
**Kind:** `class`  
**Namespace:** `apx::kernel`  
**Header:** `kernel/include/apx/kernel/mutex.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `Mutex()` | Member function. |
| `` | `~Mutex()` | Member function. |
| `void` | `lock()` | Member function. |
| `void` | `unlock()` | Member function. |

---

## Task
**Kind:** `class`  
**Namespace:** `apx::kernel`  
**Header:** `kernel/include/apx/kernel/task.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `~Task()` | Member function. |
| `void` | `start()` | Member function. |
| `void` | `stop()` | Member function. |
| `void` | `join()` | Member function. |
| `void` | `set_period_us(uint32_t us)` | Member function. |
| `void` | `sleep_until_next()` | Member function. |

---

## Topic
**Kind:** `class`  
**Namespace:** `apx::kernel`  
**Header:** `kernel/include/apx/kernel/ubus.hpp`  

### Public Methods & Members
| Return Type | Signature | Description |
|---|---|---|
| `` | `load(std::memory_order_relaxed)` | Member function. |
| `` | `store(head + 1, std::memory_order_release)` | Member function. |
| `` | `load(std::memory_order_acquire)` | Member function. |
| `` | `load(std::memory_order_acquire)` | Member function. |
| `T` | `get_latest()` | Member function. |

---
