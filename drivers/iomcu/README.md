# IO coprocessor (IO MCU) support

Some flight controllers have a second, small MCU next to the main processor.
It handles:

- the **MAIN** PWM outputs (MAIN 1–8),
- **RC input** (SBUS, DSM, PPM, ...),
- the **safety switch** and its LED,
- a **failsafe** that holds outputs at safe values if the main MCU stops talking.

The main MCU's own timers drive only the **AUX** outputs on these boards.

| Board | IO link |
|---|---|
| CubePilot Cube Orange / Orange+ | USART6 (PC6/PC7) |
| Holybro Pixhawk 6X, Pixhawk 6C | USART6 |
| Holybro Durandal | UART8 |
| SIYI N7 | UART8 |
| APX H743 Pro | USART6 (PC6/PC7) |

Every other board has no IO MCU. Their outputs and RC input stay on the main
MCU, and nothing changes for them: the driver is never started.

## How it works

`io_coprocessor.cpp` is the main-MCU side of the IO register protocol. It's
portable C++ that only needs an `apx::hal::IHalUart`.

- **Link:** UART at 1.5 Mbaud, 8N1.
- **Packet:** byte 0 is the register count (bits 0–5) and the code (bits 6–7:
  read, write / success, corrupt, error). Byte 1 is a CRC-8 (polynomial 0x07,
  initial value 0, computed with this byte zeroed). Byte 2 is the page and
  byte 3 the offset, followed by up to 22 little-endian 16-bit registers.
- **Transactions:** a read sends a request the size of the expected reply
  (registers zeroed); the IO MCU answers with the registers. A write sends
  the registers and gets a 4-byte reply.
- **Pages used:** config (protocol version and MCU id), status (safety
  switch, servo rail voltage, IO uptime and errors), raw RC input
  (16 channels, failsafe, protocol, RSSI) and direct PWM (outputs).

The driver is non-blocking. The flight loop calls `update()` every tick,
and at most one transaction is in flight with a 10 ms timeout:

| State | Meaning |
|---|---|
| probing | reads the config page every 500 ms until the IO MCU answers |
| ready | protocol version 4: RC input every 20 ms, status every 50 ms |
| unsupported | answers, but with another protocol version (other IO firmware) |
| not responding | was ready, then 20 transactions in a row failed; re-probes |

## What the firmware does with it

- **Status messages** report every state change, e.g. *IO coprocessor ready
  (protocol 4/10)*, plus safety-switch changes.
- **RC input** from the IO MCU is sent to the ground station as
  `RC_CHANNELS_RAW` at 5 Hz.
- **Outputs are never written during bring-up.** `enable_outputs()` stays
  off, so the IO MCU keeps its own disarmed/failsafe values. Motor output
  support will turn it on once arming is implemented.

## Board wiring

The board's devicetree selects the link, and `tools/import_board.py` writes it
for every board whose hardware description names an IO UART:

```dts
chosen { apx,iomcu-uart = &usart6; };
&usart6 { current-speed = <1500000>; fifo-enable; status = "okay"; ... };
```

`apx::board::iomcu_link()` returns that UART, or `nullptr` on boards without one.

## IO firmware

The driver talks to the IO firmware already on the IO MCU (protocol version 4,
which these boards normally ship with). It doesn't flash the IO MCU. If the IO
MCU runs firmware with a different protocol, the state is *unsupported*, and the
ground station is told so.

## Tests

```bash
tests/iomcu/run_tests.sh
```

These tests check the CRC against the published check value and the header
byte against the C bitfield layout. They then run the driver against a
simulated IO MCU: probe, RC and status parsing, no output writes until enabled,
corrupt replies, a silent IO MCU (lost, then recovered) and an unsupported
protocol.
