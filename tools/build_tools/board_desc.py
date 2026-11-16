# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Board pin model: which MCU pin goes to which peripheral, for the board diagram.

Sources (both "hwdef" style hardware descriptions):
  * full board descriptions (SERIAL_ORDER, SPIDEV, IMU/BARO/COMPASS, PWM(n),
    IOMCU_UART, ...), converted by tools/import_board.py into
    platforms/zephyr/boards/<vendor>/<board>/board_pins.json, or read directly
    for APX boards in platforms/stm32h7/boards/<board>/hwdef.dat (an IO MCU pin
    map sits next to it in iomcu.dat);
  * APX's own short format in platforms/stm32h7/boards/<board>/hwdef.dat
    (UART n TX RX BAUD ROLE, SPIDEV, I2CDEV, MOTOR, SERVO, LED, RELAY).

The model is a JSON-able dict:
  {"board", "title", "mcu", "notes": [...],
   "groups": [{"kind", "name", "side", "pins": [[function, port, detail]],
               "devices": [{"icon", "name", "role", "detail"}],
               "iomcu": {"mcu", "pins": [[function, port, detail]]} (kind "iomcu" only)}]}

Device icons: imu baro compass power fram sd flash osd airspeed flow chip gps radio
rc esc gcs port can. Pin rows of pwm / adc / gpio groups and IO MCU rows get their
icon (motor, servo, led, battery, current, ...) from the row text, see diagram.py.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

PIN_RE = re.compile(r"^P([A-K])(\d{1,2})$")
RAIL_RE = re.compile(r"^(n?)VDD_\w*_EN\d*$")
LED_COLORS = ("RED", "GREEN", "BLUE", "AMBER", "YELLOW", "ORANGE", "WHITE")


# ---------------------------------------------------------------------------
# Parser (include / undef aware)
# ---------------------------------------------------------------------------
@dataclass
class Pin:
    port: str
    label: str
    args: list
    comment: str = ""


@dataclass
class Hwdef:
    config: dict = field(default_factory=dict)    # last value per key
    defines: dict = field(default_factory=dict)
    pins: dict = field(default_factory=dict)      # port -> Pin
    lines: list = field(default_factory=list)     # (key, tokens) for repeatable directives

    def label(self, label):
        return next((p for p in self.pins.values() if p.label == label), None)

    def all(self, key):
        return [tok for k, tok in self.lines if k == key]


def parse_hwdef(path, hw=None, depth=0) -> Hwdef:
    if depth > 8:
        raise ValueError(f"include nesting too deep at {path}")
    hw = hw or Hwdef()
    path = Path(path)
    for raw in path.read_text(errors="replace").splitlines():
        line, _, comment = raw.partition("#")
        line = line.strip()
        if not line:
            continue
        tok = line.split()
        key = tok[0]
        if key == "include":
            parse_hwdef(path.parent / tok[1], hw, depth + 1)
        elif key == "undef":
            for name in tok[1:]:
                hw.config.pop(name, None)
                hw.defines.pop(name, None)
                hw.pins.pop(name, None)
                hw.lines = [(k, t) for k, t in hw.lines if k != name]
                for port, pin in list(hw.pins.items()):
                    if pin.label == name:
                        del hw.pins[port]
        elif key == "define" and len(tok) >= 2:
            hw.defines[tok[1]] = " ".join(tok[2:])
        elif PIN_RE.match(key) and len(tok) >= 2:
            hw.pins[key] = Pin(key, tok[1], tok[2:], comment.strip())
        else:
            hw.config[key] = " ".join(tok[1:])
            hw.lines.append((key, tok[1:]))
    return hw


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
_KIND_HINTS = (
    ("IMU", ("icm", "bmi", "mpu", "adis", "iim", "lsm", "imu", "l3gd", "invensense")),
    ("Baro", ("ms56", "ms5611", "bmp", "dps", "spl", "icp", "lps", "baro")),
    ("Compass", ("rm3100", "ist8310", "hmc", "lis3mdl", "qmc", "bmm", "ak09916", "ak8963", "mag")),
    ("FRAM (parameters)", ("ramtron", "fram")),
    ("SD card", ("sdcard", "sd_card")),
    ("OSD", ("max7456", "osd")),
    ("Log flash", ("dataflash", "w25", "jedec", "nand")),
    ("Pressure / airspeed", ("ms4525", "dlvr")),
    ("Power monitor", ("ina2", "ina3")),
    ("Optical flow", ("pixart", "pmw3901", "flow")),
)

# kind -> (icon, role shown under the part name)
_KIND_ICON = {
    "IMU": ("imu", "Gyro + accelerometer"),
    "Baro": ("baro", "Barometer (altitude)"),
    "Compass": ("compass", "Magnetometer (heading)"),
    "FRAM (parameters)": ("fram", "FRAM: parameter storage"),
    "SD card": ("sd", "microSD: logs, missions"),
    "OSD": ("osd", "Analog video OSD"),
    "Log flash": ("flash", "Flash: on-board logs"),
    "Pressure / airspeed": ("airspeed", "Airspeed sensor"),
    "Power monitor": ("power", "Power monitor (V, A)"),
    "Optical flow": ("flow", "Optical flow"),
    "device": ("chip", "device"),
}


def device(icon, name, role="", detail=""):
    return {"icon": icon, "name": name, "role": role, "detail": detail}


def device_text(d) -> str:
    """One-line text for a device (dict or the older plain string)."""
    if isinstance(d, str):
        return d
    parts = [d.get("name", "")]
    if d.get("role") and d["role"] != d.get("name"):
        parts.append(f"({d['role']})")
    if d.get("detail"):
        parts.append(d["detail"])
    return " ".join(p for p in parts if p)


def device_kind(name, known):
    if name in known:
        return known[name]
    low = name.lower()
    for kind, hints in _KIND_HINTS:
        if any(h in low for h in hints):
            return kind
    return "device"


def _part_name(name):
    """icm42688_ext -> ICM42688 (ext), ms5611 -> MS5611."""
    base, _, suffix = name.partition("_")
    out = base.upper() if len(base) <= 10 else base
    return f"{out} ({suffix.lower()})" if suffix else out


def sensor_device(name, known, detail=""):
    if re.match(r"^external\d*m\d+$", name, re.I):   # one entry per SPI mode of an external connector
        return device("port", "External SPI connector", "user sensors", detail)
    kind = device_kind(name, known)
    icon, role = _KIND_ICON.get(kind, _KIND_ICON["device"])
    return device(icon, _part_name(name), role, detail)


_ROLE_DEVICES = (  # serial role -> (icon, device name, what it carries)
    ("TELEM1", ("radio", "Telemetry radio", "MAVLink to the ground station")),
    ("TELEM2", ("radio", "Telemetry / companion", "MAVLink")),
    ("GPS1", ("gps", "GPS + compass module", "primary GNSS")),
    ("GPS2", ("gps", "Second GPS", "secondary GNSS")),
    ("RC input", ("rc", "RC receiver", "SBUS / CRSF / DSM")),
    ("ESC telemetry", ("esc", "ESC telemetry", "RPM, current, temperature")),
    ("ESC_TELEM", ("esc", "ESC telemetry", "RPM, current, temperature")),
    ("RC", ("rc", "RC receiver", "SBUS")),
    ("GPS", ("gps", "GPS + compass module", "GNSS")),
    ("TELEM", ("radio", "Telemetry radio", "MAVLink")),
    ("MAVLink", ("radio", "MAVLink device", "telemetry / companion")),
    ("MSP", ("osd", "MSP device", "OSD / sensors")),
    ("DJI OSD", ("osd", "Digital video OSD", "MSP DisplayPort")),
    ("Rangefinder", ("chip", "Rangefinder", "distance sensor")),
    ("VTX control", ("radio", "Video transmitter", "VTX control")),
)


def serial_device(role):
    for key, (icon, name, what) in _ROLE_DEVICES:
        if role and key in role:
            return device(icon, name, what)
    return device("port", "Spare serial port", "user-assigned protocol")


def _protocol_role(text):
    t = text.lower()
    for key, role in (("rcin", "RC input"), ("gps", "GPS"), ("mavlink", "MAVLink"),
                      ("esctelemetry", "ESC telemetry"), ("msp", "MSP"), ("dji", "DJI OSD"),
                      ("rangefinder", "Rangefinder"), ("smartaudio", "VTX control"),
                      ("tramp", "VTX control"), ("none", "unused")):
        if key in t:
            return role
    return None


def _first(hw, *labels):
    for label in labels:
        pin = hw.label(label)
        if pin:
            return pin
    return None


def _pinrow(function, pin, detail=""):
    return [function, pin.port if pin else "-", detail]


def led_color(pin):
    """Colour word for an LED pin from its label or its comment, else ''."""
    text = f"{pin.label} {pin.comment}".upper()
    return next((c.lower() for c in LED_COLORS if c in text), "")


def _mcu(hw, default="?"):
    return hw.config.get("MCU", "").split()[-1] if hw.config.get("MCU") else default


# ---------------------------------------------------------------------------
# Groups shared by both formats
# ---------------------------------------------------------------------------
_ADC_ROLES = (  # label pattern -> what the input measures
    (r"BATT\d?_VOLT", "battery voltage"), (r"BATT\d?_CURR", "battery current"),
    (r"VDD_5V|5V_SENS|VBUS", "5 V rail"), (r"VSERVO|SERVO_V", "servo rail"),
    (r"RSSI", "RSSI (link quality)"), (r"PRESSURE|AIRSPEED", "analog airspeed"),
    (r"VDD_3V3|3V3_SENS", "3.3 V rail"), (r"SPARE|ADC_IN", "spare analog input"),
)


def adc_group(hw):
    rows = []
    for p in hw.pins.values():
        ch = next((a for a in p.args if re.match(r"^ADC\d$", a)), None)
        if not ch:
            continue
        what = next((role for pat, role in _ADC_ROLES if re.search(pat, p.label)), "analog input")
        rows.append([p.label, p.port, f"{ch}: {what}"])
    if not rows:
        return None
    roles = [role for _, role in _ADC_ROLES]
    rows.sort(key=lambda r: (not r[0].startswith("BATT"), re.sub(r"\D", "", r[0].split("_")[0]) or "0",
                             next((i for i, x in enumerate(roles) if x in r[2]), 99)))
    return {"kind": "adc", "name": "Power & analog inputs (ADC)", "side": "left", "pins": rows, "devices": []}


def gpio_group(hw, extra=()):
    rows = list(extra)
    seen = {r[1] for r in rows}
    for p in hw.pins.values():
        if p.port in seen:
            continue
        lab = p.label
        is_out = any(a in ("OUTPUT", "ACTIVE_LOW", "ACTIVE_HIGH") for a in p.args) or not p.args
        if "LED" in lab and is_out and not lab.startswith("OTG"):
            color = led_color(p)
            rows.append(["LED", p.port, f"{lab} ({color})" if color else lab])
        elif RAIL_RE.match(lab):
            rows.append(["power", p.port, lab])
        elif "ALARM" in p.args or any(k in lab for k in ("BUZZER", "ALARM")):
            rows.append(["buzzer", p.port, lab])
        elif "SAFETY" in lab:
            rows.append(["safety", p.port, lab])
        elif "HEATER" in lab:
            rows.append(["heater", p.port, lab])
        elif lab.startswith("RELAY"):
            rows.append(["relay", p.port, lab])
        else:
            continue
        seen.add(p.port)
    if not rows:
        return None
    order = ("LED", "buzzer", "safety", "heater", "relay", "power")
    rows.sort(key=lambda r: order.index(r[0]) if r[0] in order else 99)
    return {"kind": "gpio", "name": "LEDs, buzzer, power switches", "side": "left", "pins": rows, "devices": []}


def sdmmc_group(hw):
    rows = []
    for fn in ("CK", "CMD", "D0", "D1", "D2", "D3"):
        pin = hw.label(f"SDMMC1_{fn}") or hw.label(f"SDMMC2_{fn}")
        if pin:
            rows.append([fn, pin.port, pin.label])
    if not rows:
        return None
    bus = rows[0][2].split("_")[0]
    return {"kind": "sdmmc", "name": f"{bus} (4-bit SD bus)", "side": "right", "pins": rows,
            "devices": [device("sd", "microSD card", "logs, missions, terrain")]}


# ---------------------------------------------------------------------------
# Full hardware description
# ---------------------------------------------------------------------------
def describe_full(hw: Hwdef, board: str, title: str, iomcu_hw: Hwdef | None = None) -> dict:
    mcu = _mcu(hw)
    groups = []
    io_uart = (hw.config.get("IOMCU_UART") or "").split()[0].upper() if hw.config.get("IOMCU_UART") else ""

    # sensor kinds named by IMU / BARO / COMPASS lines
    known = {}
    i2c_devices = []  # (bus index or ALL_*, device)
    for key, kind in (("IMU", "IMU"), ("BARO", "Baro"), ("COMPASS", "Compass")):
        for tok in hw.all(key):
            for t in tok:
                if t.startswith("SPI:"):
                    known[t[4:]] = kind
                elif t.startswith("I2C:"):
                    parts = t.split(":")
                    if len(parts) >= 3:
                        icon, role = _KIND_ICON[kind]
                        i2c_devices.append((parts[1], device(icon, tok[0], role, f"address {parts[2]}")))

    # --- serial ports
    order = (hw.config.get("SERIAL_ORDER") or "").split()
    default_roles = {1: "TELEM1", 2: "TELEM2", 3: "GPS1", 4: "GPS2"}
    for idx, periph in enumerate(order):
        periph = periph.upper()
        if periph == "EMPTY":
            continue
        if periph.startswith("OTG"):
            dm = _first(hw, "OTG_FS_DM" if periph == "OTG1" else "OTG_HS_DM")
            dp = _first(hw, "OTG_FS_DP" if periph == "OTG1" else "OTG_HS_DP")
            if dm or dp:
                groups.append({"kind": "usb", "name": f"SERIAL{idx} · USB ({periph})", "side": "left",
                               "pins": [_pinrow("D-", dm, dm.label if dm else ""),
                                        _pinrow("D+", dp, dp.label if dp else "")],
                               "devices": [device("gcs", "Ground station (USB)", "MAVLink, firmware upload")
                                           if idx == 0 else device("gcs", "Second USB port", "SLCAN / MAVLink")]})
            continue
        if periph == io_uart:
            continue
        role = default_roles.get(idx, f"SERIAL{idx}")
        for key in (f"HAL_SERIAL{idx}_PROTOCOL", f"DEFAULT_SERIAL{idx}_PROTOCOL"):
            proto = hw.defines.get(key) or hw.config.get(key)
            if proto and _protocol_role(proto):
                role = f"{role}: {_protocol_role(proto)}" if role.startswith(("TELEM", "GPS")) else _protocol_role(proto)
        rows = []
        for fn in ("TX", "RX", "RTS", "CTS"):
            pin = hw.label(f"{periph}_{fn}")
            if pin:
                rows.append(_pinrow(fn, pin, f"{periph}_{fn}"))
        if rows:
            proto = role.split(": ", 1)[1] if ": " in role else role
            dev = serial_device(proto if proto != "unused" else "")
            if any(r[0] == "RTS" for r in rows):
                dev["detail"] = "RTS/CTS flow control"
            shown = "spare" if role == f"SERIAL{idx}" else role
            groups.append({"kind": "serial", "name": f"SERIAL{idx} · {shown} ({periph})", "side": "left",
                           "pins": rows, "devices": [dev]})

    # --- IO coprocessor
    if io_uart:
        rows = [_pinrow(fn, hw.label(f"{io_uart}_{fn}"), f"{io_uart}_{fn}") for fn in ("TX", "RX")]
        groups.append({"kind": "iomcu", "name": f"IO MCU link ({io_uart})", "side": "right",
                       "pins": rows, "devices": [], "iomcu": describe_iomcu(iomcu_hw)})

    # --- SPI buses
    spidev = hw.all("SPIDEV")
    for bus in sorted({p.label.split("_")[0] for p in hw.pins.values() if re.match(r"^SPI\d+_", p.label)},
                      key=lambda b: int(b[3:])):
        rows = [_pinrow(fn, hw.label(f"{bus}_{fn}"), f"{bus}_{fn}") for fn in ("SCK", "MISO", "MOSI")
                if hw.label(f"{bus}_{fn}")]
        devices, seen_cs = [], {}
        for tok in spidev:
            if len(tok) >= 4 and tok[1] == bus:
                name, cs_label = tok[0], tok[3]
                cs = hw.label(cs_label)
                dev = sensor_device(name, known, f"CS {cs.port}" if cs else "")
                if dev not in devices:
                    devices.append(dev)
                if cs and cs.port not in seen_cs:
                    seen_cs[cs.port] = cs_label
        rows += [["CS", port, label] for port, label in seen_cs.items()]
        groups.append({"kind": "spi", "name": bus, "side": "right", "pins": rows,
                       "devices": devices or [device("port", "SPI connector", "no devices listed")]})

    # --- I2C buses
    i2c_order = [b.upper() for b in (hw.config.get("I2C_ORDER") or "").split()]
    mask = hw.defines.get("HAL_I2C_INTERNAL_MASK")
    mask = int(mask, 0) if mask and mask.strip().lstrip("-").isdigit() else None
    ext_compass = "HAL_PROBE_EXTERNAL_I2C_COMPASSES" in hw.defines
    buses = sorted({p.label.split("_")[0] for p in hw.pins.values() if re.match(r"^I2C\d+_", p.label)},
                   key=lambda b: int(b[3:]))
    for bus in buses:
        idx = i2c_order.index(bus) if bus in i2c_order else None
        internal = None if mask is None or idx is None else bool(mask & (1 << idx))
        rows = [_pinrow(fn, hw.label(f"{bus}_{fn}"), f"{bus}_{fn}") for fn in ("SCL", "SDA") if hw.label(f"{bus}_{fn}")]
        devices = [d for where, d in i2c_devices if idx is not None and where == str(idx)]
        devices += [d for where, d in i2c_devices
                    if (where == "ALL_EXTERNAL" and internal is False) or (where == "ALL_INTERNAL" and internal)]
        devices += [sensor_device(t[0], {}, f"address {t[2]}") for t in hw.all("I2CDEV") if len(t) >= 3 and t[1] == bus]
        if ext_compass and internal is not True:
            devices.append(device("compass", "External compass", "in the GPS module", "probed at boot"))
        suffix = "" if internal is None else (" (internal)" if internal else " (external)")
        groups.append({"kind": "i2c", "name": f"{bus}{suffix}" + (f" · bus {idx}" if idx is not None else ""),
                       "side": "right", "pins": rows,
                       "devices": devices or [device("port", "I2C connector", "external sensors")]})

    # --- PWM outputs
    pwm = []
    for pin in hw.pins.values():
        m = next((re.match(r"PWM\((\d+)\)", a) for a in pin.args if a.startswith("PWM(")), None)
        if m:
            pwm.append((int(m.group(1)), pin))
    if pwm:
        prefix = "AUX" if io_uart else "OUT"
        rows = [[f"{prefix} {n}", pin.port, pin.label] for n, pin in sorted(pwm, key=lambda x: x[0])]
        name = (f"AUX 1-{len(rows)} outputs (main MCU)" if io_uart
                else f"Motor outputs 1-{len(rows)} (PWM / DShot)")
        groups.append({"kind": "pwm", "name": name, "side": "right", "pins": rows, "devices": []})

    # --- RC input on the main MCU
    rc = [p for p in hw.pins.values() if any(a in ("RCININT", "RCINICU") for a in p.args) or p.label in ("RCIN", "RCININT")]
    if rc:
        groups.append({"kind": "rcin", "name": "RC input", "side": "left",
                       "pins": [["RCIN", p.port, p.label] for p in rc],
                       "devices": [device("rc", "RC receiver", "PPM / SBUS / DSM / CRSF")]})

    # --- CAN
    for bus in sorted({p.label.split("_")[0] for p in hw.pins.values() if re.match(r"^CAN\d_", p.label)}):
        rows = [_pinrow(fn, hw.label(f"{bus}_{fn}"), f"{bus}_{fn}") for fn in ("TX", "RX") if hw.label(f"{bus}_{fn}")]
        groups.append({"kind": "can", "name": bus, "side": "left", "pins": rows,
                       "devices": [device("can", "CAN bus (DroneCAN)", "GPS, compass, ESCs, power module")]})

    # --- storage, analog inputs, LEDs / power / buzzer
    for g in (sdmmc_group(hw), adc_group(hw), gpio_group(hw)):
        if g:
            groups.append(g)

    notes = []
    if spidev:
        notes.append("SPI device lists include every part the board's revisions may carry.")
    return {"board": board, "title": title, "mcu": mcu, "notes": notes, "groups": groups}


def describe_iomcu(io: Hwdef | None) -> dict:
    """IO MCU side: which IO MCU pin drives which MAIN output / input."""
    if io is None:
        return {"mcu": "STM32F1 IO MCU", "pins": [[f"MAIN {n}", "-", "PWM output"] for n in range(1, 9)]}
    rows = []
    pwm = []
    for pin in io.pins.values():
        m = next((re.match(r"PWM\((\d+)\)", a) for a in pin.args if a.startswith("PWM(")), None)
        if m:
            pwm.append((int(m.group(1)), pin))
    rows += [[f"MAIN {n}", p.port, p.label] for n, p in sorted(pwm, key=lambda x: x[0])]
    named = (("RCIN", "RC in (PPM/SBUS)"), ("USART1_RX", "DSM / Spektrum in"), ("USART3_TX", "SBUS out"),
             ("SAFETY_INPUT", "safety switch"), ("SAFETY_LED", "safety LED"), ("VSERVO", "servo rail sense"),
             ("VRSSI", "RSSI in"), ("HEATER", "IMU heater"), ("SPEKTRUM_PWR_EN", "Spektrum power"),
             ("USART2_TX", "link TX (to main MCU RX)"), ("USART2_RX", "link RX (from main MCU TX)"))
    for label, text in named:
        pin = io.label(label)
        if pin:
            rows.append([text, pin.port, label])
    for pin in io.pins.values():
        if "LED" in pin.label and pin.label != "SAFETY_LED" and "OUTPUT" in pin.args:
            color = led_color(pin)
            rows.append([f"{color + ' ' if color else ''}LED", pin.port, pin.label])
    return {"mcu": _mcu(io, "STM32F1"), "pins": rows}


# ---------------------------------------------------------------------------
# APX short format (platforms/stm32h7/boards/<board>/hwdef.dat)
# ---------------------------------------------------------------------------
def describe_apx(hw: Hwdef, board: str, title: str, iomcu_hw: Hwdef | None = None) -> dict:
    if hw.config.get("SERIAL_ORDER"):
        desc = describe_full(hw, board, title, iomcu_hw)
        desc["notes"] = ["APX board description (platforms/stm32h7/boards)."] + desc["notes"]
        return desc
    groups = []
    usb = (hw.label("OTG_FS_DM"), hw.label("OTG_FS_DP"))
    if all(usb):
        groups.append({"kind": "usb", "name": "USB (OTG1)", "side": "left",
                       "pins": [["D-", usb[0].port, "OTG_FS_DM"], ["D+", usb[1].port, "OTG_FS_DP"]],
                       "devices": [device("gcs", "Ground station (USB)", "MAVLink, firmware upload")]})
    for tok in hw.all("UART"):
        if len(tok) >= 3:
            n, tx, rx = tok[0], tok[1], tok[2]
            role = tok[4] if len(tok) >= 5 else ""
            baud = tok[3] if len(tok) >= 4 else ""
            dev = serial_device(role)
            dev["detail"] = f"{baud} baud" if baud else ""
            groups.append({"kind": "serial", "name": f"UART{n} · {role}", "side": "left",
                           "pins": [["TX", tx, f"USART{n}_TX"], ["RX", rx, f"USART{n}_RX"]], "devices": [dev]})
    for bus in sorted({p.label.split("_")[0] for p in hw.pins.values() if re.match(r"^SPI\d+_", p.label)}):
        rows = [[fn, hw.label(f"{bus}_{fn}").port, f"{bus}_{fn}"] for fn in ("SCK", "MISO", "MOSI", "NSS")
                if hw.label(f"{bus}_{fn}")]
        devs = [sensor_device(t[0], {}, f"CS {t[2]}") for t in hw.all("SPIDEV") if len(t) >= 3 and t[1] == bus]
        groups.append({"kind": "spi", "name": bus, "side": "right", "pins": rows,
                       "devices": devs or [device("port", "SPI connector")]})
    for bus in sorted({p.label.split("_")[0] for p in hw.pins.values() if re.match(r"^I2C\d+_", p.label)}):
        rows = [[fn, hw.label(f"{bus}_{fn}").port, f"{bus}_{fn}"] for fn in ("SCL", "SDA") if hw.label(f"{bus}_{fn}")]
        devs = [sensor_device(t[0], {}, f"address {t[2]}") for t in hw.all("I2CDEV") if len(t) >= 3 and t[1] == bus]
        groups.append({"kind": "i2c", "name": bus, "side": "right", "pins": rows,
                       "devices": devs or [device("port", "I2C connector")]})
    outs = [[f"M{t[0]}", t[3], f"TIM{t[1]}_CH{t[2]}"] for t in hw.all("MOTOR") if len(t) >= 4]
    outs += [[f"S{t[0]}", t[3], f"TIM{t[1]}_CH{t[2]} servo"] for t in hw.all("SERVO") if len(t) >= 4]
    if outs:
        groups.append({"kind": "pwm", "name": "Motor / servo outputs", "side": "right", "pins": outs, "devices": []})
    adc = adc_group(hw)
    if adc:
        groups.append(adc)
    extra = []
    for t in hw.all("LED"):
        if len(t) >= 2:
            pin = hw.pins.get(t[1])
            color = led_color(pin) if pin else ""
            extra.append(["LED", t[1], f"LED{t[0]} ({color})" if color else f"LED{t[0]}"])
    extra += [["relay", t[1], f"RELAY{t[0]}"] for t in hw.all("RELAY") if len(t) >= 2]
    gpio = gpio_group(hw, extra)
    if gpio:
        groups.append(gpio)
    return {"board": board, "title": title, "mcu": _mcu(hw), "notes": ["APX board description (short format)."],
            "groups": groups}


# ---------------------------------------------------------------------------
# Lookup for a build target
# ---------------------------------------------------------------------------
def load_for_target(target, repo_root: Path) -> dict | None:
    """Pin model for a target: board_pins.json next to its Zephyr board, else the
    APX hwdef in platforms/stm32h7/boards/ (plus iomcu.dat). None for host builds."""
    if target.kind == "host":
        return None
    for js in (repo_root / "platforms" / "zephyr" / "boards").glob(f"*/{target.board_key}/board_pins.json"):
        try:
            return json.loads(js.read_text())
        except (OSError, ValueError):
            pass
    folder = repo_root / "platforms" / "stm32h7" / "boards" / target.board_key
    if (folder / "hwdef.dat").exists():
        io = parse_hwdef(folder / "iomcu.dat") if (folder / "iomcu.dat").exists() else None
        return describe_apx(parse_hwdef(folder / "hwdef.dat"), target.board_key,
                            target.board_title or target.board_key, io)
    return None


def pin_table(desc: dict) -> list[list[str]]:
    """Flat rows: [port, function, group, connected to]."""
    rows = []
    for g in desc.get("groups", []):
        devices = "; ".join(device_text(d) for d in g.get("devices", []))
        for fn, port, detail in g.get("pins", []):
            rows.append([port, f"{fn} ({detail})" if detail and detail != fn else fn, g["name"], devices])
        if g.get("kind") == "iomcu":
            io = g["iomcu"]
            for fn, port, detail in io.get("pins", []):
                rows.append([f"IO:{port}", fn, f"IO MCU ({io.get('mcu', '')})", detail])
    return rows
