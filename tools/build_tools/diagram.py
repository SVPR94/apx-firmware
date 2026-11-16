# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Board architecture diagram.

Main MCU drawn as a chip in the middle with its pins on both edges. Every
peripheral group (UART, SPI, I2C, CAN, ADC, PWM, ...) is a block wired to the
pins it uses, and what is connected to it is drawn next to it as an icon card:
IMU, compass, barometer, power monitor, GPS, radio, RC receiver, motors, LEDs
in their real colour, buzzer, ... The IO MCU (when the board has one) is a
second chip on the far right with MAIN outputs and RC input.

layout(desc, scale) -> (shapes, width, height)
draw_canvas(canvas, shapes, on_hover)    Tk canvas (GUI)
to_svg(shapes, width, height)            same picture as an SVG file

Shapes are dicts: {"t": "rect"|"oval"|"poly"|"line"|"arc"|"text", ..., "info"}.
Everything is laid out at scale 1 and scaled at the end, so both back ends
draw exactly the same geometry.
"""

from __future__ import annotations

import math
import re
from xml.sax.saxutils import escape

FONT = "Helvetica"
BG = "#f4f6f9"
INK = "#1f2933"
MUTED = "#616e7c"
CHIP = "#263238"

COLORS = {
    "usb": "#546e7a", "serial": "#1e88e5", "rcin": "#e53935", "can": "#8d6e63", "adc": "#c0a000",
    "gpio": "#7e57c2", "iomcu": "#8e24aa", "spi": "#43a047", "i2c": "#00897b", "sdmmc": "#3949ab",
    "pwm": "#fb8c00",
}
KIND_LABEL = {
    "usb": "USB", "serial": "Serial / UART", "rcin": "RC input", "can": "CAN", "adc": "Power / ADC",
    "gpio": "LED / GPIO", "iomcu": "IO MCU link", "spi": "SPI", "i2c": "I2C", "sdmmc": "SD card bus",
    "pwm": "PWM outputs",
}
LEFT_ORDER = ("usb", "serial", "rcin", "can", "adc", "gpio")
RIGHT_ORDER = ("iomcu", "spi", "i2c", "sdmmc", "pwm")
ROW_ICON_KINDS = ("pwm", "adc", "gpio")      # one icon per pin row instead of device cards

LED_HEX = {"red": "#ff1744", "green": "#00e676", "blue": "#2979ff", "amber": "#ffab00",
           "yellow": "#ffea00", "orange": "#ff9100", "white": "#f5f5f5"}

MCU_INFO = (  # MCU name prefix -> (core, memory)
    ("STM32H743", "Cortex-M7 · 480 MHz", "2 MB flash · 1 MB RAM"),
    ("STM32H753", "Cortex-M7 · 480 MHz", "2 MB flash · 1 MB RAM"),
    ("STM32H757", "Cortex-M7 480 MHz + M4", "2 MB flash · 1 MB RAM"),
    ("STM32H755", "Cortex-M7 480 MHz + M4", "2 MB flash · 1 MB RAM"),
    ("STM32H750", "Cortex-M7 · 480 MHz", "128 KB flash · 1 MB RAM"),
    ("STM32H7A3", "Cortex-M7 · 280 MHz", "2 MB flash · 1.4 MB RAM"),
    ("STM32F405", "Cortex-M4F · 168 MHz", "1 MB flash · 192 KB RAM"),
    ("STM32F427", "Cortex-M4F · 180 MHz", "2 MB flash · 256 KB RAM"),
    ("STM32F103", "Cortex-M3 · 72 MHz", "64-128 KB flash · 20 KB RAM"),
    ("STM32F100", "Cortex-M3 · 24 MHz", "64 KB flash · 8 KB RAM"),
)


# ---------------------------------------------------------------------------
# Shape list
# ---------------------------------------------------------------------------
class Pic:
    def __init__(self):
        self.shapes = []

    def rect(self, x0, y0, x1, y1, fill="", outline="", w=1.0, r=0.0, info="", dash=None):
        self.shapes.append({"t": "rect", "x0": x0, "y0": y0, "x1": x1, "y1": y1, "fill": fill,
                            "outline": outline, "w": w, "r": r, "info": info, "dash": dash})

    def oval(self, x0, y0, x1, y1, fill="", outline="", w=1.0, info=""):
        self.shapes.append({"t": "oval", "x0": x0, "y0": y0, "x1": x1, "y1": y1, "fill": fill,
                            "outline": outline, "w": w, "info": info})

    def circle(self, cx, cy, r, fill="", outline="", w=1.0, info=""):
        self.oval(cx - r, cy - r, cx + r, cy + r, fill, outline, w, info)

    def poly(self, pts, fill="", outline="", w=1.0, info=""):
        self.shapes.append({"t": "poly", "pts": list(pts), "fill": fill, "outline": outline, "w": w, "info": info})

    def line(self, pts, color, w=1.0, info="", dash=None, arrow=False):
        pts = list(pts)
        self.shapes.append({"t": "line", "pts": pts, "color": color, "w": w, "info": info, "dash": dash})
        if arrow:
            (x0, y0), (x1, y1) = pts[-4:-2], pts[-2:]
            a = math.atan2(y1 - y0, x1 - x0)
            L, W = 3.5 + 1.5 * w, 2.2 + w
            bx, by = x1 - L * math.cos(a), y1 - L * math.sin(a)
            self.poly([x1, y1, bx + W * math.sin(a), by - W * math.cos(a),
                       bx - W * math.sin(a), by + W * math.cos(a)], color, color, 0.5, info)

    def arc(self, x0, y0, x1, y1, start, extent, color, w=1.0, info=""):
        self.shapes.append({"t": "arc", "x0": x0, "y0": y0, "x1": x1, "y1": y1, "start": start,
                            "extent": extent, "color": color, "w": w, "info": info})

    def text(self, x, y, text, anchor="w", size=10.0, weight="normal", color=INK, info=""):
        self.shapes.append({"t": "text", "x": x, "y": y, "text": str(text), "anchor": anchor, "size": size,
                            "weight": weight, "color": color, "info": info})

    def card(self, x0, y0, x1, y1, fill="#ffffff", outline="#d5dbe3", r=7.0, info="", shadow=True):
        if shadow:
            self.rect(x0 + 2, y0 + 3, x1 + 2, y1 + 3, "#dde3ea", "", 0, r)
        self.rect(x0, y0, x1, y1, fill, outline, 1, r, info)


def _tint(hex_color, amount=0.85):
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
    r, g, b = (int(c + (255 - c) * amount) for c in (r, g, b))
    return f"#{r:02x}{g:02x}{b:02x}"


def _shade(hex_color, amount=0.3):
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
    r, g, b = (int(c * (1 - amount)) for c in (r, g, b))
    return f"#{r:02x}{g:02x}{b:02x}"


def _fit(text, size, width):
    """Shorten text so it fits roughly into width pixels."""
    max_chars = max(4, int(width / (size * 0.56)))
    return text if len(text) <= max_chars else text[:max_chars - 1] + "…"


# ---------------------------------------------------------------------------
# Icons: drawn in a 32 x 32 design box at (x0, y0), scaled to `size`
# ---------------------------------------------------------------------------
def _chip_body(p, X, Y, u, x0, y0, x1, y1, label="", info="", fill=CHIP, ticks="tb"):
    if "t" in ticks or "b" in ticks:
        n = 4
        for i in range(n):
            xx = x0 + (x1 - x0) * (i + 0.5) / n
            if "t" in ticks:
                p.line([X(xx), Y(y0 - 2.5), X(xx), Y(y0)], "#9aa5b1", 1.4 * u, info)
            if "b" in ticks:
                p.line([X(xx), Y(y1), X(xx), Y(y1 + 2.5)], "#9aa5b1", 1.4 * u, info)
    if "l" in ticks or "r" in ticks:
        n = 3
        for i in range(n):
            yy = y0 + (y1 - y0) * (i + 0.5) / n
            if "l" in ticks:
                p.line([X(x0 - 2.5), Y(yy), X(x0), Y(yy)], "#9aa5b1", 1.4 * u, info)
            if "r" in ticks:
                p.line([X(x1), Y(yy), X(x1 + 2.5), Y(yy)], "#9aa5b1", 1.4 * u, info)
    p.rect(X(x0), Y(y0), X(x1), Y(y1), fill, _shade(fill, 0.3), 1, 2 * u, info)
    p.circle(X(x0 + 2.6), Y(y0 + 2.6), 0.9 * u, "#cfd8dc", "", 0, info)
    if label:
        p.text(X((x0 + x1) / 2), Y((y0 + y1) / 2 + 0.6), label, "center", 6.2 * u, "bold", "#ffffff", info)


def _blade(cx, cy, angle, length, width):
    a = math.radians(angle)
    ca, sa = math.cos(a), math.sin(a)
    pts = []
    for t, wd in ((-1.0, 0.0), (-0.75, 0.75), (-0.3, 1.0), (0.3, 1.0), (0.75, 0.75), (1.0, 0.0),
                  (0.75, -0.75), (0.3, -1.0), (-0.3, -1.0), (-0.75, -0.75)):
        px, py = t * length, wd * width
        pts += [cx + px * ca - py * sa, cy + px * sa + py * ca]
    return pts


def icon(p: Pic, name, x0, y0, size=32.0, color="", info=""):
    u = size / 32.0

    def X(v):
        return x0 + v * u

    def Y(v):
        return y0 + v * u

    if name == "imu":
        _chip_body(p, X, Y, u, 2, 9, 17, 25, "IMU", info)
        ox, oy = 22, 25
        p.line([X(ox), Y(oy), X(31), Y(oy)], "#e53935", 1.8 * u, info, arrow=True)
        p.line([X(ox), Y(oy), X(ox), Y(7)], "#1e88e5", 1.8 * u, info, arrow=True)
        p.line([X(ox), Y(oy), X(29.5), Y(17.5)], "#43a047", 1.8 * u, info, arrow=True)
        p.arc(X(17.5), Y(8.5), X(26.5), Y(14.5), 200, 280, "#1e88e5", 1.3 * u, info)
    elif name == "compass":
        p.circle(X(16), Y(16), 14 * u, "#ffffff", "#455a64", 1.6 * u, info)
        p.circle(X(16), Y(16), 11 * u, "", "#cfd8dc", 0.8 * u, info)
        for a in range(0, 360, 45):
            r0 = 11.5 if a % 90 else 10
            ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
            p.line([X(16 + r0 * ca), Y(16 - r0 * sa), X(16 + 13.2 * ca), Y(16 - 13.2 * sa)], "#78909c", 1 * u, info)
        p.poly([X(16), Y(4.5), X(19.4), Y(16), X(12.6), Y(16)], "#e53935", "#b71c1c", 0.6 * u, info)
        p.poly([X(12.6), Y(16), X(19.4), Y(16), X(16), Y(27.5)], "#b0bec5", "#78909c", 0.6 * u, info)
        p.circle(X(16), Y(16), 1.8 * u, "#37474f", "", 0, info)
    elif name == "baro":
        p.circle(X(16), Y(16), 14 * u, "#ffffff", "#455a64", 1.6 * u, info)
        p.arc(X(5.5), Y(5.5), X(26.5), Y(26.5), 225, -150, "#26a69a", 2.6 * u, info)
        p.arc(X(5.5), Y(5.5), X(26.5), Y(26.5), 75, -120, "#ef5350", 2.6 * u, info)
        a = math.radians(55)
        p.line([X(16), Y(16), X(16 + 9.5 * math.cos(a)), Y(16 - 9.5 * math.sin(a))], "#37474f", 1.8 * u, info)
        p.circle(X(16), Y(16), 2 * u, "#37474f", "", 0, info)
        p.text(X(16), Y(24), "hPa", "center", 5.2 * u, "bold", "#546e7a", info)
    elif name == "power":
        _chip_body(p, X, Y, u, 3, 6, 29, 26, "", info)
        p.poly([X(18.5), Y(7.5), X(10), Y(18), X(15.5), Y(18), X(13), Y(25), X(22), Y(14), X(16.5), Y(14),
                X(19.5), Y(7.5)], "#ffca28", "#f57f17", 0.6 * u, info)
    elif name == "battery":
        p.rect(X(2), Y(9), X(27), Y(24), "#ffffff", "#455a64", 1.6 * u, 2.5 * u, info)
        p.rect(X(27), Y(13), X(30.5), Y(20), "#455a64", "#455a64", 0.5, 1 * u, info)
        for i in range(3):
            p.rect(X(4.5 + i * 7.3), Y(11.5), X(10.3 + i * 7.3), Y(21.5), "#43a047", "", 0, 1 * u, info)
    elif name in ("current", "volt"):
        letter, col = ("A", "#e65100") if name == "current" else ("V", "#1565c0")
        p.circle(X(16), Y(16), 14 * u, "#ffffff", "#455a64", 1.6 * u, info)
        p.arc(X(6), Y(6), X(26), Y(26), 150, -120, "#b0bec5", 1.6 * u, info)
        p.line([X(16), Y(17), X(21.5), Y(9)], col, 1.4 * u, info)
        p.text(X(16), Y(22), letter, "center", 11 * u, "bold", col, info)
    elif name == "rssi":
        for i, h in enumerate((6, 11, 16, 22)):
            p.rect(X(4 + i * 6.5), Y(28 - h), X(8.5 + i * 6.5), Y(28), "#43a047" if i < 3 else "#cfd8dc",
                   "", 0, 1 * u, info)
    elif name == "airspeed":
        p.rect(X(9), Y(13.5), X(30), Y(18.5), "#90a4ae", "#546e7a", 1 * u, 2 * u, info)
        p.rect(X(24), Y(18.5), X(27), Y(28), "#90a4ae", "#546e7a", 1 * u, 0, info)
        for yy, x1 in ((8, 9), (16, 6.5), (24, 9)):
            p.line([X(1), Y(yy), X(x1), Y(yy)], "#29b6f6", 1.6 * u, info, arrow=True)
    elif name == "flow":
        p.oval(X(2), Y(8), X(30), Y(24), "#ffffff", "#455a64", 1.5 * u, info)
        p.circle(X(16), Y(16), 6.5 * u, "#5c6bc0", "", 0, info)
        p.circle(X(16), Y(16), 2.8 * u, "#1a237e", "", 0, info)
        p.circle(X(14), Y(14), 1.2 * u, "#ffffff", "", 0, info)
    elif name in ("fram", "flash", "chip"):
        _chip_body(p, X, Y, u, 3, 8, 29, 24, {"fram": "FRAM", "flash": "LOG", "chip": ""}[name], info)
    elif name == "sd":
        p.poly([X(9), Y(2), X(26), Y(2), X(26), Y(30), X(6), Y(30), X(6), Y(5)], "#1e88e5", "#0d47a1", 1 * u, info)
        for i in range(4):
            p.rect(X(10 + i * 4), Y(4), X(12.5 + i * 4), Y(10), "#ffd54f", "", 0, 0, info)
        p.text(X(16), Y(21), "SD", "center", 8 * u, "bold", "#ffffff", info)
    elif name == "osd":
        p.rect(X(2), Y(5), X(30), Y(24), CHIP, CHIP, 1, 2.5 * u, info)
        p.rect(X(4.5), Y(7.5), X(27.5), Y(21.5), "#4fc3f7", "", 0, 1 * u, info)
        p.text(X(16), Y(14.5), "OSD", "center", 7 * u, "bold", "#01579b", info)
        p.line([X(16), Y(24), X(16), Y(28)], CHIP, 2 * u, info)
        p.line([X(10), Y(28.5), X(22), Y(28.5)], CHIP, 2 * u, info)
    elif name == "gps":
        p.rect(X(12), Y(11), X(20), Y(21), "#b0bec5", "#546e7a", 1 * u, 1.5 * u, info)
        for xa, xb in ((1.5, 11), (21, 30.5)):
            p.rect(X(xa), Y(12.5), X(xb), Y(19.5), "#1e88e5", "#0d47a1", 0.8 * u, 0, info)
            p.line([X((xa + xb) / 2), Y(12.5), X((xa + xb) / 2), Y(19.5)], "#90caf9", 0.7 * u, info)
        p.line([X(16), Y(11), X(16), Y(6)], "#546e7a", 1.4 * u, info)
        p.circle(X(16), Y(5), 1.6 * u, "#ef5350", "", 0, info)
        p.arc(X(10), Y(19), X(22), Y(31), 240, 60, "#43a047", 1.4 * u, info)
        p.arc(X(6), Y(15), X(26), Y(35), 245, 50, "#43a047", 1.4 * u, info)
    elif name == "radio":
        p.rect(X(3), Y(14), X(23), Y(29), "#455a64", "#263238", 1 * u, 2.5 * u, info)
        p.circle(X(7.5), Y(18.5), 1.6 * u, "#00e676", "", 0, info)
        p.line([X(9), Y(24), X(19), Y(24)], "#90a4ae", 1.2 * u, info)
        p.line([X(19), Y(14), X(24), Y(3)], "#263238", 2.2 * u, info)
        p.circle(X(24), Y(3), 1.6 * u, "#263238", "", 0, info)
        p.arc(X(20), Y(-1), X(28), Y(7), -60, 120, "#1e88e5", 1.4 * u, info)
        p.arc(X(17), Y(-4), X(31), Y(10), -55, 110, "#1e88e5", 1.4 * u, info)
    elif name == "rc":
        p.line([X(25), Y(11), X(29), Y(2)], "#263238", 2 * u, info)
        p.rect(X(2), Y(10), X(30), Y(28), "#37474f", "#263238", 1 * u, 4 * u, info)
        for cx in (9, 23):
            p.circle(X(cx), Y(19), 4.5 * u, "#78909c", "#263238", 0.8 * u, info)
            p.circle(X(cx + 1), Y(18), 1.6 * u, "#eceff1", "", 0, info)
        p.circle(X(16), Y(14), 1.2 * u, "#ef5350", "", 0, info)
    elif name == "esc":
        p.rect(X(2), Y(7), X(25), Y(25), "#5d4037", "#3e2723", 1 * u, 2 * u, info)
        p.text(X(13.5), Y(16), "ESC", "center", 7 * u, "bold", "#ffffff", info)
        for yy, col in ((11, "#e53935"), (16, "#212121"), (21, "#fdd835")):
            p.line([X(25), Y(yy), X(31), Y(yy)], col, 1.6 * u, info)
    elif name == "gcs":
        p.rect(X(5), Y(5), X(27), Y(21), CHIP, CHIP, 1, 1.5 * u, info)
        p.rect(X(7), Y(7), X(25), Y(19), "#4fc3f7", "", 0, 0.5 * u, info)
        p.poly([X(9), Y(16), X(13), Y(12), X(16), Y(14.5), X(22), Y(9.5)], "", "#01579b", 1 * u, info)
        p.poly([X(2), Y(22), X(30), Y(22), X(28), Y(26), X(4), Y(26)], "#90a4ae", "#607d8b", 0.8 * u, info)
    elif name == "port":
        p.rect(X(3), Y(10), X(29), Y(22), "#ffffff", "#616161", 1.4 * u, 2 * u, info)
        for i in range(4):
            p.rect(X(6 + i * 5.8), Y(13.5), X(9.5 + i * 5.8), Y(18.5), "#424242", "", 0, 0, info)
    elif name == "can":
        p.line([X(1), Y(13), X(31), Y(13)], "#d84315", 2 * u, info)
        p.line([X(1), Y(19), X(31), Y(19)], "#5d4037", 2 * u, info)
        for xa, top in ((5, True), (18, False)):
            ya, yb = (2, 9) if top else (23, 30)
            p.rect(X(xa), Y(ya), X(xa + 9), Y(yb), "#8d6e63", "#4e342e", 0.8 * u, 1 * u, info)
            p.line([X(xa + 4.5), Y(yb if top else ya), X(xa + 4.5), Y(13 if top else 19)], "#4e342e", 1 * u, info)
        p.rect(X(0), Y(11.5), X(1.5), Y(20.5), "#4e342e", "", 0, 0, info)
        p.rect(X(30.5), Y(11.5), X(32), Y(20.5), "#4e342e", "", 0, 0, info)
    elif name == "motor":
        p.circle(X(16), Y(16), 14 * u, "#455a64", "#263238", 1.2 * u, info)
        p.circle(X(16), Y(16), 9.5 * u, "#78909c", "#37474f", 0.8 * u, info)
        for a in range(0, 360, 60):
            ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
            p.circle(X(16 + 11.8 * ca), Y(16 + 11.8 * sa), 0.9 * u, "#90a4ae", "", 0, info)
        p.poly(_blade(X(16), Y(16), -35, 15.5 * u, 2.6 * u), color or "#fb8c00", _shade(color or "#fb8c00"),
               0.7 * u, info)
        p.circle(X(16), Y(16), 3 * u, "#eceff1", "#37474f", 0.8 * u, info)
    elif name == "servo":
        p.rect(X(1), Y(15), X(31), Y(19), "#1565c0", "#0d47a1", 0.8 * u, 1 * u, info)
        p.rect(X(4), Y(11), X(28), Y(27), "#1e88e5", "#0d47a1", 1 * u, 2 * u, info)
        p.poly([X(19), Y(8.5), X(8), Y(4), X(7), Y(6.5), X(19), Y(13.5)], "#ffffff", "#78909c", 0.8 * u, info)
        p.circle(X(20), Y(11), 3.2 * u, "#ffffff", "#78909c", 0.8 * u, info)
    elif name == "led":
        col = color or LED_HEX["green"]
        p.oval(X(3), Y(-1), X(29), Y(25), _tint(col, 0.72), "", 0, info)
        for a in (30, 90, 150):
            ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
            p.line([X(16 + 10.5 * ca), Y(11 - 10.5 * sa), X(16 + 13.5 * ca), Y(11 - 13.5 * sa)],
                   _shade(col, 0.1), 1.2 * u, info)
        pts = []
        for i in range(13):
            a = math.radians(180 - i * 15)
            pts += [X(16 + 6.5 * math.cos(a)), Y(11 - 6.5 * math.sin(a))]
        pts += [X(22.5), Y(20), X(9.5), Y(20)]
        p.poly(pts, col, _shade(col, 0.35), 0.9 * u, info)
        p.rect(X(8), Y(19.5), X(24), Y(22.5), _shade(col, 0.15), _shade(col, 0.4), 0.7 * u, 0.8 * u, info)
        p.oval(X(11.5), Y(6.5), X(14), Y(12), "#ffffff", "", 0, info)
        p.line([X(13), Y(22.5), X(13), Y(31)], "#90a4ae", 1.4 * u, info)
        p.line([X(19), Y(22.5), X(19), Y(29)], "#90a4ae", 1.4 * u, info)
    elif name == "buzzer":
        p.circle(X(12), Y(16), 10 * u, CHIP, "#000000", 0.8 * u, info)
        p.circle(X(12), Y(16), 2.2 * u, "#eceff1", "", 0, info)
        for r in (13, 17.5):
            p.arc(X(12 - r), Y(16 - r), X(12 + r), Y(16 + r), -38, 76, "#455a64", 1.6 * u, info)
    elif name == "safety":
        p.rect(X(3), Y(3), X(29), Y(29), "#eceff1", "#616161", 1.2 * u, 3 * u, info)
        p.circle(X(16), Y(16), 8.5 * u, "#e53935", "#b71c1c", 1.2 * u, info)
        p.oval(X(12), Y(10.5), X(17), Y(14), "#ff8a80", "", 0, info)
    elif name == "switch":
        col = "#2e7d32"
        p.line([X(1), Y(22), X(8), Y(22)], "#455a64", 1.6 * u, info)
        p.line([X(24), Y(22), X(31), Y(22)], "#455a64", 1.6 * u, info)
        p.circle(X(9), Y(22), 2 * u, "#ffffff", "#455a64", 1.4 * u, info)
        p.circle(X(23), Y(22), 2 * u, "#ffffff", "#455a64", 1.4 * u, info)
        p.line([X(10.5), Y(21), X(22), Y(11)], col, 2.2 * u, info)
        p.poly([X(17), Y(2), X(13), Y(9), X(16), Y(9), X(14), Y(14), X(19), Y(7), X(16), Y(7)],
               "#ffca28", "#f57f17", 0.5 * u, info)
    elif name == "heater":
        pts = []
        for i, xx in enumerate((2, 5, 9, 13, 17, 21, 25, 28, 30)):
            pts += [X(xx), Y(24 if i in (0, 8) else (19 if i % 2 else 29))]
        p.line(pts, "#e53935", 1.8 * u, info)
        for xx in (9, 16, 23):
            p.line([X(xx), Y(15), X(xx + 2), Y(11.5), X(xx), Y(8), X(xx + 2), Y(4.5), X(xx), Y(1)], "#ff7043",
                   1.3 * u, info)
    elif name == "relay":
        p.rect(X(3), Y(6), X(29), Y(26), "#1565c0", "#0d47a1", 1 * u, 2 * u, info)
        p.text(X(16), Y(16), "RLY", "center", 7 * u, "bold", "#ffffff", info)
    else:
        _chip_body(p, X, Y, u, 3, 8, 29, 24, "", info)


# ---------------------------------------------------------------------------
# What a pin row / a device is drawn as
# ---------------------------------------------------------------------------
def _led_from_text(text):
    low = text.lower()
    return next((hexv for word, hexv in LED_HEX.items() if word in low), "")


def row_icon(kind, fn, detail):
    """(icon, label, colour) for a pin row of a pwm / adc / gpio group or an IO MCU row."""
    f, d = fn.lower(), (detail or "").lower()
    if kind == "pwm" or f.startswith("main "):
        if re.match(r"^s\d+$", f) or "servo" in d:
            return "servo", fn, ""
        if re.match(r"^m\d+$", f):
            return "motor", f"Motor {fn[1:]}", ""
        return "motor", fn, ""
    if kind == "adc":
        role = detail.split(": ", 1)[1] if ": " in detail else fn
        num = re.match(r"BATT(\d)", fn)
        if num:
            role = role.replace("battery", f"battery {num.group(1)}")
        icon_name = ("battery" if "voltage" in role and "battery" in role else
                     "current" if "current" in role else "rssi" if "rssi" in role.lower() else
                     "airspeed" if "airspeed" in role else "volt")
        return icon_name, role, ""
    if "led" in f:
        label = re.sub(r"\s*\((\w+)\)$", r" · \1", detail or fn)
        return "led", label, _led_from_text(f"{fn} {detail}") or ("#ff1744" if "safety" in f else "")
    for key, name in (("power", "switch"), ("buzzer", "buzzer"), ("safety", "safety"), ("heater", "heater"),
                      ("relay", "relay"), ("rc in", "rc"), ("dsm", "rc"), ("sbus out", "port"),
                      ("servo rail", "volt"), ("rssi", "rssi"), ("spektrum", "switch")):
        if key in f:
            if kind != "gpio":
                return name, fn, ""
            return name, detail if key.split()[0].upper() in detail.upper() or key == "power" else f"{fn} · {detail}", ""
    return "chip", fn, ""


def as_device(d):
    """Normalise a device (dict, or the older plain string) to (icon, name, role, detail)."""
    if isinstance(d, dict):
        return d.get("icon", "chip"), d.get("name", ""), d.get("role", ""), d.get("detail", "")
    low = d.lower()
    for key, name in (("imu", "imu"), ("baro", "baro"), ("compass", "compass"), ("fram", "fram"),
                      ("sd card", "sd"), ("osd", "osd"), ("flash", "flash"), ("gps", "gps"),
                      ("telemetry", "radio"), ("mavlink", "radio"), ("receiver", "rc"), ("rc input", "rc"),
                      ("esc", "esc"), ("usb", "gcs"), ("can", "can"), ("power", "power")):
        if key in low:
            m = re.match(r"^([^(]+?)\s*\((.+?)\)(?:,\s*(.*))?$", d)
            if m:
                return name, m.group(1), m.group(2), m.group(3) or ""
            return name, d, "", ""
    return "chip", d, "", ""


def _mcu_info(mcu):
    up = (mcu or "").upper()
    for prefix, core, mem in MCU_INFO:
        if up.startswith(prefix):
            return core, mem
    return "", ""


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------
M = 24.0             # margin
DW = 228.0           # device column width
DGAP = 46.0          # block -> device gap
BW = 236.0           # peripheral block width
GAP = 104.0          # block -> MCU wire length
MW = 236.0           # main MCU width
HDR = 26.0           # block header height
ROW = 20.0           # pin row
IROW = 34.0          # pin row with an icon
CARD = 48.0          # device card height
CGAP = 8.0
ICON = 30.0
PAD = 16.0           # between groups


def _summary(desc):
    groups = desc.get("groups", [])
    count, seen = {}, set()
    for g in groups:
        for d in g.get("devices", []):
            ic, name = as_device(d)[:2]
            if (ic, name) in seen:      # e.g. the external compass, probed on every external bus
                continue
            seen.add((ic, name))
            count[ic] = count.get(ic, 0) + 1
    io = next((g for g in groups if g["kind"] == "iomcu"), None)
    main = sum(1 for r in (io or {}).get("iomcu", {}).get("pins", []) if r[0].startswith("MAIN"))
    outs = sum(len(g["pins"]) for g in groups if g["kind"] == "pwm")
    badges = []
    for ic, word in (("imu", "IMU"), ("baro", "baro"), ("compass", "compass"), ("power", "power monitor")):
        if count.get(ic):
            badges.append((ic, f"{count[ic]} {word}"))
    n_ser = sum(1 for g in groups if g["kind"] == "serial")
    if n_ser:
        badges.append(("port", f"{n_ser} serial"))
    if main:
        badges.append(("motor", f"{main} MAIN (IO MCU) + {outs} AUX"))
    elif outs:
        badges.append(("motor", f"{outs} motor / servo outputs"))
    n_can = sum(1 for g in groups if g["kind"] == "can")
    if n_can:
        badges.append(("can", f"{n_can} CAN"))
    adc = next((g for g in groups if g["kind"] == "adc"), None)
    if adc and any("battery" in r[2] for r in adc["pins"]):
        badges.append(("battery", "analog battery V / A"))
    if count.get("sd"):
        badges.append(("sd", "microSD"))
    badges.append(("chip", "IO MCU" if io else "no IO MCU"))
    return badges


def _block(p, g, x0, y, side, color):
    """Peripheral block; returns (height, [(y, port, info)], row_height)."""
    rows = g.get("pins", [])
    rh = IROW if g["kind"] in ROW_ICON_KINDS else ROW
    h = HDR + len(rows) * rh + 6
    x1 = x0 + BW
    p.card(x0, y, x1, y + h, "#ffffff", _tint(color, 0.45), 7, g["name"])
    p.rect(x0, y, x1, y + HDR, color, color, 1, 7, g["name"])
    p.rect(x0, y + HDR - 7, x1, y + HDR, color, "", 0, 0, g["name"])
    p.text(x0 + 10, y + HDR / 2, _fit(g["name"], 11, BW - 20), "w", 11, "bold", "#ffffff", g["name"])
    out = []
    yy = y + HDR
    for i, (fn, port, detail) in enumerate(rows):
        cy = yy + rh / 2
        info = f"{port}  {fn}  {detail if detail != fn else ''}  ->  {g['name']}"
        if i % 2:
            p.rect(x0 + 1, yy, x1 - 1, yy + rh, _tint(color, 0.94), "", 0, 0, info)
        d = detail if detail and detail != fn else ""
        if g["kind"] == "adc":
            d = d.split(":")[0]           # the role is drawn next to the icon
        elif g["kind"] == "gpio":
            d = re.sub(r"\s*\(\w+\)$", "", d)
        if side == "left":
            p.text(x1 - 10, cy, _fit(fn, 10, BW * 0.55), "e", 10, "bold", INK, info)
            p.text(x0 + 10, cy, _fit(d, 9, BW * 0.45), "w", 9, "normal", MUTED, info)
        else:
            p.text(x0 + 10, cy, _fit(fn, 10, BW * 0.55), "w", 10, "bold", INK, info)
            p.text(x1 - 10, cy, _fit(d, 9, BW * 0.45), "e", 9, "normal", MUTED, info)
        out.append((cy, port, info))
        yy += rh
    return h, out, rh


def _device_card(p, x0, y, dev, color, info_prefix=""):
    ic, name, role, detail = as_device(dev)
    info = f"{info_prefix}{name}" + (f" - {role}" if role else "") + (f" ({detail})" if detail else "")
    p.card(x0, y, x0 + DW, y + CARD, "#ffffff", "#d5dbe3", 7, info)
    p.rect(x0, y + 6, x0 + 4, y + CARD - 6, color, "", 0, 2, info)
    p.rect(x0 + 9, y + 6, x0 + 9 + 36, y + 42, _tint(color, 0.9), "", 0, 6, info)
    icon(p, ic, x0 + 12, y + 9, 30, info=info)
    tx = x0 + 52
    p.text(tx, y + 14, _fit(name, 11, DW - 58), "w", 11, "bold", INK, info)
    if role:
        p.text(tx, y + 28, _fit(role, 9, DW - 58), "w", 9, "normal", "#3e4c59", info)
    if detail:
        p.text(tx, y + 40, _fit(detail, 8.5, DW - 58), "w", 8.5, "normal", MUTED, info)


def layout(desc: dict, scale: float = 1.0) -> tuple[list, float, float]:
    p = Pic()
    groups = desc.get("groups", [])
    left = sorted([g for g in groups if g["kind"] in LEFT_ORDER], key=lambda g: LEFT_ORDER.index(g["kind"]))
    right = sorted([g for g in groups if g["kind"] in RIGHT_ORDER], key=lambda g: RIGHT_ORDER.index(g["kind"]))
    io_group = next((g for g in right if g["kind"] == "iomcu"), None)

    ldx0 = M
    lbx0 = ldx0 + DW + DGAP
    mx0 = lbx0 + BW + GAP
    mx1 = mx0 + MW
    rbx0 = mx1 + GAP
    rdx0 = rbx0 + BW + DGAP
    width = rdx0 + DW + M

    # ---- header: title, MCU, summary badges, colour legend
    mcu = desc.get("mcu", "?")
    p.text(M, M + 10, desc.get("title", desc.get("board", "")), "w", 20, "bold", INK, desc.get("board", ""))
    core, mem = _mcu_info(mcu)
    io_mcu = io_group["iomcu"].get("mcu", "") if io_group else ""
    sub = f"Main MCU {mcu}" + (f"  ({core}, {mem})" if core else "")
    sub += f"   ·   IO MCU {io_mcu}" if io_group else "   ·   no IO MCU: every output is on the main MCU"
    p.text(M, M + 34, sub, "w", 11, "normal", MUTED, "")
    bx = M
    for ic, text in _summary(desc):
        w = 34 + len(text) * 6.4
        p.rect(bx, M + 48, bx + w, M + 72, "#ffffff", "#cfd6de", 1, 12, text)
        icon(p, ic, bx + 6, M + 52, 16, info=text)
        p.text(bx + 27, M + 60, text, "w", 10, "bold", "#3e4c59", text)
        bx += w + 8
    lx = M
    for kind in LEFT_ORDER + RIGHT_ORDER:
        if any(g["kind"] == kind for g in groups):
            p.line([lx, M + 90, lx + 18, M + 90], COLORS[kind], 4, KIND_LABEL[kind])
            p.text(lx + 23, M + 90, KIND_LABEL[kind], "w", 9.5, "normal", "#3e4c59", KIND_LABEL[kind])
            lx += 34 + len(KIND_LABEL[kind]) * 5.8
    top = M + 128

    # ---- columns
    def column(items, bx0, dx0, side):
        y = top
        wires, link_rows = [], None
        for g in items:
            color = COLORS.get(g["kind"], "#757575")
            h, rows, rh = _block(p, g, bx0, y, side, color)
            wires += [(cy, port, color, info) for cy, port, info in rows]
            used = h
            edge = bx0 if side == "left" else bx0 + BW
            if g["kind"] == "iomcu":
                link_rows = rows
            elif g["kind"] in ROW_ICON_KINDS:
                for (cy, port, info), (fn, _, detail) in zip(rows, g["pins"]):
                    ic, label, col = row_icon(g["kind"], fn, detail)
                    if side == "left":
                        ix = dx0 + DW - ICON
                        p.line([edge, cy, ix + ICON + 4, cy], color, 1.6, info)
                        p.text(ix - 8, cy, _fit(label, 10, DW - ICON - 10), "e", 10, "bold", INK, info)
                    else:
                        ix = dx0
                        p.line([edge, cy, ix - 4, cy], color, 1.6, info)
                        p.text(ix + ICON + 8, cy, _fit(label, 10, DW - ICON - 10), "w", 10, "bold", INK, info)
                    p.circle(edge, cy, 2.6, color, "#ffffff", 1, info)
                    icon(p, ic, ix, cy - ICON / 2, ICON, col, info)
            else:
                devs = g.get("devices", [])
                dh = len(devs) * (CARD + CGAP) - CGAP if devs else 0
                used = max(h, dh)
                by = y + min(h, max(HDR + 8, h / 2))
                tx = (edge + (dx0 + DW if side == "left" else dx0)) / 2
                for i, dev in enumerate(devs):
                    cy = y + i * (CARD + CGAP)
                    mid = cy + CARD / 2
                    card_edge = dx0 + DW if side == "left" else dx0
                    p.line([edge, by, tx, by, tx, mid, card_edge, mid], color, 2, g["name"])
                    _device_card(p, dx0, cy, dev, color, f"{g['name']}: ")
                if devs:
                    p.circle(edge, by, 3.2, color, "#ffffff", 1.2, g["name"])
            y += used + PAD
        return y, wires, link_rows

    ly, lwires, _ = column(left, lbx0, ldx0, "left")
    ry, rwires, link_rows = column(right, rbx0, rdx0, "right")
    bottom = max(ly, ry, top + 260) - PAD + 10

    # ---- main MCU chip
    ctop = top - 30
    p.rect(mx0 + 4, ctop + 5, mx1 + 4, bottom + 5, "#cfd6de", "", 0, 12)
    p.rect(mx0, ctop, mx1, bottom, CHIP, "#11181c", 2, 12, f"main MCU {mcu}")
    p.rect(mx0 + 8, ctop + 8, mx1 - 8, bottom - 8, "", "#37474f", 1, 8, f"main MCU {mcu}")
    p.circle(mx0 + 20, ctop + 20, 5, "#455a64", "", 0, "pin 1")
    for cy, port, color, info in lwires:
        p.line([lbx0 + BW, cy, mx0, cy], color, 2, info)
        p.circle(lbx0 + BW, cy, 2.6, color, "#ffffff", 1, info)
        p.rect(mx0 - 5, cy - 7, mx0 + 40, cy + 7, "#37474f", color, 1.2, 3, info)
        p.text(mx0 + 17, cy, port, "center", 9, "bold", "#eceff1", info)
    for cy, port, color, info in rwires:
        p.line([mx1, cy, rbx0, cy], color, 2, info)
        p.circle(rbx0, cy, 2.6, color, "#ffffff", 1, info)
        p.rect(mx1 - 40, cy - 7, mx1 + 5, cy + 7, "#37474f", color, 1.2, 3, info)
        p.text(mx1 - 17, cy, port, "center", 9, "bold", "#eceff1", info)
    cx, cmid = (mx0 + mx1) / 2, (ctop + bottom) / 2
    n = {k: sum(1 for g in groups if g["kind"] == k) for k in COLORS}
    stats = [f"{n['serial'] + n['usb']} serial · {n['spi']} SPI · {n['i2c']} I2C",
             f"{n['can']} CAN · {sum(len(g['pins']) for g in groups if g['kind'] == 'pwm')} PWM · "
             f"{sum(len(g['pins']) for g in groups if g['kind'] == 'adc')} ADC"]
    p.text(cx, cmid - 46, "MAIN MCU", "center", 10, "bold", "#78909c", "")
    p.text(cx, cmid - 24, mcu, "center", 17, "bold", "#ffffff", mcu)
    if core:
        p.text(cx, cmid - 4, core, "center", 10, "normal", "#b0bec5", "")
        p.text(cx, cmid + 12, mem, "center", 10, "normal", "#b0bec5", "")
    p.text(cx, cmid + 36, stats[0], "center", 9.5, "normal", "#90a4ae", "")
    p.text(cx, cmid + 52, stats[1], "center", 9.5, "normal", "#90a4ae", "")

    # ---- IO MCU chip, its MAIN outputs and inputs
    if io_group and link_rows:
        io = io_group["iomcu"]
        rows = io.get("pins", [])
        link = [r for r in rows if r[0].startswith("link")]
        others = [r for r in rows if not r[0].startswith("link")]
        color = COLORS["iomcu"]
        iw = 170.0
        ix0 = rdx0 + DW + 70
        ix1 = ix0 + iw
        itop = link_rows[0][0] - 64
        ibot = itop + 74 + len(others) * IROW
        p.rect(ix0 + 4, itop + 5, ix1 + 4, ibot + 5, "#d9cde0", "", 0, 12)
        p.rect(ix0, itop, ix1, ibot, "#311b3d", "#1a0f21", 2, 12, f"IO MCU {io.get('mcu', '')}")
        p.circle(ix0 + 16, itop + 16, 4.5, "#4a2a5c", "", 0, "pin 1")
        p.text((ix0 + ix1) / 2, itop + 18, "IO MCU", "center", 10, "bold", "#ce93d8", "IO coprocessor")
        p.text((ix0 + ix1) / 2, itop + 36, io.get("mcu", ""), "center", 13, "bold", "#ffffff", io.get("mcu", ""))
        icore, _ = _mcu_info(io.get("mcu", ""))
        if icore:
            p.text((ix0 + ix1) / 2, itop + 52, icore, "center", 9, "normal", "#ce93d8", "")
        # link: main TX -> IO RX, main RX <- IO TX
        lx0 = rbx0 + BW
        p.text((lx0 + ix0) / 2, link_rows[0][0] - 12, "UART · 1.5 Mbaud", "center", 9, "bold", color,
               "IO link: register protocol at 1.5 Mbaud (drivers/iomcu)")
        io_rx = next((r for r in link if r[0].startswith("link RX")), None)
        io_tx = next((r for r in link if r[0].startswith("link TX")), None)
        for (cy, port, info), io_row, toward_io in zip(link_rows, (io_rx, io_tx), (True, False)):
            label = io_row[1] if io_row else ""
            if toward_io:
                p.line([lx0, cy, ix0 - 2, cy], color, 2.4, info, arrow=True)
            else:
                p.line([ix0, cy, lx0 + 2, cy], color, 2.4, info, arrow=True)
            p.circle(lx0, cy, 2.8, color, "#ffffff", 1, info)
            if label:
                p.rect(ix0 - 4, cy - 7, ix0 + 36, cy + 7, "#4a2a5c", color, 1.2, 3, info)
                p.text(ix0 + 16, cy, label, "center", 9, "bold", "#f3e5f5", info)
        y = itop + 74
        tx = ix1 + 26
        for fn, port, detail in others:
            cy = y + IROW / 2
            info = f"IO MCU {port}  {fn}  ({detail})"
            ic, label, col = row_icon("iomcu", fn, detail)
            p.rect(ix1 - 40, cy - 7, ix1 + 4, cy + 7, "#4a2a5c", color, 1.2, 3, info)
            p.text(ix1 - 18, cy, port, "center", 9, "bold", "#f3e5f5", info)
            p.line([ix1 + 4, cy, tx, cy], color, 1.6, info)
            icon(p, ic, tx, cy - ICON / 2, ICON, col, info)
            p.text(tx + ICON + 8, cy - 5, label, "w", 10, "bold", INK, info)
            p.text(tx + ICON + 8, cy + 8, detail, "w", 8.5, "normal", MUTED, info)
            y += IROW
        width = max(width, tx + ICON + 8 + 150 + M)
        bottom = max(bottom, ibot + 10)

    yy = bottom + 22
    for note in desc.get("notes", []):
        p.text(M, yy, note, "w", 9.5, "normal", MUTED, "")
        yy += 16
    height = yy + M
    return _scaled(p.shapes, scale), width * scale, height * scale


def _scaled(shapes, s):
    if s == 1.0:
        return shapes
    out = []
    for sh in shapes:
        sh = dict(sh)
        for k in ("x0", "y0", "x1", "y1", "x", "y", "w", "r", "size"):
            if k in sh and sh[k] is not None:
                sh[k] = sh[k] * s
        if "pts" in sh:
            sh["pts"] = [v * s for v in sh["pts"]]
        out.append(sh)
    return out


# ---------------------------------------------------------------------------
# Back ends
# ---------------------------------------------------------------------------
def _round_rect_pts(x0, y0, x1, y1, r, steps=5):
    r = max(0.0, min(r, (x1 - x0) / 2, (y1 - y0) / 2))
    pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for i in range(steps + 1):
            a = math.radians(a0 + 90 * i / steps)
            pts += [cx + r * math.cos(a), cy + r * math.sin(a)]
    return pts


def draw_canvas(canvas, shapes, on_hover=None):
    canvas.delete("all")
    canvas.configure(background=BG)
    for sh in shapes:
        t = sh["t"]
        w = max(0.0, sh.get("w", 1.0))
        if t == "rect":
            outline = sh["outline"] if w > 0 else ""
            if sh["r"] > 0.5:
                item = canvas.create_polygon(_round_rect_pts(sh["x0"], sh["y0"], sh["x1"], sh["y1"], sh["r"]),
                                             fill=sh["fill"], outline=outline, width=w)
            else:
                item = canvas.create_rectangle(sh["x0"], sh["y0"], sh["x1"], sh["y1"], fill=sh["fill"],
                                               outline=outline, width=w if outline else 0)
            if sh.get("dash"):
                canvas.itemconfigure(item, dash=sh["dash"])
        elif t == "oval":
            item = canvas.create_oval(sh["x0"], sh["y0"], sh["x1"], sh["y1"], fill=sh["fill"],
                                      outline=sh["outline"] if w > 0 else "", width=w)
        elif t == "poly":
            item = canvas.create_polygon(sh["pts"], fill=sh["fill"], outline=sh["outline"] if w > 0 else "",
                                         width=w)
        elif t == "line":
            item = canvas.create_line(sh["pts"], fill=sh["color"], width=max(0.6, w), capstyle="round",
                                      joinstyle="round")
            if sh.get("dash"):
                canvas.itemconfigure(item, dash=sh["dash"])
        elif t == "arc":
            item = canvas.create_arc(sh["x0"], sh["y0"], sh["x1"], sh["y1"], start=sh["start"],
                                     extent=sh["extent"], style="arc", outline=sh["color"], width=max(0.6, w))
        else:
            weight = "bold" if sh["weight"] == "bold" else "normal"
            slant = "italic" if sh["weight"] == "italic" else "roman"
            item = canvas.create_text(sh["x"], sh["y"], text=sh["text"], anchor=sh["anchor"]
                                      if sh["anchor"] != "center" else "center",
                                      font=(FONT, -max(1, int(round(sh["size"]))), weight, slant), fill=sh["color"])
        if sh.get("info") and on_hover:
            canvas.tag_bind(item, "<Enter>", lambda e, i=sh["info"]: on_hover(i))


def to_svg(shapes, width, height) -> str:
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
           f'viewBox="0 0 {width:.0f} {height:.0f}" font-family="Helvetica, Arial, sans-serif">',
           f'<rect width="100%" height="100%" fill="{BG}"/>']

    def paint(fill, outline, w):
        f = f'fill="{fill}"' if fill else 'fill="none"'
        s = f' stroke="{outline}" stroke-width="{w:.2f}"' if outline and w > 0 else ""
        return f + s

    for sh in shapes:
        t = sh["t"]
        dash = f' stroke-dasharray="{" ".join(str(d) for d in sh["dash"])}"' if sh.get("dash") else ""
        if t == "rect":
            out.append(f'<rect x="{sh["x0"]:.1f}" y="{sh["y0"]:.1f}" width="{sh["x1"] - sh["x0"]:.1f}" '
                       f'height="{sh["y1"] - sh["y0"]:.1f}" rx="{sh["r"]:.1f}" '
                       f'{paint(sh["fill"], sh["outline"], sh["w"])}{dash}></rect>')
        elif t == "oval":
            cx, cy = (sh["x0"] + sh["x1"]) / 2, (sh["y0"] + sh["y1"]) / 2
            out.append(f'<ellipse cx="{cx:.1f}" cy="{cy:.1f}" rx="{(sh["x1"] - sh["x0"]) / 2:.2f}" '
                       f'ry="{(sh["y1"] - sh["y0"]) / 2:.2f}" {paint(sh["fill"], sh["outline"], sh["w"])}>'
                       f'</ellipse>')
        elif t == "poly":
            pts = " ".join(f"{sh['pts'][i]:.1f},{sh['pts'][i + 1]:.1f}" for i in range(0, len(sh["pts"]), 2))
            out.append(f'<polygon points="{pts}" {paint(sh["fill"], sh["outline"], sh["w"])} '
                       f'stroke-linejoin="round"></polygon>')
        elif t == "line":
            pts = " ".join(f"{sh['pts'][i]:.1f},{sh['pts'][i + 1]:.1f}" for i in range(0, len(sh["pts"]), 2))
            out.append(f'<polyline points="{pts}" fill="none" stroke="{sh["color"]}" stroke-width="{sh["w"]:.2f}" '
                       f'stroke-linecap="round" stroke-linejoin="round"{dash}></polyline>')
        elif t == "arc":
            cx, cy = (sh["x0"] + sh["x1"]) / 2, (sh["y0"] + sh["y1"]) / 2
            rx, ry = (sh["x1"] - sh["x0"]) / 2, (sh["y1"] - sh["y0"]) / 2
            a0, a1 = math.radians(sh["start"]), math.radians(sh["start"] + sh["extent"])
            x0, y0 = cx + rx * math.cos(a0), cy - ry * math.sin(a0)
            x1, y1 = cx + rx * math.cos(a1), cy - ry * math.sin(a1)
            large = 1 if abs(sh["extent"]) > 180 else 0
            sweep = 0 if sh["extent"] > 0 else 1
            out.append(f'<path d="M {x0:.2f} {y0:.2f} A {rx:.2f} {ry:.2f} 0 {large} {sweep} {x1:.2f} {y1:.2f}" '
                       f'fill="none" stroke="{sh["color"]}" stroke-width="{sh["w"]:.2f}" stroke-linecap="round">'
                       f'</path>')
        else:
            a = {"w": "start", "e": "end", "center": "middle"}[sh["anchor"]]
            style = (' font-weight="bold"' if sh["weight"] == "bold" else
                     ' font-style="italic"' if sh["weight"] == "italic" else "")
            out.append(f'<text x="{sh["x"]:.1f}" y="{sh["y"] + sh["size"] * 0.35:.1f}" font-size="{sh["size"]:.1f}" '
                       f'fill="{sh["color"]}" text-anchor="{a}"{style}>{escape(sh["text"])}</text>')
        if sh.get("info"):
            out[-1] = _with_title(out[-1], sh["info"])
    out.append("</svg>")
    return "\n".join(out) + "\n"


def _with_title(element, info):
    return f"<g><title>{escape(info)}</title>{element}</g>" if info else element
