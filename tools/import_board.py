#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Create APX Zephyr board definitions from hwdef hardware descriptions.

A hwdef description (hwdef.dat, hwdef-bl.dat, with include/undef) lists a
flight controller's MCU, crystal, flash layout, board id, USB ids and pin
functions. This tool takes only those hardware facts and writes a complete
APX board for the Zephyr port:

  platforms/zephyr/boards/<vendor>/<board>/   board.yml, Kconfig, .dts, defconfig,
                                              board.cmake, support/openocd.cfg
  platforms/zephyr/app/boards/<board>.conf         USB CDC link (firmware)
  platforms/zephyr/bootloader/boards/<board>.conf  USB CDC link (APX bootloader)

The devicetree describes what the APX firmware uses today (see
platforms/zephyr/app/src/board_zephyr.cpp): clocks, status LED, power-rail
enables, USB CDC as the GCS link, the IO coprocessor UART (boards whose main
PWM outputs and RC input are handled by a separate IO MCU), flash partitions
for the bootloader, the .apj board id and the boot-control RAM block.
Every pin is checked against Zephyr's pin tables for the MCU package.

  python3 tools/import_board.py --hwdef-root <dir with one folder per board>        # all curated boards
  python3 tools/import_board.py --hwdef-root <dir> --board matek_h743 --board pixhawk6x
  python3 tools/import_board.py --list
"""

import argparse
import re
import sys
import json
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ZEPHYR = ROOT / "platforms" / "zephyr"
WORKSPACE = ZEPHYR / "workspace"

# ---------------------------------------------------------------------------
# Curated boards: popular, in production, STM32H7 or STM32F405 (1 MB flash).
# key: (hwdef folder, vendor folder, title, board id)
# Board ids are the numbers the boards' bootloaders check (.apj board_id).
# ---------------------------------------------------------------------------
BOARDS = {
    # STM32H7 with an IO coprocessor (main outputs + RC input on the IO MCU)
    "cube_orange":       ("CubeOrange", "cubepilot", "CubePilot Cube Orange", 140),
    "pixhawk6x":         ("Pixhawk6X", "holybro", "Holybro Pixhawk 6X", 53),
    "pixhawk6c":         ("Pixhawk6C", "holybro", "Holybro Pixhawk 6C", 56),
    "durandal":          ("Durandal", "holybro", "Holybro Durandal", 139),
    "siyi_n7":           ("SIYI_N7", "siyi", "SIYI N7", 1123),
    # STM32H7 without an IO coprocessor
    "matek_h743":        ("MatekH743", "mateksys", "Matek H743 (Wing/Slim/Mini)", 1013),
    "kakute_h7":         ("KakuteH7", "holybro", "Holybro Kakute H7", 1048),
    "kakute_h7_mini":    ("KakuteH7Mini", "holybro", "Holybro Kakute H7 Mini", 1058),
    "cuav_nora":         ("CUAV-Nora", "cuav", "CUAV Nora", 1009),
    "cuav_x7":           ("CUAV-X7", "cuav", "CUAV X7 / X7 Pro", 1010),
    "cuav_7_nano":       ("CUAV-7-Nano", "cuav", "CUAV 7-Nano", 7000),
    "ark_v6x":           ("ARKV6X", "ark", "ARK V6X", 57),
    "micoair_h743":      ("MicoAir743", "micoair", "MicoAir H743", 1166),
    # STM32F405 (1 MB flash)
    "matek_f405_wing":   ("MatekF405-Wing", "mateksys", "Matek F405-Wing", 127),
    "matek_f405_te":     ("MatekF405-TE", "mateksys", "Matek F405-TE", 1054),
    "speedybee_f405_v4": ("speedybeef4v4", "speedybee", "SpeedyBee F405 V4", 1136),
    "speedybee_f405_wing": ("SpeedyBeeF405WING", "speedybee", "SpeedyBee F405 Wing", 1106),
}

# Hand-written APX boards that only get board_pins.json (pin model for the
# GUI board diagram): key -> (hwdef folder, vendor folder, title)
PINS_ONLY = {"cube_orange_plus": ("CubeOrangePlus", "cubepilot", "CubePilot Cube Orange+")}

# Boards sold with an .apj bootloader already installed (Pixhawk-class). The
# others (FPV / wing boards) usually ship with other firmware and get the APX
# bootloader once, over USB DFU or SWD.
FACTORY_APJ_BOOTLOADER = {"cube_orange", "pixhawk6x", "pixhawk6c", "durandal", "siyi_n7",
                          "cuav_nora", "cuav_x7", "cuav_7_nano", "ark_v6x"}

# Per MCU: Zephyr SoC, includes, clocks.
FAMILIES = {
    "STM32H743xx": dict(soc="stm32h743xx", kconfig="SOC_STM32H743XX", qualifier=None,
                        dtsi="st/h7/stm32h743Xi.dtsi", pinctrl="st/h7/stm32h743xihx-pinctrl.dtsi",
                        series="h7", jlink="STM32H743ZI", pyocd="stm32h743xx", openocd="target/stm32h7x.cfg",
                        supply="ldo", ram_ctl=0x2001FC00, ram_node="dtcm", ram_base=0x20000000, ram_kb=128,
                        flash_kb=2048, app_kb=896),
    "STM32F405xx": dict(soc="stm32f405xx", kconfig="SOC_STM32F405XX", qualifier=None,
                        dtsi="st/f4/stm32f405Xg.dtsi", pinctrl="st/f4/stm32f405rgtx-pinctrl.dtsi",
                        series="f4", jlink="STM32F405RG", pyocd="stm32f405rgtx", openocd="target/stm32f4x.cfg",
                        supply=None, ram_ctl=0x1000FC00, ram_node="ccm0", ram_base=0x10000000, ram_kb=64,
                        flash_kb=1024, app_kb=None),
}


# ---------------------------------------------------------------------------
# hwdef parsing: shared with the GUI's board diagram (tools/build_tools/board_desc.py)
# ---------------------------------------------------------------------------
sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_tools.board_desc import PIN_RE, describe_full, parse_hwdef as parse  # noqa: E402

def number(text, default=None):
    try:
        return int(str(text).split()[0], 0)
    except (ValueError, IndexError):
        return default


def pin_ref(pin):
    m = PIN_RE.match(pin.port)
    return f"&gpio{m.group(1).lower()} {int(m.group(2))}"


def pinctrl_labels(fam):
    hits = list(WORKSPACE.glob(f"modules/hal/stm32/dts/{fam['pinctrl']}"))
    if not hits:
        raise SystemExit(f"Zephyr pinctrl table {fam['pinctrl']} not found - run platforms/zephyr/setup_zephyr.sh")
    # entries look like "/omit-if-no-ref/ usart6_tx_pc6: usart6_tx_pc6 {"
    return set(re.findall(r"(\w+):\s*\w+\s*\{", hits[0].read_text()))


# ---------------------------------------------------------------------------
# Board model
# ---------------------------------------------------------------------------
@dataclass
class Board:
    name: str
    vendor: str
    title: str
    board_id: int
    mcu: str
    fam: dict
    hse_mhz: int
    reserve_kb: int
    led: tuple | None = None          # (pin, active_low, open_drain)
    rails: list = field(default_factory=list)   # [(pin, active_low)]
    iomcu: tuple | None = None        # (uart node, tx label, rx label)
    usb: dict = field(default_factory=dict)
    bl_usb: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)


LED_ORDER = ("LED_BOOTLOADER", "LED_ACTIVITY", "LED0", "LED1", "LED_GREEN", "LED_BLUE", "LED_RED")
RAIL_RE = re.compile(r"^(n?)VDD_\w*_EN\d*$")


def usb_ids(hw):
    out = {}
    for key, name in (("USB_VENDOR", "vid"), ("USB_PRODUCT", "pid")):
        v = number(hw.config.get(key))
        if v is not None:
            out[name] = v
    if "USB_STRING_PRODUCT" in hw.config:
        out["product"] = hw.config["USB_STRING_PRODUCT"].strip('"')
    if "USB_STRING_MANUFACTURER" in hw.config:
        out["manufacturer"] = hw.config["USB_STRING_MANUFACTURER"].strip('"')
    return out


def build_board(key, hwdef_root):
    folder, vendor, title, board_id = BOARDS[key]
    bdir = Path(hwdef_root) / folder
    hw = parse(bdir / "hwdef.dat")
    mcu = hw.config.get("MCU", "").split()[-1] if hw.config.get("MCU") else ""
    if mcu not in FAMILIES:
        raise ValueError(f"{key}: MCU {mcu or '?'} not supported by the importer")
    fam = FAMILIES[mcu]
    b = Board(key, vendor, title, board_id, mcu, fam,
              hse_mhz=number(hw.config.get("OSCILLATOR_HZ"), 8000000) // 1000000,
              reserve_kb=number(hw.config.get("FLASH_RESERVE_START_KB"), 0))
    if board_id is None:
        raise ValueError(f"{key}: no numeric board id")
    labels = pinctrl_labels(fam)

    # LED
    led_names = LED_ORDER + tuple(sorted(p.label for p in hw.pins.values() if p.label.startswith("LED")))
    led_names += tuple(sorted(p.label for p in hw.pins.values() if "LED" in p.label and p.label not in led_names))
    for name in led_names:
        pin = hw.label(name)
        if pin and "OUTPUT" in pin.args:
            active_high = hw.defines.get("HAL_LED_ON", "0").strip() == "1"
            b.led = (pin, not active_high, "OPENDRAIN" in pin.args)
            break
    # Power rails switched on at boot (initial level = enabled level)
    rails = []
    for pin in hw.pins.values():
        m = RAIL_RE.match(pin.label)
        if not m or "OUTPUT" not in pin.args:
            continue
        active_low = m.group(1) == "n"
        initial_low = "LOW" in pin.args
        if initial_low == active_low:
            rails.append((pin, active_low))
    rank = lambda r: (0 if "PERIPH" in r[0].label or "5V" in r[0].label else 2 if "SENS" in r[0].label else 1)
    b.rails = sorted(rails, key=rank)
    # USB
    dm, dp = hw.label("OTG_FS_DM"), hw.label("OTG_FS_DP")
    if not (dm and dp and dm.port == "PA11" and dp.port == "PA12"):
        raise ValueError(f"{key}: USB OTG FS on PA11/PA12 not found")
    # IO coprocessor
    io = hw.config.get("IOMCU_UART")
    if io:
        uart = io.split()[0].upper()
        tx, rx = hw.label(f"{uart}_TX"), hw.label(f"{uart}_RX")
        if not (tx and rx):
            raise ValueError(f"{key}: IO coprocessor UART {uart} pins not found")
        node = uart.lower()
        tx_l, rx_l = f"{node}_tx_{tx.port.lower()}", f"{node}_rx_{rx.port.lower()}"
        missing = [l for l in (tx_l, rx_l) if l not in labels]
        if missing:
            raise ValueError(f"{key}: pins {missing} not in Zephyr pin table")
        b.iomcu = (node, tx_l, rx_l)
    b.usb = usb_ids(hw)
    bl = bdir / "hwdef-bl.dat"
    if bl.exists():
        b.bl_usb = usb_ids(parse(bl))
    # Flash layout
    if fam["series"] == "h7" and b.reserve_kb != 128:
        raise ValueError(f"{key}: unusual flash reserve {b.reserve_kb} KB (expected 128)")
    if fam["series"] == "f4" and b.reserve_kb not in (48, 64):
        raise ValueError(f"{key}: unusual flash reserve {b.reserve_kb} KB")
    if fam["series"] == "f4":
        b.reserve_kb = 64  # sectors 0-3: APX bootloader; firmware from sector 4
    if b.led is None:
        b.notes.append("no status LED found")
    return b


# ---------------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------------
GEN = "Generated by tools/import_board.py from the board's hardware description; re-run it rather than editing."


def clocks(b):
    if b.fam["series"] == "h7":
        mhz = b.hse_mhz
        m, ref = ((mhz // 8, 8) if mhz % 8 == 0 else (mhz // 5, 5) if mhz % 5 == 0 else (mhz // 4, 4))
        n = 800 // ref
        return f"""&clk_hse {{
	clock-frequency = <DT_FREQ_M({mhz})>;
	status = "okay";
}};

/* USB 48 MHz */
&clk_hsi48 {{
	status = "okay";
}};

/* {mhz} MHz / {m} = {ref} MHz PLL input, * {n} = 800 MHz VCO, / 2 = 400 MHz SYSCLK (VOS1) */
&pll {{
	div-m = <{m}>;
	mul-n = <{n}>;
	div-p = <2>;
	div-q = <10>;
	div-r = <2>;
	clocks = <&clk_hse>;
	status = "okay";
}};

&rcc {{
	clocks = <&pll>;
	clock-frequency = <DT_FREQ_M(400)>;
	d1cpre = <1>;
	hpre = <2>;
	d1ppre = <2>;
	d2ppre1 = <2>;
	d2ppre2 = <2>;
	d3ppre = <2>;
}};

&pwr {{
	power-supply = "{b.fam['supply']}";
}};
"""
    mhz = b.hse_mhz
    m, n = (mhz // 2, 168) if mhz % 2 == 0 else (mhz, 336)
    return f"""&clk_hse {{
	clock-frequency = <DT_FREQ_M({mhz})>;
	status = "okay";
}};

/* {mhz} MHz / {m} * {n} = 336 MHz VCO; / 2 = 168 MHz SYSCLK, / 7 = 48 MHz USB */
&pll {{
	div-m = <{m}>;
	mul-n = <{n}>;
	div-p = <2>;
	div-q = <7>;
	clocks = <&clk_hse>;
	status = "okay";
}};

&rcc {{
	clocks = <&pll>;
	clock-frequency = <DT_FREQ_M(168)>;
	ahb-prescaler = <1>;
	apb1-prescaler = <4>;
	apb2-prescaler = <2>;
}};
"""


def dts(b):
    f = b.fam
    io_line = f"\t\tapx,iomcu-uart = &{b.iomcu[0]};\n" if b.iomcu else ""
    dtcm_line = "\t\tzephyr,dtcm = &dtcm;\n" if f["series"] == "h7" else ""
    led = ""
    alias = ""
    if b.led:
        pin, low, od = b.led
        flags = "GPIO_ACTIVE_LOW" if low else "GPIO_ACTIVE_HIGH"
        if od:
            flags = f"({flags} | GPIO_OPEN_DRAIN)"
        led = f"""
	leds {{
		compatible = "gpio-leds";
		status_led: status_led {{
			gpios = <{pin_ref(pin)} {flags}>;
			label = "{pin.label}";
		}};
	}};
"""
        alias = "\n\taliases {\n\t\tled0 = &status_led;\n\t};\n"
    rails = ""
    if b.rails:
        lst = ",\n\t\t\t\t     ".join(f"<{pin_ref(p)} {'GPIO_ACTIVE_LOW' if low else 'GPIO_ACTIVE_HIGH'}>"
                                     for p, low in b.rails)
        names = ", ".join(p.label for p, _ in b.rails)
        rails = f"\t\t/* {names} */\n\t\tpower-enable-gpios = {lst};\n\t\tpower-enable-delay-ms = <20>;\n"
    app_kb = f["app_kb"] or (f["flash_kb"] - b.reserve_kb)
    ctl = f["ram_ctl"]
    ram_kb = f["ram_kb"] - 1
    factory = ("\t\t/* sold with an .apj bootloader: the APX bootloader is optional */\n"
               "\t\tfactory-apj-bootloader;\n") if b.name in FACTORY_APJ_BOOTLOADER else ""
    io_node = ""
    if b.iomcu:
        node, tx, rx = b.iomcu
        fifo = "\tfifo-enable;\n" if f["series"] == "h7" else ""
        io_node = f"""
/* IO coprocessor link: main PWM outputs, RC input and safety switch are on
 * the IO MCU (drivers/iomcu). */
&{node} {{
	pinctrl-0 = <&{tx} &{rx}>;
	pinctrl-names = "default";
	current-speed = <1500000>;
{fifo}	status = "okay";
}};
"""
    return f"""/*
 * SPDX-License-Identifier: CC-BY-NC-4.0
 * Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
 * Author: Savita Pawar (SVPR94)
 * Created: 2026
 *
 * {b.title} ({b.mcu}).
 * {GEN}
 */
/dts-v1/;
#include <{f['dtsi']}>
#include <{f['pinctrl']}>
#include <zephyr/dt-bindings/gpio/gpio.h>

/ {{
	model = "{b.title}";
	compatible = "apx,{b.name.replace('_', '-')}";

	chosen {{
		zephyr,sram = &sram0;
{dtcm_line}		zephyr,flash = &flash0;
		zephyr,code-partition = &app_partition;
		apx,gcs-uart = &cdc_acm_uart0;
{io_line}	}};
{led}{alias}
	/* APX board properties - see platforms/zephyr/app/src/board_zephyr.cpp */
	zephyr,user {{
{rails}		apj-board-id = <{b.board_id}>;
{factory}	}};

	/* APX boot control (bootloader <-> firmware handshake), outside every
	 * image's RAM: the last 1 KB of {f['ram_node']}. */
	apx_boot_control: boot-control@{ctl:x} {{
		reg = <0x{ctl:08x} DT_SIZE_K(1)>;
	}};
}};

{clocks(b)}
&{f['ram_node']} {{
	reg = <0x{f['ram_base']:08x} DT_SIZE_K({ram_kb})>;
}};

&flash0 {{
	partitions {{
		compatible = "fixed-partitions";
		#address-cells = <1>;
		#size-cells = <1>;

		bootloader_partition: partition@0 {{
			label = "bootloader";
			reg = <0x00000000 DT_SIZE_K({b.reserve_kb})>;
			read-only;
		}};
		app_partition: partition@{b.reserve_kb * 1024:x} {{
			label = "apx-firmware";
			reg = <0x{b.reserve_kb * 1024:08x} DT_SIZE_K({app_kb})>;
		}};
	}};
}};

zephyr_udc0: &usbotg_fs {{
	pinctrl-0 = <&usb_otg_fs_dm_pa11 &usb_otg_fs_dp_pa12>;
	pinctrl-names = "default";
	status = "okay";

	cdc_acm_uart0: cdc_acm_uart0 {{
		compatible = "zephyr,cdc-acm-uart";
		label = "APX MAVLink";
	}};
}};
{io_node}"""


def usb_conf(b, ids, product, what):
    lines = ["# SPDX-License-Identifier: CC-BY-NC-4.0", f"# {b.title}: {what} link over USB CDC-ACM. {GEN}",
             "CONFIG_UART_LINE_CTRL=y", "CONFIG_USB_DEVICE_STACK_NEXT=y", "CONFIG_USBD_CDC_ACM_CLASS=y",
             "CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT=y", 'CONFIG_CDC_ACM_SERIAL_MANUFACTURER_STRING="APX"',
             f'CONFIG_CDC_ACM_SERIAL_PRODUCT_STRING="{product}"']
    if "vid" in ids and "pid" in ids:
        lines += ["# Board's USB ids, so ground stations recognise it",
                  f"CONFIG_CDC_ACM_SERIAL_VID=0x{ids['vid']:04X}", f"CONFIG_CDC_ACM_SERIAL_PID=0x{ids['pid']:04X}"]
    return "\n".join(lines) + "\n"


def iomcu_hwdef(hw, hwdef_root):
    """The IO MCU's own description (its pin map), picked by the board's IO firmware."""
    romfs = " ".join(" ".join(tok) for tok in hw.all("ROMFS"))
    for folder in (("iomcu-f103",) if "f103" in romfs else ()) + ("iomcu",):
        path = Path(hwdef_root) / folder / "hwdef.dat"
        if path.exists():
            return parse(path)
    return None


def pin_model(key, folder, title, hwdef_root):
    hw = parse(Path(hwdef_root) / folder / "hwdef.dat")
    io = iomcu_hwdef(hw, hwdef_root) if hw.config.get("IOMCU_UART") else None
    desc = describe_full(hw, key, title, io)
    desc["source"] = "generated by tools/import_board.py from the board's hardware description"
    return desc


def write_pins(key, vendor, desc, dry=False):
    path = ZEPHYR / "boards" / vendor / key / "board_pins.json"
    if not dry:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(desc, indent=1) + "\n")
    return path


def write_board(b, dry=False):
    f = b.fam
    d = ZEPHYR / "boards" / b.vendor / b.name
    files = {
        d / "board.yml": f"# {GEN}\nboard:\n  name: {b.name}\n  full_name: {b.title}\n  vendor: {b.vendor}\n"
                         f"  socs:\n    - name: {f['soc']}\n",
        d / f"Kconfig.{b.name}": f"# SPDX-License-Identifier: CC-BY-NC-4.0\n# {GEN}\n\nconfig BOARD_{b.name.upper()}\n"
                                  f"\tselect {f['kconfig']}\n",
        d / f"{b.name}_defconfig": "# SPDX-License-Identifier: CC-BY-NC-4.0\n"
                                   f"# {GEN}\nCONFIG_GPIO=y\nCONFIG_ARM_MPU=y\nCONFIG_HW_STACK_PROTECTION=y\n"
                                   "# Firmware lives after the bootloader partition\nCONFIG_USE_DT_CODE_PARTITION=y\n",
        d / "board.cmake": f"# SPDX-License-Identifier: CC-BY-NC-4.0\n# {GEN}\nset(APX_JLINK_DEVICE {f['jlink']})\n"
                           f"set(APX_PYOCD_TARGET {f['pyocd']})\n"
                           "include(${CMAKE_CURRENT_LIST_DIR}/../../common/stm32_runners.board.cmake)\n",
        d / "support" / "openocd.cfg": (ZEPHYR / "boards/common/openocd_stm32h7.cfg").read_text()
        .replace("target/stm32h7x.cfg", f["openocd"]).replace("APX STM32H7", "APX STM32"),
        d / f"{b.name}.dts": dts(b),
        ZEPHYR / "app" / "boards" / f"{b.name}.conf": usb_conf(b, b.usb, f"APX {b.title}", "GCS (MAVLink)"),
        ZEPHYR / "bootloader" / "boards" / f"{b.name}.conf": usb_conf(b, b.bl_usb, f"APX Bootloader ({b.title})",
                                                                      "bootloader"),
    }
    if not dry:
        for path, text in files.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
    return d


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--hwdef-root", type=Path, help="directory holding one hwdef folder per board")
    p.add_argument("--board", action="append", help="board key (repeatable; default: all curated boards)")
    p.add_argument("--list", action="store_true", help="list curated boards")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--pins-only", action="store_true",
                   help="only rewrite board_pins.json (the GUI board diagram model), not the board files")
    a = p.parse_args()
    if a.list:
        for k, (folder, vendor, title, bid) in BOARDS.items():
            print(f"{k:<22} {title:<32} id {bid}  ({vendor}/{folder})")
        return 0
    if not a.hwdef_root:
        p.error("--hwdef-root is required")
    failed = 0
    for key in a.board or list(BOARDS):
        if key not in BOARDS:
            print(f"  skip {key}: not a curated board (see --list)")
            failed += 1
            continue
        if a.pins_only:
            folder, vendor, title, _ = BOARDS[key]
            path = write_pins(key, vendor, pin_model(key, folder, title, a.hwdef_root), a.dry_run)
            print(f"  ok   {key:<22} pin model only -> {path.relative_to(ROOT)}")
            continue
        try:
            b = build_board(key, a.hwdef_root)
        except (ValueError, OSError) as exc:
            print(f"  FAIL {key}: {exc}")
            failed += 1
            continue
        d = write_board(b, a.dry_run)
        write_pins(key, b.vendor, pin_model(key, BOARDS[key][0], b.title, a.hwdef_root), a.dry_run)
        extra = []
        if b.iomcu:
            extra.append(f"IO coprocessor on {b.iomcu[0]}")
        if b.rails:
            extra.append(f"{len(b.rails)} power rail(s)")
        if b.bl_usb.get("vid"):
            extra.append(f"bootloader USB {b.bl_usb['vid']:04x}:{b.bl_usb.get('pid', 0):04x}")
        extra += b.notes
        print(f"  ok   {key:<22} {b.mcu} {b.hse_mhz} MHz, id {b.board_id}"
              f"{', ' + ', '.join(extra) if extra else ''}  -> {d.relative_to(ROOT)}")
    if not a.board:
        for key, (folder, vendor, title) in PINS_ONLY.items():
            path = write_pins(key, vendor, pin_model(key, folder, title, a.hwdef_root), a.dry_run)
            print(f"  ok   {key:<22} pin model only -> {path.relative_to(ROOT)}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
