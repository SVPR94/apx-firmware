# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Turn (target, option values) into a build plan and run it.

The runner streams output line by line to a callback, tracks progress
([n/m] from Ninja, [ nn%] from Make), counts warnings/errors, and can be
stopped (the whole process tree is terminated). Used by the GUI (in a
worker thread) and the CLI (directly).
"""

from __future__ import annotations

import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from . import env as E
from .options import options_for
from .targets import Target

GENERATED_DIR = E.REPO_ROOT / "build" / ".apx_builder"


@dataclass
class Step:
    name: str
    argv: list
    env: dict
    cwd: Path = E.REPO_ROOT


@dataclass
class BuildPlan:
    target: Target
    values: dict
    build_dir: Path
    steps: list = field(default_factory=list)
    files: dict = field(default_factory=dict)     # Path -> content written before running
    delete_dir: Path | None = None
    notes: list = field(default_factory=list)
    errors: list = field(default_factory=list)    # problems that prevent running

    def command_lines(self) -> list[str]:
        out = []
        if self.delete_dir:
            out.append(f"# delete {self.delete_dir}")
        for p, content in self.files.items():
            out.append(f"# write {p}")
        out += [_join(s.argv) for s in self.steps]
        return out


def _join(argv) -> str:
    if E.IS_WIN:
        return subprocess.list2cmdline([str(a) for a in argv])
    return " ".join(shlex.quote(str(a)) for a in argv)


def _flags(target: Target, values: dict, attr: str) -> list[str]:
    out = []
    for o in options_for(target):
        spec = getattr(o, attr)
        if not spec:
            continue
        v = values.get(o.key)
        if o.kind == "bool" and v:
            out += spec
        elif o.kind == "enum" and isinstance(spec, dict):
            out += spec.get(v, [])
    if attr == "cflags" and values.get("extra_cflags"):
        out += shlex.split(values["extra_cflags"], posix=not E.IS_WIN)
    return out


def _kconfig_lines(target: Target, values: dict, image: str = "app") -> list[str]:
    lines = []
    for o in options_for(target):
        if not o.kconfig or o.image != image:
            continue
        v, d = values.get(o.key), o.default_for(target)
        if v == d:
            continue
        if o.kind == "enum":
            lines += o.kconfig.get(v, [])
        elif o.kind == "bool":
            syms = o.kconfig if isinstance(o.kconfig, list) else [o.kconfig]
            lines += [f"{s}={'y' if v else 'n'}" for s in syms]
        elif o.kind == "int":
            lines.append(f"{o.kconfig}={int(v)}")
        elif o.kind == "hex":
            if v:
                lines.append(f"{o.kconfig}={v}")
        else:
            lines.append(f'{o.kconfig}="{v}"')
    key = "extra_kconfig" if image == "app" else "bl_extra_kconfig"
    extra = [ln.strip() for ln in str(values.get(key) or "").splitlines() if ln.strip()]
    return lines + extra


def _cache_generator(build_dir: Path) -> str | None:
    try:
        for line in (build_dir / "CMakeCache.txt").read_text(errors="replace").splitlines():
            if line.startswith("CMAKE_GENERATOR:INTERNAL="):
                return line.split("=", 1)[1]
    except OSError:
        pass
    return None


def safe_to_delete(path: Path) -> bool:
    """Only ever delete build trees: under <repo>/build or a dir holding a CMake cache."""
    path = path.resolve()
    if path in (E.REPO_ROOT, E.REPO_ROOT / "build") or not path.exists():
        return False
    try:
        path.relative_to(E.REPO_ROOT / "build")
        return True
    except ValueError:
        return (path / "CMakeCache.txt").exists()


def make_plan(target: Target, values: dict, env_settings: dict) -> BuildPlan:
    build_dir = (E.REPO_ROOT / (values.get("build_dir") or target.build_dir)).resolve()
    plan = BuildPlan(target=target, values=values, build_dir=build_dir)
    gen_dir = GENERATED_DIR / target.id.replace(":", "_")
    if E.HOST not in target.hosts:
        plan.notes.append(f"{target.id} is not officially supported on {E.HOST}.")
    plan.notes += target.notes

    if values.get("pristine") and build_dir.exists():
        if safe_to_delete(build_dir):
            plan.delete_dir = build_dir
        else:
            plan.errors.append(f"refusing to delete {build_dir}: not a build directory")

    if target.kind == "zephyr":
        _plan_zephyr(plan, values, env_settings, gen_dir)
    else:
        _plan_cmake(plan, values, env_settings, gen_dir)
    return plan


def _plan_cmake(plan: BuildPlan, values: dict, env_settings: dict, gen_dir: Path) -> None:
    t = plan.target
    cmake = E.which("cmake")
    if not cmake:
        plan.errors.append("cmake not found on PATH")
        cmake = "cmake"
    gen = E.resolve_generator(values.get("generator", "Auto"))
    old = _cache_generator(plan.build_dir)
    if old and old != gen and not plan.delete_dir:
        if safe_to_delete(plan.build_dir):
            plan.delete_dir = plan.build_dir
            plan.notes.append(f"Build backend changed ({old} -> {gen}): doing a clean build.")
        else:
            plan.errors.append(f"{plan.build_dir} was configured with {old}; choose that backend")

    # Compile/link flags go through a CMake project-include so the toolchain
    # file's CPU flags (CMAKE_<LANG>_FLAGS_INIT) are never overwritten.
    cflags, lflags = _flags(t, values, "cflags"), _flags(t, values, "lflags")
    # Lives inside the build dir so it can never go missing while the cache points at it.
    inc = plan.build_dir / "apx_builder_options.cmake"
    body = ["# Generated by tools/apx_build.py - do not edit."]
    if cflags:
        body.append("add_compile_options(" + " ".join(_cmake_quote(f) for f in cflags) + ")")
    if lflags:
        body.append("add_link_options(" + " ".join(_cmake_quote(f) for f in lflags) + ")")
    plan.files[inc] = "\n".join(body) + "\n"

    argv = [cmake, "-S", E.REPO_ROOT, "-B", plan.build_dir, "-G", gen,
            f"-DCMAKE_PROJECT_INCLUDE={inc.as_posix()}"]
    if t.kind == "baremetal":
        argv += [f"-DCMAKE_TOOLCHAIN_FILE={E.STM32H7_TOOLCHAIN_FILE.as_posix()}",
                 "-DAPX_BOARD=stm32h7", f"-DAPX_BOARD_TARGET={t.board}",
                 "-DAPX_BUILD_SITL=OFF", "-DAPX_BUILD_TESTS=OFF"]
        if not E.arm_toolchain_dir(env_settings):
            plan.errors.append("arm-none-eabi-gcc not found (set the ARM toolchain in Environment)")
    else:
        argv.append(f"-DAPX_BOARD={t.board}")
    for o in options_for(t):
        if o.cmake:
            v = values.get(o.key)
            argv.append(f"-D{o.cmake}={('ON' if v else 'OFF') if o.kind == 'bool' else v}")
    if values.get("extra_cmake"):
        argv += shlex.split(values["extra_cmake"], posix=not E.IS_WIN)
    run_env = E.cmake_env(env_settings)
    plan.steps.append(Step("Configure", argv, run_env))

    build = [cmake, "--build", plan.build_dir, "--parallel", str(values.get("jobs") or E.cpu_count())]
    if t.cmake_target:
        build += ["--target", t.cmake_target]
    if values.get("verbose"):
        build.append("--verbose")
    plan.steps.append(Step("Build", build, run_env))


def _plan_zephyr(plan: BuildPlan, values: dict, env_settings: dict, gen_dir: Path) -> None:
    t = plan.target
    west = E.west_command(env_settings)
    if not west:
        plan.errors.append("west not found: run platforms/zephyr/setup_zephyr.sh (or set it in Environment)")
        west = ["west"]
    zb = E.zephyr_base(env_settings)
    if not (zb / "VERSION").exists():
        plan.errors.append(f"Zephyr base not found at {zb}")
    if (env_settings.get("zephyr_toolchain_variant") or "gnuarmemb") == "gnuarmemb" \
            and not E.arm_toolchain_dir(env_settings):
        plan.errors.append("arm-none-eabi-gcc not found (set the ARM toolchain in Environment)")

    conf = gen_dir / "extra.conf"
    lines = _kconfig_lines(t, values)
    plan.files[conf] = "# Generated by tools/apx_build.py - do not edit.\n" + \
        "".join(ln + "\n" for ln in lines)
    with_bl = bool(values.get("bootloader", True))
    bl_conf = gen_dir / "bootloader.conf"
    bl_lines = _kconfig_lines(t, values, "apx_bootloader") if with_bl else []
    plan.files[bl_conf] = "# Generated by tools/apx_build.py - do not edit (APX bootloader image).\n" + \
        "".join(ln + "\n" for ln in bl_lines)
    cflags = " ".join(_flags(t, values, "cflags"))
    argv = [*west]
    if values.get("verbose"):
        argv.append("-v")
    argv += ["build", "--sysbuild", "-b", t.board, "-d", plan.build_dir, "-p", "auto",
             "-o=-j{}".format(values.get("jobs") or E.cpu_count())]
    if values.get("verbose"):
        argv.append("-o=-v")
    # Sysbuild: -D<image>_<VAR> targets one image; "app" is the flight firmware.
    argv += [E.ZEPHYR_APP, "--", f"-Dapp_EXTRA_CONF_FILE={conf.as_posix()}",
             f"-Dapp_EXTRA_CFLAGS={cflags}", f"-Dapp_EXTRA_CXXFLAGS={cflags}",
             f"-DSB_CONFIG_APX_BOOTLOADER={'y' if with_bl else 'n'}",
             f"-Dapx_bootloader_EXTRA_CONF_FILE={bl_conf.as_posix()}"]
    if values.get("extra_cmake"):
        argv += shlex.split(values["extra_cmake"], posix=not E.IS_WIN)
    plan.steps.append(Step("West build", argv, E.zephyr_env(env_settings)))
    if lines:
        plan.notes.append("Kconfig overrides: " + ", ".join(lines))
    if bl_lines:
        plan.notes.append("Bootloader Kconfig overrides: " + ", ".join(bl_lines))


def _cmake_quote(flag: str) -> str:
    return '"' + flag.replace("\\", "/").replace('"', '\\"') + '"'


# --------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------
DIAG_RE = re.compile(r"^(?P<file>(?:[A-Za-z]:)?[^:\s][^:]*?):(?P<line>\d+):(?:(?P<col>\d+):)?\s*"
                     r"(?P<sev>warning|error|fatal error):\s*(?P<msg>.*)$")
TOOL_DIAG_RE = re.compile(r"^(?P<file>\S*?(?:ld|ld\.bfd|collect2|ninja|make|cc1plus)(?:\.exe)?):\s*"
                          r"(?P<sev>warning|error|fatal error):\s*(?P<msg>.*)$")
LINKER_FILE_RE = re.compile(r"^(?P<file>[^:\s]+\.c):\((?P<msg>[^)]*)\):\s*(?P<sev>warning|error):\s*(?P<m2>.*)$")
KCONFIG_RE = re.compile(r"^warning: (?P<sym>[A-Z0-9_]+) \(defined at (?P<file>[^:]+):(?P<line>\d+)\) "
                        r"(?P<msg>.*)$")
CMAKE_ERR_RE = re.compile(r"^CMake (?P<sev>Error|Warning)(?: \(dev\))? at (?P<file>[^:]+):(?P<line>\d+)")
NINJA_RE = re.compile(r"^\[(\d+)/(\d+)\]")
MAKE_RE = re.compile(r"^\[\s*(\d+)%\]")
UNIT_RE = re.compile(r"Building (?:C|CXX|ASM) object|\bBuilding (?:C|CXX|ASM)\b")


@dataclass
class RunState:
    started: float = 0.0
    steps: list = field(default_factory=list)          # (name, seconds, returncode)
    diagnostics: list = field(default_factory=list)    # dicts: sev, file, line, msg
    units: int = 0
    progress: float = 0.0
    returncode: int | None = None
    cancelled: bool = False
    log: list = field(default_factory=list)

    @property
    def warnings(self) -> int:
        return sum(1 for d in self.diagnostics if d["sev"] == "warning")

    @property
    def errors(self) -> int:
        return sum(1 for d in self.diagnostics if d["sev"] != "warning")


class BuildRunner:
    """Run a plan. Callbacks: on_line(text, tag), on_progress(fraction, step_name)."""

    def __init__(self, plan: BuildPlan, on_line: Callable[[str, str], None] = lambda s, t: None,
                 on_progress: Callable[[float, str], None] = lambda f, s: None, keep_log: bool = True):
        self.plan = plan
        self.on_line = on_line
        self.on_progress = on_progress
        self.keep_log = keep_log
        self.state = RunState()
        self._proc: subprocess.Popen | None = None
        self._stop = threading.Event()
        self._seen = set()

    # ---- control
    def stop(self) -> None:
        self._stop.set()
        p = self._proc
        if p and p.poll() is None:
            try:
                if E.IS_WIN:
                    subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], capture_output=True)
                else:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except (OSError, ProcessLookupError):
                pass

    # ---- main
    def run(self) -> RunState:
        st = self.state
        st.started = time.time()
        plan = self.plan
        if plan.errors:
            for e in plan.errors:
                self._emit(f"ERROR: {e}", "error")
            st.returncode = 2
            return st
        for n in plan.notes:
            self._emit(f"NOTE: {n}", "note")
        if plan.delete_dir:
            self._emit(f"Removing {plan.delete_dir}", "step")
            shutil.rmtree(plan.delete_dir, ignore_errors=True)
        for path, content in plan.files.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            if not path.exists() or path.read_text() != content:  # keep mtime => no needless reconfigure
                path.write_text(content)
        # Progress weights: configure is quick, the compile step is most of the work.
        weights = [0.1 if s.name == "Configure" else 1.0 for s in plan.steps]
        total = sum(weights) or 1.0
        self._spans = [(sum(weights[:i]) / total, weights[i] / total) for i in range(len(weights))]
        n_steps = len(plan.steps)
        for i, step in enumerate(plan.steps):
            if self._stop.is_set():
                st.cancelled = True
                break
            self._emit(f"==> {step.name}: {_join(step.argv)}", "step")
            t0 = time.time()
            rc = self._run_step(step, i, n_steps)
            st.steps.append((step.name, time.time() - t0, rc))
            if self._stop.is_set():
                st.cancelled = True
                rc = rc or -1
            if rc != 0:
                st.returncode = rc
                break
        else:
            st.returncode = 0
            st.progress = 1.0
            self.on_progress(1.0, "done")
        if st.returncode is None:
            st.returncode = -1
        return st

    def _run_step(self, step: Step, index: int, n_steps: int) -> int:
        kwargs = {}
        if E.IS_WIN:
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kwargs["start_new_session"] = True
        try:
            self._proc = subprocess.Popen(
                [str(a) for a in step.argv], cwd=str(step.cwd), env=step.env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                text=True, encoding="utf-8", errors="replace", bufsize=1, **kwargs)
        except OSError as exc:
            self._emit(f"ERROR: cannot start {step.argv[0]}: {exc}", "error")
            return 127
        base, span = self._spans[index] if getattr(self, "_spans", None) else (index / n_steps, 1 / n_steps)
        for raw in self._proc.stdout:
            line = raw.rstrip("\r\n")
            frac = self._parse(line)
            if frac is not None:
                self.state.progress = base + frac * span
                self.on_progress(self.state.progress, step.name)
        return self._proc.wait()

    def _parse(self, line: str) -> float | None:
        tag = ""
        frac = None
        m = NINJA_RE.match(line)
        if m and int(m.group(2)):
            frac = int(m.group(1)) / int(m.group(2))
        else:
            m = MAKE_RE.match(line)
            if m:
                frac = int(m.group(1)) / 100.0
        if UNIT_RE.search(line):
            self.state.units += 1
        d = DIAG_RE.match(line)
        if d:
            self._diag(d.group("sev"), d.group("file"), int(d.group("line")), d.group("msg"))
            tag = "warning" if d.group("sev") == "warning" else "error"
        else:
            d = TOOL_DIAG_RE.match(line) or LINKER_FILE_RE.match(line)
            if d:
                msg = d.group("msg") if "m2" not in d.groupdict() else d.group("m2")
                self._diag(d.group("sev"), Path(d.group("file")).name, None, msg)
                tag = "warning" if d.group("sev") == "warning" else "error"
            elif KCONFIG_RE.match(line):
                k = KCONFIG_RE.match(line)
                self._diag("warning", "Kconfig", None,
                           f"CONFIG_{k.group('sym')} {k.group('msg')} (see console for unmet dependencies)")
                tag = "warning"
            else:
                d = CMAKE_ERR_RE.match(line)
                if d:
                    sev = "error" if d.group("sev") == "Error" else "warning"
                    self._diag(sev, d.group("file"), int(d.group("line")), "CMake " + d.group("sev"))
                    tag = sev
                elif line.startswith(("FAILED:", "ninja: build stopped", "make: ***")) or \
                        "Error " in line[:12]:
                    tag = "error"
                elif "Result : PASS" in line:
                    tag = "success"
                elif "Result : FAIL" in line or line.lstrip().startswith("ERROR  :"):
                    tag = "error"
                elif line.lstrip().startswith("WARNING:"):
                    tag = "warning"
        self._emit(line, tag)
        return frac

    def _diag(self, sev, file, line, msg) -> None:
        sev = "warning" if sev.lower() == "warning" else "error"
        key = (sev, file, line, msg)
        if key in self._seen:
            return
        self._seen.add(key)
        self.state.diagnostics.append({"sev": sev, "file": file, "line": line, "msg": msg})

    def _emit(self, text: str, tag: str) -> None:
        if self.keep_log:
            self.state.log.append(text)
        self.on_line(text, tag)
