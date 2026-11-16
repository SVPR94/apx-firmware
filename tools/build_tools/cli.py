# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Command-line front end. With no arguments the GUI starts.

  apx_build.py                               start the GUI
  apx_build.py list                          list firmware targets
  apx_build.py options <target>              options + defaults for a target
  apx_build.py build <target> [opts]         build and print statistics
  apx_build.py stats <target>                statistics of the existing build
  apx_build.py open <target>                 open the binary folder
  apx_build.py flash|debug <target>          flash / start GDB server via probe
  apx_build.py diagram <target> [--svg F]     board pin map (table / SVG block diagram)
  apx_build.py doctor                        check toolchains and tools

Build examples:
  apx_build.py build zephyr:cube_orange_plus --profile Debug
  apx_build.py build stm32h7:apx_h743_v1 -s build_type=Debug -s stack_usage=on --json out.json
  apx_build.py build host:sitl --pristine -j 8 --dry-run
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import env as E
from . import options as O
from . import probe as P
from . import stats as S
from .builder import BuildRunner, make_plan
from .targets import discover, find

_COLORS = {"warning": "\033[33m", "error": "\033[31m", "step": "\033[1;36m", "success": "\033[32m",
           "note": "\033[35m"}


def _resolve_values(target, args, settings) -> dict:
    base = O.merge(target, settings["values"].get(target.id)) if getattr(args, "saved", False) \
        else O.defaults(target)
    vals = O.apply_profile(target, args.profile, base) if getattr(args, "profile", None) else base
    for item in getattr(args, "set", None) or []:
        key, sep, raw = item.partition("=")
        if not sep or key not in O.BY_KEY:
            raise SystemExit(f"unknown option '{key}'. See: apx_build.py options {target.id}")
        opt = O.BY_KEY[key]
        if not opt.applies(target):
            raise SystemExit(f"option '{key}' does not apply to {target.id}")
        try:
            vals[key] = opt.parse(raw)
        except ValueError as exc:
            raise SystemExit(str(exc))
    if getattr(args, "pristine", False):
        vals["pristine"] = True
    if getattr(args, "jobs", None):
        vals["jobs"] = args.jobs
    if getattr(args, "verbose", False):
        vals["verbose"] = True
    return vals


def cmd_list(args, settings):
    from .targets import boards
    for key, title, ts in boards(discover()):
        print(f"{title}  [{key}, {ts[0].mcu}]")
        for t in ts:
            print(f"    {t.os_label:<22} {t.id:<28} {t.build_dir}")
            for n in t.notes:
                if "experimental" not in n:
                    print(f"    {'':<22} ! {n}")


def cmd_options(args, settings):
    t = find(discover(), args.target, getattr(args, 'os', None))
    print(f"Options for {t.id} ({t.label})\n")
    for group in O.GROUPS:
        opts = [o for o in O.options_for(t) if o.group == group]
        if not opts:
            continue
        print(f"[{group}]")
        for o in opts:
            d = t.build_dir if o.key == "build_dir" else o.default_for(t)
            if o.key == "probe_device":
                d = d or "(from MCU)"
            if o.kind == "bool":
                d = "on" if d else "off"
            print(f"  {o.key:<20} = {str(d):<16} {o.label}")
            if o.kind == "enum":
                print(f"  {'':<20}   choices: {', '.join(c[0] for c in o.choices)}")
        print()
    print("Profiles:", ", ".join(O.PROFILES))


def cmd_build(args, settings):
    t = find(discover(), args.target, getattr(args, 'os', None))
    vals = _resolve_values(t, args, settings)
    plan = make_plan(t, vals, settings["env"])
    if args.dry_run:
        for n in plan.notes:
            print(f"# NOTE: {n}")
        for e in plan.errors:
            print(f"# ERROR: {e}")
        for p, content in plan.files.items():
            print(f"# --- {p} ---\n" + "".join(f"#   {ln}\n" for ln in content.splitlines()))
        print("\n".join(plan.command_lines()))
        return 0 if not plan.errors else 2
    color = sys.stdout.isatty() and not args.no_color

    def on_line(text, tag):
        if args.quiet and tag not in ("error", "warning", "step"):
            return
        print(f"{_COLORS[tag]}{text}\033[0m" if color and tag in _COLORS else text, flush=True)

    runner = BuildRunner(plan, on_line=on_line)
    try:
        state = runner.run()
    except KeyboardInterrupt:
        runner.stop()
        state = runner.state
        state.cancelled = True
    report = S.build_report(plan, state)
    S.record(report)
    print(S.format_text(report))
    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=2, default=str))
        print(f"Statistics written to {args.json}")
    return 0 if report["ok"] else (state.returncode or 1)


def cmd_stats(args, settings):
    t = find(discover(), args.target, getattr(args, 'os', None))
    vals = O.merge(t, settings["values"].get(t.id))
    build_dir = (E.REPO_ROOT / vals["build_dir"]).resolve()
    mem = S.memory_report(t, build_dir, regenerate=args.regenerate)
    arts = S.artifacts(t, build_dir)
    if not arts:
        raise SystemExit(f"no build found in {build_dir}")
    print(f"{t.label}\nBinary dir: {S.binary_dir(t, build_dir)}")
    for a in arts:
        print(f"  {a['name']:<34}{S.fmt_bytes(a['size']):>12}")
    tot = S.memory_totals(mem)
    if tot:
        print(f"Flash {S.fmt_bytes(tot.get('flash_used'))} / {S.fmt_bytes(tot.get('flash_cap'))}, "
              f"RAM {S.fmt_bytes(tot.get('ram_used'))} / {S.fmt_bytes(tot.get('ram_cap'))}  "
              f"[{tot.get('result')}]")
    rep_txt = next((Path(a["path"]) for a in arts if a["name"].endswith(".txt")), None)
    if rep_txt and args.full:
        print(rep_txt.read_text())
    return 0


def cmd_open(args, settings):
    t = find(discover(), args.target, getattr(args, 'os', None))
    vals = O.merge(t, settings["values"].get(t.id))
    d = S.binary_dir(t, (E.REPO_ROOT / vals["build_dir"]).resolve())
    if not d.exists():
        raise SystemExit(f"{d} does not exist yet - build first")
    E.open_path(d)
    print(d)
    return 0


def _probe_cmd(args, settings, kind):
    t = find(discover(), args.target, getattr(args, 'os', None))
    if t.kind == "host":
        vals = O.merge(t, settings["values"].get(t.id))
        print(P.gdb_hint(t, (E.REPO_ROOT / vals["build_dir"]).resolve(), vals, settings["env"]))
        return 0
    vals = _resolve_values(t, args, settings)
    if args.probe:
        vals["probe"] = O.BY_KEY["probe"].parse(args.probe)
    build_dir = (E.REPO_ROOT / vals["build_dir"]).resolve()
    if kind == "flash":
        warning = P.flash_warning(t, vals)
        if warning and not args.yes:
            raise SystemExit("WARNING: " + warning + "\nRe-run with --yes to flash anyway.")
        step = P.flash_step(t, build_dir, vals, E.settings_dir())
    else:
        step = P.debug_step(t, build_dir, vals)
        print("Attach with:", P.gdb_hint(t, build_dir, vals, settings["env"]))
    from .builder import BuildPlan
    plan = BuildPlan(target=t, values=vals, build_dir=build_dir, steps=[step])
    if args.dry_run:
        print("\n".join(plan.command_lines()))
        return 0
    st = BuildRunner(plan, on_line=lambda s, tag: print(s, flush=True), keep_log=False).run()
    return st.returncode or 0


def cmd_diagram(args, settings):
    from . import board_desc as BD
    from . import diagram as DG
    t = find(discover(), args.target, getattr(args, "os", None))
    desc = BD.load_for_target(t, E.REPO_ROOT)
    if not desc:
        raise SystemExit(f"no pin description for {t.id}")
    print(f"{desc['title']} - {desc['mcu']}")
    print(f"  {'Pin':<9}{'Function':<34}{'Group / bus':<44}Connected to")
    for row in BD.pin_table(desc):
        print(f"  {row[0]:<9}{row[1][:33]:<34}{row[2][:43]:<44}{row[3][:60]}")
    if args.svg:
        shapes, w, h = DG.layout(desc)
        Path(args.svg).write_text(DG.to_svg(shapes, w, h))
        print(f"diagram written to {args.svg}")
    return 0


def cmd_doctor(args, settings):
    ok_all = True
    for r in E.doctor(settings["env"]):
        mark = "OK " if r["ok"] else "-- "
        print(f"{mark} {r['tool']:<22} {r['needed']:<24} {r['detail']}")
        ok_all &= r["ok"] or "optional" in r["needed"]
    return 0 if ok_all else 1


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="apx_build.py", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd")
    sub.add_parser("gui", help="start the GUI (default)")
    sub.add_parser("list", help="list targets")
    sp = sub.add_parser("options", help="show options for a target")
    sp.add_argument("target")
    sp.add_argument("--os", help="zephyr | baremetal | posix (when a board builds on several)")

    def value_args(sp):
        sp.add_argument("target", help="target id (zephyr:cube_orange_plus) or board name with --os")
        sp.add_argument("--os", help="zephyr | baremetal | posix (when a board builds on several)")
        sp.add_argument("--profile", choices=list(O.PROFILES))
        sp.add_argument("-s", "--set", action="append", metavar="KEY=VALUE", help="option override")
        sp.add_argument("--saved", action="store_true", help="start from the GUI's saved options")
        sp.add_argument("--dry-run", action="store_true", help="print commands only")

    b = sub.add_parser("build", help="build a target")
    value_args(b)
    b.add_argument("--pristine", action="store_true")
    b.add_argument("-j", "--jobs", type=int)
    b.add_argument("-v", "--verbose", action="store_true")
    b.add_argument("-q", "--quiet", action="store_true", help="only show steps, warnings and errors")
    b.add_argument("--json", metavar="FILE", help="write statistics JSON")
    b.add_argument("--no-color", action="store_true")
    s = sub.add_parser("stats", help="statistics of the existing build")
    s.add_argument("target")
    s.add_argument("--os")
    s.add_argument("--full", action="store_true", help="print the full memory report")
    s.add_argument("--regenerate", action="store_true")
    o = sub.add_parser("open", help="open the binary folder")
    o.add_argument("target")
    o.add_argument("--os")
    for name in ("flash", "debug"):
        f = sub.add_parser(name, help=f"{name} via the selected debug probe")
        value_args(f)
        f.add_argument("--probe", help="openocd_stlink | openocd_cmsisdap | openocd_jlink | pyocd | jlink | cubeprog")
        f.add_argument("--yes", action="store_true", help="flash even if it overwrites the bootloader")
    dg = sub.add_parser("diagram", help="board pin map: table, and an SVG block diagram with --svg")
    dg.add_argument("target")
    dg.add_argument("--os")
    dg.add_argument("--svg", metavar="FILE")
    sub.add_parser("doctor", help="check toolchains")
    return p


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    if args.cmd in (None, "gui"):
        from .gui import run_gui
        return run_gui()
    settings = E.load_settings()
    handlers = {"list": cmd_list, "options": cmd_options, "build": cmd_build, "stats": cmd_stats,
                "open": cmd_open, "doctor": cmd_doctor, "diagram": cmd_diagram,
                "flash": lambda a, s: _probe_cmd(a, s, "flash"),
                "debug": lambda a, s: _probe_cmd(a, s, "debug")}
    try:
        return handlers[args.cmd](args, settings) or 0
    except (KeyError, FileNotFoundError, ValueError) as exc:
        print(f"error: {exc.args[0] if exc.args else exc}", file=sys.stderr)
        return 2
