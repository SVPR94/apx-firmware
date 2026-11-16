# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""Tkinter GUI for the APX firmware builder (Windows / Linux / macOS).

Tkinter ships with Python, so the GUI needs no extra packages. Long-running
work (builds, flashing, tool checks) runs in worker threads; the UI thread
drains a queue every 40 ms.
"""

from __future__ import annotations

import json
import queue
import shutil
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, font as tkfont, messagebox, ttk

from . import __version__
from . import env as E
from . import options as O
from . import probe as P
from . import board_desc as BD
from . import diagram as DG
from . import stats as S
from .builder import BuildPlan, BuildRunner, make_plan, safe_to_delete
from .targets import boards, discover

PAD = 6
TAG_COLORS = {"warning": "#b36b00", "error": "#c62828", "step": "#1565c0", "success": "#2e7d32",
              "note": "#7b1fa2"}


# --------------------------------------------------------------------------
# Small widgets
# --------------------------------------------------------------------------
class Tooltip:
    def __init__(self, widget, text: str):
        self.widget, self.text, self.tip, self.after_id = widget, text, None, None
        widget.bind("<Enter>", self._schedule, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _schedule(self, _e=None):
        self.after_id = self.widget.after(500, self._show)

    def _show(self):
        if self.tip or not self.text:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.wm_geometry(f"+{x}+{y}")
        tk.Label(self.tip, text=self.text, justify="left", background="#ffffe0", foreground="#000",
                 relief="solid", borderwidth=1, wraplength=420, padx=6, pady=4).pack()

    def _hide(self, _e=None):
        if self.after_id:
            self.widget.after_cancel(self.after_id)
            self.after_id = None
        if self.tip:
            self.tip.destroy()
            self.tip = None


class ScrollFrame(ttk.Frame):
    """Vertically scrollable frame; mouse wheel works on all three OSes."""

    def __init__(self, master, **kw):
        super().__init__(master, **kw)
        self.canvas = tk.Canvas(self, highlightthickness=0, borderwidth=0)
        vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas)
        self.inner.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self._win = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self._win, width=e.width))
        self.canvas.configure(yscrollcommand=vsb.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        self.bind_all("<MouseWheel>", self._wheel, add="+")
        self.bind_all("<Button-4>", self._wheel, add="+")
        self.bind_all("<Button-5>", self._wheel, add="+")

    def _wheel(self, e):
        w = self.winfo_containing(e.x_root, e.y_root)
        while w is not None and w is not self:
            if isinstance(w, (tk.Text, ttk.Treeview, ttk.Combobox, tk.Listbox)):
                return
            w = w.master
        if w is None:
            return
        if e.num == 4:
            delta = -1
        elif e.num == 5:
            delta = 1
        else:
            delta = -1 if e.delta > 0 else 1
            if E.IS_MAC:
                delta = -e.delta
        self.canvas.yview_scroll(delta, "units")


def make_tree(master, columns, widths, anchors=None, height=10):
    frame = ttk.Frame(master)
    tree = ttk.Treeview(frame, columns=columns, show="headings", height=height)
    for i, (c, w) in enumerate(zip(columns, widths)):
        tree.heading(c, text=c, command=lambda c=c: _sort_tree(tree, c, False))
        tree.column(c, width=w, anchor=(anchors or {}).get(c, "w"), stretch=(i == len(columns) - 1))
    vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=vsb.set)
    tree.pack(side="left", fill="both", expand=True)
    vsb.pack(side="right", fill="y")
    return frame, tree


def _sort_key(text: str):
    t = text.replace(",", "").strip()
    for suffix, mul in ((" MiB", 1048576), (" KiB", 1024), (" B", 1), (" s", 1), ("%", 1)):
        if t.endswith(suffix):
            try:
                return (0, float(t[: -len(suffix)]) * mul)
            except ValueError:
                break
    try:
        return (0, float(t))
    except ValueError:
        return (1, t.lower())


def _sort_tree(tree, col, reverse):
    rows = [(tree.set(k, col), k) for k in tree.get_children("")]
    rows.sort(key=lambda r: _sort_key(r[0]), reverse=reverse)
    for i, (_, k) in enumerate(rows):
        tree.move(k, "", i)
    tree.heading(col, command=lambda: _sort_tree(tree, col, not reverse))


# --------------------------------------------------------------------------
# Main window
# --------------------------------------------------------------------------
class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.settings = E.load_settings()
        self.targets = discover()
        self.boards = boards(self.targets)       # [(key, title, [targets by OS])]
        self.target = None
        self.vars: dict = {}
        self.text_widgets: dict = {}
        self.widgets: dict = {}
        self.q: queue.Queue = queue.Queue()
        self.runner: BuildRunner | None = None
        self.busy = False
        self.busy_kind = ""
        self.t_start = 0.0
        self.report: dict | None = None

        root.title(f"APX Firmware Builder {__version__}")
        root.minsize(980, 640)
        root.geometry(self.settings.get("geometry", "1280x820"))
        self._style()
        self._build_ui()
        last = self.settings.get("last_target")
        first = next((t for t in self.targets if t.id == last), self.targets[0])
        self._select(first)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.bind_all("<Control-b>", lambda e: self.build())
        root.bind_all("<F5>", lambda e: self.build())
        root.after(40, self._poll)

    # ------------------------------------------------------------ styling
    def _style(self):
        st = ttk.Style(self.root)
        if E.HOST == "linux" and "clam" in st.theme_names():
            st.theme_use("clam")
        st.configure("Accent.TButton", font=(None, 10, "bold"))
        st.configure("Head.TLabel", font=(None, 11, "bold"))
        st.configure("Big.TLabel", font=(None, 12, "bold"))
        for name, color in (("Green", "#43a047"), ("Amber", "#fb8c00"), ("Red", "#e53935")):
            st.configure(f"{name}.Horizontal.TProgressbar", background=color)
        self.mono = tkfont.nametofont("TkFixedFont")

    # ------------------------------------------------------------ layout
    def _build_ui(self):
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)

        top = ttk.Frame(root, padding=(PAD, PAD, PAD, 0))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)
        ttk.Label(top, text="Board:", style="Head.TLabel").grid(row=0, column=0, sticky="w")
        self.board_cb = ttk.Combobox(top, state="readonly",
                                     values=[f"{title}  ({ts[0].mcu})" for _, title, ts in self.boards])
        self.board_cb.grid(row=0, column=1, sticky="ew", padx=PAD)
        self.board_cb.bind("<<ComboboxSelected>>", lambda e: self._on_board())
        Tooltip(self.board_cb, "Flight controller board (or host build).")
        ttk.Label(top, text="OS:", style="Head.TLabel").grid(row=0, column=2)
        self.os_cb = ttk.Combobox(top, state="readonly", width=20)
        self.os_cb.grid(row=0, column=3, padx=PAD)
        self.os_cb.bind("<<ComboboxSelected>>", lambda e: self._on_target())
        Tooltip(self.os_cb, "Operating system to build this board on. Zephyr RTOS runs the flight "
                            "firmware; bare-metal is the no-OS link test. New OS ports appear here.")
        ttk.Label(top, text="Profile:").grid(row=0, column=4)
        self.profile_cb = ttk.Combobox(top, state="readonly", values=list(O.PROFILES), width=10)
        self.profile_cb.current(0)
        self.profile_cb.grid(row=0, column=5, padx=PAD)
        b = ttk.Button(top, text="Apply profile", command=self.apply_profile)
        b.grid(row=0, column=6)
        Tooltip(b, "Load the profile's options on top of the defaults for this target.")
        b = ttk.Button(top, text="Reset defaults", command=self.reset_defaults)
        b.grid(row=0, column=7, padx=(PAD, 0))

        self.info = ttk.Label(root, padding=(PAD, 2), wraplength=1200, justify="left")
        self.info.grid(row=1, column=0, sticky="ew")

        bar = ttk.Frame(root, padding=(PAD, 2))
        bar.grid(row=2, column=0, sticky="ew")
        self.btn_build = ttk.Button(bar, text="Build  (Ctrl+B)", style="Accent.TButton", command=self.build)
        self.btn_stop = ttk.Button(bar, text="Stop", command=self.stop, state="disabled")
        self.btn_clean = ttk.Button(bar, text="Clean", command=self.clean)
        self.btn_open = ttk.Button(bar, text="Open binary folder", command=self.open_bin)
        self.btn_flash = ttk.Button(bar, text="Flash", command=self.flash)
        self.btn_debug = ttk.Button(bar, text="Debug server", command=self.debug)
        self.btn_cmd = ttk.Button(bar, text="Copy commands", command=self.copy_commands)
        tips = {self.btn_build: "Configure and build the selected target.",
                self.btn_stop: "Stop the running build / flash / GDB server.",
                self.btn_clean: "Delete the build directory.",
                self.btn_open: "Open the folder that holds .elf/.bin/.hex/.apj.",
                self.btn_flash: "Program the firmware over SWD with the selected probe backend.",
                self.btn_debug: "Start a GDB server on the probe (host targets: show the gdb command).",
                self.btn_cmd: "Copy the exact commands this build would run."}
        for w in (self.btn_build, self.btn_stop, self.btn_clean, self.btn_open, self.btn_flash,
                  self.btn_debug, self.btn_cmd):
            w.pack(side="left", padx=(0, PAD))
            Tooltip(w, tips[w])

        pane = ttk.PanedWindow(root, orient="horizontal")
        pane.grid(row=3, column=0, sticky="nsew", padx=PAD, pady=PAD)
        left = ttk.Frame(pane)
        self.opts_frame = ScrollFrame(left)
        self.opts_frame.pack(fill="both", expand=True)
        pane.add(left, weight=0)
        self.nb = ttk.Notebook(pane)
        pane.add(self.nb, weight=1)
        self._build_console_tab()
        self._build_stats_tab()
        self._build_problems_tab()
        self._build_history_tab()
        self._build_env_tab()
        self._build_diagram_tab()
        root.after(50, lambda: pane.sashpos(0, 440))

        status = ttk.Frame(root, padding=(PAD, 0, PAD, PAD))
        status.grid(row=4, column=0, sticky="ew")
        status.columnconfigure(1, weight=1)
        self.progress = ttk.Progressbar(status, maximum=1000, length=260)
        self.progress.grid(row=0, column=0)
        self.status = ttk.Label(status, text="Ready")
        self.status.grid(row=0, column=1, sticky="w", padx=PAD)
        self.elapsed = ttk.Label(status, text="")
        self.elapsed.grid(row=0, column=2, sticky="e")

    # ---- Board diagram
    def _build_diagram_tab(self):
        f = ttk.Frame(self.nb, padding=PAD)
        self.nb.add(f, text="Board diagram")
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        ttk.Button(bar, text="Zoom -", command=lambda: self._zoom(1 / 1.2)).pack(side="left")
        ttk.Button(bar, text="Zoom +", command=lambda: self._zoom(1.2)).pack(side="left", padx=PAD)
        ttk.Button(bar, text="100%", command=lambda: self._zoom(None)).pack(side="left")
        ttk.Button(bar, text="Fit width", command=self._fit_diagram).pack(side="left", padx=PAD)
        ttk.Button(bar, text="Export SVG...", command=self.export_diagram).pack(side="left")
        self.diag_title = ttk.Label(f, text="", style="Head.TLabel")
        self.diag_title.pack(anchor="w", pady=(PAD, 0))
        self.diag_scale = 1.0
        sub = ttk.Notebook(f)
        sub.pack(fill="both", expand=True, pady=(PAD, 0))
        cf = ttk.Frame(sub)
        sub.add(cf, text="Diagram")
        self.diag_canvas = tk.Canvas(cf, background="#ffffff", highlightthickness=0)
        vsb = ttk.Scrollbar(cf, orient="vertical", command=self.diag_canvas.yview)
        hsb = ttk.Scrollbar(cf, orient="horizontal", command=self.diag_canvas.xview)
        self.diag_canvas.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.diag_canvas.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")
        cf.rowconfigure(0, weight=1)
        cf.columnconfigure(0, weight=1)
        c = self.diag_canvas
        c.bind("<MouseWheel>", lambda e: c.yview_scroll(-1 if e.delta > 0 else 1, "units"))
        c.bind("<Shift-MouseWheel>", lambda e: c.xview_scroll(-1 if e.delta > 0 else 1, "units"))
        c.bind("<Button-4>", lambda e: c.yview_scroll(-1, "units"))
        c.bind("<Button-5>", lambda e: c.yview_scroll(1, "units"))
        c.bind("<Control-MouseWheel>", lambda e: self._zoom(1.1 if e.delta > 0 else 1 / 1.1))
        c.bind("<Control-Button-4>", lambda e: self._zoom(1.1))
        c.bind("<Control-Button-5>", lambda e: self._zoom(1 / 1.1))
        c.bind("<ButtonPress-1>", lambda e: c.scan_mark(e.x, e.y))
        c.bind("<B1-Motion>", lambda e: c.scan_dragto(e.x, e.y, gain=1))
        tf = ttk.Frame(sub)
        sub.add(tf, text="Pin table")
        fr, self.t_pins = make_tree(tf, ("Pin", "Function", "Group / bus", "Connected to"),
                                    (90, 260, 300, 500), height=20)
        fr.pack(fill="both", expand=True)
        self.diag_info = ttk.Label(f, text="Hover a pin or wire for details. Drag to pan, Ctrl+wheel to zoom.",
                                   foreground="#555")
        self.diag_info.pack(anchor="w", pady=(4, 0))
        self.diag_desc = None

    def refresh_diagram(self):
        if not hasattr(self, "diag_canvas"):
            return
        try:
            self.diag_desc = BD.load_for_target(self.target, E.REPO_ROOT)
        except (OSError, ValueError) as exc:
            self.diag_desc = None
            self.diag_info.configure(text=f"Board description unreadable: {exc}")
        self.t_pins.delete(*self.t_pins.get_children())
        c = self.diag_canvas
        if not self.diag_desc:
            c.delete("all")
            msg = ("Host build: no flight controller hardware." if self.target.kind == "host"
                   else "No pin description for this board (run tools/import_board.py).")
            c.create_text(20, 20, text=msg, anchor="nw", font=(None, 11))
            self.diag_title.configure(text=self.target.board_title)
            return
        d = self.diag_desc
        n_pins = sum(len(g.get("pins", [])) for g in d["groups"])
        has_io = any(g["kind"] == "iomcu" for g in d["groups"])
        self.diag_title.configure(text=f"{d.get('title')}  -  {d.get('mcu')}  -  {n_pins} pins"
                                       + ("  -  IO coprocessor" if has_io else ""))
        shapes, w, h = DG.layout(d, self.diag_scale)
        DG.draw_canvas(c, shapes, on_hover=lambda info: self.diag_info.configure(text=info))
        c.configure(scrollregion=(0, 0, w, h))
        for row in BD.pin_table(d):
            self.t_pins.insert("", "end", values=row)

    def _zoom(self, factor):
        self.diag_scale = 1.0 if factor is None else min(3.0, max(0.25, self.diag_scale * factor))
        self.refresh_diagram()

    def _fit_diagram(self):
        if not self.diag_desc:
            return
        _, w, _ = DG.layout(self.diag_desc, 1.0)
        avail = max(200, self.diag_canvas.winfo_width() - 10)
        self.diag_scale = min(3.0, max(0.25, avail / w))
        self.refresh_diagram()

    def export_diagram(self):
        if not self.diag_desc:
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".svg",
                                            initialfile=f"{self.target.board_key}_diagram.svg",
                                            filetypes=[("SVG image", "*.svg")])
        if path:
            shapes, w, h = DG.layout(self.diag_desc, 1.0)
            Path(path).write_text(DG.to_svg(shapes, w, h))
            self._set_status(f"Board diagram exported to {path}")

    # ---- Console
    def _build_console_tab(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Console")
        tools = ttk.Frame(f)
        tools.pack(fill="x", pady=(PAD, 2))
        ttk.Button(tools, text="Clear", command=lambda: self.console.delete("1.0", "end")).pack(side="left")
        ttk.Button(tools, text="Save log...", command=self.save_log).pack(side="left", padx=PAD)
        self.autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(tools, text="Auto-scroll", variable=self.autoscroll).pack(side="left")
        self.only_problems = tk.BooleanVar(value=False)
        ttk.Checkbutton(tools, text="Only warnings/errors", variable=self.only_problems).pack(side="left", padx=PAD)
        body = ttk.Frame(f)
        body.pack(fill="both", expand=True)
        self.console = tk.Text(body, wrap="none", font=self.mono, undo=False, background="#fbfbfb",
                               foreground="#1d1d1d")
        vsb = ttk.Scrollbar(body, orient="vertical", command=self.console.yview)
        hsb = ttk.Scrollbar(body, orient="horizontal", command=self.console.xview)
        self.console.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.console.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)
        for tag, color in TAG_COLORS.items():
            self.console.tag_configure(tag, foreground=color)
        self.console.tag_configure("step", foreground=TAG_COLORS["step"], font=(self.mono.actual("family"),
                                   self.mono.actual("size"), "bold"))

    # ---- Statistics
    def _build_stats_tab(self):
        f = ttk.Frame(self.nb, padding=PAD)
        self.nb.add(f, text="Statistics")
        head = ttk.Frame(f)
        head.pack(fill="x")
        self.stats_title = ttk.Label(head, text="No build yet", style="Big.TLabel")
        self.stats_title.pack(side="left")
        ttk.Button(head, text="Export JSON...", command=self.export_stats).pack(side="right")
        ttk.Button(head, text="Memory report", command=self.open_mem_report).pack(side="right", padx=PAD)
        ttk.Button(head, text="Reload", command=self.refresh_stats).pack(side="right")

        self.summary = ttk.Frame(f)
        self.summary.pack(fill="x", pady=PAD)
        self.sum_labels = {}
        keys = ["Result", "Duration", "Steps", "Units compiled", "Warnings / errors", "Image size",
                "Flash", "RAM", "Memory check", "Bootloader", "Binary dir"]
        for i, k in enumerate(keys):
            r, c = divmod(i, 2)
            ttk.Label(self.summary, text=k + ":").grid(row=r, column=c * 2, sticky="w", padx=(0, 4))
            v = ttk.Label(self.summary, text="-")
            v.grid(row=r, column=c * 2 + 1, sticky="w", padx=(0, 24))
            self.sum_labels[k] = v
        self.summary.columnconfigure(3, weight=1)

        self.regions = ttk.LabelFrame(f, text="Memory regions", padding=PAD)
        self.regions.pack(fill="x", pady=(0, PAD))
        self.split_lbl = ttk.Label(f, text="", justify="left")
        self.split_lbl.pack(fill="x")

        sub = self.stats_nb = ttk.Notebook(f)
        sub.pack(fill="both", expand=True, pady=(PAD, 0))
        fr, self.t_art = make_tree(sub, ("Artifact", "Size", "Modified", "Path"), (300, 90, 150, 400),
                                   {"Size": "e"})
        sub.add(fr, text="Artifacts")
        self.t_art.bind("<Double-1>", self._open_artifact)
        fr, self.t_comp = make_tree(sub, ("Component", "Flash", "RAM"), (380, 100, 100),
                                    {"Flash": "e", "RAM": "e"})
        sub.add(fr, text="Components")
        fr, self.t_fsym = make_tree(sub, ("Flash symbol", "Size"), (560, 100), {"Size": "e"})
        sub.add(fr, text="Largest flash symbols")
        fr, self.t_rsym = make_tree(sub, ("RAM symbol", "Size"), (560, 100), {"Size": "e"})
        sub.add(fr, text="Largest RAM symbols")
        fr, self.t_stack = make_tree(sub, ("Function", "Stack", "Kind", "File"), (440, 80, 80, 200),
                                     {"Stack": "e"})
        sub.add(fr, text="Stack usage")
        fr, self.t_steps = make_tree(sub, ("Step", "Seconds", "Exit code"), (200, 100, 100),
                                     {"Seconds": "e", "Exit code": "e"})
        sub.add(fr, text="Steps")

    # ---- Problems
    def _build_problems_tab(self):
        f = ttk.Frame(self.nb, padding=PAD)
        self.nb.add(f, text="Problems")
        ttk.Label(f, text="Double-click to open the file (VS Code if installed).").pack(anchor="w")
        fr, self.t_prob = make_tree(f, ("Severity", "Location", "Message"), (80, 320, 600), height=20)
        fr.pack(fill="both", expand=True)
        self.t_prob.tag_configure("error", foreground=TAG_COLORS["error"])
        self.t_prob.tag_configure("warning", foreground=TAG_COLORS["warning"])
        self.t_prob.bind("<Double-1>", self._open_problem)

    # ---- History
    def _build_history_tab(self):
        f = ttk.Frame(self.nb, padding=PAD)
        self.nb.add(f, text="History")
        fr, self.t_hist = make_tree(f, ("Time", "Target", "Result", "Duration", "Warn", "Err", "Config",
                                        "Flash", "RAM"),
                                    (150, 200, 80, 70, 50, 50, 100, 110, 110),
                                    {"Duration": "e", "Warn": "e", "Err": "e", "Flash": "e", "RAM": "e"},
                                    height=20)
        fr.pack(fill="both", expand=True)
        ttk.Button(f, text="Clear history", command=self.clear_history).pack(anchor="e", pady=(PAD, 0))

    # ---- Environment
    def _build_env_tab(self):
        f = ttk.Frame(self.nb, padding=PAD)
        self.nb.add(f, text="Environment")
        form = ttk.LabelFrame(f, text="Toolchain locations (empty = auto-detect)", padding=PAD)
        form.pack(fill="x")
        form.columnconfigure(1, weight=1)
        self.env_vars = {}
        rows = [("arm_toolchain", "ARM GNU toolchain dir", True),
                ("zephyr_base", "Zephyr base (ZEPHYR_BASE)", True),
                ("zephyr_toolchain_variant", "Zephyr toolchain variant", False),
                ("zephyr_sdk", "Zephyr SDK dir", True),
                ("west", "west executable", False)]
        for r, (key, label, browse) in enumerate(rows):
            ttk.Label(form, text=label).grid(row=r, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=self.settings["env"].get(key, ""))
            self.env_vars[key] = var
            if key == "zephyr_toolchain_variant":
                ttk.Combobox(form, textvariable=var, values=("gnuarmemb", "zephyr"),
                             state="readonly", width=14).grid(row=r, column=1, sticky="w", padx=PAD)
            else:
                ttk.Entry(form, textvariable=var).grid(row=r, column=1, sticky="ew", padx=PAD)
            if browse:
                ttk.Button(form, text="Browse...", command=lambda v=var: self._browse_dir(v)).grid(row=r, column=2)
        btns = ttk.Frame(form)
        btns.grid(row=len(rows), column=0, columnspan=3, sticky="e", pady=(PAD, 0))
        ttk.Button(btns, text="Save", command=self.save_env).pack(side="left")
        ttk.Button(btns, text="Check tools", command=self.run_doctor).pack(side="left", padx=PAD)
        fr, self.t_env = make_tree(f, ("Tool", "Status", "Used for", "Version / path"),
                                   (170, 70, 170, 600), height=14)
        fr.pack(fill="both", expand=True, pady=(PAD, 0))
        self.t_env.tag_configure("bad", foreground=TAG_COLORS["error"])
        ttk.Label(f, text=f"Host: {E.HOST}   Repo: {E.REPO_ROOT}   Settings: {E.settings_dir()}",
                  foreground="#666").pack(anchor="w", pady=(PAD, 0))
        self.root.after(300, self.run_doctor)

    # ------------------------------------------------------------ options
    def _board_targets(self):
        return self.boards[max(self.board_cb.current(), 0)][2]

    def _select(self, target):
        idx = next(i for i, b in enumerate(self.boards) if b[0] == target.board_key)
        self.board_cb.current(idx)
        ts = self._board_targets()
        self.os_cb.configure(values=[t.os_label for t in ts])
        self.os_cb.current(ts.index(target))
        self._on_target()

    def _on_board(self):
        ts = self._board_targets()
        keep_os = self.target.os if self.target else None
        pick = next((t for t in ts if t.os == keep_os), ts[0])
        self.os_cb.configure(values=[t.os_label for t in ts])
        self.os_cb.current(ts.index(pick))
        self._on_target()

    def _on_target(self):
        if self.target is not None:
            try:
                self.settings["values"][self.target.id] = self.get_values(validate=False)
            except ValueError:
                pass
        self.target = self._board_targets()[max(self.os_cb.current(), 0)]
        self.settings["last_target"] = self.target.id
        self._render_options(O.merge(self.target, self.settings["values"].get(self.target.id)))
        t = self.target
        info = f"{t.description}.   MCU: {t.mcu}.   OS: {t.os_label}.   Build dir: {t.build_dir}"
        notes = [n for n in t.notes if E.HOST not in t.hosts or "experimental" not in n]
        self.info.configure(text=info + ("\n⚠ " + "\n⚠ ".join(notes) if notes else ""),
                            foreground=TAG_COLORS["error"] if t.overwrites_bootloader else "")
        hw = t.kind != "host"
        self.btn_flash.configure(state="normal" if hw and not self.busy else "disabled")
        self.refresh_stats(quiet=True)
        self.refresh_diagram()

    def _render_options(self, values: dict):
        for w in self.opts_frame.inner.winfo_children():
            w.destroy()
        self.vars.clear()
        self.text_widgets.clear()
        self.widgets = {}
        t = self.target
        for group in O.GROUPS:
            opts = [o for o in O.options_for(t) if o.group == group]
            if not opts:
                continue
            lf = ttk.LabelFrame(self.opts_frame.inner, text=group, padding=PAD)
            lf.pack(fill="x", padx=(0, PAD), pady=(0, PAD))
            lf.columnconfigure(1, weight=1)
            for r, o in enumerate(opts):
                self._option_row(lf, r, o, values.get(o.key))
        if "bootloader" in self.vars:  # grey out bootloader options when it is not built
            self.vars["bootloader"].trace_add("write", lambda *a: self._sync_bootloader_widgets())
            self._sync_bootloader_widgets()

    def _option_row(self, parent, row, o: O.Option, value):
        tip = o.help
        if o.key == "probe_device":
            tip += "\nDefault now: " + (P.default_device(self.vars["probe"].get_value(), self.target.mcu)
                                        if "probe" in self.vars else "")
        if o.kind == "bool":
            var = tk.BooleanVar(value=bool(value))
            w = ttk.Checkbutton(parent, text=o.label, variable=var)
            w.grid(row=row, column=0, columnspan=2, sticky="w", pady=1)
            var.get_value = var.get
        else:
            lbl = ttk.Label(parent, text=o.label)
            lbl.grid(row=row, column=0, sticky="nw", pady=2, padx=(0, PAD))
            Tooltip(lbl, tip)
            if o.kind == "enum":
                labels = [c[1] for c in o.choices]
                var = tk.StringVar(value=o.choice_label(value))
                w = ttk.Combobox(parent, textvariable=var, values=labels, state="readonly", width=30)
                rev = {c[1]: c[0] for c in o.choices}
                var.get_value = lambda var=var, rev=rev: rev.get(var.get(), var.get())
            elif o.kind == "int":
                var = tk.StringVar(value=str(value))
                w = ttk.Spinbox(parent, textvariable=var, from_=o.minimum or 0, to=o.maximum or 1 << 30,
                                width=10, increment=1)
                var.get_value = var.get
            elif o.kind == "text":
                var = None
                w = tk.Text(parent, height=4, width=28, font=self.mono)
                w.insert("1.0", value or "")
                self.text_widgets[o.key] = w
            else:
                var = tk.StringVar(value="" if value is None else str(value))
                w = ttk.Entry(parent, textvariable=var, width=26)
                var.get_value = var.get
            w.grid(row=row, column=1, sticky="ew", pady=2)
        Tooltip(w, tip)
        if var is not None:
            self.vars[o.key] = var
        self.widgets[o.key] = w

    def _sync_bootloader_widgets(self):
        on = bool(self.vars["bootloader"].get())
        for o in O.options_for(self.target):
            if o.group == "Bootloader" and o.key != "bootloader" and o.key in self.widgets:
                w = self.widgets[o.key]
                try:
                    if isinstance(w, tk.Text):
                        w.configure(state="normal" if on else "disabled",
                                    background="#ffffff" if on else "#e8e8e8")
                    elif isinstance(w, ttk.Combobox):
                        w.configure(state="readonly" if on else "disabled")
                    else:
                        w.configure(state="normal" if on else "disabled")
                except tk.TclError:
                    pass

    def get_values(self, validate: bool = True) -> dict:
        vals = {}
        for o in O.options_for(self.target):
            if o.key in self.text_widgets:
                vals[o.key] = self.text_widgets[o.key].get("1.0", "end").strip()
                continue
            var = self.vars.get(o.key)
            if var is None:
                continue
            raw = var.get_value()
            if o.kind == "hex":
                try:
                    raw = o.parse(raw)
                except ValueError:
                    if validate:
                        raise ValueError(f"'{o.label}': enter a hex number such as 0x1209, or leave it empty")
                    raw = ""
            elif o.kind == "int":
                try:
                    raw = o.parse(raw)
                except ValueError:
                    if validate:
                        raise ValueError(f"'{o.label}': enter a whole number"
                                         + (f" between {o.minimum} and {o.maximum}" if o.minimum is not None else ""))
                    raw = o.default_for(self.target)
            vals[o.key] = raw
        if not vals.get("build_dir"):
            vals["build_dir"] = self.target.build_dir
        return vals

    def apply_profile(self):
        try:
            cur = self.get_values(validate=False)
        except ValueError:
            cur = None
        self._render_options(O.apply_profile(self.target, self.profile_cb.get(), O.merge(self.target, cur)))
        self._set_status(f"Applied profile '{self.profile_cb.get()}'")

    def reset_defaults(self):
        self._render_options(O.defaults(self.target))
        self.profile_cb.current(0)
        self._set_status("Options reset to defaults for this target")

    # ------------------------------------------------------------ actions
    def _plan(self) -> BuildPlan | None:
        try:
            vals = self.get_values()
        except ValueError as exc:
            messagebox.showerror("Invalid option", str(exc), parent=self.root)
            return None
        self.settings["values"][self.target.id] = vals
        E.save_settings(self.settings)
        return make_plan(self.target, vals, self.settings["env"])

    def build(self):
        if self.busy:
            return
        plan = self._plan()
        if not plan:
            return
        if plan.errors:
            messagebox.showerror("Cannot build", "\n".join(plan.errors), parent=self.root)
            return
        self.console.delete("1.0", "end")
        for tree in (self.t_prob,):
            tree.delete(*tree.get_children())
        self.nb.select(0)
        self._start(plan, "build")

    def _start(self, plan: BuildPlan, kind: str):
        self.busy, self.busy_kind = True, kind
        self.t_start = time.time()
        self.progress["value"] = 0
        self.progress.configure(mode="determinate" if kind == "build" else "indeterminate")
        if kind != "build":
            self.progress.start(15)
        self._set_buttons()
        self._set_status({"build": "Building", "flash": "Flashing", "debug": "GDB server running"}[kind]
                         + f" {self.target.id}...")
        self.runner = BuildRunner(plan, on_line=lambda s, t: self.q.put(("line", s, t)),
                                  on_progress=lambda f, s: self.q.put(("progress", f, s)))

        def work():
            try:
                state = self.runner.run()
                rep = S.build_report(plan, state) if kind == "build" else None
                if rep:
                    S.record(rep)
                self.q.put(("done", kind, state.returncode, state.cancelled, rep))
            except Exception as exc:  # surface unexpected failures instead of hanging the UI
                self.q.put(("line", f"INTERNAL ERROR: {exc!r}", "error"))
                self.q.put(("done", kind, -1, False, None))

        threading.Thread(target=work, daemon=True).start()

    def stop(self):
        if self.runner:
            self.runner.stop()
            self._set_status("Stopping...")

    def clean(self):
        if self.busy:
            return
        vals = self.get_values(validate=False)
        d = (E.REPO_ROOT / vals["build_dir"]).resolve()
        if not d.exists():
            messagebox.showinfo("Clean", f"{d} does not exist.", parent=self.root)
            return
        if not safe_to_delete(d):
            messagebox.showerror("Clean", f"Refusing to delete {d}: not a build directory.", parent=self.root)
            return
        if messagebox.askyesno("Clean", f"Delete the build directory?\n\n{d}", parent=self.root):
            shutil.rmtree(d, ignore_errors=True)
            self._log(f"Deleted {d}", "step")
            self._set_status("Build directory deleted")
            self.refresh_stats(quiet=True)

    def open_bin(self):
        vals = self.get_values(validate=False)
        build = (E.REPO_ROOT / vals["build_dir"]).resolve()
        d = S.binary_dir(self.target, build)
        target = d if d.exists() else build if build.exists() else None
        if target is None:
            messagebox.showinfo("Open binary folder", f"Nothing built yet.\n{d}", parent=self.root)
            return
        try:
            E.open_path(target)
        except OSError as exc:
            messagebox.showerror("Open binary folder", str(exc), parent=self.root)

    def flash(self):
        if self.busy or self.target.kind == "host":
            return
        vals = self.get_values(validate=False)
        warning = P.flash_warning(self.target, vals)
        if warning and not messagebox.askyesno("Bootloader will be replaced", warning + "\n\nFlash anyway?",
                                               icon="warning", parent=self.root):
            return
        build = (E.REPO_ROOT / vals["build_dir"]).resolve()
        try:
            step = P.flash_step(self.target, build, vals, E.settings_dir())
        except (FileNotFoundError, ValueError) as exc:
            messagebox.showerror("Flash", str(exc), parent=self.root)
            return
        self.nb.select(0)
        self._start(BuildPlan(target=self.target, values=vals, build_dir=build, steps=[step]), "flash")

    def debug(self):
        if self.busy:
            return
        vals = self.get_values(validate=False)
        build = (E.REPO_ROOT / vals["build_dir"]).resolve()
        hint = P.gdb_hint(self.target, build, vals, self.settings["env"])
        self._clip(hint)
        if self.target.kind == "host":
            messagebox.showinfo("Debug", f"Run in a terminal (copied to clipboard):\n\n{hint}", parent=self.root)
            return
        try:
            step = P.debug_step(self.target, build, vals)
        except ValueError as exc:
            messagebox.showerror("Debug server", str(exc), parent=self.root)
            return
        self.nb.select(0)
        self._log(f"Attach GDB with (copied to clipboard):\n  {hint}", "note")
        self._start(BuildPlan(target=self.target, values=vals, build_dir=build, steps=[step]), "debug")

    def copy_commands(self):
        plan = self._plan()
        if not plan:
            return
        lines = plan.command_lines()
        self._clip("\n".join(lines))
        self.nb.select(0)
        self._log("Commands (copied to clipboard):", "step")
        for n in plan.notes:
            self._log("NOTE: " + n, "note")
        for e in plan.errors:
            self._log("ERROR: " + e, "error")
        for ln in lines:
            self._log("  " + ln, "")
        for p, content in plan.files.items():
            self._log(f"  --- {p.name} ---", "")
            for c in content.splitlines():
                self._log("    " + c, "")

    # ------------------------------------------------------------ queue / UI updates
    def _poll(self):
        batch, tag_prev, n = [], None, 0
        try:
            while n < 800:
                item = self.q.get_nowait()
                n += 1
                if item[0] == "line":
                    _, text, tag = item
                    if self.only_problems.get() and tag not in ("warning", "error", "step", "success"):
                        continue
                    if tag != tag_prev and batch:
                        self._insert(batch, tag_prev)
                        batch = []
                    batch.append(text)
                    tag_prev = tag
                elif item[0] == "progress":
                    self.progress["value"] = int(item[1] * 1000)
                    self.status.configure(text=f"{item[2]}: {item[1] * 100:.0f}%")
                elif item[0] == "done":
                    if batch:
                        self._insert(batch, tag_prev)
                        batch = []
                    self._finish(*item[1:])
        except queue.Empty:
            pass
        if batch:
            self._insert(batch, tag_prev)
        if self.busy:
            self.elapsed.configure(text=f"{time.time() - self.t_start:6.1f} s")
        self.root.after(40, self._poll)

    def _insert(self, lines, tag):
        at_end = self.console.yview()[1] > 0.999
        self.console.insert("end", "\n".join(lines) + "\n", tag or ())
        excess = int(self.console.index("end-1c").split(".")[0]) - 50000
        if excess > 0:
            self.console.delete("1.0", f"{excess}.0")
        if self.autoscroll.get() and at_end or self.autoscroll.get() and self.busy:
            self.console.see("end")

    def _log(self, text, tag=""):
        self._insert([text], tag)

    def _finish(self, kind, rc, cancelled, rep):
        self.busy = False
        self.progress.stop()
        self.progress.configure(mode="determinate")
        self.runner = None
        dur = time.time() - self.t_start
        if kind == "build":
            self.progress["value"] = 1000 if rc == 0 else self.progress["value"]
            self.report = rep
            self._show_report(rep)
            self._fill_problems(rep)
            self._load_history()
            if cancelled:
                self._set_status(f"Build cancelled after {dur:.1f} s")
            elif rc == 0:
                tot = rep.get("totals") or {}
                extra = (f" - flash {S.fmt_bytes(tot.get('flash_used'))} / {S.fmt_bytes(tot.get('flash_cap'))}"
                         if tot.get("flash_used") else "")
                self._set_status(f"Build succeeded in {dur:.1f} s, {rep['warnings']} warning(s){extra}")
                self.nb.select(1)
            else:
                self._set_status(f"Build FAILED (exit {rc}) - {rep['errors']} error(s)")
                if rep["errors"]:
                    self.nb.select(2)
        else:
            what = "Flash" if kind == "flash" else "GDB server"
            self._set_status(f"{what} {'stopped' if cancelled else 'finished' if rc == 0 else f'failed (exit {rc})'}")
        self._set_buttons()

    def _set_buttons(self):
        idle = "disabled" if self.busy else "normal"
        for b in (self.btn_build, self.btn_clean, self.btn_cmd, self.btn_debug):
            b.configure(state=idle)
        self.btn_flash.configure(state=idle if self.target and self.target.kind != "host" else "disabled")
        self.btn_stop.configure(state="normal" if self.busy else "disabled")
        for cb in (self.board_cb, self.os_cb):
            cb.configure(state="disabled" if self.busy else "readonly")

    def _set_status(self, text):
        self.status.configure(text=text)

    # ------------------------------------------------------------ statistics view
    def refresh_stats(self, quiet: bool = False):
        """Show statistics of whatever is in the build dir (no build)."""
        vals = self.get_values(validate=False)
        build = (E.REPO_ROOT / vals["build_dir"]).resolve()
        arts = S.artifacts(self.target, build)
        if not arts:
            self._show_report(None)
            if not quiet:
                messagebox.showinfo("Statistics", f"No build found in {build}", parent=self.root)
            return
        mem = S.memory_report(self.target, build)
        rep = {"label": self.target.label, "ok": None, "cancelled": False, "returncode": None, "duration": None,
               "steps": [], "units_compiled": None, "warnings": None, "errors": None, "diagnostics": [],
               "binary_dir": str(S.binary_dir(self.target, build)), "artifacts": arts, "memory": mem,
               "totals": S.memory_totals(mem), "stack": S.stack_usage(build), "previous": None,
               "bootloader_totals": S.memory_totals(S.memory_report(self.target, build, image="apx_bootloader"))
               if self.target.kind == "zephyr" else {},
               "target": self.target.id}
        self.report = rep
        self._show_report(rep)

    def _show_report(self, rep):
        for tree in (self.t_art, self.t_comp, self.t_fsym, self.t_rsym, self.t_stack, self.t_steps):
            tree.delete(*tree.get_children())
        for w in self.regions.winfo_children():
            w.destroy()
        self.split_lbl.configure(text="")
        for v in self.sum_labels.values():
            v.configure(text="-", foreground="")
        if not rep:
            self.stats_title.configure(text=f"{self.target.label}: not built yet")
            return
        L = self.sum_labels
        if rep["ok"] is None:
            self.stats_title.configure(text=f"{rep['label']} (existing build)")
            L["Result"].configure(text="(loaded from build dir)")
        else:
            self.stats_title.configure(text=rep["label"])
            res = "CANCELLED" if rep["cancelled"] else "SUCCESS" if rep["ok"] else f"FAILED (exit {rep['returncode']})"
            L["Result"].configure(text=res, foreground=TAG_COLORS["success" if rep["ok"] else "error"])
            L["Duration"].configure(text=f"{rep['duration']:.1f} s")
            L["Steps"].configure(text=", ".join(f"{s['name']} {s['seconds']:.1f}s" for s in rep["steps"]))
            L["Units compiled"].configure(text=str(rep["units_compiled"]))
            L["Warnings / errors"].configure(text=f"{rep['warnings']} / {rep['errors']}",
                                             foreground=TAG_COLORS["error"] if rep["errors"] else
                                             TAG_COLORS["warning"] if rep["warnings"] else "")
        tot, prev = rep.get("totals") or {}, rep.get("previous") or {}
        if tot.get("image") is not None:
            L["Image size"].configure(text=f"{tot['image']:,} B{S.fmt_delta(tot['image'], prev.get('image'))}")
        for key, used, cap in (("Flash", "flash_used", "flash_cap"), ("RAM", "ram_used", "ram_cap")):
            if tot.get(used) is not None:
                pct = 100 * tot[used] / tot[cap] if tot.get(cap) else 0
                L[key].configure(text=f"{S.fmt_bytes(tot[used])} / {S.fmt_bytes(tot[cap])} ({pct:.1f}%)"
                                 f"{S.fmt_delta(tot[used], prev.get(used))}")
        if tot.get("result"):
            L["Memory check"].configure(text=tot["result"], foreground=TAG_COLORS[
                "success" if tot["result"] == "PASS" else "error" if "FAIL" in tot["result"] else "warning"])
        btot = rep.get("bootloader_totals") or {}
        if btot.get("flash_used") is not None:
            pct = 100 * btot["flash_used"] / btot["flash_cap"] if btot.get("flash_cap") else 0
            L["Bootloader"].configure(text=f"{S.fmt_bytes(btot['flash_used'])} / {S.fmt_bytes(btot['flash_cap'])}"
                                           f" ({pct:.1f}%)")
        L["Binary dir"].configure(text=rep["binary_dir"])

        mem = rep.get("memory")
        warn = float(self.settings.get("warn_pct", 85))
        if mem:
            row = 0
            for r in mem["regions"]:
                if not r["used"]:
                    continue
                pct = 100 * r["used"] / r["capacity"] if r["capacity"] else 0
                style = "Red" if pct > 100 else "Amber" if pct >= warn else "Green"
                ttk.Label(self.regions, text=r["name"], width=10).grid(row=row, column=0, sticky="w")
                pb = ttk.Progressbar(self.regions, maximum=100, value=min(pct, 100), length=320,
                                     style=f"{style}.Horizontal.TProgressbar")
                pb.grid(row=row, column=1, sticky="ew", padx=PAD, pady=2)
                ttk.Label(self.regions, text=f"{S.fmt_bytes(r['used'])} / {S.fmt_bytes(r['capacity'])}  "
                          f"({pct:.1f}%)   free {S.fmt_bytes(max(r['capacity'] - r['used'], 0))}   "
                          f"@ 0x{r['origin']:08x}").grid(row=row, column=2, sticky="w")
                row += 1
            self.regions.columnconfigure(1, weight=1)
            fl, ra = mem.get("flash", {}), mem.get("ram", {})
            self.split_lbl.configure(text=(
                f"Flash: code {S.fmt_bytes(fl.get('code'))}, read-only data {S.fmt_bytes(fl.get('rodata'))}, "
                f".data init {S.fmt_bytes(fl.get('data_init'))}      RAM: .data {S.fmt_bytes(ra.get('data'))}, "
                f".bss {S.fmt_bytes(ra.get('bss'))}, stacks/heap/noinit {S.fmt_bytes(ra.get('noinit'))}"))
            for name, e in sorted(mem.get("components", {}).items(), key=lambda kv: -kv[1]["flash"]):
                self.t_comp.insert("", "end", values=(name, S.fmt_bytes(e["flash"]), S.fmt_bytes(e["ram"])))
            for s in mem.get("top_flash", []):
                self.t_fsym.insert("", "end", values=(s["name"], S.fmt_bytes(s["size"])))
            for s in mem.get("top_ram", []):
                self.t_rsym.insert("", "end", values=(s["name"], S.fmt_bytes(s["size"])))
        else:
            ttk.Label(self.regions, text="No memory report (host builds have none).").grid(row=0, column=0)
        for a in rep["artifacts"]:
            self.t_art.insert("", "end", values=(("★ " if a.get("fresh") and rep["ok"] else "") + a["name"],
                                                 S.fmt_bytes(a["size"]),
                                                 time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(a["mtime"])),
                                                 a["path"]))
        for s in rep.get("stack", []):
            self.t_stack.insert("", "end", values=(s["function"], f"{s['bytes']} B", s["qualifier"], s["file"]))
        for s in rep["steps"]:
            self.t_steps.insert("", "end", values=(s["name"], f"{s['seconds']:.2f}", s["returncode"]))

    def _fill_problems(self, rep):
        self.t_prob.delete(*self.t_prob.get_children())
        diags = sorted(rep["diagnostics"], key=lambda d: d["sev"] != "error")
        for d in diags:
            f = d["file"]
            try:
                f = str(Path(f).relative_to(E.REPO_ROOT))
            except ValueError:
                pass
            loc = f"{f}:{d['line']}" if d["line"] else f
            self.t_prob.insert("", "end", values=(d["sev"], loc, d["msg"]), tags=(d["sev"],))
        self.nb.tab(2, text=f"Problems ({rep['errors']}/{rep['warnings']})" if diags else "Problems")

    def _open_problem(self, _e):
        sel = self.t_prob.selection()
        if not sel:
            return
        loc = self.t_prob.set(sel[0], "Location")
        path, _, line = loc.rpartition(":")
        if not line.isdigit():
            path, line = loc, ""
        p = Path(path)
        if not p.is_absolute():
            for base in (E.REPO_ROOT, Path(self.report["build_dir"]) if self.report else E.REPO_ROOT):
                if (base / p).exists():
                    p = base / p
                    break
        if p.exists():
            E.open_in_editor(str(p), int(line) if line else None)

    def _open_artifact(self, _e):
        sel = self.t_art.selection()
        if sel:
            E.open_path(Path(self.t_art.set(sel[0], "Path")).parent)

    def open_mem_report(self):
        rep = self.report
        if not rep:
            return
        for a in rep["artifacts"]:
            if a["name"].endswith((".memory.txt", "memory_report.txt")):
                E.open_path(Path(a["path"]))
                return
        messagebox.showinfo("Memory report", "No memory report for this build.", parent=self.root)

    def export_stats(self):
        if not self.report:
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".json",
                                            initialfile=f"{self.target.id.replace(':', '_')}_stats.json",
                                            filetypes=[("JSON", "*.json")])
        if path:
            Path(path).write_text(json.dumps(self.report, indent=2, default=str))
            self._set_status(f"Statistics exported to {path}")

    # ------------------------------------------------------------ history / env
    def _load_history(self):
        self.t_hist.delete(*self.t_hist.get_children())
        for h in reversed(S.load_history()):
            res = "cancelled" if h.get("cancelled") else "ok" if h["ok"] else "FAILED"
            fl = f"{S.fmt_bytes(h['flash_used'])}" if h.get("flash_used") else "-"
            ra = f"{S.fmt_bytes(h['ram_used'])}" if h.get("ram_used") else "-"
            self.t_hist.insert("", "end", values=(h["time"].replace("T", " "), h["target"], res,
                                                  f"{h['duration']:.1f} s", h["warnings"], h["errors"],
                                                  h.get("config", ""), fl, ra))

    def clear_history(self):
        if messagebox.askyesno("History", "Delete the build history?", parent=self.root):
            try:
                (E.settings_dir() / "history.json").unlink()
            except OSError:
                pass
            self._load_history()

    def save_env(self):
        self.settings["env"].update({k: v.get().strip() for k, v in self.env_vars.items()})
        E.save_settings(self.settings)
        self._set_status("Environment settings saved")
        self.run_doctor()

    def run_doctor(self):
        self.t_env.delete(*self.t_env.get_children())
        self.t_env.insert("", "end", values=("checking...", "", "", ""))
        env_settings = {k: v.get().strip() for k, v in self.env_vars.items()}

        def work():
            rows = E.doctor(env_settings)
            self.root.after(0, lambda: self._show_doctor(rows))

        threading.Thread(target=work, daemon=True).start()
        self._load_history()

    def _show_doctor(self, rows):
        self.t_env.delete(*self.t_env.get_children())
        for r in rows:
            self.t_env.insert("", "end", values=(r["tool"], "OK" if r["ok"] else "missing", r["needed"], r["detail"]),
                              tags=() if r["ok"] or "optional" in r["needed"] else ("bad",))

    def _browse_dir(self, var):
        d = filedialog.askdirectory(parent=self.root, initialdir=var.get() or str(E.REPO_ROOT))
        if d:
            var.set(d)

    # ------------------------------------------------------------ misc
    def save_log(self):
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".log",
                                            initialfile=f"{self.target.id.replace(':', '_')}_build.log")
        if path:
            Path(path).write_text(self.console.get("1.0", "end"))

    def _clip(self, text):
        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    def _on_close(self):
        if self.busy and not messagebox.askyesno("Quit", "A build is running. Stop it and quit?", parent=self.root):
            return
        if self.runner:
            self.runner.stop()
        try:
            self.settings["values"][self.target.id] = self.get_values(validate=False)
        except ValueError:
            pass
        self.settings["geometry"] = self.root.geometry()
        E.save_settings(self.settings)
        self.root.destroy()


def run_gui() -> int:
    if E.IS_WIN:
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0
