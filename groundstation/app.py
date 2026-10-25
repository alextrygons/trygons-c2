"""Trygons C2 ground station (tkinter).

    python app.py                      choose the port in the window
    python app.py tcp:127.0.0.1:5760   connect to the simulator at once
    python app.py COM5                 connect to the board at once
    python app.py gw:trygons-gw:5761   through the gateway (Raspberry Pi), asks for the login
Options (the demo uses them): --login user:password (no login window),
--driver (open the DRIVER screen too), --charts FOLDER (other chart files).

Big, high-contrast layout for use in sunlight. Everything is drawn locally
(no internet needed): map, track, plot.
"""
import math
import os
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import program
import proto
import i18n
import settings
import alerts
import crewui
import monitors
from i18n import T, TE
from link import Link, list_ports
from gwclient import GatewayLink
import shorejoy
import syncrecv
import video
import driver
import chart
import mapview
import collections
import json
from tkinter import messagebox, simpledialog

try:
    import winsound
except ImportError:          # not Windows
    winsound = None

# --- look ----------------------------------------------------------------------
BG = "#ffffff"
FG = "#000000"
PANEL = "#e8e8e8"
RED = "#d00000"
ORANGE = "#ff8c00"
GREEN = "#007a00"
BLUE = "#0033cc"
F_HUGE = ("Segoe UI", 26, "bold")
F_BIG = ("Segoe UI", 20, "bold")
F_MED = ("Segoe UI", 14, "bold")
F_TXT = ("Segoe UI", 12)
MODE_COLORS = {"STANDBY": "#606060", "MANUAL": ORANGE, "HDG_LOCK": GREEN, "AUTO": "#7a00a0",
               "TRAINING": "#a06000", "REMOTE_MANUAL": BLUE, "BENCH_TEST": "#008080", "MANEUVER": "#b03060"}
PLOT_S = 120.0              # seconds shown in the plot
LAYOUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gs_layout.json")   # window places (not in git)

# Captain actions that change how the boat is driven (proto.DRIVING_CMDS).
# Through the gateway the GATEWAY enforces it (section F1): sent only after the
# driver screen's ACCEPT, refused when no driver screen is open, or - in
# SINGLE-OPERATOR mode - after the captain's own confirmation. On the direct
# USB laptop link the local driver window asks when it is open.
DRIVING_CMDS = proto.DRIVING_CMDS


def btn(parent, key, cmd, bg=PANEL, fg=FG, font=F_MED, width=None, **kw):
    """A button whose text is the language entry `key` (follows the language switch)."""
    b = tk.Button(parent, command=cmd, bg=bg, fg=fg, font=font, activebackground="#c0c0c0",
                  relief="raised", bd=3, padx=8, pady=4)
    i18n.bind(b, key, **kw)
    if width:
        b.configure(width=width)
    return b


def proposal_text(p):
    """A gateway proposal (collision avoidance / vision) in the screen language: the gateway
    sends it in English (audit log); the language file has the same sentence as a "gw." entry."""
    return i18n.from_en((p or {}).get("text", ""))


def joy_why(d):
    """Why the joystick is not armed / in fault, in the screen language."""
    return T("joywhy.%d" % d["joy_why"]) if i18n.has("joywhy.%d" % d["joy_why"], "en") else d["joy_why_text"]


def ui_code_name(code):
    """A USB joystick control code (proto.code_name) in the screen language."""
    if code == 0:
        return "-"
    if 1 <= code <= 64:
        c = code - 1
        return T("jcode.button", dev=T("jdev.%d" % (c // 32)), n=c % 32 + 1)
    if 65 <= code <= 80:
        c = code - 65
        return T("jcode.hat", dev=T("jdev.%d" % (c // 8)), dir=T("jhat.%d" % (c % 4)))
    return "code %d" % code


def ui_axis_name(code):
    if code >= 16:
        return "-"
    return T("jcode.axis", dev=T("jdev.%d" % (code // 8)), n=code % 8, name=proto.AXIS_NAMES[code % 8])


def lbl(parent, key, **opts):
    """A label whose text is the language entry `key`."""
    return i18n.bind(tk.Label(parent, **opts), key)


def chk(parent, key, **opts):
    return i18n.bind(tk.Checkbutton(parent, **opts), key)


class App:
    def __init__(self, root, address=None, login=None, charts_dir=None, open_driver=False, layout=None, cams=None):
        self.layout_path = layout or LAYOUT    # window places (gs_layout.json)
        self.auto_cams = cams           # camera preset to ask for once connected through the gateway (demo)
        self.root = root
        self.login = login              # (user, password) for "gw:" without asking (tests)
        self.link = None
        self.t = None                   # latest telemetry
        self.t_rx = 0.0
        self.hist = []                  # (t_s, heading, target, outboard)
        self.track = []                 # (lat, lon)
        self.origin = None
        self.params = {}
        self.pending = {}               # seq -> description
        self.prog = program.new_program()   # program in the editor
        self.pdlg = None                # power raise dialog
        self.prog_dl = []               # PROG_WP_VALUE frames being downloaded
        self.last_beep = 0.0
        self.gw_status = None           # gateway status (when connected through the gateway)
        self.sync_rx = None             # shore PC: the boat's sync arrives here (sync_receiver.json)
        rc = syncrecv.load_config()
        if rc and rc.get("root") and rc.get("boats"):
            try:
                os.makedirs(rc["root"], exist_ok=True)
                self.sync_rx = syncrecv.SyncReceiver(rc["root"], rc["boats"], rc.get("port", 5770),
                                                     rc.get("listen", "0.0.0.0")).start()
            except OSError as e:
                print("sync receiver not started: %s" % e)
        self.jmap = None                # joystick map as stored on the boat
        self.jm_edit = proto.default_joymap()   # map in the editor (Joystick tab)
        self.jm_load = True             # next JOYMAP_VALUE goes into the editor
        self.raw = None                 # last USB bridge frame (JOY_RAW)
        self.wiz = None                 # guided button-mapping routine
        self.sim_usb = {"x": 0.0, "lever": 0.5, "stick": True, "throttle": True, "fault": 0}
        self.driver = None              # the DRIVER screen (second monitor), section I
        self.charts = chart.Charts(charts_dir)  # offline depth charts (tools/chartprep.py -> charts/), section B
        self.chartview = mapview.ChartView(self.charts)
        self.single_op = False          # SINGLE-OPERATOR mode as the GATEWAY has it (section F1)
        self.gw_asks = {}               # driver questions from the gateway: id -> text
        self.cconf = None               # captain's own confirmation (single-operator mode)
        self.confirm_timeout_s = 10.0   # the driver must accept a captain action within this
        self.slam_hist = collections.deque(maxlen=300)   # (t, slamming peak g) from the telemetry
        self.layout = {}
        try:
            with open(self.layout_path, encoding="utf-8") as f:
                self.layout = json.load(f)
        except (OSError, ValueError):
            pass
        self._layout_job = None

        self.title_extra = ""           # "ON BOARD - " etc. (--title)
        self.boat = None                # the boat chosen in the boat list (boats/), None = typed address
        self.boat_ok = False            # its name + live camera confirmed: HELM may be taken (CLAUDE.md F)
        self.boatsel = None             # the boat list dialog
        self.boatconf = None            # the boat confirmation dialog
        self.screen_shown = 2           # one monitor: which screen F11 shows (1 driving, 2 navigation)
        self.mapwin = None              # the full map of screen 2 (built into the Map tab)
        self.lamps = alerts.MasterLamps()   # MASTER WARNING / CAUTION (driver HUD): acknowledged = silenced here
        self.sound = None               # sound + voice (section D), made in _build_sound
        root.title(T("cap.title"))
        i18n.on_change(self.lang_changed)
        root.configure(bg=BG)
        if self.layout.get("captain"):
            root.geometry(self.layout["captain"])       # where it was last time (which monitor)
        else:
            root.geometry("1500x900")
            try:
                root.state("zoomed")    # maximised (Windows)
            except tk.TclError:
                pass
        style = ttk.Style()
        style.configure("Treeview", font=F_TXT, rowheight=26)
        style.configure("Treeview.Heading", font=F_MED)
        style.configure("TNotebook.Tab", font=("Segoe UI", 12, "bold"), padding=(5, 5))

        self._build_top()
        self._build_banners()
        body = tk.Frame(root, bg=BG)
        body.pack(fill="both", expand=True)
        self._build_values(body)
        self._build_controls(body)
        self._build_tabs(body)
        self.msg = tk.Label(root, text=T("cap.not_connected"), font=F_MED, bg="#202020", fg="#ffffff", anchor="w")
        self.msg.pack(fill="x", side="bottom")
        self._build_menu()

        if address:
            self.port_var.set(address)
            root.after(200, self.connect)
        root.after(50, self.poll)
        root.after(500, self.alarm_tick)
        root.after(1000, self.driver_beat)
        root.protocol("WM_DELETE_WINDOW", self.quit)
        for k in ("<F12>", "<Control-Tab>"):
            root.bind(k, lambda e: self.swap_screens())
        root.bind("<F11>", lambda e: self.toggle_screen())
        root.bind("<Configure>", lambda e: self.layout_changed())
        monitors.bind_keys(root, self)
        if self.layout.get("driver_open") or open_driver:
            root.after(600, self.open_driver)

    # --- language (section E) --------------------------------------------------------
    def lang_changed(self):
        """Live language switch: titles, tabs, list headings and the values drawn every second."""
        self.root.title(self.title_extra + T("cap.title"))
        self.lang_btns()
        if hasattr(self, "nb"):
            for i, key in enumerate(self.tab_keys):
                self.nb.tab(i, text=T(key))
        if hasattr(self, "ptree"):
            self.ptree.heading("#0", text=T("cap.param"))
            for c in ("value", "min", "max"):
                self.ptree.heading(c, text=T("cap.param_" + c))
        if hasattr(self, "ais_tree"):
            for i, (c, key, _w) in enumerate(self.AIS_COLS):
                self.ais_tree.heading("#0" if i == 0 else c, text=T(key) if key else "")
        if hasattr(self, "pg_tree"):
            for c in ("lat", "lon", "kn", "radius", "action"):
                self.pg_tree.heading(c, text=T("cap.pg_" + c))
            self.prog_combos()
        if hasattr(self, "sim_fault_cb"):
            self.sim_fault_cb.configure(values=[T(k) for k in self.SIM_FAULTS])
        self._build_menu()
        self.conn_btn.configure(text=T("cap.disconnect") if self.link else T("cap.connect"))
        self.so_btn.configure(text=T("cap.single_op_on") if self.single_op else T("cap.single_op"))
        if self.t:
            self.draw()
        self.draw_bench()
        self.draw_ais()
        self.draw_vision()
        self.draw_network()

    def set_lang(self, lang):
        i18n.set_lang(lang)

    def master_ack(self):
        """MASTER WARNING / CAUTION acknowledged on the driver screen: lamps off and the repeating sound
        silenced ON THIS PC. Nothing is sent to the boat (its ALARM ACK stays the captain's button)."""
        self.lamps.ack()
        if self.sound:
            self.sound.ack()

    def lang_btns(self):
        for code, b in getattr(self, "lang_b", {}).items():
            on = code == i18n.lang()
            b.configure(bg="#0033cc" if on else PANEL, fg="#ffffff" if on else FG,
                        relief="sunken" if on else "raised")

    def _build_menu(self):
        """Menu bar: Windows (open, move to a monitor), Language."""
        mb = tk.Menu(self.root)
        wm = tk.Menu(mb, tearoff=0)
        wm.add_command(label=T("menu.driver"), command=lambda: self.close_driver() if self.driver else self.open_driver())
        wm.add_separator()
        mons = monitors.list_monitors(self.root)
        for name, getw in (("captain", lambda: self.root), ("driver", lambda: self.driver and self.driver.w)):
            sub = tk.Menu(wm, tearoff=0)
            for i, m in enumerate(mons):
                sub.add_command(label=T("menu.monitor_n", n=i + 1, w=m[2] - m[0], h=m[3] - m[1]) +
                                ("   Ctrl+Shift+%d" % (i + 1) if i < 9 else ""),
                                command=lambda i=i, getw=getw: monitors.move_to(getw(), i, self))
            wm.add_cascade(label=T("menu.move_" + name), menu=sub)
        wm.add_separator()
        wm.add_command(label=T("menu.swap"), command=self.swap_screens)
        mb.add_cascade(label=T("menu.windows"), menu=wm)
        lm = tk.Menu(mb, tearoff=0)
        for code in i18n.LANGS:
            lm.add_command(label=T("menu.lang_" + code), command=lambda c=code: self.set_lang(c))
        mb.add_cascade(label=T("menu.language"), menu=lm)
        self.root.configure(menu=mb)
        self.menubar = mb

    # --- layout ------------------------------------------------------------------
    def _build_top(self):
        top = tk.Frame(self.root, bg=PANEL)
        top.pack(fill="x")
        btn(top, "boat.button", self.open_boats, font=F_MED, bg="#ffe080").pack(side="left", padx=(6, 2))
        lbl(top, "cap.connection", font=F_MED, bg=PANEL).pack(side="left", padx=6)
        self.port_var = tk.StringVar(value="tcp:127.0.0.1:5760")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, values=list_ports(), width=24, font=F_MED)
        self.port_box.pack(side="left", padx=4)
        btn(top, "cap.refresh", lambda: self.port_box.configure(values=list_ports()), font=F_TXT).pack(side="left")
        self.conn_btn = btn(top, "cap.connect", self.connect, bg="#b0ffb0")
        self.conn_btn.pack(side="left", padx=6)
        self.link_lbl = tk.Label(top, text=T("cap.link_none"), font=F_MED, bg=PANEL)
        self.link_lbl.pack(side="left", padx=12)
        top1 = top
        top = tk.Frame(self.root, bg=PANEL)          # second row: HELM, crew, profile, screens
        top.pack(fill="x")
        self.helm_lbl = tk.Label(top, text=T("cap.helm_none"), font=F_MED, bg=PANEL)
        self.helm_lbl.pack(side="left", padx=8)
        btn(top, "cap.helm_take", lambda: self.cmd(proto.T_HELM, proto.p_helm(0), TE("what.take_helm")),
            font=F_TXT).pack(side="left")
        btn(top, "cap.helm_release", self.helm_release, font=F_TXT).pack(side="left", padx=2)
        self.crew_lbl = tk.Label(top, text="", font=F_MED, bg=PANEL)
        self.crew_lbl.pack(side="left", padx=6)
        btn(top, "crew.swap", self.crew_swap, font=F_TXT).pack(side="left", padx=2)
        self.prof_lbl = tk.Label(top, text="", font=F_MED, bg=PANEL)
        self.prof_lbl.pack(side="left", padx=6)
        btn(top, "cap.driver_screen", lambda: self.close_driver() if self.driver else self.open_driver(),
            font=F_TXT, bg="#c0d8ff").pack(side="left", padx=4)
        self.so_btn = btn(top, "cap.single_op", self.single_op_toggle, font=F_TXT)
        self.so_btn.pack(side="left", padx=2)
        lbl(top, "cap.f12_hint", font=F_TXT, bg=PANEL, fg="#606060").pack(side="left")
        self.lang_b = {}
        for code in reversed(i18n.LANGS):                  # the language switch: ΕΛ / EN, right side
            b = tk.Button(top1, text=i18n.NAMES[code], font=F_MED, width=3, bd=3,
                          command=lambda c=code: self.set_lang(c))
            b.pack(side="right", padx=(0, 4))
            self.lang_b[code] = b
        self.lang_btns()
        self.log_lbl = tk.Label(top1, text="", font=F_TXT, bg=PANEL)
        self.log_lbl.pack(side="right", padx=8)

    def _build_banners(self):
        self.banners = tk.Frame(self.root, bg=BG)       # empty = zero height
        self.banners.pack(fill="x")
        self.banners.columnconfigure(0, weight=1)
        self.joy_banner = tk.Label(self.banners, text=T("cap.joy_in_control"), font=F_HUGE, bg=ORANGE, fg=FG)
        self.alarm_banner = tk.Label(self.banners, text="", font=F_MED, bg=RED, fg="#ffffff")
        self.pick_banner = tk.Label(self.banners, text="", font=F_BIG, bg="#ffd000", fg=FG)
        self.sl_banner = tk.Label(self.banners, text="", font=F_BIG, bg="#b00060", fg="#ffffff")
        self.so_banner = lbl(self.banners, "cap.so_banner", font=F_BIG, bg="#7000a0", fg="#ffffff")
        self.ft_banner = lbl(self.banners, "cap.ft_banner", font=F_BIG, bg="#ff00ff", fg="#ffffff", wraplength=1100)
        self.unc_banner = lbl(self.banners, "cap.unc_banner", font=F_BIG, bg="#004080", fg="#ffffff", wraplength=1400)
        self.ll_banner = tk.Label(self.banners, text="", font=F_BIG, bg=RED, fg="#ffffff", wraplength=1400)
        self.boat_banner = tk.Label(self.banners, text="", font=F_BIG, bg="#ffe080", fg=FG, wraplength=1400)
        self.alarm_ack = btn(self.banners, "cap.ack_alarm",
                             lambda: self.cmd(proto.T_ALARM_ACK, b"", TE("what.alarm_ack")), font=F_TXT)

    def show(self, widget, on, **grid):
        if on and not widget.winfo_manager():
            widget.grid(**grid)
        elif not on and widget.winfo_manager():
            widget.grid_remove()

    def _build_values(self, body):
        f = tk.Frame(body, bg=BG, bd=2, relief="groove")
        f.pack(side="left", fill="y", padx=4, pady=4)
        self.mode_lbl = tk.Label(f, text="---", font=F_HUGE, bg="#606060", fg="#ffffff", width=10)
        self.mode_lbl.pack(fill="x", pady=(0, 2))
        self.power_lbl = tk.Label(f, text=T("cap.power_none"), font=F_BIG, bg="#d0ffd0", fg=FG)
        self.power_lbl.pack(fill="x")
        pw = tk.Frame(f, bg=BG)
        pw.pack(fill="x", pady=(0, 6))
        btn(pw, "cap.raise_fwd", lambda: self.power_dialog(0), font=F_TXT).pack(side="left", expand=True, fill="x")
        btn(pw, "cap.raise_rev", lambda: self.power_dialog(1), font=F_TXT).pack(side="left", expand=True, fill="x")
        btn(pw, "cap.lower_now", lambda: self.cmd(proto.T_POWER_LOWER, b"", TE("what.power_lower")), bg="#b0ffb0",
            font=F_TXT).pack(side="left", expand=True, fill="x")
        self.vals = {}
        rows = ["heading", "target", "yaw", "outboard", "throttle", "gear", "gps", "pos", "sog", "src", "unc", "drift",
                "current", "cruise", "wp", "train"]
        grid = tk.Frame(f, bg=BG)
        grid.pack(fill="x")
        for i, k in enumerate(rows):
            lbl(grid, "cap.v_" + k, font=F_TXT, bg=BG, anchor="w").grid(row=i, column=0, sticky="w", padx=4)
            big = k in ("heading", "target", "outboard", "throttle", "gear")
            small = k in ("pos", "src", "drift", "wp", "train", "unc", "current", "cruise", "sog")
            v = tk.Label(grid, text="-", font=F_BIG if big else F_TXT if small else F_MED, bg=BG, anchor="w")
            v.grid(row=i, column=1, sticky="w", padx=4)
            self.vals[k] = v

    def _build_controls(self, body):
        f = tk.Frame(body, bg=BG, bd=2, relief="groove")
        f.pack(side="left", fill="y", padx=4, pady=4)

        lbl(f, "cap.mode_lit", font=F_TXT, bg=BG).pack(anchor="w")
        m = tk.Frame(f, bg=BG)
        m.pack(fill="x")
        btn(m, "cap.btn_standby", lambda: self.mode(proto.MODE_STANDBY), bg=RED, fg="#ffffff", font=F_BIG).grid(
            row=0, column=0, columnspan=3, sticky="ew", pady=2)
        self.mode_btns = {}
        for i, (name, cmd) in enumerate([("MANUAL", self.info_manual), ("HDG_LOCK", self.info_hdg),
                                         ("AUTO", self.auto_start)]):
            b = btn(m, "cap.btn_" + name.lower(), cmd, bg=PANEL, font=F_MED)
            b.grid(row=1, column=i, sticky="nsew", padx=1, pady=1)
            self.mode_btns[name] = b
        for i in range(3):
            m.columnconfigure(i, weight=1)

        lbl(f, "cap.auto_prog", font=F_TXT, bg=BG, wraplength=380, justify="left").pack(anchor="w", pady=(6, 0))
        a = tk.Frame(f, bg=BG)
        a.pack(fill="x")
        btn(a, "cap.pause", lambda: self.cmd(proto.T_AUTO_PAUSE, b"", TE("what.auto_pause")), font=F_TXT).pack(
            side="left")
        btn(a, "cap.resume", lambda: self.cmd(proto.T_AUTO_RESUME, b"", TE("what.auto_resume")), font=F_TXT).pack(
            side="left", padx=2)
        btn(a, "cap.skip", lambda: self.cmd(proto.T_AUTO_SKIP, b"", TE("what.auto_skip")), font=F_TXT).pack(
            side="left")
        self.auto_lbl = tk.Label(f, text="-", font=F_TXT, bg=BG, justify="left", anchor="w", wraplength=380)
        self.auto_lbl.pack(fill="x")

        lbl(f, "cap.hdg_title", font=F_TXT, bg=BG).pack(anchor="w", pady=(6, 0))
        h = tk.Frame(f, bg=BG)
        h.pack()
        for d in (-10, -1, 1, 10):
            btn(h, "cap.hdg_step", lambda d=d: self.cmd(proto.T_HEADING_ADJ, proto.p_heading(d),
                                                        TE("what.heading", d="%+d" % d)),
                font=F_BIG, width=3, d="%+d" % d).pack(side="left", padx=2)

        lbl(f, "cap.sea_state", font=F_TXT, bg=BG).pack(anchor="w", pady=(6, 0))
        cs = tk.Frame(f, bg=BG)
        cs.pack(fill="x")
        self.sea_var = tk.IntVar(value=2)
        for sv in range(1, 6):
            tk.Radiobutton(cs, text=str(sv), variable=self.sea_var, value=sv, font=F_MED, bg=BG, indicatoron=False,
                           width=2, command=lambda: self.cmd(proto.T_SEA_STATE, proto.p_sea_state(self.sea_var.get()),
                                                             TE("what.sea_state", n=self.sea_var.get()))).pack(
                side="left")

        lbl(f, "cap.training", font=F_TXT, bg=BG).pack(anchor="w", pady=(6, 0))
        tr = tk.Frame(f, bg=BG)
        tr.pack(fill="x")
        btn(tr, "cap.start", lambda: self.cmd(proto.T_TRAINING, proto.p_training(0), TE("what.training_start")),
            font=F_TXT).pack(side="left")
        lbl(tr, "cap.full_angle", font=F_TXT, bg=BG).pack(side="left", padx=(6, 0))
        self.full_var = tk.StringVar(value="0")
        tk.Entry(tr, textvariable=self.full_var, font=F_MED, width=4).pack(side="left")
        tr = tk.Frame(f, bg=BG)
        tr.pack(fill="x")
        btn(tr, "cap.accept", self.train_accept, bg="#b0ffb0", font=F_TXT).pack(side="left", padx=2)
        btn(tr, "cap.reject", lambda: self.cmd(proto.T_TRAINING, proto.p_training(2), TE("what.training_reject")),
            bg="#ffb0b0", font=F_TXT).pack(side="left")

        # simulator only: the panel buttons and the joystick CONFIRM
        self.sim_frame = tk.Frame(f, bg="#fff4c0")
        lbl(self.sim_frame, "cap.sim_panel", font=F_TXT, bg="#fff4c0").pack(anchor="w")
        sf = tk.Frame(self.sim_frame, bg="#fff4c0")
        sf.pack(fill="x")
        btn(sf, "cap.sim_hdg", lambda: self.cmd(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK),
                                                TE("what.sim_hdg")), font=F_TXT).pack(side="left")
        btn(sf, "cap.sim_standby", lambda: self.cmd(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY),
                                                    TE("what.sim_standby")), font=F_TXT).pack(side="left", padx=2)
        btn(sf, "cap.sim_confirm", self.sim_confirm, font=F_TXT).pack(side="left")
        Y = "#fff4c0"
        lbl(self.sim_frame, "cap.sim_usb", font=F_TXT, bg=Y).pack(anchor="w", pady=(4, 0))
        su = tk.Frame(self.sim_frame, bg=Y)
        su.pack(fill="x")
        lbl(su, "cap.sim_lever", font=F_TXT, bg=Y).grid(row=0, column=0)
        self.sim_lever = tk.Scale(su, from_=100, to=0, orient="vertical", length=110, bg=Y, showvalue=True,
                                  command=lambda v: self.sim_usb_set("lever", float(v) / 100.0))
        self.sim_lever.set(50)
        self.sim_lever.grid(row=1, column=0, rowspan=3)
        lbl(su, "cap.sim_stickx", font=F_TXT, bg=Y).grid(row=0, column=1, columnspan=3)
        self.sim_sx = tk.Scale(su, from_=-100, to=100, orient="horizontal", length=170, bg=Y,
                               command=lambda v: self.sim_usb_set("x", float(v) / 100.0))
        self.sim_sx.grid(row=1, column=1, columnspan=3)
        for i, (name, k) in enumerate((("take", proto.JM_TAKE), ("auto", proto.JM_AUTO), ("hdg", proto.JM_HDG_LOCK))):
            btn(su, "cap.sim_b_" + name, lambda k=k: self.sim_usb_press(k), font=F_TXT).grid(row=2, column=1 + i,
                                                                                           sticky="ew")
        for i, (name, k) in enumerate((("stby", proto.JM_STANDBY), ("m1", 5), ("p1", 6))):
            btn(su, "cap.sim_b_" + name, lambda k=k: self.sim_usb_press(k), font=F_TXT).grid(row=3, column=1 + i,
                                                                                           sticky="ew")
        uf = tk.Frame(self.sim_frame, bg=Y)
        uf.pack(fill="x")
        self.sim_stick_on = tk.BooleanVar(value=True)
        self.sim_thr_on = tk.BooleanVar(value=True)
        chk(uf, "cap.sim_stick_plugged", variable=self.sim_stick_on, bg=Y, font=F_TXT,
            command=lambda: self.sim_usb_set("stick", self.sim_stick_on.get())).pack(side="left")
        chk(uf, "cap.sim_thr_plugged", variable=self.sim_thr_on, bg=Y, font=F_TXT,
            command=lambda: self.sim_usb_set("throttle", self.sim_thr_on.get())).pack(side="left")
        uf = tk.Frame(self.sim_frame, bg=Y)
        uf.pack(fill="x")
        self.sim_fault_cb = ttk.Combobox(uf, values=[T(k) for k in self.SIM_FAULTS], width=16, state="readonly",
                                         font=F_TXT)
        self.sim_fault_cb.current(0)
        self.sim_fault_cb.bind("<<ComboboxSelected>>", lambda e: self.sim_usb_set(
            "fault", self.SIM_FAULTS[self.SIM_FAULT_KEYS[self.sim_fault_cb.current()]]))
        self.sim_fault_cb.pack(side="left")

    SIM_FAULTS = {"cap.sim_f_ok": 0, "cap.sim_f_noframes": 1, "cap.sim_f_corrupt": 2, "cap.sim_f_stuck": 4}
    SIM_FAULT_KEYS = list(SIM_FAULTS)

    def _build_joystick(self, parent):
        j = tk.Frame(parent, bg=BG, bd=2, relief="groove")
        j.pack(fill="x", pady=(0, 4))
        lbl(j, "cap.joystick", font=F_MED, bg=BG).pack(side="left", anchor="n", padx=4)
        self.joy_cv = tk.Canvas(j, width=130, height=130, bg="#f8f8f8", highlightthickness=2,
                                highlightbackground=FG)
        self.joy_cv.pack(side="left", pady=2)
        self.lever_cv = tk.Canvas(j, width=54, height=130, bg="#f8f8f8", highlightthickness=2,
                                  highlightbackground=FG)
        self.lever_cv.pack(side="left", pady=2, padx=(4, 0))
        jr = tk.Frame(j, bg=BG)
        jr.pack(side="left", padx=8, fill="y")
        self.joy_src = tk.Label(jr, text="-", font=F_TXT, bg=BG, justify="left")
        self.joy_src.pack(anchor="w")
        self.joy_status = tk.Label(jr, text="-", font=F_MED, bg=BG, justify="left")
        self.joy_status.pack(anchor="w")
        self.joy_btns = tk.Label(jr, text="-", font=F_MED, bg=BG)
        self.joy_btns.pack(anchor="w")
        jb = tk.Frame(j, bg=BG)
        jb.pack(side="left", padx=4)
        lbl(jb, "cap.analog_joy", font=F_TXT, bg=BG).pack(anchor="w")
        btn(jb, "cap.cal_start", lambda: self.cmd(proto.T_JOY_CAL, proto.p_joy_cal(0), TE("what.calib_start")),
            font=F_TXT).pack(fill="x")
        btn(jb, "cap.cal_finish", lambda: self.cmd(proto.T_JOY_CAL, proto.p_joy_cal(1), TE("what.calib_finish")),
            font=F_TXT).pack(fill="x")

    def _build_cameras(self, ct):
        """Camera presets, health / bitrate, front preview (FRONT-1, automatic FRONT-2)."""
        self.cam_ack = None
        self.vstreams = {}
        self.front_sel = video.FrontSelector()
        self.cam_photo = None
        top = tk.Frame(ct, bg=BG)
        top.pack(fill="x", pady=4)
        for pr, key in (("Full", "cap.cam_full"), ("Front only", "cap.cam_front"), ("Mirrors only", "cap.cam_mirrors"),
                        ("All off", "cap.cam_off")):
            btn(top, key, lambda pr=pr: self.cam_preset(pr), font=F_TXT).pack(side="left", padx=2)
        self.cam_override = tk.BooleanVar(value=False)
        chk(top, "cap.cam_override", variable=self.cam_override, font=F_TXT, bg=BG).pack(side="left", padx=8)
        top = tk.Frame(ct, bg=BG)
        top.pack(fill="x")
        btn(top, "cap.cam_inet_on", lambda: self.cam_internet(True), font=F_TXT).pack(side="left", padx=2)
        btn(top, "cap.cam_inet_off", lambda: self.cam_internet(False), font=F_TXT).pack(side="left", padx=2)
        rec = tk.Frame(ct, bg=BG)
        rec.pack(fill="x", pady=2)
        i18n.bind(tk.Button(rec, font=F_BIG, bg="#d00000", fg="white", width=8,
                            command=lambda: self.video_save("captain")), "cap.save").pack(side="left", padx=4)
        lbl(rec, "cap.ring_min", font=F_TXT, bg=BG).pack(side="left", padx=(12, 2))
        self.ring_var = tk.StringVar(value="10")
        cb = ttk.Combobox(rec, textvariable=self.ring_var, values=["5", "10", "20", "30", "60"], width=4,
                          state="readonly", font=F_TXT)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda e: (self.ring_set(), self.root.focus_set()))
        self.ring_cb = cb
        self.rec_lbl = tk.Label(rec, text="", font=F_TXT, bg=BG, justify="left", anchor="w", wraplength=700)
        self.rec_lbl.pack(side="left", padx=10)
        self.sj_save_was = False
        mid = tk.Frame(ct, bg=BG)
        mid.pack(fill="both", expand=True)
        self.cam_cv = tk.Canvas(mid, width=640, height=360, bg="#202020", highlightthickness=0)
        self.cam_cv.pack(side="top", anchor="w", padx=4, pady=4)
        self.cam_lbl = lbl(mid, "cap.cam_connect_gw", font=("Consolas", 12), bg=BG, justify="left", anchor="nw")
        self.cam_lbl.pack(side="top", fill="x", padx=6)
        # obstacle detection (eWaSR on the Pi): stages + status
        vf = tk.Frame(ct, bg=BG, bd=2, relief="groove")
        vf.pack(fill="x", padx=4, pady=4)
        lbl(vf, "cap.vis_title", font=F_MED, bg=BG).pack(anchor="w", padx=4)
        vf = tk.Frame(vf, bg=BG)
        vf.pack(fill="x")
        self.vis_vars = {}
        for k in ("stage1", "stage2", "stage3"):
            var = tk.BooleanVar(value=k != "stage3")
            self.vis_vars[k] = var
            chk(vf, "cap.vis_" + k, variable=var, font=F_TXT, bg=BG,
                command=lambda k=k: self.vision_stage(k)).pack(side="left", padx=4)
        btn(vf, "cap.vis_clear", lambda: self.vision_send({"t": "vision_clear"}), font=F_TXT).pack(side="left")
        self.vis_lbl = tk.Label(ct, text="", font=F_TXT, bg=BG, anchor="w", justify="left")
        self.vis_lbl.pack(fill="x", padx=6)
        self.root.after(100, self.cam_tick)

    def vision_send(self, msg):
        if not isinstance(self.link, GatewayLink):
            return self.say(T("cap.vis_only_gw"), RED)
        self.link.send_msg(msg)

    def vision_stage(self, k):
        on = self.vis_vars[k].get()
        msg = {"t": "vision_stage", "stage": k, "on": on}
        if k == "stage3" and on and self.single_op:          # single operator: the own confirmation
            self.vis_vars[k].set(False)
            return self.captain_confirm(T("cap.vis_stage3_confirm"),
                                        lambda: self.vision_send(dict(msg, confirm="self")))
        self.vision_send(msg)

    def draw_vision(self):
        v = (self.gw_status or {}).get("vision")
        if not v or not hasattr(self, "vis_lbl"):
            return
        for k, var in self.vis_vars.items():                # show the gateway's real setting
            if var.get() != v["stages"].get(k):
                var.set(v["stages"].get(k))
        txt = T("cap.vis_status", state=T("vis." + v["state"]), reason=i18n.from_en(v["reason"]), fps="%.1f" % v["fps"],
                model=v.get("model") or "-", horizon=v.get("horizon"), restarts=v.get("restarts", 0))
        if v.get("cap_kn") is not None:
            txt += "\n" + T("cap.vis_cap", kn="%.0f" % v["cap_kn"]) + (T("cap.vis_cap_held") if v.get("cap_held") else "")
        obs = [o for o in v.get("obstacles") or [] if o.get("confirmed")]
        if obs:
            txt += "\n" + "   ".join("%03.0f ~%s m%s%s" % (o["bearing"], o["dist"] or T("hud.far"),
                                                          T("cap.vis_in_path") if o["in_corridor"] else "",
                                                          " =" + o["ais"] if o.get("ais") else "") for o in obs[:6])
        self.vis_lbl.configure(text=txt, fg=RED if v["state"] in ("OFFLINE", "DEGRADED") or v.get("cap_held")
                               else "black")

    def drv_vision(self):
        """HEADING LOCK + STAGE 3: the slow-down proposal on the driver screen (an acknowledgement:
        the throttle stays with the driver)."""
        p = ((self.gw_status or {}).get("vision") or {}).get("proposal")
        tag = ("vision", p["id"]) if p and not p.get("acked") else None
        old = getattr(self, "vprop_tag", None)
        if old and old != tag and self.driver:
            self.driver.cancel(old)
        self.vprop_tag = tag
        if not tag or old == tag or not self.driver:
            return
        self.event(T("cap.ev_vision", text=i18n.from_en(p["text"])))
        ack = lambda why: self.vision_send({"t": "vision_ack", "id": p["id"]})       # noqa: E731
        self.driver.ask(i18n.from_en(p["text"]) + "\n" + T("drv.throttle_yours"), ack, ack, 60.0, tag=tag,
                        head=T("drv.head_vision"))

    def cam_preset(self, preset):
        if not isinstance(self.link, GatewayLink):
            return self.say(T("cap.cams_only_gw"), RED)
        self.link.send_msg({"t": "cams", "preset": preset, "override": self.cam_override.get()})

    def cam_internet(self, on):
        if isinstance(self.link, GatewayLink):
            self.link.send_msg({"t": "cam_internet", "on": on, "minutes": 30})

    def ais_list(self):
        """AIS targets from the gateway (section A), [] without one."""
        return ((self.gw_status or {}).get("ais") or {}).get("list") or []

    def video_save(self, source):
        """SAVE: the gateway keeps the ring window + 2 min after (all cameras)."""
        if not isinstance(self.link, GatewayLink):
            return self.say(T("cap.save_only_gw"), RED)
        self.link.send_msg({"t": "save", "source": source})
        if self.sound:
            self.sound.feedback("fb.shutter")

    def ring_set(self):
        if isinstance(self.link, GatewayLink):
            self.link.send_msg({"t": "rec", "ring_min": int(self.ring_var.get())})

    def on_save_ack(self, d):
        self.say(i18n.from_en(d.get("text", "")), GREEN if d.get("ok") else RED)
        self.event(T("cap.ev_video", text=i18n.from_en(d.get("text", ""))))

    def shore_save_poll(self):
        """The SAVE button on this PC's USB joystick (any time, not only while driving)."""
        try:
            now = shorejoy.save_pressed(self.sj, self.sj_cfg)
        except Exception:
            now = False
        if now and not self.sj_save_was and isinstance(self.link, GatewayLink):
            self.video_save("shore joystick")
        self.sj_save_was = now

    def on_cams_ack(self, d):
        if "streams" not in d:
            self.say(T("cap.cams_msg", text=i18n.from_en(d.get("text", ""))), GREEN if d.get("ok") else RED)
            return
        self.cam_ack = d
        for name in list(self.vstreams):
            self.vstreams.pop(name).stop()
        for name, (w, h) in (("FRONT-1", (640, 360)), ("FRONT-2", (640, 360)), ("REAR-LEFT", (400, 225)),
                             ("REAR-RIGHT", (400, 225))):
            url = self.link.video_url(d, name)
            if url:
                self.vstreams[name] = video.VideoStream(url, w, h, name).start()
        if d.get("notice"):
            self.say(i18n.from_en(d["notice"]), ORANGE)

    def cam_tick(self):
        """10 times a second: the front picture (or VIDEO FROZEN)."""
        f1, f2 = self.vstreams.get("FRONT-1"), self.vstreams.get("FRONT-2")
        c = self.cam_cv
        if f1 or f2:
            use = self.front_sel.update(f1, f2)
            vs = self.vstreams.get(use)
            c.delete("all")
            data = vs.ppm() if vs else None
            if data:
                self.cam_photo = tk.PhotoImage(data=data, format="ppm")
                c.create_image(0, 0, image=self.cam_photo, anchor="nw")
            c.create_text(8, 8, text=use, anchor="nw", fill="#ffffff", font=F_MED)
            if self.front_sel.notice:
                c.create_text(320, 340, text=video.notice_text(self.front_sel), fill="#ffd000", font=F_MED)
            if vs is None or vs.frozen():
                c.create_rectangle(120, 140, 520, 220, fill="#d00000", outline="")
                c.create_text(320, 180, text=T("hud.video_frozen"), fill="#ffffff", font=F_HUGE)
        g = (self.gw_status or {}).get("cameras")
        if g:
            lines = [T("cap.cam_relay", relay=T("cap.ok") if g["relay"] else T("cap.down"), kbps=g["total_kbps"]) +
                     (T("cap.cam_saver") if g["forced_low"] else "") + (T("cap.cam_inet_open") if g["internet"] else ""),
                     ""]
            mine = (self.cam_ack or {}).get("levels", {})
            for cam in g["cameras"]:
                lines.append(T("cap.cam_line", name="%-11s" % cam["name"], state="%-7s" % cam["state"],
                               s="%4d" % cam["for_s"], kin="%5d" % cam["kbps_in"], hi="%4d" % cam["kbps_out"]["HIGH"],
                               lo="%4d" % cam["kbps_out"]["LOW"], you=mine.get(cam["name"], "-")))
            self.cam_lbl.configure(text="\n".join(lines))
        r = (self.gw_status or {}).get("rec")
        if r and r.get("enabled"):
            rec = [c for c in r["cameras"] if c["state"] == "REC"]
            txt = T("cap.rec_line", n=len(rec), of=len(r["cameras"]), have=r["ring_have_min"], ring=r["ring_min"],
                    mb=r["ring_mb"], gbh=", %.1f GB/h" % r["gb_per_hour"] if r.get("gb_per_hour") else "")
            if r["open"]:
                txt += T("cap.rec_saving", until=r["open"][0]["until"])
            if r["full"]:
                txt += T("cap.rec_full")
            bad = [c["name"] + " " + c["state"] for c in r["cameras"] if c["state"] != "REC"]
            if bad:
                txt += T("cap.rec_not") + " " + ", ".join(bad)
            self.rec_lbl.configure(text=txt, fg=RED if (r["full"] or bad) else "#202020")
            if self.ring_var.get() != str(r["ring_min"]) and self.root.focus_get() is not self.ring_cb:
                self.ring_var.set(str(r["ring_min"]))
        if getattr(self, "sj", None) is not None:
            self.shore_save_poll()
        self.root.after(100, self.cam_tick)

    def _build_remote(self, rt):
        """REMOTE MANUAL from this (shore) PC: its USB joystick, request / release."""
        self.sj = shorejoy.Joysticks()
        self.sj_cfg = shorejoy.load_config()
        self.rsender = None
        top = tk.Frame(rt, bg=BG)
        top.pack(fill="x", pady=4)
        lbl(top, "cap.shore_joy", font=F_MED, bg=BG).pack(side="left", padx=4)
        btn(top, "cap.find_joy", self.remote_scan, font=F_TXT).pack(side="left")
        self.rsend_var = tk.BooleanVar(value=False)
        chk(top, "cap.send_joy", variable=self.rsend_var, font=F_MED, bg=BG,
            command=self.remote_send_toggle).pack(side="left", padx=10)
        m = tk.Frame(rt, bg=BG)
        m.pack(fill="x", padx=6)
        self.sj_vars = {}
        for r, key in enumerate(("steer", "thr")):
            lbl(m, "cap.sj_" + key, font=F_TXT, bg=BG).grid(row=r, column=0, sticky="w")
            dv = tk.IntVar(value=self.sj_cfg[key]["dev"])
            av = tk.StringVar(value=self.sj_cfg[key]["axis"])
            iv = tk.BooleanVar(value=self.sj_cfg[key].get("invert", False))
            lbl(m, "cap.sj_device", font=F_TXT, bg=BG).grid(row=r, column=1)
            tk.Spinbox(m, from_=0, to=15, textvariable=dv, width=3, font=F_MED).grid(row=r, column=2)
            ttk.Combobox(m, textvariable=av, values=shorejoy.AXES, width=4, state="readonly",
                         font=F_MED).grid(row=r, column=3, padx=4)
            chk(m, "cap.sj_invert", variable=iv, font=F_TXT, bg=BG).grid(row=r, column=4)
            self.sj_vars[key] = (dv, av, iv)
        self.sj_lever = tk.BooleanVar(value=self.sj_cfg["thr"].get("lever", True))
        chk(m, "cap.sj_lever", variable=self.sj_lever, font=F_TXT, bg=BG).grid(row=1, column=5, padx=8)
        btn(m, "cap.save_settings", self.remote_save_cfg, font=F_TXT).grid(row=0, column=5, sticky="w", padx=8)
        b = tk.Frame(rt, bg=BG)
        b.pack(fill="x", pady=6)
        btn(b, "cap.remote_request", lambda: self.cmd(proto.T_REMOTE, proto.p_remote(0), TE("what.remote_request")),
            bg="#c0d8ff", font=F_BIG).pack(side="left", padx=4)
        btn(b, "cap.remote_release", self.remote_release, bg="#ffb0b0", font=F_BIG).pack(side="left", padx=4)
        btn(b, "cap.maneuver_request", lambda: self.cmd(proto.T_MANEUVER, proto.p_remote(0),
                                                        TE("what.maneuver_request")),
            bg="#f0c0d0", font=F_BIG).pack(side="left", padx=4)
        lbl(rt, "cap.remote_help", font=F_TXT, bg=BG, wraplength=900, justify="left").pack(anchor="w", padx=6)
        self.remote_lbl = tk.Label(rt, text="-", font=("Consolas", 13), bg=BG, justify="left", anchor="w")
        self.remote_lbl.pack(fill="x", padx=6, pady=4)

    def remote_scan(self):
        found = self.sj.list()
        self.say(T("cap.joysticks", list=", ".join("%d %s" % (j["dev"], j["name"]) for j in found) or T("cap.none_found")),
                 GREEN if found else RED)

    def remote_save_cfg(self):
        for key, (dv, av, iv) in self.sj_vars.items():
            self.sj_cfg[key].update({"dev": dv.get(), "axis": av.get(), "invert": iv.get()})
        self.sj_cfg["thr"]["lever"] = self.sj_lever.get()
        shorejoy.save_config(self.sj_cfg)
        self.say(T("cap.sj_saved"), GREEN)

    def remote_send_toggle(self):
        if self.rsend_var.get():
            if not isinstance(self.link, GatewayLink):
                self.rsend_var.set(False)
                return self.say(T("cap.remote_only_gw"), RED)
            self.remote_save_cfg()
            self.rsender = shorejoy.RemoteSender(self.link, lambda: shorejoy.to_remote(self.sj, self.sj_cfg))
            self.rsender.start()
        elif self.rsender:
            self.rsender.stop()
            self.rsender = None

    def remote_release(self):
        if self.t and self.t.get("mode") == proto.MODE_MANEUVER:
            self.cmd(proto.T_MANEUVER, proto.p_remote(1), TE("what.maneuver_release"))
        else:
            self.cmd(proto.T_REMOTE, proto.p_remote(1), TE("what.remote_release"))
        if self.rsender and self.link:
            self.link.send_rjoy({"t": "rjoy", "seq": (self.rsender.seq + 1) & 0xFFFF, "ts": self.link.now(),
                                 "x": 0.0, "release": True})

    def draw_remote(self, d):
        if "remote_state" not in d:
            return
        s = self.rsender
        lines = [T("cap.rm_boat", mode=T("mode." + d["mode_name"]),
                   state=T("cap.rm_state_%d" % min(d["remote_state"], 2)),
                   why="  (" + T("cap.rm_why_%d" % min(d["remote_lost_why"], 3)) + ")" if d["remote_state"] == 2 else "")]
        lines.append(T("cap.rm_age", age=d["rjoy_age_ms"], lat=d["rjoy_lat_ms"], n=d["rjoy_packets"]))
        lines.append(T("cap.rm_sees", x="%+.2f" % (d["rjoy_x1000"] / 1000.0), lever="%.0f" % (d["rjoy_lever1000"] / 10.0),
                       f=d["remote_cap_f"], r=d["remote_cap_r"]))
        if d["rjoy_flags"] & 2:
            tgt = d["rjoy_pickup1000"]
            lines.append(T("cap.rm_pickup", to=T("cap.neutral") if tgt == 0xFFFF else "%.0f %%" % (tgt / 10)))
        if d["rjoy_flags"] & 4:
            lines.append(T("cap.rm_wait_confirm", s="%.0f" % d["confirm_left_s"]))
        if s:
            v = s.last or {}
            lines.append(T("cap.rm_sending", n=s.sent, missing=T("cap.rm_joy_missing") if s.missing else "",
                           x="%+.2f" % v.get("x", 0.0), lever="%.0f %%" % (100 * v["lever"]) if "lever" in v else "-"))
        cams = (self.gw_status or {}).get("cameras") or {}
        if d["mode"] == proto.MODE_REMOTE and cams and not cams.get("front_live", True):
            lines.append(T("cap.rm_no_front"))
        self.remote_lbl.configure(text="\n".join(lines), fg=RED if d["remote_state"] == 2 else FG)

    def _build_network(self, nt):
        """Internet channels (through the gateway): status, mode, Keep."""
        top = tk.Frame(nt, bg=BG)
        top.pack(fill="x", pady=4)
        btn(top, "cap.net_auto", lambda: self.chan_req("AUTO"), font=F_MED).pack(side="left")
        lbl(top, "cap.net_channel", font=F_TXT, bg=BG).pack(side="left")
        self.chan_var = tk.StringVar(value="4G")
        self.chan_box = ttk.Combobox(top, textvariable=self.chan_var, values=["4G", "4G2", "RF", "STARLINK"],
                                     width=10, font=F_MED, state="readonly")
        self.chan_box.pack(side="left", padx=4)
        btn(top, "cap.net_manual", lambda: self.chan_req("MANUAL", self.chan_var.get()), font=F_MED).pack(side="left")
        btn(top, "cap.net_strict", lambda: self.chan_req("MANUAL_STRICT", self.chan_var.get()),
            font=F_TXT).pack(side="left", padx=4)
        self.keep_btn = tk.Button(top, text=T("cap.keep"), command=self.chan_keep, bg="#b0ffb0", font=F_BIG, bd=3)
        self.keep_btn.pack(side="left", padx=12)
        self.net_lbl = lbl(nt, "cap.net_only_gw", font=("Consolas", 13), bg=BG, justify="left", anchor="w")
        self.net_lbl.pack(fill="x", padx=6, pady=4)

    def chan_req(self, mode, channel=None, confirm=False):
        if not isinstance(self.link, GatewayLink):
            return self.say(T("cap.net_only_gw_short"), RED)
        if mode == "MANUAL_STRICT" and not messagebox.askyesno(T("cap.net_strict_title"),
                                                                 T("cap.net_strict_q", ch=channel)):
            return
        self.link.send_msg({"t": "channel", "mode": mode, "channel": channel, "confirm": confirm})
        self.chan_last = (mode, channel)
        self.event("> %s %s %s" % (T("cap.net_channel").strip(), mode, channel or ""))

    def chan_keep(self):
        if isinstance(self.link, GatewayLink):
            self.link.send_msg({"t": "channel_keep"})

    def on_channel_ack(self, d):
        if d.get("needs_confirm"):
            if messagebox.askyesno(T("cap.emerg_title"), T("cap.emerg_q")):
                mode, ch = getattr(self, "chan_last", ("MANUAL", "STARLINK"))
                self.chan_req(mode, ch, confirm=True)
            return
        txt = T("cap.channel_msg", text=i18n.from_en(d.get("text", "")))
        self.say(txt, GREEN if d.get("ok") else RED)
        self.event(("OK   " if d.get("ok") else "NACK ") + txt)

    def draw_network(self):
        g = self.gw_status
        n = (g or {}).get("net")
        if not n:
            self.show(self.sl_banner, False, row=3, column=0, columnspan=2, sticky="ew")
            return
        yes, no = T("cap.yes"), T("cap.no")
        lines = [T("cap.net_head", mode=n["mode"], manual=" " + n["manual"] if n.get("manual") else "",
                   active=n["active"] or T("cap.none"), saver=T("cap.on") if n["saver"] else T("cap.off"),
                   sync=T("cap.allowed") if n["sync_allowed"] else T("cap.stopped"))]
        if n.get("fallback"):
            lines.append(T("cap.net_fallback", ch=n["active"]))
        if n.get("trial"):
            lines.append(T("cap.net_trial", to=n["trial"]["to"], s=n["trial"]["left_s"], user=n["trial"]["user"]))
        lines.append("")
        lines.append("%-22s %-6s %-8s %8s %5s %9s %9s  %s" % tuple(T("cap.net_col_" + k) for k in (
            "channel", "state", "internet", "latency", "loss", "today", "month", "signal")))
        for c in n["channels"]:
            sig = c.get("signal") or {}
            stxt = " ".join(str(v) for v in (sig.get("rssi_dbm"), sig.get("type"), sig.get("operator")) if v)
            if c.get("note"):
                stxt = c["note"][:30]
            lines.append("%-22s %-6s %-8s %8s %5s %9.1f %9.1f  %s%s" % (
                ("> " if c["active"] else "  ") + c["label"][:20], T("cap.up") if c["up"] else T("cap.down"),
                yes if c["internet"] else no, "%.0f ms" % c["latency_ms"] if c.get("latency_ms") is not None else "-",
                "%.0f%%" % c["loss_pct"] if c.get("loss_pct") is not None else "-",
                c["today_mb"], c["month_mb"], stxt[:34], T("cap.metered") if c["metered"] else ""))
        sl = n.get("starlink") or {}
        if sl:
            lines.append("")
            lines.append(T("cap.starlink_line", mb="%.1f" % sl["session_mb"], gb="%.2f" % sl["month_gb"],
                           warn=sl["warn_gb"]) + (T("cap.starlink_warn") if sl["warning"] else ""))
            dish = sl.get("dish") or {}
            if dish:
                lines.append(T("cap.dish") + " " + ", ".join("%s %s" % kv for kv in dish.items()))
        sy = (g or {}).get("sync")
        if sy:
            lines.append("")
            cur = sy.get("current")
            lines.append(T("cap.sync_line", state=sy["state"], reason=sy["reason"], kbps=sy["rate_kbps"],
                           rtt=sy["rtt_ms"], base=sy["base_ms"], n=sy["confirmed"], mb="%.1f" % sy["sent_mb"]) +
                         (T("cap.sync_errors", n=sy["errors"], err=sy["last_error"][:40]) if sy["errors"] else ""))
            if cur:
                lines.append(T("cap.sync_now", path=cur["path"][-60:], pc=100 * cur["done"] // max(1, cur["size"])))
            for k, v in sy["queue"].items():
                lines.append(T("cap.sync_wait", kind="%-20s" % k, n="%4d" % v["files"], mb="%8.1f" % v["mb"]))
            if sy.get("last"):
                lines.append(T("cap.sync_last", path=sy["last"][-70:]))
        if self.sync_rx:
            r = self.sync_rx.status()
            lines.append(T("cap.sync_rx", n=r["files"], mb="%.1f" % r["mb"], bad=r["bad"],
                           boats=", ".join(r["boats_connected"]) or "-", root=r["root"], free=r["free_gb"]))
        self.net_lbl.configure(text="\n".join(lines), fg=RED if n.get("fallback") or n["emergency"] else FG)
        if n["emergency"]:
            self.sl_banner.configure(text=T("cap.starlink_banner", mb="%.1f" % sl.get("session_mb", 0),
                                            gb="%.2f" % sl.get("month_gb", 0)) +
                                     (T("cap.data_warning") if sl.get("warning") else ""))
        self.show(self.sl_banner, bool(n["emergency"]), row=3, column=0, columnspan=2, sticky="ew")
        self.keep_btn.configure(text=T("cap.keep_s", s=n["trial"]["left_s"]) if n.get("trial") else T("cap.keep"),
                                bg="#ffd000" if n.get("trial") else "#b0ffb0")

    def _build_joytab(self, jt):
        """Joystick tab: source, link status, button map, guided mapping."""
        top = tk.Frame(jt, bg=BG)
        top.pack(fill="x", pady=4)
        lbl(top, "cap.jm_source", font=F_MED, bg=BG).grid(row=0, column=0, sticky="w", padx=4)
        self.jm_src = tk.IntVar(value=0)
        for i in range(len(proto.JOY_SOURCES)):
            i18n.bind(tk.Radiobutton(top, variable=self.jm_src, value=i, font=F_TXT, bg=BG,
                                     command=self.jm_from_widgets), "cap.joy_src_%d" % i).grid(row=0, column=1 + i,
                                                                                            sticky="w")
        lbl(top, "cap.jm_throttle", font=F_MED, bg=BG).grid(row=1, column=0, sticky="w", padx=4)
        self.jm_thr = tk.IntVar(value=1)
        for i in range(len(proto.THR_INPUTS)):
            i18n.bind(tk.Radiobutton(top, variable=self.jm_thr, value=i, font=F_TXT, bg=BG,
                                     command=self.jm_from_widgets), "cap.thr_in_%d" % i).grid(row=1, column=1 + i,
                                                                                           sticky="w")
        z = tk.Frame(top, bg=BG)
        z.grid(row=2, column=0, columnspan=4, sticky="w", pady=2)
        self.jm_vars = {}
        for k in ("nz_lo", "nz_hi", "lever_delta", "pickup_win"):
            lbl(z, "cap.jm_" + k, font=F_TXT, bg=BG).pack(side="left", padx=(6, 2))
            v = tk.StringVar()
            tk.Entry(z, textvariable=v, font=F_MED, width=4).pack(side="left")
            self.jm_vars[k] = v
        bb = tk.Frame(jt, bg=BG)
        bb.pack(fill="x", pady=2)
        btn(bb, "cap.jm_read", self.jm_read, font=F_TXT).pack(side="left")
        btn(bb, "cap.jm_default", self.jm_default, font=F_TXT).pack(side="left", padx=4)
        btn(bb, "cap.jm_wizard", lambda: self.wiz_start(None), bg="#c0d8ff", font=F_TXT).pack(side="left")
        btn(bb, "cap.jm_save", self.jm_save, bg="#b0ffb0", font=F_TXT).pack(side="left", padx=4)

        self.jl_lbl = tk.Label(jt, text="-", font=("Consolas", 13), bg=BG, justify="left", anchor="w")
        self.jl_lbl.pack(fill="x", padx=6, pady=4)

        g = tk.Frame(jt, bg=BG)
        g.pack(fill="x", padx=6)
        self.jm_rows = []
        names = ["cap.jm_axis_steer", "cap.jm_axis_thr"] + ["jm.%d" % i for i in range(len(proto.JM_BUTTONS))]
        half = 6                                    # two columns: fits a laptop screen
        for i, name in enumerate(names):
            r, c0 = (i % half) + 1, (i // half) * 4
            if i % half == 0:
                for c, h in enumerate(("cap.jm_col_func", "cap.jm_col_ctrl", "cap.jm_col_now")):
                    lbl(g, h, font=F_MED, bg=BG).grid(row=0, column=c0 + c, sticky="w", padx=4)
            lbl(g, name, font=F_TXT, bg=BG).grid(row=r, column=c0, sticky="w", padx=4)
            code = tk.Label(g, text="-", font=F_TXT, bg=BG, width=24, anchor="w")
            code.grid(row=r, column=c0 + 1, sticky="w", padx=4)
            now = tk.Label(g, text="", font=F_TXT, bg=BG, width=9)
            now.grid(row=r, column=c0 + 2)
            step = ("axis_steer", "axis_thr")[i] if i < 2 else i - 2
            i18n.bind(tk.Button(g, command=lambda st=step: self.wiz_start([st]), font=("Segoe UI", 10), pady=0),
                      "cap.jm_map").grid(row=r, column=c0 + 3, pady=1, padx=(0, 16))
            self.jm_rows.append((code, now))

        self.wiz_frame = tk.Frame(jt, bg="#e0ecff", bd=2, relief="groove")
        self.wiz_lbl = tk.Label(self.wiz_frame, text="", font=F_BIG, bg="#e0ecff", justify="left", wraplength=900)
        self.wiz_lbl.pack(anchor="w", padx=6, pady=4)
        self.wiz_info = tk.Label(self.wiz_frame, text="", font=F_MED, bg="#e0ecff", justify="left")
        self.wiz_info.pack(anchor="w", padx=6)
        wb = tk.Frame(self.wiz_frame, bg="#e0ecff")
        wb.pack(anchor="w", padx=6, pady=4)
        self.wiz_next = tk.Button(wb, text=T("cap.wiz_next"), command=self.wiz_next_click, bg="#b0ffb0", font=F_MED,
                                  bd=3)
        self.wiz_next.pack(side="left")
        self.wiz_skip = btn(wb, "cap.wiz_skip", self.wiz_skip_click)
        self.wiz_skip.pack(side="left", padx=4)
        btn(wb, "cap.cancel", self.wiz_cancel, bg="#ffb0b0").pack(side="left")
        self.jm_to_widgets()

    def _build_tabs(self, body):
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        self._build_joystick(right)
        nb = ttk.Notebook(right)
        nb.pack(fill="both", expand=True)
        self.nb = nb
        self.tab_keys = []

        def tab(frame, key):
            nb.add(frame, text=T(key))
            self.tab_keys.append(key)

        mp = tk.Frame(nb, bg=BG)
        tab(mp, "tab.map")
        self.map_tab = mp
        mt = tk.Frame(mp, bg=BG)
        mt.pack(fill="x")
        btn(mt, "cap.clear_track", lambda: self.track.clear(), font=F_TXT).pack(side="left")
        self.add_on_click = tk.BooleanVar(value=False)
        chk(mt, "cap.click_adds", variable=self.add_on_click, font=F_TXT, bg=BG).pack(side="left", padx=8)
        # the full map of screen 2 (was the separate map window, CLAUDE.md E)
        import mapwin
        self.mapwin = mapwin.MapWindow(self, parent=mp)
        self.mapwin.on_click = self.map_click

        at = tk.Frame(nb, bg=BG)
        tab(at, "tab.ais")
        self._build_ais(at)

        pg = tk.Frame(nb, bg=BG)
        tab(pg, "tab.program")
        self._build_program(pg)

        pl = tk.Frame(nb, bg=BG)
        tab(pl, "tab.plot")
        self.plot_tab = pl
        self.plot = tk.Canvas(pl, bg=BG, highlightthickness=0)
        self.plot.pack(fill="both", expand=True)

        cf = tk.Frame(nb, bg=BG)
        tab(cf, "tab.config")
        ct = tk.Frame(cf, bg=BG)
        ct.pack(fill="x")
        btn(ct, "cap.read_all", self.read_params, font=F_TXT).pack(side="left")
        lbl(ct, "cap.new_value", font=F_TXT, bg=BG).pack(side="left", padx=(12, 2))
        self.pval_var = tk.StringVar()
        tk.Entry(ct, textvariable=self.pval_var, font=F_MED, width=10).pack(side="left")
        btn(ct, "cap.write_selected", self.write_param, font=F_TXT).pack(side="left", padx=4)
        lbl(ct, "cap.param_note", font=F_TXT, bg=BG).pack(side="left")
        self.ptree = ttk.Treeview(cf, columns=("value", "min", "max"), height=12)
        for c, w in (("#0", 220), ("value", 140), ("min", 120), ("max", 120)):
            self.ptree.column(c, width=w)
        self.ptree.heading("#0", text=T("cap.param"))
        for c in ("value", "min", "max"):
            self.ptree.heading(c, text=T("cap.param_" + c))
        self.ptree.pack(fill="both", expand=True)

        jt = tk.Frame(nb, bg=BG)
        tab(jt, "tab.joystick")
        self.joytab = jt
        self._build_joytab(jt)

        ct = tk.Frame(nb, bg=BG)
        tab(ct, "tab.cameras")
        self._build_cameras(ct)

        rt = tk.Frame(nb, bg=BG)
        tab(rt, "tab.remote")
        self._build_remote(rt)

        nt = tk.Frame(nb, bg=BG)
        tab(nt, "tab.network")
        self._build_network(nt)

        sy = tk.Frame(nb, bg=BG)
        tab(sy, "tab.system")
        lbl(sy, "cap.sys_title", font=F_MED, bg=BG).pack(anchor="w", pady=6)
        self.sys_lbl = tk.Label(sy, text="-", font=("Consolas", 14), bg=BG, justify="left", anchor="w")
        self.sys_lbl.pack(anchor="w", padx=8)

        bt = tk.Frame(nb, bg=BG)
        tab(bt, "tab.bench")
        self._build_bench(bt)

        snd = tk.Frame(nb, bg=BG)
        tab(snd, "tab.sound")
        self._build_sound(snd)

        lg = tk.Frame(nb, bg=BG)
        tab(lg, "tab.events")
        self.events = tk.Listbox(lg, font=F_TXT)
        self.events.pack(fill="both", expand=True)

    # --- sound + voice (section D) -------------------------------------------------------------
    def _build_sound(self, st):
        """The sound settings and the sound test page (play every sound / callout in both languages)."""
        self.callouts = None
        try:
            import sound
            import soundtest
        except ImportError:
            return lbl(st, "cap.sound_missing", font=F_MED, bg=BG).pack(anchor="w", padx=6, pady=6)
        self.sound = sound.SoundSystem.for_app(self)
        self.callouts = sound.Callouts(self.sound, self)
        soundtest.build(st, self)

    # --- BENCH TEST (dock, USB laptop only): FIRST_TEST.md part 3 -----------------------------
    def _build_bench(self, bt):
        top = tk.Frame(bt, bg=BG)
        top.pack(fill="x", pady=4)
        btn(top, "cap.bench_enter", lambda: self.bench(proto.BENCH_ENTER, 0, TE("what.bench_enter")),
            bg="#9be0e0", font=F_TXT).pack(side="left", padx=4)
        btn(top, "cap.bench_leave", lambda: self.bench(proto.BENCH_LEAVE, 0, TE("what.bench_leave")),
            font=F_TXT).pack(side="left")
        lbl(top, "cap.bench_note", font=F_TXT, bg=BG, wraplength=640, justify="left").pack(side="left", padx=10)
        self.bench_state = tk.Label(bt, text=T("cap.bench_off"), font=F_MED, bg=BG, anchor="w")
        self.bench_state.pack(fill="x", padx=6)
        body = tk.Frame(bt, bg=BG)
        body.pack(fill="both", expand=True)
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="both", expand=True, padx=6)
        lbl(left, "cap.bench_sensors", font=F_MED, bg=BG).pack(anchor="w")
        self.bench_sens = tk.Label(left, text="-", font=("Consolas", 12), bg=BG, justify="left", anchor="nw")
        self.bench_sens.pack(anchor="w", fill="x")
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=6)

        lbl(right, "cap.bench_pulses", font=F_MED, bg=BG).pack(anchor="w")
        pr = tk.Frame(right, bg=BG)
        pr.pack(anchor="w")
        self.bench_ms = tk.IntVar(value=100)
        for ms in (100, 200, 300):
            tk.Radiobutton(pr, text="%d ms" % ms, variable=self.bench_ms, value=ms, font=F_TXT, bg=BG).pack(side="left")
        btn(pr, "cap.bench_left", lambda: self.bench(proto.BENCH_LEFT, self.bench_ms.get(),
                                                     TE("what.pulse_left", ms=self.bench_ms.get())),
            font=F_TXT).pack(side="left", padx=6)
        btn(pr, "cap.bench_right", lambda: self.bench(proto.BENCH_RIGHT, self.bench_ms.get(),
                                                      TE("what.pulse_right", ms=self.bench_ms.get())),
            font=F_TXT).pack(side="left")
        self.bench_pulse = tk.Label(right, text="-", font=F_TXT, bg=BG, anchor="w", justify="left")
        self.bench_pulse.pack(anchor="w")

        lbl(right, "cap.bench_limits", font=F_MED, bg=BG).pack(anchor="w", pady=(8, 0))
        lr = tk.Frame(right, bg=BG)
        lr.pack(anchor="w")
        self.bench_lim = []
        for key in ("cap.bench_lim_l", "cap.bench_lim_r", "cap.bench_estop"):
            lb = lbl(lr, key, font=F_MED, bg=PANEL, width=9, relief="ridge")
            lb.pack(side="left", padx=3)
            self.bench_lim.append(lb)

        lbl(right, "cap.bench_buttons", font=F_MED, bg=BG).pack(anchor="w", pady=(8, 0))
        br = tk.Frame(right, bg=BG)
        br.pack(anchor="w")
        self.bench_btn = []
        for b in (5, 1, 2, 3, 4, 0):                    # HDG LOCK, -1, +1, -10, +10, STANDBY
            lb = tk.Label(br, text=T("btn.%d" % b), font=F_TXT, bg=PANEL, width=10, relief="ridge")
            lb.pack(side="left", padx=2)
            self.bench_btn.append((b, lb))

        lbl(right, "cap.bench_buzzer", font=F_MED, bg=BG).pack(anchor="w", pady=(8, 0))
        btn(right, "cap.bench_buzz_test", lambda: self.bench(proto.BENCH_BUZZER, 1000, TE("what.buzzer")),
            font=F_TXT).pack(anchor="w")

        lbl(right, "cap.bench_servo_title", font=F_MED, bg=BG).pack(anchor="w", pady=(8, 0))
        self.bench_servo = tk.Label(right, text="-", font=("Consolas", 12), bg=BG, justify="left", anchor="w")
        self.bench_servo.pack(anchor="w")
        btn(right, "cap.bench_trip", self.bench_trip, bg="#ffb0b0", font=F_TXT).pack(anchor="w", pady=4)
        self.bench_d = None

    def bench(self, action, arg, what):
        if isinstance(self.link, GatewayLink):
            return self.say(T("cap.bench_only_usb"), RED)
        self.cmd(proto.T_BENCH, proto.p_bench(action, arg), what)

    def bench_trip(self):
        from tkinter import messagebox
        if not messagebox.askyesno(T("cap.trip_title"), T("cap.trip_q")):
            return
        self.bench(proto.BENCH_TRIP, proto.BENCH_TRIP_CODE, TE("what.trip"))

    def on_bench(self, d):
        self.bench_d = d
        self.bench_d_rx = time.time()
        self.draw_bench()

    def draw_bench(self):
        if not hasattr(self, "bench_state"):
            return
        d = self.bench_d
        fresh = d is not None and time.time() - getattr(self, "bench_d_rx", 0) < 1.0
        in_bench = bool(self.t and self.t.get("mode") == proto.MODE_BENCH)
        if not (fresh and in_bench):
            self.bench_state.configure(text=T("cap.bench_off"), fg="#606060")
            return
        self.bench_state.configure(text=T("cap.bench_on", s="%d" % d["idle_left_s"]) +
                                   (T("cap.bench_tripped") if d["mon_tripped"] else ""),
                                   fg=RED if d["mon_tripped"] else "#008080")

        def yn(v):
            return "%-3s" % (T("cap.yes") if v else T("cap.no"))
        acc = T("cap.imu_acc_%d" % d["imu_accuracy"]) if d["imu_accuracy"] < 4 else "?"
        lines = [
            T("bench.imu", ok=yn(d["imu_ok"]), n="%-8d" % d["imu_reports"], resets=d["imu_resets"], acc=acc),
            T("bench.heading", hdg="%6.1f" % d["heading"], ok=yn(d["compass_ok"])),
            T("bench.yaw", yaw="%+6.1f" % d["yaw"], ok=yn(d["gyro_ok"])),
            T("bench.roll", roll="%+6.1f" % d["roll"], pitch="%+6.1f" % d["pitch"]),
            T("bench.accel", z="%+6.2f" % d["accel_z"]),
            T("bench.gps", ok=yn(d["gps_ok"]), ubx=yn(d["gps_ubx"]), fix=yn(d["fix"]), dim=d["gps_fix_type"],
              sats=d["sats"]),
            T("bench.sog", sog="%.1f" % d["sog"], cog="%.0f" % d["cog"]),
            T("bench.spoof", spoof=proto.GPS_SPOOF[d["spoof"]] if d["spoof"] < 4 else "?",
              jam=proto.GPS_JAM[d["jam"]] if d["jam"] < 4 else "?"),
            T("bench.motor", v=T("bench.no_current") if d["current"] < 0 else "%.2f A" % d["current"]),
            T("bench.sd", card=yn(d["sd_card"]), mounted=yn(d["sd_mounted"]),
              state=proto.SD_STATES[d["sd_state"]] if d["sd_state"] < len(proto.SD_STATES) else "?",
              free="?" if d["sd_free_kb"] is None else "%.1f GB" % (d["sd_free_kb"] / 1048576.0)),
            T("bench.uart", n=d["uart_errors"]),
        ]
        self.bench_sens.configure(text="\n".join(lines))
        run = {-1: T("bench.run_left"), 1: T("bench.run_right")}.get(d["pulse"], T("bench.stopped"))
        self.bench_pulse.configure(text=T("bench.pulse", run=run, est="%+.1f" % d["estimate"], n=d["pulses"],
                                          refused=d["refused"]) +
                                   (T("bench.last_refuse", why=T("bench.refuse_%d" % d["last_refuse"]))
                                    if 0 < d["last_refuse"] < len(proto.BENCH_REFUSE) else ""))
        for lb, on, fitted in ((self.bench_lim[0], d["limit_left"], d["limits_fitted"]),
                               (self.bench_lim[1], d["limit_right"], d["limits_fitted"]),
                               (self.bench_lim[2], d["estop"] == 2, d["estop"] != 0)):
            lb.configure(bg=(ORANGE if on else "#b0ffb0") if fitted else PANEL,
                         fg=FG if fitted else "#909090")
        for b, lb in self.bench_btn:
            n = d["button_counts"][b]
            lb.configure(bg="#7de07d" if n else PANEL,
                         text="%s%s" % (T("btn.%d" % b), (" x%d" % n) if n else ""))
        self.bench_servo.configure(text=T("bench.servo", thr="%4d" % d["thr_us"], idle=d["thr_idle_us"],
                                          gear="%4d" % d["gear_us"], n=d["gear_n_us"]))

    # --- AIS ship list + collision avoidance (sections A / D / E2) ------------------
    AIS_COLS = (("name", "ais.col_ship", 200), ("cls", "ais.col_class", 50), ("dist", "ais.col_dist", 95),
                ("brg", "ais.col_brg", 70), ("cpa", "ais.col_cpa", 70), ("tcpa", "ais.col_tcpa", 80),
                ("age", "ais.col_age", 60), ("state", "", 110))

    def _build_ais(self, at):
        self.ais_state_lbl = tk.Label(at, text=T("ais.state", state="-"), font=F_MED, bg=BG, anchor="w",
                                      justify="left")
        self.ais_state_lbl.pack(fill="x", padx=6, pady=(4, 0))
        lbl(at, "ais.small_craft", font=F_TXT, bg=BG, fg="#a00000").pack(anchor="w", padx=6)
        self.ais_tree = ttk.Treeview(at, columns=[c for c, _t, _w in self.AIS_COLS[1:]], height=12)
        for i, (c, t, w) in enumerate(self.AIS_COLS):
            col = "#0" if i == 0 else c
            self.ais_tree.column(col, width=w, anchor="w" if i == 0 else "e")
            self.ais_tree.heading(col, text=T(t) if t else "")
        self.ais_tree.tag_configure("danger", background="#ffb0b0")
        self.ais_tree.tag_configure("warn", background="#fff0a0")
        self.ais_tree.tag_configure("stale", foreground="#888888")
        self.ais_tree.pack(fill="both", expand=True, padx=6, pady=4)
        self.ais_tree.bind("<<TreeviewSelect>>", lambda e: self.ais_details())
        self.ais_detail = lbl(at, "ais.click_ship", font=F_TXT, bg=BG, anchor="w", justify="left")
        self.ais_detail.pack(fill="x", padx=6, pady=(0, 6))
        bb = tk.Frame(at, bg=BG)
        bb.pack(anchor="w", padx=6, pady=(0, 6))
        lbl(bb, "ais.proposal_so", font=F_TXT, bg=BG).pack(side="left")
        btn(bb, "drv.accept", lambda: self.avoid_answer(None, True), font=F_TXT, bg="#b0ffb0").pack(side="left",
                                                                                                    padx=4)
        btn(bb, "drv.reject", lambda: self.avoid_answer(None, False), font=F_TXT, bg="#ffb0b0").pack(side="left")
        self.ais_rows = {}

    def ais_rows_now(self):
        """The AIS list joined with the collision threats (CPA / TCPA) by MMSI, most dangerous first."""
        av = (self.gw_status or {}).get("avoid") or {}
        th = {t["mmsi"]: t for t in av.get("threats") or []}
        d = self.t or {}
        rows = []
        for t in self.ais_list():
            r = dict(t)
            x = th.get(t["mmsi"], {})
            r["cpa_nm"], r["tcpa_min"], r["level"] = x.get("cpa_nm"), x.get("tcpa_min"), x.get("level")
            r["bearing"] = x.get("bearing")
            if r["bearing"] is None and d.get("pos_source") and t.get("lat") is not None:
                k = math.cos(math.radians(d["est_lat"]))
                r["bearing"] = math.degrees(math.atan2((t["lon"] - d["est_lon"]) * k, t["lat"] - d["est_lat"])) % 360
            rows.append(r)
        order = {"danger": 0, "warn": 1}
        rows.sort(key=lambda r: (order.get(r["level"], 2), r.get("dist_nm") if r.get("dist_nm") is not None else 1e9))
        return rows

    def draw_ais(self):
        if not hasattr(self, "ais_tree"):
            return
        av = (self.gw_status or {}).get("avoid") or {}
        st = av.get("state") or "-"
        txt = T("ais.state", state=T("avoid." + st.replace(" ", "_")) if st != "-" else "-")
        if av.get("why"):
            txt += " - %s" % i18n.from_en(av["why"])
        if av.get("proposal"):
            txt += "\n" + T("ais.proposal_on_driver", text=proposal_text(av["proposal"]))
        self.ais_state_lbl.configure(text=txt, fg=RED if st in ("AVOID", "DRIVER DECISION", "ALARM ONLY", "PROPOSE")
                                     else "black")
        rows = self.ais_rows_now()
        self.ais_rows = {str(r["mmsi"]): r for r in rows}
        sel = self.ais_tree.selection()
        self.ais_tree.delete(*self.ais_tree.get_children())

        def f(v, fmt):
            return "-" if v is None else fmt % v
        for r in rows:
            state = {"danger": T("ais.danger"), "warn": T("ais.warning")}.get(r["level"], "")
            if r.get("stale"):
                state = (state + " " + T("ais.stale")).strip()
            tag = "danger" if r["level"] == "danger" else "warn" if r["level"] == "warn" else \
                "stale" if r.get("stale") else ""
            self.ais_tree.insert("", "end", iid=str(r["mmsi"]), text=r.get("name") or str(r["mmsi"]), tags=(tag,),
                                 values=(r.get("cls", "?"), f(r.get("dist_nm"), "%.2f"), f(r.get("bearing"), "%03.0f"),
                                         f(r["cpa_nm"], "%.2f"),
                                         f(r["tcpa_min"] if (r["tcpa_min"] or 0) >= 0 else None, "%.1f"),
                                         f(r.get("age_s"), "%.0f"), state))
        keep = [i for i in sel if i in self.ais_rows]
        if keep:
            self.ais_tree.selection_set(keep)

    def ais_details(self):
        sel = self.ais_tree.selection()
        r = self.ais_rows.get(sel[0]) if sel else None
        if not r:
            return
        bits = [T("ais.d_name", name=r.get("name") or "?", mmsi=r["mmsi"], cls=r.get("cls", "?")) +
                (T("ais.d_online") if r.get("src") == "online" else "")]
        bits.append(T("ais.d_motion", sog=r.get("sog"), cog=r.get("cog"), hdg=r.get("heading"), rot=r.get("turn_dpm")))
        if r.get("length") or r.get("type"):
            bits.append(T("ais.d_ship", type=r.get("type"), l=r.get("length"), b=r.get("beam"), d=r.get("draught"),
                          call=r.get("callsign") or "-"))
        if r.get("dest"):
            bits.append(T("ais.d_dest", dest=r["dest"]))
        bits.append(T("ais.d_pos", lat="%.5f" % r["lat"], lon="%.5f" % r["lon"], age=r.get("age_s")) +
                    (" (%s)" % T("ais.stale") if r.get("stale") else ""))
        self.ais_detail.configure(text="\n".join(bits))

    def _build_program(self, pg):
        top = tk.Frame(pg, bg=BG)
        top.pack(fill="x")
        for key, fn in (("cap.pg_upload", self.prog_upload), ("cap.pg_download", self.prog_download),
                        ("cap.pg_save", self.prog_save), ("cap.pg_load", self.prog_load)):
            btn(top, key, fn, font=F_TXT).pack(side="left", padx=1)
        lbl(top, "cap.pg_at_end", font=F_TXT, bg=BG).pack(side="left")
        self.final_cb = ttk.Combobox(top, width=18, state="readonly", font=F_TXT)
        self.final_cb.pack(side="left")
        self.pg_tree = ttk.Treeview(pg, columns=("lat", "lon", "kn", "radius", "action"), height=10)
        for c, w in (("#0", 50), ("lat", 140), ("lon", 140), ("kn", 70), ("radius", 80), ("action", 100)):
            self.pg_tree.column(c, width=w)
            self.pg_tree.heading(c, text="#" if c == "#0" else T("cap.pg_" + c))
        self.pg_tree.pack(fill="both", expand=True)
        self.pg_tree.bind("<<TreeviewSelect>>", self.prog_select)
        ed = tk.Frame(pg, bg=BG)
        ed.pack(fill="x")
        self.pg_vars = {}
        for k, w in (("lat", 11), ("lon", 11), ("kn", 5), ("radius", 5)):
            lbl(ed, "cap.pg_" + k, font=F_TXT, bg=BG).pack(side="left")
            v = tk.StringVar()
            tk.Entry(ed, textvariable=v, font=F_MED, width=w).pack(side="left", padx=(0, 4))
            self.pg_vars[k] = v
        self.pg_vars["radius"].set("25")
        self.pg_vars["kn"].set("8")
        self.act_cb = ttk.Combobox(ed, width=8, state="readonly", font=F_TXT)
        self.act_cb.pack(side="left")
        self.prog_combos()
        self.final_cb.current(0)
        self.act_cb.current(0)
        bt = tk.Frame(pg, bg=BG)
        bt.pack(fill="x")
        for key, fn in (("cap.pg_add", self.prog_add), ("cap.pg_change", self.prog_change),
                        ("cap.pg_delete", self.prog_delete), ("cap.pg_up", lambda: self.prog_move(-1)),
                        ("cap.pg_down", lambda: self.prog_move(1)), ("cap.pg_clear", self.prog_clear)):
            btn(bt, key, fn, font=F_TXT).pack(side="left", padx=1)
        btn(bt, "cap.pg_depth", self.prog_depth_check, font=F_TXT, bg="#c0e0ff").pack(side="left", padx=8)
        lbl(pg, "cap.pg_profile_note", font=F_TXT, bg=BG).pack(anchor="w")
        self.pg_prof = tk.Canvas(pg, height=150, bg="#ffffff", highlightthickness=0)
        self.pg_prof.pack(fill="x", padx=4, pady=2)
        self.map_mark = None                 # a refused position, shown on the map

    def prog_combos(self):
        """The two program drop-downs in the screen language (the choice is kept by its index)."""
        for cb, names, key in ((self.final_cb, proto.PROG_END, "prog.end_%d"), (self.act_cb, proto.WP_ACTIONS,
                                                                               "wp.act_%d")):
            i = cb.current()
            cb.configure(values=[T(key % n) for n in range(len(names))])
            if i >= 0:
                cb.current(i)

    # --- program editor ----------------------------------------------------------
    def prog_refresh(self):
        self.pg_tree.delete(*self.pg_tree.get_children())
        for i, w in enumerate(self.prog["wps"]):
            self.pg_tree.insert("", "end", iid=str(i), text=str(i + 1),
                                values=("%.6f" % w["lat"], "%.6f" % w["lon"], "%.1f" % w["kn"], "%.0f" % w["radius"],
                                        T("wp.act_%d" % w["action"])))
        self.final_cb.current(self.prog["final"])

    def prog_entry(self):
        try:
            return {"lat": float(self.pg_vars["lat"].get()), "lon": float(self.pg_vars["lon"].get()),
                    "kn": float(self.pg_vars["kn"].get()), "radius": float(self.pg_vars["radius"].get()),
                    "action": max(0, self.act_cb.current())}
        except ValueError:
            self.say(T("cap.pg_numbers"), RED)
            return None

    def prog_selected(self):
        sel = self.pg_tree.selection()
        return int(sel[0]) if sel else None

    def prog_select(self, _ev=None):
        i = self.prog_selected()
        if i is None:
            return
        w = self.prog["wps"][i]
        for k in ("lat", "lon"):
            self.pg_vars[k].set("%.6f" % w[k])
        self.pg_vars["kn"].set("%.1f" % w["kn"])
        self.pg_vars["radius"].set("%.0f" % w["radius"])
        self.act_cb.current(w["action"])
        self.draw_profile(i)

    # --- depth (section C) --------------------------------------------------------
    def shallow_limit(self):
        """The gateway's limit (draft + margin, at least 3 m) or 3.0 m without a gateway."""
        return ((self.gw_status or {}).get("avoid") or {}).get("shallow_limit_m") or 3.0

    def draw_profile(self, i):
        c = self.pg_prof
        c.delete("all")
        wps = self.prog["wps"]
        w, h = max(200, c.winfo_width()), 150
        if not self.charts:
            return c.create_text(10, 70, anchor="w", text=T("cap.no_charts"), font=F_TXT)
        if i is None or i + 1 >= len(wps):
            return c.create_text(10, 70, anchor="w", text=T("cap.pg_select_next"), font=F_TXT)
        a, b = wps[i], wps[i + 1]
        prof = self.charts.profile(a["lat"], a["lon"], b["lat"], b["lon"], 25.0)
        lim = self.shallow_limit()
        deps = [p[3] for p in prof if p[3] is not None]
        dmax = max([lim * 2] + deps)
        L = prof[-1][0] or 1.0
        X = lambda dist: 40 + dist / L * (w - 50)
        Y = lambda dep: 10 + min(dep, dmax) / dmax * (h - 30)
        c.create_line(40, Y(lim), w - 10, Y(lim), fill="#e00000", dash=(5, 3))
        c.create_text(38, Y(lim), text="%.1f m" % lim, anchor="e", fill="#e00000", font=F_TXT)
        for (d1, _a, _b, z1), (d2, _c, _d, z2) in zip(prof, prof[1:]):
            bad = z1 is None or z2 is None or z1 < lim or z2 < lim
            z1 = 0.0 if z1 is None else max(0.0, z1)
            z2 = 0.0 if z2 is None else max(0.0, z2)
            c.create_polygon(X(d1), 10, X(d2), 10, X(d2), Y(z2), X(d1), Y(z1),
                             fill="#ffb0b0" if bad else "#b0d8f0", outline="")
        c.create_text(w - 10, h - 4, anchor="se", font=F_TXT,
                      text=T("cap.pg_leg_profile", a=i + 1, b=i + 2, m="%.0f" % L,
                             min="%.1f m" % min(deps) if deps else T("hud.no_data")))

    def prog_depth_check(self):
        """Every leg against the charts (the gateway checks the same before AUTO)."""
        if not self.charts:
            return self.say(T("cap.no_charts_auto"), RED)
        wps = self.prog["wps"]
        lim = self.shallow_limit()
        for k in range(len(wps) - 1):
            bad = self.charts.check_leg(wps[k]["lat"], wps[k]["lon"], wps[k + 1]["lat"], wps[k + 1]["lon"], lim, 50.0,
                                        step_m=40.0)
            if bad:
                self.map_mark = (bad[1], bad[2])
                self.draw_profile(k)
                self.say(T("cap.pg_leg_bad", a=k + 1, b=k + 2, why=i18n.from_en(bad[4]), lat="%.5f" % bad[1],
                           lon="%.5f" % bad[2]), RED)
                return False
        self.map_mark = None
        self.say(T("cap.pg_depth_ok", m="%.1f" % lim), GREEN)
        return True

    def prog_add(self):
        w = self.prog_entry()
        if w and len(self.prog["wps"]) < program.MAX_WP:
            self.prog["wps"].append(w)
            self.prog_refresh()

    def prog_change(self):
        i, w = self.prog_selected(), self.prog_entry()
        if i is not None and w:
            self.prog["wps"][i] = w
            self.prog_refresh()

    def prog_delete(self):
        i = self.prog_selected()
        if i is not None:
            del self.prog["wps"][i]
            self.prog_refresh()

    def prog_move(self, d):
        i = self.prog_selected()
        if i is None or not 0 <= i + d < len(self.prog["wps"]):
            return
        w = self.prog["wps"]
        w[i], w[i + d] = w[i + d], w[i]
        self.prog_refresh()
        self.pg_tree.selection_set(str(i + d))

    def prog_clear(self):
        self.prog = program.new_program()
        self.prog_refresh()

    def prog_upload(self):
        self.prog["final"] = max(0, self.final_cb.current())
        problems = program.check(self.prog)
        if problems:
            return self.say(T("cap.pg_problems", list="; ".join(i18n.from_en(p) for p in problems[:3])), RED)
        if not self.link:
            return self.say(T("cap.not_connected"), RED)
        if self.charts and not self.prog_depth_check():
            return                                          # refused here already (the gateway would too)
        frames = program.upload_frames(self.prog)
        for t, p in frames[:-1]:
            self.link.send(t, p)
        self.cmd(frames[-1][0], frames[-1][1], TE("what.program_upload", n=len(self.prog["wps"])))

    def prog_download(self):
        n = self.t["prog_n"] if self.t else 0
        if not self.link or n == 0:
            return self.say(T("cap.pg_none_on_boat"), RED)
        self.prog_dl = []
        for i in range(n):
            self.link.send(proto.T_PROG_GET, struct.pack("<B", i))

    def prog_save(self):
        from tkinter import filedialog
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[(T("cap.pg_filetype"), "*.json")])
        if path:
            self.prog["final"] = max(0, self.final_cb.current())
            program.save(self.prog, path)
            self.say(T("cap.pg_saved", path=path))

    def prog_load(self):
        from tkinter import filedialog
        path = filedialog.askopenfilename(filetypes=[(T("cap.pg_filetype"), "*.json")])
        if path:
            self.prog = program.load(path)
            self.prog_refresh()

    # --- actions -----------------------------------------------------------------
    def disconnect(self, why=None):
        if self.link:
            self.link.stop()
            self.link = None
        self.boat, self.boat_ok = None, False
        if self.boatconf:
            self.boatconf.close()
        self.conn_btn.configure(text=T("cap.connect"), bg="#b0ffb0")
        self.say(why or T("cap.disconnected"), RED if why else "#202020")

    def open_boats(self):
        """The boat list (boats/ in git): choose, log in, connect (CLAUDE.md F)."""
        if self.boatsel is None:
            self.boatsel = crewui.BoatSelector(self)
        else:
            self.boatsel.w.lift()

    def connect_boat(self, bt, user, pw, crew=None):
        """Connect to a boat of the list: the gateway must be that boat. -> None or the error."""
        if self.link:
            self.disconnect()
        if not user:
            return T("boat.need_login")
        try:
            self.link = GatewayLink("%s:%d" % (bt.ip, bt.port), user, password=pw, station="ground station (%s)" % user,
                                    log=True, crew=crew, boat=bt.name)
            w = self.link.start()
        except Exception as e:
            self.link = None
            return T("cap.cannot_connect", err=i18n.from_en(str(e)))
        self.boat, self.boat_ok = bt, False
        self.port_var.set(bt.address())
        self.say(T("cap.logged_in", user=user, role=w["role"],
                   where=T("cap.on_board") if w["local"] else T("cap.remote")), GREEN)
        if w.get("crew_note"):
            self.event(i18n.from_en(w["crew_note"]))
        self._connected()
        self.open_driver()                          # screen 1: the live camera to confirm it
        self.boatconf = crewui.BoatConfirm(self, bt)
        return None

    def connect(self):
        if self.link:
            return self.disconnect()
        addr = self.port_var.get().strip()
        try:
            if addr.startswith("gw:"):
                # through the gateway (VPN / boat LAN): user name + password
                if self.login:
                    user, pw = self.login
                else:
                    user = simpledialog.askstring(T("cap.login_title"), T("cap.login_user"), parent=self.root)
                    pw = simpledialog.askstring(T("cap.login_title"), T("cap.login_pw"), show="*",
                                                parent=self.root) if user else None
                if not user or pw is None:
                    return
                self.link = GatewayLink(addr[3:], user, password=pw, station="ground station (%s)" % user, log=True)
                w = self.link.start()
                self.say(T("cap.logged_in", user=user, role=w["role"],
                           where=T("cap.on_board") if w["local"] else T("cap.remote")), GREEN)
            else:
                self.link = Link(addr)
                self.link.start()
        except Exception as e:
            self.link = None
            self.say(T("cap.cannot_connect", err=e), RED)
            return
        self._connected()

    def _connected(self):
        self.conn_btn.configure(text=T("cap.disconnect"), bg="#ffb0b0")
        if self.port_var.get().startswith("tcp:"):
            self.sim_frame.pack(fill="x", pady=(8, 0))
        else:
            self.sim_frame.pack_forget()
        self.log_lbl.configure(text=T("cap.log", path=self.link.logger.path))
        self.root.after(500, self.read_params)
        self.jm_load = True
        self.root.after(700, lambda: self.link and self.link.send(proto.T_JOYMAP_GET))
        self.root.after(200, self.raw_tick)

    # --- two screens (section I) ----------------------------------------------------
    def open_driver(self):
        if self.driver:
            return
        geo = self.layout.get("driver")
        self.driver = driver.DriverScreen(self, geo)
        mons = monitors.list_monitors(self.root)
        if not geo and len(mons) >= 2:
            # laptop + external monitor: screen 1 (driving) on the external one, screen 2 stays on the built-in
            self.root.after(300, lambda: self.driver and monitors.move_to(self.driver.w, 1, self))
        for k in ("<F11>",):
            self.driver.w.bind(k, lambda e: self.toggle_screen())
        self.layout_changed()

    # --- crew / profile (CLAUDE.md A, B, D) ---------------------------------------------
    def crew_swap(self):
        if not isinstance(self.link, GatewayLink):
            return self.say(T("crew.need_gateway"), RED)
        crewui.request_swap(self)

    def helm_release(self):
        """Releasing HELM: on an UNCREWED boat that is a LOST LINK (STANDBY, then the engine kill)."""
        if self.t and self.t.get("profile") == proto.PROFILE_UNCREWED:
            from tkinter import messagebox
            if not messagebox.askyesno(T("cap.helm_release"), T("crew.release_uncrewed"), parent=self.root):
                return
        self.cmd(proto.T_HELM, proto.p_helm(1), TE("what.release_helm"))

    def draw_crew(self, d):
        crew, cap, lic = crewui.crew_of(self) if isinstance(self.link, GatewayLink) else (None, None, False)
        txt = T("crew.you_" + crew) if crew in ("captain", "copilot") else ""
        if crew == "copilot" and cap:
            txt += "  " + T("crew.captain_is", name=cap)
        if self.boat is not None:
            txt = "%s  %s" % (self.boat.name, txt)
        self.crew_lbl.configure(text=txt, bg="#b0ffb0" if crew == "captain" else ("#d0e0ff" if crew else PANEL))
        prof = d.get("profile")
        if prof is not None:
            self.prof_lbl.configure(text=T("prof.%d" % prof) if prof < 3 else "?",
                                    bg="#80c0ff" if prof == proto.PROFILE_UNCREWED else
                                    ("#b0ffb0" if prof == proto.PROFILE_CREWED else "#ffd080"))
        unc = prof == proto.PROFILE_UNCREWED
        self.show(self.unc_banner, unc, row=7, column=0, columnspan=2, sticky="ew")
        ll = d.get("ll_state", 0)
        if ll:
            txt = T("cap.ll_killed") if ll == 2 else T("cap.ll_lost", s="%.0f" % d.get("ll_left_s", 0.0),
                                                       near=T("cap.ll_near") if d.get("near_shore") else
                                                       T("cap.ll_far", m=d.get("coast_dist_m") or "?"))
            self.ll_banner.configure(text=txt)
        self.show(self.ll_banner, bool(ll), row=8, column=0, columnspan=2, sticky="ew")
        need = self.boat is not None and not self.boat_ok
        if need:
            self.boat_banner.configure(text=T("boat.banner", name=self.boat.name))
        self.show(self.boat_banner, need, row=9, column=0, columnspan=2, sticky="ew")

    def close_driver(self):
        if self.driver:
            d, self.driver = self.driver, None
            d.destroy()
            self.layout_changed()

    # --- the map: part of screen 2 (CLAUDE.md E; the separate map window is gone) ---------
    def open_map(self):
        """Show screen 2's Map tab."""
        self.nb.select(self.map_tab)

    def close_map(self):
        pass

    def toggle_screen(self):
        """F11. One monitor: show screen 1 (driving) or screen 2 (navigation & control) in turn.
        Two or more: screen 2 on the built-in (main) monitor, screen 1 on the external one."""
        mons = monitors.list_monitors(self.root)
        if len(mons) >= 2:
            if not self.driver:
                self.open_driver()
            monitors.move_to(self.root, 0, self)
            monitors.move_to(self.driver.w, 1, self)
            return
        self.show_screen(1 if self.screen_shown == 2 else 2)

    def show_screen(self, n):
        """One monitor: bring screen n to the front (the other stays open, behind it)."""
        if n == 1:
            if not self.driver:
                self.open_driver()
            self.driver.w.deiconify()
            self.driver.w.lift()
            self.driver.w.focus_force()
        else:
            self.root.deiconify()
            self.root.lift()
            if self.driver:
                self.driver.w.lower(self.root)
            self.root.focus_force()
        self.screen_shown = n

    def swap_screens(self):
        """F12: captain and driver windows change places (monitors)."""
        if not self.driver:
            return
        self.root.state("normal")
        g1, g2 = self.root.geometry(), self.driver.w.geometry()
        self.root.geometry(g2)
        self.driver.w.geometry(g1)
        self.layout_changed()

    def layout_changed(self):
        if self._layout_job:
            self.root.after_cancel(self._layout_job)
        self._layout_job = self.root.after(800, self.save_layout)

    def save_layout(self):
        self._layout_job = None
        try:
            self.layout = {"captain": self.root.geometry(), "driver_open": self.driver is not None,
                           "driver": self.driver.w.geometry() if self.driver else self.layout.get("driver")}
            with open(self.layout_path, "w", encoding="utf-8") as f:
                json.dump(self.layout, f)
        except (OSError, tk.TclError):
            pass

    # --- driver confirmation through the gateway / single operator (section F1) ------
    def driver_beat(self):
        """Once a second: tell the gateway whether this station shows the driver screen."""
        if isinstance(self.link, GatewayLink) and self.link.connected:
            self.link.send_msg({"t": "driver", "open": self.driver is not None})
        self.root.after(1000, self.driver_beat)

    def single_op_toggle(self, confirmed=None):
        if not isinstance(self.link, GatewayLink):
            return self.say(T("cap.so_only_gw"), RED)
        if self.single_op:
            return self.link.send_msg({"t": "single_op", "on": False})
        if confirmed is None:
            confirmed = messagebox.askyesno(T("cap.so_title"), T("cap.so_q"), icon="warning")
        if confirmed:
            self.link.send_msg({"t": "single_op", "on": True})

    def on_single_op(self, on):
        if on != self.single_op:
            self.event(T("cap.ev_so_on") if on else T("cap.ev_so_off"))
        self.single_op = on
        self.so_btn.configure(text=T("cap.single_op_on") if on else T("cap.single_op"),
                              bg="#d080ff" if on else PANEL)
        self.show(self.so_banner, on, row=5, column=0, columnspan=2, sticky="ew")

    def on_driver_ask(self, d):
        """The gateway asks this (the captain's) screen 1 to accept a copilot action."""
        if not self.driver:
            return
        if self.screen_shown != 1 and len(monitors.list_monitors(self.root)) < 2:
            self.show_screen(1)                     # one monitor: the question must be seen
        i = d["id"]
        self.gw_asks[i] = d["text"]
        send = lambda yes: isinstance(self.link, GatewayLink) and self.link.send_msg(
            {"t": "driver_answer", "id": i, "yes": yes})
        self.driver.ask(T("drv.ask_from", what=i18n.from_en(d["text"]), who=d.get("from", "?")),
                        lambda why: send(True),
                        lambda why: send(False) if "closed" not in why and "done" not in why else None,
                        d.get("timeout_s", 10.0), tag=i)

    def drv_proposal(self):
        """A HEADING LOCK collision-avoidance proposal of the gateway -> ACCEPT / REJECT on the driver screen."""
        p = ((self.gw_status or {}).get("avoid") or {}).get("proposal")
        tag = ("avoid", p["id"]) if p else None
        old = getattr(self, "prop_tag", None)
        if old and old != tag and self.driver:
            self.driver.cancel(old)                 # gone or replaced at the gateway
        self.prop_tag = tag
        if not p or old == tag or not self.driver:
            return
        self.event(T("cap.ev_proposal", text=proposal_text(p)))
        self.driver.ask(proposal_text(p), lambda why: self.avoid_answer(p["id"], True),
                        lambda why: self.avoid_answer(p["id"], False) if "closed" not in why and "done" not in why
                        else None, 60.0, tag=tag, head=T("drv.head_avoid"))

    def avoid_answer(self, pid, yes):
        """ACCEPT / REJECT of the proposal (the gateway takes it only from a driver screen or single-operator)."""
        if pid is None:
            p = ((self.gw_status or {}).get("avoid") or {}).get("proposal")
            if not p:
                return self.say(T("cap.no_proposal"), RED)
            pid = p["id"]
        if isinstance(self.link, GatewayLink):
            self.link.send_msg({"t": "avoid_answer", "id": pid, "yes": yes})

    def on_driver_ask_done(self, d):
        self.gw_asks.pop(d.get("id"), None)
        if self.driver:
            self.driver.cancel(d.get("id"))

    def captain_confirm(self, what, on_yes):
        """Single-operator mode: the captain confirms their own driving action."""
        if self.cconf is not None:
            self.cconf.destroy()
        w = tk.Toplevel(self.root, bg="#f0e0ff")
        w.title(T("cap.cc_title"))
        w.transient(self.root)
        self.cconf = w
        tk.Label(w, text=T("cap.cc_q"), font=F_BIG, bg="#f0e0ff").pack(padx=20, pady=10)
        tk.Label(w, text=i18n.from_en(what), font=F_BIG, bg="#f0e0ff", fg="#7000a0").pack(padx=20, pady=6)
        left = tk.Label(w, text="", font=F_MED, bg="#f0e0ff")
        left.pack()
        b = tk.Frame(w, bg="#f0e0ff")
        b.pack(pady=12)
        deadline = time.time() + self.confirm_timeout_s

        def done(yes):
            if self.cconf is w:
                self.cconf = None
            w.destroy()
            if yes:
                on_yes()
            else:
                self.say(T("cap.not_confirmed", what=i18n.from_en(what)), RED)

        btn(b, "cap.cc_send", lambda: done(True), bg="#b0ffb0").pack(side="left", padx=8)
        btn(b, "cap.cancel", lambda: done(False)).pack(side="left", padx=8)

        def tick():
            if self.cconf is not w:
                return
            s = deadline - time.time()
            if s <= 0:
                return done(False)
            left.configure(text=T("cap.cc_left", s="%.0f" % s))
            w.after(250, tick)
        tick()
        self.cconf_send = lambda: done(True)            # tests

    def cmd(self, ftype, payload, what):
        if not self.link:
            self.say(T("cap.not_connected"), RED)
            return
        if ftype == proto.T_HELM and payload[:1] == bytes([0]) and self.boat is not None and not self.boat_ok:
            # a boat from the list: its name + live camera confirmed first (CLAUDE.md F)
            self.say(T("boat.confirm_first", name=self.boat.name), RED)
            if self.boatconf is None:
                self.boatconf = crewui.BoatConfirm(self, self.boat)
            return
        if isinstance(self.link, GatewayLink) and proto.is_driving_cmd(ftype, payload):
            if self.single_op:
                return self.captain_confirm(what, lambda: self._send_cmd(ftype, payload, what, confirm="self"))
            if crewui.crew_of(self)[0] == "copilot" and ftype not in (proto.T_AUTO_PAUSE,):
                # the gateway asks the captain's screen 1 (CLAUDE.md D)
                self.say(T("cap.wait_captain", what=i18n.from_en(what)), ORANGE)
                self.event(T("cap.ev_captain_asked", what=i18n.from_en(what)))
        self._send_cmd(ftype, payload, what)

    def _send_cmd(self, ftype, payload, what, confirm=None):
        if not self.link:
            return self.say(T("cap.not_connected"), RED)
        if isinstance(self.link, GatewayLink):
            seq = self.link.send(ftype, payload, what=what, confirm=confirm)
        else:
            seq = self.link.send(ftype, payload)
        self.pending[seq] = what
        self.event("> " + i18n.from_en(what))

    def mode(self, m):
        self.cmd(proto.T_SET_MODE, proto.p_set_mode(m), TE("what.mode", mode=proto.MODES[m]))

    # --- power limit raise ------------------------------------------------------
    def power_dialog(self, direction):
        """Ask the boat for a code, show it, the user types it back, then the
        joystick CONFIRM within 10 s does one step up."""
        if not self.link:
            return self.say(T("cap.not_connected"), RED)
        if self.pdlg is not None:
            self.pdlg.destroy()
        self.cmd(proto.T_POWER_CODE, proto.p_power_code(direction), TE("what.power_code", dir=("fwd", "rev")[direction]))
        d = tk.Toplevel(self.root, bg=BG)
        d.title(T("cap.pw_title"))
        d.transient(self.root)
        self.pdlg = d
        self.pdlg_dir = direction
        tk.Label(d, text=T("cap.pw_head_%s" % ("fwd", "rev")[direction]), font=F_BIG,
                 bg=BG).pack(padx=20, pady=(14, 4))
        lbl(d, "cap.pw_step1", font=F_MED, bg=BG).pack(anchor="w", padx=20)
        self.pdlg_code = tk.Label(d, text="....", font=("Consolas", 40, "bold"), bg="#ffffe0")
        self.pdlg_code.pack(padx=20, pady=6)
        self.pdlg_var = tk.StringVar()
        e = tk.Entry(d, textvariable=self.pdlg_var, font=("Consolas", 32, "bold"), width=6, justify="center")
        e.pack(pady=4)
        e.focus_set()
        lbl(d, "cap.pw_step2", font=F_MED, bg=BG).pack(anchor="w", padx=20, pady=(8, 4))
        b = tk.Frame(d, bg=BG)
        b.pack(pady=12)
        btn(b, "cap.pw_send", self.power_send, bg="#ffd000").pack(side="left", padx=6)
        btn(b, "cap.cancel", self.power_close).pack(side="left", padx=6)
        e.bind("<Return>", lambda _ev: self.power_send())

    def power_send(self):
        try:
            code = int(self.pdlg_var.get())
        except ValueError:
            return self.say(T("cap.pw_type_code"), RED)
        self.cmd(proto.T_POWER_RAISE, proto.p_power_raise(self.pdlg_dir, code), TE("what.power_raise", code=code))
        self.power_close()

    def power_close(self):
        if self.pdlg is not None:
            self.pdlg.destroy()
            self.pdlg = None

    def info_manual(self):
        self.say(T("cap.info_manual"))

    def info_hdg(self):
        self.say(T("cap.info_hdg"))

    def auto_start(self):
        self.cmd(proto.T_AUTO_START, b"", TE("what.auto_start"))

    def sim_confirm(self):
        """Simulator: press and release the joystick CONFIRM (stick centred)."""
        if not self.link:
            return
        if self.t and self.t["usb"]:
            return self.sim_usb_press(proto.JM_CONFIRM)
        self.link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        self.root.after(150, lambda: self.link and self.link.send(
            proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0)))

    def train_accept(self):
        try:
            full = float(self.full_var.get())
        except ValueError:
            return self.say(T("cap.full_angle_num"), RED)
        self.cmd(proto.T_TRAINING, proto.p_training(1, full), TE("what.training_accept"))

    def read_params(self):
        if self.link:
            for pid in range(32):
                self.link.send(proto.T_PARAM_GET, proto.p_param_get(pid))

    def write_param(self):
        sel = self.ptree.selection()
        if not sel:
            return self.say(T("cap.select_param"), RED)
        try:
            v = float(self.pval_var.get())
        except ValueError:
            return self.say(T("cap.enter_number"), RED)
        pid = int(sel[0])
        self.cmd(proto.T_PARAM_SET, proto.p_param_set(pid, v), TE("what.set_param", name=self.params[pid]["name"],
                                                                     v="%g" % v))

    def map_click(self, lat, lon):
        """Screen 2's map, "click adds waypoint": into the route editor (nothing is sent)."""
        if not self.add_on_click.get():
            return
        self.pg_vars["lat"].set("%.6f" % lat)
        self.pg_vars["lon"].set("%.6f" % lon)
        self.prog_add()
        self.say(T("cap.wp_added", n=len(self.prog["wps"])))

    # --- feedback ----------------------------------------------------------------
    def say(self, text, color="#202020"):
        self._evlog = (getattr(self, "_evlog", []) + [text])[-200:]
        self.msg.configure(text=text, bg=color)

    def ev_texts(self):
        """Recent event / message texts (tests)."""
        return list(getattr(self, "_evlog", []))

    def event(self, text):
        self._evlog = (getattr(self, "_evlog", []) + [text])[-200:]
        self.events.insert(0, time.strftime("%H:%M:%S  ") + text)
        if self.events.size() > 500:
            self.events.delete(500, "end")

    def poll(self):
        if self.link:
            n = 0
            while n < 500:
                try:
                    kind, d = self.link.events.get_nowait()
                except queue.Empty:
                    break
                n += 1
                if kind == "telemetry":
                    self.on_telem(d)
                elif kind == "ack":
                    what = self.pending.pop(d["seq"], None)
                    if d["cmd"] == proto.T_PARAM_GET and d["result"] == 10:
                        continue                # "Read all" probes past the last id
                    if what is not None or not d["ok"]:
                        txt = "%s: %s" % (i18n.from_en(what) if what else d["cmd_name"], alerts.result_text(d))
                        self.say(txt, GREEN if d["ok"] else RED)
                        self.event(("OK   " if d["ok"] else "NACK ") + txt)
                        if self.sound and what is not None:
                            self.sound.feedback("fb.confirm" if d["ok"] else "fb.refuse")
                elif kind == "param":
                    self.on_param(d)
                elif kind == "prog_wp":
                    self.prog_dl.append(d)
                    if len(self.prog_dl) >= d["n"]:
                        self.prog = program.from_wp_values(self.prog_dl)
                        self.prog_refresh()
                        self.say(T("cap.pg_downloaded", n=len(self.prog["wps"])), GREEN)
                elif kind == "prog_error":
                    where = "" if d["index"] == 255 else T("cap.pg_at_wp", n=d["index"] + 1)
                    why = T("prog.err_%d" % d["error"]) if i18n.has("prog.err_%d" % d["error"], "en") \
                        else d["text"]
                    self.say(T("cap.pg_refused", why=why, where=where), RED)
                    self.event(T("cap.pg_refused", why=why, where=where))
                elif kind == "joymap":
                    self.on_joymap(d)
                elif kind == "joy_raw":
                    self.on_raw(d)
                elif kind == "bench":
                    self.on_bench(d)
                elif kind == "gw_status":
                    self.gw_status = d
                    me = [s for s in d.get("stations", []) if s.get("id") == d.get("you")]
                    self.on_single_op(bool(me and me[0].get("single_op")))
                    self.draw_network()
                    self.draw_ais()
                    if self.auto_cams:
                        self.cam_preset(self.auto_cams)
                        self.auto_cams = None
                    self.draw_vision()
                    self.drv_vision()
                    self.drv_proposal()
                    if self.callouts:
                        self.callouts.gw(d)
                elif kind == "channel_ack":
                    self.on_channel_ack(d)
                elif kind == "cams_ack":
                    self.on_cams_ack(d)
                elif kind == "save_ack":
                    self.on_save_ack(d)
                elif kind == "vision_ack":
                    self.say(T("cap.vis_msg", text=i18n.from_en(d.get("text", ""))), GREEN if d.get("ok") else RED)
                    self.event(T("cap.ev_vision", text=i18n.from_en(d.get("text", ""))))
                elif kind == "avoid_ack":
                    self.say(T("cap.prop_msg", text=i18n.from_en(d.get("text", ""))), GREEN if d.get("ok") else RED)
                    self.event(T("cap.ev_proposal", text=i18n.from_en(d.get("text", ""))))
                elif kind == "driver_ask":
                    self.on_driver_ask(d)
                elif kind == "driver_ask_done":
                    self.on_driver_ask_done(d)
                elif kind == "single_op_ack":
                    self.on_single_op(bool(d.get("on")))
                elif kind == "crew_swap_ask":
                    crewui.ask_swap(self, d)
                elif kind == "crew_swap_done":
                    txt = T("crew.swap_ok") if d.get("ok") else T("crew.swap_refused",
                                                                   why=i18n.from_en(d.get("text", "")))
                    self.say(txt, GREEN if d.get("ok") else RED)
                    self.event(txt)
                elif kind == "crew":
                    self.event(T("crew.changed", why=i18n.from_en(d.get("why", ""))))
                elif kind == "status":
                    self.say(i18n.from_en(d), RED if "DISCONNECTED" in d else "#202020")
                    self.event(i18n.from_en(d))
            age = time.time() - self.t_rx if self.t_rx else 99
            ok = self.link.connected and age < 1.0
            self.link_lbl.configure(text=T("cap.link_ok") if ok else T("cap.link_nodata"), fg=GREEN if ok else RED)
        self.root.after(50, self.poll)

    def on_param(self, d):
        self.params[d["id"]] = d
        iid = str(d["id"])
        vals = ("%g" % d["value"], "%g" % d["min"], "%g" % d["max"])
        if self.ptree.exists(iid):
            self.ptree.item(iid, values=vals)
        else:
            self.ptree.insert("", "end", iid=iid, text=d["name"], values=vals)

    def on_telem(self, d):
        prev = self.t
        self.t = d
        self.t_rx = time.time()
        if (prev or {}).get("mode") != d.get("mode"):
            self.draw_bench()                   # entered / left the bench test
        if "slam_peak" in d:
            self.slam_hist.append((d["t_ms"], d["slam_peak"]))
        ts = d["t_ms"] / 1000.0
        if self.hist and ts < self.hist[-1][0]:
            self.hist.clear()               # boat restarted
        self.hist.append((ts, d["heading"], d["target"], d["outboard"]))
        while self.hist and self.hist[0][0] < ts - PLOT_S:
            self.hist.pop(0)
        if d["pos_source"] != 0:
            if not self.track or abs(self.track[-1][0] - d["est_lat"]) + abs(self.track[-1][1] - d["est_lon"]) > 2e-6:
                self.track.append((d["est_lat"], d["est_lon"]))
                if len(self.track) > 5000:
                    self.track.pop(0)
        if prev and prev["mode"] != d["mode"]:
            self.event(T("cap.ev_mode", a=T("mode." + prev["mode_name"]), b=T("mode." + d["mode_name"])))
        if prev and prev["alarm_bits"] != d["alarm_bits"]:
            new = d["alarm_bits"] & ~prev["alarm_bits"]
            if new:
                self.event(T("cap.alarm_banner", list=", ".join(alerts.texts(new))))
        if self.callouts:
            self.callouts.telem(prev, d)
        # draw at most 10 times per second
        if not prev or d["t_ms"] - getattr(self, "_drawn", 0) >= 100 or d["t_ms"] < getattr(self, "_drawn", 0):
            self._drawn = d["t_ms"]
            self.draw()

    # --- drawing -----------------------------------------------------------------
    def draw(self):
        d = self.t
        mode = d["mode_name"]
        self.mode_lbl.configure(text=T("mode." + mode), bg=MODE_COLORS.get(mode, "#606060"))
        v = self.vals
        v["heading"].configure(text="%05.1f°" % d["heading"] if d["compass_ok"] else T("cap.no_compass"),
                               fg=FG if d["compass_ok"] else RED)
        v["target"].configure(text="%05.1f°" % d["target"])
        v["yaw"].configure(text="%+.1f °/s" % d["yaw_rate"])
        v["outboard"].configure(text="%+.1f°" % d["outboard"])
        v["throttle"].configure(text="%.0f %%" % d["throttle"] if d["throttle_demand"] <= d["throttle"] + 0.5
                                else T("cap.thr_asked", thr="%.0f" % d["throttle"], ask="%.0f" % d["throttle_demand"]))
        self.power_lbl.configure(text=T("cap.power_lbl", f="%.0f" % d["power_fwd"], r="%.0f" % d["power_rev"]),
                                 bg="#ffc080" if d["power_raised"] else "#d0ffd0")
        if self.pdlg is not None:
            ok = d["power_code"] and d["power_code_dir"] == self.pdlg_dir
            self.pdlg_code.configure(text="%04d" % d["power_code"] if ok else "....")
        v["gear"].configure(text=T("gear.%d" % d["gear"]) if d["gear"] < 3 else "?",
                            fg={0: FG, 1: GREEN, 2: RED}.get(d["gear"], FG))
        v["gps"].configure(text=T("cap.fix") if d["gps_fix"] else T("cap.no_fix"), fg=GREEN if d["gps_fix"] else RED)
        v["pos"].configure(text="%.6f  %.6f" % (d["est_lat"], d["est_lon"]))
        v["sog"].configure(text="%.1f kn  %03.0f°" % (d["est_sog"], d["est_cog"]))
        src = T("pos.%d" % d["pos_source"]) if d["pos_source"] < 4 else "?"
        if d["pos_source"] in (2, 3):
            src += " (%s, %.0f s)" % (T("dr.%d" % min(d["dr_reason"], len(proto.DR_REASONS) - 1)), d["dr_time_s"])
        v["src"].configure(text=src, fg=FG if d["pos_source"] == 1 else RED)
        v["unc"].configure(text="± %.0f m" % d["unc_m"])
        v["drift"].configure(text="%03.0f° %.2f kn (%.0f %%)" % (d["set_deg"], d["drift_kn"], 100 * d["current_conf"]))
        v["current"].configure(text="%.1f A" % d["motor_current"] if d["motor_current"] >= 0 else T("cap.no_sensor"))
        v["cruise"].configure(text="%.1f kn" % d["speed_set_kn"] if d["mode"] == proto.MODE_AUTO else "-")
        v["wp"].configure(text="%d/%d  %.1f kn" % (d["auto_wp"] + 1, d["prog_n"], d["wp_kn"]) if d["prog_ok"]
                          else T("cap.no_program"))
        for name, b in self.mode_btns.items():
            on = mode == name
            b.configure(bg=MODE_COLORS[name] if on else PANEL, fg="#ffffff" if on else FG)
        st = T("auto.%d" % d["auto_state"]) if d["auto_state"] < 4 else "?"
        if not d["prog_ok"]:
            txt = T("cap.no_program_boat")
        elif d["mode"] == proto.MODE_AUTO:
            txt = T("cap.auto_wp", n=d["auto_wp"] + 1, of=d["prog_n"], state=st)
            if d["speed_limited"]:
                txt += "\n" + T("cap.speed_limited")
        elif d["prog_suspended"]:
            txt = T("cap.prog_interrupted", n=d["auto_wp"] + 1)
        elif d["prog_finished"]:
            txt = T("cap.prog_finished")
        else:
            txt = T("cap.prog_stored", n=d["prog_n"])
        self.auto_lbl.configure(text=txt)
        if d["mode"] == proto.MODE_TRAINING:
            tr = T("train.stage_%d" % min(d["train_stage"], 2))
            if d["train_stage"] >= 1:
                tr += " / " + T("train.phase_%d" % min(d["train_phase"], 10))
        else:
            tr = T("cap.train_last", r=T("train.result_%d" % min(d["train_result"], 11)))
        v["train"].configure(text=tr)

        # banners
        if d["confirm_kind"]:
            self.joy_banner.configure(text=T("cap.confirm_banner", what=T("confirm.%d" % min(
                d["confirm_kind"], len(proto.CONFIRM_KINDS) - 1)), s="%.0f" % d["confirm_left_s"]), bg="#ffd000")
        elif d["mode"] == proto.MODE_REMOTE:
            st = d.get("remote_state", 1)
            self.joy_banner.configure(text=T("cap.remote_lost_banner") if st == 2 else T("cap.remote_banner"),
                                      bg=RED if st == 2 else BLUE)
        else:
            self.joy_banner.configure(text=T("cap.joy_in_control"), bg=ORANGE)
        self.show(self.joy_banner, d["mode"] in (proto.MODE_MANUAL, proto.MODE_REMOTE) or d["confirm_kind"] != 0,
                  row=0, column=0, columnspan=2, sticky="ew")
        alarms = alerts.texts(d["alarm_bits"])
        on = d["alarm"] or bool(alarms)
        if on:
            self.alarm_banner.configure(text=T("cap.alarm_banner", list=", ".join(alarms) if alarms
                                               else T("cap.alarm_autopilot")))
        self.show(self.alarm_banner, on, row=1, column=0, sticky="ew")
        self.show(self.alarm_ack, on, row=1, column=1, sticky="ns")
        pick = d["pickup"] and d["mode"] in (proto.MODE_MANUAL, proto.MODE_HDG_LOCK)
        if pick:
            tgt = d["pickup_target"]
            self.pick_banner.configure(text=T("cap.pickup_banner", to="%.0f %%" % (100 * tgt) if tgt is not None else
                                              T("cap.pickup_neutral", lo="%.0f" % (100 * d["nz_lo"]),
                                                hi="%.0f" % (100 * d["nz_hi"])),
                                              now="%.0f" % (100 * (d["lever"] or 0.0))))
        self.show(self.pick_banner, pick, row=2, column=0, columnspan=2, sticky="ew")
        self.show(self.ft_banner, bool(d.get("first_test")), row=6, column=0, columnspan=2, sticky="ew")
        self.draw_crew(d)

        per = ((50000, "cap.sys_core0"), (1000, "cap.sys_cpu1"), (10000, "cap.sys_core2"))
        keys = (("t_core0_avg", "t_core0_max", "ovr_core0"), ("t_mon_avg", "t_mon_max", "ovr_mon"),
                ("t_log_avg", "t_log_max", "ovr_log"))
        lines = ["%-22s %8s %8s %7s %9s" % ("", T("cap.sys_avg"), T("cap.sys_max"), T("cap.sys_maxpc"),
                                           T("cap.sys_overruns"))]
        for (p, name), (ka, km, ko) in zip(per, keys):
            lines.append("%-22s %8d %8d %6.1f%% %9d" % (T(name), d[ka], d[km], 100.0 * d[km] / p, d[ko]))
        lines.append("")
        lines.append(T("cap.sys_link", n=d["rx_frames"], crc=d["rx_crc_errors"]))
        lines.append(T("cap.sys_log", sd=T("cap.sd_present") if d["log_flags"] & 2 else T("cap.sd_missing"),
                       state=T("cap.log_failure") if d["log_flags"] & 1 else T("cap.log_writing"),
                       n=d["log_files"], dropped=d["log_dropped"]))
        hid = d.get("helm")
        if hid is not None:
            name = {0: T("cap.helm_nobody"), 1: T("cap.helm_usb")}.get(hid, T("cap.helm_station", n=hid))
            if self.gw_status and self.gw_status.get("helm"):
                name = self.gw_status["helm"]["name"]
            mine = (self.gw_status or {}).get("you")
            me = (hid == mine) if mine else (hid == 1 and isinstance(self.link, Link))
            self.helm_lbl.configure(text=T("cap.helm_you", who=name) if me else T("cap.helm", who=name),
                                    bg="#b0ffb0" if me else ("#ffd080" if hid else PANEL))
        if self.gw_status:
            lines.append("")
            lines.append(T("cap.sys_stations"))
            for st in self.gw_status.get("stations", []):
                lines.append("  %-3d %-24s %-7s %-7s %6s ms  %s %s%s" % (
                    st["id"], st["name"][:24], st["role"], T("cap.on_board") if st["local"] else T("cap.remote"),
                    st["rtt_ms"] if st["rtt_ms"] is not None else "-", T("cap.sys_telemetry"), st["tmode"],
                    T("cap.sys_you") if st["id"] == self.gw_status.get("you") else ""))
        if "gw_state" in d:
            lines.append(T("cap.sys_gw", state=d["gw_state_name"], n=d["gw_stations"], seq=d["pi_log_seq"],
                           gaps=d["pi_log_gaps"], dropped=d["log_dropped_pi"], skipped=d["log_skipped"]))
        self.sys_lbl.configure(text="\n".join(lines))

        self.draw_joystick(d)
        self.draw_remote(d)
        shown = self.nb.select()                    # draw the map / plot only while their tab is open
        if shown == str(self.plot_tab):
            self.draw_plot()

    def draw_joystick(self, d):
        c = self.joy_cv
        c.delete("all")
        s = 130
        c.create_line(s / 2, 0, s / 2, s, fill="#a0a0a0")
        c.create_line(0, s / 2, s, s / 2, fill="#a0a0a0")
        db = 0.15 * s / 2
        c.create_rectangle(s / 2 - db, s / 2 - db, s / 2 + db, s / 2 + db, outline="#a0a0a0", dash=(2, 2))
        st = d["joy_status"]
        col = {0: "#a0a0a0", 1: ORANGE, 2: BLUE, 3: RED}.get(st, FG)
        if d["mode"] == proto.MODE_MANUAL:
            col = ORANGE
        x = s / 2 + d["joy_x"] * (s / 2 - 8)
        y = s / 2 - d["joy_y"] * (s / 2 - 8)
        c.create_oval(x - 9, y - 9, x + 9, y + 9, fill=col, outline=FG, width=2)
        self.draw_lever(d)
        txt = T("joy.st_%d" % st) if st < 4 else "?"
        if st == 3 or (st == 1 and d["joy_why"]):
            txt += "\n" + joy_why(d)
        elif st == 1:
            txt = T("joy.not_neutral") if d["has_lever"] else txt
        self.joy_status.configure(text="%s\nX %+.2f  Y %+.2f" % (txt, d["joy_x"], d["joy_y"]),
                                  fg=RED if st in (1, 3) else FG)
        if d["usb"]:
            ok = lambda b: T("cap.ok") if b else T("joy.unplugged")         # noqa: E731
            src = T("joy.usb_src", fps=d["usb_fps"], crc=d["usb_crc"], stick=ok(d["usb_dev0"]),
                    thr=ok(d["usb_dev1"])) + (T("joy.learn") if d["usb_learn"] else
                                              T("joy.starting") if d["usb_starting"] else "")
        else:
            src = T("joy.analog")
        self.joy_src.configure(text=src, fg=RED if d["usb"] and not (d["usb_dev0"] and d["usb_dev1"]) else FG)
        self.joy_btns.configure(text="%s %s  %s %s  %s %s" % (
            T("jm.0"), "■" if d["joy_take"] else "□", T("jm.1"), "■" if d["joy_auto"] else "□",
            T("jm.2"), "■" if d["joy_confirm"] else "□"))
        self.draw_joytab(d)

    def draw_lever(self, d):
        """Lever bar: forward up, reverse down, the neutral zone in grey, the
        pickup target as a yellow mark."""
        c = self.lever_cv
        c.delete("all")
        h, w = 130, 54
        if d["lever"] is None:
            c.create_text(w / 2, h / 2, text=T("joy.no_lever"), font=F_TXT, justify="center")
            return
        top, bot = 12, h - 12

        def Y(v):
            return bot - v * (bot - top)
        c.create_rectangle(16, Y(1.0), w - 16, Y(d["nz_hi"]), fill="#c8f0c8", outline="")
        c.create_rectangle(16, Y(d["nz_lo"]), w - 16, Y(0.0), fill="#f8c8c8", outline="")
        pick = d["pickup"] and d["mode"] in (proto.MODE_MANUAL, proto.MODE_HDG_LOCK)
        nz_col = "#ffd000" if pick and d["pickup_target"] is None else "#a0a0a0"
        c.create_rectangle(16, Y(d["nz_hi"]), w - 16, Y(d["nz_lo"]), fill=nz_col, outline="")
        c.create_text(w / 2, 6, text=T("gear.1"), font=F_TXT)
        c.create_text(w / 2, h - 5, text=T("gear.2"), font=F_TXT)
        if pick and d["pickup_target"] is not None:
            ty = Y(d["pickup_target"])
            c.create_polygon(w - 14, ty, w - 2, ty - 7, w - 2, ty + 7, fill="#ffd000", outline=FG)
        ly = Y(d["lever"])
        c.create_rectangle(6, ly - 4, w - 16, ly + 4, fill=ORANGE if pick else BLUE, outline=FG)

    # --- Joystick tab ---------------------------------------------------------------
    def jm_to_widgets(self):
        m = self.jm_edit
        self.jm_src.set(m["source"])
        self.jm_thr.set(m["thr_input"])
        for k in ("nz_lo", "nz_hi", "lever_delta", "pickup_win"):
            self.jm_vars[k].set("%g" % round(100 * m[k], 1))
        names = [ui_axis_name(m["axis_steer"]) + (T("joy.inverted") if m["inv_steer"] else ""),
                 ui_axis_name(m["axis_thr"]) + (T("joy.inverted") if m["inv_thr"] else "")]
        names += [ui_code_name(cd) for cd in m["btn"]]
        for (lbl, _), n in zip(self.jm_rows, names):
            lbl.configure(text=n)

    def jm_from_widgets(self):
        m = self.jm_edit
        m["source"] = self.jm_src.get()
        m["thr_input"] = self.jm_thr.get()
        for k in ("nz_lo", "nz_hi", "lever_delta", "pickup_win"):
            try:
                m[k] = float(self.jm_vars[k].get()) / 100.0
            except ValueError:
                pass
        return m

    def jm_read(self):
        self.jm_load = True
        self.cmd(proto.T_JOYMAP_GET, b"", TE("what.joymap_read"))

    def jm_default(self):
        self.jm_edit = proto.default_joymap(self.jm_src.get())
        self.jm_to_widgets()
        self.say(T("cap.jm_factory"))

    def jm_save(self):
        m = self.jm_from_widgets()
        err = proto.check_joymap(m)
        if err:
            return self.say(T("cap.jm_not_sent", why=i18n.from_en(err)), RED)
        if self.t and self.t["mode"] != proto.MODE_STANDBY:
            return self.say(T("cap.jm_not_sent", why=T("cap.jm_standby")), RED)
        self.cmd(proto.T_JOYMAP_SET, proto.p_joymap_set(m, True), TE("what.joymap_save"))

    def on_joymap(self, d):
        self.jmap = d
        if self.jm_load:
            self.jm_load = False
            self.jm_edit = dict(d, btn=list(d["btn"]))
            self.jm_to_widgets()

    def raw_tick(self):
        """Poll the raw USB frame (5 Hz) while the Joystick tab is open or the
        mapping routine runs."""
        if not self.link:
            return
        try:
            on_tab = self.nb.select() == str(self.joytab)
        except tk.TclError:
            on_tab = False
        if self.t and self.t["usb"] and (on_tab or self.wiz):
            self.link.send(proto.T_JOY_RAW_GET)
        self.root.after(200, self.raw_tick)

    def on_raw(self, d):
        self.raw = d
        if self.wiz:
            self.wiz_raw(d)

    @staticmethod
    def code_now(d, code):
        """Is control `code` pressed now (telemetry)?"""
        if 1 <= code <= 64:
            c = code - 1
            return bool((d["usb_btn1"] if c >= 32 else d["usb_btn0"]) >> (c % 32) & 1)
        if 65 <= code <= 80:
            c = code - 65
            dev, hat, dr = c // 8, (c // 4) % 2, c % 4
            if hat:
                return False                     # only hat 0 is in the telemetry
            v = (d["usb_hats"] >> 4) if dev else (d["usb_hats"] & 0x0F)
            return v == 2 * dr
        return False

    def draw_joytab(self, d):
        con = lambda b: "%-10s" % (T("joy.connected") if b else T("joy.unplugged"))         # noqa: E731
        if d["usb"]:
            lines = [T("joy.tab_usb", fps="%3d" % d["usb_fps"], crc=d["usb_crc"]),
                     T("joy.tab_devs", stick=con(d["usb_dev0"]), thr=con(d["usb_dev1"]),
                       hub="%-4s" % (T("cap.yes") if d["usb_hub"] else T("cap.no")),
                       learn=T("joy.tab_learn") if d["usb_learn"] else "")]
        else:
            lines = [T("joy.tab_analog"), ""]
        st = d["joy_status"]
        lines.append(T("joy.tab_status", st=T("joy.st_%d" % st) if st < 4 else "?", why=joy_why(d)))
        if d["lever"] is not None:
            lines.append(T("joy.tab_lever", lever="%.0f" % (100 * d["lever"]), lo="%.0f" % (100 * d["nz_lo"]),
                           hi="%.0f" % (100 * d["nz_hi"]), pickup=T("joy.pickup_pending") if d["pickup"] else ""))
        self.jl_lbl.configure(text="\n".join(lines), fg=RED if st in (1, 3) else FG)
        m = self.jm_edit
        for i, (_, now) in enumerate(self.jm_rows):
            if i < 2:
                now.configure(text="")
                continue
            code = m["btn"][i - 2]
            on = d["usb"] and code and self.code_now(d, code)
            now.configure(text=T("joy.pressed") if on else "", fg=GREEN)

    # --- guided button mapping ------------------------------------------------------

    def wiz_start(self, steps):
        if not self.t or not self.t["usb"]:
            return self.say(T("wiz.need_usb"), RED)
        if self.t["mode"] != proto.MODE_STANDBY:
            return self.say(T("wiz.only_standby"), RED)
        self.jm_from_widgets()
        full = steps is None
        if full:
            steps = ["axis_steer", "axis_thr"] + list(range(len(proto.JM_BUTTONS)))
        self.wiz = {"steps": ["base"] + steps, "i": 0, "base": None, "release": False, "full": full}
        self.wiz_frame.pack(fill="x", padx=6, pady=6, before=self.jl_lbl)    # at the top: always visible
        self.wiz_show()

    def wiz_step(self):
        w = self.wiz
        return w["steps"][w["i"]] if w and w["i"] < len(w["steps"]) else None

    def wiz_show(self, info=""):
        st = self.wiz_step()
        if st is None:
            err = proto.check_joymap(self.jm_edit)
            self.wiz_lbl.configure(text=T("wiz.finished") if not err else T("wiz.finished_but", err=i18n.from_en(err)))
            self.wiz_info.configure(text="")
            self.wiz_next.configure(text=T("cap.close"), command=self.wiz_cancel)
            self.wiz_skip.pack_forget()
            self.jm_to_widgets()
            self.wiz = None
            return
        self.wiz_next.configure(text=T("cap.wiz_next"), command=self.wiz_next_click)
        if st == "base":
            txt = T("wiz.base")
        elif st == "axis_steer":
            txt = T("wiz.axis_steer")
        elif st == "axis_thr":
            txt = T("wiz.axis_thr_lever" if self.jm_thr.get() == 1 else "wiz.axis_thr_y")
        else:
            txt = T("wiz.press", func=T("jm.%d" % st), hint=T("wiz.hint_%d" % st))
        optional = isinstance(st, int) and st not in (proto.JM_TAKE, proto.JM_AUTO, proto.JM_CONFIRM)
        if optional:
            self.wiz_skip.pack(side="left", padx=4)
        else:
            self.wiz_skip.pack_forget()
        self.wiz_lbl.configure(text=txt)
        self.wiz_info.configure(text=info)

    def wiz_cancel(self):
        self.wiz = None
        self.wiz_frame.pack_forget()
        self.jm_to_widgets()

    def wiz_advance(self, info=""):
        self.wiz["i"] += 1
        self.jm_to_widgets()
        self.wiz_show(info)

    def wiz_skip_click(self):
        st = self.wiz_step()
        if isinstance(st, int):
            self.jm_edit["btn"][st] = 0
            self.wiz_advance(T("wiz.not_used", func=T("jm.%d" % st)))

    def wiz_next_click(self):
        w, st, raw = self.wiz, self.wiz_step(), self.raw
        if w is None or st is None:
            return
        if raw is None:
            return self.wiz_info.configure(text=T("wiz.no_data"))
        if st == "base":
            w["base"] = raw
            return self.wiz_advance()
        if st in ("axis_steer", "axis_thr"):
            best, code = 0.0, None
            for dv in range(2):
                for a in range(8):
                    v, b = raw["dev"][dv]["axes"][a], w["base"]["dev"][dv]["axes"][a]
                    cd = dv * 8 + a
                    if v is None or b is None or (st == "axis_thr" and cd == self.jm_edit["axis_steer"]):
                        continue
                    if abs(v - b) > abs(best):
                        best, code = v - b, cd
            if code is None or abs(best) < 0.5:
                return self.wiz_info.configure(text=T("wiz.no_axis"))
            key = "steer" if st == "axis_steer" else "thr"
            self.jm_edit["axis_" + key] = code
            self.jm_edit["inv_" + key] = 1 if best < 0 else 0
            return self.wiz_advance("%s = %s%s" % (T("wiz.steering") if key == "steer" else T("wiz.throttle"),
                                                   ui_axis_name(code), T("joy.inverted") if best < 0 else ""))
        # buttons are detected automatically (wiz_raw); Next does nothing here
        self.wiz_info.configure(text=T("wiz.press_itself"))

    def wiz_raw(self, raw):
        """Button steps: the first newly pressed button / hat direction is taken.
        A control already used by another function is refused."""
        w, st = self.wiz, self.wiz_step()
        if not isinstance(st, int) or w["base"] is None:
            return
        base = set(proto.pressed_codes(w["base"]))
        now = [c for c in proto.pressed_codes(raw) if c not in base]
        if w["release"]:
            if not now:
                w["release"] = False
            return
        if not now:
            return
        code = now[0]
        for i, other in enumerate(self.jm_edit["btn"]):
            if other == code and i != st:
                self.wiz_info.configure(text=T("wiz.already", ctrl=ui_code_name(code), func=T("jm.%d" % i)), fg=RED)
                w["release"] = True
                return
        self.wiz_info.configure(fg=FG)
        self.jm_edit["btn"][st] = code
        w["release"] = True
        self.wiz_advance("%s = %s" % (T("jm.%d" % st), ui_code_name(code)))

    # --- simulator: USB joystick ---------------------------------------------------
    def sim_usb_set(self, key, value):
        self.sim_usb[key] = value
        self.sim_usb_send()

    def sim_usb_send(self, codes=()):
        if not self.link:
            return
        b = [0, 0]
        hats = [15, 15]
        for c in codes:
            if 1 <= c <= 64:
                b[(c - 1) // 32] |= 1 << ((c - 1) % 32)
            elif 65 <= c <= 80:
                k = c - 65
                if (k // 4) % 2 == 0:
                    hats[k // 8] = 2 * (k % 4)
        u = self.sim_usb
        self.link.send(proto.T_SIM_BRIDGE, proto.p_sim_bridge(
            u["x"], 0.0, u["lever"], b[0], b[1], hats[0], hats[1], u["stick"], u["throttle"], False, u["fault"]))

    def sim_usb_press(self, func):
        """Press and release the USB button mapped to function `func`."""
        m = self.jmap or proto.default_joymap(1)
        code = m["btn"][func]
        if not code:
            return self.say(T("cap.not_mapped", func=T("jm.%d" % func)), RED)
        self.sim_usb_send([code])
        self.root.after(200, self.sim_usb_send)

    def draw_plot(self):
        c = self.plot
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 50 or not self.hist:
            return
        top_h = h * 0.62
        t1 = self.hist[-1][0]
        t0 = t1 - PLOT_S
        X = lambda t: 60 + (t - t0) / PLOT_S * (w - 80)
        # heading vs target, window centred on the target
        ref = self.hist[-1][2]
        span = max(20.0, max(abs(((p[1] - ref + 180) % 360) - 180) for p in self.hist) * 1.2)
        Y1 = lambda deg: 20 + (1 - (((deg - ref + 180) % 360 - 180) + span) / (2 * span)) * (top_h - 40)
        c.create_text(10, 8, text=T("plot.heading", span="%.0f" % span, ref="%.0f" % ref), anchor="nw", font=F_MED)
        for k in (-1, 0, 1):
            y = Y1(ref + k * span * 0.5)
            c.create_line(60, y, w - 20, y, fill="#d0d0d0")
            c.create_text(55, y, text="%.0f" % ((ref + k * span * 0.5) % 360), anchor="e", font=F_TXT)
        for idx, col, wd in ((2, BLUE, 2), (1, FG, 3)):
            pts = []
            for p in self.hist:
                pts += [X(p[0]), Y1(p[idx])]
            if len(pts) >= 4:
                c.create_line(*pts, fill=col, width=wd)
        # outboard angle
        oy0 = top_h + 10
        Y2 = lambda a: oy0 + 20 + (1 - (max(-40, min(40, a)) + 40) / 80) * (h - oy0 - 30)
        c.create_text(10, oy0, text=T("plot.outboard"), anchor="nw", font=F_MED)
        for a in (-30, 0, 30):
            c.create_line(60, Y2(a), w - 20, Y2(a), fill="#d0d0d0")
            c.create_text(55, Y2(a), text="%+d" % a, anchor="e", font=F_TXT)
        pts = []
        for p in self.hist:
            pts += [X(p[0]), Y2(p[3])]
        if len(pts) >= 4:
            c.create_line(*pts, fill=ORANGE, width=3)
        c.create_text(w - 20, h - 4, text=T("plot.last", s="%.0f" % PLOT_S), anchor="se", font=F_TXT)

    # --- alarm sound -------------------------------------------------------------
    def alarm_tick(self):
        d = self.t
        stale = self.link is not None and self.t_rx and time.time() - self.t_rx > 3.0
        # MASTER lamps + the repeating warning sounds work with or without the driver screen open
        self.lamps.update(d["alarm_bits"] if d else 0, no_data=bool(stale))
        if self.callouts and self.link is not None and self.t_rx:
            self.callouts.link(not stale)
        if (d and (d["alarm"] or d["alarm_bits"])) or stale:
            flash = int(time.time() * 2) % 2 == 0
            self.alarm_banner.configure(bg=RED if flash else "#800000")
            if stale:
                self.alarm_banner.configure(text=T("cap.alarm_nodata"))
                self.show(self.alarm_banner, True, row=1, column=0, sticky="ew")
            if time.time() - self.last_beep > 1.0 and not (self.sound and self.sound.enabled()):
                # without the sound system (section D) only: it plays the warnings itself
                self.last_beep = time.time()
                if winsound:     # in a thread: Beep blocks while it sounds
                    f = 2000 if (d and d["mode"] == proto.MODE_MANUAL) else 1500
                    threading.Thread(target=self._beep, args=(f,), daemon=True).start()
                else:
                    self.root.bell()
        self.root.after(500, self.alarm_tick)

    @staticmethod
    def _beep(freq):
        try:
            winsound.Beep(freq, 250)
        except RuntimeError:
            pass

    def quit(self):
        if self._layout_job:
            self.root.after_cancel(self._layout_job)
        self.save_layout()
        if self.driver:
            self.driver.destroy()
            self.driver = None
        if self.link:
            self.link.stop()
        self.root.destroy()


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Trygons C2 ground station")
    ap.add_argument("address", nargs="?", help="tcp:host:port, COM5 or gw:host:port")
    ap.add_argument("--login", help="user:password for the gateway (else a login window)")
    ap.add_argument("--driver", action="store_true", help="open the DRIVER screen too")
    ap.add_argument("--charts", help="chart folder (default: <repo>/charts)")
    ap.add_argument("--layout", help="window-place file (default: gs_layout.json here)")
    ap.add_argument("--title", help="window title (e.g. ON BOARD)")
    ap.add_argument("--cams", help="camera preset to start with (e.g. Full)")
    a = ap.parse_args()
    login = tuple(a.login.split(":", 1)) if a.login and ":" in a.login else None
    root = tk.Tk()
    app = App(root, a.address, login=login, charts_dir=a.charts, open_driver=a.driver, layout=a.layout,
              cams=a.cams)
    if a.title:
        app.title_extra = "%s - " % a.title
        root.title(app.title_extra + T("cap.title"))
    root.mainloop()


if __name__ == "__main__":
    main()
