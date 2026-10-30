"""Trygons C2 ground station - screen 2: NAVIGATION & CONTROL (tkinter).

    python app.py                      choose the boat / port in the window
    python app.py tcp:127.0.0.1:5760   connect to the simulator at once
    python app.py COM5                 connect to the board at once
    python app.py gw:trygons-gw:5761   through the gateway PC, asks for the login
Options (the demo uses them): --login user:password (no login window),
--driver (open screen 1, the driving screen, too), --charts FOLDER (other chart files),
--place shore | onboard (shown in the window title).

Screen 2 (CLAUDE.md section E): the MAP fills the window - the boat always in the centre,
north-up - and everything else lies on it in four corner panels (overlay.py; semi-transparent,
collapsible, never over the boat):
  top-left      first line: the boat dropdown (boats/ by name) + the lamp CONNECTED (found +
                connected + name check passed) + "name verified" (CLAUDE.md H); then mode, power
                limit, heading / target / rate of turn / gear / throttle / GPS
  top-right     the ALARMS (always drawn, one per line, flashing until acknowledged on this PC,
                ACK ALARM), then HELM + captain / copilot,
                the link (channels, CREWED / UNCREWED, near shore / open sea, lost-link countdown, engine)
  bottom-left   mode buttons, AUTO program + route / geofence / max speed editing, heading lock
                (+ STANDBY, always visible, bottom-left corner)
  bottom-right  AIS list + collision alerts, above the logo
Banners (joystick, confirm, lost link ...) at the top between the panels; zoom / back to boat /
tools at the bottom. The other pages (connection, AIS details, route table, plot, parameters,
joystick, cameras, remote joystick, network, system, bench test, sound, events) are in the
TOOLS window (menu Tools or the Tools button). Look: theme.py (shared with screen 1).
Manual driving (REMOTE MANUAL) and MANEUVER are started from screen 1 by a user with the manual
licence (CLAUDE.md D); here only in single-operator mode by a licensed user.
Everything is drawn locally (no internet needed): map, track, plot.
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
import theme
import alerts
import boats
import crewui
import academymenu
import about
import monitors
import overlay
from i18n import T, TE
from alerts import CAN_ROLE_KEYS, CAN_STATE_KEYS, CAN_WHY_KEYS, can_roles  # noqa: E402  (DroneCAN, 11d)
from link import Link, list_ports
from gwclient import GatewayLink
import shorejoy
import syncrecv
import video
import driver
import chart
import mapview
import mapwin
import collections
import json
from tkinter import messagebox, simpledialog

try:
    import winsound
except ImportError:          # not Windows
    winsound = None

theme.dpi_aware()           # sharp text at 125 / 150 % Windows scaling (before the first window)

# --- look: everything from theme.py (the same module as screen 1) -----------------------------------
C = theme.C
RED, ORANGE, GREEN, BLUE = theme.WARN, theme.CAUTION, theme.GOOD, theme.ADVISORY
F_HUGE, F_BIG, F_MED, F_TXT = theme.font("huge"), theme.font("title"), theme.font("value"), theme.font("label")
F_MONO = theme.font("label", bold=False, mono=True)
F_SMALL = theme.font("small")
MODE_COLORS = theme.MODE_COLOURS
PLOT_S = 120.0              # seconds shown in the plot
RELAY_PUSH_S = 15.0         # the boat file's relay table: no re-send to the AURIX before this (CLAUDE.md M)
THRUSTER_PUSH_S = 15.0      # the boat file's DP thruster settings: no re-send before this (CLAUDE.md P)
DP_MAX_KN = 3.0             # DP CRUISE / TRIP speed ceiling (the AURIX's dp_max_kn, TO CONFIRM; it refuses more)
DP_KN_STEP = 0.5
CRANK_PERIOD_MS = 200       # engine START held: CRANK to the AURIX at 5 Hz (it releases after 300 ms without one)
LAYOUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gs_layout.json")   # window places (not in git)
P_MAX_LEG = 4               # parameter "max_leg_kn" (app/link.c): the highest leg speed of a program
AIS_ROWS = (2, 4, 8)        # AIS rows in the corner panel: low (< 720 px) / small / big screen
ALARM_ROWS = (5, 8)         # alarm rows in the top-right panel: small / big screen (then "+N more")

# Captain actions that change how the boat is driven (proto.DRIVING_CMDS, incl. a new AUTO program).
# Through the gateway the GATEWAY enforces it (CLAUDE.md D): a copilot's waits for the captain's
# ACCEPT on screen 1 (shown here as WAITING / ACCEPTED / REJECTED), SINGLE-OPERATOR mode = the
# own confirmation. On the direct USB laptop link they go straight to the boat.
DRIVING_CMDS = proto.DRIVING_CMDS


def btn(parent, key, cmd, kind="normal", font=None, width=None, **kw):
    """A button whose text is the language entry `key` (follows the language switch).
    kind: normal / ok / no / info / warn / sim / standby (theme.button_opts)."""
    b = tk.Button(parent, command=cmd, **theme.button_opts(kind))
    if font:
        b.configure(font=font)
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
    return T("jcode.code", n=code)


def ui_axis_name(code):
    if code >= 16:
        return "-"
    return T("jcode.axis", dev=T("jdev.%d" % (code // 8)), n=code % 8, name=proto.AXIS_NAMES[code % 8])


def lbl(parent, key, **opts):
    """A label whose text is the language entry `key`."""
    opts.setdefault("bg", C["bg"])
    opts.setdefault("fg", C["fg"])
    return i18n.bind(tk.Label(parent, **opts), key)


def chk(parent, key, **opts):
    opts.setdefault("bg", C["bg"])
    opts.setdefault("fg", C["fg"])
    opts.setdefault("selectcolor", C["entry"])
    opts.setdefault("activebackground", C["bg"])
    return i18n.bind(tk.Checkbutton(parent, **opts), key)


def quality(c):
    """A channel's quality 0..4 (bars) from up / internet / latency / loss (CLAUDE.md B link panel)."""
    if not c.get("up"):
        return 0
    if not c.get("internet"):
        return 1
    q = 4
    lat, loss = c.get("latency_ms"), c.get("loss_pct")
    if lat is not None:
        q -= (lat > 150) + (lat > 400) + (lat > 900)
    if loss is not None:
        q -= (loss > 2) + (loss > 10)
    return max(1, q)


wrap = overlay.wrap_lines        # word wrap for the banners / message line


def st_text(state, family="st."):
    """A state name the gateway / boat sends in English (OK, FROZEN, SYNCING ...) in the screen language."""
    key = family + str(state)
    return T(key) if i18n.has(key, "en") else str(state)


class VTabs:
    """The tools window's pages: a vertical list of page buttons (never cut off, any language / scaling)
    + the page shown. Like ttk.Notebook for the rest of the code: add / select / tab / tabs."""

    def __init__(self, parent):
        self.side = tk.Frame(parent, bg=C["head"])
        self.side.pack(side="left", fill="y")
        self.body = tk.Frame(parent, bg=C["bg"])
        self.body.pack(side="left", fill="both", expand=True)
        self.frames, self.btns = [], []
        self.cur = None

    def add(self, frame, text=""):
        i = len(self.frames)
        b = tk.Button(self.side, text=text, anchor="w", command=lambda: self.select(i), **theme.button_opts())
        b.configure(bg=C["head"], highlightthickness=0, padx=12, pady=2)
        b.pack(fill="x")
        self.frames.append(frame)
        self.btns.append(b)
        if self.cur is None:
            self.select(0)

    def _index(self, x):
        if isinstance(x, int):
            return x
        for i, f in enumerate(self.frames):
            if f is x or str(f) == str(x):
                return i
        raise tk.TclError("no such page: %s" % x)

    def select(self, x=None):
        if x is None:
            return str(self.frames[self.cur]) if self.cur is not None else ""
        i = self._index(x)
        if self.cur is not None:
            self.frames[self.cur].pack_forget()
            self.btns[self.cur].configure(bg=C["head"], fg=C["btn_fg"])
        self.cur = i
        self.frames[i].pack(in_=self.body, fill="both", expand=True)
        self.btns[i].configure(bg=C["sel"], fg=C["fg"])

    def tab(self, x, option=None, **kw):
        b = self.btns[self._index(x)]
        if "text" in kw:
            b.configure(text=kw["text"])
        return b.cget(option) if option else None

    def tabs(self):
        return [str(f) for f in self.frames]


def fit_columns(tree, cols, pad=24):
    """Every ttk table column at least as wide as its heading (no cut-off headings in any language)."""
    import tkinter.font as tkf
    f = tkf.Font(root=tree, font=theme.font("button"))
    for c in cols:
        h = tree.heading(c, "text") or ""
        need = f.measure(h) + pad
        if int(tree.column(c, "width")) < need:
            tree.column(c, width=need, minwidth=need)


class App:
    def __init__(self, root, address=None, login=None, charts_dir=None, open_driver=False, layout=None, cams=None,
                 place=None):
        self.layout_path = layout or LAYOUT    # window places (gs_layout.json)
        self.auto_cams = cams           # camera preset to ask for once connected through the gateway (demo)
        self.root = root
        self.login = login              # (user, password) for "gw:" without asking (tests)
        self.place = place              # "shore" / "onboard" / None: in the window title
        self.link = None
        self.t = None                   # latest telemetry
        self.t_rx = 0.0
        self.hist = []                  # (t_s, heading, target, outboard)
        self.track = []                 # (lat, lon)
        self.origin = None
        self.params = {}
        self.pending = {}               # seq -> description
        self.prog = program.new_program()   # program in the editor (= the route on the map)
        self.route_dirty = False        # the route on the map was changed and not sent yet
        self.fence_draft = []           # the geofence being edited [(lat, lon)]
        self.fence_dirty = False
        self.new_wp = {"kn": 8.0, "radius": 25.0}   # speed / radius of a waypoint added on the map
        self.maxkn_edit = None          # the max leg speed being set (None = the boat's value)
        self.dp_kn = 1.0                # DP CRUISE / TRIP speed asked by this station (screen 2 "dp" panel)
        self.decisions = collections.deque(maxlen=8)    # copilot actions -> the captain's answer (shown)
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
        self.driver = None              # screen 1 (driving), section I
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
        self.title_extra = ""           # old --title text (now --place)
        self.boat = None                # the boat chosen in the boat list (boats/), None = typed address
        self.boat_pick = None           # the boat picked in screen 2's dropdown (before / while connecting)
        self.boat_menu = None           # the dropdown menu (built on click)
        self.boat_ok = False            # its name + live camera confirmed: HELM may be taken (CLAUDE.md F)
        self.boat_pushed = False        # the boat file's settings pushed to the AURIX (CLAUDE.md L): HELM + STANDBY
        self.boat_push_seq = None       # the seq of that PARAM_SET (a NACK re-arms the push)
        self.boat_push_at = 0.0         # no retry before this time
        self.relay_push_at = 0.0        # the relay table (PROTO_T_RELAY_CFG): no re-send before this time
        self.relay_push_seq = None
        self.thr_push_at = 0.0          # the DP thruster settings (PROTO_T_THRUSTER_CFG): no re-send before this time
        self.thr_push_seq = None
        self.boatsel = None             # the boat list dialog
        self.boatconf = None            # the boat confirmation dialog
        self.screen_shown = 2           # one monitor: which screen F11 shows (1 driving, 2 navigation)
        self.lamps = alerts.MasterLamps()   # MASTER WARNING / CAUTION (driver HUD): acknowledged = silenced here
        self.sound = None               # sound + voice (section D), made in _build_sound
        self.msg_text, self.msg_col = T("cap.not_connected"), None
        self.banner_tags = []           # the banners of the last drawing (tests)
        self.rects = {}
        self._press_ov = False
        self._render_t = 0.0
        self.boat_list = boats.load()
        self.prober = boats.Prober(self.boat_list).start()
        root.title(self.window_title())
        theme.set_app_icon(root)        # the app icon on every window of this station + the taskbar (13i)
        i18n.on_change(self.lang_changed)
        theme.on_change(self.theme_changed)
        root.configure(bg=C["bg"])
        if self.layout.get("captain"):
            root.geometry(self.layout["captain"])       # where it was last time (which monitor)
        else:
            root.geometry("1500x900")
            try:
                root.state("zoomed")    # maximised (Windows)
            except tk.TclError:
                pass
        theme.apply_ttk(root)
        # screen 2 = the map (the whole window) + the panels on it
        self.cv = tk.Canvas(root, highlightthickness=0, bg=C["bg"], cursor="crosshair")
        self.cv.pack(fill="both", expand=True)
        self.mapwin = mapwin.MapView(self, self.cv)
        self.ov = overlay.Overlay(self.cv, settings.get("s2_collapsed") or [])
        self.ov.on_collapse = self.collapse_changed
        self.port_var = tk.StringVar(value="tcp:127.0.0.1:5760")
        self._build_tools()
        self._build_menu()
        c = self.cv
        c.bind("<ButtonPress-1>", self.cv_press)
        c.bind("<B1-Motion>", self.cv_motion)
        c.bind("<ButtonRelease-1>", self.cv_release)
        c.bind("<ButtonPress-3>", lambda e: self.mapwin.right(e.x, e.y))
        c.bind("<MouseWheel>", lambda e: self.mapwin.zoom(0.8 if e.delta > 0 else 1.25))
        c.bind("<Button-4>", lambda e: self.mapwin.zoom(0.8))
        c.bind("<Button-5>", lambda e: self.mapwin.zoom(1.25))
        c.bind("<Configure>", lambda e: (self.mapwin.redraw(), self.render()))
        for k, fn in (("plus", lambda: self.mapwin.zoom(0.6)), ("minus", lambda: self.mapwin.zoom(1 / 0.6)),
                      ("KP_Add", lambda: self.mapwin.zoom(0.6)), ("KP_Subtract", lambda: self.mapwin.zoom(1 / 0.6)),
                      ("b", self.mapwin.back_to_boat), ("B", self.mapwin.back_to_boat)):
            root.bind("<%s>" % k if len(k) > 1 else k, lambda e, fn=fn: fn())

        if address:
            self.port_var.set(address)
            root.after(200, self.connect)
        root.after(50, self.poll)
        root.after(500, self.alarm_tick)
        root.after(1000, self.driver_beat)
        root.after(100, self.ov_tick)
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
        self.root.title(self.window_title())
        self.tools.title(i18n.W("tools.title"))
        for i, key in enumerate(self.tab_keys):
            self.nb.tab(i, text=T(key))
        self.ptree.heading("#0", text=T("cap.param"))
        for c in ("value", "min", "max"):
            self.ptree.heading(c, text=T("cap.param_" + c))
        for i, (c, key, _w) in enumerate(self.AIS_COLS):
            self.ais_tree.heading("#0" if i == 0 else c, text=T(key) if key else "")
        for c in ("lat", "lon", "kn", "radius", "action"):
            self.pg_tree.heading(c, text=T("cap.pg_" + c))
        self.fit_tables()
        self.prog_combos()
        self.sim_fault_cb.configure(values=[T(k) for k in self.SIM_FAULTS])
        self._build_menu()
        self.conn_btn.configure(text=T("cap.disconnect") if self.link else T("cap.connect"))
        if self.t:
            self.draw()
        self.draw_bench()
        self.draw_ais()
        self.draw_vision()
        self.draw_network()
        self.render(force=True)

    def set_lang(self, lang):
        i18n.set_lang(lang)

    def master_ack(self):
        """MASTER WARNING / CAUTION acknowledged on the driver screen: lamps off and the repeating sound
        silenced ON THIS PC. Nothing is sent to the boat (its ALARM ACK stays the captain's button)."""
        self.lamps.ack()
        if self.sound:
            self.sound.ack()

    def _build_menu(self):
        """Menu bar: Windows (screens, monitors), Tools (the pages), View (theme, map layers), Language."""
        mb = tk.Menu(self.root)
        wm = tk.Menu(mb, tearoff=0)
        wm.add_command(label=T("menu.driver"), command=lambda: self.close_driver() if self.driver else self.open_driver())
        wm.add_command(label=T("menu.toggle_screen"), command=self.toggle_screen)
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
        mb.add_cascade(label=T("menu.tools"), menu=self.tools_menu(mb))
        academymenu.add_menu(self, mb)          # Academy: its own window (simulator only) + the levels file
        vm = tk.Menu(mb, tearoff=0)
        vm.add_command(label=T("menu.theme_dark") if theme.name() == "light" else T("menu.theme_light"),
                       command=theme.toggle)
        vm.add_separator()
        for k in ("depth", "contours", "marks", "ais", "rings"):
            v = tk.BooleanVar(value=self.mapwin.rings if k == "rings" else self.mapwin.layers[k])
            vm.add_checkbutton(label=T("map.layer_" + k), variable=v, command=lambda k=k: self.mapwin.toggle_layer(k))
        mb.add_cascade(label=T("menu.view"), menu=vm)
        lm = tk.Menu(mb, tearoff=0)
        for code in i18n.LANGS:
            lm.add_command(label=T("menu.lang_" + code), command=lambda c=code: self.set_lang(c))
        mb.add_cascade(label=T("menu.language"), menu=lm)
        hm = tk.Menu(mb, tearoff=0)
        hm.add_command(label=T("menu.about"), command=self.open_about)
        mb.add_cascade(label=T("menu.help"), menu=hm)
        self.root.configure(menu=mb)
        self.menubar = mb

    def open_about(self):
        """Help -> About: logo, version, commit, licences / credits (13i, display only)."""
        self.about = about.open_about(self.root, i18n.lang(), getattr(self, "about", None))
        return self.about

    SIM_FAULTS = {"cap.sim_f_ok": 0, "cap.sim_f_noframes": 1, "cap.sim_f_corrupt": 2, "cap.sim_f_stuck": 4}

    SIM_FAULT_KEYS = list(SIM_FAULTS)

    def _build_joystick(self, parent):
        j = tk.Frame(parent, bg=C["bg"], highlightthickness=1, highlightbackground=C["line"])
        j.pack(fill="x", pady=(0, 4))
        lbl(j, "cap.joystick", font=F_MED).pack(side="left", anchor="n", padx=4)
        self.joy_cv = tk.Canvas(j, width=130, height=130, bg=C["entry"], highlightthickness=2,
                                highlightbackground=C["line"])
        self.joy_cv.pack(side="left", pady=2)
        self.lever_cv = tk.Canvas(j, width=54, height=130, bg=C["entry"], highlightthickness=2,
                                  highlightbackground=C["line"])
        self.lever_cv.pack(side="left", pady=2, padx=(4, 0))
        jr = tk.Frame(j, bg=C["bg"])
        jr.pack(side="left", padx=8, fill="y")
        self.joy_src = tk.Label(jr, text="-", font=F_TXT, bg=C["bg"], fg=C["fg"], justify="left")
        self.joy_src.pack(anchor="w")
        self.joy_status = tk.Label(jr, text="-", font=F_MED, bg=C["bg"], fg=C["fg"], justify="left")
        self.joy_status.pack(anchor="w")
        self.joy_btns = tk.Label(jr, text="-", font=F_MED, bg=C["bg"], fg=C["fg"])
        self.joy_btns.pack(anchor="w")
        jb = tk.Frame(j, bg=C["bg"])
        jb.pack(side="left", padx=4)
        lbl(jb, "cap.analog_joy", font=F_TXT).pack(anchor="w")
        btn(jb, "cap.cal_start", lambda: self.cmd(proto.T_JOY_CAL, proto.p_joy_cal(0), TE("what.calib_start"))).pack(
            fill="x")
        btn(jb, "cap.cal_finish", lambda: self.cmd(proto.T_JOY_CAL, proto.p_joy_cal(1),
                                                   TE("what.calib_finish"))).pack(fill="x")

    def _build_cameras(self, ct):
        """Camera presets, health / bitrate, front preview (FRONT-1, automatic FRONT-2)."""
        self.cam_ack = None
        self.vstreams = {}
        self.front_sel = video.FrontSelector()
        self.cam_photo = None
        top = tk.Frame(ct, bg=C["bg"])
        top.pack(fill="x", pady=4)
        for pr, key in (("Full", "cap.cam_full"), ("Front only", "cap.cam_front"), ("Mirrors only", "cap.cam_mirrors"),
                        ("All off", "cap.cam_off")):
            btn(top, key, lambda pr=pr: self.cam_preset(pr), font=F_TXT).pack(side="left", padx=2)
        self.cam_override = tk.BooleanVar(value=False)
        chk(top, "cap.cam_override", variable=self.cam_override, font=F_TXT, bg=C["bg"]).pack(side="left", padx=8)
        top = tk.Frame(ct, bg=C["bg"])
        top.pack(fill="x")
        btn(top, "cap.cam_inet_on", lambda: self.cam_internet(True), font=F_TXT).pack(side="left", padx=2)
        btn(top, "cap.cam_inet_off", lambda: self.cam_internet(False), font=F_TXT).pack(side="left", padx=2)
        rec = tk.Frame(ct, bg=C["bg"])
        rec.pack(fill="x", pady=2)
        i18n.bind(tk.Button(rec, font=F_BIG, bg=theme.WARN, fg=theme.WHITE, width=8,
                            command=lambda: self.video_save("captain")), "cap.save").pack(side="left", padx=4)
        lbl(rec, "cap.ring_min", font=F_TXT, bg=C["bg"]).pack(side="left", padx=(12, 2))
        self.ring_var = tk.StringVar(value="10")
        cb = ttk.Combobox(rec, textvariable=self.ring_var, values=["5", "10", "20", "30", "60"], width=4,
                          state="readonly", font=F_TXT)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda e: (self.ring_set(), self.root.focus_set()))
        self.ring_cb = cb
        self.rec_lbl = tk.Label(rec, text="", font=F_TXT, bg=C["bg"], justify="left", anchor="w", wraplength=700)
        self.rec_lbl.pack(side="left", padx=10)
        self.sj_save_was = False
        mid = tk.Frame(ct, bg=C["bg"])
        mid.pack(fill="both", expand=True)
        self.cam_cv = tk.Canvas(mid, width=640, height=360, bg=theme.BLACK, highlightthickness=0)
        self.cam_cv.pack(side="top", anchor="w", padx=4, pady=4)
        self.cam_lbl = lbl(mid, "cap.cam_connect_gw", font=F_MONO, bg=C["bg"], justify="left", anchor="nw")
        self.cam_lbl.pack(side="top", fill="x", padx=6)
        # obstacle detection (eWaSR on the gateway PC): stages + status
        vf = tk.Frame(ct, bg=C["bg"], bd=2, relief="groove")
        vf.pack(fill="x", padx=4, pady=4)
        lbl(vf, "cap.vis_title", font=F_MED, bg=C["bg"]).pack(anchor="w", padx=4)
        vf = tk.Frame(vf, bg=C["bg"])
        vf.pack(fill="x")
        self.vis_vars = {}
        for k in ("stage1", "stage2", "stage3"):
            var = tk.BooleanVar(value=k != "stage3")
            self.vis_vars[k] = var
            chk(vf, "cap.vis_" + k, variable=var, font=F_TXT, bg=C["bg"],
                command=lambda k=k: self.vision_stage(k)).pack(side="left", padx=4)
        btn(vf, "cap.vis_clear", lambda: self.vision_send({"t": "vision_clear"}), font=F_TXT).pack(side="left")
        self.vis_lbl = tk.Label(ct, text="", font=F_TXT, bg=C["bg"], anchor="w", justify="left")
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
            c.create_text(8, 8, text=use, anchor="nw", fill=theme.WHITE, font=F_MED)
            if self.front_sel.notice:
                c.create_text(320, 340, text=video.notice_text(self.front_sel), fill=theme.YELLOW, font=F_MED)
            if vs is None or vs.frozen():
                c.create_rectangle(120, 140, 520, 220, fill=theme.WARN, outline="")
                c.create_text(320, 180, text=T("hud.video_frozen"), fill=theme.WHITE, font=F_HUGE)
        g = (self.gw_status or {}).get("cameras")
        if g:
            lines = [T("cap.cam_relay", relay=T("cap.ok") if g["relay"] else T("cap.down"), kbps=g["total_kbps"]) +
                     (T("cap.cam_saver") if g["forced_low"] else "") + (T("cap.cam_inet_open") if g["internet"] else ""),
                     ""]
            mine = (self.cam_ack or {}).get("levels", {})
            for cam in g["cameras"]:
                lines.append(T("cap.cam_line", name="%-11s" % cam["name"], state="%-7s" % st_text(cam["state"]),
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
            bad = [c["name"] + " " + st_text(c["state"]) for c in r["cameras"] if c["state"] != "REC"]
            if bad:
                txt += T("cap.rec_not") + " " + ", ".join(bad)
            self.rec_lbl.configure(text=txt, fg=RED if (r["full"] or bad) else C["fg"])
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
        top = tk.Frame(rt, bg=C["bg"])
        top.pack(fill="x", pady=4)
        lbl(top, "cap.shore_joy", font=F_MED, bg=C["bg"]).pack(side="left", padx=4)
        btn(top, "cap.find_joy", self.remote_scan, font=F_TXT).pack(side="left")
        self.rsend_var = tk.BooleanVar(value=False)
        chk(top, "cap.send_joy", variable=self.rsend_var, font=F_MED, bg=C["bg"],
            command=self.remote_send_toggle).pack(side="left", padx=10)
        m = tk.Frame(rt, bg=C["bg"])
        m.pack(fill="x", padx=6)
        self.sj_vars = {}
        for r, key in enumerate(("steer", "thr")):
            lbl(m, "cap.sj_" + key, font=F_TXT, bg=C["bg"]).grid(row=r, column=0, sticky="w")
            dv = tk.IntVar(value=self.sj_cfg[key]["dev"])
            av = tk.StringVar(value=self.sj_cfg[key]["axis"])
            iv = tk.BooleanVar(value=self.sj_cfg[key].get("invert", False))
            lbl(m, "cap.sj_device", font=F_TXT, bg=C["bg"]).grid(row=r, column=1)
            tk.Spinbox(m, from_=0, to=15, textvariable=dv, width=3, font=F_MED).grid(row=r, column=2)
            ttk.Combobox(m, textvariable=av, values=shorejoy.AXES, width=4, state="readonly",
                         font=F_MED).grid(row=r, column=3, padx=4)
            chk(m, "cap.sj_invert", variable=iv, font=F_TXT, bg=C["bg"]).grid(row=r, column=4)
            self.sj_vars[key] = (dv, av, iv)
        self.sj_lever = tk.BooleanVar(value=self.sj_cfg["thr"].get("lever", True))
        chk(m, "cap.sj_lever", variable=self.sj_lever, font=F_TXT, bg=C["bg"]).grid(row=1, column=5, padx=8)
        btn(m, "cap.save_settings", self.remote_save_cfg, font=F_TXT).grid(row=0, column=5, sticky="w", padx=8)
        b = tk.Frame(rt, bg=C["bg"])
        b.pack(fill="x", pady=6)
        # REMOTE MANUAL / MANEUVER are asked for on screen 1 (the captain with the manual licence, CLAUDE.md D);
        # releasing is always possible here
        btn(b, "cap.remote_release", self.remote_release, kind="no", font=F_BIG).pack(side="left", padx=4)
        lbl(rt, "cap.remote_help", font=F_TXT, bg=C["bg"], wraplength=900, justify="left").pack(anchor="w", padx=6)
        self.remote_lbl = tk.Label(rt, text="-", font=F_MONO, bg=C["bg"], justify="left", anchor="w")
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
        self.remote_lbl.configure(text="\n".join(lines), fg=RED if d["remote_state"] == 2 else C["fg"])

    def _build_network(self, nt):
        """Internet channels (through the gateway): status, mode, Keep."""
        top = tk.Frame(nt, bg=C["bg"])
        top.pack(fill="x", pady=4)
        btn(top, "cap.net_auto", lambda: self.chan_req("AUTO"), font=F_MED).pack(side="left")
        lbl(top, "cap.net_channel", font=F_TXT, bg=C["bg"]).pack(side="left")
        self.chan_var = tk.StringVar(value="4G")
        self.chan_box = ttk.Combobox(top, textvariable=self.chan_var, values=["4G", "4G2", "RF", "STARLINK"],
                                     width=10, font=F_MED, state="readonly")
        self.chan_box.pack(side="left", padx=4)
        btn(top, "cap.net_manual", lambda: self.chan_req("MANUAL", self.chan_var.get()), font=F_MED).pack(side="left")
        btn(top, "cap.net_strict", lambda: self.chan_req("MANUAL_STRICT", self.chan_var.get()),
            font=F_TXT).pack(side="left", padx=4)
        self.keep_btn = tk.Button(top, text=T("cap.keep"), command=self.chan_keep, bg=C["btn_ok"], fg=C["btn_fg"], font=F_BIG, bd=0)
        self.keep_btn.pack(side="left", padx=12)
        self.net_lbl = lbl(nt, "cap.net_only_gw", font=F_MONO, bg=C["bg"], justify="left", anchor="w")
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
            return
        yes, no = T("cap.yes"), T("cap.no")
        lines = [T("cap.net_head", mode=st_text(n["mode"], "netmode."), manual=" " + n["manual"] if n.get("manual")
                   else "",
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
            lines.append(T("cap.sync_line", state=st_text(sy["state"]), reason=i18n.from_en(sy["reason"]),
                           kbps=sy["rate_kbps"],
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
        self.net_lbl.configure(text="\n".join(lines), fg=C["bad_txt"] if n.get("fallback") or n["emergency"]
                               else C["fg"])
        self.keep_btn.configure(text=T("cap.keep_s", s=n["trial"]["left_s"]) if n.get("trial") else T("cap.keep"),
                                bg=C["btn_warn"] if n.get("trial") else C["btn_ok"])

    def _build_joytab(self, jt):
        """Joystick tab: source, link status, button map, guided mapping."""
        top = tk.Frame(jt, bg=C["bg"])
        top.pack(fill="x", pady=4)
        lbl(top, "cap.jm_source", font=F_MED, bg=C["bg"]).grid(row=0, column=0, sticky="w", padx=4)
        self.jm_src = tk.IntVar(value=0)
        for i in range(len(proto.JOY_SOURCES)):
            i18n.bind(tk.Radiobutton(top, variable=self.jm_src, value=i, font=F_TXT, bg=C["bg"],
                                     command=self.jm_from_widgets), "cap.joy_src_%d" % i).grid(row=0, column=1 + i,
                                                                                            sticky="w")
        lbl(top, "cap.jm_throttle", font=F_MED, bg=C["bg"]).grid(row=1, column=0, sticky="w", padx=4)
        self.jm_thr = tk.IntVar(value=1)
        for i in range(len(proto.THR_INPUTS)):
            i18n.bind(tk.Radiobutton(top, variable=self.jm_thr, value=i, font=F_TXT, bg=C["bg"],
                                     command=self.jm_from_widgets), "cap.thr_in_%d" % i).grid(row=1, column=1 + i,
                                                                                           sticky="w")
        z = tk.Frame(top, bg=C["bg"])
        z.grid(row=2, column=0, columnspan=4, sticky="w", pady=2)
        self.jm_vars = {}
        for k in ("nz_lo", "nz_hi", "lever_delta", "pickup_win"):
            lbl(z, "cap.jm_" + k, font=F_TXT, bg=C["bg"]).pack(side="left", padx=(6, 2))
            v = tk.StringVar()
            tk.Entry(z, textvariable=v, font=F_MED, width=4).pack(side="left")
            self.jm_vars[k] = v
        bb = tk.Frame(jt, bg=C["bg"])
        bb.pack(fill="x", pady=2)
        btn(bb, "cap.jm_read", self.jm_read, font=F_TXT).pack(side="left")
        btn(bb, "cap.jm_default", self.jm_default, font=F_TXT).pack(side="left", padx=4)
        btn(bb, "cap.jm_wizard", lambda: self.wiz_start(None), kind="info", font=F_TXT).pack(side="left")
        btn(bb, "cap.jm_save", self.jm_save, kind="ok", font=F_TXT).pack(side="left", padx=4)

        self.jl_lbl = tk.Label(jt, text="-", font=F_MONO, bg=C["bg"], justify="left", anchor="w")
        self.jl_lbl.pack(fill="x", padx=6, pady=4)

        g = tk.Frame(jt, bg=C["bg"])
        g.pack(fill="x", padx=6)
        self.jm_rows = []
        names = ["cap.jm_axis_steer", "cap.jm_axis_thr"] + ["jm.%d" % i for i in range(len(proto.JM_BUTTONS))]
        half = 6                                    # two columns: fits a laptop screen
        for i, name in enumerate(names):
            r, c0 = (i % half) + 1, (i // half) * 4
            if i % half == 0:
                for c, h in enumerate(("cap.jm_col_func", "cap.jm_col_ctrl", "cap.jm_col_now")):
                    lbl(g, h, font=F_MED, bg=C["bg"]).grid(row=0, column=c0 + c, sticky="w", padx=4)
            lbl(g, name, font=F_TXT, bg=C["bg"]).grid(row=r, column=c0, sticky="w", padx=4)
            code = tk.Label(g, text="-", font=F_TXT, bg=C["bg"], width=24, anchor="w")
            code.grid(row=r, column=c0 + 1, sticky="w", padx=4)
            now = tk.Label(g, text="", font=F_TXT, bg=C["bg"], width=9)
            now.grid(row=r, column=c0 + 2)
            step = ("axis_steer", "axis_thr")[i] if i < 2 else i - 2
            i18n.bind(tk.Button(g, command=lambda st=step: self.wiz_start([st]), font=F_SMALL, bg=C["btn"], fg=C["btn_fg"], relief="flat", pady=0),
                      "cap.jm_map").grid(row=r, column=c0 + 3, pady=1, padx=(0, 16))
            self.jm_rows.append((code, now))

        self.wiz_frame = tk.Frame(jt, bg=C["head"], bd=2, relief="groove")
        self.wiz_lbl = tk.Label(self.wiz_frame, text="", font=F_BIG, bg=C["head"], justify="left", wraplength=900)
        self.wiz_lbl.pack(anchor="w", padx=6, pady=4)
        self.wiz_info = tk.Label(self.wiz_frame, text="", font=F_MED, bg=C["head"], justify="left")
        self.wiz_info.pack(anchor="w", padx=6)
        wb = tk.Frame(self.wiz_frame, bg=C["head"])
        wb.pack(anchor="w", padx=6, pady=4)
        self.wiz_next = tk.Button(wb, text=T("cap.wiz_next"), command=self.wiz_next_click, bg=C["btn_ok"], fg=C["btn_fg"], font=F_MED,
                                  bd=3)
        self.wiz_next.pack(side="left")
        self.wiz_skip = btn(wb, "cap.wiz_skip", self.wiz_skip_click)
        self.wiz_skip.pack(side="left", padx=4)
        btn(wb, "cap.cancel", self.wiz_cancel, kind="no").pack(side="left")
        self.jm_to_widgets()

    # --- sound + voice (section D) -------------------------------------------------------------
    def _build_sound(self, st):
        """The sound settings and the sound test page (play every sound / callout in both languages)."""
        self.callouts = None
        try:
            import sound
            import soundtest
        except ImportError:
            return lbl(st, "cap.sound_missing", font=F_MED, bg=C["bg"]).pack(anchor="w", padx=6, pady=6)
        self.sound = sound.SoundSystem.for_app(self)
        self.callouts = sound.Callouts(self.sound, self)
        soundtest.build(st, self)

    # --- engine controls + remote restart (CLAUDE.md N): the station's engine page ------------------------
    def _build_engine(self, fr):
        """IGNITION -> START (hold) -> SHUTOFF, and RESTART (clears the ENGINE KILL, or the controlled reset
        after a monitor trip). Requests only: the AURIX checks HELM, idle + neutral, the e-stop and the time
        limits every time (START <= 5 s per hold, then 10 s cool-down)."""
        self.crank_job = None
        lbl(fr, "eng.title", font=F_MED).pack(anchor="w", padx=6, pady=(6, 2))
        lbl(fr, "eng.note", font=F_TXT, wraplength=820, justify="left").pack(anchor="w", padx=6)
        self.eng_state = tk.Label(fr, text="-", font=F_MED, bg=C["bg"], fg=C["fg"], anchor="nw", justify="left", height=6)
        self.eng_state.pack(fill="x", padx=6, pady=6)
        row = tk.Frame(fr, bg=C["bg"])
        row.pack(anchor="w", padx=6, pady=4)
        self.eng_relays = tk.Frame(row, bg=C["bg"])          # IGNITION / SHUTOFF of the boat file (engine group)
        self.eng_relays.pack(side="left")
        self.eng_start = btn(row, "eng.start", None, kind="warn", font=F_MED)
        self.eng_start.pack(side="left", padx=12)
        self.eng_start.bind("<ButtonPress-1>", lambda _e: self.crank_press())
        self.eng_start.bind("<ButtonRelease-1>", lambda _e: self.crank_release())
        btn(row, "eng.restart", self.engine_restart, kind="info", font=F_MED).pack(side="left", padx=12)
        self.eng_relay_n = None

    def engine_relays(self):
        """The boat file's IGNITION / SHUTOFF relays (engine group, by their role): name, n. The STARTER (role
        "starter") is the START button, never a plain relay switch; the AURIX switches an engine relay ON only
        with idle + neutral confirmed and the e-stop released."""
        b = self.boat
        if b is None or getattr(b, "error", None):
            return []
        out = []
        for r in getattr(b, "relays", []) or []:
            if r.get("group") == "engine" and r.get("role") in ("ignition", "shutoff"):
                name = r.get("name_el") if i18n.lang() == "el" else r.get("name_en")
                out.append((name or "%d" % r["n"], r["n"]))
        return out

    def draw_engine(self, d):
        if not hasattr(self, "eng_state"):
            return
        rel = self.engine_relays()
        if rel != self.eng_relay_n:                             # (re)build the IGNITION / SHUTOFF buttons
            self.eng_relay_n = rel
            for w in self.eng_relays.winfo_children():
                w.destroy()
            for name, n in rel:
                f = tk.Frame(self.eng_relays, bg=C["bg"])
                f.pack(side="left", padx=4)
                tk.Label(f, text=name, font=F_TXT, bg=C["bg"], fg=C["fg"]).pack()
                tk.Button(f, text=T("eng.on"), command=lambda n=n, nm=name: self.engine_relay(n, True, nm),
                          **theme.button_opts("normal")).pack(side="left")
                tk.Button(f, text=T("eng.off"), command=lambda n=n, nm=name: self.engine_relay(n, False, nm),
                          **theme.button_opts("normal")).pack(side="left")
        if not d or "eng_flags" not in d:
            self.eng_show(T("eng.no_data"), C["dim"])
            return
        lines = []
        if d["eng_killed"]:
            lines.append(T("eng.killed"))
        if d["eng_tripped"]:
            lines.append(T("eng.tripped"))
        if d["eng_reset_pending"]:
            lines.append(T("eng.reset_pending"))
        if d["eng_crank"] == 1:
            lines.append(T("eng.cranking", s="%.1f" % (d["eng_hold_ms"] / 1000.0)))
        elif d["eng_crank"] == 2:
            lines.append(T("eng.cooling", s="%.0f" % d["eng_cool_s"]))
        lines.append(T("eng.ready") if d["eng_ready"] else T("eng.not_ready"))
        if not d["eng_starter"]:
            lines.append(T("eng.no_starter"))
        if d["eng_why"]:
            lines.append(T("eng.last", why=T("eng.why_%d" % d["eng_why"]) if d["eng_why"] < len(proto.ENG_WHY)
                           else str(d["eng_why"])))
        bad = d["eng_killed"] or d["eng_tripped"]
        self.eng_show("\n".join(lines), RED if bad else (GREEN if d["eng_ready"] else ORANGE))

    def eng_show(self, text, fg):
        """The engine page's state text - only when it changes."""
        if (text, fg) != getattr(self, "eng_shown", None):
            self.eng_shown = (text, fg)
            self.eng_state.configure(text=text, fg=fg)

    def engine_relay(self, n, on, name):
        self.cmd(proto.T_RELAY, proto.p_relay(n, 1 if on else 0), TE("what.relay_onoff", name=name,
                                                                   state="ON" if on else "OFF"))

    def engine_restart(self):
        self.cmd(proto.T_RESTART, proto.p_restart(proto.ENG_RESTART), TE("what.engine_restart"))

    def crank_press(self):
        """START held: one CRANK now (through cmd: logged), then every CRANK_PERIOD_MS while held."""
        if not self.link:
            return self.say(T("cap.not_connected"), RED)
        if crewui.crew_of(self)[0] == "copilot":
            return self.say(T("eng.captain_only"), RED)          # (the gateway refuses it too: code 118)
        self.cmd(proto.T_RESTART, proto.p_restart(proto.ENG_CRANK), TE("what.engine_start"))
        self.crank_job = self.root.after(CRANK_PERIOD_MS, self.crank_tick)

    def crank_tick(self):
        self.crank_job = None
        if not self.link:
            return
        if isinstance(self.link, GatewayLink):
            self.link.send(proto.T_RESTART, proto.p_restart(proto.ENG_CRANK), what=TE("what.engine_start"))
        else:
            self.link.send(proto.T_RESTART, proto.p_restart(proto.ENG_CRANK))
        self.crank_job = self.root.after(CRANK_PERIOD_MS, self.crank_tick)

    def crank_release(self):
        if self.crank_job is not None:
            self.root.after_cancel(self.crank_job)
            self.crank_job = None
        if self.link:
            self._send_cmd(proto.T_RESTART, proto.p_restart(proto.ENG_RELEASE), TE("what.engine_release"))

    # --- BENCH TEST (dock, USB laptop only): FIRST_TEST.md part 3 -----------------------------
    def _build_bench(self, bt):
        top = tk.Frame(bt, bg=C["bg"])
        top.pack(fill="x", pady=4)
        btn(top, "cap.bench_enter", lambda: self.bench(proto.BENCH_ENTER, 0, TE("what.bench_enter")),
            kind="info", font=F_TXT).pack(side="left", padx=4)
        btn(top, "cap.bench_leave", lambda: self.bench(proto.BENCH_LEAVE, 0, TE("what.bench_leave")),
            font=F_TXT).pack(side="left")
        lbl(top, "cap.bench_note", font=F_TXT, bg=C["bg"], wraplength=640, justify="left").pack(side="left", padx=10)
        self.bench_state = tk.Label(bt, text=T("cap.bench_off"), font=F_MED, bg=C["bg"], anchor="w")
        self.bench_state.pack(fill="x", padx=6)
        body = tk.Frame(bt, bg=C["bg"])
        body.pack(fill="both", expand=True)
        left = tk.Frame(body, bg=C["bg"])
        left.pack(side="left", fill="both", expand=True, padx=6)
        lbl(left, "cap.bench_sensors", font=F_MED, bg=C["bg"]).pack(anchor="w")
        self.bench_sens = tk.Label(left, text="-", font=F_MONO, bg=C["bg"], justify="left", anchor="nw")
        self.bench_sens.pack(anchor="w", fill="x")
        lbl(left, "can.title", font=F_MED, bg=C["bg"]).pack(anchor="w", pady=(8, 0))      # DroneCAN node table (11d)
        self.bench_can = tk.Label(left, text=T("can.none"), font=F_MONO, bg=C["bg"], justify="left", anchor="nw")
        self.bench_can.pack(anchor="w", fill="x")
        self.can_d = None
        right = tk.Frame(body, bg=C["bg"])
        right.pack(side="left", fill="both", expand=True, padx=6)

        lbl(right, "cap.bench_pulses", font=F_MED, bg=C["bg"]).pack(anchor="w")
        pr = tk.Frame(right, bg=C["bg"])
        pr.pack(anchor="w")
        self.bench_ms = tk.IntVar(value=100)
        for ms in (100, 200, 300):
            tk.Radiobutton(pr, text="%d ms" % ms, variable=self.bench_ms, value=ms, font=F_TXT, bg=C["bg"]).pack(side="left")
        btn(pr, "cap.bench_left", lambda: self.bench(proto.BENCH_LEFT, self.bench_ms.get(),
                                                     TE("what.pulse_left", ms=self.bench_ms.get())),
            font=F_TXT).pack(side="left", padx=6)
        btn(pr, "cap.bench_right", lambda: self.bench(proto.BENCH_RIGHT, self.bench_ms.get(),
                                                      TE("what.pulse_right", ms=self.bench_ms.get())),
            font=F_TXT).pack(side="left")
        self.bench_pulse = tk.Label(right, text="-", font=F_TXT, bg=C["bg"], anchor="w", justify="left")
        self.bench_pulse.pack(anchor="w")

        lbl(right, "cap.bench_limits", font=F_MED, bg=C["bg"]).pack(anchor="w", pady=(8, 0))
        lr = tk.Frame(right, bg=C["bg"])
        lr.pack(anchor="w")
        self.bench_lim = []
        for key in ("cap.bench_lim_l", "cap.bench_lim_r", "cap.bench_estop"):
            lb = lbl(lr, key, font=F_MED, bg=C["btn"], width=9, relief="ridge")
            lb.pack(side="left", padx=3)
            self.bench_lim.append(lb)

        lbl(right, "cap.bench_buttons", font=F_MED, bg=C["bg"]).pack(anchor="w", pady=(8, 0))
        br = tk.Frame(right, bg=C["bg"])
        br.pack(anchor="w")
        self.bench_btn = []
        for b in (5, 1, 2, 3, 4, 0):                    # HDG LOCK, -1, +1, -10, +10, STANDBY
            lb = tk.Label(br, text=T("btn.%d" % b), font=F_TXT, bg=C["btn"], width=10, relief="ridge")
            lb.pack(side="left", padx=2)
            self.bench_btn.append((b, lb))

        lbl(right, "cap.bench_buzzer", font=F_MED, bg=C["bg"]).pack(anchor="w", pady=(8, 0))
        btn(right, "cap.bench_buzz_test", lambda: self.bench(proto.BENCH_BUZZER, 1000, TE("what.buzzer")),
            font=F_TXT).pack(anchor="w")

        lbl(right, "cap.bench_servo_title", font=F_MED, bg=C["bg"]).pack(anchor="w", pady=(8, 0))
        self.bench_servo = tk.Label(right, text="-", font=F_MONO, bg=C["bg"], justify="left", anchor="w")
        self.bench_servo.pack(anchor="w")
        btn(right, "cap.bench_trip", self.bench_trip, kind="no", font=F_TXT).pack(anchor="w", pady=4)
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

    def on_can(self, d):
        """T_CAN_NODES (USB link, 1 Hz while a CAN bus is fitted): the DroneCAN bus + node table (bench page)."""
        self.can_d = d
        self.can_d_rx = time.time()
        self.draw_can()

    def draw_can(self):
        if not hasattr(self, "bench_can"):
            return
        d = getattr(self, "can_d", None)
        if d is None or time.time() - getattr(self, "can_d_rx", 0) > 3.0:
            self.bench_can.configure(text=T("can.none"), fg=C["dim"])
            return
        lines = [T("can.bus", state=T(CAN_STATE_KEYS[d["state"]]) if d["state"] < len(CAN_STATE_KEYS) else "?",
                   tec=d["tec"], rec=d["rec"], rx=d["rx"], tx=d["tx"], drops=d["tx_drops"], err=d["rx_errors"],
                   boffs=d["busoffs"])]
        if d["conflict"]:
            lines.append(T("can.conflict"))
        for n in d["nodes"]:
            st = T("can.ok") if not n["lost"] else T(CAN_WHY_KEYS[n["why"]]) if n["why"] < len(CAN_WHY_KEYS) else "?"
            lines.append(T("can.node", role=T(CAN_ROLE_KEYS[n["role"]]) if n["role"] < len(CAN_ROLE_KEYS) else "?",
                           id=n["id"], state=st + (" " + T("can.warn_short") if n["warning"] else ""),
                           health=n["health_name"], mode=n["mode_name"], up=n["uptime_s"],
                           age="-" if n["age_ms"] is None else "%.1f" % (n["age_ms"] / 1000.0), reb=n["restarts"]))
        bad = d["fault"] or d["conflict"] or any(n["lost"] and n["required"] for n in d["nodes"])
        self.bench_can.configure(text="\n".join(lines), fg=RED if bad else C["fg"])

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
            self.bench_state.configure(text=T("cap.bench_off"), fg=C["dim"])
            return
        self.bench_state.configure(text=T("cap.bench_on", s="%d" % d["idle_left_s"]) +
                                   (T("cap.bench_tripped") if d["mon_tripped"] else ""),
                                   fg=RED if d["mon_tripped"] else MODE_COLORS["BENCH_TEST"])

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
            lb.configure(bg=(ORANGE if on else C["btn_ok"]) if fitted else C["btn"],
                         fg=C["fg"] if fitted else C["faint"])
        for b, lb in self.bench_btn:
            n = d["button_counts"][b]
            lb.configure(bg=C["lit"] if n else C["btn"],
                         text="%s%s" % (T("btn.%d" % b), (" x%d" % n) if n else ""))
        self.bench_servo.configure(text=T("bench.servo", thr="%4d" % d["thr_us"], idle=d["thr_idle_us"],
                                          gear="%4d" % d["gear_us"], n=d["gear_n_us"]))

    # --- AIS ship list + collision avoidance (sections A / D / E2) ------------------
    AIS_COLS = (("name", "ais.col_ship", 200), ("cls", "ais.col_class", 50), ("dist", "ais.col_dist", 95),
                ("brg", "ais.col_brg", 70), ("cpa", "ais.col_cpa", 70), ("tcpa", "ais.col_tcpa", 80),
                ("age", "ais.col_age", 60), ("state", "", 110))

    def _build_ais(self, at):
        self.ais_state_lbl = tk.Label(at, text=T("ais.state", state="-"), font=F_MED, bg=C["bg"], anchor="w",
                                      justify="left")
        self.ais_state_lbl.pack(fill="x", padx=6, pady=(4, 0))
        lbl(at, "ais.small_craft", font=F_TXT, bg=C["bg"], fg=C["bad_txt"]).pack(anchor="w", padx=6)
        self.ais_tree = ttk.Treeview(at, columns=[c for c, _t, _w in self.AIS_COLS[1:]], height=12)
        for i, (c, t, w) in enumerate(self.AIS_COLS):
            col = "#0" if i == 0 else c
            self.ais_tree.column(col, width=w, anchor="w" if i == 0 else "e")
            self.ais_tree.heading(col, text=T(t) if t else "")
        self.ais_tree.tag_configure("danger", background=C["row_danger"])
        self.ais_tree.tag_configure("warn", background=C["row_warn"])
        self.ais_tree.tag_configure("stale", foreground=C["row_stale"])
        self.ais_tree.pack(fill="both", expand=True, padx=6, pady=4)
        self.ais_tree.bind("<<TreeviewSelect>>", lambda e: self.ais_details())
        self.ais_detail = lbl(at, "ais.click_ship", font=F_TXT, bg=C["bg"], anchor="w", justify="left")
        self.ais_detail.pack(fill="x", padx=6, pady=(0, 6))
        bb = tk.Frame(at, bg=C["bg"])
        bb.pack(anchor="w", padx=6, pady=(0, 6))
        lbl(bb, "ais.proposal_so", font=F_TXT, bg=C["bg"]).pack(side="left")
        btn(bb, "drv.accept", lambda: self.avoid_answer(None, True), font=F_TXT, kind="ok").pack(side="left",
                                                                                                    padx=4)
        btn(bb, "drv.reject", lambda: self.avoid_answer(None, False), font=F_TXT, kind="no").pack(side="left")
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
        top = tk.Frame(pg, bg=C["bg"])
        top.pack(fill="x")
        for key, fn in (("cap.pg_upload", self.prog_upload), ("cap.pg_download", self.prog_download),
                        ("cap.pg_save", self.prog_save), ("cap.pg_load", self.prog_load)):
            btn(top, key, fn, font=F_TXT).pack(side="left", padx=1)
        lbl(top, "cap.pg_at_end", font=F_TXT, bg=C["bg"]).pack(side="left")
        self.final_cb = ttk.Combobox(top, width=18, state="readonly", font=F_TXT)
        self.final_cb.pack(side="left")
        self.pg_tree = ttk.Treeview(pg, columns=("lat", "lon", "kn", "radius", "action"), height=10)
        for c, w in (("#0", 50), ("lat", 140), ("lon", 140), ("kn", 70), ("radius", 80), ("action", 100)):
            self.pg_tree.column(c, width=w)
            self.pg_tree.heading(c, text="#" if c == "#0" else T("cap.pg_" + c))
        self.pg_tree.pack(fill="both", expand=True)
        self.pg_tree.bind("<<TreeviewSelect>>", self.prog_select)
        ed = tk.Frame(pg, bg=C["bg"])
        ed.pack(fill="x")
        self.pg_vars = {}
        for k, w in (("lat", 11), ("lon", 11), ("kn", 5), ("radius", 5)):
            lbl(ed, "cap.pg_" + k, font=F_TXT, bg=C["bg"]).pack(side="left")
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
        bt = tk.Frame(pg, bg=C["bg"])
        bt.pack(fill="x")
        for key, fn in (("cap.pg_add", self.prog_add), ("cap.pg_change", self.prog_change),
                        ("cap.pg_delete", self.prog_delete), ("cap.pg_up", lambda: self.prog_move(-1)),
                        ("cap.pg_down", lambda: self.prog_move(1)), ("cap.pg_clear", self.prog_clear)):
            btn(bt, key, fn, font=F_TXT).pack(side="left", padx=1)
        btn(bt, "cap.pg_depth", self.prog_depth_check, font=F_TXT, kind="info").pack(side="left", padx=8)
        lbl(pg, "cap.pg_profile_note", font=F_TXT, bg=C["bg"]).pack(anchor="w")
        self.pg_prof = tk.Canvas(pg, height=150, bg=C["prof_bg"], highlightthickness=0)
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
        c.create_line(40, Y(lim), w - 10, Y(lim), fill=theme.WARN, dash=(5, 3))
        c.create_text(38, Y(lim), text="%.1f m" % lim, anchor="e", fill=theme.WARN, font=F_TXT)
        for (d1, _a, _b, z1), (d2, _c, _d, z2) in zip(prof, prof[1:]):
            bad = z1 is None or z2 is None or z1 < lim or z2 < lim
            z1 = 0.0 if z1 is None else max(0.0, z1)
            z2 = 0.0 if z2 is None else max(0.0, z2)
            c.create_polygon(X(d1), 10, X(d2), 10, X(d2), Y(z2), X(d1), Y(z1),
                             fill=C["prof_bad"] if bad else C["prof_ok"], outline="")
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
        self.conn_btn.configure(text=T("cap.connect"), **theme.button_opts("ok"))
        self.say(why or T("cap.disconnected"), RED if why else None)

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
        if bt.error:
            return T("boat.invalid", name=bt.name, err=bt.error, file=bt.path)     # a bad file is never connected
        try:
            self.link = GatewayLink("%s:%d" % (bt.ip, bt.port), user, password=pw, station="ground station (%s)" % user,
                                    log=True, crew=crew, boat=bt.name)
            w = self.link.start()
        except Exception as e:
            self.link = None
            return T("cap.cannot_connect", err=i18n.from_en(str(e)))
        self.boat, self.boat_ok = bt, False
        self.boat_pushed, self.boat_push_seq, self.boat_push_at = False, None, 0.0
        self.relay_push_at, self.relay_push_seq = 0.0, None
        self.thr_push_at, self.thr_push_seq = 0.0, None
        self.boat_pick = bt
        self.prober.found(bt)                       # the gateway answered: found (screen 2's lamp)
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
        self.conn_btn.configure(text=T("cap.disconnect"), **theme.button_opts("no"))
        if self.port_var.get().startswith("tcp:"):
            self.sim_frame.pack(fill="x", pady=(8, 0), padx=6)
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

    def close_driver(self):
        if self.driver:
            d, self.driver = self.driver, None
            d.destroy()
            self.layout_changed()

    def open_map(self):
        """Screen 2 is the map: bring it to the front, centred on the boat."""
        self.mapwin.back_to_boat()
        self.show_screen(2)

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
                           "driver": self.driver.w.geometry() if self.driver else self.layout.get("driver"),
                           "tools": self.tools.geometry()}
            with open(self.layout_path, "w", encoding="utf-8") as f:
                json.dump(self.layout, f)
        except (OSError, tk.TclError):
            pass

    # --- driver confirmation through the gateway / single operator (section F1) ------
    def driver_beat(self):
        """Twice a second: tell the gateway whether this station shows screen 1 (the gateway drops a screen
        whose beat is late - 2.5 s, test F1 1.5 s - so a busy PC must not get near that)."""
        if isinstance(self.link, GatewayLink) and self.link.connected:
            self.link.send_msg({"t": "driver", "open": self.driver is not None})
        self.root.after(500, self.driver_beat)

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
        w = tk.Toplevel(self.root, bg=C["head"])
        w.title(i18n.W("cap.cc_title"))
        w.transient(self.root)
        self.cconf = w
        tk.Label(w, text=T("cap.cc_q"), font=F_BIG, bg=C["head"]).pack(padx=20, pady=10)
        tk.Label(w, text=i18n.from_en(what), font=F_BIG, bg=C["head"], fg=theme.BANNERS["single_op"][1]).pack(padx=20, pady=6)
        left = tk.Label(w, text="", font=F_MED, bg=C["head"])
        left.pack()
        b = tk.Frame(w, bg=C["head"])
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

        btn(b, "cap.cc_send", lambda: done(True), kind="ok").pack(side="left", padx=8)
        btn(b, "cap.cancel", lambda: done(False)).pack(side="left", padx=8)
        theme.style_tree(w)

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
        copilot = False
        if isinstance(self.link, GatewayLink) and proto.is_driving_cmd(ftype, payload, self.relay_groups_known()):
            if self.single_op:
                return self.captain_confirm(what, lambda: self._send_cmd(ftype, payload, what, confirm="self"))
            if crewui.crew_of(self)[0] == "copilot" and ftype not in (proto.T_AUTO_PAUSE, proto.T_AUTO_RETURN):
                # the gateway asks the captain's screen 1 (CLAUDE.md D)
                copilot = True
                self.say(T("cap.wait_captain", what=i18n.from_en(what)), ORANGE)
                self.event(T("cap.ev_captain_asked", what=i18n.from_en(what)))
        seq = self._send_cmd(ftype, payload, what)
        if copilot and seq is not None:
            self.track_decision(seq, what)

    def _send_cmd(self, ftype, payload, what, confirm=None):
        if not self.link:
            return self.say(T("cap.not_connected"), RED)
        if isinstance(self.link, GatewayLink):
            seq = self.link.send(ftype, payload, what=what, confirm=confirm)
        else:
            seq = self.link.send(ftype, payload)
        self.pending[seq] = what
        self.event("> " + i18n.from_en(what))
        return seq

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
        d = tk.Toplevel(self.root, bg=C["bg"])
        d.title(i18n.W("cap.pw_title"))
        d.transient(self.root)
        self.pdlg = d
        self.pdlg_dir = direction
        tk.Label(d, text=T("cap.pw_head_%s" % ("fwd", "rev")[direction]), font=F_BIG,
                 bg=C["bg"]).pack(padx=20, pady=(14, 4))
        lbl(d, "cap.pw_step1", font=F_MED, bg=C["bg"]).pack(anchor="w", padx=20)
        self.pdlg_code = tk.Label(d, text="....", font=theme.font("code", mono=True), bg=C["entry"], fg=C["entry_fg"])
        self.pdlg_code.pack(padx=20, pady=6)
        self.pdlg_var = tk.StringVar()
        e = tk.Entry(d, textvariable=self.pdlg_var, font=theme.font("huge", mono=True), width=6, justify="center")
        e.pack(pady=4)
        e.focus_set()
        lbl(d, "cap.pw_step2", font=F_MED, bg=C["bg"]).pack(anchor="w", padx=20, pady=(8, 4))
        b = tk.Frame(d, bg=C["bg"])
        b.pack(pady=12)
        btn(b, "cap.pw_send", self.power_send, kind="warn").pack(side="left", padx=6)
        btn(b, "cap.cancel", self.power_close).pack(side="left", padx=6)
        e.bind("<Return>", lambda _ev: self.power_send())
        theme.style_tree(d)

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

    def info_hdg(self):
        self.say(T("cap.info_hdg"))

    def auto_start(self):
        self.cmd(proto.T_AUTO_START, b"", TE("what.auto_start"))

    def auto_return(self):
        """AUTO RETURN (CLAUDE.md R): the program backwards from the current leg to waypoint 1; the boat waits for
        the joystick CONFIRM. A copilot's goes straight (no captain's ACCEPT), like AUTO pause."""
        self.cmd(proto.T_AUTO_RETURN, b"", TE("what.auto_return"))

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
        """A click on the map outside the editing modes: nothing (the panels' buttons start the editing)."""
        return None

    # --- feedback ----------------------------------------------------------------
    def say(self, text, color=None):
        """The message line of screen 2 (+ the tools window). color: None, GREEN, ORANGE, RED, BLUE."""
        self._evlog = (getattr(self, "_evlog", []) + [text])[-200:]
        self.msg_text, self.msg_col = text, color
        try:
            self.tools_msg.configure(text=text, bg=color or C["head"],
                                     fg=theme.BLACK if color in (GREEN, ORANGE, BLUE) else
                                     (theme.WHITE if color == RED else C["fg"]))
        except (AttributeError, tk.TclError):
            pass
        if hasattr(self, "ov"):
            self.render(force=True)

    def ev_texts(self):
        """Recent event / message texts (tests)."""
        return list(getattr(self, "_evlog", []))

    def event(self, text):
        self._evlog = (getattr(self, "_evlog", []) + [text])[-200:]
        self.events.insert(0, time.strftime("%H:%M:%S  ") + text)
        if self.events.size() > 500:
            self.events.delete(500, "end")

    URGENT = ("driver_ask", "driver_ask_done", "crew_swap_ask")      # questions for this station: never behind a backlog

    def poll(self):
        if self.link:
            batch = []
            while len(batch) < 500:
                try:
                    batch.append(self.link.events.get_nowait())
                except queue.Empty:
                    break
            # questions first (a busy PC must still show the captain's question within its time); of the
            # telemetry only every frame's track / history, the screens are drawn from the newest one
            batch = [e for e in batch if e[0] in self.URGENT] + [e for e in batch if e[0] not in self.URGENT]
            for kind, d in batch:
                if kind == "telemetry":
                    self.on_telem(d)
                elif kind == "ack":
                    what = self.pending.pop(d["seq"], None)
                    self.on_decision_ack(d)
                    if d["cmd"] == proto.T_PROG_COMMIT and d["ok"]:
                        self.route_dirty = False
                    if d["cmd"] == proto.T_PARAM_GET and d["result"] == 10:
                        continue                # "Read all" probes past the last id
                    if d["cmd"] == proto.T_PARAM_SET and d["ok"]:
                        self.root.after(300, self.read_params)
                    if d["seq"] == self.boat_push_seq and not d["ok"]:
                        self.boat_pushed, self.boat_push_at = False, time.time() + 5.0   # e.g. left STANDBY: again later
                    if d["seq"] == self.relay_push_seq and not d["ok"]:
                        self.relay_push_at = time.time() + 5.0      # the relay table: again later (STANDBY, CONFIRM ready)
                    if d["seq"] == self.thr_push_seq and not d["ok"]:
                        self.thr_push_at = time.time() + 5.0        # the thruster settings: again later (STANDBY)
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
                        self.route_dirty = False
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
                elif kind == "can":
                    self.on_can(d)
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
                elif kind == "geofence_ack":
                    self.on_fence_ack(d)
                elif kind == "academy_levels_ack":
                    academymenu.on_ack(self, d)
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
                    self.say(i18n.from_en(d), RED if "DISCONNECTED" in d else None)
                    self.event(i18n.from_en(d))
            age = time.time() - self.t_rx if self.t_rx else 99
            ok = self.link.connected and age < 1.0
            self.link_lbl.configure(text=T("cap.link_ok") if ok else T("cap.link_nodata"),
                                    fg=C["ok_txt"] if ok else C["bad_txt"])
        self.root.after(50, self.poll)

    def on_param(self, d):
        self.params[d["id"]] = d
        iid = str(d["id"])
        vals = ("%g" % d["value"], "%g" % d["min"], "%g" % d["max"])
        if self.ptree.exists(iid):
            self.ptree.item(iid, values=vals)
        else:
            self.ptree.insert("", "end", iid=iid, text=d["name"], values=vals)

    def draw_steer(self, d):
        """Control page (step 11e): the steering position servo's model - DISPLAY ONLY."""
        w = getattr(self, "steer_lbl", None)
        if w is None:
            return
        if not d.get("sv_servo"):
            txt = ""
        else:
            pos = T("steer.unknown") if d.get("sv_pos") is None else "%+.2f" % d["sv_pos"]
            if d.get("sv_trained"):
                vals = T("steer.trained", rr="%.3f" % d["sv_r_right"], rl="%.3f" % d["sv_r_left"],
                         b="%.3f" % d["sv_play_stored"], k="%.2f" % d["sv_k"], tau="%.1f" % d["sv_tau"],
                         conf="%d" % d["sv_conf"])
            else:
                vals = T("steer.untrained")
            adapt = T("steer.adapted", k="%d" % d["sv_k_pct"], b="%.3f" % d["sv_play_use"]) if d.get("sv_adapted") \
                else T("steer.stored")
            txt = T("steer.info", pos=pos, cmd="%+.2f" % d["sv_cmd"], vals=vals, adapt=adapt,
                    est="%d" % d.get("thr_est_pct", 0), travel="%.1f" % d.get("thr_travel_s", 0.0))
            if d.get("sv_reset_why"):
                txt += "\n" + T("steer.last_reset", why=T("steer.why_%d" % d["sv_reset_why"]) if d["sv_reset_why"] <= 5
                                 else str(d["sv_reset_why"]))
        if w.cget("text") != txt:
            w.configure(text=txt)

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
            self.boat_pushed = False        # its RAM-only boat file settings are gone: push them again
            self.relay_push_at = 0.0        # (the relay table is in its data flash: sent only if it differs)
        self.boat_push(d)
        self.draw_engine(d)
        self.draw_steer(d)
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
        if prev and alerts.bits_of(prev) != alerts.bits_of(d):         # both alarm words
            new = alerts.bits_of(d) & ~alerts.bits_of(prev)
            if new:
                self.event(T("cap.alarm_banner", list=", ".join(alerts.texts(new))))
        if self.callouts:
            self.callouts.telem(prev, d)
        # draw at most 10 times per second
        if not prev or d["t_ms"] - getattr(self, "_drawn", 0) >= 100 or d["t_ms"] < getattr(self, "_drawn", 0):
            self._drawn = d["t_ms"]
            self.draw()

    # --- drawing (the tools pages; the panels are drawn by render()) -----------------------------------------------
    def draw(self):
        d = self.t
        if self.pdlg is not None:
            ok = d["power_code"] and d["power_code_dir"] == self.pdlg_dir
            self.pdlg_code.configure(text="%04d" % d["power_code"] if ok else "....")
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
        if self.gw_status:
            lines.append("")
            lines.append(T("cap.sys_stations"))
            for st in self.gw_status.get("stations", []):
                lines.append("  %-3d %-14s %-10s %-9s %-9s %6s ms  %s %s%s" % (
                    st["id"], (st.get("user") or "?")[:14], T("crew.role_" + st["crew"]) if st.get("crew") in
                    ("captain", "copilot") else T("role." + st["role"]) if i18n.has("role." + st["role"], "en")
                    else st["role"], T("role." + st["role"]) if i18n.has("role." + st["role"], "en") else st["role"],
                    T("cap.on_board") if st["local"] else T("cap.remote"),
                    st["rtt_ms"] if st["rtt_ms"] is not None else "-", T("cap.sys_telemetry"),
                    T("tmode." + st["tmode"]) if i18n.has("tmode." + str(st["tmode"]), "en") else st["tmode"],
                    T("cap.sys_you") if st["id"] == self.gw_status.get("you") else ""))
        if "gw_state" in d:
            lines.append(T("cap.sys_gw", state=st_text(d["gw_state"], "gwstate."), n=d["gw_stations"],
                seq=d["pi_log_seq"], gaps=d["pi_log_gaps"], dropped=d["log_dropped_pi"], skipped=d["log_skipped"]))
        self.sys_lbl.configure(text="\n".join(lines))
        self.draw_joystick(d)
        self.draw_remote(d)
        try:
            if self.tools.state() != "withdrawn" and self.nb.select() == str(self.plot_tab):
                self.draw_plot()
        except tk.TclError:
            pass

    def draw_joystick(self, d):
        c = self.joy_cv
        c.delete("all")
        s = 130
        c.create_line(s / 2, 0, s / 2, s, fill=C["faint"])
        c.create_line(0, s / 2, s, s / 2, fill=C["faint"])
        db = 0.15 * s / 2
        c.create_rectangle(s / 2 - db, s / 2 - db, s / 2 + db, s / 2 + db, outline=C["faint"], dash=(2, 2))
        st = d["joy_status"]
        col = {0: C["faint"], 1: ORANGE, 2: BLUE, 3: RED}.get(st, C["fg"])
        if d["mode"] == proto.MODE_MANUAL:
            col = ORANGE
        x = s / 2 + d["joy_x"] * (s / 2 - 8)
        y = s / 2 - d["joy_y"] * (s / 2 - 8)
        c.create_oval(x - 9, y - 9, x + 9, y + 9, fill=col, outline=C["fg"], width=2)
        self.draw_lever(d)
        txt = T("joy.st_%d" % st) if st < 4 else "?"
        if st == 3 or (st == 1 and d["joy_why"]):
            txt += "\n" + joy_why(d)
        elif st == 1:
            txt = T("joy.not_neutral") if d["has_lever"] else txt
        self.joy_status.configure(text="%s\nX %+.2f  Y %+.2f" % (txt, d["joy_x"], d["joy_y"]),
                                  fg=RED if st in (1, 3) else C["fg"])
        if d["usb"]:
            ok = lambda b: T("cap.ok") if b else T("joy.unplugged")         # noqa: E731
            src = T("joy.usb_src", fps=d["usb_fps"], crc=d["usb_crc"], stick=ok(d["usb_dev0"]),
                    thr=ok(d["usb_dev1"])) + (T("joy.learn") if d["usb_learn"] else
                                              T("joy.starting") if d["usb_starting"] else "")
        else:
            src = T("joy.analog")
        self.joy_src.configure(text=src, fg=RED if d["usb"] and not (d["usb_dev0"] and d["usb_dev1"]) else C["fg"])
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
        c.create_rectangle(16, Y(1.0), w - 16, Y(d["nz_hi"]), fill=C["btn_ok"], outline="")
        c.create_rectangle(16, Y(d["nz_lo"]), w - 16, Y(0.0), fill=C["btn_no"], outline="")
        pick = d["pickup"] and d["mode"] in (proto.MODE_MANUAL, proto.MODE_HDG_LOCK)
        nz_col = theme.YELLOW if pick and d["pickup_target"] is None else C["faint"]
        c.create_rectangle(16, Y(d["nz_hi"]), w - 16, Y(d["nz_lo"]), fill=nz_col, outline="")
        c.create_text(w / 2, 6, text=T("gear.1"), font=F_TXT)
        c.create_text(w / 2, h - 5, text=T("gear.2"), font=F_TXT)
        if pick and d["pickup_target"] is not None:
            ty = Y(d["pickup_target"])
            c.create_polygon(w - 14, ty, w - 2, ty - 7, w - 2, ty + 7, fill=theme.YELLOW, outline=C["fg"])
        ly = Y(d["lever"])
        c.create_rectangle(6, ly - 4, w - 16, ly + 4, fill=ORANGE if pick else BLUE, outline=C["fg"])

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
        self.jl_lbl.configure(text="\n".join(lines), fg=RED if st in (1, 3) else C["fg"])
        m = self.jm_edit
        for i, (_, now) in enumerate(self.jm_rows):
            if i < 2:
                now.configure(text="")
                continue
            code = m["btn"][i - 2]
            on = d["usb"] and code and self.code_now(d, code)
            now.configure(text=T("joy.pressed") if on else "", fg=GREEN)

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
        self.wiz_info.configure(fg=C["fg"])
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
            c.create_line(60, y, w - 20, y, fill=C["plot_grid"])
            c.create_text(55, y, text="%.0f" % ((ref + k * span * 0.5) % 360), anchor="e", font=F_TXT)
        for idx, col, wd in ((2, BLUE, 2), (1, C["fg"], 3)):
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
            c.create_line(60, Y2(a), w - 20, Y2(a), fill=C["plot_grid"])
            c.create_text(55, Y2(a), text="%+d" % a, anchor="e", font=F_TXT)
        pts = []
        for p in self.hist:
            pts += [X(p[0]), Y2(p[3])]
        if len(pts) >= 4:
            c.create_line(*pts, fill=ORANGE, width=3)
        c.create_text(w - 20, h - 4, text=T("plot.last", s="%.0f" % PLOT_S), anchor="se", font=F_TXT)

    def alarm_tick(self):
        d = self.t
        stale = self.link is not None and self.t_rx and time.time() - self.t_rx > 3.0
        # MASTER lamps + the repeating warning sounds work with or without the driver screen open
        self.lamps.update(alerts.bits_of(d), no_data=bool(stale))
        if self.callouts and self.link is not None and self.t_rx:
            self.callouts.link(not stale)
        if (d and (d["alarm"] or alerts.bits_of(d))) or stale:
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
        self.prober.stop()
        if self.driver:
            self.driver.destroy()
            self.driver = None
        if self.link:
            self.link.stop()
        self.root.destroy()

    def window_title(self):
        """ "Trygons C2 – Σταθμός ξηράς – Πλοήγηση & έλεγχος" (step 13i: the same titles everywhere; the
        station's place: shore by default, "Σταθμός σκάφους" with --place onboard)."""
        where = T("win.station_onboard") if self.place == "onboard" else T("win.station_shore")
        return self.title_extra + T("win.title", name=where + " – " + T("cap.title"))

    def theme_changed(self, old):
        """Dark / light switched (both screens): the widgets get the new colours, the map + panels redraw."""
        for w in (self.root, self.tools):
            theme.retint(w, old)
            theme.style_tree(w)
        theme.apply_ttk(self.root)
        self.cv.configure(bg=C["bg"])
        self.mapwin.redraw()
        self.render(force=True)

    def collapse_changed(self, keys):
        settings.put("s2_collapsed", keys)
        self.render(force=True)

    def tools_menu(self, parent):
        m = tk.Menu(parent, tearoff=0)
        for key in self.tab_keys:
            m.add_command(label=T(key), command=lambda k=key: self.open_tab(k))
        return m

    # --- the tools window: every page that is not on the map ------------------------------------------
    def _build_tools(self):
        w = tk.Toplevel(self.root, bg=C["bg"])
        w.title(i18n.W("tools.title"))
        w.geometry(self.layout.get("tools") or "1180x780")
        w.withdraw()
        w.protocol("WM_DELETE_WINDOW", w.withdraw)
        self.tools = w
        nb = VTabs(w)
        self.nb = nb
        self.tab_keys = []
        self.tab_frames = {}

        def tab(key, build=None):
            fr = tk.Frame(nb.body, bg=C["bg"])
            nb.add(fr, text=T(key))
            self.tab_keys.append(key)
            self.tab_frames[key] = fr
            if build:
                build(fr)
            return fr

        tab("tab.connection", self._build_connection)
        tab("tab.control", self._build_control_tab)
        tab("tab.program", self._build_program)
        tab("tab.ais", self._build_ais)
        self.plot_tab = tab("tab.plot")
        self.plot = tk.Canvas(self.plot_tab, bg=C["bg"], highlightthickness=0)
        self.plot.pack(fill="both", expand=True)
        cf = tab("tab.config")
        ct = tk.Frame(cf, bg=C["bg"])
        ct.pack(fill="x")
        btn(ct, "cap.read_all", self.read_params).pack(side="left")
        lbl(ct, "cap.new_value", font=F_TXT).pack(side="left", padx=(12, 2))
        self.pval_var = tk.StringVar()
        tk.Entry(ct, textvariable=self.pval_var, font=F_MED, width=10, bg=C["entry"], fg=C["entry_fg"],
                 insertbackground=C["entry_fg"]).pack(side="left")
        btn(ct, "cap.write_selected", self.write_param).pack(side="left", padx=4)
        lbl(ct, "cap.param_note", font=F_TXT).pack(side="left")
        self.ptree = ttk.Treeview(cf, columns=("value", "min", "max"), height=12)
        for c, wd in (("#0", 220), ("value", 140), ("min", 120), ("max", 120)):
            self.ptree.column(c, width=wd)
        self.ptree.heading("#0", text=T("cap.param"))
        for c in ("value", "min", "max"):
            self.ptree.heading(c, text=T("cap.param_" + c))
        self.ptree.pack(fill="both", expand=True)
        self.joytab = tab("tab.joystick")
        self._build_joystick(self.joytab)
        self._build_joytab(self.joytab)
        tab("tab.cameras", self._build_cameras)
        tab("tab.remote", self._build_remote)
        tab("tab.network", self._build_network)
        sy = tab("tab.system")
        lbl(sy, "cap.sys_title", font=F_MED).pack(anchor="w", pady=6)
        self.sys_lbl = tk.Label(sy, text="-", font=F_MONO, bg=C["bg"], fg=C["fg"], justify="left", anchor="w")
        self.sys_lbl.pack(anchor="w", padx=8)
        tab("tab.bench", self._build_bench)
        tab("tab.engine", self._build_engine)
        tab("tab.sound", self._build_sound)
        lg = tab("tab.events")
        self.events = tk.Listbox(lg, font=F_TXT, bg=C["panel"], fg=C["fg"], selectbackground=C["sel"])
        self.events.pack(fill="both", expand=True)
        self.tools_msg = tk.Label(w, text="", font=F_TXT, bg=C["head"], fg=C["fg"], anchor="w")
        self.tools_msg.pack(fill="x", side="bottom", before=nb.side)
        self.fit_tables()
        theme.style_tree(w)

    def fit_tables(self):
        fit_columns(self.ptree, ("#0", "value", "min", "max"))
        fit_columns(self.ais_tree, ["#0"] + [c for c, _t, _w in self.AIS_COLS[1:]])
        fit_columns(self.pg_tree, ("#0", "lat", "lon", "kn", "radius", "action"))

    def open_tab(self, key):
        """Show the tools window on page `key` (tab.*)."""
        self.tools.deiconify()
        self.tools_fit()
        self.tools.lift()
        self.nb.select(self.tab_frames[key])

    def tools_fit(self):
        """The tools window at least as high as its page list (no page name squeezed away), on the screen."""
        w = self.tools
        w.update_idletasks()
        need = self.nb.side.winfo_reqheight() + self.tools_msg.winfo_reqheight() + 4
        sh = w.winfo_screenheight() - 60
        if w.winfo_height() < min(need, sh):
            w.geometry("%dx%d" % (max(w.winfo_width(), 900), min(need, sh)))
            w.update_idletasks()

    def _build_connection(self, cn):
        """Connection page: the boat list, a typed address (USB board, simulator, service), the log file."""
        top = tk.Frame(cn, bg=C["bg"])
        top.pack(fill="x", pady=8)
        btn(top, "boat.button", self.open_boats, kind="warn").pack(side="left", padx=(6, 12))
        lbl(top, "cap.connection", font=F_MED).pack(side="left", padx=6)
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, values=list_ports(), width=28, font=F_MED)
        self.port_box.pack(side="left", padx=4)
        btn(top, "cap.refresh", lambda: self.port_box.configure(values=list_ports())).pack(side="left")
        self.conn_btn = btn(top, "cap.connect", self.connect, kind="ok")
        self.conn_btn.pack(side="left", padx=6)
        self.link_lbl = tk.Label(cn, text=T("cap.link_none"), font=F_MED, bg=C["bg"], fg=C["fg"], anchor="w")
        self.link_lbl.pack(fill="x", padx=8, pady=4)
        lbl(cn, "cap.conn_help", font=F_TXT, wraplength=900, justify="left").pack(anchor="w", padx=8, pady=4)
        self.log_lbl = tk.Label(cn, text="", font=F_TXT, bg=C["bg"], fg=C["dim"], anchor="w")
        self.log_lbl.pack(fill="x", padx=8)

    def _build_control_tab(self, f):
        """Control page: sea state, TRAINING (outboard calibration), the simulator's panel + joystick."""
        lbl(f, "cap.sea_state", font=F_TXT).pack(anchor="w", pady=(6, 0), padx=6)
        cs = tk.Frame(f, bg=C["bg"])
        cs.pack(fill="x", padx=6)
        self.sea_var = tk.IntVar(value=2)
        for sv in range(1, 6):
            tk.Radiobutton(cs, text=str(sv), variable=self.sea_var, value=sv, font=F_MED, bg=C["btn"], fg=C["fg"],
                           selectcolor=C["sel"], indicatoron=False, width=3,
                           command=lambda: self.cmd(proto.T_SEA_STATE, proto.p_sea_state(self.sea_var.get()),
                                                    TE("what.sea_state", n=self.sea_var.get()))).pack(side="left")
        lbl(f, "cap.training", font=F_TXT).pack(anchor="w", pady=(10, 0), padx=6)
        tr = tk.Frame(f, bg=C["bg"])
        tr.pack(fill="x", padx=6)
        btn(tr, "cap.start", lambda: self.cmd(proto.T_TRAINING, proto.p_training(0), TE("what.training_start"))).pack(
            side="left")
        lbl(tr, "cap.full_angle", font=F_TXT).pack(side="left", padx=(6, 0))
        self.full_var = tk.StringVar(value="0")
        tk.Entry(tr, textvariable=self.full_var, font=F_MED, width=4, bg=C["entry"], fg=C["entry_fg"],
                 insertbackground=C["entry_fg"]).pack(side="left")
        btn(tr, "cap.accept", self.train_accept, kind="ok").pack(side="left", padx=6)
        btn(tr, "cap.reject", lambda: self.cmd(proto.T_TRAINING, proto.p_training(2), TE("what.training_reject")),
            kind="no").pack(side="left")
        # step 11e: the DroneCAN steering position servo - the TRAINING values, the adapted ones, the throttle model
        self.steer_lbl = tk.Label(f, text="", font=F_TXT, bg=C["bg"], fg=C["fg"], justify="left", anchor="w")
        self.steer_lbl.pack(fill="x", padx=6, pady=(4, 0))
        # simulator only: the panel buttons and the joystick CONFIRM
        Y = C["btn_sim"]
        self.sim_frame = tk.Frame(f, bg=Y)
        lbl(self.sim_frame, "cap.sim_panel", font=F_TXT, bg=Y).pack(anchor="w")
        sf = tk.Frame(self.sim_frame, bg=Y)
        sf.pack(fill="x")
        btn(sf, "cap.sim_hdg", self.sim_hdg).pack(side="left")
        btn(sf, "cap.sim_standby", self.sim_standby).pack(side="left", padx=2)
        btn(sf, "cap.sim_confirm", self.sim_confirm).pack(side="left")
        lbl(self.sim_frame, "cap.sim_usb", font=F_TXT, bg=Y).pack(anchor="w", pady=(4, 0))
        su = tk.Frame(self.sim_frame, bg=Y)
        su.pack(fill="x")
        lbl(su, "cap.sim_lever", font=F_TXT, bg=Y).grid(row=0, column=0)
        self.sim_lever = tk.Scale(su, from_=100, to=0, orient="vertical", length=110, bg=Y, fg=C["fg"], showvalue=True,
                                  highlightthickness=0, command=lambda v: self.sim_usb_set("lever", float(v) / 100.0))
        self.sim_lever.set(50)
        self.sim_lever.grid(row=1, column=0, rowspan=3)
        lbl(su, "cap.sim_stickx", font=F_TXT, bg=Y).grid(row=0, column=1, columnspan=3)
        self.sim_sx = tk.Scale(su, from_=-100, to=100, orient="horizontal", length=170, bg=Y, fg=C["fg"],
                               highlightthickness=0, command=lambda v: self.sim_usb_set("x", float(v) / 100.0))
        self.sim_sx.grid(row=1, column=1, columnspan=3)
        for i, (name, k) in enumerate((("take", proto.JM_TAKE), ("auto", proto.JM_AUTO), ("hdg", proto.JM_HDG_LOCK))):
            btn(su, "cap.sim_b_" + name, lambda k=k: self.sim_usb_press(k)).grid(row=2, column=1 + i, sticky="ew")
        for i, (name, k) in enumerate((("stby", proto.JM_STANDBY), ("m1", 5), ("p1", 6))):
            btn(su, "cap.sim_b_" + name, lambda k=k: self.sim_usb_press(k)).grid(row=3, column=1 + i, sticky="ew")
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

    def sim_hdg(self):
        self.cmd(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK), TE("what.sim_hdg"))

    def sim_standby(self):
        self.cmd(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY), TE("what.sim_standby"))

    # --- the map canvas: clicks go to a panel first, else to the map ----------------------------------------
    def cv_press(self, e):
        self._press_ov = self.ov.click(e.x, e.y)
        if self._press_ov:
            self.render(force=True)
        else:
            self.mapwin.press(e.x, e.y)

    def cv_motion(self, e):
        if not self._press_ov:
            self.mapwin.motion(e.x, e.y)

    def cv_release(self, e):
        if self._press_ov:
            self.ov.release()                       # a press-and-hold button (a momentary relay, START) lets go
        if not self._press_ov:
            self.mapwin.release(e.x, e.y)
            self.render(force=True)
        self._press_ov = False

    def ov_tick(self):
        try:
            self.mapwin.tick()
            self.render()
        except tk.TclError:
            return
        self.root.after(200, self.ov_tick)

    # --- screen 2 panels (overlay.py) ---------------------------------------------------------------------
    def render(self, force=False):
        """Lay out and draw the panels (5 x per second, at once after a click)."""
        now = time.time()
        if not force and now - self._render_t < 0.15:
            return
        self._render_t = now
        W, H = max(200, self.cv.winfo_width()), max(150, self.cv.winfo_height())
        ko = self.mapwin.keepout(W, H)
        self.ov.dpi = theme.dpi_factor(self.root)
        self.ov.begin(W, H)
        self.build_panels(W, H)
        self.rects = self.ov.render(W, H, ko, self.extras_size, self.draw_extras)
        mid = self.rects.get("middle")
        if mid is not None:
            nb = self.note_box
            if nb != self.mapwin.note_box:
                self.mapwin.note_box = nb
                self.mapwin.redraw()
                self.cv.tag_raise("ov")

    def extras_size(self):
        """STANDBY, the logo and the map bar at the current fit factor (overlay.layout)."""
        ov = self.ov
        W, H = ov.W, ov.H
        fs = ov.size("title")
        tw, _th = ov.measure(T("cap.btn_standby"), "title", True)
        sb = (tw + 2.4 * fs, fs * 2.1)
        lh = theme.logo_height(W, H)
        lw, _ = theme.logo_size(lh)
        bar = sum(ov.measure(s, "button", True)[0] + 2 * ov.size("button") * theme.BUTTON["pad_x"]
                  for s in self.bar_labels()) + ov.size("button") * 0.6 * len(self.bar_labels())
        return {"standby": sb, "logo": (lw, lh), "bar": (bar, ov.size("button") * theme.BUTTON["height"])}

    def bar_labels(self):
        out = [T("ov.zoom_out"), T("ov.zoom_in")]
        if not self.mapwin.follow:
            out.append(T("ov.back_to_boat"))
        out.append(T("ov.tools"))
        return out

    def draw_extras(self, rects):
        """STANDBY (always), the logo, the banners (top middle), the map bar + message + chart note
        (bottom middle)."""
        ov, cv = self.ov, self.cv
        W, H = ov.W, ov.H
        g = 8 * theme.unit(W, H)
        # STANDBY - always visible, also with every panel collapsed
        r = rects["standby"]
        self.ov_button(r, T("cap.btn_standby"), lambda: self.mode(proto.MODE_STANDBY), "standby", "title",
                       tag="standby")
        # the logo, bottom-right corner
        r = rects["logo"]
        img = theme.logo_pil(int(r[3] - r[1]))
        if img is not None:
            from PIL import ImageTk
            self._logo_photo = ImageTk.PhotoImage(img)
            cv.create_image(r[0], r[1], image=self._logo_photo, anchor="nw", tags="ov")
        else:
            cv.create_text((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, text=theme.LOGO_TEXT, fill=C["dim"],
                           font=ov.font(ov.size("value"), True), tags="ov")
        ov.ops.append({"k": "logo", "r": r, "panel": None, "s": "", "tag": "logo"})
        mx0, _my0, mx1, my1 = rects["middle"]
        cx, cy, kr = self.mapwin.keepout(W, H)
        # banners: from the top down, never into the boat's circle
        y = g
        limit = cy - kr - g
        self.banner_tags = []
        bl = self.banner_list()
        fnt = ov.font(ov.size("value"), True)
        for i, (tag, text, fg, fill, act) in enumerate(bl):
            lines = wrap(text, fnt, mx1 - mx0 - 2.2 * ov.size("value") - (ov.measure(T("cap.ack_alarm"), "button")[0] +
                                                                          2 * ov.size("button") if act else 0))
            lh = fnt.metrics("linespace")
            h = lh * len(lines) + ov.size("value") * 0.5
            if y + h > limit:
                more = len(bl) - i
                rr = (mx0, y, mx1, min(limit, y + lh))
                if rr[3] - rr[1] > 8:
                    cv.create_text((mx0 + mx1) / 2, (rr[1] + rr[3]) / 2, text=T("ov.more_banners", n=more),
                                   fill=theme.WARN, font=ov.font(ov.size("small"), True), tags="ov")
                break
            rr = (mx0, y, mx1, y + h)
            cv.create_polygon(*overlay.rrect_pts(rr[0], rr[1], rr[2], rr[3], 6), smooth=True, fill=fill, outline="",
                              tags="ov")
            cv.create_text(mx0 + ov.size("value") * 0.6, (rr[1] + rr[3]) / 2, text="\n".join(lines), anchor="w",
                           fill=fg, font=fnt, tags="ov")
            ov.ops.append({"k": "banner", "r": rr, "panel": None, "s": text, "tag": tag})
            self.banner_tags.append(tag)
            if act:
                bw = ov.measure(T("cap.ack_alarm"), "button", True)[0] + 2 * ov.size("button")
                bh = min(h - 6, ov.size("button") * theme.BUTTON["height"])
                br = (mx1 - bw - 4, (rr[1] + rr[3]) / 2 - bh / 2, mx1 - 4, (rr[1] + rr[3]) / 2 + bh / 2)
                self.ov_button(br, T("cap.ack_alarm"), act, "normal", "button", tag="ack")
            y += h + g * 0.5
        # the map bar at the bottom: zoom, back to boat, tools
        bh = ov.size("button") * theme.BUTTON["height"]
        labels = self.bar_labels()
        acts = [lambda: self.mapwin.zoom(1 / 0.6), lambda: self.mapwin.zoom(0.6)]
        if not self.mapwin.follow:
            acts.append(self.mapwin.back_to_boat)
        acts.append(self.popup_tools)
        ws = [ov.measure(s, "button", True)[0] + 2 * ov.size("button") * theme.BUTTON["pad_x"] for s in labels]
        gap = ov.size("button") * 0.6
        x = (mx0 + mx1) / 2 - (sum(ws) + gap * (len(ws) - 1)) / 2
        yb = my1 - bh
        for s, a, wd in zip(labels, acts, ws):
            kind = "warn" if s == T("ov.back_to_boat") else "normal"
            self.ov_button((x, yb, x + wd, my1), s, a, kind, "button", tag="bar")
            x += wd + gap
        # the message line above the bar
        y1 = yb - g * 0.5
        if self.msg_text:
            f = ov.font(ov.size("label"), True)
            lines = wrap(self.msg_text, f, mx1 - mx0 - 16)[:2]
            mh = f.metrics("linespace") * len(lines) + 6
            rr = (mx0, y1 - mh, mx1, y1)
            col = self.msg_col or C["head"]
            fg = theme.BLACK if col in (theme.GOOD, theme.CAUTION, theme.ADVISORY) else (theme.WHITE if col == theme.WARN
                                                                                       else C["fg"])
            cv.create_polygon(*overlay.rrect_pts(rr[0], rr[1], rr[2], rr[3], 5), smooth=True, fill=col, outline="",
                              tags="ov")
            cv.create_text(mx0 + 8, (rr[1] + rr[3]) / 2, text="\n".join(lines), anchor="w", fill=fg, font=f,
                           tags="ov")
            ov.ops.append({"k": "message", "r": rr, "panel": None, "s": self.msg_text, "tag": "message"})
            y1 = rr[1] - g * 0.5
        self.note_box = (mx0, y1 - 40 * max(0.8, theme.unit(W, H)), mx1, y1)

    def ov_button(self, r, label, action, kind, level, tag=None):
        ov, cv = self.ov, self.cv
        fs = ov.size(level)
        fill = {"standby": theme.WARN, "warn": C["btn_warn"]}.get(kind, C["btn"])
        fg = theme.WHITE if kind == "standby" else C["btn_fg"]
        cv.create_polygon(*overlay.rrect_pts(r[0], r[1], r[2], r[3], fs * theme.BUTTON["radius"] * 2), smooth=True,
                          fill=fill, outline=C["line"], width=2 if kind == "standby" else 1, tags="ov")
        cv.create_text((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, text=label, fill=fg, font=ov.font(fs, True), tags="ov")
        ov.ops.append({"k": "button", "r": r, "panel": None, "s": label, "tag": tag, "enabled": True})
        ov.hits.append((r, action))

    def popup_tools(self):
        m = self.tools_menu(self.root)
        m.add_separator()
        m.add_command(label=T("menu.theme_dark") if theme.name() == "light" else T("menu.theme_light"),
                      command=theme.toggle)
        m.add_command(label=T("menu.toggle_screen"), command=self.toggle_screen)
        x, y = self.root.winfo_pointerxy()
        try:
            m.tk_popup(x, y)
        finally:
            m.grab_release()

    def banner_on(self, tag):
        """Is banner `tag` shown on screen 2 now (tests)?"""
        return tag in self.banner_tags

    def banner_list(self):
        """[(tag, text, text colour, fill, button action or None)] most important first."""
        out = []
        d = self.t
        B = theme.BANNERS
        # the alarms (and NO DATA) are in the top-right "alarms" panel (panel_alarms), not a banner
        if d is not None:
            ll = d.get("ll_state", 0)
            if ll:
                txt = T("cap.ll_killed") if ll == 2 else T("cap.ll_lost", s="%.0f" % d.get("ll_left_s", 0.0),
                                                           near=T("cap.ll_near") if d.get("near_shore") else
                                                           T("cap.ll_far", m=d.get("coast_dist_m") or "?"))
                out.append(("lost", txt) + B["lost"] + (None,))
            if d["confirm_kind"]:
                out.append(("confirm", T("cap.confirm_banner", what=T("confirm.%d" % min(
                    d["confirm_kind"], len(proto.CONFIRM_KINDS) - 1)), s="%.0f" % d["confirm_left_s"])) +
                    B["confirm"] + (None,))
            if d["mode"] == proto.MODE_REMOTE:
                st = d.get("remote_state", 1)
                out.append(("remote", T("cap.remote_lost_banner") if st == 2 else T("cap.remote_banner")) +
                           ((theme.WHITE, theme.WARN) if st == 2 else B["remote"]) + (None,))
            elif d["mode"] == proto.MODE_MANEUVER:
                out.append(("maneuver", T("drv.maneuver")) + B["maneuver"] + (None,))
            elif d["mode"] == proto.MODE_MANUAL:
                out.append(("joystick", T("cap.joy_in_control")) + B["joystick"] + (None,))
            if d["pickup"] and d["mode"] in (proto.MODE_MANUAL, proto.MODE_HDG_LOCK):
                tgt = d["pickup_target"]
                out.append(("pickup", T("cap.pickup_banner", to="%.0f %%" % (100 * tgt) if tgt is not None else
                                        T("cap.pickup_neutral", lo="%.0f" % (100 * d["nz_lo"]),
                                          hi="%.0f" % (100 * d["nz_hi"])),
                                        now="%.0f" % (100 * (d["lever"] or 0.0)))) + B["pickup"] + (None,))
            if d.get("first_test"):
                out.append(("first_test", T("cap.ft_banner")) + B["first_test"] + (None,))
            if d.get("nv_fault"):
                out.append(("nv_fault", T("nv.fault")) + B["pickup"] + (None,))
            elif d.get("nv_reset"):
                out.append(("nv_reset", T("nv.reset")) + B["pickup"] + (None,))
            if d.get("batt_no_data"):                   # a notice, never an alarm (CLAUDE.md J)
                out.append(("batt_no_data", T("batt.no_data")) + B["pickup"] + (None,))
            if d.get("can_fitted") and d.get("can_warn"):   # DroneCAN node health WARNING: a notice only (OD-6)
                out.append(("can_warn", T("can.warning", nodes=can_roles(d["can_warn"]))) + B["pickup"] + (None,))
            if d.get("can_fitted") and d.get("can_anon"):   # a node without a static node ID (never answered)
                out.append(("can_anon", T("can.anon")) + B["pickup"] + (None,))
            if d.get("sv_servo") and d.get("sv_needs_training"):   # step 11e: the position servo has no TRAINING yet
                out.append(("steer_train", T("steer.needs_training")) + B["pickup"] + (None,))
            if d.get("sv_servo") and d.get("sv_reset_notice"):     # back to the TRAINING values (30 s notice)
                out.append(("steer_reset", T("steer.reset_notice")) + B["pickup"] + (None,))
            gs = d.get("gen_state", 0)                  # generator mode (CLAUDE.md K): notices, the GEAR alarm is the alarm
            if gs == 2:
                out.append(("gen_starting", T("gen.starting")) + B["pickup"] + (None,))
            elif gs == 6:
                out.append(("gen_fault", T("gen.fault")) + B["pickup"] + (None,))
            elif gs == 7:
                out.append(("gen_done", T("gen.done")) + B["pickup"] + (None,))
            if d.get("profile") == proto.PROFILE_UNCREWED:
                out.append(("uncrewed", T("cap.unc_banner")) + B["uncrewed"] + (None,))
        av = (self.gw_status or {}).get("avoid") or {}
        sh = av.get("shallow")
        if sh:
            out.append(("shallow", T("map.shallow_ahead", s="%.0f" % sh["in_s"], depth=T("hud.no_data")
                        if sh["depth"] is None else "%.1f m" % sh["depth"], limit="%.1f" % sh["limit"]),
                        theme.WHITE, theme.WARN, None))
        danger = [t for t in av.get("threats") or [] if t.get("level") == "danger"]
        if danger or av.get("state") == "DRIVER DECISION":
            t = danger[0] if danger else None
            txt = T("drv.collision", name=t["name"], cpa="%.2f" % t["cpa_nm"], tcpa="%.1f" % t["tcpa_min"],
                    brg="%03.0f" % t["bearing"]) if t else ""
            if av.get("state") == "DRIVER DECISION":
                txt = (T("drv.decision") + "   " + txt).strip()
            out.append(("collision", txt, theme.WHITE, theme.WARN, None))
        n = (self.gw_status or {}).get("net") or {}
        if n.get("emergency"):
            sl = n.get("starlink") or {}
            out.append(("starlink", T("cap.starlink_banner", mb="%.1f" % sl.get("session_mb", 0),
                                      gb="%.2f" % sl.get("month_gb", 0)) +
                        (T("cap.data_warning") if sl.get("warning") else "")) + B["starlink"] + (None,))
        if self.single_op:
            out.append(("single_op", T("cap.so_banner")) + B["single_op"] + (None,))
        if self.boat is not None and not self.boat_ok:
            out.append(("boat", T("boat.banner", name=self.boat.name)) + B["boat"] + (None,))
        if self.mapwin.edit:
            out.append(("edit", T("ov.edit_help_" + self.mapwin.edit), theme.BLACK, theme.ADVISORY, None))
        return out

    def build_panels(self, W, H):
        self.panel_status()
        self.panel_control()
        self.panel_alarms()                 # top-right, first: always drawn (CLAUDE.md G)
        self.panel_helm()
        self.panel_link()
        self.panel_traffic(H)
        self.panel_relays()                 # placed by the fit test (CLAUDE.md O)

    def panel_alarms(self):
        """Top-right (0), ALWAYS drawn (also with every panel collapsed; no alarm = a one-line header): every
        active alarm, one per line, most severe first. Not acknowledged on this PC (MASTER lamp state) =
        flashing on the lamp clock (WARNING red, CAUTION amber; the text always drawn, only the fill /
        outline alternates); acknowledged and ADVISORY = steady. ACK ALARM = the boat's alarm acknowledge
        (clears only alarms whose cause is gone)."""
        rows = self.lamps.rows()
        d = self.t
        p = self.ov.panel("alarms", T("ov.alarms", n=len(rows)) if rows else T("ov.no_alarms"), "ne",
                          collapsible=False)
        if not rows and d is not None and d["alarm"]:
            rows = [(T("cap.alarm_autopilot"), alerts.CAUTION, True)]
        if not rows:
            return
        lit = alerts.flash()
        n = ALARM_ROWS[0] if self.ov.H < 900 else ALARM_ROWS[1]
        for i, (txt, lv, steady) in enumerate(rows[:n]):
            col = theme.WARN if lv >= alerts.WARNING else theme.CAUTION if lv == alerts.CAUTION else theme.ADVISORY
            if lv >= alerts.CAUTION and not steady:
                p.badge(txt, col if lit else "", fg=(theme.WHITE if lv >= alerts.WARNING else theme.BLACK) if lit
                        else col, level="label", tag="alarm_row", outline=col, wrap=0.26)
            else:
                p.text(txt, "label", colour=col, bold=True, tag="alarm_row", wrap=0.26)
            if i == 0:                      # ACK ALARM beside the first row (the panel stays low)
                p.button(T("ov.ack_alarm"), lambda: self.cmd(proto.T_ALARM_ACK, b"", TE("what.alarm_ack")), "warn",
                         level="small", tag="ack")
            p.br()
        if len(rows) > n:
            p.text(T("ov.more_banners", n=len(rows) - n), "small", colour=theme.WARN, bold=True, tag="alarm_more")
            p.br()

    def panel_status(self):
        """Top-left: mode, power limit, heading / target / rate of turn / gear / throttle / GPS."""
        p = self.ov.panel("status", T("ov.status"), "nw")
        self.boat_line(p)                   # the FIRST line: the boat dropdown + lamp (CLAUDE.md H)
        p.br()
        d = self.t
        if d is None:
            p.badge(T("cap.not_connected") if not self.link else T("alarm.no_data"), MODE_COLORS["STANDBY"])
            return
        mode = d["mode_name"]
        p.badge(T("mode." + mode), MODE_COLORS.get(mode, MODE_COLORS["STANDBY"]), tag="mode")
        if d["mode"] in (proto.MODE_HDG_LOCK, proto.MODE_AUTO):
            p.text("→ %05.1f°" % d["target"], "value", tag="target")
        elif d["mode"] == proto.MODE_DP and "dp_heading" in d:
            p.text("→ %05.1f°" % d["dp_heading"], "value", tag="target")
        p.br()
        p.text(T("cap.power_lbl", f="%.0f" % d["power_fwd"], r="%.0f" % d["power_rev"]), "label",
               colour=C["note_txt"] if d["power_raised"] else None, bold=True, tag="power")
        p.br(stretch=True)
        p.button(T("cap.raise_fwd"), lambda: self.power_dialog(0))
        p.button(T("cap.raise_rev"), lambda: self.power_dialog(1))
        p.button(T("cap.lower_now"), lambda: self.cmd(proto.T_POWER_LOWER, b"", TE("what.power_lower")), "ok")
        p.br()
        bad, ok = C["bad_txt"], C["ok_txt"]
        gear = d["gear"] if d["gear"] < 3 else None
        thr = "%.0f %%" % d["throttle"] if d["throttle_demand"] <= d["throttle"] + 0.5 else \
            T("cap.thr_asked", thr="%.0f" % d["throttle"], ask="%.0f" % d["throttle_demand"])
        src = T("pos.%d" % d["pos_source"]) if d["pos_source"] < 4 else "?"
        rows = [([T("cap.v_heading"), "%05.1f°" % d["heading"] if d["compass_ok"] else T("cap.no_compass")],
                 None if d["compass_ok"] else bad),
                ([T("cap.v_dp"), self.dp_status_text(d)], C["bad_txt"] if d.get("dp_offspot") else None)
                if d["mode"] == proto.MODE_DP else ([T("cap.v_target"), "%05.1f°" % d["target"]], None),
                ([T("cap.v_yaw"), "%+.1f °/s" % d["yaw_rate"]], None),
                ([T("cap.v_gear"), T("gear.%d" % gear) if gear is not None else "?"],
                 {1: ok, 2: bad}.get(gear)),
                ([T("cap.v_throttle"), thr], None),
                ([T("cap.v_outboard"), "%+.1f°" % d["outboard"]], None),
                ([T("cap.v_gps"), (T("cap.fix") if d["gps_fix"] else T("cap.no_fix")) + "  " + src +
                  "  ± %.0f m" % d["unc_m"]], ok if d["gps_fix"] and d["pos_source"] == 1 else bad),
                ([T("cap.v_sog"), "%.1f kn  %03.0f°" % (d["est_sog"], d["est_cog"])], None)]
        # the 12 V bank (TELEM2 group L): filtered volts, red while LOW BATTERY; "no data" when not measured
        bv = d.get("bank_v")
        # generator mode (TELEM2 group L) on the same row while generator auto is on: its state, the throttle
        # it asks while running (a row of its own would not fit at 1366 x 658)
        gs = d.get("gen_state", 0)
        btxt = "%.1f V" % bv if bv is not None else T("cap.batt_none")
        if gs:
            btxt += "  ·  " + T("cap.v_generator") + " " + T("gen.st_%d" % min(gs, 7)) +                     (" %d %%" % d.get("gen_pct", 0) if gs in (3, 4) else "")
        rows.append(([T("cap.v_battery"), btxt], bad if d.get("batt_low") or gs == 6 else ok if gs == 3 else None))
        p.table(None, rows, ["l", "l"], tag="values", level="label")

    def panel_control(self):
        """Bottom-left: mode buttons, AUTO, the route / geofence / max speed on the map, heading lock,
        the captain's answers to this station's actions."""
        p = self.ov.panel("control", T("ov.control"), "sw")
        d = self.t or {}
        mode = d.get("mode_name")
        if d.get("mode") == proto.MODE_DP:
            # DP replaces the mode buttons and the AUTO rows (CLAUDE.md P, step 11): none of those modes can be
            # entered from DP (the AURIX's mode table), STANDBY / DP STOP / the joystick leave it
            self.dp_rows(p, d)
        else:
            p.button(T("cap.btn_hdg_lock"), self.info_hdg, on=mode == "HDG_LOCK", tag="btn_hdg")
            p.button(T("cap.btn_auto"), self.auto_start, on=mode == "AUTO", tag="btn_auto")
            if self.manual_here():
                p.button(T("cap.btn_manual"), self.remote_request, "info", on=mode == "REMOTE_MANUAL",
                         tag="btn_manual")
                p.button(T("cap.btn_maneuver"), self.maneuver_request, "info", on=mode == "MANEUVER",
                         tag="btn_maneuver")
            p.br()
            p.text(self.auto_text(), "label", tag="auto")
            p.br(stretch=True)
            p.button(T("cap.pause"), lambda: self.cmd(proto.T_AUTO_PAUSE, b"", TE("what.auto_pause")))
            p.button(T("cap.resume"), lambda: self.cmd(proto.T_AUTO_RESUME, b"", TE("what.auto_resume")))
            p.button(T("cap.skip"), lambda: self.cmd(proto.T_AUTO_SKIP, b"", TE("what.auto_skip")))
            p.button(T("cap.return"), self.auto_return, tag="btn_return")
            self.gen_button(p, d)
            p.br()
            if self.dp_available():
                self.dp_rows(p, d)
        wps = self.prog["wps"]
        p.text(T("ov.route", n=len(wps), nm="%.1f" % self.route_nm(), kn="%.0f" % self.new_wp["kn"]) +
               (T("ov.not_sent") if self.route_dirty else ""), "label",
               colour=C["note_txt"] if self.route_dirty else None, tag="route")
        p.br()
        ed = self.mapwin.edit
        p.button(T("ov.edit_route"), lambda: self.set_edit("route"), on=ed == "route", tag="edit_route")
        p.button("−", lambda: self.wp_speed(-1))
        p.button("+", lambda: self.wp_speed(1))
        p.button(T("ov.send_route"), self.route_send, "ok", enabled=bool(wps), tag="send_route")
        p.button(T("ov.clear"), self.prog_clear, enabled=bool(wps))
        p.br()
        cur = self.params.get(P_MAX_LEG, {}).get("value")
        v = self.maxkn_edit if self.maxkn_edit is not None else cur
        p.text(T("ov.max_speed", kn="-" if v is None else "%.0f" % v), "label",
               colour=C["note_txt"] if self.maxkn_edit is not None and self.maxkn_edit != cur else None, tag="maxkn")
        p.button("−", lambda: self.maxkn_step(-1))
        p.button("+", lambda: self.maxkn_step(1))
        p.button(T("ov.set"), self.maxkn_send, "ok", enabled=self.maxkn_edit is not None, tag="maxkn_set")
        p.br()
        fl = self.fence_live()
        if self.fence_dirty:
            ft = T("ov.fence_changed", n=len(self.fence_draft))
        elif fl:
            ft = T("ov.fence_set", n=len(fl), by=(((self.gw_status or {}).get("geofence") or {}).get("by") or "?"))
        else:
            ft = T("ov.fence_none")
        p.text(ft, "label", colour=C["note_txt"] if self.fence_dirty else None, tag="fence")
        p.br()
        p.button(T("ov.edit_fence"), lambda: self.set_edit("fence"), on=ed == "fence", tag="edit_fence")
        p.button(T("ov.send_fence"), self.fence_send, "ok", enabled=self.fence_dirty, tag="send_fence")
        p.button(T("ov.clear_fence"), self.fence_clear, enabled=bool(fl or self.fence_draft), tag="clear_fence")
        p.br(stretch=True)
        for dd in (-10, -1, 1, 10):
            p.button("%+d" % dd, lambda dd=dd: self.cmd(proto.T_HEADING_ADJ, proto.p_heading(dd),
                                                        TE("what.heading", d="%+d" % dd)), level="value")
        now = time.time()
        shown = [x for x in self.decisions if x["state"] == "wait" or now - x["t"] < 60.0][-3:]
        if shown:
            p.br()
            p.text(T("ov.captain_answers"), "small", colour=C["dim"], bold=True)
            for x in shown:
                p.br()
                col = {"wait": theme.CAUTION, "ok": C["ok_txt"]}.get(x["state"], C["bad_txt"])
                left = max(0.0, x["t"] + self.confirm_timeout_s - now)
                st = T("ov.dec_wait", s="%.0f" % left) if x["state"] == "wait" else T("ov.dec_" + x["state"])
                if x.get("why"):
                    st += " - " + x["why"]
                p.text("%s: %s" % (i18n.from_en(x["what"]), st), "small", colour=col, tag="decision")
        if self.port_var.get().startswith("tcp:") and self.link is not None:
            p.br()
            p.button(T("cap.sim_hdg"), self.sim_hdg, "sim")
            p.button(T("cap.sim_standby"), self.sim_standby, "sim")
            p.button(T("cap.sim_confirm"), self.sim_confirm, "sim", tag="sim_confirm")

    def boat_state(self):
        """(the boat shown in the dropdown or None, lamp state). The lamp is "connected" ONLY when that
        boat's gateway was found by the prober AND this station is connected to it AND the welcome name
        check passed (CLAUDE.md H); otherwise "online" (found, not connected), "offline", "no_ip",
        "checking", "address" (connected to a typed address: no boat check) or "none" (nothing chosen)."""
        bt = self.boat or self.boat_pick
        if bt is None:
            return None, "address" if self.link is not None else "none"
        st = self.prober.status(bt)                 # "invalid" / "no_ip" without probing (boats.Boat.fixed_status)
        st = "checking" if st == "?" else st
        lk = self.link
        if st == "online" and self.boat is not None and self.boat.id == bt.id and lk is not None and \
                getattr(lk, "connected", False) and boats.check_welcome(bt, getattr(lk, "welcome", None)) is None:
            st = "connected"
        return bt, st

    def boat_line(self, p):
        """Screen 2 status panel, first line: the boat dropdown (boats/ by name), the lamp, "name verified"."""
        bt, st = self.boat_state()
        if bt is not None:
            label = bt.name
        elif st == "address":
            label = self.port_var.get()
        else:
            label = T("ov.choose_boat")
        p.button(label + "  ▾", self.boat_dropdown, "info", level="value", tag="boat_dd")
        if st == "connected":
            p.badge("● " + T("ov.connected"), theme.GOOD, fg=theme.BLACK, level="label", tag="boat_lamp")
        elif st != "none":
            txt = T("ov.no_boat_check") if st == "address" else T("boat.st_" + st)
            p.badge("● " + txt, C["head"], fg=C["dim"], level="label", tag="boat_lamp", outline=C["line"])
        if self.boat is not None and self.link is not None:
            p.text(T("ov.verified") if self.boat_ok else T("ov.not_verified"), "small",
                   colour=C["ok_txt"] if self.boat_ok else theme.CAUTION, bold=True, tag="verified")

    def boat_menu_entries(self):
        """The dropdown's entries: [(label, fn)], None = a separator. The boats by name (status, the IP
        only as a detail at the end), then another address / disconnect."""
        out = []
        for bt in self.boat_list:
            st = self.prober.status(bt)
            st = "checking" if st == "?" else st
            if self.boat is not None and self.boat.id == bt.id and self.boat_state()[1] == "connected":
                st = "connected"
            stt = T("ov.connected") if st == "connected" else T("boat.st_" + st)
            out.append(("%s    %s    · %s" % (bt.name, stt, bt.ip if bt.ip_ok() else T("ov.ip_not_set")),
                        lambda bt=bt: self.choose_boat(bt)))
        out.append(None)
        out.append((T("ov.other_address"), lambda: self.open_tab("tab.connection")))
        if self.link is not None:
            out.append((T("cap.disconnect"), self.disconnect))
        return out

    def boat_dropdown(self, post=True):
        """Open the boat dropdown under its button (post=False: only build it - the tests)."""
        m = tk.Menu(self.root, tearoff=0, font=theme.font("value"), bg=C["panel"], fg=C["fg"],
                    activebackground=C["sel"], activeforeground=C["fg"])
        for e in self.boat_menu_entries():
            if e is None:
                m.add_separator()
            else:
                m.add_command(label=e[0], command=e[1])
        self.boat_menu = m
        if post:
            r = [o["r"] for o in self.ov.ops if o.get("tag") == "boat_dd"]
            x, y = (r[0][0], r[0][3]) if r else (0, 0)
            try:
                m.tk_popup(int(self.cv.winfo_rootx() + x), int(self.cv.winfo_rooty() + y))
            finally:
                m.grab_release()
        return m

    def panel_helm(self):
        """Top-right (2): who holds HELM, my role, take / release / swap, screen 1, single operator."""
        p = self.ov.panel("helm", T("ov.helm"), "ne")
        p.text(self.helm_text(), "label", bold=True, colour=C["ok_txt"] if self.helm_mine() else None, tag="helm")
        crew, cap, lic = crewui.crew_of(self) if isinstance(self.link, GatewayLink) else (None, None, False)
        if crew in ("captain", "copilot"):
            p.br()
            txt = T("crew.you_" + crew) + "  " + (T("ov.licence_yes") if lic else T("ov.licence_no"))
            if crew == "copilot" and cap:
                txt += "  " + T("crew.captain_is", name=cap)
            p.text(txt, "label", colour=C["ok_txt"] if crew == "captain" else None, tag="crew")
        p.br(stretch=True)
        p.button(T("cap.helm_take"), lambda: self.cmd(proto.T_HELM, proto.p_helm(0), TE("what.take_helm")), "ok")
        p.button(T("cap.helm_release"), self.helm_release)
        p.button(T("crew.swap"), self.crew_swap, enabled=isinstance(self.link, GatewayLink))
        p.button(T("ov.screen1"), lambda: self.close_driver() if self.driver else self.open_driver(), "info",
                 on=self.driver is not None)
        p.button(T("cap.single_op_on") if self.single_op else T("cap.single_op"), self.single_op_toggle,
                 on=self.single_op, enabled=isinstance(self.link, GatewayLink), tag="single_op")

    def panel_link(self):
        """Top-right (3): channel quality, CREWED / UNCREWED, near shore / open sea, lost link, engine."""
        p = self.ov.panel("link", T("ov.link"), "ne")
        d = self.t or {}
        age = time.time() - self.t_rx if self.t_rx else None
        ok = self.link is not None and self.link.connected and age is not None and age < 1.0
        p.text(T("cap.link_ok") if ok else (T("cap.link_nodata") if self.link else T("cap.link_none")), "label",
               bold=True, colour=C["ok_txt"] if ok else C["bad_txt"], tag="data")
        n = (self.gw_status or {}).get("net")
        if n and n.get("channels"):
            rows = []
            for c in n["channels"]:
                q = quality(c)
                bars = "▮" * q + "▯" * (4 - q)
                rows.append((["▶ " + c["name"] if c["active"] else "   " + c["name"], bars,
                              "%.0f ms" % c["latency_ms"] if c.get("latency_ms") is not None else "-",
                              "%.0f %%" % c["loss_pct"] if c.get("loss_pct") is not None else "-",
                              T("cap.up") if c["up"] else T("cap.down")],
                             C["ok_txt"] if q >= 3 else (C["note_txt"] if q >= 1 else C["bad_txt"])))
            p.br()
            p.table([T("ov.ch"), T("ov.quality"), T("cap.net_col_latency"), T("cap.net_col_loss"),
                     T("cap.net_col_state")], rows, ["l", "l", "r", "r", "l"], tag="channels")
        elif self.link is not None:
            p.br()
            p.text(T("ov.direct_link"), "small", colour=C["dim"])
        p.br()
        prof = d.get("profile")
        if prof is not None:
            p.badge(T("prof.%d" % prof) if prof < 3 else "?",
                    theme.BANNERS["uncrewed"][1] if prof == proto.PROFILE_UNCREWED else C["lit"], tag="profile",
                    level="label")
        if d:
            if d.get("coast_valid"):
                # (the boat sends 0xFFFE / 0xFFFF beyond the 16-bit range: farther than 65 km = no number)
                dm = d.get("coast_dist_m")
                ms = "%.0f" % dm if dm is not None else "> 65000"
                txt = T("ov.near_shore", m=ms) if d.get("near_shore") else T("ov.open_sea", m=ms)
            else:
                txt = T("ov.shore_unknown")
            p.text(txt, "label", colour=C["note_txt"] if d.get("near_shore") or not d.get("coast_valid") else None,
                   tag="shore")
            ll = d.get("ll_state", 0)
            p.br()
            if ll == 1:
                p.badge(T("ov.ll_near") if d.get("near_shore") else T("ov.ll_count", s="%.0f" % d.get("ll_left_s", 0)),
                        theme.WARN, tag="lostlink", level="label")
            elif ll == 2:
                p.badge(T("ov.engine_killed"), theme.WARN, tag="engine", level="label")
            if ll != 2:
                p.text(T("ov.engine_run"), "label", colour=C["dim"], tag="engine")

    # --- relay buttons (CLAUDE.md O): screen 2 panel "relays" + the HUD box on screen 1 ---------------------
    def relay_groups_known(self):
        """{n: group} for proto.is_driving_cmd: the boat file's groups, but only when the AURIX's ACTIVE table is
        the file's (TELEM2 group M hash) AND the gateway knows that table too (gw_status "relays") - so this
        station never thinks a relay free that the gateway would hold for the captain's ACCEPT. Else None
        (every relay = a driving command)."""
        b = self.boat
        d = self.t or {}
        if b is None or b.error or not b.relays or not d.get("relay_table"):
            return None
        h = proto.relay_table_hash(b.relays)
        rs = (self.gw_status or {}).get("relays") or {}
        if d.get("relay_hash") != h or rs.get("hash") != h or not rs.get("known"):
            return None
        return proto.groups_of(b.relays)

    def relay_view(self):
        """(active, rows): the boat file's relays as both screens show them. active = the AURIX uses exactly this
        table (TELEM2 group M hash = the file's, nothing pending). rows: dicts of the file + "name" (screen
        language) + "state" on / off / unknown (the AURIX readback; unknown = table not active, board silent,
        not read) + "hold" (press-and-hold: momentary, the STARTER)."""
        b = self.boat
        relays = list(b.relays) if b is not None and not b.error and b.relays else []
        d = self.t or {}
        active = bool(relays) and bool(d.get("relay_table")) and not d.get("relay_pending") and \
            d.get("relay_hash") == proto.relay_table_hash(relays)
        on, unk = set(d.get("relay_on") or []), set(d.get("relay_unknown") or [])
        el = i18n.lang() == "el"
        rows = []
        for r in relays:
            name = (r.get("name_el") if el else r.get("name_en")) or r.get("name_en") or "%d" % r["n"]
            if not active or not d.get("relay_ok") or r["n"] in unk:
                state = "unknown"
            else:
                state = "on" if r["n"] in on else "off"
            if r.get("role") == "starter" and state != "unknown":
                state = "on" if d.get("eng_crank") == 1 else "off"
            rows.append(dict(r, name=name, state=state, hold=r["type"] == "momentary" or r.get("role") == "starter"))
        return active, rows

    def relay_press(self, n):
        """A relay button pressed (screen 2 or screen 1). Requests only - the AURIX decides every time (HELM,
        the table, the pair interlock, max_s, the engine rules of section N); the gateway asks the captain for a
        copilot's engine relay. STARTER = START hold (crank_press); momentary = a board-timed pulse of at most
        max_s, OFF at release; onoff / latching / timed = toggle by the readback state."""
        active, rows = self.relay_view()
        r = next((x for x in rows if x["n"] == n), None)
        if r is None:
            return
        if not active:
            return self.say(T("ov.relay_not_active"), RED)
        if r.get("role") == "starter":
            return self.crank_press()
        d = self.t or {}
        if r["state"] == "unknown" and not (d.get("relay_no_readback") and d.get("relay_ok")):
            # (a DroneCAN board without readback: the state is known only once commanded - the press sends ON)
            return self.say(T("ov.relay_unknown", name=r["name"]), RED)
        if r["hold"]:
            self.relay_held = n
            self.cmd(proto.T_RELAY, proto.p_relay(n, proto.RELAY_CMD_PULSE, int(r["max_s"] * 1000)),
                     TE("what.relay_hold", name=r.get("name_en") or "%d" % n))
            return
        on = r["state"] != "on"
        self.cmd(proto.T_RELAY, proto.p_relay(n, proto.RELAY_CMD_ON if on else proto.RELAY_CMD_OFF),
                 TE("what.relay_onoff", name=r.get("name_en") or "%d" % n, state="ON" if on else "OFF"))

    def relay_release(self, n):
        """The press-and-hold relay let go: the STARTER = START released; a momentary relay = OFF now (the
        board's own pulse timer ends it anyway at max_s if this never arrives)."""
        _active, rows = self.relay_view()
        r = next((x for x in rows if x["n"] == n), None)
        if r is not None and r.get("role") == "starter":
            if self.crank_job is not None:
                self.crank_release()
            return
        if getattr(self, "relay_held", None) != n:
            return
        self.relay_held = None
        self.cmd(proto.T_RELAY, proto.p_relay(n, proto.RELAY_CMD_OFF),
                 TE("what.relay_onoff", name=(r or {}).get("name_en") or "%d" % n, state="OFF"))

    def relay_lines(self, rows):
        """[(group, [rows of one line])]: by group (engine, trim, extra), an interlocked pair on one line, other
        relays up to 3 per line."""
        out = []
        for grp in proto.RELAY_GROUPS:
            mine = [r for r in rows if r["group"] == grp]
            lines, done = [], set()
            for r in mine:
                if r["n"] in done:
                    continue
                mate = next((m for m in mine if r.get("pair") and m["n"] == r["pair"]), None)
                if mate is not None:
                    lines.append([r, mate])             # an interlocked pair: one line, never both on
                    done |= {r["n"], mate["n"]}
                elif lines and len(lines[-1]) < 3 and not lines[-1][0].get("pair"):
                    lines[-1].append(r)
                    done.add(r["n"])
                else:
                    lines.append([r])
                    done.add(r["n"])
            out += [(grp, ln) for ln in lines]
        return out

    def panel_relays(self):
        """Screen 2 "relays" (CLAUDE.md O): the boat file's relays by group (engine / trim / extra), the state from
        the AURIX readback (TELEM2 group M: lit = on, grey = unknown), momentary = press-and-hold, an interlocked
        pair on one line; the engine group also holds the restart page (IGNITION / START hold / SHUTOFF,
        RESTART). Only when the boat file has relays. Requests only (relay_press / relay_release)."""
        active, rows = self.relay_view()
        if not rows:
            return
        p = self.ov.panel("relays", T("ov.relays"), ("se", "sw", "ne", "nw"))
        d = self.t or {}
        if not active:
            # step 13h (review ce2e4d0 LOW-1): TELEM2 group M flag 128 - a stop / timer is still owed to the board,
            # so the AURIX refuses a table change (NACK 23, no timeout: a stop is never dropped) - say why the
            # boat file's table is not taken (in place of "not active": the panel never grows by a line)
            note = "ov.relay_pending" if d.get("relay_pending") else \
                "ov.relay_stop" if d.get("relay_stop") else "ov.relay_not_active"
            p.text(T(note), "small", colour=C["note_txt"], tag="relay_note", wrap=0.24)
            p.br()
        elif not d.get("relay_ok"):
            p.text(T("ov.relay_silent"), "small", colour=C["note_txt"], tag="relay_note", wrap=0.24)
            p.br()
        elif d.get("relay_no_readback"):
            p.text(T("ov.relay_no_readback"), "small", colour=C["note_txt"], tag="relay_note", wrap=0.24)
            p.br()
        last = None
        for grp, line in self.relay_lines(rows):
            p.text(T("ov.relay_grp_" + grp) if grp != last else "", "small", colour=C["dim"], bold=True,
                   tag="relay_grp")
            last = grp
            for r in line:
                p.button(r["name"], lambda n=r["n"]: self.relay_press(n),
                         "warn" if r.get("role") == "starter" else "normal", on=r["state"] == "on",
                         enabled=active and (r["state"] != "unknown" or r.get("role") == "starter" or
                                             bool(d.get("relay_no_readback") and d.get("relay_ok"))),
                         level="small", tag="relay_%d" % r["n"],
                         release=(lambda n=r["n"]: self.relay_release(n)) if r["hold"] else None)
            if any(r["hold"] for r in line):
                p.text(T("ov.relay_hold"), "small", colour=C["dim"], tag="relay_hold")
            p.br()
        if any(r["group"] == "engine" for r in rows):
            p.text(T("ov.relay_grp_engine") if last is None else "", "small")
            p.button(T("eng.restart"), self.engine_restart, "info", level="small", tag="relay_restart")
            if d.get("eng_killed") or d.get("eng_tripped"):
                p.text(T("eng.killed") if d.get("eng_killed") else T("eng.tripped"), "small", colour=C["bad_txt"],
                       bold=True, tag="relay_eng", wrap=0.18)
            p.br()

    # --- DP (CLAUDE.md P, step 11): the DP rows of screen 2's control panel + the status line; requests only -------
    def dp_available(self):
        """The DP rows of the control panel: in DP, or when the AURIX has both thrusters fitted (the boat file's
        settings taken)."""
        d = self.t or {}
        return d.get("mode") == proto.MODE_DP or bool(d.get("thr_fitted_bow") and d.get("thr_fitted_stern"))

    def dp_status_text(self, d):
        """'SPOT · 2.3 m' / 'CRUISE · 1.5 kn' / 'TRIP · WP 3 · 2.0 kn': the DP sub-state and its main number."""
        sub = d.get("dp_sub", 0)
        name = T("dp.sub_%d" % sub) if 0 <= sub < len(proto.DP_SUBS) else "?"
        if sub in (proto.DP_SPOT, proto.DP_JOG):
            return "%s · %s" % (name, T("dp.err", m="%.1f" % d.get("dp_err_m", 0.0)))
        if sub == proto.DP_CRUISE:
            return "%s · %.1f kn" % (name, d.get("dp_set_kn", 0.0))
        if sub == proto.DP_TRIP:
            return "%s · %s · %.1f kn" % (name, T("dp.wp", n=d.get("dp_wp", 0) + 1), d.get("dp_trip_kn", 0.0))
        return name

    def dp_err_text(self, d):
        """Where the spot is from the boat: '1.2 m ahead, 0.4 m to port'."""
        f, sb = d.get("dp_err_fwd", 0.0), d.get("dp_err_stbd", 0.0)
        return T("dp.err_dir", f="%.1f" % abs(f), fd=T("dp.ahead") if f >= 0 else T("dp.astern"),
                 s="%.1f" % abs(sb), sd=T("dp.stbd") if sb >= 0 else T("dp.port"))

    def dp_start(self, sub, cont=False):
        """START SPOT / CRUISE / TRIP (TRIP "continue" = from the DP's waypoint after a STOP waypoint). From STANDBY
        the boat waits for the joystick CONFIRM (onboard CREWED / shore UNCREWED); in DP a SPOT is at once. The
        gateway checks a TRIP like AUTO (shallow 114, geofence 120); a copilot's start waits for the captain."""
        kn = self.dp_kn if sub in (proto.DP_CRUISE, proto.DP_TRIP) else 0.0
        arg = sub | (proto.DP_TRIP_CONTINUE if cont else 0)
        if sub == proto.DP_CRUISE:
            what = TE("what.dp_cruise", kn="%.1f" % kn)
        elif sub == proto.DP_TRIP:
            what = TE("what.dp_trip_cont" if cont else "what.dp_trip", kn="%.1f" % kn)
        else:
            what = TE("what.dp_spot")
        self.cmd(proto.T_DP, proto.p_dp(proto.DP_START, arg, int(round(kn * 100))), what)

    def dp_stop(self):
        """DP STOP: thrusters 0, STANDBY (also cancels a waiting DP request). A copilot's goes straight."""
        self.cmd(proto.T_DP, proto.p_dp(proto.DP_STOP), TE("what.dp_stop"))

    def dp_jog(self, direction):
        key = ("what.dp_jog_ahead", "what.dp_jog_astern", "what.dp_jog_port", "what.dp_jog_stbd")[direction]
        self.cmd(proto.T_DP, proto.p_dp(proto.DP_JOG_ACT, direction), TE(key, m="%.1f" % self.dp_jog_m()))

    def dp_kn_step(self, dd):
        self.dp_kn = max(DP_KN_STEP, min(DP_MAX_KN, round((self.dp_kn + dd * DP_KN_STEP) / DP_KN_STEP) * DP_KN_STEP))
        self.render(force=True)

    def dp_speed_send(self):
        """CRUISE / TRIP: the new speed set point (DP only)."""
        self.cmd(proto.T_DP, proto.p_dp(proto.DP_SPEED, 0, int(round(self.dp_kn * 100))),
                 TE("what.dp_speed", kn="%.1f" % self.dp_kn))

    def dp_jog_m(self):
        b = self.boat
        t = getattr(b, "thrusters", None) if b is not None and not getattr(b, "error", None) else None
        try:
            return float((t or {}).get("jog_m", 1.5))
        except (TypeError, ValueError, AttributeError):
            return 1.5

    def gen_button(self, p, d):
        """Generator auto (CLAUDE.md K): the captain switches it on / off (a copilot's ON needs the captain's ACCEPT -
        the gateway holds it); the boat itself decides when it runs (STANDBY / DP, neutral, low bank); its state is in
        the status panel."""
        if d.get("gen_state", 0) == 0:
            p.button(T("cap.btn_gen_on"), lambda: self.cmd(proto.T_GENERATOR, bytes([1]), TE("what.generator_on")),
                     tag="btn_gen")
        else:
            p.button(T("cap.btn_gen_off"), lambda: self.cmd(proto.T_GENERATOR, bytes([0]), TE("what.generator_off")),
                     on=True, tag="btn_gen")

    def dp_rows(self, p, d):
        """The DP part of the control panel (CLAUDE.md P, step 11): in DP it replaces the mode buttons and the AUTO
        rows - the state, DP
        STOP, generator; SPOT / CRUISE / TRIP (+ continue after a STOP waypoint) and the CRUISE / TRIP speed; in SPOT /
        JOG the JOG arrows + where the spot is. The heading = the panel's +-1 / +-10 (HEADING_ADJ, a DP heading change
        in DP). Outside DP (both thrusters fitted) one line: SPOT / CRUISE / TRIP + the speed. Requests only - the
        AURIX decides every time (HELM, CONFIRM, GPS, thrusters, heat budget)."""
        in_dp = d.get("mode") == proto.MODE_DP
        sub = d.get("dp_sub", 0) if in_dp else 0
        if in_dp:
            p.badge("DP " + self.dp_status_text(d), MODE_COLORS.get("DP", MODE_COLORS["STANDBY"]), level="label",
                    tag="dp_state")
            p.button(T("dp.stop"), self.dp_stop, "warn", level="small", tag="dp_stop")
            self.gen_button(p, d)
            p.br()
        else:
            p.text(T("dp.label"), "small", colour=C["dim"], bold=True, tag="dp_state")
        p.button(T("dp.spot"), lambda: self.dp_start(proto.DP_SPOT), on=sub in (proto.DP_SPOT, proto.DP_JOG),
                 level="small", tag="dp_spot")
        p.button(T("dp.cruise"), lambda: self.dp_start(proto.DP_CRUISE), on=sub == proto.DP_CRUISE, level="small",
                 tag="dp_cruise")
        p.button(T("dp.trip"), lambda: self.dp_start(proto.DP_TRIP), on=sub == proto.DP_TRIP, level="small",
                 tag="dp_trip")
        if in_dp and sub == proto.DP_SPOT and d.get("dp_wp", 0) > 0 and not d.get("dp_trip_done"):
            p.button(T("dp.trip_cont"), lambda: self.dp_start(proto.DP_TRIP, cont=True), level="small",
                     tag="dp_trip_cont")
        cur = d.get("dp_set_kn") if sub == proto.DP_CRUISE else d.get("dp_trip_kn") if sub == proto.DP_TRIP else None
        p.text(T("dp.speed", kn="%.1f" % self.dp_kn), "small",
               colour=C["note_txt"] if cur is not None and abs(cur - self.dp_kn) > 0.05 else None, tag="dp_kn")
        p.button("−", lambda: self.dp_kn_step(-1), level="small", tag="dp_kn_minus")
        p.button("+", lambda: self.dp_kn_step(1), level="small", tag="dp_kn_plus")
        if in_dp and sub in (proto.DP_CRUISE, proto.DP_TRIP):
            p.button(T("ov.set"), self.dp_speed_send, "ok", level="small",
                     enabled=cur is None or abs(cur - self.dp_kn) > 0.05, tag="dp_kn_set")
        if not in_dp and d.get("dp_pending"):
            p.button(T("dp.stop"), self.dp_stop, "warn", level="small", tag="dp_stop")     # cancels the request
        p.br()
        if d.get("dp_pending"):
            p.text(T("dp.pending"), "small", colour=theme.CAUTION, bold=True, tag="dp_pending")
            p.br()
        if in_dp and sub in (proto.DP_SPOT, proto.DP_JOG):
            p.text(T("dp.jog", m="%.1f" % self.dp_jog_m()), "small", colour=C["dim"], tag="dp_jog_lbl")
            for k, lab in ((proto.DP_JOG_AHEAD, "▲"), (proto.DP_JOG_ASTERN, "▼"), (proto.DP_JOG_PORT, "◀"),
                           (proto.DP_JOG_STBD, "▶")):
                p.button(lab, lambda k=k: self.dp_jog(k), level="small", tag="dp_jog_%d" % k)
            p.text(self.dp_err_text(d), "small", colour=C["bad_txt"] if d.get("dp_offspot") else None, tag="dp_err")
            p.br()

    def panel_traffic(self, H):
        """Bottom-right (above the logo): AIS list + collision alerts."""
        rows = self.ais_rows_now()
        p = self.ov.panel("traffic", T("ov.traffic", n=len(rows)), "se")
        av = (self.gw_status or {}).get("avoid") or {}
        st = av.get("state") or "-"
        txt = T("ais.state", state=T("avoid." + st.replace(" ", "_")) if st != "-" else "-")
        if av.get("why"):
            txt += " - " + i18n.from_en(av["why"])
        p.text(txt, "label", tag="avoid", wrap=0.28,
               colour=C["bad_txt"] if st in ("AVOID", "DRIVER DECISION", "ALARM ONLY", "PROPOSE") else None)
        if av.get("proposal"):
            p.br()
            p.text(T("ais.proposal_on_driver", text=proposal_text(av["proposal"])), "label", colour=C["note_txt"],
                   tag="proposal", wrap=0.28)
            if self.single_op:
                p.br()
                p.button(T("drv.accept"), lambda: self.avoid_answer(None, True), "ok")
                p.button(T("drv.reject"), lambda: self.avoid_answer(None, False), "no")
        n = AIS_ROWS[0] if H < 720 else AIS_ROWS[1] if H < 900 else AIS_ROWS[2]
        if rows:
            def fv(v, fmt):
                return "-" if v is None else fmt % v
            trs, acts = [], []
            for r in rows[:n]:
                colr = C["bad_txt"] if r["level"] == "danger" else C["note_txt"] if r["level"] == "warn" else \
                    (C["faint"] if r.get("stale") else None)
                trs.append(([(r.get("name") or str(r["mmsi"]))[:20], fv(r.get("dist_nm"), "%.2f"),
                             fv(r.get("bearing"), "%03.0f"), fv(r["cpa_nm"], "%.2f"),
                             fv(r["tcpa_min"] if (r["tcpa_min"] or 0) >= 0 else None, "%.1f")], colr))
                acts.append(lambda m=r["mmsi"]: self.ais_open(m))
            p.br()
            p.table([T("ais.col_ship"), T("ais.col_dist"), T("ais.col_brg"), T("ais.col_cpa"), T("ais.col_tcpa")],
                    trs, ["l", "r", "r", "r", "r"], tag="ais", row_actions=acts)
            if len(rows) > n:
                p.br()
                p.text(T("ov.more_ships", n=len(rows) - n), "small", colour=C["dim"], action=lambda: self.open_tab(
                    "tab.ais"))
        p.br()
        p.text(T("ais.small_craft"), "small", colour=C["bad_txt"], wrap=0.28)

    def ais_open(self, mmsi):
        self.open_tab("tab.ais")
        if self.ais_tree.exists(str(mmsi)):
            self.ais_tree.selection_set(str(mmsi))
            self.ais_details()

    # --- texts for the panels ------------------------------------------------------------------------------
    def auto_text(self):
        d = self.t
        if not d:
            return T("cap.no_program")
        st = T("auto.%d" % d["auto_state"]) if d["auto_state"] < 4 else "?"
        if not d["prog_ok"]:
            txt = T("cap.no_program_boat")
        elif d["mode"] == proto.MODE_AUTO:
            txt = T("cap.auto_wp", n=d["auto_wp"] + 1, of=d["prog_n"], state=st)
            if d.get("prog_returning"):
                txt += "\n" + T("cap.returning")
            if d["speed_limited"]:
                txt += "\n" + T("cap.speed_limited")
        elif d["prog_suspended"]:
            txt = T("cap.prog_interrupted", n=d["auto_wp"] + 1)
        elif d["prog_finished"]:
            txt = T("cap.prog_finished")
        else:
            txt = T("cap.prog_stored", n=d["prog_n"])
        return txt

    def helm_mine(self):
        hid = (self.t or {}).get("helm")
        mine = (self.gw_status or {}).get("you")
        return bool(hid) and ((hid == mine) if mine else (hid == 1 and isinstance(self.link, Link)))

    def boat_push(self, d):
        """CLAUDE.md L: once this station holds HELM, it pushes the boat file's settings to the AURIX (RAM only):
        the generator cap gen_max_pct = min(file, the AURIX's own ceiling) - a v1 / invalid file pushes 0 =
        generator disabled. PARAM_SET is STANDBY only, so the push waits for STANDBY; a NACK re-arms it (5 s);
        a boat restart (t_ms back to 0) re-arms it too. Then the relay table (relay_push, PROTO_T_RELAY_CFG) and
        the DP thruster settings (thruster_push, PROTO_T_THRUSTER_CFG). Never from a copilot (it never holds
        HELM); the captain's own setting, so no ACCEPT; single-operator mode = its own confirmation."""
        if self.boat is None or not self.boat_ok or self.link is None:
            return                                  # only a confirmed boat (name + live camera) gets its settings
        if not self.helm_mine() or d.get("mode_name") != "STANDBY":
            return
        if not self.boat_pushed and time.time() >= self.boat_push_at:
            pid = next((k for k, p in self.params.items() if p.get("name") == "gen_max_pct"), None)
            if pid is None:
                return                              # the parameter list is not read yet
            st = boats.aurix_settings(self.boat, self.params[pid].get("max", boats.GEN_MAX_CEILING))
            v = st["gen_max_pct"]
            self.boat_pushed = True
            self.boat_push_seq = self._send_cmd(proto.T_PARAM_SET, proto.p_param_set(pid, v),
                                                TE("what.boat_gen", v="%g" % v, name=self.boat.name),
                                                confirm="self" if self.single_op else None)
            return
        self.relay_push(d)
        self.thruster_push(d)

    def thruster_push(self, d):
        """CLAUDE.md L / P (step 9): the boat file's DP thruster settings (PROTO_T_THRUSTER_CFG: max_pct, run_s,
        rest_s per thruster, jog_m, spot_alarm_m). RAM only on the AURIX - after every AURIX restart they are gone
        (TELEM2 group N "thr_hash" None) and are sent again. The same hash as the file = nothing to send; a
        different one is sent at most every THRUSTER_PUSH_S (HELM + STANDBY: boat_push; the AURIX refuses it
        outside STANDBY). No CONFIRM on the boat (thrust only ever comes with DP, which needs its own CONFIRM);
        a v1 / invalid file pushes "no thrusters". Single-operator mode: the own-confirmation question first,
        like the relay table."""
        if "thr_flags" not in d or time.time() < self.thr_push_at:
            return
        thr = boats.aurix_settings(self.boat)["thrusters"]
        payload = proto.p_thruster_cfg(thr)
        if d.get("thr_hash") == proto.crc16(payload):
            return
        self.thr_push_at = time.time() + THRUSTER_PUSH_S
        what = TE("what.boat_thrusters", name=self.boat.name, bow="%g" % thr["bow"]["max_pct"],
                  stern="%g" % thr["stern"]["max_pct"])
        if self.single_op:
            if self.cconf is None:                  # (one question at a time; asked again after THRUSTER_PUSH_S)
                self.captain_confirm(what, lambda: self._thr_push_send(payload, what, "self"))
            return
        self._thr_push_send(payload, what, None)

    def _thr_push_send(self, payload, what, confirm):
        self.thr_push_seq = self._send_cmd(proto.T_THRUSTER_CFG, payload, what, confirm=confirm)

    def relay_push(self, d):
        """CLAUDE.md M (step 6): the boat file's relay table (PROTO_T_RELAY_CFG). TELEM2 group M carries the
        hash of the table the AURIX uses: the same one = nothing to send; a different one is sent again every
        RELAY_PUSH_S while no CONFIRM is pending - the AURIX takes it only in STANDBY and only after the
        joystick CONFIRM (its stored table stays until then), so the captain sees the CONFIRM banner each time.
        A v1 file (no relays) pushes the EMPTY table (the file is the source: the stored table is then cleared,
        after the CONFIRM). HELM + STANDBY as the generator cap.
        Never while ANY CONFIRM is pending on the boat (confirm_kind: a power raise, AUTO start ...): the AURIX
        refuses the offer then, and the push must not get in the way of the user's own request. The automatic
        push never confirms itself: in single-operator mode the operator answers the usual own-confirmation
        question first (only then it goes, with confirm "self"); otherwise the captain's push is their own."""
        if d.get("relay_hash") is None or d.get("relay_pending") or time.time() < self.relay_push_at:
            return
        if d.get("relay_stop"):
            return                                  # step 13g: a relay stop / timer is pending - the AURIX refuses
                                                    # any new table (NACK 23): wait, never a question that must fail
        if d.get("confirm_kind"):
            return                                  # another CONFIRM waits on the boat: not now
        relays = [] if self.boat.error else self.boat.relays
        if proto.relay_table_hash(relays) == d["relay_hash"]:
            rs = (self.gw_status or {}).get("relays")
            if not (relays and isinstance(self.link, GatewayLink) and rs and rs.get("hash") == d["relay_hash"] and
                    not rs.get("known")):
                return
            # the AURIX has this table (from its data flash) but the gateway doesn't know its groups (a new
            # gateway SSD): send the SAME table once more - the AURIX takes it at once (no CONFIRM), the gateway
            # learns which relays are trim / extra (CLAUDE.md O; until then every relay = the captain's ACCEPT)
        self.relay_push_at = time.time() + RELAY_PUSH_S
        what = TE("what.boat_relays", n=len(relays), name=self.boat.name)
        payload = proto.p_relay_cfg(relays)
        if self.single_op:
            if self.cconf is None:                  # (one question at a time; asked again after RELAY_PUSH_S)
                self.captain_confirm(what, lambda: self._relay_push_send(payload, what, "self"))
            return
        self._relay_push_send(payload, what, None)

    def _relay_push_send(self, payload, what, confirm):
        self.relay_push_seq = self._send_cmd(proto.T_RELAY_CFG, payload, what, confirm=confirm)

    def helm_text(self):
        """Who holds HELM, in the screen language (the gateway's own name of it is English)."""
        hid = (self.t or {}).get("helm")
        if hid is None:
            return T("cap.helm_none")
        if hid == 0:
            name = T("cap.helm_nobody")
        elif hid == 1:
            name = T("cap.helm_usb")
        else:
            st = [s for s in (self.gw_status or {}).get("stations", []) if s.get("id") == hid]
            name = T("ov.station", user=st[0].get("user") or "?", where=T("cap.on_board") if st[0].get("local") else
                     T("cap.remote")) if st else T("cap.helm_station", n=hid)
        return T("cap.helm_you", who=name) if self.helm_mine() else T("cap.helm", who=name)

    def manual_here(self):
        """MANUAL (REMOTE MANUAL) / MANEUVER buttons on screen 2: only single-operator mode with the licence."""
        return isinstance(self.link, GatewayLink) and self.single_op and crewui.crew_of(self)[2]

    def manual_allowed(self):
        """Screen 1: REMOTE MANUAL / MANEUVER for the captain with the manual licence (or single-operator +
        licence). The gateway and the AURIX check it again (licence, captain, CONFIRM on the joystick)."""
        if not isinstance(self.link, GatewayLink):
            return False
        crew, _cap, lic = crewui.crew_of(self)
        return bool(lic) and (crew == "captain" or self.single_op)

    def route_nm(self):
        w = self.prog["wps"]
        return sum(math.hypot((b["lon"] - a["lon"]) * math.cos(math.radians(a["lat"])), b["lat"] - a["lat"]) * 60.0
                   for a, b in zip(w, w[1:]))

    # --- REMOTE MANUAL / MANEUVER requests (screen 1, or screen 2 in single-operator mode) ----------------------
    def remote_request(self):
        self.sender_on()
        self.cmd(proto.T_REMOTE, proto.p_remote(0), TE("what.remote_request"))

    def maneuver_request(self):
        self.sender_on()
        self.cmd(proto.T_MANEUVER, proto.p_remote(0), TE("what.maneuver_request"))

    def sender_on(self):
        """The shore joystick must be sent for REMOTE / MANEUVER (and for the shore CONFIRM)."""
        if isinstance(self.link, GatewayLink) and not self.rsend_var.get():
            self.rsend_var.set(True)
            self.remote_send_toggle()

    # --- editing on the map: route, geofence, max speed (CLAUDE.md D: the copilot's work) -------------------------
    def set_edit(self, what):
        if what == "fence" and self.mapwin.edit != "fence" and not self.fence_dirty:
            self.fence_draft = [tuple(p) for p in self.fence_live()]
        self.mapwin.set_edit(what)
        self.render(force=True)

    def map_add(self, what, lat, lon):
        if what == "route":
            if len(self.prog["wps"]) >= program.MAX_WP:
                return self.say(T("ov.route_full", n=program.MAX_WP), RED)
            self.prog["wps"].append({"lat": lat, "lon": lon, "kn": self.new_wp["kn"], "radius": self.new_wp["radius"],
                                     "action": 0})
            self.say(T("cap.wp_added", n=len(self.prog["wps"])))
        else:
            if len(self.fence_draft) >= 50:
                return self.say(T("ov.fence_full"), RED)
            self.fence_draft.append((lat, lon))
        self.map_edited(what)

    def map_delete(self, what, i):
        if what == "route" and i < len(self.prog["wps"]):
            del self.prog["wps"][i]
        elif what == "fence" and i < len(self.fence_draft):
            del self.fence_draft[i]
        self.map_edited(what)

    def map_edited(self, what):
        if what == "route":
            self.route_dirty = True
            self.prog_refresh()
        else:
            self.fence_dirty = True
        self.mapwin.redraw()
        self.render(force=True)

    def wp_speed(self, step):
        self.new_wp["kn"] = max(1.0, min(35.0, self.new_wp["kn"] + step))
        if self.mapwin.edit == "route":
            for w in self.prog["wps"]:
                w["kn"] = self.new_wp["kn"]
            self.route_dirty = bool(self.prog["wps"])
            self.prog_refresh()
        self.render(force=True)

    def route_send(self):
        """The route on the map -> the boat (a copilot's: after the captain's ACCEPT on screen 1)."""
        self.prog_upload()

    def maxkn_step(self, step):
        cur = self.params.get(P_MAX_LEG, {}).get("value")
        base = self.maxkn_edit if self.maxkn_edit is not None else (cur if cur is not None else 8.0)
        lo, hi = self.params.get(P_MAX_LEG, {}).get("min", 3.0), self.params.get(P_MAX_LEG, {}).get("max", 35.0)
        self.maxkn_edit = max(lo, min(hi, round(base + step)))
        self.render(force=True)

    def maxkn_send(self):
        if self.maxkn_edit is None:
            return
        v = float(self.maxkn_edit)
        self.cmd(proto.T_PARAM_SET, proto.p_param_set(P_MAX_LEG, v), TE("what.max_speed", v="%g" % v))
        self.maxkn_edit = None

    def fence_live(self):
        g = (self.gw_status or {}).get("geofence") or {}
        return [tuple(p) for p in g.get("poly") or []]

    def fence_send(self, poly=None):
        """The geofence -> the gateway (it checks every route against it; a copilot's change waits for the
        captain's ACCEPT on screen 1)."""
        if not isinstance(self.link, GatewayLink):
            return self.say(T("ov.fence_only_gw"), RED)
        poly = self.fence_draft if poly is None else poly
        if poly and len(poly) < 3:
            return self.say(T("ov.fence_3"), RED)
        what = TE("what.geofence", n=len(poly)) if poly else TE("what.geofence_clear")
        msg = {"t": "geofence", "poly": [[round(a, 7), round(b, 7)] for a, b in poly], "what": what}
        if self.single_op:
            return self.captain_confirm(what, lambda: self._fence_go(dict(msg, confirm="self"), what))
        self._fence_go(msg, what)

    def _fence_go(self, msg, what):
        self.link.send_msg(msg)
        self.event("> " + i18n.from_en(what))
        if crewui.crew_of(self)[0] == "copilot":
            self.say(T("cap.wait_captain", what=i18n.from_en(what)), ORANGE)
        self.decisions.append({"what": what, "seq": "fence", "state": "wait", "t": time.time(), "why": ""})

    def fence_clear(self):
        self.fence_draft = []
        if self.fence_live():
            self.fence_send([])
        self.fence_dirty = False
        if self.mapwin.edit == "fence":
            self.mapwin.set_edit("fence")
        self.mapwin.redraw()

    def on_fence_ack(self, d):
        for x in reversed(self.decisions):
            if x["seq"] == "fence" and x["state"] == "wait":
                x["state"] = "ok" if d.get("ok") else ("no" if d.get("code") == 110 else "fail")
                x["why"] = "" if d.get("ok") else alerts.result_text({"result": d.get("code", 0), "ok": False})
                x["t"] = time.time()
                break
        if d.get("ok"):
            self.fence_dirty = False
            if self.mapwin.edit == "fence":
                self.mapwin.set_edit("fence")
        code = d.get("code") or 0
        txt = T("ov.fence_msg", text=alerts.result_text({"result": code, "ok": False}) if code and not d.get("ok")
                else i18n.from_en(d.get("text", "")))
        self.say(txt, GREEN if d.get("ok") else RED)
        self.event(txt)
        self.render(force=True)

    def track_decision(self, seq, what):
        """A copilot's driving action: WAITING until the captain answers (the ack)."""
        self.decisions.append({"what": what, "seq": seq, "state": "wait", "t": time.time(), "why": ""})

    def on_decision_ack(self, d):
        for x in self.decisions:
            if x["seq"] == d["seq"] and x["state"] == "wait":
                r = d["result"]
                x["state"] = "ok" if d["ok"] else ("no" if r == 110 else "fail")
                x["why"] = "" if d["ok"] or r == 110 else alerts.result_text(d)
                x["t"] = time.time()

    def choose_boat(self, bt):
        """A boat chosen in screen 2's dropdown: log in + connect (the confirm step follows, CLAUDE.md F / H)."""
        self.boat_pick = bt
        if self.boatsel is None:
            self.boatsel = crewui.BoatSelector(self)
        else:
            self.boatsel.w.lift()
        self.boatsel.select(bt)


def main():
    import argparse
    import splash
    ap = argparse.ArgumentParser(description="Trygons C2 ground station")
    ap.add_argument("address", nargs="?", help="tcp:host:port, COM5 or gw:host:port")
    ap.add_argument("--login", help="user:password for the gateway (else a login window)")
    ap.add_argument("--driver", action="store_true", help="open screen 1 (driving) too")
    ap.add_argument("--charts", help="chart folder (default: <repo>/charts)")
    ap.add_argument("--layout", help="window-place file (default: gs_layout.json here)")
    ap.add_argument("--place", choices=("shore", "onboard"), help="where this station is (window title)")
    ap.add_argument("--title", help=argparse.SUPPRESS)        # old: free text, replaced by --place
    ap.add_argument("--cams", help="camera preset to start with (e.g. Full)")
    ap.add_argument("--no-splash", action="store_true", help="no start-up screen (the demo shows its own)")
    ap.add_argument("--ready-file", help="written once the window is open (the demo's start-up screen closes)")
    a = ap.parse_args()
    login = tuple(a.login.split(":", 1)) if a.login and ":" in a.login else None
    theme.set_app_id()
    root = tk.Tk()
    root.withdraw()                     # shown only once it is built: the start-up screen never covers it
    place = a.place or ({"shore": "shore", "on board": "onboard"}.get((a.title or "").split(" (")[0].lower()))
    sp = None
    if not a.no_splash:
        sp = splash.Splash(root, what="splash.what_station", steps=["splash.p_station"])
        sp.step("splash.p_station")
    try:
        App(root, a.address, login=login, charts_dir=a.charts, open_driver=a.driver, layout=a.layout, cams=a.cams,
            place=place)
    except Exception as e:              # shown on the start-up screen (never a silent hang), then the traceback
        if sp is None:
            raise
        sp.fail("%s: %s" % (type(e).__name__, e))
        root.wait_window(sp.w)
        raise
    if sp is not None:
        sp.close()
    root.deiconify()
    if a.ready_file:
        def ready():
            try:
                with open(a.ready_file, "w", encoding="utf-8") as f:
                    f.write("%.3f\n" % time.time())
            except OSError:
                pass
        root.after(300, ready)
    root.mainloop()


if __name__ == "__main__":
    main()
