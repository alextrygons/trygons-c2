"""Trygons C2 ground station (tkinter).

    python app.py                      choose the port in the window
    python app.py tcp:127.0.0.1:5760   connect to the simulator at once
    python app.py COM5                 connect to the board at once

Big, high-contrast layout for use in sunlight. Everything is drawn locally
(no internet needed): map, track, plot.
"""
import math
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import program
import proto
from link import Link, list_ports

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
               "TRAINING": "#a06000"}
PLOT_S = 120.0              # seconds shown in the plot


def btn(parent, text, cmd, bg=PANEL, fg=FG, font=F_MED, width=None):
    b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg, font=font, activebackground="#c0c0c0",
                  relief="raised", bd=3, padx=8, pady=4)
    if width:
        b.configure(width=width)
    return b


class App:
    def __init__(self, root, address=None):
        self.root = root
        self.link = None
        self.t = None                   # latest telemetry
        self.t_rx = 0.0
        self.hist = []                  # (t_s, heading, target, outboard)
        self.track = []                 # (lat, lon)
        self.origin = None
        self.map_m_per_px = 2.0
        self.params = {}
        self.pending = {}               # seq -> description
        self.prog = program.new_program()   # program in the editor
        self.pdlg = None                # power raise dialog
        self.prog_dl = []               # PROG_WP_VALUE frames being downloaded
        self.last_beep = 0.0

        root.title("Trygons C2 ground station")
        root.configure(bg=BG)
        root.geometry("1500x900")
        try:
            root.state("zoomed")        # maximised (Windows)
        except tk.TclError:
            pass
        style = ttk.Style()
        style.configure("Treeview", font=F_TXT, rowheight=26)
        style.configure("Treeview.Heading", font=F_MED)
        style.configure("TNotebook.Tab", font=F_MED, padding=(14, 6))

        self._build_top()
        self._build_banners()
        body = tk.Frame(root, bg=BG)
        body.pack(fill="both", expand=True)
        self._build_values(body)
        self._build_controls(body)
        self._build_tabs(body)
        self.msg = tk.Label(root, text="Not connected", font=F_MED, bg="#202020", fg="#ffffff", anchor="w")
        self.msg.pack(fill="x", side="bottom")

        if address:
            self.port_var.set(address)
            root.after(200, self.connect)
        root.after(50, self.poll)
        root.after(500, self.alarm_tick)
        root.protocol("WM_DELETE_WINDOW", self.quit)

    # --- layout ------------------------------------------------------------------
    def _build_top(self):
        top = tk.Frame(self.root, bg=PANEL)
        top.pack(fill="x")
        tk.Label(top, text="Connection:", font=F_MED, bg=PANEL).pack(side="left", padx=6)
        self.port_var = tk.StringVar(value="tcp:127.0.0.1:5760")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, values=list_ports(), width=24, font=F_MED)
        self.port_box.pack(side="left", padx=4)
        btn(top, "Refresh", lambda: self.port_box.configure(values=list_ports()), font=F_TXT).pack(side="left")
        self.conn_btn = btn(top, "Connect", self.connect, bg="#b0ffb0")
        self.conn_btn.pack(side="left", padx=6)
        self.link_lbl = tk.Label(top, text="link: -", font=F_MED, bg=PANEL)
        self.link_lbl.pack(side="left", padx=12)
        self.log_lbl = tk.Label(top, text="", font=F_TXT, bg=PANEL)
        self.log_lbl.pack(side="right", padx=8)

    def _build_banners(self):
        self.banners = tk.Frame(self.root, bg=BG)       # empty = zero height
        self.banners.pack(fill="x")
        self.banners.columnconfigure(0, weight=1)
        self.joy_banner = tk.Label(self.banners, text="JOYSTICK IN CONTROL", font=F_HUGE, bg=ORANGE, fg=FG)
        self.alarm_banner = tk.Label(self.banners, text="", font=F_MED, bg=RED, fg="#ffffff")
        self.alarm_ack = btn(self.banners, "ACK ALARM\n(STANDBY)",
                             lambda: self.cmd(proto.T_ALARM_ACK, b"", "alarm ack"), font=F_TXT)

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
        self.power_lbl = tk.Label(f, text="POWER LIMIT --", font=F_BIG, bg="#d0ffd0", fg=FG)
        self.power_lbl.pack(fill="x")
        pw = tk.Frame(f, bg=BG)
        pw.pack(fill="x", pady=(0, 6))
        btn(pw, "Raise fwd", lambda: self.power_dialog(0), font=F_TXT).pack(side="left", expand=True, fill="x")
        btn(pw, "Raise rev", lambda: self.power_dialog(1), font=F_TXT).pack(side="left", expand=True, fill="x")
        btn(pw, "Lower now", lambda: self.cmd(proto.T_POWER_LOWER, b"", "power lower"), bg="#b0ffb0",
            font=F_TXT).pack(side="left", expand=True, fill="x")
        self.vals = {}
        rows = [("heading", "Heading"), ("target", "Target"), ("yaw", "Yaw rate"), ("outboard", "Outboard"),
                ("throttle", "Throttle"), ("gear", "Gear"), ("gps", "GPS"), ("pos", "Position"),
                ("sog", "SOG / COG"), ("src", "Pos. source"), ("unc", "Uncertainty"), ("drift", "Set / drift"),
                ("current", "Motor current"), ("cruise", "Speed set"), ("wp", "Waypoint"), ("train", "Training")]
        grid = tk.Frame(f, bg=BG)
        grid.pack(fill="x")
        for i, (k, name) in enumerate(rows):
            tk.Label(grid, text=name, font=F_TXT, bg=BG, anchor="w").grid(row=i, column=0, sticky="w", padx=4)
            big = k in ("heading", "target", "outboard", "throttle", "gear")
            small = k in ("pos", "src", "drift", "wp", "train", "unc", "current", "cruise", "sog")
            v = tk.Label(grid, text="-", font=F_BIG if big else F_TXT if small else F_MED, bg=BG, anchor="w")
            v.grid(row=i, column=1, sticky="w", padx=4)
            self.vals[k] = v

    def _build_controls(self, body):
        f = tk.Frame(body, bg=BG, bd=2, relief="groove")
        f.pack(side="left", fill="y", padx=4, pady=4)

        tk.Label(f, text="Mode (lit = active)", font=F_TXT, bg=BG).pack(anchor="w")
        m = tk.Frame(f, bg=BG)
        m.pack(fill="x")
        btn(m, "STANDBY", lambda: self.mode(proto.MODE_STANDBY), bg=RED, fg="#ffffff", font=F_BIG).grid(
            row=0, column=0, columnspan=3, sticky="ew", pady=2)
        self.mode_btns = {}
        for i, (name, cmd) in enumerate([("MANUAL", self.info_manual), ("HDG_LOCK", self.info_hdg),
                                         ("AUTO", self.auto_start)]):
            b = btn(m, name.replace("_", " ") if name != "AUTO" else "AUTO\nSTART", cmd, bg=PANEL, font=F_MED)
            b.grid(row=1, column=i, sticky="nsew", padx=1, pady=1)
            self.mode_btns[name] = b
        for i in range(3):
            m.columnconfigure(i, weight=1)

        tk.Label(f, text="AUTO program (start / resume need joystick CONFIRM)", font=F_TXT, bg=BG).pack(
            anchor="w", pady=(6, 0))
        a = tk.Frame(f, bg=BG)
        a.pack(fill="x")
        btn(a, "Pause", lambda: self.cmd(proto.T_AUTO_PAUSE, b"", "AUTO pause"), font=F_TXT).pack(side="left")
        btn(a, "Resume", lambda: self.cmd(proto.T_AUTO_RESUME, b"", "AUTO resume"), font=F_TXT).pack(side="left",
                                                                                                    padx=2)
        btn(a, "Skip waypoint", lambda: self.cmd(proto.T_AUTO_SKIP, b"", "AUTO skip"), font=F_TXT).pack(side="left")
        self.auto_lbl = tk.Label(f, text="-", font=F_TXT, bg=BG, justify="left", anchor="w")
        self.auto_lbl.pack(fill="x")

        tk.Label(f, text="Heading (HDG LOCK)", font=F_TXT, bg=BG).pack(anchor="w", pady=(6, 0))
        h = tk.Frame(f, bg=BG)
        h.pack()
        for d in (-10, -1, 1, 10):
            btn(h, "%+d" % d, lambda d=d: self.cmd(proto.T_HEADING_ADJ, proto.p_heading(d), "heading %+d" % d),
                font=F_BIG, width=3).pack(side="left", padx=2)

        tk.Label(f, text="Sea state", font=F_TXT, bg=BG).pack(anchor="w", pady=(6, 0))
        cs = tk.Frame(f, bg=BG)
        cs.pack(fill="x")
        self.sea_var = tk.IntVar(value=2)
        for sv in range(1, 6):
            tk.Radiobutton(cs, text=str(sv), variable=self.sea_var, value=sv, font=F_MED, bg=BG, indicatoron=False,
                           width=2, command=lambda: self.cmd(proto.T_SEA_STATE, proto.p_sea_state(self.sea_var.get()),
                                                             "sea state %d" % self.sea_var.get())).pack(side="left")

        tk.Label(f, text="Training", font=F_TXT, bg=BG).pack(anchor="w", pady=(6, 0))
        tr = tk.Frame(f, bg=BG)
        tr.pack(fill="x")
        btn(tr, "Start", lambda: self.cmd(proto.T_TRAINING, proto.p_training(0), "training start"),
            font=F_TXT).pack(side="left")
        tk.Label(tr, text="full angle", font=F_TXT, bg=BG).pack(side="left", padx=(6, 0))
        self.full_var = tk.StringVar(value="0")
        tk.Entry(tr, textvariable=self.full_var, font=F_MED, width=4).pack(side="left")
        btn(tr, "Accept", self.train_accept, bg="#b0ffb0", font=F_TXT).pack(side="left", padx=2)
        btn(tr, "Reject", lambda: self.cmd(proto.T_TRAINING, proto.p_training(2), "training reject"),
            bg="#ffb0b0", font=F_TXT).pack(side="left")

        # simulator only: the panel buttons and the joystick CONFIRM
        self.sim_frame = tk.Frame(f, bg="#fff4c0")
        tk.Label(self.sim_frame, text="Simulator: panel / joystick", font=F_TXT, bg="#fff4c0").pack(anchor="w")
        sf = tk.Frame(self.sim_frame, bg="#fff4c0")
        sf.pack(fill="x")
        btn(sf, "HDG LOCK", lambda: self.cmd(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK),
                                             "sim HDG LOCK button"), font=F_TXT).pack(side="left")
        btn(sf, "STANDBY", lambda: self.cmd(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY),
                                            "sim STANDBY button"), font=F_TXT).pack(side="left", padx=2)
        btn(sf, "CONFIRM", self.sim_confirm, font=F_TXT).pack(side="left")

    def _build_joystick(self, parent):
        j = tk.Frame(parent, bg=BG, bd=2, relief="groove")
        j.pack(fill="x", pady=(0, 4))
        tk.Label(j, text="Joystick", font=F_MED, bg=BG).pack(side="left", anchor="n", padx=4)
        self.joy_cv = tk.Canvas(j, width=130, height=130, bg="#f8f8f8", highlightthickness=2,
                                highlightbackground=FG)
        self.joy_cv.pack(side="left", pady=2)
        jr = tk.Frame(j, bg=BG)
        jr.pack(side="left", padx=8, fill="y")
        self.joy_status = tk.Label(jr, text="-", font=F_MED, bg=BG, justify="left")
        self.joy_status.pack(anchor="w")
        self.joy_btns = tk.Label(jr, text="TAKE  AUTO", font=F_MED, bg=BG)
        self.joy_btns.pack(anchor="w")
        jb = tk.Frame(j, bg=BG)
        jb.pack(side="left", padx=4)
        btn(jb, "Calibrate start", lambda: self.cmd(proto.T_JOY_CAL, proto.p_joy_cal(0), "calibration start"),
            font=F_TXT).pack(fill="x")
        btn(jb, "Calibrate finish", lambda: self.cmd(proto.T_JOY_CAL, proto.p_joy_cal(1), "calibration finish"),
            font=F_TXT).pack(fill="x")

    def _build_tabs(self, body):
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        self._build_joystick(right)
        nb = ttk.Notebook(right)
        nb.pack(fill="both", expand=True)

        mp = tk.Frame(nb, bg=BG)
        nb.add(mp, text="Map")
        mt = tk.Frame(mp, bg=BG)
        mt.pack(fill="x")
        btn(mt, "Zoom +", lambda: self.zoom(0.5), font=F_TXT).pack(side="left")
        btn(mt, "Zoom -", lambda: self.zoom(2.0), font=F_TXT).pack(side="left")
        btn(mt, "Clear track", lambda: self.track.clear(), font=F_TXT).pack(side="left")
        self.add_on_click = tk.BooleanVar(value=False)
        tk.Checkbutton(mt, text="Click adds a program waypoint", variable=self.add_on_click, font=F_TXT,
                       bg=BG).pack(side="left", padx=8)
        tk.Label(mt, text="(north up, centred on the boat)", font=F_TXT, bg=BG).pack(side="left")
        self.map = tk.Canvas(mp, bg="#d8ecff", highlightthickness=0)
        self.map.pack(fill="both", expand=True)
        self.map.bind("<Button-1>", self.map_click)

        pg = tk.Frame(nb, bg=BG)
        nb.add(pg, text="Program")
        self._build_program(pg)

        pl = tk.Frame(nb, bg=BG)
        nb.add(pl, text="Plot")
        self.plot = tk.Canvas(pl, bg=BG, highlightthickness=0)
        self.plot.pack(fill="both", expand=True)

        cf = tk.Frame(nb, bg=BG)
        nb.add(cf, text="Config")
        ct = tk.Frame(cf, bg=BG)
        ct.pack(fill="x")
        btn(ct, "Read all", self.read_params, font=F_TXT).pack(side="left")
        tk.Label(ct, text="new value", font=F_TXT, bg=BG).pack(side="left", padx=(12, 2))
        self.pval_var = tk.StringVar()
        tk.Entry(ct, textvariable=self.pval_var, font=F_MED, width=10).pack(side="left")
        btn(ct, "Write selected", self.write_param, font=F_TXT).pack(side="left", padx=4)
        tk.Label(ct, text="(STANDBY only; RAM only, lost at power-off)", font=F_TXT, bg=BG).pack(side="left")
        self.ptree = ttk.Treeview(cf, columns=("value", "min", "max"), height=12)
        for c, w in (("#0", 220), ("value", 140), ("min", 120), ("max", 120)):
            self.ptree.column(c, width=w)
        self.ptree.heading("#0", text="parameter")
        for c in ("value", "min", "max"):
            self.ptree.heading(c, text=c)
        self.ptree.pack(fill="both", expand=True)

        sy = tk.Frame(nb, bg=BG)
        nb.add(sy, text="System")
        tk.Label(sy, text="CPU time per cycle (min / average / max of the last window) and overruns",
                 font=F_MED, bg=BG).pack(anchor="w", pady=6)
        self.sys_lbl = tk.Label(sy, text="-", font=("Consolas", 14), bg=BG, justify="left", anchor="w")
        self.sys_lbl.pack(anchor="w", padx=8)

        lg = tk.Frame(nb, bg=BG)
        nb.add(lg, text="Events")
        self.events = tk.Listbox(lg, font=F_TXT)
        self.events.pack(fill="both", expand=True)

    def _build_program(self, pg):
        top = tk.Frame(pg, bg=BG)
        top.pack(fill="x")
        for text, fn in (("Upload to boat", self.prog_upload), ("Download", self.prog_download),
                         ("Save file", self.prog_save), ("Load file", self.prog_load)):
            btn(top, text, fn, font=F_TXT).pack(side="left", padx=1)
        tk.Label(top, text="  at the end:", font=F_TXT, bg=BG).pack(side="left")
        self.final_var = tk.StringVar(value=proto.PROG_END[0])
        ttk.Combobox(top, textvariable=self.final_var, values=proto.PROG_END, width=18, state="readonly",
                     font=F_TXT).pack(side="left")
        self.pg_tree = ttk.Treeview(pg, columns=("lat", "lon", "kn", "radius", "action"), height=10)
        for c, w in (("#0", 50), ("lat", 140), ("lon", 140), ("kn", 70), ("radius", 80), ("action", 100)):
            self.pg_tree.column(c, width=w)
            self.pg_tree.heading(c, text="#" if c == "#0" else c)
        self.pg_tree.pack(fill="both", expand=True)
        self.pg_tree.bind("<<TreeviewSelect>>", self.prog_select)
        ed = tk.Frame(pg, bg=BG)
        ed.pack(fill="x")
        self.pg_vars = {}
        for k, w in (("lat", 11), ("lon", 11), ("kn", 5), ("radius", 5)):
            tk.Label(ed, text=k, font=F_TXT, bg=BG).pack(side="left")
            v = tk.StringVar()
            tk.Entry(ed, textvariable=v, font=F_MED, width=w).pack(side="left", padx=(0, 4))
            self.pg_vars[k] = v
        self.pg_vars["radius"].set("25")
        self.pg_vars["kn"].set("8")
        self.pg_act = tk.StringVar(value=proto.WP_ACTIONS[0])
        ttk.Combobox(ed, textvariable=self.pg_act, values=proto.WP_ACTIONS, width=8, state="readonly",
                     font=F_TXT).pack(side="left")
        bt = tk.Frame(pg, bg=BG)
        bt.pack(fill="x")
        for text, fn in (("Add", self.prog_add), ("Change selected", self.prog_change), ("Delete", self.prog_delete),
                         ("Up", lambda: self.prog_move(-1)), ("Down", lambda: self.prog_move(1)),
                         ("Clear", self.prog_clear)):
            btn(bt, text, fn, font=F_TXT).pack(side="left", padx=1)

    # --- program editor ----------------------------------------------------------
    def prog_refresh(self):
        self.pg_tree.delete(*self.pg_tree.get_children())
        for i, w in enumerate(self.prog["wps"]):
            self.pg_tree.insert("", "end", iid=str(i), text=str(i + 1),
                                values=("%.6f" % w["lat"], "%.6f" % w["lon"], "%.1f" % w["kn"], "%.0f" % w["radius"],
                                        proto.WP_ACTIONS[w["action"]]))
        self.final_var.set(proto.PROG_END[self.prog["final"]])

    def prog_entry(self):
        try:
            return {"lat": float(self.pg_vars["lat"].get()), "lon": float(self.pg_vars["lon"].get()),
                    "kn": float(self.pg_vars["kn"].get()), "radius": float(self.pg_vars["radius"].get()),
                    "action": proto.WP_ACTIONS.index(self.pg_act.get())}
        except ValueError:
            self.say("Waypoint: lat, lon, kn and radius must be numbers", RED)
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
        self.pg_act.set(proto.WP_ACTIONS[w["action"]])

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
        self.prog["final"] = proto.PROG_END.index(self.final_var.get())
        problems = program.check(self.prog)
        if problems:
            return self.say("Program: " + "; ".join(problems[:3]), RED)
        if not self.link:
            return self.say("Not connected", RED)
        frames = program.upload_frames(self.prog)
        for t, p in frames[:-1]:
            self.link.send(t, p)
        self.cmd(frames[-1][0], frames[-1][1], "program upload (%d waypoints)" % len(self.prog["wps"]))

    def prog_download(self):
        n = self.t["prog_n"] if self.t else 0
        if not self.link or n == 0:
            return self.say("No program on the boat", RED)
        self.prog_dl = []
        for i in range(n):
            self.link.send(proto.T_PROG_GET, struct.pack("<B", i))

    def prog_save(self):
        from tkinter import filedialog
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("program", "*.json")])
        if path:
            self.prog["final"] = proto.PROG_END.index(self.final_var.get())
            program.save(self.prog, path)
            self.say("Program saved: " + path)

    def prog_load(self):
        from tkinter import filedialog
        path = filedialog.askopenfilename(filetypes=[("program", "*.json")])
        if path:
            self.prog = program.load(path)
            self.prog_refresh()

    # --- actions -----------------------------------------------------------------
    def connect(self):
        if self.link:
            self.link.stop()
            self.link = None
            self.conn_btn.configure(text="Connect", bg="#b0ffb0")
            self.say("Disconnected")
            return
        try:
            self.link = Link(self.port_var.get().strip())
            self.link.start()
        except Exception as e:
            self.link = None
            self.say("Cannot connect: %s" % e, RED)
            return
        self.conn_btn.configure(text="Disconnect", bg="#ffb0b0")
        if self.port_var.get().startswith("tcp:"):
            self.sim_frame.pack(fill="x", pady=(8, 0))
        else:
            self.sim_frame.pack_forget()
        self.log_lbl.configure(text="log: " + self.link.logger.path)
        self.root.after(500, self.read_params)

    def cmd(self, ftype, payload, what):
        if not self.link:
            self.say("Not connected", RED)
            return
        seq = self.link.send(ftype, payload)
        self.pending[seq] = what
        self.event("> " + what)

    def mode(self, m):
        self.cmd(proto.T_SET_MODE, proto.p_set_mode(m), "mode " + proto.MODES[m])

    # --- power limit raise ------------------------------------------------------
    def power_dialog(self, direction):
        """Ask the boat for a code, show it, the user types it back, then the
        joystick CONFIRM within 10 s does one step up."""
        if not self.link:
            return self.say("Not connected", RED)
        if self.pdlg is not None:
            self.pdlg.destroy()
        self.cmd(proto.T_POWER_CODE, proto.p_power_code(direction), "power code (%s)" % ("fwd", "rev")[direction])
        d = tk.Toplevel(self.root, bg=BG)
        d.title("Raise the power limit")
        d.transient(self.root)
        self.pdlg = d
        self.pdlg_dir = direction
        tk.Label(d, text="Raise the %s power limit ONE step" % ("FORWARD", "REVERSE")[direction], font=F_BIG,
                 bg=BG).pack(padx=20, pady=(14, 4))
        tk.Label(d, text="1. Type the code shown by the boat:", font=F_MED, bg=BG).pack(anchor="w", padx=20)
        self.pdlg_code = tk.Label(d, text="....", font=("Consolas", 40, "bold"), bg="#ffffe0")
        self.pdlg_code.pack(padx=20, pady=6)
        self.pdlg_var = tk.StringVar()
        e = tk.Entry(d, textvariable=self.pdlg_var, font=("Consolas", 32, "bold"), width=6, justify="center")
        e.pack(pady=4)
        e.focus_set()
        tk.Label(d, text="2. Then press CONFIRM on the joystick within 10 s (stick centred).", font=F_MED,
                 bg=BG).pack(anchor="w", padx=20, pady=(8, 4))
        b = tk.Frame(d, bg=BG)
        b.pack(pady=12)
        btn(b, "Send code", self.power_send, bg="#ffd000").pack(side="left", padx=6)
        btn(b, "Cancel", self.power_close).pack(side="left", padx=6)
        e.bind("<Return>", lambda _ev: self.power_send())

    def power_send(self):
        try:
            code = int(self.pdlg_var.get())
        except ValueError:
            return self.say("Type the 4-digit code", RED)
        self.cmd(proto.T_POWER_RAISE, proto.p_power_raise(self.pdlg_dir, code), "power raise (code %d)" % code)
        self.power_close()

    def power_close(self):
        if self.pdlg is not None:
            self.pdlg.destroy()
            self.pdlg = None

    def info_manual(self):
        self.say("MANUAL: move the joystick or press TAKE (the PC can't select it)")

    def info_hdg(self):
        self.say("HDG LOCK: press the HDG LOCK button on the panel (the PC can't select it)")

    def auto_start(self):
        self.cmd(proto.T_AUTO_START, b"", "AUTO start")

    def sim_confirm(self):
        """Simulator: press and release the joystick CONFIRM (stick centred)."""
        if not self.link:
            return
        self.link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        self.root.after(150, lambda: self.link and self.link.send(
            proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0)))

    def train_accept(self):
        try:
            full = float(self.full_var.get())
        except ValueError:
            return self.say("Full angle: enter a number (0 = unknown)", RED)
        self.cmd(proto.T_TRAINING, proto.p_training(1, full), "training accept")

    def read_params(self):
        if self.link:
            for pid in range(32):
                self.link.send(proto.T_PARAM_GET, proto.p_param_get(pid))

    def write_param(self):
        sel = self.ptree.selection()
        if not sel:
            return self.say("Select a parameter first", RED)
        try:
            v = float(self.pval_var.get())
        except ValueError:
            return self.say("Enter a number", RED)
        pid = int(sel[0])
        self.cmd(proto.T_PARAM_SET, proto.p_param_set(pid, v), "set %s = %g" % (self.params[pid]["name"], v))

    def zoom(self, k):
        self.map_m_per_px = min(50.0, max(0.1, self.map_m_per_px * k))

    def map_click(self, ev):
        if not self.t or not self.add_on_click.get():
            return
        w, h = self.map.winfo_width(), self.map.winfo_height()
        e = (ev.x - w / 2) * self.map_m_per_px
        n = (h / 2 - ev.y) * self.map_m_per_px
        lat = self.t["est_lat"] + n / 111320.0
        lon = self.t["est_lon"] + e / (111320.0 * math.cos(math.radians(self.t["est_lat"])))
        self.pg_vars["lat"].set("%.6f" % lat)
        self.pg_vars["lon"].set("%.6f" % lon)
        self.prog_add()
        self.say("Waypoint %d added (Program tab), not uploaded yet" % len(self.prog["wps"]))

    # --- feedback ----------------------------------------------------------------
    def say(self, text, color="#202020"):
        self.msg.configure(text=text, bg=color)

    def event(self, text):
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
                        txt = "%s: %s" % (what or d["cmd_name"], d["text"])
                        self.say(txt, GREEN if d["ok"] else RED)
                        self.event(("OK   " if d["ok"] else "NACK ") + txt)
                elif kind == "param":
                    self.on_param(d)
                elif kind == "prog_wp":
                    self.prog_dl.append(d)
                    if len(self.prog_dl) >= d["n"]:
                        self.prog = program.from_wp_values(self.prog_dl)
                        self.prog_refresh()
                        self.say("Program downloaded (%d waypoints)" % len(self.prog["wps"]), GREEN)
                elif kind == "prog_error":
                    where = "" if d["index"] == 255 else " at waypoint %d" % (d["index"] + 1)
                    self.say("PROGRAM REFUSED: %s%s" % (d["text"], where), RED)
                    self.event("program refused: %s%s" % (d["text"], where))
                elif kind == "status":
                    self.say(d, RED if "DISCONNECTED" in d else "#202020")
                    self.event(d)
            age = time.time() - self.t_rx if self.t_rx else 99
            ok = self.link.connected and age < 1.0
            self.link_lbl.configure(text="link: OK" if ok else "link: NO DATA", fg=GREEN if ok else RED)
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
            self.event("mode %s -> %s" % (prev["mode_name"], d["mode_name"]))
        if prev and prev["alarm_bits"] != d["alarm_bits"]:
            new = d["alarm_bits"] & ~prev["alarm_bits"]
            if new:
                self.event("ALARM: " + ", ".join(proto.alarm_texts(new)))
        # draw at most 10 times per second
        if not prev or d["t_ms"] - getattr(self, "_drawn", 0) >= 100 or d["t_ms"] < getattr(self, "_drawn", 0):
            self._drawn = d["t_ms"]
            self.draw()

    # --- drawing -----------------------------------------------------------------
    def draw(self):
        d = self.t
        mode = d["mode_name"]
        self.mode_lbl.configure(text=mode, bg=MODE_COLORS.get(mode, "#606060"))
        v = self.vals
        v["heading"].configure(text="%05.1f°" % d["heading"] if d["compass_ok"] else "NO COMPASS",
                               fg=FG if d["compass_ok"] else RED)
        v["target"].configure(text="%05.1f°" % d["target"])
        v["yaw"].configure(text="%+.1f °/s" % d["yaw_rate"])
        v["outboard"].configure(text="%+.1f°" % d["outboard"])
        v["throttle"].configure(text="%.0f %%" % d["throttle"] if d["throttle_demand"] <= d["throttle"] + 0.5
                                else "%.0f %% (asked %.0f)" % (d["throttle"], d["throttle_demand"]))
        self.power_lbl.configure(text="POWER LIMIT %.0f %% / R %.0f %%" % (d["power_fwd"], d["power_rev"]),
                                 bg="#ffc080" if d["power_raised"] else "#d0ffd0")
        if self.pdlg is not None:
            ok = d["power_code"] and d["power_code_dir"] == self.pdlg_dir
            self.pdlg_code.configure(text="%04d" % d["power_code"] if ok else "....")
        v["gear"].configure(text=proto.GEARS[d["gear"]] if d["gear"] < 3 else "?",
                            fg={0: FG, 1: GREEN, 2: RED}.get(d["gear"], FG))
        v["gps"].configure(text="FIX" if d["gps_fix"] else "NO FIX", fg=GREEN if d["gps_fix"] else RED)
        v["pos"].configure(text="%.6f  %.6f" % (d["est_lat"], d["est_lon"]))
        v["sog"].configure(text="%.1f kn  %03.0f°" % (d["est_sog"], d["est_cog"]))
        src = proto.POS_SOURCES[d["pos_source"]] if d["pos_source"] < 4 else "?"
        if d["pos_source"] in (2, 3):
            src += " (%s, %.0f s)" % (proto.DR_REASONS[min(d["dr_reason"], 6)], d["dr_time_s"])
        v["src"].configure(text=src, fg=FG if d["pos_source"] == 1 else RED)
        v["unc"].configure(text="± %.0f m" % d["unc_m"])
        v["drift"].configure(text="%03.0f° %.2f kn (%.0f %%)" % (d["set_deg"], d["drift_kn"], 100 * d["current_conf"]))
        v["current"].configure(text="%.1f A" % d["motor_current"] if d["motor_current"] >= 0 else "no sensor")
        v["cruise"].configure(text="%.1f kn" % d["speed_set_kn"] if d["mode"] == proto.MODE_AUTO else "-")
        v["wp"].configure(text="%d/%d  %.1f kn" % (d["auto_wp"] + 1, d["prog_n"], d["wp_kn"]) if d["prog_ok"]
                          else "no program")
        for name, b in self.mode_btns.items():
            on = mode == name
            b.configure(bg=MODE_COLORS[name] if on else PANEL, fg="#ffffff" if on else FG)
        st = proto.AUTO_STATES[d["auto_state"]] if d["auto_state"] < 4 else "?"
        if not d["prog_ok"]:
            txt = "no program on the boat"
        elif d["mode"] == proto.MODE_AUTO:
            txt = "waypoint %d of %d, %s" % (d["auto_wp"] + 1, d["prog_n"], st)
            if d["speed_limited"]:
                txt += "\nSPEED LIMITED BY POWER LIMIT"
        elif d["prog_suspended"]:
            txt = "program interrupted at waypoint %d: Resume + CONFIRM" % (d["auto_wp"] + 1)
        elif d["prog_finished"]:
            txt = "program finished"
        else:
            txt = "%d waypoints stored" % d["prog_n"]
        self.auto_lbl.configure(text=txt)
        if d["mode"] == proto.MODE_TRAINING:
            tr = proto.TRAIN_STAGES[min(d["train_stage"], 2)]
            if d["train_stage"] >= 1:
                tr += " / " + proto.TRAIN_PHASES[min(d["train_phase"], 10)]
        else:
            tr = "last: " + proto.TRAIN_RESULTS[min(d["train_result"], 11)]
        v["train"].configure(text=tr)

        # banners
        if d["confirm_kind"]:
            self.joy_banner.configure(text="PRESS JOYSTICK CONFIRM - %s - %.0f s" % (
                proto.CONFIRM_KINDS[min(d["confirm_kind"], 4)], d["confirm_left_s"]), bg="#ffd000")
        else:
            self.joy_banner.configure(text="JOYSTICK IN CONTROL", bg=ORANGE)
        self.show(self.joy_banner, d["mode"] == proto.MODE_MANUAL or d["confirm_kind"] != 0,
                  row=0, column=0, columnspan=2, sticky="ew")
        alarms = proto.alarm_texts(d["alarm_bits"])
        on = d["alarm"] or bool(alarms)
        if on:
            self.alarm_banner.configure(text="ALARM: " + (", ".join(alarms) if alarms else "autopilot"))
        self.show(self.alarm_banner, on, row=1, column=0, sticky="ew")
        self.show(self.alarm_ack, on, row=1, column=1, sticky="ns")

        per = ((50000, "core 0  control 20 Hz"), (1000, "CPU1    monitor 1 kHz"), (10000, "core 2  logger"))
        keys = (("t_core0_avg", "t_core0_max", "ovr_core0"), ("t_mon_avg", "t_mon_max", "ovr_mon"),
                ("t_log_avg", "t_log_max", "ovr_log"))
        lines = ["%-22s %8s %8s %7s %9s" % ("", "avg us", "max us", "max %", "overruns")]
        for (p, name), (ka, km, ko) in zip(per, keys):
            lines.append("%-22s %8d %8d %6.1f%% %9d" % (name, d[ka], d[km], 100.0 * d[km] / p, d[ko]))
        lines.append("")
        lines.append("link: %d frames received, %d CRC errors" % (d["rx_frames"], d["rx_crc_errors"]))
        self.sys_lbl.configure(text="
".join(lines))

        self.draw_joystick(d)
        self.draw_plot()
        self.draw_map(d)

    def draw_joystick(self, d):
        c = self.joy_cv
        c.delete("all")
        s = 150
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
        txt = proto.JOY_STATUS[st] if st < 4 else "?"
        if st == 3:
            txt += " (%s axis)" % ("X" if d["joy_fault_axis"] == 1 else "Y")
        self.joy_status.configure(text="%s\nX %+.2f  Y %+.2f" % (txt, d["joy_x"], d["joy_y"]),
                                  fg=RED if st in (1, 3) else FG)
        self.joy_btns.configure(text="TAKE %s  AUTO %s  CONFIRM %s" % (
            "■" if d["joy_take"] else "□", "■" if d["joy_auto"] else "□", "■" if d["joy_confirm"] else "□"))

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
        c.create_text(10, 8, text="Heading (black) vs target (blue), ±%.0f° around %.0f°" % (span, ref),
                      anchor="nw", font=F_MED)
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
        c.create_text(10, oy0, text="Outboard angle (estimate), ±40°", anchor="nw", font=F_MED)
        for a in (-30, 0, 30):
            c.create_line(60, Y2(a), w - 20, Y2(a), fill="#d0d0d0")
            c.create_text(55, Y2(a), text="%+d" % a, anchor="e", font=F_TXT)
        pts = []
        for p in self.hist:
            pts += [X(p[0]), Y2(p[3])]
        if len(pts) >= 4:
            c.create_line(*pts, fill=ORANGE, width=3)
        c.create_text(w - 20, h - 4, text="last %.0f s" % PLOT_S, anchor="se", font=F_TXT)

    def draw_map(self, d):
        c = self.map
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 50 or d["pos_source"] == 0:
            c.create_text(w / 2, h / 2, text="no position yet", font=F_BIG)
            return
        lat0, lon0 = d["est_lat"], d["est_lon"]
        k = math.cos(math.radians(lat0)) * 111320.0
        m = self.map_m_per_px

        def P(lat, lon):
            return w / 2 + (lon - lon0) * k / m, h / 2 - (lat - lat0) * 111320.0 / m

        # scale bar
        bar_m = 10 ** math.floor(math.log10(max(1.0, m * 150)))
        c.create_line(20, h - 20, 20 + bar_m / m, h - 20, width=4)
        c.create_text(20, h - 26, text="%g m" % bar_m, anchor="sw", font=F_TXT)
        c.create_text(w - 20, 20, text="N ↑", font=F_BIG, anchor="ne")
        # track
        if len(self.track) >= 2:
            pts = []
            for la, lo in self.track[-2000:]:
                pts += P(la, lo)
            c.create_line(*pts, fill="#404040", width=2)
        # program (editor copy) and the active waypoint (from the boat)
        pts = [P(wp["lat"], wp["lon"]) for wp in self.prog["wps"]]
        if len(pts) >= 2:
            c.create_line(*[v for p in pts for v in p], fill="#b080c0", width=2)
        for i, (x, y) in enumerate(pts):
            c.create_oval(x - 6, y - 6, x + 6, y + 6, outline="#7a00a0", width=2)
            c.create_text(x + 9, y, text=str(i + 1), anchor="w", font=F_TXT, fill="#7a00a0")
        if d["wp_set"]:
            x, y = P(d["wp_lat"], d["wp_lon"])
            c.create_line(w / 2, h / 2, x, y, fill="#7a00a0", dash=(6, 4), width=2)
            c.create_oval(x - 11, y - 11, x + 11, y + 11, outline="#7a00a0", width=4)
        # uncertainty circle
        r = d["unc_m"] / m
        c.create_oval(w / 2 - r, h / 2 - r, w / 2 + r, h / 2 + r,
                      outline=RED if d["pos_source"] in (2, 3) else GREEN, width=2, dash=(4, 3))
        # drift arrow (current), 1 kn = 60 px
        if d["drift_kn"] > 0.02:
            a = math.radians(d["set_deg"])
            L = min(120.0, d["drift_kn"] * 60.0)
            x2, y2 = w / 2 + L * math.sin(a), h / 2 - L * math.cos(a)
            c.create_line(w / 2, h / 2, x2, y2, arrow="last", arrowshape=(14, 16, 6), fill=BLUE, width=3)
            c.create_text(x2, y2 - 12, text="drift %.2f kn" % d["drift_kn"], font=F_TXT, fill=BLUE)
        # boat
        a = math.radians(d["heading"])
        pts = []
        for ang, rr in ((0, 18), (140, 12), (180, 6), (220, 12)):
            b = a + math.radians(ang)
            pts += [w / 2 + rr * math.sin(b), h / 2 - rr * math.cos(b)]
        c.create_polygon(*pts, fill=MODE_COLORS.get(d["mode_name"], FG), outline=FG, width=2)
        c.create_text(10, 10, anchor="nw", font=F_TXT,
                      text="%.0f m/px   track %d points" % (m, len(self.track)) if m >= 1 else
                      "%.1f m/px   track %d points" % (m, len(self.track)))

    # --- alarm sound -------------------------------------------------------------
    def alarm_tick(self):
        d = self.t
        stale = self.link is not None and self.t_rx and time.time() - self.t_rx > 3.0
        if (d and (d["alarm"] or d["alarm_bits"])) or stale:
            flash = int(time.time() * 2) % 2 == 0
            self.alarm_banner.configure(bg=RED if flash else "#800000")
            if stale:
                self.alarm_banner.configure(text="ALARM: NO DATA FROM THE BOAT")
                self.show(self.alarm_banner, True, row=1, column=0, sticky="ew")
            if time.time() - self.last_beep > 1.0:
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
        if self.link:
            self.link.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root, sys.argv[1] if len(sys.argv) > 1 else None)
    root.mainloop()


if __name__ == "__main__":
    main()
