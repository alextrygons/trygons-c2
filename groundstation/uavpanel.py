"""«ΜΑΓΓΕΜΑ UAV» = the UAV magazine page of the TOOLS window (CLAUDE.md V "Registry", step U2; DISPLAY + requests
only: the gateway checks every request again).

Six slots, slot 1 = the top = the next out. Per slot: the owner's UAV picture (uavpics), the name, sysid / UID,
ArduPilot firmware, battery, GPS, VTX channel, ELRS binding, its route / mission / flight (U3), the state (ready /
NOT launchable + why / not registered / no drone) and REGISTER = {"t": "uav_register", "slot": n} to the gateway
(read only on the drone). Under the rows (U3): the hatch, the launch sequence, PREPARE LAUNCH (the captain's YES / NO
on screen 1; a single operator his own), STOP (any crew station), HATCH CLOSE (the captain's own, after a stop) and
the gateway's UAV notices; (U4) FLIGHT LOGS = the gateway's per-drone flight logs, exported as CSV / GPX.
The data = gw_status "uav" (1 Hz); the answers = "uav_ack". Not connected / a boat without the magazine = one line.
"""
import i18n
import theme
import uavpics
from i18n import T

ROWS = 6


def _tr(text):
    return i18n.from_en(text or "")


def row_view(s, next_slot):
    """The texts of one slot row (pure: tests UV2) -> dict(head, state, state_kind, details, assist, can_register)."""
    head = T("uav.slot", n=s["slot"]) + ("   " + T("uav.next") if next_slot == s["slot"] else "")
    if s.get("name"):
        head += "   " + s["name"]
    if not s.get("usb"):
        state, kind = T("uav.no_usb"), "off"
    elif s.get("registering"):
        state, kind = T("uav.registering"), "info"
    elif not s.get("registered"):
        state, kind = T("uav.not_registered") + ("" if s.get("present") else " - " + T("uav.empty")), "off"
    elif s.get("launchable"):
        state, kind = T("uav.launchable"), "good"
    else:
        state, kind = T("uav.not_launchable", why=_tr(s.get("why"))), "caution"
    if s.get("armed"):
        state, kind = T("uav.armed") + " - " + state, "caution"
    parts = []
    if s.get("sysid") is not None:
        parts.append(T("uav.fw", fw=s.get("fw") or "?", sysid=s["sysid"]))
    parts.append(T("uav.batt", pct=s["battery_pct"], v=s.get("battery_v") or "-") if s.get("battery_pct") is not None
                 else T("uav.batt_none"))
    parts.append(T("uav.gps", fix=s.get("gps_fix") or 0, sats=s.get("sats") or 0) if s.get("present")
                 else T("uav.gps_none"))
    parts.append(T("uav.vtx", vtx=s.get("vtx") or "-", elrs=s.get("elrs") or "-"))
    if s.get("flying"):
        m = T("uav.m_flying_" + s["flying"]) if i18n.has("uav.m_flying_" + s["flying"]) else s["flying"]
    elif s.get("mission"):
        m = T("uav.m_loaded", n=s.get("route") or 0)
    elif s.get("route"):
        m = T("uav.m_route", n=s["route"])
    else:
        m = T("uav.mission_no")
    parts.append(T("uav.mission", m=m))
    a = s.get("assist")
    if not a:
        assist = T("uav.assist_never")
    elif a.get("ok"):
        what = "+".join(T("uav.what_" + w) if i18n.has("uav.what_" + w) else w
                        for w in (a.get("what") or "").split("+") if w)
        assist = T("uav.assist", what=what, age=a.get("age_s", 0))
    else:
        assist = T("uav.assist_no", why=_tr(a.get("why")))
    return {"head": head, "state": state, "kind": kind, "details": " · ".join(parts), "assist": assist,
            "can_register": bool(s.get("usb")) and not s.get("registering"),
            "problems": [_tr(p) for p in s.get("problems") or []]}


def launch_view(st):
    """The launch / hatch lines of the page (pure: tests UV5) -> dict(hatch, hatch_kind, seq, notices, can_prepare,
    can_stop, can_close)."""
    h = (st or {}).get("hatch") or {}
    L = (st or {}).get("launch") or {}
    hs = h.get("state") or "UNKNOWN"
    hatch = T("uav.hatch", state=T("uav.hs_" + hs) if i18n.has("uav.hs_" + hs) else hs)
    if h.get("why"):
        hatch += " - " + _tr(h["why"])
    if not h.get("relay"):
        hatch += " - " + T("uav.no_hatch_relay")
    state = L.get("state") or "IDLE"
    seq = T("uav.seq_" + state) if i18n.has("uav.seq_" + state) else state
    if state in ("RUN", "CLOSING") and L.get("cur"):
        ph = L.get("phase") or ""
        seq += " - " + T("uav.seq_cur", n=L["cur"], phase=T("uav.ph_" + ph) if i18n.has("uav.ph_" + ph) else ph)
    if L.get("launched"):
        seq += " - " + T("uav.seq_launched", lst=", ".join(str(x) for x in L["launched"]))
    if L.get("why"):
        seq += " - " + _tr(L["why"])
    busy = state in ("ASK", "RUN", "CLOSING")
    notes = [_tr(n.get("text")) for n in (st or {}).get("notices") or []][-3:]
    if (st or {}).get("high_speed"):              # owner W59: a NON-blocking line (the launch is still allowed)
        notes = [T("uav.high_speed")] + notes
    return {"hatch": hatch, "hatch_kind": "caution" if hs in ("FAULT", "UNKNOWN") or (hs == "OPEN" and not busy)
            else "info", "seq": seq, "notices": notes, "high_speed": bool((st or {}).get("high_speed")),
            "can_prepare": not busy and hs in ("CLOSED", "OPEN"), "can_stop": busy,
            "can_close": not busy and hs != "CLOSED"}


def log_line(r):
    """One flight log in the list: "UAV-1  2026-11-04 16:09 UTC  12 kB"."""
    f = r.get("file") or ""
    when = "%s-%s-%s %s:%s" % (f[:4], f[4:6], f[6:8], f[9:11], f[11:13]) if len(f) >= 13 else f
    return T("uav.log_line", name=r.get("name", "?"), when=when, kb=max(1, int((r.get("bytes") or 0) / 1024)))


def fly_view(st, mine):
    """The manual-control part of the page (pure: tests UV13): the flying UAVs that may be asked for, who flies
    which (gw_status "uav" "pilots")."""
    drones = (st or {}).get("drones") or {}
    pilots = (st or {}).get("pilots") or {}
    can = [int(k) for k, d in sorted(drones.items()) if str(k).isdigit() and d.get("armed") and not d.get("landed")
           and not d.get("in_slot") and not d.get("landing") and int(k) != mine]
    lines = [T("uavf.flown_by", n=k, name=p.get("name") or "?", role=T("uavf.r_captain") if p.get("captain") else
               T("uavf.r_copilot")) for k, p in sorted(pilots.items())]
    if mine is not None:
        lines.insert(0, T("uavf.banner", n=mine))
    return {"can_take": can, "mine": mine, "text": "   ".join(lines)}


class Page:
    def __init__(self, fr, app):
        import tkinter as tk
        self.tk, self.app, self.fr = tk, app, fr
        C = theme.C
        BG = C["bg"]
        self.status = None
        self.sel = None
        i18n.bind(tk.Label(fr, font=theme.font("value"), bg=BG, fg=C["fg"]), "uav.title").pack(anchor="w", padx=6,
                                                                                               pady=(6, 2))
        i18n.bind(tk.Label(fr, font=theme.font("label"), bg=BG, fg=C["fg"], wraplength=900, justify="left"),
                  "uav.note").pack(anchor="w", padx=6)
        self.head = tk.Label(fr, font=theme.font("label"), bg=BG, fg=C["fg"], anchor="w", justify="left")
        self.head.pack(fill="x", padx=6, pady=(6, 2))
        self.pic = uavpics.photo(32, master=fr)
        self.rows = []
        box = tk.Frame(fr, bg=BG)
        box.pack(fill="x", padx=6)
        for i in range(ROWS):
            r = tk.Frame(box, bg=C["panel"], highlightthickness=1, highlightbackground=C["fg"])
            r.pack(fill="x", pady=2)
            pic = tk.Label(r, image=self.pic, bg=C["panel"]) if self.pic else tk.Label(r, text=" ", bg=C["panel"])
            pic.pack(side="left", padx=6, pady=4)
            col = tk.Frame(r, bg=C["panel"])
            col.pack(side="left", fill="x", expand=True)
            labels = {}
            for k, lvl in (("head", "button"), ("state", "label"), ("details", "small"), ("assist", "small")):
                lb = tk.Label(col, font=theme.font(lvl), bg=C["panel"], fg=C["fg"], anchor="w", justify="left")
                lb.pack(fill="x")
                lb.bind("<Button-1>", lambda _e, n=i + 1: self.show_problems(n))
                labels[k] = lb
            b = tk.Button(r, command=lambda n=i + 1: self.register(n), **theme.button_opts())
            i18n.bind(b, "uav.register")
            b.pack(side="right", padx=8)
            self.rows.append((labels, b))
        # the launch (step U3): hatch, sequence, PREPARE LAUNCH (the captain answers YES / NO on screen 1), STOP,
        # HATCH CLOSE (the captain's own, after a stop)
        lf = tk.Frame(fr, bg=BG)
        lf.pack(fill="x", padx=6, pady=(6, 0))
        self.l_hatch = tk.Label(lf, font=theme.font("label"), bg=BG, fg=C["fg"], anchor="w", justify="left")
        self.l_hatch.pack(fill="x")
        self.l_seq = tk.Label(lf, font=theme.font("label"), bg=BG, fg=C["fg"], anchor="w", justify="left",
                              wraplength=900)
        self.l_seq.pack(fill="x")
        bf = tk.Frame(lf, bg=BG)
        bf.pack(fill="x", pady=(2, 0))
        self.b_prep = tk.Button(bf, command=self.prepare, **theme.button_opts("ok"))
        i18n.bind(self.b_prep, "uav.prepare")
        self.b_prep.pack(side="left", padx=(0, 6))
        self.b_stop = tk.Button(bf, command=self.stop, **theme.button_opts())
        i18n.bind(self.b_stop, "uav.stop")
        self.b_stop.pack(side="left", padx=(0, 6))
        self.b_close = tk.Button(bf, command=self.hatch_close, **theme.button_opts())
        i18n.bind(self.b_close, "uav.hatch_close")
        self.b_close.pack(side="left")
        # the per-drone flight logs (step U4): the gateway's list, exported as CSV / GPX to a file on this PC
        self.b_logs = tk.Button(bf, command=self.ask_logs, **theme.button_opts())
        i18n.bind(self.b_logs, "uav.flightlogs")
        self.b_logs.pack(side="left", padx=(18, 6))
        self.b_csv = tk.Button(bf, command=lambda: self.export("csv"), **theme.button_opts())
        i18n.bind(self.b_csv, "uav.export_csv")
        self.b_csv.pack(side="left", padx=(0, 6))
        self.b_gpx = tk.Button(bf, command=lambda: self.export("gpx"), **theme.button_opts())
        i18n.bind(self.b_gpx, "uav.export_gpx")
        self.b_gpx.pack(side="left", padx=(0, 6))
        self.b_tlog = tk.Button(bf, command=lambda: self.export("tlog"), **theme.button_opts())   # step X1
        i18n.bind(self.b_tlog, "uav.export_tlog")
        self.b_tlog.pack(side="left")
        # manual control (step U5): «ΧΕΙΡΙΣΜΟΣ UAV n» for every flying UAV, «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ» (the gateway's
        # pilot lock decides: one pilot per drone, the captain only with the boat in STANDBY)
        ff = tk.Frame(lf, bg=BG)
        ff.pack(fill="x", pady=(4, 0))
        self.b_fly = []
        for i in range(ROWS):
            b = tk.Button(ff, command=lambda n=i + 1: self.fly(n), text=T("uavf.take", n=i + 1),
                          **theme.button_opts())
            b.pack(side="left", padx=(0, 4))
            self.b_fly.append(b)
        self.b_give = tk.Button(ff, command=self.give, **theme.button_opts())
        i18n.bind(self.b_give, "uavf.give")
        self.b_give.pack(side="left", padx=(12, 0))
        self.l_pilots = tk.Label(lf, font=theme.font("small"), bg=BG, fg=C["fg"], anchor="w", justify="left",
                                 wraplength=900)
        self.l_pilots.pack(fill="x")
        self.logs = []
        self.lb_logs = tk.Listbox(lf, height=4, font=theme.font("small"), bg=C["panel"], fg=C["fg"],
                                  exportselection=False)
        self.lb_logs.pack(fill="x", pady=(2, 0))
        self.l_notes = tk.Label(lf, font=theme.font("small"), bg=BG, fg=C["fg"], anchor="w", justify="left",
                                wraplength=900)
        self.l_notes.pack(fill="x")
        self.stats = tk.Label(fr, font=theme.font("small"), bg=BG, fg=C["fg"], anchor="w")
        self.stats.pack(fill="x", padx=6, pady=(4, 0))
        self.msg = tk.Label(fr, font=theme.font("label"), bg=BG, fg=C["fg"], anchor="w", justify="left",
                            wraplength=900)
        self.msg.pack(fill="x", padx=6, pady=(4, 0))
        self.prob = tk.Label(fr, font=theme.font("small"), bg=BG, fg=C["fg"], anchor="w", justify="left",
                             wraplength=900)
        self.prob.pack(fill="x", padx=6, pady=(2, 6))
        i18n.on_change(self.lang_changed)
        self.update(None)

    def lang_changed(self):
        try:
            self.update(self.status)
        except self.tk.TclError:
            raise RuntimeError("uav page gone")      # (i18n drops this listener)

    def connected(self):
        link = getattr(self.app, "link", None)
        return link is not None and hasattr(link, "send_msg") and getattr(link, "connected", False)

    def register(self, slot):
        if not self.connected():
            return self.say(T("uav.not_connected"), False)
        self.app.link.send_msg({"t": "uav_register", "slot": slot})
        self.sel = slot

    def prepare(self):
        if not self.connected():
            return self.say(T("uav.not_connected"), False)
        msg = {"t": "uav_launch_prepare"}
        if getattr(self.app, "single_op", False) and hasattr(self.app, "captain_confirm"):
            # U3 review M3: a single operator answers his OWN YES / NO first, like every other single-operator path;
            # only then "confirm": "self" goes to the gateway (which checks again)
            msg["confirm"] = "self"
            return self.app.captain_confirm(T("uav.prepare"), lambda: self.app.link.send_msg(msg))
        self.app.link.send_msg(msg)

    def stop(self):
        if self.connected():
            self.app.link.send_msg({"t": "uav_launch_stop"})

    def hatch_close(self):
        if self.connected():
            self.app.link.send_msg({"t": "uav_hatch_close"})

    def fly(self, n):
        if not self.connected():
            return self.say(T("uav.not_connected"), False)
        self.app.uav_take(n)

    def give(self):
        self.app.uav_give()

    def ask_logs(self):
        if not self.connected():
            return self.say(T("uav.not_connected"), False)
        self.app.link.send_msg({"t": "uav_logs"})

    def on_logs(self, d):
        """"uav_logs" from the gateway: the flight logs, newest first."""
        self.logs = sorted(d.get("logs") or [], key=lambda r: -float(r.get("t") or 0))
        try:
            self.lb_logs.delete(0, "end")
            for r in self.logs:
                self.lb_logs.insert("end", log_line(r))
        except self.tk.TclError:
            pass
        if not self.logs:
            self.say(T("uav.no_logs"))

    def export(self, fmt, path=None):
        """Export the selected flight log as CSV / GPX (a save dialog unless path is given: tests)."""
        try:
            sel = self.lb_logs.curselection()
        except self.tk.TclError:
            sel = ()
        if not sel or sel[0] >= len(self.logs):
            return self.say(T("uav.pick_log"), False)
        r = self.logs[sel[0]]
        if path is None:
            from tkinter import filedialog
            path = filedialog.asksaveasfilename(defaultextension="." + fmt, initialfile="%s_%s.%s" % (
                r["name"], r["file"][:-4], fmt), filetypes=[(fmt.upper(), "*." + fmt)])
            if not path:
                return
        self.app.uav_log_export(r["name"], r["file"], fmt, path)

    def say(self, text, ok=None):
        col = theme.CAUTION if ok is False else theme.C["fg"]      # (green only on the video; amber = a caution)
        try:
            self.msg.configure(text=text, fg=col)
        except self.tk.TclError:
            pass

    def on_ack(self, d):
        """"uav_ack" from the gateway."""
        self.say(T("uav.msg", text=_tr(d.get("text"))), bool(d.get("ok")))
        if d.get("slot"):
            self.sel = d["slot"]
        if not d.get("ok") and d.get("problems"):
            self.prob.configure(text=T("uav.problems", n=d.get("slot")) + "\n" +
                                "\n".join("- " + _tr(p) for p in d["problems"]))
        elif d.get("ok"):
            self.prob.configure(text="")

    def show_problems(self, slot):
        self.sel = slot
        st = self.status or {}
        for s in st.get("slots", []):
            if s["slot"] == slot:
                v = row_view(s, st.get("next"))
                self.prob.configure(text=(T("uav.problems", n=slot) + "\n" + "\n".join("- " + p for p in v["problems"]))
                                    if v["problems"] else "")

    def update(self, st):
        """gw_status "uav" (None = not connected / an older gateway)."""
        self.status = st
        C = theme.C
        if not st or not st.get("enabled"):
            self.head.configure(text=T("uav.router_off") if st else T("uav.not_connected"))
            self.stats.configure(text="")
            for w in (self.l_hatch, self.l_seq, self.l_notes):
                w.configure(text="")
            for b in (self.b_prep, self.b_stop, self.b_close, self.b_logs, self.b_give, *self.b_fly):
                b.configure(state="disabled")
            self.l_pilots.configure(text="")
            for labels, b in self.rows:
                for lb in labels.values():
                    lb.configure(text="")
                b.configure(state="disabled")
            return
        why = _tr(st.get("why") or "")
        self.head.configure(text=T("uav.router", state=T("uav.st_" + str(st.get("state"))) if i18n.has("uav.st_" + str(
            st.get("state"))) else st.get("state", "-"), why=("(%s)" % why) if why else "") +
                            ("  " + T("uav.sim") if st.get("sim") else ""),
                            fg=C["fg"] if st.get("state") == "ACTIVE" else theme.CAUTION)
        r = (st.get("router") or {}).get("stats") or {}
        self.stats.configure(text=T("uav.stats", rx=r.get("rx", 0), tx=r.get("tx", 0), din=r.get("denied_in", 0),
                                    dout=r.get("denied_out", 0)))
        lv = launch_view(st)
        self.l_hatch.configure(text=lv["hatch"], fg=theme.CAUTION if lv["hatch_kind"] == "caution" else C["fg"])
        self.l_seq.configure(text=lv["seq"])
        self.l_notes.configure(text=chr(10).join(lv["notices"]))
        on = st.get("state") == "ACTIVE"
        self.b_prep.configure(state="normal" if on and lv["can_prepare"] else "disabled")
        self.b_stop.configure(state="normal" if lv["can_stop"] else "disabled")
        self.b_close.configure(state="normal" if lv["can_close"] else "disabled")
        self.b_logs.configure(state="normal")
        fv = fly_view(st, getattr(getattr(self.app, "uav_fly", None), "slot", None))
        for i, b in enumerate(self.b_fly):
            b.configure(text=T("uavf.take", n=i + 1), state="normal" if on and (i + 1) in fv["can_take"] else "disabled")
        self.b_give.configure(state="normal" if fv["mine"] is not None else "disabled")
        self.l_pilots.configure(text=fv["text"])
        slots = {s["slot"]: s for s in st.get("slots", [])}
        for i, (labels, b) in enumerate(self.rows):
            s = slots.get(i + 1)
            if s is None:
                for lb in labels.values():
                    lb.configure(text="")
                b.configure(state="disabled")
                continue
            v = row_view(s, st.get("next"))
            labels["head"].configure(text=v["head"])
            labels["state"].configure(text=v["state"], fg=theme.CAUTION if v["kind"] == "caution" else C["fg"])
            labels["details"].configure(text=v["details"])
            labels["assist"].configure(text=v["assist"])
            b.configure(state="normal" if v["can_register"] and st.get("state") == "ACTIVE" else "disabled")


def build_page(fr, app):
    return Page(fr, app)
