"""«ΜΑΓΓΕΜΑ UAV» = the UAV magazine page of the TOOLS window (CLAUDE.md V "Registry", step U2; DISPLAY + requests
only: the gateway checks every request again).

Six slots, slot 1 = the top = the next out. Per slot: the owner's UAV picture (uavpics), the name, sysid / UID,
ArduPilot firmware, battery, GPS, VTX channel, ELRS binding, mission (U3), the state (ready / NOT launchable + why /
not registered / no drone) and REGISTER = {"t": "uav_register", "slot": n} to the gateway (read only on the drone).
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
    parts.append(T("uav.mission", m=T("uav.mission_no") if not s.get("mission") else s["mission"]))
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
            for labels, b in self.rows:
                for lb in labels.values():
                    lb.configure(text="")
                b.configure(state="disabled")
            return
        why = _tr(st.get("why") or "")
        self.head.configure(text=T("uav.router", state=st.get("state", "-"), why=("(%s)" % why) if why else "") +
                            ("  " + T("uav.sim") if st.get("sim") else ""),
                            fg=C["fg"] if st.get("state") == "ACTIVE" else theme.CAUTION)
        r = (st.get("router") or {}).get("stats") or {}
        self.stats.configure(text=T("uav.stats", rx=r.get("rx", 0), tx=r.get("tx", 0), din=r.get("denied_in", 0),
                                    dout=r.get("denied_out", 0)))
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
