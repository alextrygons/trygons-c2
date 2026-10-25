"""Screen 2: choosing the boat, confirming it, and the crew (CLAUDE.md sections D + F).

  BoatSelector  the boats of boats/ (git) with ONLINE / OFFLINE / IP NOT SET; user name +
                password; connects with the boat's name (the gateway and this station both
                refuse another boat).
  BoatConfirm   after connecting: type the boat's name AND tick "I see its live front camera"
                (only possible while a front camera picture is live) - only then this station
                may take HELM (app.boat_ok).
  crew          the gateway's captain / copilot roles: shown on screen 2 (and screen 1),
                role swap ask / answer.
Display and dialogs only: what the boat does is decided by the gateway and the AURIX.
"""
import tkinter as tk

import boats
import i18n
from i18n import T

F = ("Segoe UI", 12)
FB = ("Segoe UI", 12, "bold")
FH = ("Segoe UI", 16, "bold")
STATUS_COL = {"online": "#008000", "offline": "#a00000", "no_ip": "#a06000", "checking": "#606060"}


def front_live(app):
    """A front camera picture is live (not frozen) on this station."""
    vs = getattr(app, "vstreams", {}) or {}
    return any(("FRONT" in name.upper()) and s is not None and not s.frozen() for name, s in vs.items())


class BoatSelector:
    def __init__(self, app, folder=None):
        self.app = app
        self.boats = boats.load(folder)
        self.prober = boats.Prober(self.boats).start()
        self.w = tk.Toplevel(app.root)
        self.w.title(T("boat.title"))
        self.w.protocol("WM_DELETE_WINDOW", self.close)
        tk.Label(self.w, text=T("boat.head"), font=FH).pack(anchor="w", padx=10, pady=(10, 2))
        tk.Label(self.w, text=T("boat.help"), font=F, justify="left", wraplength=620).pack(anchor="w", padx=10)
        self.lb = tk.Listbox(self.w, font=("Consolas", 13), height=max(3, len(self.boats)), width=60,
                             exportselection=False)
        self.lb.pack(fill="x", padx=10, pady=8)
        f = tk.Frame(self.w)
        f.pack(fill="x", padx=10)
        tk.Label(f, text=T("cap.login_user"), font=F).grid(row=0, column=0, sticky="w")
        self.user = tk.Entry(f, font=F, width=20)
        self.user.grid(row=0, column=1, sticky="w", padx=4)
        tk.Label(f, text=T("cap.login_pw"), font=F).grid(row=1, column=0, sticky="w")
        self.pw = tk.Entry(f, font=F, width=20, show="*")
        self.pw.grid(row=1, column=1, sticky="w", padx=4)
        tk.Label(f, text=T("boat.crew"), font=F).grid(row=2, column=0, sticky="w")
        self.crew = tk.StringVar(value="captain")
        cf = tk.Frame(f)
        cf.grid(row=2, column=1, sticky="w")
        for v in ("captain", "copilot"):
            rb = tk.Radiobutton(cf, variable=self.crew, value=v, font=F)
            i18n.bind(rb, "crew." + v)
            rb.pack(side="left")
        b = tk.Frame(self.w)
        b.pack(fill="x", padx=10, pady=10)
        self.go = tk.Button(b, text=T("boat.connect"), font=FB, bg="#b0ffb0", command=self.connect)
        self.go.pack(side="left")
        tk.Button(b, text=T("boat.close"), font=F, command=self.close).pack(side="left", padx=8)
        self.msg = tk.Label(self.w, text="", font=F, fg="#a00000", wraplength=620, justify="left")
        self.msg.pack(anchor="w", padx=10, pady=(0, 10))
        if not self.boats:
            self.msg.configure(text=T("boat.none", folder=folder or boats.BOATS_DIR))
        self.tick()

    def rows(self):
        out = []
        for bt in self.boats:
            st = "no_ip" if (bt.error or not bt.ip_ok()) else self.prober.status(bt)
            st = "checking" if st == "?" else st
            out.append((bt, st))
        return out

    def tick(self):
        try:
            sel = self.lb.curselection()
            self.lb.delete(0, "end")
            for i, (bt, st) in enumerate(self.rows()):
                self.lb.insert("end", "%-12s %-24s %s" % (bt.name, (bt.ip if bt.ip_ok() else "-"),
                                                         T("boat.st_" + st)))
                self.lb.itemconfigure(i, fg=STATUS_COL.get(st, "#202020"))
            if sel:
                self.lb.selection_set(sel[0])
            self.w.after(1000, self.tick)
        except tk.TclError:
            pass

    def selected(self):
        sel = self.lb.curselection()
        return self.rows()[sel[0]] if sel else (None, None)

    def connect(self):
        bt, st = self.selected()
        if bt is None:
            return self.msg.configure(text=T("boat.pick"))
        if st == "no_ip":
            return self.msg.configure(text=T("boat.no_ip", name=bt.name, file=bt.path))
        if st != "online":
            return self.msg.configure(text=T("boat.offline", name=bt.name))
        err = self.app.connect_boat(bt, self.user.get().strip(), self.pw.get(), self.crew.get())
        if err:
            self.msg.configure(text=err)
        else:
            self.close()

    def close(self):
        self.prober.stop()
        try:
            self.w.destroy()
        except tk.TclError:
            pass
        if self.app.boatsel is self:
            self.app.boatsel = None


class BoatConfirm:
    """The name typed back + the live front camera seen -> app.boat_ok (HELM allowed)."""

    def __init__(self, app, boat):
        self.app = app
        self.boat = boat
        self.w = tk.Toplevel(app.root, bg="#fff4c0")
        self.w.title(T("boat.confirm_title"))
        self.w.attributes("-topmost", True)
        tk.Label(self.w, text=T("boat.confirm_head", name=boat.name), font=FH, bg="#fff4c0").pack(padx=14, pady=(12, 4))
        tk.Label(self.w, text=T("boat.confirm_help", name=boat.name), font=F, bg="#fff4c0", wraplength=560,
                 justify="left").pack(padx=14)
        self.name = tk.Entry(self.w, font=FH, width=16, justify="center")
        self.name.pack(pady=8)
        self.cam = tk.BooleanVar(value=False)
        self.cam_cb = tk.Checkbutton(self.w, variable=self.cam, font=FB, bg="#fff4c0",
                                     text=T("boat.confirm_cam", name=boat.name))
        self.cam_cb.pack(padx=14)
        self.cam_lbl = tk.Label(self.w, text="", font=F, bg="#fff4c0", fg="#a00000")
        self.cam_lbl.pack()
        b = tk.Frame(self.w, bg="#fff4c0")
        b.pack(pady=10)
        tk.Button(b, text=T("boat.confirm_ok"), font=FB, bg="#b0ffb0", command=self.ok).pack(side="left", padx=6)
        tk.Button(b, text=T("boat.confirm_no"), font=F, command=self.wrong).pack(side="left", padx=6)
        self.msg = tk.Label(self.w, text="", font=F, bg="#fff4c0", fg="#a00000")
        self.msg.pack(pady=(0, 10))
        self.tick()

    def tick(self):
        try:
            live = front_live(self.app)
            self.cam_cb.configure(state="normal" if live else "disabled")
            if not live:
                self.cam.set(False)
            self.cam_lbl.configure(text="" if live else T("boat.confirm_nocam"))
            self.w.after(500, self.tick)
        except tk.TclError:
            pass

    def ok(self):
        if not boats.confirmed(self.name.get(), self.boat, self.cam.get() and front_live(self.app)):
            return self.msg.configure(text=T("boat.confirm_bad", name=self.boat.name))
        self.app.boat_ok = True
        self.app.say(T("boat.confirmed", name=self.boat.name), "#00a000")
        self.app.event(T("boat.confirmed", name=self.boat.name))
        self.close()

    def wrong(self):
        """Not this boat: disconnect."""
        self.close()
        self.app.disconnect(T("boat.not_this", name=self.boat.name))

    def close(self):
        try:
            self.w.destroy()
        except tk.TclError:
            pass
        if self.app.boatconf is self:
            self.app.boatconf = None


def crew_of(app):
    """(crew of this station, captain's name, licence) from the gateway status / welcome."""
    gw = app.gw_status or {}
    me = [s for s in gw.get("stations", []) if s.get("id") == gw.get("you")]
    w = getattr(app.link, "welcome", None) or {}
    crew = me[0].get("crew") if me else w.get("crew")
    lic = me[0].get("licence") if me else w.get("licence")
    cap = [s for s in gw.get("stations", []) if s.get("crew") == "captain"]
    return crew, (cap[0].get("name") if cap else None), bool(lic)


def ask_swap(app, d):
    """The other side asks for a role swap: yes / no."""
    from tkinter import messagebox
    yes = messagebox.askyesno(T("crew.swap_title"), T("crew.swap_ask", name=d.get("name", "?")), parent=app.root)
    app.link.send_msg({"t": "crew_swap_answer", "yes": bool(yes)})
    app.event(T("crew.swap_answered", yes=T("crew.yes") if yes else T("crew.no")))


def request_swap(app):
    """This station asks for a role swap (the captain: with the copilot that holds a licence)."""
    gw = app.gw_status or {}
    crew, _cap, _lic = crew_of(app)
    msg = {"t": "crew_swap"}
    if crew == "captain":
        cops = [s for s in gw.get("stations", []) if s.get("crew") == "copilot" and s.get("licence")]
        if not cops:
            return app.say(T("crew.no_partner"), "#a00000")
        msg["with"] = cops[0]["id"]
    app.link.send_msg(msg)
    app.event(T("crew.swap_asked"))
