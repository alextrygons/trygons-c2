"""Screen 2: choosing the boat, confirming it, and the crew (CLAUDE.md sections D + F).

  BoatSelector  the boats of boats/ (git) by NAME with ONLINE / OFFLINE / IP NOT SET (the IP only
                as a detail); user name + password; connects with the boat's name (the gateway
                and this station both refuse another boat). Screen 2's boat dropdown (first line
                of the status panel) opens it with the chosen boat selected (select()).
  (13n2, owner 2026-10-31 E2: NO confirm dialog any more - the welcome name check is automatic and
                HELM is taken at once, app.connect_boat; a dead camera is only the warning "no video")
  crew          the gateway's captain / copilot roles: shown on screen 2 (and screen 1),
                role swap ask / answer.
Display and dialogs only: what the boat does is decided by the gateway and the AURIX.
"""
import tkinter as tk

import boats
import i18n
import theme
from i18n import T

F = theme.font("label")
FB = theme.font("button")
FH = theme.font("title")
C = theme.C


def status_col(st):
    return {"online": C["ok_txt"], "offline": C["bad_txt"], "no_ip": C["note_txt"], "invalid": C["bad_txt"],
            "checking": C["dim"]}.get(st, C["fg"])


def lab(parent, text, font=F, **kw):
    kw.setdefault("bg", C["bg"])
    kw.setdefault("fg", C["fg"])
    return tk.Label(parent, text=text, font=font, **kw)


def entry(parent, **kw):
    return tk.Entry(parent, bg=C["entry"], fg=C["entry_fg"], insertbackground=C["entry_fg"], **kw)


def front_live(app):
    """A front camera picture is live (not frozen) on this station."""
    vs = getattr(app, "vstreams", {}) or {}
    return any(("FRONT" in name.upper()) and s is not None and not s.frozen() for name, s in vs.items())


class BoatSelector:
    def __init__(self, app, folder=None):
        self.app = app
        lst = getattr(app, "boat_list", None)
        self.boats = list(lst) if folder is None and lst else boats.load(folder)    # the same list as screen 2
        self.prober = boats.Prober(self.boats).start()
        self.w = tk.Toplevel(app.root, bg=C["bg"])
        self.w.title(i18n.W("boat.title"))
        self.w.protocol("WM_DELETE_WINDOW", self.close)
        lab(self.w, T("boat.head"), FH).pack(anchor="w", padx=10, pady=(10, 2))
        lab(self.w, T("boat.help"), justify="left", wraplength=620).pack(anchor="w", padx=10)
        self.lb = tk.Listbox(self.w, font=theme.font("value"), height=max(3, len(self.boats)), width=46,
                             exportselection=False, bg=C["panel"], fg=C["fg"], selectbackground=C["sel"])
        self.lb.pack(fill="x", padx=10, pady=8)
        f = tk.Frame(self.w, bg=C["bg"])
        f.pack(fill="x", padx=10)
        lab(f, T("cap.login_user")).grid(row=0, column=0, sticky="w")
        self.user = entry(f, font=F, width=20)
        self.user.grid(row=0, column=1, sticky="w", padx=4)
        lab(f, T("cap.login_pw")).grid(row=1, column=0, sticky="w")
        self.pw = entry(f, font=F, width=20, show="*")
        self.pw.grid(row=1, column=1, sticky="w", padx=4)
        lab(f, T("boat.crew")).grid(row=2, column=0, sticky="w")
        self.crew = tk.StringVar(value="captain")
        cf = tk.Frame(f, bg=C["bg"])
        cf.grid(row=2, column=1, sticky="w")
        for v in ("captain", "copilot"):
            rb = tk.Radiobutton(cf, variable=self.crew, value=v, font=F, bg=C["bg"], fg=C["fg"],
                                selectcolor=C["entry"], activebackground=C["bg"])
            i18n.bind(rb, "crew." + v)
            rb.pack(side="left")
        b = tk.Frame(self.w, bg=C["bg"])
        b.pack(fill="x", padx=10, pady=10)
        self.go = tk.Button(b, text=T("boat.connect"), command=self.connect, **theme.button_opts("ok"))
        self.go.pack(side="left")
        tk.Button(b, text=T("boat.close"), command=self.close, **theme.button_opts()).pack(side="left", padx=8)
        self.msg = lab(self.w, "", fg=C["bad_txt"], wraplength=620, justify="left")
        self.msg.pack(anchor="w", padx=10, pady=(0, 10))
        if not self.boats:
            self.msg.configure(text=T("boat.none", folder=folder or boats.BOATS_DIR))
        self.tick()

    def rows(self):
        out = []
        for bt in self.boats:
            st = self.prober.status(bt)             # "invalid" / "no_ip" without probing
            st = "checking" if st == "?" else st
            out.append((bt, st))
        return out

    def tick(self):
        try:
            sel = self.lb.curselection()
            self.lb.delete(0, "end")
            for i, (bt, st) in enumerate(self.rows()):
                # the NAME first; the address only as a detail at the end
                self.lb.insert("end", "%s    %s    · %s" % (bt.name, T("boat.st_" + st),
                                                          bt.ip if bt.ip_ok() else T("ov.ip_not_set")))
                self.lb.itemconfigure(i, fg=status_col(st))
            if sel:
                self.lb.selection_set(sel[0])
            self.w.after(1000, self.tick)
        except tk.TclError:
            pass

    def select(self, boat):
        """Pre-select `boat` (chosen in screen 2's boat dropdown)."""
        for i, bt in enumerate(self.boats):
            if bt.id == boat.id:
                self.lb.selection_clear(0, "end")
                self.lb.selection_set(i)
                self.user.focus_set()

    def selected(self):
        sel = self.lb.curselection()
        return self.rows()[sel[0]] if sel else (None, None)

    def connect(self):
        bt, st = self.selected()
        if bt is None:
            return self.msg.configure(text=T("boat.pick"))
        if st == "invalid":
            return self.msg.configure(text=self.app.boat_fail(bt, T("boat.invalid", name=bt.name, err=bt.error,
                                                                     file=bt.path)))
        if st == "no_ip":
            return self.msg.configure(text=self.app.boat_fail(bt, T("boat.no_ip", name=bt.name, file=bt.path)))
        if st != "online":
            return self.msg.configure(text=self.app.boat_fail(bt, T("boat.offline", name=bt.name)))
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
            return app.say(T("crew.no_partner"), theme.WARN)
        msg["with"] = cops[0]["id"]
    app.link.send_msg(msg)
    app.event(T("crew.swap_asked"))
