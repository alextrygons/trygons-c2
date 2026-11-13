"""The HARBOUR ROUTE window (teach and repeat, step HX6, CLAUDE.md Z; docs/harbour_design.md 8.2). Opened by the
«ΔΙΑΔΡΟΜΗ ΛΙΜΕΝΑ» button of the TRACKS window: a saved track you drove by hand out of / into the harbour -> a harbour
route (harbourroute.py: cut the ends, Douglas-Peucker to <= 50 waypoints, corridor) drawn on the map as a PREVIEW, checked by
the gateway (114 / 120 / 129), saved per harbour, sent through the NORMAL program upload (app.harbour_upload).
Sending only STORES the route on the boat; starting is the separate START with the captain's YES. DISPLAY + requests only:
this module sends nothing itself - the window calls app.harbour_check_request / app.harbour_upload (the app's senders).
"""
import os
import tkinter as tk
from tkinter import filedialog

import harbourroute
import i18n
import theme
from i18n import T

C = theme.C


class HarbourRouteWindow:
    def __init__(self, app):
        self.app = app
        self.track = None
        self.track_path = None
        self.route = None
        self.checked = False
        self.sample = False
        self.w = tk.Toplevel(app.root, bg=C["bg"])
        self.w.title(i18n.W("harb.rt_title"))
        self.w.protocol("WM_DELETE_WINDOW", self.close)
        theme.set_app_icon(self.w)
        tk.Label(self.w, text=T("harb.rt_head"), font=theme.font("title"), bg=C["bg"], fg=C["fg"]).pack(
            anchor="w", padx=10, pady=(10, 2))
        tk.Label(self.w, text=T("harb.rt_help"), font=theme.font("label"), bg=C["bg"], fg=C["fg"], justify="left",
                 wraplength=620).pack(anchor="w", padx=10)
        self.track_lbl = tk.Label(self.w, text=T("harb.rt_no_track"), font=theme.font("value"), bg=C["bg"], fg=C["fg"])
        self.track_lbl.pack(anchor="w", padx=10, pady=(8, 2))
        form = tk.Frame(self.w, bg=C["bg"])
        form.pack(fill="x", padx=10, pady=4)
        D = harbourroute.DEFAULTS
        self.vars = {"harbour": tk.StringVar(value="daskalio"), "name": tk.StringVar(value="exit"),
                     "from": tk.StringVar(value="0"), "to": tk.StringVar(value="100"),
                     "cut": tk.StringVar(value="%g" % D["cut_m"]), "corr": tk.StringVar(value="%g" % D["corr_m"]),
                     "tol": tk.StringVar(value="%g" % D["tol_m"])}
        self.entries = {}
        for r, key in enumerate(("harbour", "name", "from", "to", "cut", "corr", "tol")):
            tk.Label(form, text=T("harb.rt_" + key), font=theme.font("label"), bg=C["bg"], fg=C["fg"]).grid(
                row=r, column=0, sticky="w", padx=(0, 8), pady=2)
            e = tk.Entry(form, textvariable=self.vars[key], width=14)
            e.grid(row=r, column=1, sticky="w", pady=2)
            self.entries[key] = e
        b = tk.Frame(self.w, bg=C["bg"])
        b.pack(fill="x", padx=10, pady=6)
        self.buttons = {}
        for key, fn, kind in (("make", self.make, "ok"), ("check", self.check, "normal"), ("save", self.save, "normal"),
                              ("load", self.load, "normal"), ("send", self.send, "normal"),
                              ("close", self.close, "normal")):
            bt = tk.Button(b, text=T("harb.rt_btn_" + key), command=fn, **theme.button_opts(kind))
            bt.pack(side="left", padx=(0, 6), pady=4)
            self.buttons[key] = bt
        self.msg = tk.Label(self.w, text="", font=theme.font("label"), bg=C["bg"], fg=C["fg"], justify="left",
                            wraplength=620)
        self.msg.pack(anchor="w", padx=10, pady=(0, 4))
        self.warn = tk.Label(self.w, text="", font=theme.font("label"), bg=C["bg"], fg=C["note_txt"], justify="left",
                             wraplength=620)
        self.warn.pack(anchor="w", padx=10, pady=(0, 4))
        tk.Label(self.w, text=T("harb.rt_notice"), font=theme.font("small"), bg=C["bg"], fg=C["dim"],
                 justify="left", wraplength=620).pack(anchor="w", padx=10, pady=(0, 10))
        theme.style_tree(self.w)

    # --- state ----------------------------------------------------------------------------------------------------
    def alive(self):
        try:
            return bool(self.w.winfo_exists())
        except tk.TclError:
            return False

    def say(self, text, bad=False):
        self.msg.configure(text=text, fg=C["bad_txt"] if bad else C["fg"])

    def set_track(self, path):
        try:
            self.track = self.app.tracks.load(path)
        except OSError as e:
            self.track = None
            return self.say(T("harb.rt_load_error", err=e), True)
        self.track_path = path
        s = self.track.stats()
        self.track_lbl.configure(text="%s · %d · %.1f nm" % (os.path.basename(path)[:-6], s["n"], s["nm"]))
        return None

    def _num(self, key, lo, hi, default):
        try:
            v = float(self.vars[key].get().replace(",", "."))
        except ValueError:
            return default
        return max(lo, min(hi, v))

    def opts(self):
        return {"cut_m": self._num("cut", 0.0, 200.0, harbourroute.DEFAULTS["cut_m"]),
                "corr_m": self._num("corr", 4.0, harbourroute.DEFAULTS["corr_max_m"], harbourroute.DEFAULTS["corr_m"]),
                "tol_m": self._num("tol", 0.5, 20.0, harbourroute.DEFAULTS["tol_m"])}

    # --- the buttons ------------------------------------------------------------------------------------------------
    def make(self):
        if self.track is None:
            return self.say(T("harb.rt_no_track"), True)
        n = len(self.track.pts)
        a = int(self._num("from", 0.0, 100.0, 0.0) / 100.0 * (n - 1))
        z = int(self._num("to", 0.0, 100.0, 100.0) / 100.0 * (n - 1))
        o = self.opts()
        o["cut_m"] = o["cut_m"]
        r = harbourroute.from_track(self.track.pts, self.vars["harbour"].get(), self.vars["name"].get(),
                                    {"cut_m": o["cut_m"], "corr_m": o["corr_m"], "tol_m": o["tol_m"]}, a, max(a, z),
                                    source=os.path.basename(self.track_path or ""))
        if r["route"] is None:
            self.route, self.checked = None, False
            self.app.harbour_preview = None
            self.warn.configure(text="")
            self.app.render(force=True)
            return self.say(T("harb.rt_too_short", cut="%.0f" % o["cut_m"]), True)
        self.set_route(r["route"], r["tol_m"])
        return None

    def set_route(self, route, tol=None, sample=False):
        self.route, self.checked, self.sample = route, False, sample
        self.app.harbour_check_seq = getattr(self.app, "harbour_check_seq", 0) + 1     # an older check's ack counts for nothing
        self.app.harbour_preview = route
        text = T("harb.rt_made", n=len(route["wps"]), len="%.0f" % harbourroute.length_m(route),
                 tol="%.1f" % (tol if tol is not None else route.get("tol_m") or 0.0), corr="%.0f" % route["corr_m"])
        if sample:
            text += " · " + T("harb.rt_sample")
        self.say(text)
        self.warn.configure(text="\n".join(self._warnings(route)))
        self.app.render(force=True)
        try:
            self.app.mapwin.redraw()
        except AttributeError:
            pass

    def _warnings(self, route):
        out = [T("harb.rt_" + k, **a) for k, a in harbourroute.quay_warnings(route, self.app.harbour_shore_dist)]
        out += [T("harb.rt_" + k, **a) for k, a in harbourroute.local_check(route)]
        return out

    def check(self):
        if self.route is None:
            return self.say(T("harb.rt_check_first"), True)
        bad = harbourroute.local_check(self.route)
        if bad:
            self.checked = False
            return self.say(T("harb.rt_check_refused", code=129, why="; ".join(T("harb.rt_" + k, **a) for k, a in bad)),
                            True)
        if not self.app.harbour_check_request(self.route):
            return self.say(T("harb.rt_no_gateway"), True)
        return self.say(T("harb.rt_checking"))

    def on_ack(self, d):
        self.checked = bool(d.get("ok"))
        if d.get("ok"):
            self.say(T("harb.rt_check_ok"))
        else:
            self.say(T("harb.rt_check_refused", code=d.get("code", "?"), why=i18n.from_en(d.get("why", ""))), True)

    def save(self):
        if self.route is None:
            return self.say(T("harb.rt_check_first"), True)
        self.route["harbour"] = self.vars["harbour"].get()
        self.route["name"] = self.vars["name"].get()
        try:
            p = harbourroute.save(self.route)
        except OSError as e:
            return self.say(T("harb.rt_save_error", err=e), True)
        return self.say(T("harb.rt_saved", path=p))

    def load(self, path=None):
        if path is None:
            path = filedialog.askopenfilename(parent=self.w, initialdir=harbourroute.ROUTES_DIR if os.path.isdir(
                harbourroute.ROUTES_DIR) else None, filetypes=[("JSON", "*.json")])
        if not path:
            return None
        try:
            r = harbourroute.load(path)
        except ValueError as e:
            return self.say(T("harb.rt_load_error", err=e), True)
        sample = bool(r.get("not_for_navigation")) or os.path.basename(os.path.dirname(os.path.dirname(path))) == "_samples"
        self.vars["harbour"].set(str(r.get("harbour") or ""))
        self.vars["name"].set(str(r.get("name") or ""))
        self.vars["corr"].set("%g" % r["corr_m"])
        self.set_route(r, sample=sample)
        self.say(T("harb.rt_loaded", name=r.get("name") or "?", n=len(r["wps"])) +
                 ((" · " + T("harb.rt_sample")) if sample else ""))
        return r

    def send(self):
        """The normal program upload with the harbour flag - only after the gateway said OK (CHECK) and only the route
        as it was checked (any MAKE / LOAD resets the check). It never starts the boat."""
        if self.sample:
            return self.say(T("harb.rt_sample"), True)          # a committed TEST sample is never sent to a boat
        if self.route is None or not self.checked:
            return self.say(T("harb.rt_check_first"), True)
        if self.app.harbour_upload(self.route):
            self.say(T("harb.rt_sent", n=len(self.route["wps"])))
            self.app.harbour_preview = None                      # now it is the boat's route (drawn from harbour_route)
            self.app.render(force=True)
        return None

    def close(self):
        self.app.harbour_preview = None
        try:
            self.w.destroy()
        except tk.TclError:
            pass
        try:
            self.app.render(force=True)
        except Exception:
            pass
