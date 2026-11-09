"""The search pattern panel «ΜΟΤΙΒΑ ΕΡΕΥΝΑΣ» (CLAUDE.md X, docs/sar_design.md 4; step PF2). DISPLAY + REQUESTS only.

  PatternCtl(app)    the logic (no Tk): datum, parameters, the plan (sar_pattern), the preview on the map, send()
  PatternPanel(app)  the window: kind, datum (map click / MOB / boat / typed), spacing, first leg, size, speed, altitude

Nothing moves a boat or a UAV from here by itself:
  * the plan is DRAWN on the map first (numbered legs, length, time, area);
  * SEND asks for the target (the "Send route" box: ΣΚΑΦΟΣ / ΕΠΟΜΕΝΟ UAV / a UAV), then for a YES (the captain's own, on
    screen 1, or the single operator's; a copilot's boat route additionally waits for the captain's ACCEPT in the
    gateway - the normal route upload);
  * the boat route is the normal program upload (shallow water, geofence, 50 waypoints, max speed) and the AUTO start
    rules stay as they are; the UAV mission goes through uav_route (energy, range, altitude, no-fly) and the launch
    approval is the captain's, as always;
  * more than 50 waypoints for the boat = REFUSED with the count and the choice «cut at 50» (it says on which leg);
    never cut silently.
"""
import math
import time

import sar_geo
import sar_pattern
import settings
import uavroute
import theme
import ynbox
from i18n import T

RED, ORANGE = theme.WARN, theme.CAUTION

DEFAULT_ALT_M = 50.0
KIND_NAMES = {"ss": "pat.k_ss", "vs": "pat.k_vs", "ps": "pat.k_ps", "cs": "pat.k_cs"}


class PatternCtl:
    def __init__(self, app):
        self.app = app
        self.kind = "ss"
        self.datum = None               # (lat, lon)
        self.datum_src = ""             # "map" | "mob" | "boat" | "typed"
        self.p = {"S": 100.0, "C": 0.0, "R": 500.0, "W": 500.0, "Lg": 1000.0, "kn": 5.0, "turn": "starboard"}
        self.alt = DEFAULT_ALT_M
        self.plan = None
        self.why = None                 # the last reason there is no plan / no send
        self.cut = False                # the operator chose «cut at 50»
        self.pending = None             # the plan waiting for the captain's YES
        self.asked = 0

    # --- the inputs -------------------------------------------------------------------------------------
    def set_kind(self, kind):
        if kind in sar_pattern.KINDS:
            self.kind = kind
            self.cut = False
            self.compute()

    def set_datum(self, lat, lon, src="typed"):
        self.datum, self.datum_src, self.cut = (float(lat), float(lon)), src, False
        self.compute()

    def set_param(self, key, value):
        try:
            v = float(value) if key != "turn" else ("port" if value == "port" else "starboard")
        except (TypeError, ValueError):
            self.plan, self.why = None, "input"
            self.app.sar_preview = None
            return
        self.p[key] = v
        self.cut = False
        self.compute()

    def set_alt(self, v):
        try:
            self.alt = max(10.0, min(120.0, float(v)))
        except (TypeError, ValueError):
            pass

    def default_course(self):
        """The first leg: into the current when it is known (IAMSAR: the datum drifts WITH the current, the search runs
        up-current first - the design says the current's set reversed), else the typed course."""
        import mob
        cur = mob.current_of(self.app.t)
        if cur and cur.get("set_deg") is not None and (cur.get("conf") or 0.0) >= 0.5:
            return (cur["set_deg"] + 180.0) % 360.0
        return None

    # --- the plan ---------------------------------------------------------------------------------------
    def params(self):
        q = dict(self.p)
        if self.kind == "ss":
            q["R"] = self.p["R"]
        return q

    def compute(self):
        if self.datum is None:
            self.plan, self.why = None, "datum"
        else:
            pl = sar_pattern.plan(self.kind, self.datum, self.params())
            if pl.get("why"):
                self.plan, self.why = None, pl["why"]
            else:
                self.plan, self.why = pl, None
        self.app.sar_preview = self.plan
        try:
            self.app.mapwin.redraw()
        except Exception:
            pass
        return self.plan

    def clear(self):
        self.plan, self.datum, self.pending = None, None, None
        self.app.sar_preview = None
        try:
            self.app.mapwin.redraw()
        except Exception:
            pass

    def lines(self):
        """The panel's text under the inputs: counts or the reason there is no plan."""
        if self.plan is None:
            return T("pat.why_" + (self.why or "datum"))
        s = sar_pattern.summary(self.plan)
        t = T("pat.summary", n=s["n"], legs=s["nlegs"], nm="%.2f" % s["nm"], m="%d" % round(s["m"]),
              min="%d" % round(s["min"]), area="%.2f" % s["area_nm2"])
        uc = sar_pattern.uav_check(self.plan)
        if uc:
            t += " " + T("pat.uav_over", n=uc["n"], max=uc["max"])
        if self.plan["n"] > sar_pattern.MAX_BOAT_WP:
            t += " " + T("pat.cut_note" if self.cut else "pat.boat_over", n=self.plan["n"],
                         max=sar_pattern.MAX_BOAT_WP, leg=sar_pattern.MAX_BOAT_WP - 1)
        return t

    # --- sending ----------------------------------------------------------------------------------------
    def send(self):
        """SEND: the target box (when a UAV magazine exists), then the YES. Nothing leaves before it."""
        a = self.app
        if self.plan is None:
            a.say(T("pat.why_" + (self.why or "datum")))
            return False
        uav = (a.gw_status or {}).get("uav")
        if uavroute.magazine_on(uav) and a.gw_is_gateway():
            uavroute.TargetBox(a.root, uav, self.choose)
            return True
        return self.choose("boat")

    def choose(self, target):
        """The target is chosen: check, then ask. Returns True when a question was asked."""
        a = self.app
        pl = self.plan
        if pl is None:
            return False
        if target == "boat":
            chk = sar_pattern.boat_check(pl)
            if chk and self.cut:
                pl = sar_pattern.cut_at_limit(pl)           # the operator's explicit choice, said on the panel
                chk = None
            if chk and chk["why"] == "short_leg":
                a.say(T("pat.short_leg", m="%d" % chk["m"], min="%d" % chk["min"]), ORANGE)
                a.event(T("pat.ev_refused", n=pl["n"]))
                self.why = "short_leg"
                return False
            if chk:
                a.say(T("pat.too_many", n=chk["n"], max=chk["max"], leg=chk["cut_leg"]), ORANGE)
                a.event(T("pat.ev_refused", n=chk["n"]))
                self.why = "too_many"
                return False
        elif not a.gw_is_gateway():
            a.say(T("uav.not_connected"), RED)
            return False
        else:
            uc = sar_pattern.uav_check(pl)                  # PF2-r MED-2: the limit is said BEFORE the YES
            if uc:
                a.say(T("pat.uav_over", n=uc["n"], max=uc["max"]), ORANGE)
                a.event(T("pat.ev_refused", n=uc["n"]))
                self.why = "uav_too_many"
                return False
        self.pending = (target, pl)
        text = T("pat.ask_" + ("boat" if target == "boat" else "uav"), kind=T(KIND_NAMES[pl["kind"]]),
                 n=pl["n"], nm="%.2f" % (pl["total_m"] / sar_pattern.M_PER_NM),
                 min="%d" % round(pl["time_s"] / 60.0), who=(T("uavr.boat") if target == "boat" else str(target)))
        self.asked += 1
        return self.ask(text, lambda t=target, p=pl: self.confirmed(t, p))

    def ask(self, text, yes):
        """The captain's YES: screen 1's vertical YES / NO when it is open, else this station's own box."""
        a = self.app
        drv = getattr(a, "driver")
        if drv is not None and not drv.question:
            drv.ask(text, yes, lambda: setattr(self, "pending", None), 60.0, tag=("pat", "send"), head=T("pat.title"))
            return True
        if ynbox.ask(a.root, T("pat.title"), text):
            yes()
        else:
            self.pending = None
        return True

    def confirmed(self, target, pl):
        """YES: the boat route through the NORMAL upload, or the UAV mission through the normal uav_route."""
        a = self.app
        self.pending = None
        if target == "boat":
            a.prog = sar_pattern.to_boat_program(pl)
            a.route_dirty = True
            a.prog_refresh()
            a.say(T("pat.sent_boat", n=pl["n"]))
            a.event(T("pat.ev_boat", kind=T(KIND_NAMES[pl["kind"]]), n=pl["n"]))
            a.prog_upload(final=a.prog["final"])
        else:
            a.uav_route(target, sar_pattern.uav_points(pl, self.alt))
            a.event(T("pat.ev_uav", kind=T(KIND_NAMES[pl["kind"]]), n=pl["n"], who=str(target)))


def open_panel(app, datum=None, src="map"):
    """Opens (or raises) the panel; a datum given = set it. -> the PatternCtl."""
    ctl = app.sar_ctl
    if datum is not None:
        ctl.set_datum(datum[0], datum[1], src)
    try:
        if app.sar_panel is not None and app.sar_panel.top.winfo_exists():
            app.sar_panel.top.lift()
            app.sar_panel.refresh()
            return ctl
    except Exception:
        pass
    app.sar_panel = PatternPanel(app)
    return ctl


class PatternPanel:
    def __init__(self, app):
        import tkinter as tk
        self.app, self.ctl, self.tk = app, app.sar_ctl, tk
        C = theme.C
        self.top = tk.Toplevel(app.root, bg=C["bg"])
        self.top.title(T("pat.title"))
        try:
            self.top.transient(app.root)
        except tk.TclError:
            pass
        self.top.protocol("WM_DELETE_WINDOW", self.close)
        self.v = {}
        row = tk.Frame(self.top, bg=C["bg"])
        row.pack(fill="x", padx=10, pady=(8, 2))
        self.kind_var = tk.StringVar(value=self.ctl.kind)
        for k in sar_pattern.KINDS:
            tk.Radiobutton(row, text=T(KIND_NAMES[k]), value=k, variable=self.kind_var, command=self.on_kind,
                           bg=C["bg"], fg=C["fg"], selectcolor=C["panel"], activebackground=C["bg"],
                           font=theme.font("small")).pack(side="left", padx=4)
        self.note = tk.Label(self.top, text="", font=theme.font("small"), bg=C["bg"], fg=C["fg"], justify="left",
                             wraplength=420, anchor="w")
        self.note.pack(fill="x", padx=10)
        self.grid = tk.Frame(self.top, bg=C["bg"])
        self.grid.pack(fill="x", padx=10, pady=4)
        self.fields = {}
        self.build_fields()
        db = tk.Frame(self.top, bg=C["bg"])
        db.pack(fill="x", padx=10, pady=2)
        self.datum_lbl = tk.Label(db, text="", font=theme.font("small"), bg=C["bg"], fg=C["fg"], anchor="w")
        self.datum_lbl.pack(fill="x")
        for text, fn in (("pat.d_map", self.pick_map), ("pat.d_mob", self.pick_mob), ("pat.d_boat", self.pick_boat)):
            tk.Button(db, text=T(text), command=fn, **theme.button_opts()).pack(side="left", padx=(0, 4), pady=2)
        self.sum = tk.Label(self.top, text="", font=theme.font("small"), bg=C["bg"], fg=C["fg"], justify="left",
                            wraplength=420, anchor="w")
        self.sum.pack(fill="x", padx=10, pady=4)
        bb = tk.Frame(self.top, bg=C["bg"])
        bb.pack(fill="x", padx=10, pady=(2, 10))
        self.cut_btn = tk.Button(bb, text=T("pat.cut"), command=self.on_cut, **theme.button_opts())
        tk.Button(bb, text=T("pat.send"), command=self.ctl.send, **theme.button_opts()).pack(side="left", padx=(0, 4))
        self.cut_btn.pack(side="left", padx=(0, 4))
        tk.Button(bb, text=T("pat.clear"), command=self.on_clear, **theme.button_opts()).pack(side="left", padx=(0, 4))
        tk.Button(bb, text=T("pat.close"), command=self.close, **theme.button_opts()).pack(side="right")
        self.refresh()

    def build_fields(self):
        tk = self.tk
        C = theme.C
        for w in self.grid.winfo_children():
            w.destroy()
        self.fields.clear()
        kind = self.ctl.kind
        rows = [("S", "pat.f_S"), ("C", "pat.f_C")]
        if kind == "ss":
            rows.append(("R", "pat.f_R_ss"))
        elif kind == "vs":
            rows.append(("R", "pat.f_R_vs"))
        else:
            rows += [("W", "pat.f_W"), ("Lg", "pat.f_Lg")]
        rows += [("kn", "pat.f_kn"), ("alt", "pat.f_alt")]
        for i, (key, lbl) in enumerate(rows):
            tk.Label(self.grid, text=T(lbl), font=theme.font("small"), bg=C["bg"], fg=C["fg"], anchor="w").grid(
                row=i, column=0, sticky="w")
            val = self.ctl.alt if key == "alt" else self.ctl.p[key]
            var = tk.StringVar(value=("%g" % val))
            e = tk.Entry(self.grid, textvariable=var, width=9, font=theme.font("small"))
            e.grid(row=i, column=1, padx=6, pady=1)
            e.bind("<Return>", lambda _e, k=key: self.on_field(k))
            e.bind("<FocusOut>", lambda _e, k=key: self.on_field(k))
            self.fields[key] = var
        n = len(rows)
        self.turn_var = tk.StringVar(value=self.ctl.p["turn"])
        tr = tk.Frame(self.grid, bg=C["bg"])
        tr.grid(row=n, column=0, columnspan=2, sticky="w")
        for v, lbl in (("starboard", "pat.t_stbd"), ("port", "pat.t_port")):
            tk.Radiobutton(tr, text=T(lbl), value=v, variable=self.turn_var, command=self.on_turn, bg=C["bg"],
                           fg=C["fg"], selectcolor=C["panel"], activebackground=C["bg"],
                           font=theme.font("small")).pack(side="left")

    def on_kind(self):
        self.ctl.set_kind(self.kind_var.get())
        self.build_fields()
        self.refresh()

    def on_field(self, key):
        v = self.fields[key].get().replace(",", ".")
        if key == "alt":
            self.ctl.set_alt(v)
        else:
            self.ctl.set_param(key, v)
        self.refresh()

    def on_turn(self):
        self.ctl.set_param("turn", self.turn_var.get())
        self.refresh()

    def on_cut(self):
        self.ctl.cut = True
        self.ctl.compute()
        self.refresh()

    def on_clear(self):
        self.ctl.clear()
        self.refresh()

    def pick_map(self):
        """The next map click is the datum (one shot)."""
        a = self.app
        prev = a.map_click

        def got(lat, lon):
            a.map_click = prev
            self.ctl.set_datum(lat, lon, "map")
            self.refresh()
        a.map_click = got
        a.say(T("pat.click_map"))

    def pick_mob(self):
        ms = self.app.sar.open_mobs()
        if not ms:
            return self.app.say(T("pat.no_mob"), ORANGE)
        m = ms[-1]
        self.ctl.set_datum(m["lat"], m["lon"], "mob")
        self.refresh()

    def pick_boat(self):
        d = self.app.t or {}
        if not d.get("pos_source"):
            return self.app.say(T("map.no_position"), RED)
        self.ctl.set_datum(d["est_lat"], d["est_lon"], "boat")
        self.refresh()

    def refresh(self):
        c = self.ctl
        try:
            self.note.configure(text=T("pat.n_" + c.kind))
            self.datum_lbl.configure(text=(T("pat.datum", lat="%.5f" % c.datum[0], lon="%.5f" % c.datum[1],
                                             src=T("pat.src_" + (c.datum_src or "typed"))) if c.datum
                                           else T("pat.why_datum")))
            self.sum.configure(text=c.lines())
            self.cut_btn.configure(state="normal" if (c.plan and c.plan["n"] > sar_pattern.MAX_BOAT_WP)
                                   else "disabled")
        except Exception:
            pass

    def close(self):
        try:
            self.ctl.clear()
            self.top.destroy()
        except Exception:
            pass
        self.app.sar_panel = None
