"""The practical test windows (ACADEMY_SPEC G + J, step 21) - opened by the GROUND STATION (menu Academy ->
Practical test / Test area), inside the station's own process, with linkview.LinkView over its boat link:

  * PracticalWindow: the examiner's own login (password + 2FA), the trainee, the kind (upgrade / check ride /
    refresher), the shown start checks (calm water, start power limit, examiner on board, inside the area, wind,
    hours ...), the tasks (seen in the telemetry -> approved by the examiner), the examiner's ALARM (this screen
    only), Abort, and the final approval = CONFIRM_ONLY on the onboard joystick. Then the level change
    (accounts.Store.grant_practical), audit-logged. Leaving the area = alert + test stopped (nothing is sent).
  * AreaWindow: the instructor tool for the test area: map taps, the boat's GPS "add point", .geojson import,
    the ramp top / bottom; CAPTAIN only (2FA), audit-logged; never a chart.
Neither window sends anything but CONFIRM_ONLY (LinkView); both show the banner PRACTICAL TEST - REAL BOAT.
"""
import datetime
import math
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
ACADEMY = os.path.dirname(HERE)
for _d in (ACADEMY, HERE):
    if _d not in sys.path:
        sys.path.append(_d)

import accounts                                                         # noqa: E402
import auth                                                             # noqa: E402
import ptarea                                                           # noqa: E402
import ptest                                                            # noqa: E402
import simguard                                                         # noqa: E402
import text                                                             # noqa: E402
from text import T                                                      # noqa: E402

WARN = "#c0392b"
BG, FG, DIM = "#0e141b", "#e6edf3", "#8fa3b5"
POLL_MS = 200
TASK_STATES = ("todo", "seen", "approved", "failed")


def banner_text():
    return T("pt.banner")


class LoginBox(object):
    """User + password + 2FA code -> a Session (store.login)."""

    def __init__(self, parent, store, on_login, role_key):
        self.store, self.on_login = store, on_login
        self.session = None
        f = tk.LabelFrame(parent, text=T(role_key), bg=BG, fg=FG)
        f.pack(side="top", fill="x", padx=8, pady=4)
        self.v_user, self.v_pw, self.v_code = tk.StringVar(), tk.StringVar(), tk.StringVar()
        for i, (k, v, show) in enumerate((("pt.user", self.v_user, ""), ("pt.password", self.v_pw, "*"),
                                          ("pt.code", self.v_code, ""))):
            tk.Label(f, text=T(k), bg=BG, fg=FG).grid(row=0, column=2 * i, sticky="e", padx=2)
            tk.Entry(f, textvariable=v, show=show, width=14).grid(row=0, column=2 * i + 1, padx=2)
        self.b = tk.Button(f, text=T("pt.login"), command=self.login)
        self.b.grid(row=0, column=6, padx=4)
        self.l = tk.Label(f, text=T("pt.not_logged"), bg=BG, fg=DIM)
        self.l.grid(row=1, column=0, columnspan=7, sticky="w")

    def login(self):
        try:
            s = self.store.login(self.v_user.get().strip(), self.v_pw.get(), self.v_code.get().strip())
        except auth.AuthError as e:
            self.session = None
            self.l.configure(text=T("pt.login_failed", why=str(e)), fg=WARN)
            return None
        why = self.store.session_ok(s, role="examiner")
        if why:
            self.session = None
            self.l.configure(text=T("pt.login_failed", why=why), fg=WARN)
            return None
        self.session = s
        self.v_pw.set("")
        self.v_code.set("")
        self.l.configure(text=T("pt.logged", user=s.user), fg=FG)
        self.on_login(s)
        return s


class PracticalWindow(object):
    def __init__(self, top, view, store, cfg=None, now=time.time, when=datetime.datetime.now):
        self.top, self.view, self.store, self.now, self.when = top, view, store, now, when
        self.cfg = cfg or ptest.config()
        self.test = None
        self.closed = False
        self.granted = None
        top.title(T("pt.title"))
        top.configure(bg=BG)
        self.banner = tk.Label(top, text=banner_text(), bg=WARN, fg="#ffffff", font=("Segoe UI", 15, "bold"), pady=5)
        self.banner.pack(side="top", fill="x")
        tk.Label(top, text=T("pt.observe_only"), bg=BG, fg=DIM, wraplength=900, justify="left").pack(
            side="top", anchor="w", padx=8)
        self.login = LoginBox(top, store, lambda s: self.say(T("pt.logged", user=s.user)), "pt.examiner")
        f = tk.Frame(top, bg=BG)
        f.pack(side="top", fill="x", padx=8, pady=4)
        tk.Label(f, text=T("pt.trainee"), bg=BG, fg=FG).grid(row=0, column=0, sticky="e")
        self.v_trainee = tk.StringVar()
        self.cb_trainee = ttk.Combobox(f, textvariable=self.v_trainee, values=sorted(store.accounts), width=16)
        self.cb_trainee.grid(row=0, column=1, sticky="w", padx=4)
        self.v_kind = tk.StringVar(value="upgrade")
        for i, k in enumerate(ptest.KINDS):
            tk.Radiobutton(f, text=T("pt.kind_%s" % k), variable=self.v_kind, value=k, bg=BG, fg=FG,
                           selectcolor=BG).grid(row=0, column=2 + i, padx=4)
        tk.Label(f, text=T("pt.wind_kn"), bg=BG, fg=FG).grid(row=1, column=0, sticky="e")
        self.v_wind = tk.StringVar()
        tk.Entry(f, textvariable=self.v_wind, width=6).grid(row=1, column=1, sticky="w", padx=4)
        tk.Label(f, text=T("pt.wind_from"), bg=BG, fg=FG).grid(row=1, column=2, sticky="e")
        self.v_from = tk.StringVar()
        tk.Entry(f, textvariable=self.v_from, width=6).grid(row=1, column=3, sticky="w", padx=4)
        self.ticks = {}
        for i, k in enumerate(("calm", "on_board", "wind")):
            v = tk.BooleanVar(value=False)
            tk.Checkbutton(f, text=T("pt.tick_%s" % k), variable=v, bg=BG, fg=FG, selectcolor=BG).grid(
                row=2, column=i * 2, columnspan=2, sticky="w")
            self.ticks[k] = v
        bar = tk.Frame(top, bg=BG)
        bar.pack(side="top", fill="x", padx=8, pady=4)
        self.buttons = {}
        for k, fn in (("pt.check", self.check), ("pt.start", self.start), ("pt.approve", self.approve),
                      ("pt.fail", self.fail), ("pt.alarm", self.alarm), ("pt.abort", self.abort),
                      ("pt.final", self.final)):
            b = tk.Button(bar, text=T(k), command=fn)
            b.pack(side="left", padx=3)
            self.buttons[k] = b
        self.tasks = ttk.Treeview(top, columns=("task", "state", "note"), show="headings", height=7)
        for c, w in (("task", 300), ("state", 110), ("note", 380)):
            self.tasks.heading(c, text=T("pt.col_%s" % c))
            self.tasks.column(c, width=w, anchor="w")
        self.tasks.pack(side="top", fill="x", padx=8)
        self.l_status = tk.Label(top, text="", bg=BG, fg=FG, anchor="w", justify="left")
        self.l_status.pack(side="top", fill="x", padx=8)
        self.l_alert = tk.Label(top, text="", bg=BG, fg=WARN, anchor="w", font=("Segoe UI", 13, "bold"))
        self.l_alert.pack(side="top", fill="x", padx=8)
        self.msg = tk.Label(top, text="", bg=BG, fg=FG, anchor="w", justify="left", wraplength=900)
        self.msg.pack(side="top", fill="x", padx=8, pady=(0, 6))
        top.protocol("WM_DELETE_WINDOW", self.close)
        self.top.after(POLL_MS, self.poll)

    # --- helpers
    def say(self, s, colour=None):
        self.msg.configure(text=s, fg=colour or FG)

    def new_test(self):
        s = self.login.session
        try:
            area = ptarea.load(self.store)
        except FileNotFoundError:
            return None
        return ptest.Test(self.store, area, self.v_kind.get(), self.v_trainee.get().strip(), s, self.cfg)

    def tick_values(self):
        return {k: bool(v.get()) for k, v in self.ticks.items()}

    def draw_tasks(self):
        self.tasks.delete(*self.tasks.get_children())
        if not self.test:
            return
        tt = self.cfg.get("tasks", {})
        lang = text.lang()
        for t in self.test.tasks:
            title = (tt.get(t["id"]) or {}).get("title_" + lang, t["id"])
            self.tasks.insert("", "end", iid=t["id"], values=(title, T("pt.state_%s" % t["state"]), t["note"]))

    def selected_task(self):
        sel = self.tasks.selection()
        return sel[0] if sel else None

    # --- actions
    def check(self):
        tst = self.new_test()
        if tst is None:
            return self.say(T("pt.no_area"), WARN)
        t, rx = self.view.telemetry()
        age = (self.now() - rx) if t else None
        probs = tst.start_problems(t, age, self.tick_values(), self.v_wind.get(), self.v_from.get(), self.when())
        self.say(T("pt.problems", list="\n".join("- " + p for p in probs)) if probs else T("pt.ready"),
                 WARN if probs else None)
        return probs

    def start(self):
        if self.test and self.test.state in ("running", "confirming"):
            return self.say(T("pt.running"))
        tst = self.new_test()
        if tst is None:
            return self.say(T("pt.no_area"), WARN)
        t, rx = self.view.telemetry()
        age = (self.now() - rx) if t else None
        try:
            tst.start(t, age, self.tick_values(), self.v_wind.get(), self.v_from.get(), self.when(), now=self.now())
        except simguard.Refused as e:
            return self.say(T("pt.refused", why=str(e)), WARN)
        self.test = tst
        self.view.audit()                                   # (only what comes after the start)
        self.granted = None
        self.l_alert.configure(text="")
        self.say(T("pt.started", id=tst.id, file=tst.recording))
        self.draw_tasks()
        return tst

    def approve(self):
        tid = self.selected_task()
        if not self.test or not tid:
            return self.say(T("pt.pick_task"))
        if not self.test.approve(tid):
            return self.say(T("pt.not_seen"), WARN)
        self.draw_tasks()

    def fail(self):
        tid = self.selected_task()
        if not self.test or not tid:
            return self.say(T("pt.pick_task"))
        self.test.fail(tid, T("pt.failed_by_examiner"))
        self.draw_tasks()

    def alarm(self):
        if not self.test or not self.test.trigger_alarm(self.now()):
            return self.say(T("pt.alarm_not_now"), WARN)
        self.l_alert.configure(text=T("pt.alarm_shown"))
        try:
            self.top.bell()
        except tk.TclError:
            pass

    def abort(self):
        if self.test and self.test.abort():
            self.say(T("pt.aborted"))
            self.draw_tasks()

    def final(self):
        if not self.test or not self.test.all_approved():
            return self.say(T("pt.not_all"), WARN)
        tok = self.test.final_request(self.view, self.now())
        if tok is None:
            return self.say(T("pt.not_all"), WARN)
        self.say(T("pt.confirm_now", token=tok))

    def poll(self):
        if self.closed:
            return
        try:
            self.step()
        finally:
            if not self.closed:
                self.top.after(POLL_MS, self.poll)

    def step(self):
        t, rx = self.view.telemetry()
        tst = self.test
        if tst is None:
            self.l_status.configure(text=T("pt.idle"))
            return
        now = self.now()
        before = tst.state
        tst.audit(self.view.audit())
        alert = tst.update(t, (now - rx) if t else None, now)
        if alert:
            self.l_alert.configure(text=T("pt.alert", why=alert))
        pos = ptest.position(t)
        dr = tst._dist_ramp(pos)
        self.l_status.configure(text=T("pt.status", state=T("pt.st_%s" % tst.state), mode=(t or {}).get("mode_name", "-"),
                                       ramp=ptest.dist_text(dr),
                                       inside=T("acc.yes") if pos and tst.area.inside(*pos) else T("acc.no")))
        if tst.state != before:
            self.draw_tasks()
            if tst.state == "running" and before == "confirming":
                self.say(T("pt.no_confirm"), WARN)
            if tst.state == "passed" and self.granted is None:
                self.grant()
        elif any(x["state"] == "seen" for x in tst.tasks):
            self.draw_tasks()

    def grant(self):
        try:
            self.granted = self.store.grant_practical(self.test.result(), self.login.session)
            self.say(T("pt.passed", user=self.test.trainee, level=self.granted.level,
                       expiry=self.granted.expiry.isoformat() if self.granted.expiry else "-"))
        except simguard.Refused as e:
            self.granted = False
            self.say(T("pt.refused", why=str(e)), WARN)

    def close(self):
        if self.test and self.test.state in ("running", "confirming"):
            self.test.abort("window closed")
        self.closed = True
        try:
            self.top.destroy()
        except tk.TclError:
            pass


class AreaWindow(object):
    """The instructor tool for the test area (map taps, boat GPS, .geojson, ramp points). Display + file only."""
    W, H = 640, 480

    def __init__(self, top, view, store):
        self.top, self.view, self.store = top, view, store
        self.closed = False
        try:
            self.base = ptarea.load(store)
        except FileNotFoundError:
            self.base = ptarea.Area(ptarea.load(None, ptarea.CONTENT_AREA).doc)
        self.draft = ptarea.Draft(self.base)
        pts = self.base.polygon or [[37.78, 24.04]]
        self.c_lat = sum(p[0] for p in pts) / len(pts)
        self.c_lon = sum(p[1] for p in pts) / len(pts)
        self.m_per_px = 5.0
        self.tap = "corner"
        top.title(T("ar.title"))
        top.configure(bg=BG)
        self.banner = tk.Label(top, text=banner_text(), bg=WARN, fg="#ffffff", font=("Segoe UI", 15, "bold"), pady=5)
        self.banner.pack(side="top", fill="x")
        tk.Label(top, text=T("ar.not_chart"), bg=BG, fg=DIM, wraplength=760, justify="left").pack(side="top", anchor="w",
                                                                                                 padx=8)
        self.login = LoginBox(top, store, lambda s: self.say(T("pt.logged", user=s.user)), "ar.who")
        bar = tk.Frame(top, bg=BG)
        bar.pack(side="top", fill="x", padx=8)
        self.buttons = {}
        for k, fn in (("ar.gps_point", self.gps_point), ("ar.ramp_top_gps", lambda: self.gps_ramp("top")),
                      ("ar.ramp_bottom_gps", lambda: self.gps_ramp("bottom")),
                      ("ar.tap_corner", lambda: self.set_tap("corner")), ("ar.tap_top", lambda: self.set_tap("top")),
                      ("ar.tap_bottom", lambda: self.set_tap("bottom")), ("ar.undo", self.undo),
                      ("ar.geojson", self.geojson), ("ar.save", self.save)):
            b = tk.Button(bar, text=T(k), command=fn)
            b.pack(side="left", padx=2)
            self.buttons[k] = b
        self.cv = tk.Canvas(top, width=self.W, height=self.H, bg="#10202c", highlightthickness=0)
        self.cv.pack(side="top", padx=8, pady=4)
        self.cv.bind("<Button-1>", self.on_tap)
        self.msg = tk.Label(top, text="", bg=BG, fg=FG, anchor="w", justify="left", wraplength=760)
        self.msg.pack(side="top", fill="x", padx=8, pady=(0, 6))
        top.protocol("WM_DELETE_WINDOW", self.close)
        self.draw()

    def say(self, s, colour=None):
        self.msg.configure(text=s, fg=colour or FG)

    # local flat projection around the centre: x east, y north (m) -> canvas
    def to_xy(self, lat, lon):
        dy = (lat - self.c_lat) * 111320.0
        dx = (lon - self.c_lon) * 111320.0 * math.cos(math.radians(self.c_lat))
        return self.W / 2 + dx / self.m_per_px, self.H / 2 - dy / self.m_per_px

    def to_ll(self, x, y):
        dx, dy = (x - self.W / 2) * self.m_per_px, (self.H / 2 - y) * self.m_per_px
        return (self.c_lat + dy / 111320.0, self.c_lon + dx / (111320.0 * math.cos(math.radians(self.c_lat))))

    def draw(self):
        cv = self.cv
        cv.delete("all")
        pts = [self.to_xy(*p) for p in self.draft.polygon]
        if len(pts) >= 2:
            cv.create_line(*[c for p in pts + pts[:1] for c in p], fill="#ffb000", width=2, tags=("area",))
        for i, (x, y) in enumerate(pts):
            cv.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#ffb000", tags=("corner",))
            cv.create_text(x + 8, y - 8, text=str(i + 1), fill=FG, tags=("corner",))
        for which, col in (("top", "#4caf50"), ("bottom", "#2196f3")):
            p = getattr(self.draft, "ramp_" + which)
            if p:
                x, y = self.to_xy(*p)
                cv.create_rectangle(x - 5, y - 5, x + 5, y + 5, fill=col, tags=("ramp",))
                cv.create_text(x + 10, y + 10, text=T("ar.ramp_%s" % which), fill=col, anchor="w", tags=("ramp",))
        t, _rx = self.view.telemetry() if self.view else (None, 0)
        g = ptarea.Draft.gps_of(t)
        if g:
            x, y = self.to_xy(*g)
            cv.create_polygon(x, y - 8, x - 6, y + 6, x + 6, y + 6, fill="#ffffff", tags=("boat",))
        cv.create_text(8, self.H - 10, text=T("ar.scale", m=int(self.m_per_px * 100)), fill=DIM, anchor="w")

    def need_login(self):
        if self.store.session_ok(self.login.session, role="examiner"):
            self.say(T("ar.login_first"), WARN)
            return True
        return False

    def set_tap(self, which):
        self.tap = which
        self.say(T("ar.tap_mode_%s" % which))

    def on_tap(self, ev):
        lat, lon = self.to_ll(ev.x, ev.y)
        try:
            if self.tap == "corner":
                self.draft.add(lat, lon, "map")
            else:
                self.draft.set_ramp(self.tap, lat, lon, "map")
                self.tap = "corner"
        except ValueError as e:
            return self.say(T("ar.failed", why=str(e)), WARN)
        self.draw()

    def gps_point(self):
        t, _rx = self.view.telemetry()
        try:
            self.draft.add_gps(t)
        except ValueError as e:
            return self.say(T("ar.failed", why=str(e)), WARN)
        self.draw()

    def gps_ramp(self, which):
        t, _rx = self.view.telemetry()
        g = ptarea.Draft.gps_of(t)
        if g is None:
            return self.say(T("ar.failed", why="no trusted GPS fix"), WARN)
        self.draft.set_ramp(which, g[0], g[1], "boat GPS")
        self.draw()

    def undo(self):
        if self.draft.polygon:
            self.draft.remove(len(self.draft.polygon) - 1)
            self.draw()

    def geojson(self, path=None):
        path = path or filedialog.askopenfilename(parent=self.top, title=T("ar.geojson"),
                                                  filetypes=[("GeoJSON", "*.geojson *.json")])
        if not path:
            return
        try:
            self.draft.load_geojson(path)
        except (ValueError, OSError) as e:
            return self.say(T("ar.failed", why=str(e)), WARN)
        self.say(T("ar.imported", n=len(self.draft.polygon)))
        self.draw()

    def save(self):
        if self.need_login():
            return None
        try:
            a = ptarea.save(self.store, self.login.session, self.draft, self.base)
        except (ValueError, simguard.Refused) as e:
            return self.say(T("ar.failed", why=str(e)), WARN)
        self.base = a
        self.say(T("ar.saved", n=len(a.polygon), path=a.path))
        return a

    def close(self):
        self.closed = True
        try:
            self.top.destroy()
        except tk.TclError:
            pass
