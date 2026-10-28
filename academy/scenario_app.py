"""The emergency scenarios window (ACADEMY_SPEC E, step 19).

    python academy/scenario_app.py [--lang el|en]      (or the academy window's "Emergency scenarios")

The same simulator inputs as the lessons (lesson_app.LessonWindow: panel buttons, joystick pad, TAKE /
CONFIRM, the PC's power code / raise / lower / acknowledge) on the SIMULATOR of this PC (simguard.SimLink,
port 5760 only). A scenario prepares a normal situation, then the emergency comes: a simulator fault
(the simulated AURIX reacts with its own code) or a hazard / alarm the ACADEMY draws in its picture of
the station (map + AIS list + depth + forward camera; the academy never plays the gateway). The reaction
is timed and scored (scenario.py); at the end the debrief + the replay (a recorded .jsonl file read back
through simguard.check_recording). Results go to the selected account's logbook (kind "scenario").
Every academy screen shows the banner (AC8): the LessonWindow draws it first.
"""
import math
import os
import sys
import time
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import lesson_app                                                       # noqa: E402  (groundstation/ on the path)
import scenario                                                         # noqa: E402
import simguard                                                         # noqa: E402
import text                                                             # noqa: E402
from banner import banner_text                                          # noqa: E402
from lesson_app import CAUTION, COL, FAMILY, GOOD, WARN                 # noqa: E402
from text import T                                                      # noqa: E402

import alerts                                                           # noqa: E402
import i18n                                                             # noqa: E402
import proto                                                            # noqa: E402

MAP_W, MAP_H = 300, 230                     # the map (north up, boat-centred at the event point)
CAM_W, CAM_H = 300, 90                      # the forward camera strip (a drawing)
MAP_M = 160.0                               # metres from the centre to the map edge
CAM_FOV = 60.0                              # camera: +-30 degrees
CAM_RANGE = 150.0                           # camera: objects closer than this are drawn
HINT_S = 15.0                               # a prep / follow-up step's hint after this long


class ScenarioWindow(lesson_app.LessonWindow):
    """Scenarios on a Tk root / Toplevel (same inputs as the lessons). replay_folder = where the replay
    files go (default academy/data/replays)."""

    def __init__(self, root, lang=None, store=None, user=None, link_factory=None, starter=None, clock=time.time,
                 docs=None, replay_folder=None):
        docs = scenario.scenarios() if docs is None else docs
        self.replay_folder = replay_folder or scenario.replay_dir()
        self.last_replay = None
        self.replay_win = None
        self.refusals = 0
        self._nack_seen = 0
        self._refused_t, self._refused_text = -1e9, ""
        lesson_app.LessonWindow.__init__(self, root, lang, store, user, link_factory, starter, clock, docs)
        L = self.lang
        root.title(T("scen.title", L))
        try:
            root.geometry("1320x900")
        except tk.TclError:
            pass
        self.l_title.configure(text=T("scen.pick", L))
        lb = self.b_start.master
        self.b_replay = tk.Button(lb, text=T("scen.replay", L), command=self.open_replay, state="disabled")
        self.b_replay.pack(side="left", padx=4)
        mid = self.l_state.master
        self.l_shown = tk.Label(mid, text="", bg=COL["bg"], fg=CAUTION, anchor="w", justify="left", wraplength=520)
        self.l_shown.pack(side="top", fill="x")
        pics = tk.Frame(mid, bg=COL["bg"])
        pics.pack(side="top", fill="x", pady=4)
        self.map = tk.Canvas(pics, width=MAP_W, height=MAP_H, bg="#0b2236", highlightthickness=0)
        self.map.pack(side="left")
        side = tk.Frame(pics, bg=COL["bg"])
        side.pack(side="left", fill="both", expand=True, padx=6)
        self.cam = tk.Canvas(side, width=CAM_W, height=CAM_H, bg="#30506a", highlightthickness=0)
        self.cam.pack(side="top", anchor="w")
        self.l_ais = tk.Label(side, text="", bg=COL["panel"], fg=COL["fg"], anchor="nw", justify="left",
                              font=(FAMILY, 10), wraplength=300, padx=4, pady=4)
        self.l_ais.pack(side="top", fill="x", pady=4)
        self.debrief = tk.Label(mid, text="", bg=COL["panel"], fg=COL["fg"], anchor="nw", justify="left",
                                font=(FAMILY, 11), wraplength=520, padx=8, pady=6)
        self.debrief.pack(side="top", fill="x", pady=4)
        self.draw_picture(None)

    # --- the list (scenarios by level, a tick for the passed ones)
    def fill_list(self):
        self.list.delete(0, "end")
        self.rows = []
        done = set()
        if self.store is not None and self.user:
            try:
                done = {e.get("id") for e in self.store.entries()
                        if e.get("user") == self.user and e.get("kind") == "scenario" and e.get("done")}
            except Exception:
                done = set()
        groups = {}
        for d in self.docs:
            groups.setdefault(d.get("level", "DRIVER"), []).append(d)
        for lv in scenario.content.LEVELS:
            if not groups.get(lv):
                continue
            self.list.insert("end", "— %s —" % T("level.%s" % lv, self.lang))
            self.rows.append(None)
            for d in groups[lv]:
                mark = "✓ " if d["id"] in done else "   "
                self.list.insert("end", mark + (d.get("title_" + self.lang) or d.get("title_el")))
                self.rows.append(d)

    # --- a scenario
    def connect(self):
        if self.sim is not None and not getattr(self.sim.link, "closed", False):
            return True
        try:
            self.sim = scenario.Sim(self.link_factory())
            self.conn.configure(text=T("lesson.connected", self.lang), fg=GOOD)
            return True
        except simguard.Refused as e:
            self.conn.configure(text=T("lesson.sim_refused", self.lang, why=str(e)), fg=WARN)
        except OSError:
            self.conn.configure(text=T("lesson.no_sim", self.lang), fg=WARN)
        self.sim = None
        return False

    def begin(self, doc):
        if not self.connect():
            return None
        self.run = scenario.Run(doc, self.lang)
        self.logged = False
        self.refusals = 0
        self._nack_seen = getattr(self.sim, "refused_n", 0)
        for a in self.run.start(self.clock()):
            self.sim.do(a)
        self.code.delete(0, "end")
        self.l_title.configure(text=self.run.title())
        self.l_intro.configure(text=doc.get("intro_" + self.lang) or doc.get("intro_el", ""))
        self.l_result.configure(text="")
        self.debrief.configure(text="")
        self.b_retry.configure(state="normal")
        self.show()
        return self.run

    def retry(self):
        if self.run is None or not self.connect():
            return
        for a in self.run.retry(self.clock()):
            self.sim.do(a)
        self.logged = False
        self.refusals = 0
        self.l_result.configure(text="")
        self.debrief.configure(text="")
        self.show()

    def finish(self):
        self.logged = True
        r = self.run
        path = None
        try:
            path = scenario.save_replay(r, self.replay_folder, self.user)
            self.last_replay = path
            self.b_replay.configure(state="normal")
        except OSError as ex:                               # a disk error: said, the debrief still shows
            self.l_result.configure(text=T("lesson.log_failed", self.lang, why=str(ex)), fg=WARN)
        e = r.log_entry(replay=os.path.basename(path) if path else None)
        e["refusals"] = self.refusals
        if r.status == "done":
            self.l_result.configure(text=T("scen.passed", self.lang, score=e["score"]), fg=GOOD)
        else:
            self.l_result.configure(text=T("scen.failed", self.lang), fg=WARN)
        self.debrief.configure(text="\n".join(r.debrief()))
        if self.store is not None and self.user:
            try:
                self.store.log(self.user, "scenario", **e)
            except Exception as ex:
                self.l_result.configure(text=T("lesson.log_failed", self.lang, why=str(ex)), fg=WARN)
        if r.status == "done":
            self.fill_list()

    def step(self):
        lesson_app.LessonWindow.step(self)
        if self.sim is not None:
            total = getattr(self.sim, "refused_n", 0)
            new = self.sim.nacks()[-(total - self._nack_seen):] if total > self._nack_seen else []
            for a in new:
                if a.get("cmd") in (proto.T_POWER_CODE, proto.T_POWER_RAISE):
                    self.refusals += 1
                self._refused_t = self.clock()
                self._refused_text = T("scen.refused", self.lang, why=i18n.TL(
                    self.lang, "result.%d" % a.get("result", 11)))
            self._nack_seen = total

    def show(self):
        L = self.lang
        st = self.sim.state if self.sim else {}
        r = self.run
        if r is not None:
            s = r.status
            steps = r.doc.get("prep" if s in ("waiting", "prep") else "after", [])
            if s == "waiting":
                self.l_where.configure(text="")
                self.l_step.configure(text=T("lesson.waiting", L))
            elif s in ("prep", "after"):
                self.l_where.configure(text=T("scen.prep_of" if s == "prep" else "scen.after_of", L,
                                              n=r.index + 1, of=len(steps)))
                self.l_step.configure(text=r.step_text("do"))
            elif s in ("wait_event", "pending"):
                self.l_where.configure(text="")
                self.l_step.configure(text=T("scen.watch", L))
            elif s == "react":
                self.l_where.configure(text=T("scen.react_t", L, t="%.0f" % (self.clock() - r.t_event)))
                self.l_step.configure(text=T("scen.emergency", L, say=r.say()))
            elif s == "observe":
                self.l_where.configure(text=T("scen.observe", L))
            # the hint line: a refused PC command (5 s), else the step's hint after HINT_S
            hint = ""
            if self.clock() - self._refused_t < 5.0:
                hint = self._refused_text
            elif s in ("prep", "after") and self.clock() - (r.t_step or self.clock()) > HINT_S:
                hint = T("lesson.hint", L) + " " + r.step_text("hint")
            self.l_hint.configure(text=hint, fg=WARN if self.clock() - self._refused_t < 5.0 else CAUTION)
            if r.shown:
                self.l_shown.configure(text=T("scen.shown", L, alarms=", ".join(
                    i18n.TL(L, "alarm." + c) for c in r.shown)))
            else:
                self.l_shown.configure(text="")
        if st:
            bits = alerts.bits_of(st) | (r.shown_bits() if r is not None else 0)
            al = ", ".join(i18n.TL(L, "alarm." + c) for _b, c, _l in alerts.active(bits)) or "-"
            ck = st.get("confirm_kind", 0)
            pend = "-"
            if ck:
                pend = "%s (%.0f s)" % (proto.CONFIRM_KINDS[ck] if ck < len(proto.CONFIRM_KINDS) else ck,
                                        st.get("confirm_left_s", 0))
            self.l_state.configure(text=T(
                "lesson.state", L, mode=i18n.TL(L, "mode." + st.get("mode_name", "STANDBY")),
                heading="%.0f" % st.get("heading", 0), target="%.0f" % st.get("target", 0),
                throttle="%.0f" % st.get("throttle", 0), power="%.0f / %.0f" % (st.get("power_fwd", 0), st.get("power_rev", 0)),
                code=st.get("power_code") or "-", confirm=pend, alarms=al) + "\n" +
                T("scen.speed", L, kn="%.1f" % st.get("est_sog", 0)))
        if hasattr(self, "map"):
            self.draw_picture(r.picture(st, self.clock()) if (r is not None and st) else None, r)

    # --- the academy's picture of the station: map, forward camera, AIS list, depth
    def draw_picture(self, pic, run=None):
        L = self.lang
        m = self.map
        m.delete("all")
        m.create_text(6, 6, text=T("scen.map", L), anchor="nw", fill=COL["dim"], font=(FAMILY, 9))
        self.cam.delete("all")
        self.cam.create_rectangle(0, CAM_H * 0.45, CAM_W, CAM_H, fill="#1d3b52", outline="")
        self.cam.create_text(6, 6, text=T("scen.camera", L), anchor="nw", fill="#e0e8f0", font=(FAMILY, 9))
        gw_lost = run is not None and "gateway_lost" in run.shown
        if gw_lost:                                             # no gateway: no cameras, no AIS, no charts
            self.cam.create_rectangle(0, 0, CAM_W, CAM_H, fill="#000000", outline="")
            self.cam.create_text(CAM_W / 2, CAM_H / 2, text=T("scen.no_camera", L), fill=WARN, font=(FAMILY, 12, "bold"))
        if pic is None:
            self.l_ais.configure(text=T("scen.ais_none", L) if not gw_lost else T("scen.no_ais", L))
            return
        cx, cy, k = MAP_W / 2.0, MAP_H / 2.0, (MAP_H / 2.0) / MAP_M

        def xy(p):
            return cx + p[1] * k, cy - p[0] * k
        bx, by = xy(pic["boat"])
        h = math.radians(pic["heading"])
        pts = []
        for ang, rr in ((0.0, 9.0), (2.5, 6.0), (-2.5, 6.0)):
            pts += [bx + rr * math.sin(h + ang), by - rr * math.cos(h + ang)]
        if run is not None:                                     # the track so far
            tr = [(s["n"], s["e"]) for s in run.timeline if "n" in s]
            if len(tr) > 1:
                m.create_line(*[c for p in tr for c in xy(p)], fill="#7fb2d8")
        m.create_polygon(*pts, fill=CAUTION, outline="#000000", tags="boat")
        hz = pic.get("hazard")
        lines = []
        if hz is not None and not gw_lost:
            hx, hy = xy(hz["pos"])
            if hz["type"] == "ais":
                c = math.radians(hz["course"])
                m.create_oval(hx - 5, hy - 5, hx + 5, hy + 5, fill=WARN, outline="#000000", tags="ais")
                m.create_line(hx, hy, hx + 30 * math.sin(c), hy - 30 * math.cos(c), fill=WARN, arrow="last")
                lines.append(T("scen.ais_row", L, name=hz["name"], cpa="%.0f" % hz["cpa_m"],
                               tcpa="%.0f" % max(0.0, hz["tcpa_s"]), kn="%.0f" % hz["kn"]))
            elif hz["type"] == "shallow":
                rr = hz["r"] * k
                m.create_oval(hx - rr, hy - rr, hx + rr, hy + rr, fill="#b89b5e", outline="#e0c080", tags="shallow")
                lines.append(T("scen.depth", L, d="%.1f" % hz["depth"]))
            if hz["type"] in ("ais", "swimmer"):                # the camera: what is ahead (a drawing)
                dn, de = hz["pos"][0] - pic["boat"][0], hz["pos"][1] - pic["boat"][1]
                dist = math.hypot(dn, de)
                rel = (math.degrees(math.atan2(de, dn)) - pic["heading"] + 180.0) % 360.0 - 180.0
                if abs(rel) < CAM_FOV / 2 and dist < CAM_RANGE * (4 if hz["type"] == "ais" else 1):
                    px = CAM_W / 2 + rel / (CAM_FOV / 2) * (CAM_W / 2)
                    sz = max(3.0, 600.0 / max(dist, 5.0)) * (2.5 if hz["type"] == "ais" else 1.0)
                    py = CAM_H * 0.45 + 4
                    self.cam.create_oval(px - sz, py - sz / 2, px + sz, py + sz / 2,
                                         fill="#ff8c1a" if hz["type"] == "swimmer" else "#dddddd", outline="#000000",
                                         tags="cam_" + hz["type"])
            if hz["type"] == "swimmer":
                lines.append(T("scen.no_ais_target", L))
        if not lines:
            lines.append(T("scen.ais_none", L))
        self.l_ais.configure(text=("\n".join(lines)) if not gw_lost else T("scen.no_ais", L))

    # --- the replay
    def open_replay(self, path=None):
        path = path or self.last_replay
        if not path:
            return None
        try:
            self.replay_win = ReplayWindow(tk.Toplevel(self.root), path, self.lang)
        except (simguard.Refused, OSError, ValueError) as e:
            self.l_result.configure(text=T("scen.replay_refused", self.lang, why=str(e)), fg=WARN)
            self.replay_win = None
        return self.replay_win


class ReplayWindow(object):
    """The replay of one scenario run: the recorded track on the map, a time slider + play, the events
    and the debrief. Reads the file only through scenario.load_replay -> simguard.check_recording."""

    def __init__(self, top, path, lang="el"):
        self.top = top
        self.lang = lang
        self.head, self.samples = scenario.load_replay(path)
        docs = {d["id"]: d for d in scenario.scenarios()}
        self.doc = docs.get(self.head.get("id"), {})
        L = lang
        top.title(T("scen.replay_title", L))
        top.configure(bg=COL["bg"])
        self.banner = tk.Label(top, text=banner_text(L), bg=CAUTION, fg="#000000", font=(FAMILY, 16, "bold"), pady=6)
        self.banner.pack(side="top", fill="x")
        tk.Label(top, text=self.doc.get("title_" + L) or self.head.get("id"), bg=COL["bg"], fg=COL["fg"],
                 font=(FAMILY, 14, "bold"), anchor="w").pack(side="top", fill="x", padx=10)
        body = tk.Frame(top, bg=COL["bg"])
        body.pack(side="top", fill="both", expand=True, padx=10, pady=4)
        self.canvas = tk.Canvas(body, width=MAP_W + 100, height=MAP_H + 60, bg="#0b2236", highlightthickness=0)
        self.canvas.pack(side="left")
        right = tk.Frame(body, bg=COL["bg"])
        right.pack(side="left", fill="both", expand=True, padx=8)
        self.l_now = tk.Label(right, text="", bg=COL["panel"], fg=COL["fg"], anchor="nw", justify="left",
                              font=(FAMILY, 11), wraplength=340, padx=6, pady=4)
        self.l_now.pack(side="top", fill="x")
        ev = "\n".join(T("scen.ev_line", L, t="%.1f" % e.get("t", 0), what=T(EVENT_PREFIX + e["what"], L))
                       for e in self.head.get("events", []) if e.get("what") in EVENT_KEYS)
        tk.Label(right, text=ev, bg=COL["bg"], fg=COL["dim"], anchor="nw", justify="left",
                 font=(FAMILY, 10)).pack(side="top", fill="x", pady=4)
        tk.Label(right, text=self.result_text(), bg=COL["bg"], fg=COL["fg"], anchor="nw", justify="left",
                 wraplength=340, font=(FAMILY, 10)).pack(side="top", fill="x")
        n = max(0, len(self.samples) - 1)
        self.pos = tk.Scale(top, from_=0, to=n, orient="horizontal", showvalue=False, command=self._moved,
                            bg=COL["bg"], fg=COL["fg"], highlightthickness=0)
        self.pos.pack(side="top", fill="x", padx=10)
        bar = tk.Frame(top, bg=COL["bg"])
        bar.pack(side="top", fill="x", padx=10, pady=4)
        self.b_play = tk.Button(bar, text=T("scen.replay_play", L), command=self.play)
        self.b_play.pack(side="left")
        tk.Button(bar, text=T("scen.close", L), command=self.close).pack(side="left", padx=4)
        self.playing = False
        self.alive = True
        top.protocol("WM_DELETE_WINDOW", self.close)
        self.show(n)

    def result_text(self):
        L, h = self.lang, self.head
        failed = "scen.db_failed_" + (h.get("why") or "late")      # late / forbid / outcome
        out = [T("scen.db_passed", L, score=h.get("score", 0)) if h.get("status") == "done" else T(failed, L)]
        if h.get("reaction_s") is not None:
            out.append(T("scen.db_reaction_short", L, s="%.1f" % h["reaction_s"]))
        if h.get("min_m") is not None:
            out.append(T("scen.db_min_short", L, m="%.0f" % h["min_m"]))
        if self.doc:
            out.append(T("scen.db_right", L) + " " + (self.doc.get("right_" + L) or self.doc.get("right_el", "")))
            out.append(self.doc.get("debrief_" + L) or self.doc.get("debrief_el", ""))
        return "\n".join(out)

    def _moved(self, v):
        self.show(int(float(v)))

    def show(self, i):
        c = self.canvas
        c.delete("all")
        self.index = i
        if not self.samples:
            c.create_text(10, 10, text=T("scen.replay_none", self.lang), anchor="nw", fill=COL["fg"])
            return
        pts = [s for s in self.samples if "n" in s]
        W, H = MAP_W + 100, MAP_H + 60
        span = max([60.0] + [max(abs(s["n"]), abs(s["e"])) for s in pts] +
                   [max(abs(s.get("hz_n", 0)), abs(s.get("hz_e", 0))) for s in pts]) * 1.15
        k = min(W, H) / 2.0 / span

        def xy(n, e):
            return W / 2.0 + e * k, H / 2.0 - n * k
        upto = [s for s in self.samples[:i + 1] if "n" in s]
        if len(upto) > 1:
            c.create_line(*[v for s in upto for v in xy(s["n"], s["e"])], fill=CAUTION, width=2, tags="track")
        cur = self.samples[i]
        last = upto[-1] if upto else None                   # the boat where it last was (before the event: none)
        if last is not None:
            x, y = xy(last["n"], last["e"])
            c.create_oval(x - 4, y - 4, x + 4, y + 4, fill=CAUTION, outline="#000000", tags="boat")
            if "hz_n" in last:
                hx, hy = xy(last["hz_n"], last["hz_e"])
                if self.head.get("hazard") == "shallow" and self.head.get("hazard_r"):
                    rr = self.head["hazard_r"] * k
                    c.create_oval(hx - rr, hy - rr, hx + rr, hy + rr, fill="#b89b5e", outline="", tags="hazard")
                else:
                    c.create_oval(hx - 4, hy - 4, hx + 4, hy + 4, fill=WARN, outline="#000000", tags="hazard")
        L = self.lang
        al = ", ".join(i18n.TL(L, "alarm." + cc) for _b, cc, _l in alerts.active(cur.get("alm", 0))) or "-"
        self.l_now.configure(text=T("scen.replay_now", L, t="%.1f" % cur.get("t", 0),
                                    mode=i18n.TL(L, "mode." + (cur.get("mode") or "STANDBY")),
                                    hdg="%.0f" % cur.get("hdg", 0), kn="%.1f" % cur.get("sog", 0),
                                    thr="%.0f" % cur.get("thr", 0), alarms=al))

    def play(self):
        self.playing = True
        if self.index >= len(self.samples) - 1:
            self.index = 0
        self._tick()

    def _tick(self):
        if not (self.alive and self.playing):
            return
        if self.index >= len(self.samples) - 1:
            self.playing = False
            return
        self.pos.set(self.index + 1)
        self.show(self.index + 1)
        self.top.after(int(scenario.SAMPLE_S * 1000 / 4), self._tick)     # 4 x faster than real time

    def close(self):
        self.alive = False
        try:
            self.top.destroy()
        except tk.TclError:
            pass


EVENT_PREFIX = "scen.ev_"                   # + an EVENT_KEYS name = its language key
EVENT_KEYS = ("ready", "prep", "injected", "event", "cleared", "reacted", "observed", "after", "done", "failed")


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--lang" in argv:
        text.set_lang(argv[argv.index("--lang") + 1])
    if simguard.running_on_gateway():
        print(T("refuse.on_gateway"))
        return 1
    if lesson_app.theme is not None:
        try:
            lesson_app.theme.dpi_aware()
        except Exception:
            pass
    root = tk.Tk()
    ScenarioWindow(root, text.lang())
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
