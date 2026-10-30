"""The 3D simulator window (ACADEMY_SPEC H, step 22): the lesson-simulator choice, the missions for a future
3D world (Godot), their attempts + scores, and the drone-photo import for the 3D view. No 3D world now.

    python academy/sim3d_app.py [--lang el|en]       (or the academy window's "3D simulator" button)

Files only (academy/sim3d.py, docs/academy_sim_interface.md): no connection, no process (test_academy
AC1-AC3); the boat in every mission is the simulated AURIX on this PC; a mission result is logged, it never
changes a level; a drone photo is a picture for the 3D view, NEVER a chart. Every academy screen shows the banner.
"""
import os
import sys
import tkinter as tk
from tkinter import filedialog

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import content                                                          # noqa: E402
import sim3d                                                            # noqa: E402
import simguard                                                         # noqa: E402
import text                                                             # noqa: E402
import look                                                             # noqa: E402  (13i: the station's look)
import tour_app                                                         # noqa: E402
from banner import banner_text                                          # noqa: E402
from text import T                                                      # noqa: E402

CAUTION, WARN, GOOD, COL, FAMILY = tour_app.CAUTION, tour_app.WARN, tour_app.GOOD, tour_app.COL, tour_app.FAMILY
LANGS = ("el", "en")
# keys built from names (test_academy AC9 checks them too)
TEXT_KEYS = {"sim3d.backend_" + b for b in sim3d.BACKENDS} | {"sim3d.goal_" + g for g in sim3d.GOAL_TYPES}


class Sim3DWindow(object):
    def __init__(self, root, lang=None, store=None, user=None, env=None, docs=None):
        self.root = root
        self.lang = lang if lang in LANGS else text.lang()
        self.store, self.user, self.env = store, user, env
        self.docs = sim3d.missions() if docs is None else docs
        L = self.lang
        look.install(root)                  # the station's look (13i)
        look.title(root, "sim3d.title", L)
        root.configure(bg=COL["bg"])
        try:
            root.geometry("1100x640")
        except tk.TclError:
            pass
        # the banner: top, always visible, CAUTION colour (CLAUDE.md S, AC8)
        self.banner = look.banner(root, L)
        self.banner.pack(side="top", fill="x")
        tk.Label(root, text=T("sim3d.note", L), bg=COL["bg"], fg=COL["dim"], anchor="w", justify="left",
                 wraplength=1060).pack(side="top", fill="x", padx=10)
        who = T("lesson.who", L, user=user) if user else T("lesson.practice", L)
        tk.Label(root, text=who, bg=COL["bg"], fg=COL["dim"], anchor="w").pack(side="top", fill="x", padx=10)
        # the lesson simulator choice
        row = tk.Frame(root, bg=COL["bg"])
        row.pack(side="top", fill="x", padx=10, pady=6)
        tk.Label(row, text=T("sim3d.backend", L), bg=COL["bg"], fg=COL["fg"]).pack(side="left")
        self.backend = tk.StringVar(value=sim3d.chosen_backend(env))
        self.radios = {}
        for b in sim3d.BACKENDS:
            key = "sim3d.backend_" + b
            r = tk.Radiobutton(row, text=T(key, L), value=b, variable=self.backend,
                               command=lambda b=b: self.choose(b), bg=COL["bg"], fg=COL["fg"],
                               selectcolor=COL["panel"], activebackground=COL["bg"])
            r.pack(side="left", padx=6)
            self.radios[b] = r
        self.l_backend = tk.Label(row, text="", bg=COL["bg"], fg=COL["dim"], anchor="w")
        self.l_backend.pack(side="left", padx=10)
        body = tk.Frame(root, bg=COL["bg"])
        body.pack(side="top", fill="both", expand=True, padx=10, pady=4)
        self.list = tk.Listbox(body, width=44, bg=COL["panel"], fg=COL["fg"], selectbackground=CAUTION,
                               selectforeground="#000000", activestyle="none", exportselection=False,
                               font=(FAMILY, 11), highlightthickness=0)
        self.list.pack(side="left", fill="y")
        self.list.bind("<<ListboxSelect>>", lambda _e: self.show_selected())
        mid = tk.Frame(body, bg=COL["bg"])
        mid.pack(side="left", fill="both", expand=True, padx=10)
        self.l_title = tk.Label(mid, text=T("sim3d.pick", L), bg=COL["bg"], fg=COL["fg"], anchor="w",
                                font=(FAMILY, 15, "bold"), justify="left", wraplength=560)
        self.l_title.pack(side="top", fill="x")
        self.l_brief = tk.Label(mid, text="", bg=COL["bg"], fg=COL["fg"], anchor="nw", justify="left",
                                wraplength=560, font=(FAMILY, 11))
        self.l_brief.pack(side="top", fill="x", pady=(4, 6))
        self.l_goals = tk.Label(mid, text="", bg=COL["bg"], fg=COL["dim"], anchor="nw", justify="left",
                                wraplength=560)
        self.l_goals.pack(side="top", fill="x")
        bar = tk.Frame(mid, bg=COL["bg"])
        bar.pack(side="top", fill="x", pady=8)
        self.b_start = tk.Button(bar, text=T("sim3d.start", L), command=self.start_selected)
        self.b_start.pack(side="left")
        self.b_collect = tk.Button(bar, text=T("sim3d.collect", L), command=self.collect)
        self.b_collect.pack(side="left", padx=4)
        self.b_import = tk.Button(bar, text=T("sim3d.import", L), command=self.import_photos)
        self.b_import.pack(side="left", padx=4)
        self.msg = tk.Label(mid, text="", bg=COL["bg"], fg=COL["fg"], anchor="nw", justify="left", wraplength=560)
        self.msg.pack(side="top", fill="x")
        tk.Label(root, text=T("academy.photo_note", L), bg=COL["bg"], fg=CAUTION, anchor="w").pack(
            side="bottom", fill="x", padx=10, pady=4)
        self.rows = []
        self.fill_list()
        self.update_backend()

    # --- view
    def say(self, s, colour=None):
        self.msg.configure(text=s, fg=colour or COL["fg"])

    def tr(self, doc, key):
        return doc.get(key + "_" + self.lang) or doc.get(key + "_el", "")

    def fill_list(self):
        self.list.delete(0, "end")
        self.rows = []
        level = None
        for d in self.docs:
            if d.get("level") != level:
                level = d.get("level")
                key = "level.%s" % level
                self.list.insert("end", T(key, self.lang) if level in content.LEVELS else str(level))
                self.list.itemconfigure("end", foreground=CAUTION)
                self.rows.append(None)
            extra = ""
            if self.store is not None and self.user:
                n = sim3d.attempts(self.store, self.user, d["id"])
                best = sim3d.best_score(self.store, self.user, d["id"])
                extra = "  " + T("sim3d.tries", self.lang, n=n, best="-" if best is None else best)
            self.list.insert("end", "   " + self.tr(d, "title") + extra)
            self.rows.append(d)

    def update_backend(self):
        b, why = sim3d.active_backend(self.env)
        if self.backend.get() == "godot3d" and b != "godot3d":
            self.l_backend.configure(text=T("sim3d.not_running", self.lang, why=why), fg=WARN)
        else:
            key = "sim3d.backend_" + b
            self.l_backend.configure(text=T("sim3d.in_use", self.lang, name=T(key, self.lang)), fg=GOOD)
        return b

    def choose(self, name):
        try:
            sim3d.select_backend(name, self.env)
        except (simguard.Refused, ValueError, OSError) as e:
            self.say(T("sim3d.refused", self.lang, why=str(e)), WARN)
        self.backend.set(sim3d.chosen_backend(self.env))
        return self.update_backend()

    def selected(self):
        sel = self.list.curselection()
        return self.rows[sel[0]] if sel and self.rows[sel[0]] is not None else None

    def select(self, mission_id):
        for i, d in enumerate(self.rows):
            if d is not None and d["id"] == mission_id:
                self.list.selection_clear(0, "end")
                self.list.selection_set(i)
                self.show_selected()
                return d
        return None

    def show_selected(self):
        d = self.selected()
        if d is None:
            return
        self.l_title.configure(text=self.tr(d, "title"))
        self.l_brief.configure(text=self.tr(d, "brief"))
        lines = []
        for g in d.get("goals", []):
            key = "sim3d.goal_" + g["type"]
            lines.append(T(key, self.lang, **{k: v for k, v in g.items() if k != "type"}))
        lines.append(T("sim3d.pass", self.lang, score=d.get("pass_score"),
                       tries=d.get("max_attempts") or T("sim3d.no_limit", self.lang)))
        self.l_goals.configure(text="\n".join("- " + s for s in lines))

    # --- actions
    def start_selected(self):
        d = self.selected()
        if d is None:
            self.say(T("sim3d.pick", self.lang), CAUTION)
            return None
        b = self.update_backend()
        if b != "godot3d":
            self.say(T("sim3d.need_3d", self.lang), WARN)
            return None
        try:
            path = sim3d.issue(d, self.user, self.store, self.lang, self.env)
        except (simguard.Refused, OSError) as e:
            self.say(T("sim3d.refused", self.lang, why=str(e)), WARN)
            return None
        self.say(T("sim3d.issued", self.lang, file=os.path.basename(path)), GOOD)
        self.fill_list()
        return path

    def collect(self):
        try:
            ok, bad = sim3d.collect_results(self.store, self.env)
        except (simguard.Refused, OSError) as e:
            self.say(T("sim3d.refused", self.lang, why=str(e)), WARN)
            return None
        lines = [T("sim3d.result", self.lang, mission=e["mission"], score=e["score"],
                   status=T("sim3d.passed" if e["passed"] else "sim3d.failed", self.lang)) for e in ok]
        lines += [T("sim3d.bad_result", self.lang, file=f, why=w) for f, w in bad]
        self.say("\n".join(lines) or T("sim3d.no_results", self.lang), WARN if bad else GOOD)
        self.fill_list()
        return ok, bad

    def import_photos(self, paths=None, area=None):
        if paths is None:
            paths = filedialog.askopenfilenames(parent=self.root, title=T("sim3d.import", self.lang),
                                                filetypes=[("JPEG / PNG / TIFF", "*.jpg *.jpeg *.png *.tif *.tiff")])
        if not paths:
            return None
        d = self.selected()
        area = area or (d.get("world") if d else None) or "daskalio_demo"
        try:
            man, skipped = sim3d.import_photos(list(paths), area, self.env)
        except (simguard.Refused, OSError, ValueError, ImportError) as e:
            self.say(T("sim3d.refused", self.lang, why=str(e)), WARN)
            return None
        s = T("sim3d.imported", self.lang, n=len(paths) - len(skipped), area=area, total=len(man["photos"]))
        if skipped:
            s += "\n" + "\n".join(T("sim3d.skipped", self.lang, file=f, why=w) for f, w in skipped)
        self.say(s, WARN if skipped else GOOD)
        return man, skipped


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    lang = None
    if "--lang" in argv and argv.index("--lang") + 1 < len(argv):
        lang = argv[argv.index("--lang") + 1]
        text.set_lang(lang)
    root = tk.Tk()
    Sim3DWindow(root, lang)
    root.mainloop()


if __name__ == "__main__":
    main()
