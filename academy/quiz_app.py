"""The quiz window (ACADEMY_SPEC D, step 18): multiple choice, picture (click the asked part on a
DRAWING of the screen) and alarm-sound questions; pools, random order, explanations.

    python academy/quiz_app.py [--lang el|en]       (or the academy window's "Quizzes" button)

No connection, no process (test_academy AC1-AC3). The alarm sounds = the ground station's own tones,
written by the station (groundstation/academymenu.py prepare_academy_sounds) into its sound cache;
this window only PLAYS them through simguard.check_recording. A sound question whose file is not
there (yet) is left out and the window says so. Scores go to the selected account's logbook (kind
"quiz", topic = the pool: the instructor view's weak areas). Every academy screen shows the banner.
"""
import os
import sys
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import quiz                                                             # noqa: E402  (puts groundstation/ on the path)
import simguard                                                         # noqa: E402
import text                                                             # noqa: E402
import look                                                             # noqa: E402  (13i: the station's look)
import tour_app                                                         # noqa: E402
from banner import banner_text                                          # noqa: E402
from text import T                                                      # noqa: E402

CAUTION, WARN, GOOD, COL, FAMILY = tour_app.CAUTION, tour_app.WARN, tour_app.GOOD, tour_app.COL, tour_app.FAMILY
MAX_CHOICES = 4


class QuizWindow(tour_app.Drawing):
    """A quiz on a Tk root / Toplevel. start(pool_ids) / answer(i) / click_at(fx, fy) / next()."""

    def __init__(self, root, lang=None, store=None, user=None, player=None, cache=None, docs=None, seed=None):
        self.root = root
        self.lang = lang if lang in quiz.LANGS else text.lang()
        self.store, self.user = store, user
        self.player = player or tour_app.default_player()
        self.cache = cache
        self.seed = seed
        self.docs = quiz.pools() if docs is None else docs
        self.run = None
        self.logged = False
        self.sound_state = None             # "played" / "missing" (the tests read it)
        L = self.lang
        look.install(root)                  # the station's look (13i)
        look.title(root, "quiz.title", L)
        root.configure(bg=COL["bg"])
        try:
            root.geometry("1180x760")
        except tk.TclError:
            pass
        # the banner: top, always visible, CAUTION colour (CLAUDE.md S, AC8)
        self.banner = look.banner(root, L)
        self.banner.pack(side="top", fill="x")
        who = T("lesson.who", L, user=user) if user else T("lesson.practice", L)
        tk.Label(root, text=who, bg=COL["bg"], fg=COL["dim"], anchor="w").pack(side="top", fill="x", padx=10)
        body = tk.Frame(root, bg=COL["bg"])
        body.pack(side="top", fill="both", expand=True, padx=10, pady=6)
        left = tk.Frame(body, bg=COL["bg"])
        left.pack(side="left", fill="y")
        tk.Label(left, text=T("quiz.pools", L), bg=COL["bg"], fg=COL["fg"], font=(FAMILY, 11, "bold")).pack(anchor="w")
        self.list = tk.Listbox(left, width=32, selectmode="extended", bg=COL["panel"], fg=COL["fg"],
                               selectbackground=CAUTION, selectforeground="#000000", activestyle="none",
                               exportselection=False, font=(FAMILY, 11), highlightthickness=0)
        for d in self.docs:
            self.list.insert("end", "%s · %s" % (T("level.%s" % d.get("level", "OBSERVER"), L),
                                                 d.get("title_" + L) or d.get("title_el") or d["id"]))
        self.list.pack(side="top", fill="y", expand=True)
        tk.Button(left, text=T("quiz.start", L), command=self.start_selected).pack(side="top", fill="x", pady=4)
        right = tk.Frame(body, bg=COL["bg"])
        right.pack(side="left", fill="both", expand=True, padx=(10, 0))
        self.l_where = tk.Label(right, text="", bg=COL["bg"], fg=COL["dim"], anchor="w")
        self.l_where.pack(side="top", fill="x")
        self.l_q = tk.Label(right, text=T("quiz.pick", L), bg=COL["bg"], fg=COL["fg"], anchor="nw", justify="left",
                            wraplength=760, font=(FAMILY, 15, "bold"))
        self.l_q.pack(side="top", fill="x", pady=4)
        self.cv = tk.Canvas(right, bg="#000000", highlightthickness=0, width=760, height=420)
        self.cv.bind("<Button-1>", self._on_click)
        self.cv.bind("<Configure>", lambda e: self.draw())
        self.b_sound = tk.Button(right, text=T("quiz.play_again", L), command=self.play)
        self.cframe = tk.Frame(right, bg=COL["bg"])
        self.cframe.pack(side="top", fill="x")
        self.b_choices = []
        for i in range(MAX_CHOICES):
            b = tk.Button(self.cframe, text="", anchor="w", justify="left", wraplength=720,
                          command=lambda k=i: self.answer(k))
            self.b_choices.append(b)
        self.l_res = tk.Label(right, text="", bg=COL["bg"], fg=GOOD, anchor="nw", justify="left", wraplength=760,
                              font=(FAMILY, 12, "bold"))
        self.l_res.pack(side="top", fill="x", pady=6)
        self.b_next = tk.Button(right, text=T("quiz.next", L), command=self.next, state="disabled")
        self.b_next.pack(side="top", anchor="w")
        self.l_note = tk.Label(right, text="", bg=COL["bg"], fg=COL["dim"], anchor="w")
        self.l_note.pack(side="top", fill="x")

    # --- sounds (only the station's prepared tones)
    def sound_file(self, name):
        try:
            return simguard.check_recording(quiz.tone_path(name, self.cache))
        except simguard.Refused:
            return None

    def play(self):
        q = self.run.current() if self.run else None
        if not q or q["type"] != "sound":
            return
        p = self.sound_file(q["sound"])
        if p is None:
            self.sound_state = "missing"
            return
        self.player.play(p)
        self.sound_state = "played"

    # --- a quiz
    def start_selected(self):
        sel = list(self.list.curselection()) or list(range(len(self.docs)))
        return self.start([self.docs[i]["id"] for i in sel])

    def start(self, pool_ids):
        docs = [d for d in self.docs if d["id"] in pool_ids]
        self.run = quiz.Run(docs, self.lang, seed=self.seed, sound_ok=lambda n: self.sound_file(n) is not None)
        self.logged = False
        self.l_note.configure(text=T("quiz.sounds_left_out", self.lang, n=self.run.left_out) if self.run.left_out else "")
        self.show()
        return self.run

    def show(self):
        L = self.lang
        q = self.run.current() if self.run else None
        for b in self.b_choices:
            b.pack_forget()
        self.cv.pack_forget()
        self.b_sound.pack_forget()
        if q is None:
            if self.run is not None:
                self.end()
            return
        self.l_where.configure(text=T("quiz.where", L, n=q["n"], of=q["of"]))
        self.l_q.configure(text=q["text"])
        self.l_res.configure(text="")
        self.b_next.configure(state="disabled")
        if q["type"] == "picture":
            self.cv.pack(side="top", fill="both", expand=True, before=self.cframe)
            self.cv.update_idletasks()
            self.draw()
        else:
            if q["type"] == "sound":
                self.b_sound.pack(side="top", anchor="w", before=self.cframe)
                self.play()
            for i, c in enumerate(q["choices"][:MAX_CHOICES]):
                self.b_choices[i].configure(text=c, state="normal", bg=COL["panel"], fg=COL["fg"])
                self.b_choices[i].pack(side="top", fill="x", pady=2)

    def draw(self, result=None):
        q = self.run.current() if self.run else None
        if not q or q["type"] != "picture":
            return
        self.draw_stop({"show": q["show"], "target": None, "title": "", "level": None, "level_text": ""})
        if result is not None:                              # after the answer: the right part outlined
            r = self.R(*result["right"])
            self.cv.create_rectangle(*r, outline=GOOD, width=5)

    def _on_click(self, e):
        fx, fy = self.to_frac(e.x, e.y)
        return self.click_at(fx, fy)

    def click_at(self, fx, fy):
        q = self.run.current() if self.run else None
        if not q or q["type"] != "picture" or q["result"]:
            return None
        res = self.run.click(fx, fy)
        self.draw(res)
        return self.result(res)

    def answer(self, i):
        q = self.run.current() if self.run else None
        if not q or q["type"] == "picture" or q["result"]:
            return None
        res = self.run.answer(i)
        for k, b in enumerate(self.b_choices[:len(q["choices"])]):
            b.configure(state="disabled", bg=GOOD if k == res["right"] else (WARN if k == i else COL["panel"]),
                        fg="#000000" if k in (res["right"], i) else COL["fg"])
        return self.result(res)

    def result(self, res):
        L = self.lang
        head = T("quiz.right", L) if res["correct"] else T("quiz.wrong", L)
        self.l_res.configure(text=head + "  " + res["why"], fg=GOOD if res["correct"] else WARN)
        self.b_next.configure(state="normal")
        return res

    def next(self):
        if self.run is None:
            return
        self.run.next()
        self.show()

    def end(self):
        L = self.lang
        sc = self.run.score()
        self.l_where.configure(text="")
        parts = ["%s %d %%" % (e["id"], e["score"]) for e in self.run.log_entries()]
        self.l_q.configure(text=T("quiz.score", L, score=sc) + ("\n" + ", ".join(parts) if parts else ""))
        self.l_res.configure(text="")
        self.b_next.configure(state="disabled")
        if not self.logged and self.store is not None and self.user:
            self.logged = True
            for e in self.run.log_entries():
                try:
                    self.store.log(self.user, "quiz", **e)
                except Exception as ex:
                    self.l_note.configure(text=T("lesson.log_failed", L, why=str(ex)))


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--lang" in argv:
        text.set_lang(argv[argv.index("--lang") + 1])
    if simguard.running_on_gateway():
        print(T("refuse.on_gateway"))
        return 1
    if tour_app.theme is not None:
        try:
            tour_app.theme.dpi_aware()
        except Exception:
            pass
    root = tk.Tk()
    QuizWindow(root, text.lang())
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
