"""The exam window (ACADEMY_SPEC G + J, step 21): the exam per level for the selected account.

    (the academy window's "Exams" button)

Same question types as the quizzes (quiz_app: choice / picture / alarm sound); the answers are NOT shown during
the exam (only the score at the end, then the explanations in the logbook view of the quizzes). The rules are
exam.py's: the required lessons first, 3 attempts then a 24 h cool-down (W27), online only in a supervised
session; starting = an attempt. Passing never changes a level (only the practical test on the boat does).
No connection, no process (AC1-AC3); the banner on top (AC8).
"""
import os
import sys
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import exam                                                             # noqa: E402
import quiz_app                                                         # noqa: E402
import simguard                                                         # noqa: E402
import text                                                             # noqa: E402
import look                                                             # noqa: E402  (13i: the station's look)
from banner import banner_text                                          # noqa: E402,F401  (AC8: the banner)
from text import T                                                      # noqa: E402

COL, WARN, GOOD = quiz_app.COL, quiz_app.WARN, quiz_app.GOOD


class ExamWindow(quiz_app.QuizWindow):
    """An exam for one account: the list = the exams; start_selected() -> exam.Run (or the refusal said)."""

    def __init__(self, root, lang=None, store=None, user=None, player=None, cache=None, seed=None, env=None):
        self.exams = exam.exams()
        self.env = env
        self.exam_run = None
        super(ExamWindow, self).__init__(root, lang, store, user, player, cache, docs=self.exams, seed=seed)
        L = self.lang
        look.title(root, "exam.title", L)
        self.list.configure(selectmode="browse")
        self.l_q.configure(text=T("exam.pick", L) if user else T("exam.no_user", L))
        self.refresh_list()

    def refresh_list(self):
        L = self.lang
        self.list.delete(0, "end")
        for d in self.exams:
            why = exam.refused(self.store, self.user, d, env=self.env) if (self.store and self.user) else "-"
            left = exam.attempts_left(self.store.entries(), self.user, d) if (self.store and self.user) else 0
            mark = "✓" if self.store and self.user and exam.passed(self.store.entries(), self.user, d.get("level")) else " "
            self.list.insert("end", "%s %s · %s (%s)" % (mark, T("level.%s" % d.get("level", "DRIVER"), L),
                                                        d.get("title_" + L) or d["id"],
                                                        T("exam.left", L, n=left) if not why else T("exam.closed", L)))

    def start_selected(self):
        sel = list(self.list.curselection())
        if not sel:
            self.l_q.configure(text=T("exam.pick", self.lang))
            return None
        return self.start_exam(self.exams[sel[0]]["id"])

    def start_exam(self, exam_id):
        L = self.lang
        if not self.store or not self.user:
            self.l_q.configure(text=T("exam.no_user", L))
            return None
        d = exam.by_id(exam_id)
        try:
            self.exam_run = exam.Run(self.store, self.user, d, L, seed=self.seed,
                                     sound_ok=lambda n: self.sound_file(n) is not None, env=self.env)
        except simguard.Refused as e:
            self.exam_run = None
            self.l_q.configure(text=T("exam.refused", L, why=str(e)))
            return None
        self.run = self.exam_run.quiz
        self.logged = False
        self.l_note.configure(text=T("quiz.sounds_left_out", L, n=self.run.left_out) if self.run.left_out else "")
        self.show()
        return self.exam_run

    def result(self, res):
        """During an exam: no right / wrong shown (only "answered"), on to the next question."""
        self.l_res.configure(text=T("exam.answered", self.lang), fg=COL["fg"])
        self.b_next.configure(state="normal")
        return res

    def answer(self, i):
        q = self.run.current() if self.run else None
        if not q or q["type"] == "picture" or q["result"]:
            return None
        res = self.run.answer(i)
        for b in self.b_choices[:len(q["choices"])]:
            b.configure(state="disabled")
        return self.result(res)

    def draw(self, result=None):
        return super(ExamWindow, self).draw(None)          # (never outline the right part during an exam)

    def end(self):
        L = self.lang
        if self.exam_run is None:
            return
        r = self.exam_run.finish()
        self.l_where.configure(text="")
        self.l_q.configure(text=T("exam.score", L, score=r["score"], need=self.exam_run.doc.get("pass_pct", 80)) +
                           "\n" + (T("exam.passed", L) if r["passed"] else T("exam.failed", L)))
        self.l_res.configure(text="")
        self.b_next.configure(state="disabled")
        self.logged = True
        self.refresh_list()
