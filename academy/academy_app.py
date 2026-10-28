"""The academy window (ACADEMY_SPEC A, step 16): accounts, levels, instructor view, gateway file, backup.

Started from the ground station's menu "Academy" (its OWN process and window - the station's boat
link is never handed over, owner W31) or alone:

    python academy/academy_app.py [--lang el|en] [--data FOLDER]

It never opens a connection (test_academy AC1-AC3). The levels reach the gateway only as a SIGNED
FILE (accounts.Store.make_push) that the ground station sends over its own login (menu Academy ->
send levels); the gateway checks the signature (gateway/trainlevels.py) and uses the levels at the
next login - never mid-trip. Every academy screen shows banner_text() at the top (AC8).
"""
import datetime
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.append(os.path.join(os.path.dirname(HERE), "groundstation"))   # theme.py: one look

import accounts                                                         # noqa: E402
import text                                                             # noqa: E402
from banner import banner_text                                          # noqa: E402
from text import T                                                      # noqa: E402

try:
    import theme                                                        # noqa: E402
    CAUTION = theme.CAUTION
    COL = theme.C
except Exception:                                                       # pragma: no cover - academy alone
    theme = None
    CAUTION = "#ffb000"
    COL = {"bg": "#0e141b", "fg": "#e6edf3", "panel": "#101a24", "dim": "#8fa3b5"}

COLUMNS = ("user", "name", "level", "effective", "expiry", "days_left", "lessons", "exams_passed", "hours",
           "current", "weak")


class AcademyApp(object):
    def __init__(self, root, store):
        self.root, self.store = root, store
        root.title(T("academy.title"))
        root.configure(bg=COL["bg"])
        root.geometry("1100x560")
        # the banner: top, always visible, CAUTION colour (CLAUDE.md S, AC8)
        self.banner = tk.Label(root, text=banner_text(), bg=CAUTION, fg="#000000",
                               font=(theme.FAMILY if theme else "Segoe UI", 16, "bold"), pady=6)
        self.banner.pack(side="top", fill="x")
        tk.Label(root, text=T("academy.sim_only"), bg=COL["bg"], fg=COL["dim"]).pack(side="top", anchor="w", padx=10)
        tk.Label(root, text=T("acc.instructor_view"), bg=COL["bg"], fg=COL["fg"],
                 font=(theme.FAMILY if theme else "Segoe UI", 13, "bold")).pack(side="top", anchor="w", padx=10, pady=(6, 0))
        frame = tk.Frame(root, bg=COL["bg"])
        frame.pack(side="top", fill="both", expand=True, padx=10, pady=4)
        self.tree = ttk.Treeview(frame, columns=COLUMNS, show="headings", height=14)
        for c in COLUMNS:
            self.tree.heading(c, text=T("acc.col_%s" % c))
            self.tree.column(c, width=150 if c in ("name", "weak") else 90, anchor="w")
        self.tree.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(frame, orient="vertical", command=self.tree.yview)
        sb.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=sb.set)
        bar = tk.Frame(root, bg=COL["bg"])
        bar.pack(side="top", fill="x", padx=10, pady=4)
        self.buttons = {}
        for key, fn in (("acc.new", self.new_account), ("acc.set_level", self.set_level),
                        ("acc.log_hours", self.log_hours), ("acc.make_push", self.make_push),
                        ("acc.key", self.show_key), ("acc.backup", self.backup), ("acc.refresh", self.refresh),
                        ("acc.tour", self.open_tour), ("acc.lessons", self.open_lessons),
                        ("acc.quizzes", self.open_quizzes)):
            b = tk.Button(bar, text=T(key), command=fn)
            b.pack(side="left", padx=3)
            self.buttons[key] = b
        self.msg = tk.Label(root, text="", bg=COL["bg"], fg=COL["fg"], anchor="w", justify="left")
        self.msg.pack(side="top", fill="x", padx=10, pady=(0, 6))
        self.refresh()

    # --- view
    def refresh(self):
        self.store.load()
        self.tree.delete(*self.tree.get_children())
        for r in self.store.summary():
            vals = []
            for c in COLUMNS:
                v = r.get(c)
                if c in ("level", "effective"):
                    v = T("level.%s" % v)
                elif c == "current":
                    v = T("acc.yes") if v else T("acc.no")
                elif c == "weak":
                    v = ", ".join(v)
                elif v is None:
                    v = "-"
                vals.append(v)
            self.tree.insert("", "end", iid=r["user"], values=vals)

    def say(self, s):
        self.msg.configure(text=s)

    def selected(self):
        sel = self.tree.selection()
        if not sel:
            self.say(T("acc.pick"))
            return None
        return sel[0]

    # --- actions
    def new_account(self):
        u = simpledialog.askstring(T("acc.new"), T("acc.ask_user"), parent=self.root)
        if not u:
            return
        name = simpledialog.askstring(T("acc.new"), T("acc.ask_name"), parent=self.root) or ""
        try:
            self.store.add(u.strip(), name.strip())
            self.say(T("acc.added", user=u.strip()))
        except ValueError as e:
            self.say(T("acc.failed", why=str(e)))
        self.refresh()

    def set_level(self):
        u = self.selected()
        if not u:
            return
        lv = simpledialog.askstring(T("acc.set_level"), T("acc.ask_level", levels=" / ".join(accounts.LEVELS)),
                                    parent=self.root)
        if not lv:
            return
        lv = lv.strip().upper()
        exp = None
        if lv in accounts.LEVELS and lv != "OBSERVER":
            s = simpledialog.askstring(T("acc.set_level"), T("acc.ask_expiry"), parent=self.root)
            if s:
                try:
                    exp = datetime.date.fromisoformat(s.strip())
                except ValueError:
                    return self.say(T("acc.failed", why="YYYY-MM-DD"))
        try:
            a = self.store.set_level(u, lv, exp, by="instructor")
            self.say(T("acc.level_set", user=u, level=T("level.%s" % a.level),
                       expiry=a.expiry.isoformat() if a.expiry else "-"))
        except (ValueError, KeyError) as e:
            self.say(T("acc.failed", why=str(e)))
        self.refresh()

    def log_hours(self):
        u = self.selected()
        if not u:
            return
        h = simpledialog.askfloat(T("acc.log_hours"), T("acc.ask_hours"), parent=self.root, minvalue=0.1,
                                  maxvalue=24.0)
        if h:
            self.store.log(u, "hours", h=round(h, 2))
            self.say(T("acc.hours_logged", user=u, h="%g" % h))
        self.refresh()

    def make_push(self):
        try:
            path, doc = self.store.make_push()
        except ValueError:
            return self.say(T("acc.no_key"))
        self.say(T("acc.push_made", n=len(doc["records"]), path=path))

    def show_key(self):
        k = self.store.key()
        if k is None:
            if not messagebox.askyesno(T("acc.key"), T("acc.key_new_q"), parent=self.root):
                return
            k = self.store.key(create=True)
        self.root.clipboard_clear()
        self.root.clipboard_append(k)
        self.say(T("acc.key_shown", path=self.store.key_path))

    def open_tour(self):
        """The guided tour (step 17) in its own window of THIS process (no process, no link)."""
        import tour_app
        self.tour = tour_app.TourWindow(tk.Toplevel(self.root), text.lang())
        return self.tour

    def chosen_user(self):
        """The selected account (lessons / quizzes are logged for it), or None = practice (nothing logged)."""
        sel = self.tree.selection()
        return sel[0] if sel else None

    def open_lessons(self):
        """The simulator lessons (step 18) in a window of THIS process; the link = simguard only."""
        import lesson_app
        self.lessons = lesson_app.LessonWindow(tk.Toplevel(self.root), text.lang(), store=self.store,
                                               user=self.chosen_user())
        return self.lessons

    def open_quizzes(self):
        """The quizzes (step 18) in a window of THIS process (no link at all)."""
        import quiz_app
        self.quizzes = quiz_app.QuizWindow(tk.Toplevel(self.root), text.lang(), store=self.store,
                                           user=self.chosen_user())
        return self.quizzes

    def backup(self):
        d = filedialog.askdirectory(parent=self.root, title=T("acc.backup"))
        if not d:
            return
        try:
            dst = self.store.backup(d)
            self.say(T("acc.backup_done", path=dst))
        except Exception as e:                                  # refused target or a disk error
            self.say(T("acc.backup_refused", why=str(e)))


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    data = None
    if "--lang" in argv:
        text.set_lang(argv[argv.index("--lang") + 1])
    if "--data" in argv:
        data = argv[argv.index("--data") + 1]
    try:
        store = accounts.Store(data)
    except Exception as e:                                       # e.g. on a gateway PC
        print(T("refuse.on_gateway") + " (%s)" % e)
        return 1
    if theme is not None:
        try:
            theme.dpi_aware()
        except Exception:
            pass
    root = tk.Tk()
    AcademyApp(root, store)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
