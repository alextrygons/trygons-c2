"""The start-up screen (step 13i, DISPLAY ONLY): dark navy, the logo in the middle, "Trygons C2" +
version / commit and a progress line ("Εκκίνηση προσομοιωτή... gateway... καμερών... σταθμών...").

Shown while something LOADS, never over a running window:
  START_DEMO (demo/demo.py start --splash)   run_task(): the demo's start in a thread, the splash in
                                             this process; it closes when the ground stations report
                                             their windows open (their --ready-file)
  the ground station (app.py main)           Splash(root) while App() is built, closed BEFORE the
  the academy (academy_app.py main)          station / academy window is shown
Rules: closes by itself when ready; at most MAX_S (30 s) even when nobody reports; an error shows its
message (+ "Close", Esc) instead of hanging and closes itself after ERROR_HOLD_S. Esc always closes it.
Everything visual from theme.py, every text from lang/ (i18n.TL: the caller's language).
"""
import queue
import threading
import time
import tkinter as tk

import i18n
import theme
import version

MAX_S = 30.0
ERROR_HOLD_S = 120.0
POLL_MS = 100


def screen_box(win):
    """The full screen of the window's monitor: (w, h)."""
    return win.winfo_screenwidth(), win.winfo_screenheight()


class Splash(object):
    """The start-up screen. master = an existing Tk (a Toplevel is made on it) or None (own Tk root).
    what = the lang key of the thing starting (splash.what_demo / _station / _academy)."""

    def __init__(self, master=None, lang=None, what="splash.what_station", steps=(), fullscreen=True, max_s=MAX_S,
                 error_hold_s=ERROR_HOLD_S):
        self.lang = lang if lang in i18n.LANGS else i18n.lang()
        self.own = master is None
        if self.own:
            theme.set_app_id()
            self.w = tk.Tk()
        else:
            self.w = tk.Toplevel(master)
        self.state = "loading"              # loading -> closed, or loading -> failed -> closed
        self.steps = list(steps)            # the lang keys still to come (progress bar length)
        self.done = []
        self.error_text = ""
        self.max_s, self.error_hold_s = max_s, error_hold_s
        self.t0 = time.time()
        self._afters = []                   # cancelled on close (no late callback into a closed window)
        S = theme.SPLASH
        w = self.w
        theme.set_app_icon(w)
        w.title(i18n.TL(self.lang, "win.title", name=i18n.TL(self.lang, what)))
        w.configure(bg=S["bg"])
        sw, sh = screen_box(w)
        if fullscreen:
            w.overrideredirect(True)        # no frame: the whole screen is the splash
            w.geometry("%dx%d+0+0" % (sw, sh))
        else:
            w.geometry("%dx%d+%d+%d" % (sw // 2, sh // 2, sw // 4, sh // 4))
        self.size = (sw, sh) if fullscreen else (sw // 2, sh // 2)
        box = tk.Frame(w, bg=S["bg"])
        box.place(relx=0.5, rely=0.46, anchor="center")
        self.logo_img = self._logo(int(self.size[0] * 0.36))
        if self.logo_img is not None:
            tk.Label(box, image=self.logo_img, bg=S["bg"], bd=0).pack(pady=(0, 18))
        tk.Label(box, text=theme.APP_NAME, bg=S["bg"], fg=S["fg"], font=theme.font("huge")).pack()
        tk.Label(box, text=i18n.TL(self.lang, what), bg=S["bg"], fg=S["fg"], font=theme.font("value")).pack(pady=(2, 0))
        tk.Label(box, text=i18n.TL(self.lang, "splash.version", v=version.text()), bg=S["bg"], fg=S["dim"],
                 font=theme.font("small")).pack(pady=(2, 16))
        self.bar = tk.Canvas(box, width=360, height=6, bg=S["bar"], highlightthickness=0)
        self.bar.pack()
        self.l_prog = tk.Label(box, text=self.progress_text(), bg=S["bg"], fg=S["dim"], font=theme.font("label"))
        self.l_prog.pack(pady=(10, 0))
        self.l_err = tk.Label(box, text="", bg=S["bg"], fg=S["error"], font=theme.font("label"), justify="center",
                              wraplength=int(self.size[0] * 0.6))
        self.l_err.pack(pady=(10, 0))
        self.b_close = tk.Button(box, text=i18n.TL(self.lang, "splash.close"), command=self.close,
                                 **theme.button_opts("normal"))
        w.bind("<Escape>", lambda e: self.close())
        self._draw_bar()
        self.after(int(self.max_s * 1000), self._max_time)
        self.update()

    def _logo(self, width):
        p = theme.logo_path(dark=True)
        if p is None:
            return None
        try:
            from PIL import Image, ImageTk
            im = Image.open(p).convert("RGBA")
            h = max(1, int(im.height * width / float(im.width)))
            return ImageTk.PhotoImage(im.resize((width, h), Image.LANCZOS), master=self.w)
        except Exception:
            try:
                img = tk.PhotoImage(master=self.w, file=p)
                k = max(1, int(round(img.width() / float(width))))
                return img.subsample(k, k) if k > 1 else img
            except Exception:
                return None

    # --- progress ----------------------------------------------------------------------------------
    def progress_text(self):
        """ "Εκκίνηση προσομοιωτή... gateway... καμερών..." (the steps reached so far)."""
        if not self.done:
            return i18n.TL(self.lang, "splash.loading")
        return i18n.TL(self.lang, "splash.starting") + " " + " ".join(
            i18n.TL(self.lang, k) + "..." for k in self.done)

    def _draw_bar(self):
        n = len(self.done) + len(self.steps)
        f = (len(self.done) / float(n)) if n else 0.0
        self.bar.delete("all")
        self.bar.create_rectangle(0, 0, int(360 * f), 6, fill=theme.SPLASH["accent"], outline="")

    def step(self, key):
        """A new step started (its lang key, e.g. splash.p_sim)."""
        if self.state != "loading":
            return
        if key in self.steps:
            self.steps.remove(key)
        self.done.append(key)
        self.l_prog.configure(text=self.progress_text())
        self._draw_bar()
        self.update()

    def fail(self, message):
        """An error: shown on the splash (never a hang); Close / Esc, else it closes after error_hold_s."""
        if self.state == "closed":
            return
        self.state = "failed"
        self.error_text = i18n.TL(self.lang, "splash.failed") + "\n" + str(message)
        self.l_err.configure(text=self.error_text)
        self.b_close.pack(pady=(12, 0))
        self.after(int(self.error_hold_s * 1000), self.close)
        self.update()

    def _max_time(self):
        if self.state == "loading":         # nobody said "ready" in time: never stay up longer
            self.close()

    def after(self, ms, fn):
        if self.state != "closed":
            self._afters.append(self.w.after(ms, fn))

    def close(self):
        if self.state == "closed":
            return
        self.state = "closed"
        for a in self._afters:
            try:
                self.w.after_cancel(a)
            except tk.TclError:
                pass
        try:
            self.w.destroy()
        except tk.TclError:
            pass

    def update(self):
        if self.state == "closed":
            return
        try:
            self.w.update()
        except tk.TclError:
            self.state = "closed"


def run_task(task, steps, lang=None, what="splash.what_demo", fullscreen=True, max_s=MAX_S,
             error_hold_s=ERROR_HOLD_S, on_splash=None):
    """Runs task(progress) in a thread with the splash in this (main) thread. progress(key) = a step;
    task returns (ok, message) or raises (= an error with its text). The splash closes when the task
    succeeds, after max_s, or (an error) after its message was shown. Returns (ok, message)."""
    q = queue.Queue()
    out = {}

    def work():
        try:
            r = task(lambda key: q.put(("step", key)))
            ok, msg = r if isinstance(r, tuple) else (bool(r), "")
        except Exception as e:                      # an error is shown, never a hang
            ok, msg = False, "%s: %s" % (type(e).__name__, e)
        out["r"] = (ok, msg)
        q.put(("done", ok, msg))

    sp = Splash(None, lang=lang, what=what, steps=steps, fullscreen=fullscreen, max_s=max_s,
                error_hold_s=error_hold_s)
    if on_splash:
        on_splash(sp)
    th = threading.Thread(target=work, daemon=True)
    th.start()

    def poll():
        while True:
            try:
                m = q.get_nowait()
            except queue.Empty:
                break
            if m[0] == "step":
                sp.step(m[1])
            elif m[1]:
                sp.close()
            else:
                sp.fail(m[2] or "?")
        sp.after(POLL_MS, poll)

    sp.after(POLL_MS, poll)
    sp.w.mainloop()
    th.join()                                       # the work goes on after a max-time close (the demo)
    return out.get("r", (False, ""))
