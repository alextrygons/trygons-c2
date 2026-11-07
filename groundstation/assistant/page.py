"""The Tools page «Βοηθός» (step AM2): the talking manual's settings on THIS PC - on / off, its OWN microphone
(separate from the loudhailer's devices), the push-to-talk key, the state, the last recognised question.

DISPLAY + per-PC SETTINGS ONLY: it gets the assistant client + the settings module, never `app`.
"""
import time

import i18n
import theme

from . import client, mic, paths

T = i18n.T


class Page:
    def __init__(self, fr, asst, settings):
        import tkinter as tk
        self.tk, self.a, self.settings = tk, asst, settings
        C = theme.C
        BG = C["bg"]
        F_TXT, F_MED = theme.font("label"), theme.font("button")
        self.fr = fr
        head = tk.Frame(fr, bg=BG)
        head.pack(anchor="w", padx=6, pady=(6, 2))
        theme.ai_badge(head).pack(side="left", padx=(0, 6))      # speech recognition = a neural network
        i18n.bind(tk.Label(head, font=theme.font("value"), bg=BG, fg=C["fg"]), "asst.title").pack(side="left")
        self.note = tk.Label(fr, font=F_TXT, bg=BG, fg=C["fg"], wraplength=520, justify="left")
        self.note.pack(anchor="w", padx=6)
        self.on = tk.BooleanVar(value=asst.enabled())
        i18n.bind(tk.Checkbutton(fr, variable=self.on, font=F_MED, bg=BG, fg=C["fg"], selectcolor=C["entry"],
                                 wraplength=500, justify="left", command=self.toggle), "asst.enable").pack(
            anchor="w", padx=6, pady=(8, 2))
        self.ask = tk.Button(fr, width=12, **theme.button_opts())
        i18n.bind(self.ask, "asst.btn")
        self.ask.bind("<ButtonPress-1>", lambda _e: self.a.press("button"))
        self.ask.bind("<ButtonRelease-1>", lambda _e: self.a.release())
        self.ask.pack(anchor="w", padx=6, pady=4)
        # its OWN microphone (never the loudhailer's in_dev): the label above, the choice below (narrow windows)
        i18n.bind(tk.Label(fr, font=F_TXT, bg=BG, fg=C["fg"], wraplength=520, justify="left"), "asst.mic").pack(
            anchor="w", padx=6, pady=(6, 0))
        self.mics = [(None, "")] + mic.list_inputs()
        self.mic_var = tk.StringVar()
        self.mic_menu = tk.OptionMenu(fr, self.mic_var, "")
        self.mic_menu.configure(**dict(theme.button_opts(), font=F_TXT))
        self.mic_menu.pack(anchor="w", padx=6, pady=2)
        # the push-to-talk key (a USB foot pedal sends the same key)
        i18n.bind(tk.Label(fr, font=F_TXT, bg=BG, fg=C["fg"], wraplength=520, justify="left"), "asst.key").pack(
            anchor="w", padx=6, pady=(6, 0))
        self.key_var = tk.StringVar(value=asst.key())
        km = tk.OptionMenu(fr, self.key_var, *client.KEYS, command=self.set_key)
        km.configure(**dict(theme.button_opts(), font=F_TXT, width=5))
        km.pack(anchor="w", padx=6, pady=2)
        self.state = tk.Label(fr, font=F_MED, bg=BG, fg=C["fg"], anchor="w", justify="left", wraplength=520)
        self.state.pack(fill="x", padx=6, pady=(8, 2))
        self.models = tk.Label(fr, font=F_TXT, bg=BG, fg=C["fg"], anchor="w", justify="left", wraplength=520)
        self.models.pack(fill="x", padx=6)
        self.last = tk.Label(fr, font=F_TXT, bg=BG, fg=C["fg"], anchor="w", justify="left", wraplength=520)
        self.last.pack(fill="x", padx=6, pady=(6, 2))
        self._sig, self._t = None, 0.0
        self.fill_mics()
        i18n.on_change(self.lang_changed)
        self.refresh()

    def lang_changed(self):
        self.fill_mics()
        self.refresh()

    def fill_mics(self):
        m = self.mic_menu["menu"]
        m.delete(0, "end")
        cur = self.a.mic_device()
        for idx, name in self.mics:
            lab = T("asst.mic_default") if idx is None else "%d  %s" % (idx, name)
            m.add_command(label=lab, command=lambda i=idx, lab=lab: self.set_mic(i, lab))
            if idx == cur:
                self.mic_var.set(lab)
        if not self.mic_var.get() or (cur is None and self.mic_var.get() != T("asst.mic_default")):
            self.mic_var.set(T("asst.mic_default"))

    def set_mic(self, idx, lab):
        self.mic_var.set(lab)
        self.settings.put("assistant_mic", idx)
        self.restart()

    def set_key(self, k):
        self.settings.put("assistant_key", k)
        self.refresh()

    def toggle(self):
        self.settings.put("assistant", bool(self.on.get()))
        self.restart()

    def restart(self):
        self.a.stop()
        self.a.start()
        self.refresh()

    def _signature(self):
        a = self.a
        return (a.state, a.device, id(a.last), a.key(), i18n.lang(), str(a.view_bar()), a.index_err)

    def maybe_refresh(self):
        """From the station's 100 ms tick: refreshed only on a change, else once a second (AM2 review LOW-6)."""
        now = time.monotonic()
        if self._signature() != self._sig or now - self._t >= 1.0:
            self.refresh()

    def refresh(self):
        a = self.a
        self._sig, self._t = self._signature(), time.monotonic()
        try:
            self.note.configure(text=T("asst.note", k=a.key()))
            st = T("asst.state." + a.state)
            if a.device and a.state in (client.READY, client.LISTENING, client.THINKING):
                st += "  ·  " + T("asst.device", dev=a.device)
            self.state.configure(text=st, fg=theme.CAUTION if a.out_of_service() else theme.C["fg"])
            mt = T("asst.models_ok") if paths.models_installed() else T("asst.models_missing")
            if a.index_err:                     # AM3-r MED-1: a stale index is never searched
                mt += "\n" + T("asst.n_index")
            self.models.configure(text=mt)
            h = a.last
            txt = T("asst.last", q=h["q"], lang=h.get("lang", ""), audio=h.get("audio_s", 0),
                    ms=h.get("ms", 0)) if h and h.get("q") else T("asst.none")
            if h and h.get("q") and h.get("found"):
                txt += "\n" + found_line(h["found"])
            self.last.configure(text=txt)
            v = a.view_bar()
            self.ask.configure(state="normal" if v and v["enabled"] else "disabled")
        except self.tk.TclError:
            pass


def found_line(f):
    """Where the manual search found the question (step AM3: the place only - the answer comes in AM4 / AM5)."""
    hits = f.get("hits") or []
    if f.get("cant_find") or not hits:
        return T("asst.not_found")
    h = hits[0]
    where = " / ".join(x for x in (h.get("heading") or "", h.get("term") or "") if x)
    if h.get("ch") == "tour":
        return T("asst.found_tour", title=where or h.get("chapter", ""))
    return T("asst.found", ch=h.get("ch", ""), chapter=h.get("chapter", ""), where=where or "-")


def build_page(fr, asst, settings):
    return Page(fr, asst, settings)
