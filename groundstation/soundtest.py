"""The Sound tab of the captain screen (section D6): sound settings, the voices found,
and every sound and callout to play in Greek and in English.

Playing here never sends anything to the boat; it uses the same player as the
callouts (one sound at a time).
"""
import os
import subprocess
import tkinter as tk
from tkinter import ttk

import alerts
import i18n
import settings
import sound
import tts
from i18n import T

BG = "#ffffff"
F_TXT = ("Segoe UI", 11)
F_MED = ("Segoe UI", 12, "bold")


def examples():
    """(code, level, tone, voice keys) of everything the system can say / sound, for the test list."""
    out = []
    for name in sound.TONE_NAMES:
        out.append(("tone." + name, None, name, []))
    for code, (lv, tn, keys, _q) in sound.CALLOUTS.items():
        if keys:
            out.append((code, lv, tn, keys))
    out.append(("traffic", alerts.CAUTION, "caution", ["say.traffic", "say.clock_2", sound.dist_key(800)]))
    out.append(("traffic", alerts.CAUTION, "caution", ["say.traffic", "say.clock_11", sound.dist_key(2400)]))
    out.append(("nav.turning", alerts.ADVISORY, "advisory", ["say.nav.turning"] + sound.heading_keys(245)))
    out.append(("power.fwd.50", alerts.ADVISORY, "advisory", ["say.power_fwd_50"]))
    out.append(("power.rev.10", alerts.ADVISORY, "advisory", ["say.power_rev_10"]))
    for bit, (code, lv) in sorted(alerts.ALARM_LEVELS.items()):
        out.append(("alarm." + code, lv, sound.LEVEL_TONE.get(lv), ["say.alarm." + code]))
    out.append(("alarm." + alerts.NO_DATA, alerts.WARNING, "warning", ["say.alarm." + alerts.NO_DATA]))
    return out


def build(parent, app):
    snd = app.sound
    st = {}

    def put(key, value):
        settings.put(key, value)
        snd.apply_settings()

    top = tk.Frame(parent, bg=BG)
    top.pack(fill="x", padx=6, pady=4)
    st["on"] = tk.BooleanVar(value=bool(settings.get("sound", True)))
    i18n.bind(tk.Checkbutton(top, variable=st["on"], font=F_MED, bg=BG,
                             command=lambda: put("sound", st["on"].get())), "snd.on").pack(side="left")
    i18n.bind(tk.Label(top, font=F_TXT, bg=BG), "snd.volume").pack(side="left", padx=(14, 2))
    vol = tk.Scale(top, from_=0, to=100, orient="horizontal", length=160, bg=BG, highlightthickness=0,
                   command=lambda v: put("volume", int(float(v))))
    vol.set(int(settings.get("volume", 80)))
    vol.pack(side="left")
    top = tk.Frame(parent, bg=BG)
    top.pack(fill="x", padx=6)
    for key, skey in (("snd.mute_adv", "mute_advisory"), ("snd.mute_fb", "mute_feedback"), ("snd.quiet", "quiet")):
        var = tk.BooleanVar(value=bool(settings.get(skey)))
        st[skey] = var
        i18n.bind(tk.Checkbutton(top, variable=var, font=F_TXT, bg=BG,
                                 command=lambda skey=skey, var=var: put(skey, var.get())), key).pack(side="left",
                                                                                                   padx=4)
    top = tk.Frame(parent, bg=BG)
    top.pack(fill="x", padx=6)
    i18n.bind(tk.Label(top, font=F_TXT, bg=BG), "snd.repeat").pack(side="left", padx=(4, 2))
    rep = tk.Spinbox(top, from_=5, to=120, increment=5, width=4, font=F_TXT,
                     command=lambda: put("repeat_s", float(rep.get())))
    rep.delete(0, "end")
    rep.insert(0, "%d" % int(settings.get("repeat_s", 20)))
    rep.pack(side="left")
    i18n.bind(tk.Label(parent, font=F_TXT, bg=BG, fg="#a00000", justify="left", wraplength=900, anchor="w"),
              "snd.warn_note").pack(fill="x", padx=8)

    vf = tk.Frame(parent, bg=BG, bd=2, relief="groove")
    vf.pack(fill="x", padx=6, pady=4)
    i18n.bind(tk.Label(vf, font=F_MED, bg=BG), "snd.voice_lang").grid(row=0, column=0, sticky="w", padx=4)
    vl = tk.StringVar(value=snd.voice_lang)
    for i, code in enumerate(i18n.LANGS):
        tk.Radiobutton(vf, text=i18n.NAMES[code], variable=vl, value=code, font=F_MED, bg=BG, indicatoron=False,
                       width=4, command=lambda: snd.set_voice_lang(vl.get())).grid(row=0, column=1 + i, padx=2)
    eng = tk.Label(vf, font=("Consolas", 11), bg=BG, justify="left", anchor="w")
    eng.grid(row=1, column=0, columnspan=6, sticky="w", padx=4)
    bb = tk.Frame(vf, bg=BG)
    bb.grid(row=2, column=0, columnspan=6, sticky="w", padx=2, pady=2)
    i18n.bind(tk.Button(bb, font=F_TXT, command=lambda: [snd.lib.pregenerate(lg) for lg in i18n.LANGS]),
              "snd.make_cache").pack(side="left", padx=2)

    def open_folder():
        path = snd.lib.overrides
        os.makedirs(os.path.join(path, "el"), exist_ok=True)
        os.makedirs(os.path.join(path, "en"), exist_ok=True)
        if os.name == "nt":
            subprocess.Popen(["explorer", os.path.normpath(path)])
    i18n.bind(tk.Button(bb, font=F_TXT, command=open_folder), "snd.own_files").pack(side="left", padx=2)
    bb = tk.Frame(vf, bg=BG)
    bb.grid(row=3, column=0, columnspan=6, sticky="w", padx=2, pady=2)
    for lg in i18n.LANGS:
        i18n.bind(tk.Button(bb, font=F_TXT, command=lambda lg=lg: play_all(lg)), "snd.play_all_" + lg).pack(
            side="left", padx=2)

    def engines():
        try:
            lines = []
            for lg in i18n.LANGS:
                d = tts.describe(lg)
                n = snd.lib.made.get(lg)
                lines.append(T("snd.engine", lang=i18n.NAMES[lg], what=d or T("snd.no_voice"),
                               cache=T("snd.making") if snd.lib.busy.get(lg) else
                               ("" if n is None else T("snd.cached", n=n))))
            eng.configure(text="\n".join(lines), fg="#000000" if all(tts.engine(lg) for lg in i18n.LANGS)
                          else "#a00000")
            eng.after(2000, engines)
        except tk.TclError:
            pass
    engines()

    # the list: every sound and callout, play in Greek or English
    box = tk.Frame(parent, bg=BG)
    box.pack(fill="both", expand=True, padx=6, pady=4)
    cols = (("level", "snd.col_level", 90), ("code", "snd.col_code", 170), ("text", "snd.col_text", 520))
    tree = ttk.Treeview(box, columns=[c for c, _k, _w in cols], show="headings", height=14)
    for c, key, wd in cols:
        tree.column(c, width=wd, anchor="w")
        tree.heading(c, text=T(key))
    sb = ttk.Scrollbar(box, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=sb.set)
    tree.pack(side="left", fill="both", expand=True)
    sb.pack(side="left", fill="y")
    rows = examples()

    def fill():
        tree.delete(*tree.get_children())
        for i, (code, lv, tn, keys) in enumerate(rows):
            if keys:
                txt = ", ".join(i18n.T(k) for k in keys)
            else:
                txt = T("snd.tone_" + tn) if lv is None else T("snd.tone_only")
            tree.insert("", "end", iid=str(i), values=(T("snd.level_%d" % lv) if lv is not None else T("snd.tone"),
                                                       code, txt))
        for c, key, _wd in cols:
            tree.heading(c, text=T(key))
    fill()
    i18n.on_change(fill)
    pb = tk.Frame(parent, bg=BG)
    pb.pack(fill="x", padx=6, pady=(0, 6))
    i18n.bind(tk.Label(pb, font=F_TXT, bg=BG), "snd.play_sel").pack(side="left")
    for lg in i18n.LANGS:
        tk.Button(pb, text="▶ " + i18n.NAMES[lg], font=F_MED, width=6,
                  command=lambda lg=lg: play_sel(lg)).pack(side="left", padx=3)
    tree.bind("<Double-1>", lambda e: play_sel(snd.voice_lang))

    def play(i, lang):
        code, lv, tn, keys = rows[i]
        v = snd.sched.vol(lv if lv is not None else alerts.ADVISORY)
        path, secs = snd.lib.clip(tn, keys, lang, v)
        snd.player.stop()
        snd.player.play(path)
        app.last_test_sound = (code, lang, path, secs)
        return secs

    def play_sel(lang):
        sel = tree.selection()
        if sel:
            play(int(sel[0]), lang)

    def play_all(lang, i=0):
        if i >= len(rows):
            return
        tree.selection_set(str(i))
        tree.see(str(i))
        secs = play(i, lang)
        parent.after(int(1000 * secs) + 350, lambda: play_all(lang, i + 1))

    app.sound_test = {"rows": rows, "play": play, "tree": tree}
