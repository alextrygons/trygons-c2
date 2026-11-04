"""TRACKS (screen 2 bar 2, step 13n3, owner 2026-10-31 E3): the list of the tracks saved on this PC.

Date, boat, duration, nm, on the map; select one or more -> SHOW ON MAP (grey dashed with the date), HIDE, REMOVE
FROM MAP (the file stays), EXPORT GPX (a saved one or the live track), IMPORT GPX. DISPLAY ONLY: nothing is sent to
the boat or the gateway. The look = theme.py (one look, test_screen2 S9).
"""
import os
import time
import tkinter as tk
from tkinter import filedialog, ttk

import i18n
import theme
import track as trackmod
from i18n import T

C = theme.C
IMPORT_ERRORS = ("too_big", "dtd", "not_xml", "too_many", "no_points")


def fmt_dur(s):
    if s is None:
        return "-"
    s = int(s)
    return "%d:%02d" % (s // 3600, (s % 3600) // 60)


class TracksWindow:
    COLS = (("date", "track.col_date", 150), ("boat", "track.col_boat", 140), ("dur", "track.col_duration", 80),
            ("nm", "track.col_nm", 70), ("map", "track.col_map", 100))

    def __init__(self, app):
        self.app = app
        self.w = tk.Toplevel(app.root, bg=C["bg"])
        self.w.title(i18n.W("track.title"))
        self.w.protocol("WM_DELETE_WINDOW", self.close)
        theme.set_app_icon(self.w)
        tk.Label(self.w, text=T("track.head"), font=theme.font("title"), bg=C["bg"], fg=C["fg"]).pack(
            anchor="w", padx=10, pady=(10, 2))
        tk.Label(self.w, text=T("track.help"), font=theme.font("label"), bg=C["bg"], fg=C["fg"], justify="left",
                 wraplength=640).pack(anchor="w", padx=10)
        self.tree = ttk.Treeview(self.w, columns=[c for c, _k, _w in self.COLS], show="headings", height=12,
                                 selectmode="extended")
        for c, key, wd in self.COLS:
            self.tree.heading(c, text=T(key))
            self.tree.column(c, width=wd, anchor="w")
        self.tree.pack(fill="both", expand=True, padx=10, pady=8)
        b = tk.Frame(self.w, bg=C["bg"])
        b.pack(fill="x", padx=10)
        self.buttons = {}
        for key, fn, kind in (("show", self.show, "ok"), ("hide", self.hide, "normal"),
                              ("remove", self.remove, "normal"), ("export", self.export, "normal"),
                              ("export_live", self.export_live, "normal"), ("import", self.import_gpx, "normal"),
                              ("close", self.close, "normal")):
            bt = tk.Button(b, text=T("track.btn_" + key), command=fn, **theme.button_opts(kind))
            bt.pack(side="left", padx=(0, 6), pady=4)
            self.buttons[key] = bt
        self.msg = tk.Label(self.w, text="", font=theme.font("label"), bg=C["bg"], fg=C["fg"], justify="left",
                            wraplength=640)
        self.msg.pack(anchor="w", padx=10, pady=(0, 10))
        theme.style_tree(self.w)
        self.rows = []
        self.refresh()

    def alive(self):
        try:
            return bool(self.w.winfo_exists())
        except tk.TclError:
            return False

    def refresh(self):
        self.rows = self.app.tracks.saved_list()
        sel = set(self.selected())
        self.tree.delete(*self.tree.get_children())
        for r in self.rows:
            vis = self.app.track_shown.get(r["path"])
            on = T("track.on_map") if vis else (T("track.hidden") if vis is False else "")
            date = time.strftime("%d.%m.%Y %H:%M", time.localtime(r["start"])) if r["start"] is not None else \
                r["file"][:15]
            self.tree.insert("", "end", iid=r["path"], values=(date, r["boat"], fmt_dur(r["duration_s"]),
                                                               "%.1f" % r["nm"], on))
        keep = [p for p in sel if self.tree.exists(p)]
        if keep:
            self.tree.selection_set(keep)
        if not self.rows:
            self.say(T("track.none_saved"))

    def selected(self):
        try:
            return list(self.tree.selection())
        except tk.TclError:
            return []

    def say(self, text, bad=False):
        self.msg.configure(text=text, fg=C["bad_txt"] if bad else C["fg"])

    def _need(self):
        sel = self.selected()
        if not sel:
            self.say(T("track.select_first"), True)
        return sel

    def show(self):
        for p in self._need():
            self.app.track_show(p, True)
        self.refresh()

    def hide(self):
        for p in self._need():
            if p in self.app.track_shown:
                self.app.track_show(p, False)
        self.refresh()

    def remove(self):
        for p in self._need():
            self.app.track_remove(p)
        self.refresh()

    def export(self, dst=None):
        """The selected saved tracks -> GPX (asked where, one by one; dst = a folder: there)."""
        sel = self._need()
        if not sel:
            return None
        out = []
        for p in sel:
            d = os.path.join(dst, os.path.basename(p)[:-6] + ".gpx") if dst else None     # dst = a folder (tests)
            if d is None:
                d = filedialog.asksaveasfilename(parent=self.w, defaultextension=".gpx",
                                                 initialfile=os.path.basename(p)[:-6] + ".gpx",
                                                 filetypes=[("GPX", "*.gpx")])
                if not d:
                    continue
            try:
                tr = self.app.tracks.load(p)        # inside the try: a file gone meanwhile = a message (review 13n3 L6)
                out.append(trackmod.gpx_write(tr, d, name=tr.boat or None))
            except OSError as e:
                self.say(T("track.gpx_error", err=e), True)
                return out
        if out:
            self.say(T("track.exported", n=len(out), path=out[-1]))
        return out

    def export_live(self, dst=None):
        lv = self.app.tracks.live
        if lv is None or not lv.track.pts:
            self.say(T("track.nothing"), True)
            return None
        if dst is None:
            dst = filedialog.asksaveasfilename(parent=self.w, defaultextension=".gpx",
                                               initialfile=trackmod.safe_name(lv.boat) + "_live.gpx",
                                               filetypes=[("GPX", "*.gpx")])
            if not dst:
                return None
        try:
            tr = lv.track
            if tr.dropped and os.path.exists(lv.path):
                tr = trackmod.read_file(lv.path)    # the RAM holds only the newest points: every one from the file
            trackmod.gpx_write(tr, dst, name=lv.boat)
        except OSError as e:
            self.say(T("track.gpx_error", err=e), True)
            return None
        self.say(T("track.exported", n=1, path=dst))
        return dst

    def import_gpx(self, src=None):
        if src is None:
            src = filedialog.askopenfilename(parent=self.w, filetypes=[("GPX", "*.gpx"), ("*", "*.*")])
            if not src:
                return None
        try:
            path = self.app.tracks.import_gpx(src)
        except ValueError as e:
            code = str(e) if str(e) in IMPORT_ERRORS else "not_xml"
            self.say(T("track.import_error", err=T("track.err_" + code)), True)
            return None
        except OSError as e:
            self.say(T("track.import_error", err=e), True)
            return None
        self.app.track_show(path, True)
        self.refresh()
        self.say(T("track.imported", name=os.path.basename(path)[:-6]))
        return path

    def close(self):
        try:
            self.w.destroy()
        except tk.TclError:
            pass
        if self.app.tracks_win is self:
            self.app.tracks_win = None
