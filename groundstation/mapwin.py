"""The separate full-size MAP window (section C) - for a 2nd / 3rd monitor. VIEW ONLY:
route editing stays on the captain screen; nothing here sends anything to the boat.

  depth chart, sea marks, AIS targets + list (name, distance, bearing, CPA, TCPA, age),
  own boat, track, route + waypoints, the avoidance route, shallow-water / collision
  warnings, DR uncertainty circle, range rings
  follow the boat / free pan (drag), zoom (buttons, mouse wheel), north-up / course-up,
  day / dark / night theme; its monitor, place and size are remembered (gs_layout.json)
  like the captain and driver windows; Ctrl+Shift+1..9 = to monitor n.
Course-up turns the map in 5 degree steps (the depth shading is redrawn per step).
"""
import math
import time
import tkinter as tk
from tkinter import ttk

import i18n
import mapview
import monitors
import settings
from i18n import T

F_TXT = ("Segoe UI", 11)
F_BTN = ("Segoe UI", 11, "bold")
THEME_ORDER = ("dark", "day", "night")


def nice_ring(m_per_px, px=120.0):
    """Range-ring spacing in metres (a round number of nm, or 100 / 200 / 500 m when close)."""
    want = m_per_px * px
    for nm in (0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0):
        if nm * 1852.0 >= want * 0.7:
            return nm * 1852.0
    return 20 * 1852.0


class MapWindow:
    def __init__(self, app, geometry=None):
        self.app = app
        self.w = tk.Toplevel(app.root, bg="#000000")
        self.w.title(T("mw.title"))
        self.w.geometry(geometry or "1400x900")
        self.w.protocol("WM_DELETE_WINDOW", app.close_map)
        cfg = dict(settings.DEFAULTS["map"], **(settings.get("map") or {}))
        self.theme = cfg.get("theme", "dark") if cfg.get("theme") in THEME_ORDER else "dark"
        self.course_up = bool(cfg.get("course_up"))
        self.m = float(cfg.get("m_per_px") or 10.0)
        self.follow = True
        self.center = None                       # (lat, lon) when panned by hand
        self.drag = None
        self.rings = True
        self.layers = {k: tk.BooleanVar(value=True) for k in ("depth", "contours", "marks", "ais")}
        self.chartview = mapview.ChartView(app.charts)
        self._t = 0.0
        top = tk.Frame(self.w, bg="#202830")
        top.pack(fill="x")
        self.btns = {}

        def b(key, cmd, **kw):
            bt = tk.Button(top, command=cmd, font=F_BTN, bg="#34404c", fg="#ffffff", activebackground="#506070",
                           bd=2, padx=6, **kw)
            i18n.bind(bt, key)
            bt.pack(side="left", padx=2, pady=3)
            self.btns[key] = bt
            return bt
        b("mw.follow", self.toggle_follow)
        b("mw.zoom_in", lambda: self.zoom(0.6))
        b("mw.zoom_out", lambda: self.zoom(1 / 0.6))
        b("mw.orient", self.toggle_orient)
        b("mw.theme", self.next_theme)
        b("mw.rings", self.toggle_rings)
        for k, v in self.layers.items():
            cb = tk.Checkbutton(top, variable=v, font=F_TXT, bg="#202830", fg="#ffffff", selectcolor="#34404c",
                                activebackground="#202830", command=self.redraw)
            i18n.bind(cb, "map.layer_" + k)
            cb.pack(side="left", padx=2)
        self.view_only = tk.Label(top, font=F_TXT, bg="#202830", fg="#a0b0c0")
        i18n.bind(self.view_only, "mw.view_only")
        self.view_only.pack(side="right", padx=8)
        body = tk.PanedWindow(self.w, orient="horizontal", bg="#000000", sashwidth=5)
        body.pack(fill="both", expand=True)
        self.cv = tk.Canvas(body, bg="#05101a", highlightthickness=0)
        body.add(self.cv, stretch="always", minsize=400)
        side = tk.Frame(body, bg="#101820")
        body.add(side, minsize=260, width=380)
        self.ais_lbl = tk.Label(side, font=F_BTN, bg="#101820", fg="#c8d8e8", anchor="w", justify="left")
        self.ais_lbl.pack(fill="x", padx=6, pady=(6, 2))
        self.cols = (("name", "ais.col_ship", 120), ("dist", "ais.col_dist", 70), ("brg", "ais.col_brg", 55),
                     ("cpa", "ais.col_cpa", 55), ("tcpa", "ais.col_tcpa", 60), ("age", "ais.col_age", 45))
        style = ttk.Style(self.w)
        style.configure("Map.Treeview", background="#101820", fieldbackground="#101820", foreground="#c8d8e8",
                        rowheight=22, font=F_TXT)
        style.configure("Map.Treeview.Heading", font=("Segoe UI", 9, "bold"))
        self.tree = ttk.Treeview(side, columns=[c for c, _k, _w in self.cols[1:]], height=20, style="Map.Treeview")
        for i, (c, key, wd) in enumerate(self.cols):
            col = "#0" if i == 0 else c
            self.tree.column(col, width=wd, anchor="w" if i == 0 else "e", stretch=i == 0)
            self.tree.heading(col, text=T(key))
        self.tree.tag_configure("danger", background="#601010", foreground="#ffd0d0")
        self.tree.tag_configure("warn", background="#504000", foreground="#fff0c0")
        self.tree.tag_configure("stale", foreground="#707070")
        self.tree.pack(fill="both", expand=True, padx=4)
        self.small = tk.Label(side, font=("Segoe UI", 9), bg="#101820", fg="#ff8080", wraplength=360, justify="left")
        i18n.bind(self.small, "ais.small_craft")
        self.small.pack(fill="x", padx=6, pady=4)
        self.info = tk.Label(self.w, font=F_TXT, bg="#202830", fg="#c8d8e8", anchor="w")
        self.info.pack(fill="x")
        self.cv.bind("<ButtonPress-1>", self.on_press)
        self.cv.bind("<B1-Motion>", self.on_motion)
        self.cv.bind("<ButtonRelease-1>", lambda e: setattr(self, "drag", None))
        self.cv.bind("<MouseWheel>", lambda e: self.zoom(0.8 if e.delta > 0 else 1.25))
        self.cv.bind("<Button-4>", lambda e: self.zoom(0.8))
        self.cv.bind("<Button-5>", lambda e: self.zoom(1.25))
        self.w.bind("<Configure>", lambda e: app.layout_changed())
        for k, fn in (("f", self.toggle_follow), ("F", self.toggle_follow), ("c", self.toggle_orient),
                      ("C", self.toggle_orient), ("t", self.next_theme), ("T", self.next_theme),
                      ("plus", lambda: self.zoom(0.6)), ("minus", lambda: self.zoom(1 / 0.6)),
                      ("KP_Add", lambda: self.zoom(0.6)), ("KP_Subtract", lambda: self.zoom(1 / 0.6))):
            self.w.bind("<%s>" % k if len(k) > 1 else k, lambda e, fn=fn: fn())
        for k in ("<F12>", "<Control-Tab>"):
            self.w.bind(k, lambda e: app.swap_screens())
        monitors.bind_keys(self.w, app)
        i18n.on_change(self.lang_changed)
        self.update_buttons()
        self.tick()

    # --- settings ---------------------------------------------------------------------------------------
    def save(self):
        settings.put("map", {"theme": self.theme, "course_up": self.course_up, "m_per_px": round(self.m, 3)})

    def update_buttons(self):
        self.btns["mw.follow"].configure(text=T("mw.follow") if self.follow else T("mw.free"),
                                         bg="#2a6040" if self.follow else "#34404c")
        self.btns["mw.orient"].configure(text=T("mw.course_up") if self.course_up else T("mw.north_up"))
        self.btns["mw.theme"].configure(text=T("mw.theme_" + self.theme))
        self.btns["mw.rings"].configure(bg="#2a6040" if self.rings else "#34404c")

    def lang_changed(self):
        try:
            self.w.title(T("mw.title"))
            for i, (c, key, _wd) in enumerate(self.cols):
                self.tree.heading("#0" if i == 0 else c, text=T(key))
            self.update_buttons()
        except tk.TclError:
            raise RuntimeError("map window gone")

    def toggle_follow(self):
        self.follow = not self.follow
        if self.follow:
            self.center = None
        self.update_buttons()
        self.redraw()

    def toggle_orient(self):
        self.course_up = not self.course_up
        self.save()
        self.update_buttons()
        self.redraw()

    def next_theme(self):
        self.theme = THEME_ORDER[(THEME_ORDER.index(self.theme) + 1) % len(THEME_ORDER)]
        self.save()
        self.update_buttons()
        self.redraw()

    def toggle_rings(self):
        self.rings = not self.rings
        self.update_buttons()
        self.redraw()

    def zoom(self, k):
        self.m = min(400.0, max(0.5, self.m * k))
        self.save()
        self.redraw()

    def on_press(self, e):
        self.drag = (e.x, e.y)

    def on_motion(self, e):
        """Drag = free pan (the map stops following the boat)."""
        if not self.drag or not self.view:
            return
        lat0, lon0, rot, w, h = self.view
        _P, inv = mapview.ChartView.projection(lat0, lon0, self.m, w, h, rot)
        dx, dy = e.x - self.drag[0], e.y - self.drag[1]
        self.center = inv(w / 2 - dx, h / 2 - dy)
        self.drag = (e.x, e.y)
        if self.follow:
            self.follow = False
            self.update_buttons()
        self.redraw()

    def destroy(self):
        self.w.destroy()

    # --- drawing ----------------------------------------------------------------------------------------
    def tick(self):
        try:
            if time.time() - self._t >= 0.5:
                self.redraw()
        except tk.TclError:
            return
        self.w.after(250, self.tick)

    view = None

    def redraw(self):
        self._t = time.time()
        a = self.app
        d = a.t
        c = self.cv
        c.delete("all")
        th = mapview.THEMES[self.theme]
        c.configure(bg=th["bg"])
        w, h = max(100, c.winfo_width()), max(100, c.winfo_height())
        self.draw_ais_list()
        if not d or not d.get("pos_source"):
            c.create_text(w / 2, h / 2, text=T("map.no_position"), fill=th["text"], font=("Segoe UI", 18, "bold"))
            self.view = None
            return
        boat = (d["est_lat"], d["est_lon"])
        if self.follow or self.center is None:
            self.center = boat
        lat0, lon0 = self.center
        rot = round(d["heading"] / 5.0) * 5.0 % 360.0 if self.course_up else 0.0
        self.view = (lat0, lon0, rot, w, h)
        m = self.m
        P, _inv = mapview.ChartView.projection(lat0, lon0, m, w, h, rot)
        av = (a.gw_status or {}).get("avoid") or {}
        self.chartview.draw(c, lat0, lon0, m, w, h, layers={k: v.get() for k, v in self.layers.items()}, opacity=0.9,
                            ais=a.ais_list(), threats=av.get("threats"), route=av.get("route"),
                            shallow=av.get("shallow"), theme=self.theme, rot=rot,
                            vision=((a.gw_status or {}).get("vision") or {}).get("obstacles"))
        bx, by = P(*boat)
        # range rings around the boat
        if self.rings:
            step = nice_ring(m)
            for i in range(1, 6):
                r = step * i / m
                if r > max(w, h):
                    break
                c.create_oval(bx - r, by - r, bx + r, by + r, outline=th["contour"][1], dash=(2, 4), tags="ring")
                lab = "%.2g %s" % (step * i / 1852.0, T("hud.nm")) if step >= 185 else "%.0f m" % (step * i)
                c.create_text(bx + r * 0.707 + 3, by - r * 0.707 - 3, text=lab, anchor="sw", fill=th["text"],
                              font=("Segoe UI", 9), tags="ring")
        # track
        tr = a.track[-3000:]
        if len(tr) >= 2:
            pts = []
            for la, lo in tr[::max(1, len(tr) // 1500)]:
                pts += P(la, lo)
            c.create_line(*pts, fill=th["track"], width=2, tags="track")
        # the program (route + waypoints) and the active waypoint
        wps = (a.prog or {}).get("wps") or []
        wpts = [P(wp["lat"], wp["lon"]) for wp in wps]
        if len(wpts) >= 2:
            c.create_line(*[v for p in wpts for v in p], fill=th["route"], width=2, dash=(8, 4), tags="prog")
        for i, (x, y) in enumerate(wpts):
            c.create_oval(x - 6, y - 6, x + 6, y + 6, outline=th["route"], width=2, tags="prog")
            c.create_text(x + 9, y, text=str(i + 1), anchor="w", font=("Segoe UI", 10, "bold"), fill=th["route"],
                          tags="prog")
        if d.get("wp_set"):
            x, y = P(d["wp_lat"], d["wp_lon"])
            c.create_line(bx, by, x, y, fill=th["route"], dash=(6, 4), width=2, tags="active_wp")
            c.create_oval(x - 11, y - 11, x + 11, y + 11, outline=th["route"], width=4, tags="active_wp")
        # DR uncertainty
        r = d["unc_m"] / m
        if r > 1:
            c.create_oval(bx - r, by - r, bx + r, by + r, outline=th["warn"] if d["pos_source"] in (2, 3) else
                          th["good"], width=2, dash=(4, 3), tags="unc")
        # own boat (turned like the map)
        hd = math.radians(d["heading"] - rot)
        pts = []
        for ang, rr in ((0, 18), (140, 12), (180, 6), (220, 12)):
            bb = hd + math.radians(ang)
            pts += [bx + rr * math.sin(bb), by - rr * math.cos(bb)]
        c.create_polygon(*pts, fill=th["own"], outline="#000000", width=2, tags="boat")
        cg = math.radians(d["est_cog"] - rot)
        L = min(200.0, d["est_sog"] * 1852.0 / 10.0 / m)                 # 6 minutes over ground
        if L > 4:
            c.create_line(bx, by, bx + L * math.sin(cg), by - L * math.cos(cg), fill=th["own"], width=2,
                          arrow="last", tags="boat")
        # warnings: shallow water ahead, collision risk
        warn = []
        sh = av.get("shallow")
        if sh:
            warn.append(T("map.shallow_ahead", s="%.0f" % sh["in_s"], depth=T("hud.no_data") if sh["depth"] is None
                          else "%.1f m" % sh["depth"], limit="%.1f" % sh["limit"]))
        danger = [t for t in av.get("threats") or [] if t.get("level") == "danger"]
        if danger:
            t = danger[0]
            warn.append(T("drv.collision", name=t["name"], cpa="%.2f" % t["cpa_nm"], tcpa="%.1f" % t["tcpa_min"],
                          brg="%03.0f" % t["bearing"]))
        if av.get("state") == "DRIVER DECISION":
            warn.append(T("drv.decision"))
        for i, txt in enumerate(warn[:3]):
            c.create_rectangle(w / 2 - 330, 10 + i * 34, w / 2 + 330, 40 + i * 34, fill="#300000", outline=th["warn"],
                               width=2, tags="warning")
            c.create_text(w / 2, 25 + i * 34, text=txt, fill=th["warn"], font=("Segoe UI", 13, "bold"),
                          width=640, tags="warning")
        # north arrow + scale bar
        na = math.radians(-rot)
        nx, ny = w - 40, 40
        c.create_line(nx - 22 * math.sin(na), ny + 22 * math.cos(na), nx + 22 * math.sin(na), ny - 22 * math.cos(na),
                      fill=th["text"], width=3, arrow="last", tags="north")
        c.create_text(nx + 30 * math.sin(na), ny - 30 * math.cos(na), text=T("mw.north"), fill=th["text"],
                      font=("Segoe UI", 12, "bold"), tags="north")
        bar_m = 10 ** math.floor(math.log10(max(1.0, m * 150)))
        c.create_line(20, h - 50, 20 + bar_m / m, h - 50, width=4, fill=th["text"], tags="scale")
        c.create_text(20, h - 56, text="%g m" % bar_m if bar_m < 1852 else "%g %s" % (bar_m / 1852.0, T("hud.nm")),
                      anchor="sw", fill=th["text"], font=("Segoe UI", 10), tags="scale")
        self.info.configure(text=T("mw.info", lat="%.5f" % d["est_lat"], lon="%.5f" % d["est_lon"],
                                   sog="%.1f" % d["est_sog"], cog="%03.0f" % d["est_cog"], hdg="%03.0f" % d["heading"],
                                   src=T("pos.%d" % d["pos_source"]) if d["pos_source"] < 4 else "?",
                                   unc="%.0f" % d["unc_m"], scale="%.1f" % m))

    def draw_ais_list(self):
        a = self.app
        rows = a.ais_rows_now() if hasattr(a, "ais_tree") else []
        self.ais_lbl.configure(text=T("mw.ais_title", n=len(rows)))
        sel = self.tree.selection()
        self.tree.delete(*self.tree.get_children())

        def f(v, fmt):
            return "-" if v is None else fmt % v
        for r in rows:
            tag = "danger" if r.get("level") == "danger" else "warn" if r.get("level") == "warn" else \
                "stale" if r.get("stale") else ""
            self.tree.insert("", "end", iid=str(r["mmsi"]), text=r.get("name") or str(r["mmsi"]), tags=(tag,),
                             values=(f(r.get("dist_nm"), "%.2f"), f(r.get("bearing"), "%03.0f"), f(r.get("cpa_nm"), "%.2f"),
                                     f(r["tcpa_min"] if (r.get("tcpa_min") or 0) >= 0 else None, "%.1f"),
                                     f(r.get("age_s"), "%.0f")))
        keep = [i for i in sel if self.tree.exists(i)]
        if keep:
            self.tree.selection_set(keep)
