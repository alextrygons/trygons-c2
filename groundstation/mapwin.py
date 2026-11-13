"""The map of screen 2 (navigation & control, CLAUDE.md section E): it fills the whole window,
the panels of overlay.py lie on top of it (the same canvas).

  * the boat is ALWAYS in the centre and the map moves under it; north-up, locked (no
    course-up); the boat symbol turns with the heading; zoom in / out (buttons, mouse wheel)
  * a temporary pan (drag; e.g. while editing a route far away) snaps back to the boat after
    PAN_BACK_S (10 s) without input, or at once with "back to boat" (back_to_boat())
  * editing (the copilot or the captain, CLAUDE.md D): edit = "route" -> a click on the map adds a
    waypoint at the end, dragging a waypoint moves it, a right click deletes it (app.prog = the
    route editor, nothing is sent); edit = "fence" -> the same for the geofence corners
    (app.fence_draft). Sending them is the panels' job (app.py): through the gateway, a copilot's
    change waits for the captain's ACCEPT on screen 1.
  * drawn with tag "map": depth chart, sea marks, AIS (mapview.ChartView), range rings, track,
    route + waypoints, the active waypoint, the geofence, a refused leg, drift, DR uncertainty,
    the boat + its 6-minute course line. The note "Not for navigation ..." is drawn where the
    panels leave room (note_box).
Nothing here sends anything to the boat.
"""
import math
import time

import mapview
import settings
import theme
from i18n import T

PAN_BACK_S = 10.0           # a temporary pan snaps back to the boat after this long without input
HIT_PX = 14                 # grab radius of a waypoint / fence corner
ZOOM_MIN, ZOOM_MAX = 0.5, 400.0      # metres per pixel


def nice_ring(m_per_px, px=120.0):
    """Range-ring spacing in metres (a round number of nm, or 100 / 200 / 500 m when close)."""
    want = m_per_px * px
    for nm in (0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0):
        if nm * 1852.0 >= want * 0.7:
            return nm * 1852.0
    return 20 * 1852.0


class MapView:
    def __init__(self, app, canvas):
        self.app = app
        self.cv = canvas
        self.embedded = True
        cfg = dict(settings.DEFAULTS["map"], **(settings.get("map") or {}))
        self.m = min(ZOOM_MAX, max(ZOOM_MIN, float(cfg.get("m_per_px") or 10.0)))
        lay = settings.get("map_layers") or {}
        self.layers = {k: bool(lay.get(k, True)) for k in ("depth", "contours", "marks", "ais")}
        self.rings = bool(settings.get("map_rings", True))
        self.pan = None                          # (lat, lon) of the window centre while panned, else None
        self.pan_t = 0.0                         # last pan / edit input (the snap-back clock)
        self.edit = None                         # None / "route" / "fence"
        self.grab = None                         # ("route" | "fence", index) being dragged
        self.press_at = None
        self.drag_from = None
        self.view = None                         # (lat0, lon0, w, h) of the last drawing
        self.boat = None                         # (lat, lon) of the boat at the last drawing
        self.note_box = None                     # (x0, y0, x1, y1) where the chart note goes (set by the app)
        self.chartview = mapview.ChartView(app.charts)
        self._t = 0.0
        self.redraws = 0

    # --- settings ---------------------------------------------------------------------------------------
    def save(self):
        settings.put("map", {"m_per_px": round(self.m, 3)})
        settings.put("map_layers", dict(self.layers))
        settings.put("map_rings", self.rings)

    @property
    def follow(self):
        return self.pan is None

    @property
    def center(self):
        return self.pan if self.pan is not None else self.boat

    def zoom(self, k):
        self.m = min(ZOOM_MAX, max(ZOOM_MIN, self.m * k))
        if self.pan is not None:
            self.pan_t = time.time()
        self.save()
        self.redraw()

    def toggle_layer(self, k):
        if k == "rings":
            self.rings = not self.rings
        else:
            self.layers[k] = not self.layers[k]
        self.save()
        self.redraw()

    def back_to_boat(self):
        self.pan = None
        self.redraw()

    def set_edit(self, what):
        """None / "route" / "fence" (the panels' buttons)."""
        self.edit = what if what != self.edit else None
        self.grab = None
        self.redraw()

    # --- geometry ---------------------------------------------------------------------------------------
    def size(self):
        return max(100, self.cv.winfo_width()), max(100, self.cv.winfo_height())

    def P(self):
        lat0, lon0, w, h = self.view
        return mapview.ChartView.projection(lat0, lon0, self.m, w, h, 0.0)

    def boat_xy(self):
        if not self.view or self.boat is None:
            return None
        return self.P()[0](*self.boat)

    def keepout(self, W=None, H=None):
        """The boat and the area right around it: (cx, cy, r) - no panel may cover it."""
        if W is None:
            W, H = self.size()
        return W / 2.0, H / 2.0, 0.17 * min(W, H)

    # --- mouse (the app passes the clicks that are not on a panel) --------------------------------------------
    def _points(self, what):
        if what == "route":
            return [(w["lat"], w["lon"]) for w in (self.app.prog or {}).get("wps") or []]
        return list(self.app.fence_draft or [])

    def _near(self, x, y):
        if not self.view or not self.edit:
            return None
        P, _inv = self.P()
        best = None
        for i, (la, lo) in enumerate(self._points(self.edit)):
            px, py = P(la, lo)
            dd = math.hypot(px - x, py - y)
            if dd <= HIT_PX and (best is None or dd < best[1]):
                best = (i, dd)
        return best[0] if best else None

    def press(self, x, y):
        self.press_at = (x, y)
        self.drag_from = (x, y)
        i = self._near(x, y)
        self.grab = (self.edit, i) if i is not None else None

    def motion(self, x, y):
        if not self.view or self.drag_from is None:
            return
        P, inv = self.P()
        if self.grab:
            what, i = self.grab
            la, lo = inv(x, y)
            if what == "route":
                wps = self.app.prog["wps"]
                if i < len(wps):
                    wps[i]["lat"], wps[i]["lon"] = la, lo
            else:
                if i < len(self.app.fence_draft):
                    self.app.fence_draft[i] = (la, lo)
            self.pan_t = time.time()
            self.app.map_edited(what)
        else:                                       # drag = a temporary pan
            lat0, lon0, w, h = self.view
            dx, dy = x - self.drag_from[0], y - self.drag_from[1]
            self.pan = inv(w / 2 - dx, h / 2 - dy)
            self.view = (self.pan[0], self.pan[1], w, h)
            self.pan_t = time.time()
        self.drag_from = (x, y)
        self.redraw()

    def release(self, x, y):
        p, self.press_at = self.press_at, None
        self.drag_from = None
        grabbed, self.grab = self.grab, None
        if p is None or grabbed or abs(x - p[0]) + abs(y - p[1]) > 4 or not self.view:
            return
        la, lo = self.P()[1](x, y)
        if not self.edit and getattr(self.app, "sar_map_hit", None) and self.app.sar_map_hit(x, y):
            return                                  # a marked point: its menu (step PF4)
        if not self.edit and getattr(self.app, "uav_map_hit", None) and self.app.uav_map_hit(x, y):
            return                                  # a UAV symbol: its video (step U4)
        if self.edit:
            self.pan_t = time.time()
            self.app.map_add(self.edit, la, lo)
        elif self.app.map_click:
            self.app.map_click(la, lo)

    def right(self, x, y):
        """A right click on a waypoint / fence corner while editing: delete it."""
        i = self._near(x, y)
        if i is not None:
            self.pan_t = time.time()
            self.app.map_delete(self.edit, i)
        elif self.view and not self.edit and getattr(self.app, "map_menu", None):
            # step U3 (owner H2): «Πήγαινε εδώ» (the boat) / «Αποστολή UAV εδώ» at the clicked point
            la, lo = self.P()[1](x, y)
            self.app.map_menu(x, y, la, lo)

    # --- drawing ----------------------------------------------------------------------------------------
    def tick(self):
        """Called by the app 4 x per second: the snap-back and a redraw twice a second."""
        now = time.time()
        if self.pan is not None and self.drag_from is None and now - self.pan_t >= PAN_BACK_S:
            self.pan = None
            self._t = 0.0
        if now - self._t >= 0.5:
            self.redraw()

    def redraw(self):
        self._t = time.time()
        self.redraws += 1
        a = self.app
        d = a.t
        c = self.cv
        c.delete("map")
        mark = c.create_line(0, 0, 0, 0, fill="")
        th = mapview.THEMES[theme.map_theme()]
        c.configure(bg=th["bg"])
        w, h = self.size()
        boat = (d["est_lat"], d["est_lon"]) if d and d.get("pos_source") else None
        if boat is None and a.track:
            boat = a.track[-1]                      # the last known position (no fix now)
        self.boat = boat
        if boat is None:
            self.view = None
            c.create_text(w / 2, h / 2 - 0.22 * min(w, h), text=T("map.no_position"), fill=th["text"],
                          font=theme.tkfont("value", theme.px("value", w, h)), tags="nopos")
            self._tag(mark)
            return
        lat0, lon0 = self.pan if self.pan is not None else boat
        self.view = (lat0, lon0, w, h)
        m = self.m
        P, _inv = self.P()
        av = (a.gw_status or {}).get("avoid") or {}
        self.chartview.draw(c, lat0, lon0, m, w, h, layers=dict(self.layers), opacity=0.9, ais=a.ais_list(),
                            threats=av.get("threats"), route=av.get("route"), shallow=av.get("shallow"),
                            theme=theme.map_theme(), vision=((a.gw_status or {}).get("vision") or {}).get("obstacles"),
                            note_box=self.note_box, origin=boat, text_k=getattr(a.ov, "k", 1.0) * theme.unit(w, h))
        f_small = theme.tkfont("small", theme.px("small", w, h))
        f_label = theme.tkfont("label", theme.px("label", w, h), bold=True)
        bx, by = P(*boat)
        if self.rings:
            step = nice_ring(m)
            for i in range(1, 6):
                r = step * i / m
                if r > max(w, h):
                    break
                c.create_oval(bx - r, by - r, bx + r, by + r, outline=th["contour"][1], dash=(2, 4), tags="ring")
                lab = "%.2g %s" % (step * i / 1852.0, T("hud.nm")) if step >= 185 else "%.0f m" % (step * i)
                c.create_text(bx + r * 0.707 + 3, by - r * 0.707 - 3, text=lab, anchor="sw", fill=th["text"],
                              font=f_small, tags="ring")
        # tracks (step 13n3): the saved ones shown (grey dashed + date label), the live one (white / black by the
        # background, every point kept - only the drawing is simplified + cut to the window)
        for path, tr in a.tracks_on_map():
            runs = tr.runs(P, w, h, m)
            for pts in runs:
                c.create_line(*pts, fill=th["track_old"], width=2, dash=(6, 4), tags="track_old")
            if runs:
                x0, y0 = runs[0][0], runs[0][1]
                c.create_text(x0 + 6, y0 - 6, text=a.track_label(tr), anchor="sw", fill=th["track_old"],
                              font=f_small, tags="track_old")
        lv = a.tracks.live
        if lv is not None:
            for pts in lv.track.runs(P, w, h, m):
                c.create_line(*pts, fill=th["track"], width=2, tags="track")
        # the UAVs (step U4): the owner's picture turned with the heading, their own track style + a label
        if getattr(a, "uav_store", None) is not None:
            import uavmap
            uavmap.draw(c, P, a.uav_store, th, f_small, k=max(1.0, theme.unit(w, h)))
        if getattr(a, "adsb_state", None) is not None:
            import adsbmap
            adsbmap.draw(c, P, a, th, f_small, w, h, k=max(1.0, theme.unit(w, h)))
        if getattr(a, "sar", None) is not None:
            self.draw_sar(c, P, th, f_small)
            self.draw_points(c, P, th, f_small)
            self.draw_pattern(c, P, th, f_small)
        self.draw_asked_routes(c, P, th, f_small)
        if getattr(a, "harbour_model", None) is not None or getattr(a, "harbour_preview", None):
            import harbourmap                                # HX6: corridor / tracks / planned path (display only)
            harbourmap.draw(c, P, a, th, f_small, w, h, k=max(1.0, theme.unit(w, h)))
        pk = getattr(a, "track_pick", None)
        if pk:
            x, y = P(pk["lat"], pk["lon"])
            c.create_oval(x - 6, y - 6, x + 6, y + 6, outline=th["text"], width=2, tags="track_pick")
            c.create_text(x + 10, y - 8, text=pk["text"], anchor="sw", fill=th["text"], font=f_label,
                          tags="track_pick")
        # the geofence (as sent / being edited)
        fence = a.fence_draft if (self.edit == "fence" or a.fence_dirty) else a.fence_live()
        if fence and len(fence) >= 2:
            pts = [v for la, lo in fence for v in P(la, lo)]
            col = theme.CAUTION if (self.edit == "fence" or a.fence_dirty) else th["good"]
            if len(fence) >= 3:
                c.create_polygon(*pts, outline=col, fill="", width=3, dash=(10, 5), tags="fence")
            else:
                c.create_line(*pts, fill=col, width=3, dash=(10, 5), tags="fence")
            if self.edit == "fence":
                for i, (la, lo) in enumerate(fence):
                    x, y = P(la, lo)
                    c.create_rectangle(x - 6, y - 6, x + 6, y + 6, outline=col, width=2, fill=th["bg"],
                                       tags="fence")
        # the route (the editor's program) + the active waypoint
        wps = (a.prog or {}).get("wps") or []
        wpts = [P(wp["lat"], wp["lon"]) for wp in wps]
        rcol = theme.CAUTION if (self.edit == "route" or a.route_dirty) else th["route"]
        if len(wpts) >= 2:
            c.create_line(*[v for p in wpts for v in p], fill=rcol, width=2 if self.edit != "route" else 3,
                          dash=(8, 4), tags="prog")
        for i, (x, y) in enumerate(wpts):
            r = 8 if self.edit == "route" else 6
            c.create_oval(x - r, y - r, x + r, y + r, outline=rcol, width=2, tags="prog")
            c.create_text(x + r + 3, y, text=str(i + 1), anchor="w", font=f_label, fill=rcol, tags="prog")
        if d and d.get("wp_set"):
            x, y = P(d["wp_lat"], d["wp_lon"])
            c.create_line(bx, by, x, y, fill=th["route"], dash=(6, 4), width=2, tags="active_wp")
            c.create_oval(x - 11, y - 11, x + 11, y + 11, outline=th["route"], width=4, tags="active_wp")
        mk = getattr(a, "map_mark", None)
        if mk:
            x, y = P(*mk)
            c.create_line(x - 12, y - 12, x + 12, y + 12, fill=th["warn"], width=4, tags="mark")
            c.create_line(x - 12, y + 12, x + 12, y - 12, fill=th["warn"], width=4, tags="mark")
            c.create_text(x + 14, y, anchor="w", text=T("map.refused_leg"), fill=th["warn"], font=f_label, tags="mark")
        if d and d.get("drift_kn", 0.0) > 0.02:
            ang = math.radians(d["set_deg"])
            L = min(120.0, d["drift_kn"] * 60.0)
            x2, y2 = bx + L * math.sin(ang), by - L * math.cos(ang)
            c.create_line(bx, by, x2, y2, arrow="last", arrowshape=(14, 16, 6), fill=th["route"], width=3, tags="drift")
            c.create_text(x2, y2 - 12, text=T("map.drift", kn="%.2f" % d["drift_kn"]), font=f_small,
                          fill=th["route"], tags="drift")
        if not a.charts:
            c.create_text(w / 2, h / 2 + 0.24 * min(w, h), text=T("cap.no_charts"), fill=th["warn"], font=f_label,
                          tags="nocharts")
        if d:
            r = d["unc_m"] / m
            if r > 1:
                c.create_oval(bx - r, by - r, bx + r, by + r, outline=th["warn"] if d["pos_source"] in (2, 3) else
                              th["good"], width=2, dash=(4, 3), tags="unc")
            # own boat: always north-up, the symbol turns with the heading
            hd = math.radians(d["heading"])
            k = max(1.0, theme.unit(w, h) * 1.2)
            pts = []
            for ang, rr in ((0, 20), (140, 13), (180, 6), (220, 13)):
                bb = hd + math.radians(ang)
                pts += [bx + k * rr * math.sin(bb), by - k * rr * math.cos(bb)]
            c.create_polygon(*pts, fill=th["own"], outline=theme.BLACK, width=2, tags="boat")
            cg = math.radians(d["est_cog"])
            L = min(200.0, d["est_sog"] * 1852.0 / 10.0 / m)            # 6 minutes over ground
            if L > 4:
                c.create_line(bx, by, bx + L * math.sin(cg), by - L * math.cos(cg), fill=th["own"], width=2,
                              arrow="last", tags="boat")
        self._tag(mark)

    def draw_sar(self, c, P, th, font):
        """Open MOB markers (PF3): a cross + label, the drift ESTIMATE (dashed line + circle, said to be an estimate).
        Red only because a MOB is a WARNING alarm (the colour code)."""
        import mob
        a = self.app
        now = time.time()
        col = th["warn"]
        for m in a.sar.open_mobs():
            x, y = P(m["lat"], m["lon"])
            c.create_line(x - 9, y - 9, x + 9, y + 9, fill=col, width=3, tags=("sar", "sar_mob"))
            c.create_line(x - 9, y + 9, x + 9, y - 9, fill=col, width=3, tags=("sar", "sar_mob"))
            c.create_text(x + 12, y - 10, text=mob.line(m, a.mob_boat(), now), anchor="sw", fill=col, font=font,
                          tags=("sar", "sar_label"))
            est = mob.drift_estimate(m, now)
            if est:
                ex, ey = P(est["lat"], est["lon"])
                rpx = abs(P(est["lat"] + est["r_m"] / 111320.0, est["lon"])[1] - ey)
                c.create_line(x, y, ex, ey, fill=col, width=2, dash=(5, 4), tags=("sar", "sar_drift"))
                c.create_oval(ex - rpx, ey - rpx, ex + rpx, ey + rpx, outline=col, width=2, dash=(5, 4),
                              tags=("sar", "sar_drift"))
                c.create_text(ex + 6, ey + rpx + 4, text=T("mob.estimate"), anchor="nw", fill=col, font=font,
                              tags=("sar", "sar_drift"))

    def draw_points(self, c, P, th, font):
        """Points marked from the video (PF4): a diamond, the name, the dashed uncertainty circle (the radius grows with
        the range) and ALWAYS the ESTIMATE text. A person in the water gets the extra note (not red: it is no alarm)."""
        a = self.app
        col = theme.CAUTION
        for m in a.mark_points():
            x, y = P(m["lat"], m["lon"])
            r_m = float(m.get("r_m") or 0.0)
            rpx = max(0.0, abs(P(m["lat"] + r_m / 111320.0, m["lon"])[1] - y))
            if rpx > 3:
                c.create_oval(x - rpx, y - rpx, x + rpx, y + rpx, outline=col, width=2, dash=(5, 4),
                              tags=("sar", "sar_point"))
            c.create_polygon(x, y - 8, x + 8, y, x, y + 8, x - 8, y, outline=col, fill="", width=3,
                             tags=("sar", "sar_point"))
            name = m.get("name") or T("mark.name", n=m.get("n"))
            c.create_text(x + 12, y - 10, text=T("mark.label", name=name, r="%d" % round(r_m)), anchor="sw",
                          fill=col, font=font, tags=("sar", "sar_label"))
            note = T("mark.estimate") + ("  ·  " + T("mark.person_note") if m.get("person") else "")
            c.create_text(x + 12, y + 8, text=note, anchor="nw", fill=col, font=font, tags=("sar", "sar_label"))

    def draw_pattern(self, c, P, th, font):
        """The search pattern being prepared (PF2): numbered legs, the datum; CAUTION colour = not sent yet."""
        pl = getattr(self.app, "sar_preview", None)
        if not pl:
            return
        col = theme.CAUTION
        pts = [P(la, lo) for la, lo in pl["wps"]]
        if len(pts) >= 2:
            c.create_line(*[v for p in pts for v in p], fill=col, width=2, dash=(6, 3), tags=("sar", "sar_pat"))
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            mx, my = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
            c.create_text(mx, my, text=str(i + 1), fill=col, font=font, tags=("sar", "sar_pat"))
        for x, y in pts[:1]:
            c.create_oval(x - 7, y - 7, x + 7, y + 7, outline=col, width=3, tags=("sar", "sar_pat"))
        if pts:
            c.create_text(pts[0][0] + 10, pts[0][1] - 10, text=T("pat.draw_note"), anchor="sw", fill=col, font=font,
                          tags=("sar", "sar_pat"))

    def draw_asked_routes(self, c, P, th, font):
        """PF4-r MED-2c: a UAV route that waits for the captain's YES (driver_ask "uav_route") is DRAWN while the
        question is open: CAUTION dashed, numbered points, «ΖΗΤΕΙΤΑΙ ΕΓΚΡΙΣΗ – UAV n» (display only)."""
        for r in list((getattr(self.app, "ask_routes", None) or {}).values()):
            col = theme.CAUTION
            pts = [P(w[0], w[1]) for w in r.get("wps") or [] if len(w) >= 2]
            if len(pts) >= 2:
                c.create_line(*[v for p in pts for v in p], fill=col, width=2, dash=(3, 3), tags=("ask_route",))
            for i, (x, y) in enumerate(pts):
                c.create_oval(x - 5, y - 5, x + 5, y + 5, outline=col, width=2, tags=("ask_route",))
                c.create_text(x + 8, y + 8, text=str(i + 1), anchor="nw", fill=col, font=font, tags=("ask_route",))
            if pts:
                c.create_text(pts[-1][0] + 10, pts[-1][1] - 10, text=T("uavr.ask_note", n=r.get("slot", "?")),
                              anchor="sw", fill=col, font=font, tags=("ask_route",))

    def _tag(self, mark):
        """Everything drawn after `mark` belongs to the map (below the panels)."""
        c = self.cv
        for i in c.find_all():
            if i > mark:
                c.addtag_withtag("map", i)
        c.delete(mark)
        c.tag_lower("map")
