"""The guided tour's REAL screenshots (step 13t, owner queue 2026-11-05 item 3a): which picture a tour stop
shows and which frame it lights, read from academy/tour_shots/shots.json.

The pictures + every item's place come from the real ground station running the demo
(tools/demo_screens.py --tour): screen 1 (the HUD over the camera video; its boxes from hud.Layout, its
buttons from the HUD's own hit boxes, the question box from its widgets), screen 2 (the overlay's panels,
the mode bar, the bars and their buttons from overlay.Overlay.ops, the boat from mapwin.keepout) and every
page of the tools window (the page list's buttons; on the UAV magazine page every button). A stop's frame
= the union of the places of the screen items it covers (else its target's place), so a highlight is
always where the real button is.

Pure model - no window, no network, no process (test_academy AC1-AC3). The window is tour_app.py.

    python academy/tourshots.py      -> the check (exit 1 when a picture / place is missing or stale)
"""
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SHOTS_DIR = os.path.join(HERE, "tour_shots")
MANIFEST = os.path.join(SHOTS_DIR, "shots.json")
SCREEN_SHOWS = ("s1", "s2", "tools")        # the stops shown on a real picture (the others keep the drawings)
# the tools window pictures: one per page
ITEM_PREFIXES = ("hud:", "hudbtn:", "s2:", "tools:")


def load(path=None):
    """The manifest (dict) or None when there is none / it is broken."""
    try:
        with open(path or MANIFEST, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return None
    return doc if isinstance(doc, dict) and doc.get("kind") == "tour_shots" else None


def screen_items(tour):
    """Every covered item that is a real place on a screen (must be inside its stop's picture + frame)."""
    uav = set("uav:" + x for x in tour.uav_buttons())
    return lambda item: item.startswith(ITEM_PREFIXES) or item in uav


def signature(tour, doc=None):
    """What the pictures depend on (the LAYOUT, not the texts): the HUD boxes + logo, the HUD buttons, the
    screen-2 panels + extras, the tools pages, the UAV page buttons and every screen stop's show / target /
    covers. A different signature = the pictures are stale (tools/demo_screens.py --tour)."""
    import hud
    doc = doc or tour.load()
    parts = {
        "hud": sorted((k, [round(v, 4) for v in r]) for k, r in hud.DEFAULT_LAYOUT.items()),
        "logo": [round(v, 4) for v in hud.LOGO_BOX],
        "hudbtn": tour.hud_buttons() + list(tour.HUD_BUTTONS_EXTRA),
        "s2": tour.s2_panels() + list(tour.S2_EXTRA),
        "tools": tour.tools_pages(),
        "uav": tour.uav_buttons(),
        "stops": [(st["id"], st["show"], st.get("target", ""), st.get("covers") or [])
                  for ch in doc["chapters"] for st in ch["stops"] if st["show"] in SCREEN_SHOWS],
    }
    return hashlib.sha1(json.dumps(parts, sort_keys=True, ensure_ascii=True).encode("ascii")).hexdigest()[:16]


def shot_of(stop, lang):
    """The picture name of a stop: s1 (the question box for the YES / NO stop), s2, the tools page."""
    show = stop["show"]
    if show == "s1":
        return "s1q_%s" % lang if "hudbtn:accept" in (stop.get("covers") or []) else "s1_%s" % lang
    if show == "s2":
        return "s2_%s" % lang
    if show == "tools":
        return "tools_%s_%s" % (stop.get("target", "").replace("tab.", ""), lang)
    return None


def target_item(stop):
    show, t = stop["show"], stop.get("target", "")
    if not t:
        return None                                 # the whole screen (an overview stop)
    if show == "s1":
        return "hud:" + t
    if show == "s2":
        return "s2:" + t
    return "tools:" + t


def union(rects):
    rects = [r for r in rects if r]
    if not rects:
        return None
    x0 = min(r[0] for r in rects)
    y0 = min(r[1] for r in rects)
    x1 = max(r[0] + r[2] for r in rects)
    y1 = max(r[1] + r[3] for r in rects)
    return [x0, y0, x1 - x0, y1 - y0]


def view(stop, lang, man, is_item):
    """-> {"file", "w", "h", "frame" [fx, fy, fw, fh] fractions, "items" {item: rect}} or None (no picture)."""
    if man is None or stop["show"] not in SCREEN_SHOWS:
        return None
    name = shot_of(stop, lang)
    sh = (man.get("shots") or {}).get(name)
    if not sh:
        return None
    places = sh.get("items") or {}
    covers = [c for c in (stop.get("covers") or []) if is_item(c)]
    items = {c: places.get(c) for c in covers}
    frame = union(items.values())
    if frame is None:
        ti = target_item(stop)
        frame = places.get(ti) if ti else [0.0, 0.0, 1.0, 1.0]
    if frame is None:
        return None
    return {"name": name, "file": os.path.join(SHOTS_DIR, sh["file"]), "w": sh["w"], "h": sh["h"],
            "frame": frame, "items": items}


def inside(r, box, tol=0.002):
    return (r[0] >= box[0] - tol and r[1] >= box[1] - tol and r[0] + r[2] <= box[0] + box[2] + tol and
            r[1] + r[3] <= box[1] + box[3] + tol)


def check(tour, man=None, doc=None, langs=None):
    """Problems (empty = OK): the manifest is there and current, every screen stop of both languages has its
    picture (the file, the size), a frame inside the picture and EVERY screen item it covers inside the
    picture and inside its frame."""
    man = load() if man is None else man
    doc = doc or tour.load()
    if man is None:
        return ["no tour pictures (academy/tour_shots/shots.json): python tools/demo_screens.py --tour"]
    p = []
    sig = signature(tour, doc)
    if man.get("signature") != sig:
        p.append("the tour pictures are STALE (signature %s, the code now %s): python tools/demo_screens.py --tour"
                 % (man.get("signature"), sig))
    is_item = screen_items(tour)
    whole = [0.0, 0.0, 1.0, 1.0]
    for lc in langs or tour.LANGS:
        for ch in doc["chapters"]:
            for st in ch["stops"]:
                if st["show"] not in SCREEN_SHOWS:
                    continue
                v = view(st, lc, man, is_item)
                if v is None:
                    p.append("%s %s: no picture / frame (%s)" % (st["id"], lc, shot_of(st, lc)))
                    continue
                if not os.path.isfile(v["file"]):
                    p.append("%s %s: picture file %s missing" % (st["id"], lc, os.path.basename(v["file"])))
                if not inside(v["frame"], whole) or v["frame"][2] <= 0 or v["frame"][3] <= 0:
                    p.append("%s %s: frame %s outside the picture" % (st["id"], lc, v["frame"]))
                for it, r in v["items"].items():
                    if r is None:
                        p.append("%s %s: %s is NOT in the picture %s" % (st["id"], lc, it, v["name"]))
                    elif not inside(r, whole):
                        p.append("%s %s: %s %s outside the picture" % (st["id"], lc, it, r))
                    elif not inside(r, v["frame"]):
                        p.append("%s %s: %s %s outside the frame %s" % (st["id"], lc, it, r, v["frame"]))
    return p


def main():
    sys.path.insert(0, HERE)
    import tour
    probs = check(tour)
    for x in probs:
        print("PROBLEM", x)
    man = load()
    print("%d pictures, %d problems" % (len((man or {}).get("shots") or {}), len(probs)))
    return 1 if probs else 0


if __name__ == "__main__":
    sys.exit(main())
