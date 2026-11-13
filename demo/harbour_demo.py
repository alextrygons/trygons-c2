"""The HARBOUR demo (CLAUDE.md Z, step HX7) - a PRESENTER'S GUIDE for "harbour exit with traffic", simulator only.

Started by DEMO_HARBOUR.bat (= the normal demo + this window; `python demo/demo.py start --harbour --splash`) or alone
against a running demo:  python demo/harbour_demo.py

WHAT IT IS: a small always-on-top window that shows the presenter the NEXT STEP (Greek / English) and ticks it off when
it sees it happen (it only LISTENS to the gateway as the read-only demo user "tool": telemetry + gw_status "harbour"):
  1 the harbour sensors OK        2 a harbour route stored (TRACKS -> HARBOUR ROUTE)    3 START (the captain's YES, then
  the CONFIRM)    4 the boat RUNs the route under autonomy    5 the crossing ferry / kayak: SLOW or HOLD
  6 RESUME or TAKE OVER (the presenter decides)    7 STOP
WHAT IT NEVER DOES: it never answers a YES / NO question, never presses a boat button, never sends a command to the
gateway or the boat (the link object is used for receiving only; demo/test_harbour_demo.py HD1 / HD2 prove it). The
captain's YES and the CONFIRM are clicked by the presenter. Steps without a detectable result are ticked by the
presenter's "Done" button.

The simulated harbour (gateway/harbour/simworld.py via demo_world.harbour_world(): moored boats, a crossing ferry with
AIS, a vessel and a kayak without AIS, a swimmer, a buoy) is placed around the boat's first position. The pure part
(HarbourDemo.feed) has no GUI and no network.
"""
import os
import queue
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
RUN = os.path.join(REPO, "demo_run")
GATEWAY = "127.0.0.1:5761"
USER = "tool"
PASSWORD = "demo"

# (id, auto-detected?, el prompt, en prompt)
STEPS = (
    ("panel", True,
     "ΒΗΜΑ 1 – Οθόνη 2: το πλαίσιο «ΛΙΜΕΝΑΣ» εμφανίζεται όταν υπάρχει διαδρομή λιμένα. Δείτε στον χάρτη τα σκάφη, την "
     "προβλήτα, τη σημαδούρα (σύμβολα ραντάρ / LiDAR). Ραντάρ και LiDAR πρέπει να είναι OK.",
     "STEP 1 – Screen 2: the HARBOUR panel appears once a harbour route exists. On the map see the boats, the pier, the "
     "buoy (radar / LiDAR symbols). Radar and LiDAR must be OK."),
    ("route", True,
     "ΒΗΜΑ 2 – Διαδρομή λιμένα: ΔΙΑΔΡΟΜΕΣ → «ΔΙΑΔΡΟΜΗ ΛΙΜΕΝΑ» → ΔΗΜΙΟΥΡΓΙΑ → ΕΛΕΓΧΟΣ → ΑΠΟΣΤΟΛΗ (μετά από μια "
     "καταγραφή με το χέρι). Το σκάφος την αποθηκεύει· δεν ξεκινά.",
     "STEP 2 – A harbour route: TRACKS -> \"HARBOUR ROUTE\" -> MAKE -> CHECK -> SEND (after a recorded manual track). The "
     "boat stores it; nothing starts."),
    ("start", True,
     "ΒΗΜΑ 3 – ΕΝΑΡΞΗ: ΝΑΙ στην ερώτηση (ποντίκι ή σκανδάλη), μετά CONFIRM στο χειριστήριο του σκάφους. Εσείς πατάτε· "
     "αυτός ο οδηγός δεν απαντά ποτέ.",
     "STEP 3 – START: YES in the question (mouse or trigger), then CONFIRM on the boat's joystick. You press; this guide "
     "never answers."),
    ("run", True,
     "ΒΗΜΑ 4 – Το σκάφος ακολουθεί τη διαδρομή μόνο του (RUN), ≤ 5 kn, μέσα στη ζώνη ±10 m. Δείξτε τη ζώνη και τη "
     "σχεδιασμένη πορεία.",
     "STEP 4 – The boat follows the route by itself (RUN), <= 5 kn, inside the +-10 m corridor. Point at the corridor and "
     "the planned path."),
    ("traffic", True,
     "ΒΗΜΑ 5 – Το πλοιάριο που διασταυρώνεται / το καγιάκ: το σκάφος επιβραδύνει (SLOW) ή σταματά (HOLD, "
     "«ΑΒΕΒΑΙΟΤΗΤΑ»). Δείξτε τον λόγο στο πλαίσιο. Δεν στρίβει ποτέ για κάμερα ή AIS μόνο.",
     "STEP 5 – The crossing ferry / the kayak: the boat slows (SLOW) or stops (HOLD, \"UNSURE\"). Show the reason in the "
     "panel. It never turns for a camera or AIS alone."),
    ("decide", False,
     "ΒΗΜΑ 6 – Αποφασίζετε ΕΣΕΙΣ: ΣΥΝΕΧΕΙΑ (όταν καθαρίσει) ή ΑΝΑΛΗΨΗ ΕΛΕΓΧΟΥ με το joystick. Μετά πατήστε «Έγινε».",
     "STEP 6 – YOU decide: RESUME (when it is clear) or TAKE OVER with the joystick. Then press \"Done\"."),
    ("stop", True,
     "ΒΗΜΑ 7 – Πατήστε STOP (ή STANDBY στον πίνακα): κερδίζει πάντα, ρελαντί και νεκρά.",
     "STEP 7 – Press STOP (or the panel STANDBY): it always wins, idle and neutral."),
    ("end", False,
     "ΤΕΛΟΣ – Η επίδειξη ολοκληρώθηκε. Κλείστε το παράθυρο· STOP_DEMO.bat σταματά τα πάντα.",
     "END – The demo is finished. Close this window; STOP_DEMO.bat stops everything."),
)
IDS = tuple(s[0] for s in STEPS)


class HarbourDemo:
    """The progress model. feed(kind, data) takes the events of a gateway link (telemetry / gw_status); `step` is the
    index of the step shown. Pure: no network, no GUI, nothing is ever sent."""

    def __init__(self):
        self.step = 0
        self.mode = ""
        self.state = ""              # the AURIX harbour state: OFF / RUN / SLOW / HOLD / DONE
        self.route = False           # the planner is active (a harbour route on the boat)
        self.sensors_ok = False
        self.ran = False             # RUN was seen
        self.traffic = False         # SLOW / HOLD was seen after RUN
        self.stopped = False         # STANDBY seen after the run
        self.done = set()
        self.link_ok = False

    def feed(self, kind, d):
        if not isinstance(d, dict):
            return
        if kind == "telemetry":
            self.mode = str(d.get("mode_name") or self.mode)
            if self.ran and self.mode == "STANDBY":
                self.stopped = True
        elif kind == "gw_status":
            h = d.get("harbour") or {}
            if isinstance(h, dict):
                s = h.get("sensors") or {}
                live = [v for v in s.values() if isinstance(v, dict) and v.get("state") != "OFF"]
                self.sensors_ok = bool(live) and all(v.get("state") == "OK" for v in live)
                pl = h.get("planner") or {}
                st = pl.get("stat") or {}
                self.state = str(st.get("harb_state_name") or self.state)
                if self.state == "RUN":
                    self.ran = True
                if self.ran and self.state in ("SLOW", "HOLD"):
                    self.traffic = True
                self.route = bool(pl.get("active")) or self.route
        self._advance()

    def seen(self, sid):
        if sid == "panel":
            return self.sensors_ok
        if sid == "route":
            return self.route or self.ran
        if sid == "start":
            return self.mode == "AUTO" or self.ran
        if sid == "run":
            return self.ran
        if sid == "traffic":
            return self.traffic
        if sid == "stop":
            return self.stopped
        return False

    def _advance(self):
        while self.step < len(STEPS) - 1:
            sid, auto = STEPS[self.step][0], STEPS[self.step][1]
            if (auto and self.seen(sid)) or sid in self.done:
                self.step += 1
            else:
                break

    def done_pressed(self):
        """The presenter's "Done" (the only way past a step the demo can't see)."""
        if self.step < len(STEPS) - 1:
            self.done.add(STEPS[self.step][0])
            self._advance()

    def back(self):
        if self.step > 0:
            self.step -= 1
            self.done.discard(STEPS[self.step][0])

    def prompt(self, lang="el"):
        s = STEPS[self.step]
        return s[2] if lang == "el" else s[3]


class Listener(threading.Thread):
    """Receives from the gateway as the read-only user "tool" (events only; this class has no send call)."""

    def __init__(self, q):
        super().__init__(daemon=True)
        self.q = q
        self.stop_ev = threading.Event()
        self.link = None

    def run(self):
        sys.path.insert(0, os.path.join(REPO, "gateway"))
        sys.path.insert(0, os.path.join(REPO, "groundstation"))
        from gwclient import GatewayLink
        while not self.stop_ev.is_set():
            try:
                self.link = GatewayLink(GATEWAY, USER, password=PASSWORD, station="Harbour demo guide")
                self.link.start()
                self.q.put(("link", {"ok": True}))
                while not self.stop_ev.is_set() and self.link.connected:
                    try:
                        k, v = self.link.events.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    if k in ("telemetry", "gw_status"):
                        self.q.put((k, v))
            except Exception:
                pass
            self.q.put(("link", {"ok": False}))
            try:
                if self.link:
                    self.link.stop()
            except Exception:
                pass
            self.stop_ev.wait(2.0)

    def close(self):
        self.stop_ev.set()
        try:
            if self.link:
                self.link.stop()
        except Exception:
            pass


def gui(lang="el"):
    import tkinter as tk
    root = tk.Tk()
    root.title("Trygons C2 – Harbour demo")
    root.attributes("-topmost", True)
    root.geometry("560x330+40+40")
    m = HarbourDemo()
    q = queue.Queue()
    lis = Listener(q)
    lis.start()
    state = {"lang": lang}
    head = tk.Label(root, text="", font=("Segoe UI", 11, "bold"), anchor="w")
    head.pack(fill="x", padx=10, pady=(8, 2))
    body = tk.Label(root, text="", font=("Segoe UI", 12), wraplength=530, justify="left", anchor="nw")
    body.pack(fill="both", expand=True, padx=10)
    note = tk.Label(root, text="", font=("Segoe UI", 9), fg="#555", anchor="w", wraplength=530, justify="left")
    note.pack(fill="x", padx=10)
    bar = tk.Frame(root)
    bar.pack(fill="x", padx=10, pady=8)
    T = {"el": ("Έγινε", "Πίσω", "EN", "Κλείσιμο",
                "Ο οδηγός ΜΟΝΟ ακούει και δείχνει. ΔΕΝ απαντά ΝΑΙ/ΟΧΙ και δεν πατά κουμπιά του σκάφους.",
                "Αναμονή για το gateway…"),
         "en": ("Done", "Back", "ΕΛ", "Close",
                "This guide only listens and shows. It never answers YES/NO and never presses a boat button.",
                "Waiting for the gateway…")}
    b_done = tk.Button(bar, command=m.done_pressed)
    b_back = tk.Button(bar, command=m.back)
    b_lang = tk.Button(bar, command=lambda: state.__setitem__("lang", "en" if state["lang"] == "el" else "el"))
    b_close = tk.Button(bar, command=lambda: close())
    for b in (b_done, b_back, b_lang, b_close):
        b.pack(side="left", padx=3)

    def close():
        lis.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)

    def tick():
        try:
            while True:
                k, v = q.get_nowait()
                if k == "link":
                    m.link_ok = v["ok"]
                else:
                    m.feed(k, v)
        except queue.Empty:
            pass
        t = T[state["lang"]]
        b_done.config(text=t[0])
        b_back.config(text=t[1])
        b_lang.config(text=t[2])
        b_close.config(text=t[3])
        head.config(text="Harbour demo  %d / %d  %s" % (min(m.step + 1, len(STEPS) - 1), len(STEPS) - 1,
                                                        "" if m.link_ok else "– " + t[5]))
        body.config(text=m.prompt(state["lang"]))
        note.config(text=t[4])
        root.after(300, tick)

    tick()
    root.mainloop()
    lis.close()
    return 0


def main():
    lang = "en" if "--lang" in sys.argv and sys.argv[sys.argv.index("--lang") + 1:][:1] == ["en"] else "el"
    return gui(lang)


if __name__ == "__main__":
    sys.exit(main())
