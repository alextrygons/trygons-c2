"""The SAR demo (CLAUDE.md X, K5, step PF6) - a PRESENTER'S GUIDE for the search-and-rescue tools, simulator only.

Started by DEMO_SAR.bat (= the normal demo + this window; `python demo/demo.py start --sar --splash`) or alone
against a running demo:  python demo/sar_demo.py

WHAT IT IS: a small always-on-top window that shows the presenter the NEXT STEP (Greek / English) and ticks it off
when it sees it happen in the demo (it only LISTENS to the gateway as the read-only demo user "tool"):
  1 boat in AUTO          2 MOB pressed        3 return route offered (the captain's YES)
  4 UAV over the MOB (the launch question: the captain's YES)   5 expanding-square search for a second person
  6 mark that person from the UAV video         7 the helicopter crosses (ADS-B warning)   8 the UAV's HOLD proposal
WHAT IT NEVER DOES: it never answers a YES / NO question, never presses a boat button, never sends a command to the
gateway or the boat (no message leaves this program: the link object is used for receiving only; the test
SD1 proves it). Every step that needs the captain's decision WAITS for the presenter (the mouse or the shore
stick's trigger on the question box). The ONLY thing it does besides showing text: the button «Send the helicopter»
writes a cue file that the SIMULATED ADS-B receiver (gateway/simadsb.py, TRYGONS_SIM_ADSB_CUE) waits for - it
moves a simulated aircraft, nothing else. Steps without a detectable result (3, 5, 8) are ticked by the
presenter's «Done» button, any step can be skipped / repeated.

The pure part (Steps / SarDemo.feed) has no GUI and no network: groundstation tests / demo/test_sar_demo.py use it.
"""
import json
import os
import queue
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
RUN = os.path.join(REPO, "demo_run")
CUE = os.path.join(RUN, "adsb_cue")
GATEWAY = "127.0.0.1:5761"
USER = "tool"
PASSWORD = "demo"

# (id, auto-detected?, el prompt, en prompt)
STEPS = (
    ("auto", True,
     "ΒΗΜΑ 1 – Το σκάφος σε AUTO. Οθόνη 2 → Πρόγραμμα → «Φόρτωση αρχείου» demo\\demo_mission.json → AUTO START → "
     "επιβεβαίωση (ΣΤΑΘΜΟΣ ΜΕ ΧΕΙΡΙΣΤΗΡΙΟ).",
     "STEP 1 – The boat in AUTO. Screen 2 → Program → \"Load file\" demo\\demo_mission.json → AUTO START → "
     "confirm (the station with the joystick)."),
    ("mob", True,
     "ΒΗΜΑ 2 – Πατήστε MOB (κουμπί ή πλήκτρο M). Σημειώνεται η θέση, ακούγεται «Άνθρωπος στη θάλασσα».",
     "STEP 2 – Press MOB (the button or the M key). The position is marked, you hear \"Man overboard\"."),
    ("return", False,
     "ΒΗΜΑ 3 – Προτείνεται διαδρομή επιστροφής (τελειώνει 30 μ. ΠΡΙΝ από τον άνθρωπο). Δείτε τη στον χάρτη και "
     "απαντήστε ΝΑΙ στο ερώτημα (ποντίκι ή σκανδάλη). Μετά πατήστε «Έγινε».",
     "STEP 3 – A return route is offered (ends 30 m BEFORE the person). Look at it on the map and answer YES in the "
     "question box (mouse or trigger). Then press \"Done\"."),
    ("uav", True,
     "ΒΗΜΑ 4 – «UAV πάνω από το σημείο»: ΝΑΙ στη διαδρομή, ΝΑΙ στην έγκριση εκτόξευσης (καπετάνιος). Ο θάλαμος "
     "ανοίγει, το UAV 1 πετά.",
     "STEP 4 – \"UAV over the spot\": YES to the route, YES to the launch approval (captain). The hatch opens, UAV 1 flies."),
    ("pattern", False,
     "ΒΗΜΑ 5 – Δεύτερο άτομο: δεξί κλικ στον χάρτη → «Μοτίβο έρευνας εδώ» → Επεκτεινόμενο τετράγωνο → "
     "στόχος UAV → ΝΑΙ. Μετά πατήστε «Έγινε».",
     "STEP 5 – A second person: right-click on the map → \"Search pattern here\" → Expanding square → "
     "target UAV → YES. Then press \"Done\"."),
    ("mark", True,
     "ΒΗΜΑ 6 – Βίντεο UAV 1 (κάμερα οθόνης 2 → UAV-1): ο φωτεινός κύκλος είναι το άτομο. Κλικ πάνω του (ή πλήκτρο K) → "
     "σημειώνεται εκτίμηση θέσης με κύκλο αβεβαιότητας.",
     "STEP 6 – UAV 1 video (screen 2 camera view → UAV-1): the bright disc is the person. Click it (or the K key) → "
     "an estimated position with an uncertainty circle is marked."),
    ("heli", True,
     "ΒΗΜΑ 7 – Πατήστε «Στείλε το ελικόπτερο» εδώ. Εμφανίζεται στον χάρτη (ADS-B), CAUTION / WARNING «Αεροσκάφος "
     "κοντά στο UAV 1».",
     "STEP 7 – Press \"Send the helicopter\" here. It appears on the map (ADS-B), CAUTION / WARNING \"Aircraft near UAV 1\"."),
    ("hold", False,
     "ΒΗΜΑ 8 – Ερώτημα «ΚΡΑΤΗΣΗ / ΚΑΤΕΒΑΣΜΑ UAV 1»: απαντήστε εσείς (ΝΑΙ ή ΟΧΙ). Τίποτα δεν γίνεται αυτόματα. "
     "Μετά πατήστε «Έγινε».",
     "STEP 8 – The question \"HOLD / DESCEND UAV 1\": you answer (YES or NO). Nothing happens by itself. "
     "Then press \"Done\"."),
    ("end", False,
     "ΤΕΛΟΣ – Η επίδειξη ολοκληρώθηκε. Κλείστε το παράθυρο· STOP_DEMO.bat σταματά τα πάντα.",
     "END – The demo is finished. Close this window; STOP_DEMO.bat stops everything."),
)
IDS = tuple(s[0] for s in STEPS)


class SarDemo:
    """The progress model. feed(kind, data) takes the events of a gateway link (telemetry / gw_status /
    sar_markers); `step` is the index of the step shown. Pure: no network, no GUI, nothing is ever sent."""

    def __init__(self):
        self.step = 0
        self.mode = ""
        self.mobs = 0                # open MOB markers seen
        self.finds = 0               # other markers (a point / find marked from the video), after the MOB
        self.uav_flying = False
        self.adsb_warn = False
        self.adsb_prop = False
        self.heli_sent = False
        self.done = set()
        self.link_ok = False

    def feed(self, kind, d):
        if not isinstance(d, dict):
            return
        if kind == "telemetry":
            self.mode = str(d.get("mode_name") or self.mode)
        elif kind == "sar_markers":
            lst = [m for m in (d.get("list") or []) if isinstance(m, dict) and not m.get("closed")]
            self.mobs = sum(1 for m in lst if m.get("kind") == "mob")
            self.finds = sum(1 for m in lst if m.get("kind") in ("point", "find"))
        elif kind == "gw_status":
            u = d.get("uav") or {}
            fl = u.get("flying") if isinstance(u, dict) else None
            self.uav_flying = bool(fl)
            a = d.get("adsb") or {}
            self.adsb_warn = bool(isinstance(a, dict) and a.get("warn"))
            self.adsb_prop = bool(isinstance(a, dict) and a.get("proposals"))
        self._advance()

    def seen(self, sid):
        """Has the demo shown that this step happened?"""
        if sid == "auto":
            return self.mode == "AUTO"
        if sid == "mob":
            return self.mobs > 0
        if sid == "uav":
            return self.uav_flying
        if sid == "mark":
            return self.finds > 0
        if sid == "heli":
            return self.heli_sent and (self.adsb_warn or self.adsb_prop)
        return False

    def _advance(self):
        while self.step < len(STEPS) - 1:
            sid, auto = STEPS[self.step][0], STEPS[self.step][1]
            if (auto and self.seen(sid)) or sid in self.done:
                self.step += 1
            else:
                break

    def done_pressed(self):
        """The presenter's «Done» (the only way past a step the demo can't see)."""
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

    def can_heli(self):
        return STEPS[self.step][0] == "heli"


def send_helicopter():
    """The one action: the simulated ADS-B receiver waits for this cue file (demo.py start --sar sets
    TRYGONS_SIM_ADSB_CUE). It moves a SIMULATED aircraft only."""
    os.makedirs(RUN, exist_ok=True)
    with open(CUE, "w", encoding="utf-8") as f:
        f.write("go")


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
                self.link = GatewayLink(GATEWAY, USER, password=PASSWORD, station="SAR demo guide")
                self.link.start()
                self.q.put(("link", {"ok": True}))
                while not self.stop_ev.is_set() and self.link.connected:
                    try:
                        k, v = self.link.events.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    if k in ("telemetry", "gw_status", "sar_markers"):
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
    root.title("Trygons C2 – SAR demo")
    root.attributes("-topmost", True)
    root.geometry("560x330+40+40")
    m = SarDemo()
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
    T = {"el": ("Έγινε", "Πίσω", "Στείλε το ελικόπτερο", "EN", "Κλείσιμο",
                "Ο οδηγός ΜΟΝΟ ακούει και δείχνει. ΔΕΝ απαντά ΝΑΙ/ΟΧΙ και δεν πατά κουμπιά του σκάφους.",
                "Αναμονή για το gateway…"),
         "en": ("Done", "Back", "Send the helicopter", "ΕΛ", "Close",
                "This guide only listens and shows. It never answers YES/NO and never presses a boat button.",
                "Waiting for the gateway…")}
    b_done = tk.Button(bar, command=m.done_pressed)
    b_heli = tk.Button(bar, command=lambda: (send_helicopter(), m.__setattr__("heli_sent", True)))
    b_lang = tk.Button(bar, command=lambda: state.__setitem__("lang", "en" if state["lang"] == "el" else "el"))
    b_close = tk.Button(bar, command=lambda: close())
    for b in (b_done, b_heli, b_lang, b_close):
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
        b_heli.config(text=t[2], state="normal" if m.can_heli() else "disabled")
        b_lang.config(text=t[3])
        b_close.config(text=t[4])
        head.config(text="SAR demo  %d / %d  %s" % (min(m.step + 1, len(STEPS) - 1), len(STEPS) - 1,
                                                    "" if m.link_ok else "– " + t[6]))
        body.config(text=m.prompt(state["lang"]))
        note.config(text=t[5])
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
