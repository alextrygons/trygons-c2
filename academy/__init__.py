"""Trygons C2 Academy (CLAUDE.md section S, docs/ACADEMY_SPEC.md).

Training for observers, drivers and captains. GROUND RULE: the academy runs ONLY against the
simulator (autopilot_sim server, the demo) or recorded files - never a real-boat link.

  simguard.py  the ONLY academy module that may open a connection or a recording; it refuses
               everything that is not the simulator on this PC (loopback + sim ports + a sim boat
               name) or a recorded file (test_academy.py AC1-AC7 prove it)
  banner.py    the banner «ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ» every academy screen shows
  text.py      screen texts from lang/el.txt (default) + lang/en.txt
  accounts.py  levels OBSERVER -> DRIVER -> CAPTAIN (step 15: the model only; storage + the
               gateway push = step 16)
  content.py   loads + checks the commented content files under content/

The PRACTICAL TEST (spec G, step 21) is the one exception: it will live in academy/practical/
(not created yet), read-only on the normal boat link; no other academy module may import it.

Step 15 = this skeleton + the tests; nothing here changes the boat, the gateway or the ground
station.
"""
