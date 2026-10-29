"""The academy's PRACTICAL TEST (ACADEMY_SPEC G + J, step 21; design decision D4 / D17).

The ONLY academy code allowed on the normal boat link - and only through linkview.LinkView, which can READ the
boat's telemetry + the station's audit log and SEND exactly one frame type: PROTO_T_CONFIRM_ONLY (a question for
the onboard joystick CONFIRM that changes nothing on the boat). No other academy module imports this package
(test_academy AC14); its own proof is practical/test_practical.py.
"""
