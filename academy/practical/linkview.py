"""The practical test's ONE door to the boat link (ACADEMY_SPEC G: "it only observes (telemetry + audit log) and
grades, never sends its own driving commands").

LinkView wraps the ground station that runs the test (groundstation/app.py App, through
groundstation/practicalmenu.py). It offers:
  * telemetry()     -> (the newest telemetry dict incl. TELEM2, its receive time)   READ ONLY
  * audit()         -> the station's new audit / event lines since the last call    READ ONLY
  * confirm_only(t) -> sends PROTO_T_CONFIRM_ONLY with token t through the station's own command path (so the
                       station's event log and the gateway's audit log record it). Nothing else can be sent:
                       the frame type is fixed here, there is no generic send (test_practical P1 / P2).
The AURIX checks the request like every command (HELM station, CREWED, never FIRST TEST, never replacing another
pending CONFIRM) and it changes NOTHING on the boat - only the onboard joystick's CONFIRM answers it.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GS_DIR = os.path.join(os.path.dirname(os.path.dirname(HERE)), "groundstation")
if GS_DIR not in sys.path:
    sys.path.append(GS_DIR)

import proto                                                            # noqa: E402  (frame numbers only)

FRAME = proto.T_CONFIRM_ONLY            # the one frame the practical test may send


class LinkView(object):
    __slots__ = ("_app", "_last")

    def __init__(self, app):
        self._app = app
        self._last = None

    def connected(self):
        return bool(getattr(self._app, "link", None))

    def telemetry(self):
        d = getattr(self._app, "t", None)
        return (dict(d) if d else None), float(getattr(self._app, "t_rx", 0.0) or 0.0)

    def audit(self):
        """New lines of the station's event log (app.event keeps the last 200 in _evlog)."""
        lines = list(getattr(self._app, "_evlog", []) or [])
        if self._last is None:
            new = lines
        else:
            k = len(lines)
            for i in range(len(lines) - 1, -1, -1):           # after the last line we saw
                if lines[i] == self._last:
                    k = i + 1
                    break
            else:
                k = 0
            new = lines[k:]
        if lines:
            self._last = lines[-1]
        return new

    def confirm_only(self, token, what="confirm-only (practical test)"):
        """PROTO_T_CONFIRM_ONLY through the station's command path. -> the station's answer of cmd() (seq) or None."""
        token = int(token) & 0xFFFF
        if not self.connected():
            return None
        return self._app.cmd(FRAME, proto.p_confirm_only(token), what)
