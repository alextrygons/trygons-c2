"""Academy accounts + levels (ACADEMY_SPEC A) - step 15: the MODEL only.

Step 16 adds the storage (logbook on the shore PC, backup / sync), the instructor view and the
push of level + expiry to the gateway user list. Rules fixed now (CLAUDE.md section S):
  * levels OBSERVER < DRIVER < CAPTAIN; CAPTAIN = the gateway user's "licence" flag
    (gateway/users.py: HELM, REMOTE, MANEUVER, power raise, examiner);
  * the gateway user list stays the AUTHORITY: the academy only pushes (level, expiry); the
    gateway keeps its last copy when the academy can't be reached; a change never takes effect
    mid-trip (next login / HELM take);
  * an expired level counts as OBSERVER; a role UPGRADE only through the practical test
    (examiner + onboard joystick CONFIRM, spec G) - nothing online upgrades (spec J);
  * validity / reminder / currency numbers come from content/limits.json (never hard-coded).
"""
import datetime

LEVELS = ("OBSERVER", "DRIVER", "CAPTAIN")


def rank(level):
    """0 / 1 / 2; an unknown level = OBSERVER (never more rights by a typo)."""
    return LEVELS.index(level) if level in LEVELS else 0


def at_least(level, needed):
    return rank(level) >= rank(needed)


def licence(level):
    """The gateway's "licence" flag for this level (only CAPTAIN)."""
    return level == "CAPTAIN"


class Account(object):
    """One trainee / instructor (fields per spec A; the logbook entries come with step 16)."""

    def __init__(self, user, level="OBSERVER", expiry=None, instructor=False):
        self.user = user
        self.level = level if level in LEVELS else "OBSERVER"
        self.expiry = expiry                  # datetime.date or None (never had a level)
        self.instructor = bool(instructor)
        self.logbook = []                     # step 16: lessons, scores, attempts, exams, hours

    def effective_level(self, today=None):
        """The level the rules use: expired (or no expiry for a level above OBSERVER) = OBSERVER."""
        today = today or datetime.date.today()
        if self.level == "OBSERVER":
            return "OBSERVER"
        if self.expiry is None or today > self.expiry:
            return "OBSERVER"
        return self.level

    def gateway_record(self, today=None):
        """What the push sends to the gateway user list (step 16): level + expiry + licence."""
        lv = self.effective_level(today)
        return {"user": self.user, "level": lv, "licence": licence(lv),
                "expiry": self.expiry.isoformat() if self.expiry else None}


def push_to_gateway(accounts):
    """Step 16 (spec A): send the gateway_record()s to the gateway user list. Not in step 15."""
    raise NotImplementedError("academy step 16")
