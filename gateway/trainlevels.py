"""Training levels from the shore PC's academy (CLAUDE.md section S, ACADEMY_SPEC A, step 16).

The gateway user list (secrets.json, users.py) stays the AUTHORITY: it decides who may log in and
with which role; the academy never creates a user, never changes a role or a password. What the
academy adds is the training level of a user the gateway already knows:

  OBSERVER < DRIVER < CAPTAIN;  CAPTAIN = the "licence" flag (HELM as captain, REMOTE, MANEUVER,
  power raise, examiner) - only CAPTAIN, never DRIVER / OBSERVER.

How a push reaches the gateway: the academy (shore PC, no boat link) writes a SIGNED file; the
ground station sends it over its normal gateway login ({"t": "academy_levels", "doc": {...}}).
The gateway checks it here:
  * signed with the academy key (HMAC-SHA256; secrets.json "academy_key", the same text as the
    shore PC's academy key file) - no key configured = every push refused;
  * format 1, kind "academy_levels", seq newer than the last accepted one (no replay of an old
    file that still granted a licence);
  * every record: user, level, licence == (level is CAPTAIN), expiry YYYY-MM-DD; a licence
    without an expiry is refused; users the gateway doesn't know are ignored (counted).
The accepted file is kept on the SSD (state/academy_levels.json) = the gateway's LAST COPY: used
while the academy can't be reached, read again at every gateway start.

Effect (users.Users.licence): a user WITH an academy record has the licence only while the record
says CAPTAIN and its expiry (inclusive) is not past on the gateway's clock; a user WITHOUT a record
keeps the secrets.json flag (unchanged behaviour). The flag is read only at LOGIN (stations.py /
web.py), so a push never changes a station that is already logged in: never revoked mid-trip; it
takes effect at the next login (CLAUDE.md S: "never revoked mid-trip").

Pure Python, no network (the stations module hands the message in).
"""
import datetime
import hashlib
import hmac
import json
import os

LEVELS = ("OBSERVER", "DRIVER", "CAPTAIN")
FORMAT = 1
KIND = "academy_levels"
MAX_RECORDS = 500
MIN_KEY = 16


def canonical(doc):
    """The signed bytes: the doc without "mac", sorted keys, no spaces (the academy uses the same)."""
    body = {k: v for k, v in doc.items() if k != "mac"}
    return json.dumps(body, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def sign(doc, key):
    return hmac.new(key.encode("utf-8"), canonical(doc), hashlib.sha256).hexdigest()


def _date(s):
    try:
        return datetime.date.fromisoformat(s) if isinstance(s, str) and len(s) == 10 else None
    except ValueError:
        return None


def verify(doc, key, last_seq=0):
    """-> (records dict user -> record, None) or (None, why). No side effects."""
    if not isinstance(key, str) or len(key) < MIN_KEY:
        return None, "no academy key on this gateway"
    if not isinstance(doc, dict):
        return None, "not an academy levels file"
    mac = doc.get("mac")
    if not isinstance(mac, str) or not hmac.compare_digest(mac, sign(doc, key)):
        return None, "wrong signature (not made with this gateway's academy key)"
    if doc.get("format") != FORMAT or doc.get("kind") != KIND:
        return None, "unknown format"
    seq = doc.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq <= int(last_seq or 0):
        return None, "older than the levels the gateway already has"
    recs = doc.get("records")
    if not isinstance(recs, list) or len(recs) > MAX_RECORDS:
        return None, "bad record list"
    out = {}
    for r in recs:
        if not isinstance(r, dict):
            return None, "bad record"
        u, lv, lic, exp = r.get("user"), r.get("level"), r.get("licence"), r.get("expiry")
        if not isinstance(u, str) or not u or len(u) > 64 or u in out:
            return None, "bad or repeated user name"
        if lv not in LEVELS or lic is not (lv == "CAPTAIN"):
            return None, "%s: the licence must match the level (CAPTAIN only)" % u
        if exp is not None and _date(exp) is None:
            return None, "%s: bad expiry" % u
        if lic and exp is None:
            return None, "%s: a licence needs an expiry" % u
        out[u] = {"level": lv, "licence": lic, "expiry": exp}
    return out, None


class AcademyLevels:
    """The gateway's last accepted copy of the academy levels."""

    def __init__(self, path, key=None, today=None):
        self.path = path
        self.key = key
        self.today = today or datetime.date.today      # the gateway's clock (set from GPS)
        self.records, self.seq, self.made, self.by = {}, 0, None, None
        self.load()

    def load(self):
        """The copy on the SSD; a damaged file = no records (every user keeps the secrets.json flag)."""
        if not self.path:
            return
        try:
            with open(self.path, encoding="utf-8") as f:
                d = json.load(f)
            recs = d.get("records") or {}
            ok = isinstance(recs, dict) and all(
                isinstance(r, dict) and r.get("level") in LEVELS and r.get("licence") is (r.get("level") == "CAPTAIN")
                for r in recs.values())
            if ok:
                self.records, self.seq = recs, int(d.get("seq") or 0)
                self.made, self.by = d.get("made"), d.get("by")
        except (OSError, ValueError, TypeError, AttributeError):
            pass

    def apply(self, doc, known_users, by=None):
        """A push from a station. -> (ok, text, n applied, n ignored). Kept on the SSD at once."""
        recs, why = verify(doc, self.key, self.seq)
        if recs is None:
            return False, "academy levels refused: " + why, 0, 0
        kept = {u: r for u, r in recs.items() if u in known_users}
        ignored = len(recs) - len(kept)
        self.records, self.seq = kept, doc["seq"]
        self.made, self.by = str(doc.get("made") or "")[:32], by
        if not self.path:
            return True, "academy levels taken (%d users) but not saved: no SSD" % len(kept), len(kept), ignored
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"seq": self.seq, "made": self.made, "by": by, "records": kept}, f, indent=1)
            os.replace(tmp, self.path)
        except OSError as e:
            return True, "academy levels taken (%d users) but not saved: %s" % (len(kept), e), len(kept), ignored
        return True, "academy levels taken: %d users (%d unknown ignored); next login" % (len(kept), ignored), \
            len(kept), ignored

    def record(self, user):
        return self.records.get(user)

    def licence(self, user, base):
        """The licence at login: the academy record if there is one (CAPTAIN + not expired), else base."""
        r = self.records.get(user)
        if r is None:
            return bool(base)
        if not r.get("licence"):
            return False
        exp = _date(r.get("expiry"))
        return exp is not None and self.today() <= exp

    def status(self):
        return {"seq": self.seq, "made": self.made, "n": len(self.records), "key": bool(self.key)}
