"""Academy accounts, levels, logbook, instructor view, gateway push file, backup (ACADEMY_SPEC A).

Rules (CLAUDE.md section S, owner answers W28 2026-10-28):
  * levels OBSERVER < DRIVER < CAPTAIN; CAPTAIN = the gateway user's "licence" flag
    (gateway/users.py: HELM as captain, REMOTE, MANEUVER, power raise, examiner) - CAPTAIN only;
  * an unknown level, an expired one, or a level above OBSERVER without an expiry = OBSERVER;
  * the gateway user list stays the AUTHORITY: the academy only pushes (user, level, licence,
    expiry) for users the gateway already knows; the gateway keeps its last copy when the academy
    can't be reached; a change never takes effect mid-trip (the gateway reads it at the next login);
  * the academy NEVER talks to a boat: the push is a SIGNED FILE (HMAC-SHA256 with the academy key,
    the same text as the gateway's secrets.json "academy_key"); the ground station hands it to the
    gateway over its own login (menu Academy -> send levels); gateway/trainlevels.py checks it;
  * storage on the shore PC: data_dir() = academy/data/ (gitignored) or TRYGONS_ACADEMY_DATA:
    accounts/<user>.json (one per account, atomic write), logbook.jsonl (append-only: lessons,
    scores, attempts, exams, hours, level changes), push/levels.json (the last push file),
    academy_key.txt (the key, never in git);
  * backup (W28): a copy to a folder the instructor chooses (a second shore PC / a USB disk);
    NEVER a boat gateway (refused on a gateway PC, and to a network path on a boats/ address or
    the Tailscale range), never inside the data folder itself;
  * validity / reminder / currency numbers come from content/limits.json (never hard-coded).
No network module here (test_academy AC1-AC3).
"""
import datetime
import hmac
import hashlib
import ipaddress
import json
import os
import re
import secrets
import shutil
import time

import simguard

LEVELS = ("OBSERVER", "DRIVER", "CAPTAIN")
HERE = os.path.dirname(os.path.abspath(__file__))
PUSH_FORMAT = 1
PUSH_KIND = "academy_levels"
LOG_KINDS = ("lesson", "quiz", "scenario", "exam", "attempt", "hours", "level", "note")
USER_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,64}$")


def rank(level):
    """0 / 1 / 2; an unknown level = OBSERVER (never more rights by a typo)."""
    return LEVELS.index(level) if level in LEVELS else 0


def at_least(level, needed):
    return rank(level) >= rank(needed)


def licence(level):
    """The gateway's "licence" flag for this level (only CAPTAIN)."""
    return level == "CAPTAIN"


def _date(v):
    if isinstance(v, datetime.date):
        return v
    try:
        return datetime.date.fromisoformat(v) if isinstance(v, str) and len(v) == 10 else None
    except ValueError:
        return None


class Account(object):
    """One trainee / instructor (spec A). The logbook lives in the store's logbook.jsonl."""

    def __init__(self, user, level="OBSERVER", expiry=None, instructor=False, name=""):
        self.user = user
        self.level = level if level in LEVELS else "OBSERVER"
        self.expiry = _date(expiry)           # datetime.date or None (never had a level)
        self.instructor = bool(instructor)
        self.name = str(name or "")
        self.logbook = []                     # filled by Store.load (entries of this user)

    def effective_level(self, today=None):
        """The level the rules use: expired (or no expiry for a level above OBSERVER) = OBSERVER."""
        today = today or datetime.date.today()
        if self.level == "OBSERVER":
            return "OBSERVER"
        if self.expiry is None or today > self.expiry:
            return "OBSERVER"
        return self.level

    def gateway_record(self, today=None):
        """What the push sends to the gateway user list: level + expiry + licence."""
        lv = self.effective_level(today)
        return {"user": self.user, "level": lv, "licence": licence(lv),
                "expiry": self.expiry.isoformat() if self.expiry and lv != "OBSERVER" else None}

    def to_dict(self):
        return {"user": self.user, "name": self.name, "level": self.level, "instructor": self.instructor,
                "expiry": self.expiry.isoformat() if self.expiry else None}

    @classmethod
    def from_dict(cls, d):
        return cls(d.get("user"), d.get("level", "OBSERVER"), d.get("expiry"), d.get("instructor", False),
                   d.get("name", ""))


def data_dir(env=None):
    env = os.environ if env is None else env
    return env.get("TRYGONS_ACADEMY_DATA") or os.path.join(HERE, "data")


def limits():
    try:
        with open(os.path.join(HERE, "content", "limits.json"), encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


class Store(object):
    """The academy data on the shore PC (refuses to work on a gateway PC: simguard)."""

    def __init__(self, folder=None, env=None):
        if simguard.running_on_gateway(env):
            raise simguard.Refused("the academy never runs on a boat's gateway PC")
        self.folder = folder or data_dir(env)
        self.accounts = {}
        self.load()

    # -- paths
    def _acc_path(self, user):
        return os.path.join(self.folder, "accounts", user + ".json")

    @property
    def logbook_path(self):
        return os.path.join(self.folder, "logbook.jsonl")

    @property
    def push_path(self):
        return os.path.join(self.folder, "push", "levels.json")

    @property
    def key_path(self):
        return os.path.join(self.folder, "academy_key.txt")

    # -- accounts
    def load(self):
        self.accounts = {}
        d = os.path.join(self.folder, "accounts")
        if os.path.isdir(d):
            for f in sorted(os.listdir(d)):
                if f.endswith(".json"):
                    try:
                        with open(os.path.join(d, f), encoding="utf-8") as fh:
                            a = Account.from_dict(json.load(fh))
                        if a.user and USER_RE.match(a.user):
                            self.accounts[a.user] = a
                    except (OSError, ValueError, AttributeError):
                        pass
        for e in self.entries():
            a = self.accounts.get(e.get("user"))
            if a is not None:
                a.logbook.append(e)

    def save(self, acc):
        if not acc.user or not USER_RE.match(acc.user):
            raise ValueError("user name: letters, digits, . _ - only")
        _write_json(self._acc_path(acc.user), acc.to_dict())
        self.accounts[acc.user] = acc

    def add(self, user, name="", instructor=False):
        if user in self.accounts:
            raise ValueError("%s exists" % user)
        a = Account(user, "OBSERVER", None, instructor, name)
        self.save(a)
        self.log(user, "level", level="OBSERVER", note="account created")
        return a

    def set_level(self, user, level, expiry=None, by=None, today=None):
        """Sets a level; an expiry defaults to today + validity_months (limits.json)."""
        if level not in LEVELS:
            raise ValueError("unknown level %r" % level)
        a = self.accounts[user]
        if level != "OBSERVER" and expiry is None:
            today = today or datetime.date.today()
            expiry = today + datetime.timedelta(days=int(30.44 * int(limits().get("validity_months", 12))))
        a.level, a.expiry = level, _date(expiry) if level != "OBSERVER" else None
        self.save(a)
        self.log(user, "level", level=level, expiry=a.expiry.isoformat() if a.expiry else None, by=by)
        return a

    # -- logbook (append-only)
    def log(self, user, kind, **fields):
        if kind not in LOG_KINDS:
            raise ValueError("unknown logbook kind %r" % kind)
        e = dict(fields, user=user, kind=kind, t=time.strftime("%Y-%m-%dT%H:%M:%S"))
        os.makedirs(self.folder, exist_ok=True)
        with open(self.logbook_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
        a = self.accounts.get(user)
        if a is not None:
            a.logbook.append(e)
        return e

    def entries(self):
        out = []
        try:
            with open(self.logbook_path, encoding="utf-8") as f:
                for line in f:
                    try:
                        e = json.loads(line)
                        if isinstance(e, dict):
                            out.append(e)
                    except ValueError:
                        pass                            # a cut last line (power loss) is skipped
        except OSError:
            pass
        return out

    # -- instructor view
    def summary(self, today=None):
        """One row per account: trainees, progress, weak areas, expiry, currency (spec A)."""
        today = today or datetime.date.today()
        lim = limits()
        rows = []
        for u in sorted(self.accounts):
            a = self.accounts[u]
            lessons = {e.get("id") for e in a.logbook if e.get("kind") == "lesson" and e.get("done")}
            scores = {}
            for e in a.logbook:
                if e.get("kind") in ("quiz", "exam", "scenario") and isinstance(e.get("score"), (int, float)):
                    scores.setdefault(e.get("topic") or e.get("id") or "?", []).append(float(e["score"]))
            weak = sorted(t for t, s in scores.items() if sum(s) / len(s) < 70.0)
            hours = sum(float(e.get("h", 0) or 0) for e in a.logbook if e.get("kind") == "hours")
            drove = [_date(str(e.get("t", ""))[:10]) for e in a.logbook if e.get("kind") == "hours"]
            drove = [d for d in drove if d]
            last = max(drove) if drove else None
            eff = a.effective_level(today)
            days_left = (a.expiry - today).days if a.expiry and a.level != "OBSERVER" else None
            rows.append({
                "user": u, "name": a.name, "level": a.level, "effective": eff, "instructor": a.instructor,
                "expiry": a.expiry.isoformat() if a.expiry else None, "days_left": days_left,
                "reminder": days_left is not None and 0 <= days_left <= int(lim.get("reminder_days", 30)),
                "lessons": len(lessons), "attempts": sum(1 for e in a.logbook if e.get("kind") == "attempt"),
                "exams_passed": sum(1 for e in a.logbook if e.get("kind") == "exam" and e.get("passed")),
                "hours": round(hours, 1), "last_drive": last.isoformat() if last else None,
                "current": bool(last and (today - last).days <= int(lim.get("currency_days", 90))),
                "weak": weak})
        return rows

    # -- the gateway push (a signed file; the ground station sends it)
    def key(self, create=False):
        try:
            with open(self.key_path, encoding="utf-8") as f:
                k = f.read().strip()
            if len(k) >= 16:
                return k
        except OSError:
            pass
        if not create:
            return None
        k = secrets.token_urlsafe(32)
        os.makedirs(self.folder, exist_ok=True)
        with open(self.key_path, "w", encoding="utf-8") as f:
            f.write(k + "\n")
        return k

    def make_push(self, today=None, key=None):
        """Writes push/levels.json (every account's gateway_record, signed). -> (path, doc)."""
        key = key or self.key()
        if not key:
            raise ValueError("no academy key: make one first (the same text goes into the gateway's secrets.json)")
        doc = make_push_doc([self.accounts[u] for u in sorted(self.accounts)], key, today)
        _write_json(self.push_path, doc)
        return self.push_path, doc

    # -- backup (W28)
    def backup(self, target, env=None, boats_dir=None, stamp=None):
        """Copies accounts + logbook + the last push file to target/trygons_academy_<stamp>/ (never the
        key: it stays on this PC). -> the backup folder."""
        why = backup_refused(target, self.folder, env, boats_dir)
        if why:
            raise simguard.Refused(why)
        dst = os.path.join(target, "trygons_academy_" + (stamp or time.strftime("%Y%m%d_%H%M%S")))
        os.makedirs(dst)
        acc = os.path.join(self.folder, "accounts")
        if os.path.isdir(acc):
            shutil.copytree(acc, os.path.join(dst, "accounts"))
        for p in (self.logbook_path, self.push_path):
            if os.path.isfile(p):
                shutil.copy2(p, os.path.join(dst, os.path.basename(p)))
        return dst


def canonical(doc):
    """The signed bytes (gateway/trainlevels.py uses the same rule)."""
    body = {k: v for k, v in doc.items() if k != "mac"}
    return json.dumps(body, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def make_push_doc(accs, key, today=None, seq=None):
    doc = {"format": PUSH_FORMAT, "kind": PUSH_KIND, "seq": int(seq if seq is not None else time.time() * 1000),
           "made": time.strftime("%Y-%m-%dT%H:%M:%S"),
           "records": [a.gateway_record(today) for a in accs]}
    doc["mac"] = hmac.new(key.encode("utf-8"), canonical(doc), hashlib.sha256).hexdigest()
    return doc


def push_to_gateway(accounts, key, today=None):
    """Spec A "the academy pushes level + expiry": the SIGNED document for the gateway. The academy never
    sends it itself (no boat link): the ground station's menu Academy -> send levels does."""
    return make_push_doc(accounts, key, today)


def backup_refused(target, data_folder, env=None, boats_dir=None):
    """Why a backup target is refused (None = fine). W28: a folder of the instructor's choice, never a
    boat gateway."""
    if simguard.running_on_gateway(env):
        return "the academy never runs on a boat's gateway PC"
    if not target or not os.path.isdir(target):
        return "not a folder"
    t, d = os.path.abspath(target), os.path.abspath(data_folder)
    if os.path.normcase(t) == os.path.normcase(d) or os.path.normcase(t).startswith(os.path.normcase(d) + os.sep):
        return "inside the academy data folder itself"
    m = re.match(r"^(?:\\\\|//)([^\\/]+)", target)
    if m:
        host = m.group(1).strip("[]")
        names, ips = simguard.boat_files(boats_dir)
        if host in ips or host.lower() in names:
            return "a boat's address (never a boat gateway)"
        try:
            if ipaddress.ip_address(host) in simguard.TAILSCALE:
                return "a Tailscale address (the boats are there: never a boat gateway)"
        except ValueError:
            pass
    return None
