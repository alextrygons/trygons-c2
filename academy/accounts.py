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
  * step 21 (spec G + J): logins (auth.py: password + TOTP 2FA for CAPTAIN / instructor, W29); ROLE UPGRADES
    ONLY VIA THE PRACTICAL TEST (grant_practical: the examiner's 2FA session + the onboard CONFIRM_ONLY answer);
    set_level may only lower a level (or set the first levels while no CAPTAIN instructor exists = bootstrap);
    nothing online upgrades; renewal (renew: DRIVER = refresher lesson + quiz + scenario, CAPTAIN = the same +
    a check ride every 24 months, expired > 3 months = the full practical test); currency (no logged driving
    in currency_days = no licence in the gateway file until a supervised refresher trip); supervised online
    exam sessions (supervise).
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

import auth
import online
import simguard

LEVELS = ("OBSERVER", "DRIVER", "CAPTAIN")
HERE = os.path.dirname(os.path.abspath(__file__))
PUSH_FORMAT = 1
PUSH_KIND = "academy_levels"
LOG_KINDS = ("lesson", "quiz", "scenario", "videolesson", "exam", "attempt", "hours", "level", "note",
             "login", "supervise", "renewal", "practical", "check_ride", "refresher", "area", "certificate")
# entries that count as logged driving for the currency rule (spec J): driving hours, a practical test /
# check ride / supervised refresher trip on the real boat, a level granted by the practical test
DRIVING_KINDS = ("hours", "practical", "check_ride", "refresher")
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

    def gateway_record(self, today=None, current=True):
        """What the push sends to the gateway user list: level + expiry + licence. current = False (no logged
        driving in currency_days, spec J) keeps the level but NOT the licence: no solo captain rights until a
        supervised refresher trip."""
        lv = self.effective_level(today)
        return {"user": self.user, "level": lv, "licence": licence(lv) and bool(current),
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
        self.auth = auth.Auth(self.folder)
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

    def set_level(self, user, level, expiry=None, by=None, today=None, env=None):
        """Sets a level; an expiry defaults to today + validity_months (limits.json).
        Step 21 (spec G / J): a level may only be LOWERED here (or the expiry shortened). Raising it (a higher
        level, or a later expiry = a renewal) is refused - role upgrades only via the practical test
        (grant_practical), renewals only via renew() - EXCEPT while the academy has no CAPTAIN instructor yet
        (the first set-up: bootstrap()); and never in an online session ("nothing online upgrades")."""
        if level not in LEVELS:
            raise ValueError("unknown level %r" % level)
        a = self.accounts[user]
        today = today or datetime.date.today()
        if level != "OBSERVER" and expiry is None:
            expiry = today + datetime.timedelta(days=int(30.44 * int(limits().get("validity_months", 12))))
        new_exp = _date(expiry) if level != "OBSERVER" else None
        cur = a.effective_level(today)
        raising = rank(level) > rank(cur) or (
            level != "OBSERVER" and level == cur and (a.expiry is None or (new_exp and new_exp > a.expiry)))
        if raising:
            if online.is_online(env):
                raise simguard.Refused("nothing online upgrades a level (online session)")
            if not self.bootstrap(today):
                raise simguard.Refused("role upgrades only via the practical test (renewals: renew)")
        return self._apply_level(a, level, new_exp, by, why="bootstrap" if raising else "set")

    def _apply_level(self, a, level, expiry, by, why, **extra):
        a.level, a.expiry = level, expiry if level != "OBSERVER" else None
        self.save(a)
        self.log(a.user, "level", level=level, expiry=a.expiry.isoformat() if a.expiry else None, by=by, why=why,
                 **extra)
        return a

    def bootstrap(self, today=None):
        """The first set-up: no instructor account holds a valid CAPTAIN level yet."""
        return not any(a.instructor and a.effective_level(today) == "CAPTAIN" for a in self.accounts.values())

    # -- logins (auth.py, spec J: 2FA for CAPTAIN + instructor)
    def needs_2fa(self, user):
        a = self.accounts.get(user)
        return bool(a and (a.instructor or a.level == "CAPTAIN"))

    def login(self, user, password, code=None, t=None):
        """-> auth.Session (2FA checked for CAPTAIN / instructor accounts), or auth.AuthError."""
        if user not in self.accounts:
            raise auth.AuthError("unknown account")
        s = self.auth.login(user, password, code, self.needs_2fa(user), t)
        self.log(user, "login", twofa=s.twofa)
        return s

    def session_ok(self, session, role="instructor", today=None, now=None):
        """Why a session may NOT do an instructor / examiner action (None = it may)."""
        if not isinstance(session, auth.Session) or not session.valid(self.auth.id, now):
            return "log in first (password + 2FA)"
        a = self.accounts.get(session.user)
        if a is None:
            return "unknown account"
        if role == "instructor" and not a.instructor:
            return "%s is not an instructor" % a.user
        if role in ("instructor", "examiner") and a.effective_level(today) != "CAPTAIN":
            return "%s is not a valid CAPTAIN" % a.user
        if not session.twofa:
            return "2FA needed"
        return None

    # -- supervised online sessions (spec J: exams online only when an instructor marks the session supervised)
    def supervise(self, user, session, minutes=120, now=None):
        why = self.session_ok(session)
        if why:
            raise simguard.Refused(why)
        if user not in self.accounts:
            raise KeyError(user)
        now = time.time() if now is None else now
        return self.log(user, "supervise", by=session.user, until=now + 60.0 * max(1, min(int(minutes), 480)))

    def supervised(self, user, now=None):
        now = time.time() if now is None else now
        return any(e.get("kind") == "supervise" and float(e.get("until", 0) or 0) >= now
                   for e in self.accounts.get(user, Account(user)).logbook)

    # -- currency + renewal (spec J)
    def last_drive(self, user):
        """The date of the last logged driving (DRIVING_KINDS), else of the last level change (a fresh level counts
        as current), else None."""
        a = self.accounts.get(user)
        if a is None:
            return None
        ds = [_date(str(e.get("t", ""))[:10]) for e in a.logbook
              if e.get("kind") in DRIVING_KINDS or (e.get("kind") == "level" and e.get("level") != "OBSERVER")]
        ds = [d for d in ds if d]
        return max(ds) if ds else None

    def current(self, user, today=None):
        today = today or datetime.date.today()
        last = self.last_drive(user)
        return bool(last and (today - last).days <= int(limits().get("currency_days", 90)))

    def renewal_status(self, user, today=None):
        """What a renewal of the current level still needs: {"level", "due", "needs": [...], "full_test": bool}.
        DRIVER = a refresher lesson + a quiz (>= 70 %) + one emergency scenario, all within renew_window_days;
        CAPTAIN = the same + a check ride within captain_check_ride_months; expired more than
        expired_full_test_months = the full practical test (no renewal)."""
        today = today or datetime.date.today()
        lim = limits()
        a = self.accounts[user]
        out = {"level": a.level, "needs": [], "full_test": False, "due": None}
        if a.level == "OBSERVER" or a.expiry is None:
            out["needs"].append("no level to renew")
            return out
        out["due"] = a.expiry.isoformat()
        if (today - a.expiry).days > int(30.44 * int(lim.get("expired_full_test_months", 3))):
            out["full_test"] = True
            out["needs"].append("expired more than %s months: the full practical test" %
                                lim.get("expired_full_test_months", 3))
            return out
        since = today - datetime.timedelta(days=int(lim.get("renew_window_days", 90)))

        def recent(kind, ok=lambda e: True, start=since):
            for e in a.logbook:
                d = _date(str(e.get("t", ""))[:10])
                if e.get("kind") == kind and d and d >= start and ok(e):
                    return True
            return False
        if not recent("lesson", lambda e: e.get("done")):
            out["needs"].append("a refresher lesson")
        if not recent("quiz", lambda e: float(e.get("score", 0) or 0) >= 70):
            out["needs"].append("a quiz (70 %)")
        if not recent("scenario", lambda e: e.get("done")):
            out["needs"].append("one emergency scenario")
        if a.level == "CAPTAIN":
            ride_from = today - datetime.timedelta(days=int(30.44 * int(lim.get("captain_check_ride_months", 24))))
            if not recent("check_ride", lambda e: e.get("passed"), ride_from) and \
                    not recent("practical", lambda e: e.get("passed"), ride_from):
                out["needs"].append("a practical check ride (every %s months)" % lim.get("captain_check_ride_months", 24))
        return out

    def renew(self, user, session, today=None):
        """Extends the expiry of the SAME level by validity_months (from the old expiry while it is still valid),
        only when renewal_status needs nothing; the instructor's 2FA session."""
        why = self.session_ok(session, today=today)
        if why:
            raise simguard.Refused(why)
        today = today or datetime.date.today()
        st = self.renewal_status(user, today)
        if st["needs"]:
            raise simguard.Refused("renewal needs: " + "; ".join(st["needs"]))
        a = self.accounts[user]
        base = a.expiry if a.expiry and a.expiry >= today else today
        exp = base + datetime.timedelta(days=int(30.44 * int(limits().get("validity_months", 12))))
        self.log(user, "renewal", level=a.level, expiry=exp.isoformat(), by=session.user)
        return self._apply_level(a, a.level, exp, session.user, "renewal")

    # -- the practical test's result (academy/practical/, spec G): the ONLY way up
    def grant_practical(self, result, session, today=None, env=None):
        """Applies a PASSED practical test / check ride / refresher trip (practical.Test.result()):
          upgrade    -> the trainee's level = the target level, expiry = today + validity_months;
          check_ride -> logged (CAPTAIN renewal needs it every 24 months);
          refresher  -> logged (currency back: logged driving).
        Refused unless: the examiner's own 2FA session (a valid CAPTAIN, not the trainee), every task approved,
        the onboard CONFIRM_ONLY answered with the test's token, not online, the trainee is eligible."""
        today = today or datetime.date.today()
        if online.is_online(env):
            raise simguard.Refused("nothing online upgrades a level (online session)")
        why = self.session_ok(session, role="examiner", today=today)
        if why:
            raise simguard.Refused(why)
        r = result if isinstance(result, dict) else {}
        user, kind, target = r.get("trainee"), r.get("kind"), r.get("target")
        if user not in self.accounts:
            raise simguard.Refused("unknown trainee")
        if r.get("examiner") != session.user:
            raise simguard.Refused("the result is not this examiner's")
        if user == session.user:
            raise simguard.Refused("an examiner never examines himself")
        if not r.get("passed") or not r.get("tasks") or not all(t.get("state") == "approved" for t in r["tasks"]):
            raise simguard.Refused("not every task approved by the examiner")
        co = r.get("confirm_only") or {}
        if co.get("state") != "confirmed" or not co.get("token") or co.get("token") != r.get("token"):
            raise simguard.Refused("no onboard CONFIRM for this test (CONFIRM_ONLY)")
        a = self.accounts[user]
        cur = a.effective_level(today)
        entry = {k: r.get(k) for k in ("id", "target", "score", "recording", "area", "wind_kn", "wind_from", "token",
                                       "started", "ended")}
        entry["test_kind"] = kind
        if kind == "upgrade":
            if target not in ("DRIVER", "CAPTAIN") or rank(target) != rank(cur) + 1:
                raise simguard.Refused("an upgrade is one level up from %s" % cur)
            self.log(user, "practical", passed=True, by=session.user, **entry)
            exp = today + datetime.timedelta(days=int(30.44 * int(limits().get("validity_months", 12))))
            return self._apply_level(a, target, exp, session.user, "practical test", test=r.get("id"))
        if kind == "check_ride":
            if cur != "CAPTAIN":
                raise simguard.Refused("a check ride is for a valid CAPTAIN")
            self.log(user, "check_ride", passed=True, by=session.user, **entry)
            return a
        if kind == "refresher":
            if rank(cur) < rank("DRIVER"):
                raise simguard.Refused("a refresher trip is for a DRIVER / CAPTAIN")
            self.log(user, "refresher", passed=True, by=session.user, **entry)
            return a
        raise simguard.Refused("unknown test kind %r" % kind)

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
                if e.get("kind") in ("quiz", "exam", "scenario", "videolesson") and isinstance(e.get("score"), (int, float)):
                    scores.setdefault(e.get("topic") or e.get("id") or "?", []).append(float(e["score"]))
            weak = sorted(t for t, s in scores.items() if sum(s) / len(s) < 70.0)
            hours = sum(float(e.get("h", 0) or 0) for e in a.logbook if e.get("kind") == "hours")
            last = self.last_drive(u)
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
        doc = make_push_doc([self.accounts[u] for u in sorted(self.accounts)], key, today,
                            current={u: self.current(u, today) for u in self.accounts})
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


def make_push_doc(accs, key, today=None, seq=None, current=None):
    doc = {"format": PUSH_FORMAT, "kind": PUSH_KIND, "seq": int(seq if seq is not None else time.time() * 1000),
           "made": time.strftime("%Y-%m-%dT%H:%M:%S"),
           "records": [a.gateway_record(today, (current or {}).get(a.user, True)) for a in accs]}
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
