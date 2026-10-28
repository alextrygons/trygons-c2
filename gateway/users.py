"""Users and roles (section B1).

secrets.json (never in git, chmod 600):
    "users": {
      "alex":   {"role": "HELM",   "pw": "pbkdf2_sha256$200000$<salt>$<hash>", "licence": true},
      "phone":  {"role": "VIEWER", "keys": ["<sha256 of a long random key>"]}
    }

VIEWER: watch only. HELM: may hold HELM (only one station at a time does,
enforced on the AURIX). "licence": true = the user holds a manual (boat
driving) licence: only such a user may ask for REMOTE MANUAL or MANEUVER
(and be the captain, section D). Missing = no licence. Log in with a password, or with a key (a long
random token, e.g. for a PC that logs in by itself).

Add a user (prints the line to paste into secrets.json):
    python3 users.py add alex HELM [licence]   (licence: holds a manual licence)
    python3 users.py key phone VIEWER          (makes a random key)
"""
import getpass
import hashlib
import hmac
import json
import os
import secrets
import sys
import time

ROLES = ("VIEWER", "HELM")
ITER = 200000


def hash_pw(pw, salt=None, it=ITER):
    salt = salt or secrets.token_hex(16)
    h = hashlib.pbkdf2_hmac("sha256", pw.encode(), bytes.fromhex(salt), it).hex()
    return "pbkdf2_sha256$%d$%s$%s" % (it, salt, h)


def check_pw(pw, stored):
    try:
        algo, it, salt, h = stored.split("$")
        if algo != "pbkdf2_sha256":
            return False
        return hmac.compare_digest(hash_pw(pw, salt, int(it)).split("$")[3], h)
    except (ValueError, AttributeError):
        return False


def hash_key(key):
    return hashlib.sha256(key.encode()).hexdigest()


class Users:
    def __init__(self, users, max_fail=5, lock_s=60.0):
        self.users = users or {}
        self.max_fail = max_fail
        self.lock_s = lock_s
        self.fails = {}             # ip -> [count, until]
        self.academy = None         # trainlevels.AcademyLevels (step 16) or None

    def locked(self, ip):
        f = self.fails.get(ip)
        return bool(f and f[0] >= self.max_fail and time.time() < f[1])

    def login(self, ip, user, password=None, key=None):
        """-> (role, None) or (None, reason). Too many failures from one
        address lock it out for lock_s."""
        if self.locked(ip):
            return None, "too many failed logins - wait a minute"
        u = self.users.get(user)
        ok = False
        if u and password is not None and u.get("pw"):
            ok = check_pw(password, u["pw"])
        elif u and key is not None:
            ok = any(hmac.compare_digest(hash_key(key), k) for k in u.get("keys", []))
        if not ok or u.get("role") not in ROLES:
            f = self.fails.setdefault(ip, [0, 0.0])
            f[0] += 1
            f[1] = time.time() + self.lock_s
            return None, "wrong user name or password"
        self.fails.pop(ip, None)
        return u["role"], None

    def licence(self, user):
        """True if the user holds a manual (driving) licence - only flagged users. With academy levels
        (trainlevels.py) a user that has an academy record gets the licence only as CAPTAIN within
        its expiry; a user without one keeps the flag. Only users of this list (never a new one).
        Read at LOGIN only: a push never changes a logged-in station (never revoked mid-trip)."""
        u = self.users.get(user)
        base = bool(u and u.get("licence") is True)
        if not u or self.academy is None:
            return base
        return self.academy.licence(user, base)


def main():
    if len(sys.argv) < 4 or sys.argv[1] not in ("add", "key") or sys.argv[3] not in ROLES:
        print(__doc__)
        return 1
    name, role = sys.argv[2], sys.argv[3]
    lic = len(sys.argv) > 4 and sys.argv[4] == "licence"
    if sys.argv[1] == "add":
        pw = getpass.getpass("password for %s: " % name)
        if len(pw) < 10:
            print("use at least 10 characters")
            return 1
        if pw != getpass.getpass("again: "):
            print("not the same")
            return 1
        entry = {"role": role, "pw": hash_pw(pw)}
    else:
        key = secrets.token_urlsafe(32)
        print("the key (give it to the device, it is shown only now):\n  %s" % key)
        entry = {"role": role, "keys": [hash_key(key)]}
    if lic:
        entry["licence"] = True
    print('paste into "users" in secrets.json:\n  %s: %s' % (json.dumps(name), json.dumps(entry)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
