"""Academy logins: password + 2FA (ACADEMY_SPEC J, step 21; owner W29: an offline TOTP app, 6-digit codes, no SMS).

  * every account may get a password (PBKDF2-SHA256, 200 000 rounds, own salt);
  * CAPTAIN and instructor accounts ALSO need a TOTP code (RFC 6238: HMAC-SHA1, 30 s, 6 digits - the
    standard of every offline authenticator app); a CAPTAIN / instructor without an enrolled TOTP can't log
    in (enrol first, with the password);
  * the secrets live in <data>/auth/<user>.json - NOT in the account file, never in a backup (a restored
    account enrols again), never in git (academy/data/ is gitignored);
  * 5 wrong logins in a row = locked for 10 minutes (the lock is in the same file).
A Session (login()) is what the instructor actions and the practical test's examiner need; it is valid only
in this process, for SESSION_S. No network module here (test_academy AC1).
"""
import base64
import hashlib
import hmac
import json
import os
import secrets
import struct
import time

ROUNDS = 200000
TOTP_STEP = 30
TOTP_DIGITS = 6
TOTP_WINDOW = 1                 # one 30 s step before / after (clock drift of the phone)
LOCK_AFTER = 5
LOCK_S = 600
SESSION_S = 4 * 3600
ISSUER = "Trygons Academy"


def hash_password(pw, salt=None, rounds=ROUNDS):
    salt = salt or secrets.token_bytes(16)
    dk = hashlib.pbkdf2_hmac("sha256", pw.encode("utf-8"), salt, rounds)
    return {"salt": salt.hex(), "rounds": rounds, "hash": dk.hex()}


def check_password(pw, rec):
    try:
        dk = hashlib.pbkdf2_hmac("sha256", (pw or "").encode("utf-8"), bytes.fromhex(rec["salt"]), int(rec["rounds"]))
        return hmac.compare_digest(dk.hex(), rec["hash"])
    except (KeyError, TypeError, ValueError):
        return False


def new_totp_secret():
    return base64.b32encode(secrets.token_bytes(20)).decode("ascii").rstrip("=")


def totp(secret, t=None, step=TOTP_STEP, digits=TOTP_DIGITS):
    """RFC 6238 TOTP (SHA-1) of a base32 secret at time t."""
    key = base64.b32decode(secret + "=" * (-len(secret) % 8), casefold=True)
    counter = int((time.time() if t is None else t) // step)
    mac = hmac.new(key, struct.pack(">Q", counter), hashlib.sha1).digest()
    off = mac[-1] & 0x0F
    code = (struct.unpack(">I", mac[off:off + 4])[0] & 0x7FFFFFFF) % (10 ** digits)
    return str(code).zfill(digits)


def totp_ok(secret, code, t=None, window=TOTP_WINDOW):
    if not secret or not isinstance(code, str) or not code.strip().isdigit():
        return False
    t = time.time() if t is None else t
    code = code.strip()
    return any(hmac.compare_digest(totp(secret, t + k * TOTP_STEP), code) for k in range(-window, window + 1))


def otpauth_uri(user, secret):
    """For the authenticator app (typed in by hand or as a QR code made elsewhere)."""
    return "otpauth://totp/%s:%s?secret=%s&issuer=%s&digits=%d&period=%d" % (
        ISSUER.replace(" ", "%20"), user, secret, ISSUER.replace(" ", "%20"), TOTP_DIGITS, TOTP_STEP)


class AuthError(Exception):
    pass


class Session(object):
    """A successful login (this process only). twofa = a TOTP code was checked."""

    def __init__(self, user, twofa, store_id, t=None):
        self.user, self.twofa, self.store_id = user, bool(twofa), store_id
        self.t = time.time() if t is None else t

    def valid(self, store_id, now=None):
        now = time.time() if now is None else now
        return self.store_id == store_id and 0 <= now - self.t <= SESSION_S


class Auth(object):
    """The secrets of one academy data folder (<folder>/auth/)."""

    def __init__(self, folder):
        self.dir = os.path.join(folder, "auth")
        self.id = secrets.token_hex(8)          # sessions are valid only for this object (this process)

    def _path(self, user):
        return os.path.join(self.dir, user + ".json")

    def get(self, user):
        try:
            with open(self._path(user), encoding="utf-8") as f:
                d = json.load(f)
            return d if isinstance(d, dict) else {}
        except (OSError, ValueError):
            return {}

    def _put(self, user, rec):
        os.makedirs(self.dir, exist_ok=True)
        tmp = self._path(user) + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(rec, f, indent=1)
        os.replace(tmp, self._path(user))

    def has_password(self, user):
        return "password" in self.get(user)

    def has_totp(self, user):
        return bool(self.get(user).get("totp"))

    def set_password(self, user, new, old=None):
        """The first password freely; a change needs the old one."""
        if not isinstance(new, str) or len(new) < 8:
            raise AuthError("password: at least 8 characters")
        rec = self.get(user)
        if "password" in rec and not check_password(old, rec["password"]):
            raise AuthError("the old password is wrong")
        rec["password"] = hash_password(new)
        self._put(user, rec)

    def enrol_totp(self, user, password, code=None, secret=None, t=None):
        """-> the new secret (for the app). Needs the password; replacing an existing one also needs its code."""
        rec = self.get(user)
        if not check_password(password, rec.get("password")):
            raise AuthError("wrong password")
        if rec.get("totp") and not totp_ok(rec["totp"], code, t):
            raise AuthError("the current 2FA code is needed to replace it")
        rec["totp"] = secret or new_totp_secret()
        self._put(user, rec)
        return rec["totp"]

    def login(self, user, password, code=None, need_2fa=False, t=None):
        """-> Session, or AuthError (why). need_2fa: CAPTAIN / instructor (the caller decides from the level)."""
        now = time.time() if t is None else t
        rec = self.get(user)
        if rec.get("locked_until", 0) > now:
            raise AuthError("locked for %d s after %d wrong logins" % (int(rec["locked_until"] - now), LOCK_AFTER))
        if "password" not in rec:
            raise AuthError("no password set for %s" % user)
        ok = check_password(password, rec["password"])
        if ok and need_2fa:
            if not rec.get("totp"):
                raise AuthError("2FA not enrolled (CAPTAIN / instructor: enrol the authenticator app first)")
            ok = totp_ok(rec["totp"], code, now)
        if not ok:
            rec["fails"] = int(rec.get("fails", 0)) + 1
            if rec["fails"] >= LOCK_AFTER:
                rec["locked_until"], rec["fails"] = now + LOCK_S, 0
            self._put(user, rec)
            raise AuthError("wrong password or 2FA code")
        if rec.get("fails") or rec.get("locked_until"):
            rec.pop("fails", None)
            rec.pop("locked_until", None)
            self._put(user, rec)
        return Session(user, need_2fa, self.id, now)
