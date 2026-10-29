"""Exams per level (ACADEMY_SPEC G + J, step 21): the model - no window, no network, no process.

An exam = content/exams/*.json (kind "exam"): level (DRIVER / CAPTAIN), pass_pct, the pools its questions
come from, questions (how many), required (lessons; else every required lesson of the level), attempts +
cooldown_h (else limits.json: owner W27 = 3 attempts, then 24 h). Rules:
  * who may sit it: a DRIVER exam = an OBSERVER or DRIVER (a renewal); a CAPTAIN exam = a valid DRIVER or
    CAPTAIN - the account's EFFECTIVE level (expired = OBSERVER);
  * every required lesson passed first (lesson.missing_required);
  * attempts: an exam started = an attempt (logbook kind "attempt" - an exam left half-way counts as failed);
    after `attempts` failed attempts in a row the next one waits cooldown_h after the last failure, then the
    count starts again; a pass resets it;
  * online (online.is_online) only while an instructor marked the session SUPERVISED (Store.supervise);
  * passing an exam NEVER changes a level: the level comes only from the practical test (spec G / J).
Questions = quiz.Run over the pools (random order, shuffled choices; a sound question without its tone file is
left out like in the quizzes). Results -> logbook kind "exam" (id, level, score, passed, online, supervised).
"""
import datetime
import time

import accounts
import content
import lesson
import online
import quiz

KIND = "exam"
LEVELS = content.LEVELS
WHO_MAY = {"DRIVER": ("OBSERVER", "DRIVER"), "CAPTAIN": ("DRIVER", "CAPTAIN")}


def exams(content_dir=None):
    out = [d for d in (content.load(f) for f in content.files(content_dir)) if d.get("kind") == KIND]
    return sorted(out, key=lambda d: (LEVELS.index(d["level"]) if d.get("level") in LEVELS else 9, d.get("id")))


def by_id(exam_id, content_dir=None):
    for d in exams(content_dir):
        if d.get("id") == exam_id:
            return d
    raise KeyError(exam_id)


def check_doc(doc, name="?", pool_ids=None, lesson_ids=None):
    """Exam-specific checks (content.check does the common ones). [] = OK."""
    p = []
    if doc.get("kind") != KIND:
        return p
    if not doc.get("id"):
        p.append("%s: no id" % name)
    if doc.get("level") not in WHO_MAY:
        p.append("%s: level must be DRIVER or CAPTAIN" % name)
    pp = doc.get("pass_pct")
    if not isinstance(pp, (int, float)) or not 50 <= pp <= 100:
        p.append("%s: pass_pct 50..100" % name)
    n = doc.get("questions")
    if not isinstance(n, int) or n < 1:
        p.append("%s: questions >= 1" % name)
    pools = doc.get("pools")
    if not isinstance(pools, list) or not pools:
        p.append("%s: no pools" % name)
    elif pool_ids is not None:
        for x in pools:
            if x not in pool_ids:
                p.append("%s: unknown pool %r" % (name, x))
    if lesson_ids is not None:
        for x in doc.get("required") or []:
            if x not in lesson_ids:
                p.append("%s: unknown required lesson %r" % (name, x))
    for k in ("attempts", "cooldown_h"):
        if k in doc and (not isinstance(doc[k], int) or doc[k] < 1):
            p.append("%s: %s >= 1" % (name, k))
    return p


def rules(doc):
    lim = accounts.limits()
    return int(doc.get("attempts", lim.get("exam_attempts", 3))), float(doc.get("cooldown_h", lim.get("exam_cooldown_h", 24)))


def _ts(e):
    try:
        return time.mktime(time.strptime(str(e.get("t", "")), "%Y-%m-%dT%H:%M:%S"))
    except (ValueError, OverflowError):
        return 0.0


def _scan(entries, user, doc):
    """(failed attempts in a row now, cool-down end epoch s or 0), in logbook order: a pass resets the count;
    after `attempts` failures the cool-down runs cooldown_h from the last one; an attempt after it starts a new
    count."""
    n_max, cool_h = rules(doc)
    fails, until = 0, 0.0
    for e in entries:
        if e.get("user") != user or e.get("id") != doc.get("id"):
            continue
        if e.get("kind") == "exam" and e.get("passed"):
            fails, until = 0, 0.0
        elif e.get("kind") == "attempt":
            if until and _ts(e) >= until:
                fails, until = 0, 0.0
            fails += 1
            if fails >= n_max and not until:
                until = _ts(e) + cool_h * 3600.0
    return fails, until


def cooldown_until(entries, user, doc):
    """When the next attempt is allowed (epoch s), 0 = now."""
    return _scan(entries, user, doc)[1]


def attempts_left(entries, user, doc, now=None):
    """Attempts still allowed before the cool-down (0 = in the cool-down now)."""
    now = time.time() if now is None else now
    n_max, _ = rules(doc)
    fails, until = _scan(entries, user, doc)
    if until:
        return 0 if now < until else n_max
    return max(0, n_max - fails)


def refused(store, user, doc, now=None, env=None, today=None):
    """Why this user may NOT start this exam now (None = may). English (logs); the window translates."""
    now = time.time() if now is None else now
    if user not in store.accounts:
        return "unknown account"
    a = store.accounts[user]
    eff = a.effective_level(today or datetime.date.today())
    if eff not in WHO_MAY.get(doc.get("level"), ()):
        return "the %s exam is for %s (your level: %s)" % (doc.get("level"), " / ".join(WHO_MAY.get(doc.get("level"), ())), eff)
    miss = lesson.missing_required(store.entries(), user, doc)
    if miss:
        return "lessons first: " + ", ".join(miss)
    until = cooldown_until(store.entries(), user, doc)
    if until and now < until:
        return "no attempts left: cool-down until %s" % time.strftime("%Y-%m-%d %H:%M", time.localtime(until))
    if online.is_online(env) and not store.supervised(user, now):
        return "online exams only in a session an instructor marked supervised"
    return None


def passed(entries, user, level, since=None):
    """The newest passed exam of this level (logbook entry) or None. since = a datetime.date."""
    best = None
    for e in entries:
        if e.get("user") == user and e.get("kind") == "exam" and e.get("passed") and e.get("level") == level:
            if since is not None:
                d = accounts._date(str(e.get("t", ""))[:10])
                if not d or d < since:
                    continue
            best = e
    return best


class Run(object):
    """One sitting. start() logs the attempt (so an exam left half-way counts); the questions = a quiz.Run;
    finish() logs the result. Never changes a level."""

    def __init__(self, store, user, doc, lang="el", seed=None, sound_ok=None, env=None, now=None):
        why = refused(store, user, doc, now, env)
        if why:
            raise accounts.simguard.Refused(why)
        self.store, self.user, self.doc, self.env = store, user, doc, env
        pools = {d["id"]: d for d in quiz.pools()}
        docs = [pools[p] for p in doc.get("pools", []) if p in pools]
        self.quiz = quiz.Run(docs, lang, n=int(doc.get("questions", 10)), seed=seed, sound_ok=sound_ok)
        self.online = online.is_online(env)
        self.supervised = store.supervised(user, now) if self.online else False
        store.log(user, "attempt", id=doc["id"], level=doc.get("level"), online=self.online)
        self.result = None

    def finish(self):
        if self.result is not None:
            return self.result
        score = self.quiz.score()
        ok = score >= float(self.doc.get("pass_pct", 80))
        self.result = self.store.log(self.user, "exam", id=self.doc["id"], topic=self.doc["id"],
                                     level=self.doc.get("level"), score=score, passed=ok, online=self.online,
                                     supervised=self.supervised,
                                     right=sum(1 for it in self.quiz.items if it["result"] and it["result"]["correct"]),
                                     n=len(self.quiz.items))
        return self.result
