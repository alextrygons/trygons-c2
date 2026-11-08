"""The talking manual's FIND part (step AM3): the manuals -> passages, Greek / English normalisation, BM25.

    build(lang)            MANUAL_EL.txt / MANUAL.txt + the tour texts -> {"passages", "sources", ...}
    save(lang) / load(lang) groundstation/assistant/index/manual_<lang>.json (committed; rebuilt by
                           tools/assistant_index.py build; tools/check.sh FAILS on a stale one, AM3-r MED-1)
    stale(lang)            the reasons the committed index no longer matches its sources ([] = current)
    Searcher(idx).search(q, k=3) -> {"hits": [{passage, score, cover}], "cant_find": bool, "why"}

Pure file reading + text work: no network, no process, never writes outside the index folder (and only when
asked to save). A passage = {id, kind manual | tour, ch, chapter, heading, term, text, lines [a, b]}
(term = the definition-list name of a mode / alarm / trouble, else ""; a tour stop has ch "tour", lines [0, 0]).
Can't find = the best passage below the score / coverage thresholds (tuned on the TUNING part of the question
list tests/assistant/questions_*.json, the held-out part tests/assistant/heldout.json only measured - AM3-r;
docs/assistant_design.md section 4). The assistant's own manual chapter is not indexed (AM3-r LOW-3).
"""
import hashlib
import json
import math
import os
import re
import unicodedata

from . import paths

# 2 (AM3-r): the assistant chapters left out, a line range per split part. 3 (AM3-r3, W104): bumped because the
# MEANING of stale() changed in AM3-r2 (it also compares the committed passages with a fresh build) without a bump -
# rule: every change to what stale() judges = INDEX_VERSION + 1, the index rebuilt + committed
INDEX_VERSION = 3
INDEX_DIR = os.path.join(paths.HERE, "index")
MANUALS = {"el": "MANUAL_EL.txt", "en": "MANUAL.txt"}
TOUR = os.path.join("academy", "content", "tour.json")
SELF = os.path.join("groundstation", "assistant", "index.py")   # a parser change = the index is stale too

MIN_WORDS = 40              # a shorter piece is joined with the next one under the same heading
MAX_WORDS = 120             # a longer one is split at sentences
MIN_KEEP = 5                # a passage of fewer words is no answer ("Ε  ΕΚΜΑΘΗΣΗ") and is left out
# MANUAL_EL 14 = the Greek - English term table: a translation list, no answer. MANUAL_EL 17 / MANUAL 22 = the
# assistant's own chapter (AM3-r LOW-3): it describes the assistant, not the C2 - never an answer source
SKIP_CHAPTERS = {"el": {"14", "17"}, "en": {"22"}}
TOUR_FACTOR = 0.9           # a tour stop's score x this (short "where is it" texts; measured, AM3)
HEAD_WEIGHT = 3             # the heading path (chapter, heading, term) counts three times
K1, B = 1.2, 0.5
# can't find: the best hit below EITHER threshold - tuned on the question list's TUNING part only, the HELD-OUT part
# (tests/assistant/heldout.json) measured once after (AM3-r MED-2, TEST_REPORT.md)
MIN_SCORE = 3.0
# AM3-r: re-tuned on the TUNING part only (was 0.35, tuned on the whole list). AM3-r2 MED-2: per language - the
# English 0.30 refused fewer out-of-manual questions (2 / 6) than 0.35 did (3 / 6); 0.33 lies inside the flat
# tuning range 0.31 .. 0.34 (en tuning: 3 / 5 refused right, no false can't find, top-1 / top-3 unchanged; 0.30 =
# 2 / 5; 0.35 = 3 / 5 but 1 FALSE can't find - AM3-r3 LOW-3, measured again)
MIN_COVER = {"el": 0.30, "en": 0.33}

CHAPTER_RE = re.compile(r"^(\d+[A-Z]?)\.\s+(\S.*)$")
RULE_RE = re.compile(r"^\s*([-=])\1{10,}\s*$")
BULLET_RE = re.compile(r"^\s*(\*|-|\d+\.|\(\w\))\s+\S")
WORD_RE = re.compile(r"[0-9a-zα-ω]+")
SENT_RE = re.compile(r"(?<=[.!·])\s+(?=\S)")

GREEK = set("αβγδεζηθικλμνξοπρστυφχψω")
LOOK = dict(zip("αβεηικμνορτυχζ", "abehikmnoptyxz"))   # Greek look-alikes inside a Latin word (ΗDG)

STOP = {
    "el": set("""ο η το οι τα του τησ των τον την τη τουσ τισ ενα ενασ μια μια μιασ ενοσ και κι η ητε να θα
        σε στο στη στην στον στα στισ στουσ στησ στου με για απο που πωσ τι τι ποιο ποια ποιοσ ποιεσ ποιουσ
        ποιου ποιον ειναι ειμαι εισαι δεν δε μη μην αν οτι οταν ποτε αυτο αυτη αυτοσ αυτα αυτον αυτην μου σου
        μασ σασ του τουσ εγω εσυ εμεισ εσεισ μπορω μπορει μπορουμε πρεπει κανω κανει κανουμε γινεται γινει
        εχω εχει εχουμε ειχε θελω πια λοιπον ρε αμα ωστε επειδη ομωσ αλλα παρα ουτε ποσο ποση ποσα ποσεσ
        ποσοσ πουθε εδω εκει τωρα αρα δηλαδη μετα πριν παω πω λεω λεει ενω οσο εαν εστω καπου καπωσ κατι
        κατα προσ σημαινει σημαινουν γινεται γινει γινονται βρισκω βλεπω ξερω λενε μπορουμε μπορεισ
        κανεισ κανετε εχεισ εχετε σωστα λιγο πολυ οκ πεσ πειτε""".split()),
    "en": set("""a an the of to in on at for from by with and or is are was be been do does did i you we it
        my your our this that these those what which who how when where why can could should would will
        shall may might must there here then than so if not no yes me us them they he she his her its
        about into out up down get got have has had am any some just please tell say does mean now""".split()),
}
# Greek endings (after the accents and the final sigma were normalised), longest first
EL_SUFFIX = sorted(set("""ουμε ουνε ουσα οντασ ησεισ ησει ησω ηση ησησ ησαν ματα ματων ματοσ ειτε ετε ουν εισ
    ει ουσ ου ων οσ οι εσ ασ ησ ια ιο ιων ιεσ εων αω αει αμε ατε ανε αμαι εται ονται α ο η ε ι υ ω""".split()),
                   key=len, reverse=True)
EN_SUFFIX = ("ings", "ing", "edly", "ed", "ies", "es", "s", "ly")
STEM_LEN = {"el": 6, "en": 7}  # stems are cut to this length (Greek verb / noun forms differ in their ends)
# the words an operator SAYS -> the word the manual uses (both sides of the search go through it; the stems are
# made at import): a curated list (the term chapter MANUAL_EL 14 + the question list), never a statement about
# the C2's behaviour. "σήμα" is NOT a synonym of "σύνδεση": the manual's "σήμα AI" is the badge.
SAID = {
    "drone": "UAV", "drones": "UAV", "ντρον": "UAV", "UAVs": "UAV",
    "joystick": "χειριστήριο", "λεβιές": "χειριστήριο", "λεβιέ": "χειριστήριο",
    "μηχανή": "κινητήρας", "μηχανής": "κινητήρας", "μοτέρ": "κινητήρας", "motor": "engine",
    "κόρνα": "μεγάφωνο",
    "κλείσιμο": "κλείνω", "κλείσει": "κλείνω", "κλείνει": "κλείνω",
    "άνοιγμα": "ανοιχτή", "ανοίγω": "ανοιχτή", "ανοίξει": "ανοιχτή", "ανοιχτό": "ανοιχτή",
    "πετάξω": "πετάω", "πετούν": "πετάω", "πτήση": "πετάω", "πετάει": "πετάω",
    "εκτοξεύσω": "εκτόξευση", "εκτοξεύεται": "εκτόξευση", "απογείωση": "εκτόξευση", "απογειώνεται": "εκτόξευση",
    "ξεκίνημα": "ξεκινά",
    "ψηλά": "ύψος", "ύψους": "ύψος",
    "μόνιτορ": "οθόνη",
    "σταματάω": "STOP", "σταματώ": "STOP", "σταματά": "STOP", "σταματήσω": "STOP", "σταματήστε": "STOP",
    "σταμάτημα": "STOP", "σταματάει": "STOP", "σταματήσει": "STOP",
    "σβήσιμο": "σβήνω", "σβήνει": "σβήνω", "σβηστή": "σβήνω", "σβήστηκε": "σβήνω", "σβησμένη": "σβήνω",
    "σβήσει": "σβήνω",
}
SYNONYMS = {}


# where the SOURCES are read (the manuals, the tour, this parser): the repository; a test copies them to a temp
# folder, changes one and points TRYGONS_ASSISTANT_SRC there (AM3-r MED-1 mutation test AS14). The index
# itself always stays in INDEX_DIR.
SRC_ROOT = os.environ.get("TRYGONS_ASSISTANT_SRC") or paths.REPO


def repo_path(rel):
    return os.path.join(SRC_ROOT, rel)


def fold(text):
    """Lower case, accents / diaeresis removed, final sigma = sigma."""
    t = unicodedata.normalize("NFD", text)
    t = "".join(c for c in t if unicodedata.category(c) != "Mn")
    return t.lower().replace("ς", "σ")


def stem(tok, lang):
    if tok.isdigit():
        return tok
    if lang == "el" or tok[0] in GREEK:
        if any(c in GREEK for c in tok) and any("a" <= c <= "z" for c in tok):
            tok = "".join(LOOK.get(c, c) for c in tok)          # HDG typed with a Greek H
        if tok[0] in GREEK:
            for s in EL_SUFFIX:
                if len(tok) - len(s) >= 3 and tok.endswith(s):
                    tok = tok[:-len(s)]
                    break
            return tok[:STEM_LEN["el"]]
    for s in EN_SUFFIX:
        if len(tok) - len(s) >= 3 and tok.endswith(s):
            tok = tok[:-len(s)]
            break
    return tok[:STEM_LEN["en"]]


def tokens(text, lang, stop=True):
    """The search terms of a text (both languages' stop words dropped: questions mix them)."""
    out = []
    for w in WORD_RE.findall(fold(text)):
        if stop and (w in STOP["el"] or w in STOP["en"]):
            continue
        if len(w) == 1 and not w.isdigit():
            continue
        t = stem(w, lang)
        out.append(SYNONYMS.get(t, t))
    return out


def _make_synonyms():
    for said, manual in SAID.items():
        for w in WORD_RE.findall(fold(said)):
            SYNONYMS[stem(w, "el")] = stem(WORD_RE.findall(fold(manual))[0], "el")


_make_synonyms()


def with_pairs(toks):
    """The words + every pair of neighbours ("οριο_ισχυ"): a name of two words counts more than its parts."""
    return toks + [a + "_" + b for a, b in zip(toks, toks[1:])]


# --- the manual -> passages -------------------------------------------------------------------------------------
def _indent(s):
    return len(s) - len(s.lstrip(" "))


def _is_heading(line, nxt):
    """A sub-heading: at column 0, short, no "label: text" form, and upper case (before any bracket) or a
    short line followed by an indented block."""
    s = line.strip()
    if not s or line[0] == " " or len(s) > 80 or RULE_RE.match(line) or BULLET_RE.match(line):
        return False
    head = s.split("(")[0]
    outside = re.sub(r"\([^)]*\)?", "", s)          # the text outside the brackets
    if re.search(r":\s+\S", outside) or s.endswith(("->", ",", ".")) or ". " in s:
        return False                              # "ΜΕΓΕΘΟΣ: η οθόνη 2 ..." = a paragraph with a label
    if _open(s) and nxt.startswith(" "):
        return False                              # "ΤΙΤΛΟΣ (... και" + an indented rest = a paragraph
    letters = [c for c in head if c.isalpha()]
    if len(letters) >= 3 and sum(1 for c in letters if c.isupper()) / float(len(letters)) >= 0.75:
        return True
    return len(s) <= 50 and bool(nxt) and nxt.startswith("  ") and not s.endswith((".", ":", ";"))


def _open(s):
    """A heading whose bracket / quotes are still open continues on the next column-0 line."""
    return s.count("(") > s.count(")") or s.count("«") > s.count("»") or s.count('"') % 2 == 1


def _words(s):
    return len(s.split())


def _split_long(text):
    """A long piece -> parts of <= MAX_WORDS words at sentence ends (a word cut only as the last resort)."""
    parts, cur = [], []
    for sent in SENT_RE.split(text):
        if cur and _words(" ".join(cur + [sent])) > MAX_WORDS:
            parts.append(" ".join(cur))
            cur = []
        cur.append(sent)
    if cur:
        parts.append(" ".join(cur))
    out = []
    for p in parts:
        w = p.split()
        while len(w) > MAX_WORDS * 1.5:
            out.append(" ".join(w[:MAX_WORDS]))
            w = w[MAX_WORDS:]
        out.append(" ".join(w))
    return out


DEF_RE = re.compile(r"^ {2,4}(\S(?:.{0,46}?\S)?) {2,}(\S.*)$")
LEVEL_RE = re.compile(r"^ {2,4}[ΣΠΕ] {2}(\S.*)$")       # the alarm chapter: "  Σ  ΑΠΩΛΕΙΑ ΠΥΞΙΔΑΣ   ..."
CAPS_RE = re.compile(r"^((?:[^a-zα-ωά-ώ]+?)(?=\s+[a-zα-ωά-ώ]|$))")


def _term(line):
    """A definition-list line ("  STANDBY           ρελαντί ...", "  Σ  ΑΠΩΛΕΙΑ ΠΥΞΙΔΑΣ   σε AUTO ...") -> its
    term, else None. The term names its passage (a mode, an alarm, a trouble)."""
    if BULLET_RE.match(line):
        return None
    lv = LEVEL_RE.match(line)
    if lv:
        rest = lv.group(1)
        m = re.match(r"(\S(?:.{0,46}?\S)?) {2,}", rest)
        if m:
            return m.group(1)
        m = CAPS_RE.match(rest)
        return (m.group(1).strip() if m else rest.split()[0]) or None
    m = DEF_RE.match(line)
    if not m or sum(c.isalpha() for c in m.group(1)) < 2:
        return None
    return m.group(1)


def parse_manual(text, lang):
    """MANUAL text -> passages (the contents list before chapter 1 is not a passage)."""
    lines = text.replace("\r\n", "\n").split("\n")
    out = []
    ch = ch_title = None
    heading = ""
    units = []                       # [(text, term, the line number of every word)] of the current heading

    def flush():
        acc, wl = [], []
        for (ut, term, uwl) in units:
            if term:                                   # a defined term = its own passage
                if acc:
                    emit(" ".join(acc), "", wl)
                    acc, wl = [], []
                emit(ut, term, uwl)
                continue
            acc.append(ut)
            wl = wl + uwl
            if _words(" ".join(acc)) >= MIN_WORDS:
                emit(" ".join(acc), "", wl)
                acc, wl = [], []
        if acc:
            last = out[-1] if out else None
            if last and last["ch"] == ch and last["heading"] == heading and not last["term"] and                     _words(last["text"]) + _words(" ".join(acc)) <= MAX_WORDS:
                last["text"] += " " + " ".join(acc)
                last["lines"][1] = wl[-1]
            else:
                emit(" ".join(acc), "", wl)
        del units[:]

    def emit(t, term, wl):
        """wl = the manual line of every word of t: each split part gets its OWN line range (AM3-r LOW-4)."""
        if ch in SKIP_CHAPTERS.get(lang, ()):
            return
        at = 0
        for part in _split_long(t):
            nw = len(part.split())
            a, b = wl[at], wl[at + nw - 1]
            at += nw
            if nw < MIN_KEEP:
                continue                                # "Ε  ΕΚΜΑΘΗΣΗ", "Περισσότερα: κεφάλαιο 18." = no answer
            out.append({"id": "%s:%d" % (lang, len(out) + 1), "kind": "manual", "ch": ch, "chapter": ch_title,
                        "heading": heading, "term": term, "text": part, "lines": [a, b]})

    cur = []                         # the current unit's (line number, stripped text)
    cur_term = [""]

    def end_unit():
        if cur:
            units.append((" ".join(t for _, t in cur), cur_term[0], [ln for ln, t in cur for _ in t.split()]))
            del cur[:]
        cur_term[0] = ""

    i = 0
    n = len(lines)
    heading_open = False
    while i < n:
        line = lines[i].rstrip()
        m = CHAPTER_RE.match(line)
        near_rule = (i > 0 and RULE_RE.match(lines[i - 1])) or (i + 1 < n and RULE_RE.match(lines[i + 1]))
        if m and near_rule:
            end_unit()
            if ch is not None:
                flush()
            ch, ch_title, heading = m.group(1), m.group(2).strip(), ""
            i += 1
            continue
        if ch is None or RULE_RE.match(line):
            if ch is not None:
                end_unit()
            i += 1
            continue
        if not line.strip():
            end_unit()
            heading_open = False
            i += 1
            continue
        nxt = lines[i + 1].rstrip() if i + 1 < n else ""
        if heading_open and line[0] != " ":       # a heading wrapped onto a second line "(... ...)"
            heading += " " + line.strip()
            heading_open = _open(heading)
            i += 1
            continue
        heading_open = False
        if _is_heading(line, nxt):
            end_unit()
            flush()
            heading = line.strip()
            heading_open = _open(heading)
            i += 1
            continue
        prev = lines[i - 1].rstrip() if i > 0 else ""
        term = _term(line)
        if term and cur and cur_term[0] and term.startswith("(") and _indent(line) == _indent(lines[cur[0][0] - 1]):
            cur_term[0] += " " + term                  # "  (MANUAL)   ..." under "  ΧΕΙΡΟΚΙΝΗΤΟ   ..."
        elif term:
            end_unit()
            cur_term[0] = term
        elif cur and prev.strip() and (BULLET_RE.match(line) or (_indent(line) <= 4 and _indent(line) < _indent(prev))):
            end_unit()
        cur.append((i + 1, line.strip()))
        i += 1
    end_unit()
    if ch is not None:
        flush()
    return out


def tour_passages(lang, path=None):
    """The guided tour's stops = small "where is it" passages (marked kind "tour")."""
    with open(path or repo_path(TOUR), encoding="utf-8") as f:
        tour = json.load(f)
    out = []
    for c in tour.get("chapters", []):
        for s in c.get("stops", []):
            text = s.get("text_" + lang) or ""
            if not text:
                continue
            out.append({"id": "tour:%s:%s" % (lang, s.get("id")), "kind": "tour", "ch": "tour",
                        "chapter": c.get("title_" + lang, ""), "heading": s.get("title_" + lang, ""),
                        "term": "", "text": " ".join(text.split()), "lines": [0, 0]})
    return out


# --- sources, stale check, save / load -------------------------------------------------------------------------
def sources(lang):
    """What the index is made from (AM3-r LOW-1: the lang files are NOT indexed - button names reach the
    speech recognition as term hints only, hints.py - so they are no source of the index either)."""
    return [MANUALS[lang], TOUR, SELF]


def sha_text(rel):
    """sha256 of a source with its line ends normalised (a Windows checkout with CRLF = the same index)."""
    with open(repo_path(rel), "rb") as f:
        data = f.read().replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def build(lang):
    with open(repo_path(MANUALS[lang]), encoding="utf-8") as f:
        ps = parse_manual(f.read(), lang)
    ps += tour_passages(lang)
    return {"version": INDEX_VERSION, "lang": lang,
            "sources": {rel.replace("\\", "/"): sha_text(rel) for rel in sources(lang)},
            "passages": ps}


def index_path(lang, folder=None):
    return os.path.join(folder or INDEX_DIR, "manual_%s.json" % lang)


def save(idx, folder=None):
    folder = folder or INDEX_DIR
    os.makedirs(folder, exist_ok=True)
    p = index_path(idx["lang"], folder)
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(idx, f, ensure_ascii=False, indent=0, sort_keys=True)
        f.write("\n")
    os.replace(tmp, p)
    return p


def load(lang, folder=None):
    with open(index_path(lang, folder), encoding="utf-8") as f:
        return json.load(f)


def stale(lang, folder=None):
    """Why the committed index does not match its sources ([] = current)."""
    try:
        idx = load(lang, folder)
    except (OSError, ValueError) as e:
        return ["no index: %s" % e]
    why = []
    if idx.get("version") != INDEX_VERSION:
        why.append("index version %r != %d" % (idx.get("version"), INDEX_VERSION))
    have = idx.get("sources") or {}
    for rel in sources(lang):
        key = rel.replace("\\", "/")
        if have.get(key) != sha_text(rel):
            why.append("%s changed" % key)
    # AM3-r2 LOW: the passages themselves = a fresh build (a hand-edited manual_*.json with the right hashes FAILS)
    if not why:
        try:
            fresh = build(lang)["passages"]
        except (OSError, ValueError) as e:
            return ["sources not readable: %s" % e]
        if idx.get("passages") != fresh:
            why.append("passages differ from a fresh build (index edited by hand?)")
    return why


# --- search ---------------------------------------------------------------------------------------------------
class Searcher:
    """BM25 over every passage (text + the heading path x HEAD_WEIGHT)."""

    def __init__(self, idx, min_score=MIN_SCORE, min_cover=None):
        self.lang = idx["lang"]
        self.passages = idx["passages"]
        self.min_score = min_score
        self.min_cover = MIN_COVER.get(self.lang, 0.35) if min_cover is None else min_cover
        self.docs = []
        df = {}
        for p in self.passages:
            head = " ".join(x for x in (p.get("chapter") or "", p.get("heading") or "", p.get("term") or "") if x)
            toks = with_pairs(tokens(p["text"], self.lang)) + with_pairs(tokens(head, self.lang)) * HEAD_WEIGHT
            tf = {}
            for t in toks:
                tf[t] = tf.get(t, 0) + 1
            self.docs.append((tf, len(toks)))
            for t in tf:
                df[t] = df.get(t, 0) + 1
        self.n = len(self.docs)
        self.avg = sum(d[1] for d in self.docs) / float(max(1, self.n))
        self.idf = {t: math.log(1 + (self.n - c + 0.5) / (c + 0.5)) for t, c in df.items()}
        self.idf_unknown = math.log(1 + (self.n + 0.5) / 0.5)

    def search(self, q, k=3):
        qt = []
        for t in tokens(q, self.lang):
            if t not in qt:
                qt.append(t)
        total = sum(self.idf.get(t, self.idf_unknown) for t in qt)
        if not qt or total <= 0:
            return {"hits": [], "cant_find": True, "why": "no search words", "terms": qt}
        terms = [t for t in with_pairs(qt) if t in self.idf]
        scored = []
        for i, (tf, ln) in enumerate(self.docs):
            s = cov = 0.0
            for t in terms:
                f = tf.get(t)
                if not f:
                    continue
                idf = self.idf[t]
                s += idf * f * (K1 + 1) / (f + K1 * (1 - B + B * ln / self.avg))
                if "_" not in t:
                    cov += idf
            if s > 0:
                if self.passages[i]["kind"] == "tour":
                    s *= TOUR_FACTOR
                scored.append((s, cov / total, i))
        scored.sort(key=lambda x: (-x[0], x[2]))
        hits = [{"passage": self.passages[i], "score": round(s, 3), "cover": round(c, 3),
                 "after": following(self.passages, i)} for s, c, i in scored[:k]]
        if not hits:
            return {"hits": [], "cant_find": True, "why": "no passage has any search word", "terms": qt}
        best = hits[0]
        why = ""
        if best["score"] < self.min_score:
            why = "best score %.2f < %.2f" % (best["score"], self.min_score)
        elif best["cover"] < self.min_cover:
            why = "best covers %.0f %% of the question < %.0f %%" % (best["cover"] * 100, self.min_cover * 100)
        return {"hits": hits, "cant_find": bool(why), "why": why, "terms": qt}


FOLLOWING_N = 2


def following(passages, i, n=FOLLOWING_N):
    """The n MANUAL passages that FOLLOW passages[i] as consecutive manual text UNDER THE SAME HEADING (the next
    ones in the index, the same `ch`, the same `heading`, the same `term`, their line range going on) - AM4-r4 MED-1
    (a): a picked sentence that ends its passage is offered together with the limiting sentence that OPENS the next
    passage (answer.pick_units). AM4-r5 (W126): the chain STOPS at a heading change (a statement and its limits
    never cross a heading - the next heading is another subject); MED-3: a defined TERM whose definition the index
    builder split over consecutive passages (the same term, the lines following on: the UNCREWED HDG LOCK block,
    SCR) IS chained - a DIFFERENT term is never chained to (two definitions would be glued into one "sentence")."""
    out = []
    p = passages[i] if 0 <= i < len(passages) else None
    if not p or p.get("kind") != "manual":
        return out
    last = p
    for q in passages[i + 1:i + 1 + n]:
        if q.get("kind") != "manual" or q.get("ch") != p.get("ch") or (q.get("heading") or "") != (
                p.get("heading") or "") or (q.get("term") or "") != (p.get("term") or "") or (
                q.get("lines") or [0])[0] < (last.get("lines") or [0, 0])[1]:
            break
        out.append(q)
        last = q
    return out


def _summary(p):
    return {"id": p["id"], "ch": p["ch"], "chapter": p["chapter"], "heading": p["heading"], "term": p.get("term", ""),
            "lines": p["lines"], "text": p["text"]}


def hit_summary(res):
    """The worker's / the Tools page's short form of a search (no full texts); "after" = the following passages of
    the chapter (AM4-r4 MED-1 a, for the pick units)."""
    return {"cant_find": res["cant_find"], "why": res["why"],
            "hits": [dict(_summary(h["passage"]), score=h["score"], cover=h["cover"],
                          after=[_summary(a) for a in h.get("after", ())]) for h in res["hits"]]}
