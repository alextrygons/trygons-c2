"""The talking manual's answer step (step AM4): the found passages -> 1-2 short spoken sentences, CHECKED.

    a = Answerer(llm, lang_terms)            llm = LlamaLlm(...) | StubLlm(...) | None (= retrieval only)
    a.answer(q, lang, found) -> {"a", "src", "mode", "cite", "id", "passage", "problems", "ms", "gen_ms", "tokens"}

src = what is said:
    "llm"        the model's answer passed the post-check
    "extract"    (answer mode "extract", AM4-r) the model only PICKED passage sentences by number; they are said
                 word for word
    "passage"    the model's answer FAILED the post-check (a name / number / negation not in the passages, the
                 wrong language) -> «Δεν βρίσκω απάντηση στο εγχειρίδιο.» is said and the best passage is SHOWN with
                 its chapter (AM4-r, the review's rule; W105; before AM4-r the passage's first sentence was said)
    "retrieval"  no model / retrieval-only mode / the model too slow -> the best passage's SHORTEST full sentence
    "cant_find"  the search found nothing (index.Searcher's thresholds) OR the model said the passages don't answer
                 -> «Δεν βρίσκω απάντηση στο εγχειρίδιο.» / "I can't find that in the manual."

Grounding (docs/assistant_design.md 5): the model sees ONLY the question + the top passages (numbered, chapter +
heading) and a fixed instruction in the question's language; temperature 0; at most 2 sentences are kept.
POST-CHECK before anything is used (docs/assistant_design.md 5c):
  - every NAME in the answer (a word in capitals, a «quoted» / "quoted" span, a C2 name from the screens' capital
    texts, a Capitalised "X button" / «κουμπί X» - also when X is an allowed abbreviation like PC) must be in the
    passages;
  - every NUMBER (digits, or number WORDS - "εβδομήντα", "τριακόσια", "eighty", "twenty-five" - turned into digits
    first; the sign kept: "-3 s" is not "3 s"): with a KNOWN unit ("5 kn", "3 s", "50 m", "30 %", "120 m", "3 hrs",
    "50 feet", "20 per cent") it must be in the passages with that unit; followed by any OTHER word, the number
    AND that word must stand together in the passages ("50 feet" with "50 m" in the passages = refused); alone, the
    value must be in the passages; AND the number must be in a passage SENTENCE together with a word standing next
    to it in the answer (a swapped "20 % reverse / 5 % forward" is caught when the manual states them in separate
    sentences; in ONE manual sentence a swap is not caught - a lexical check, stated in the docs);
  - a negation (never / not / no / δεν / ποτέ / όχι / χωρίς ...) in an answer sentence must also be in the passage
    sentence that supports it, and the other way round ("STANDBY never wins", "the PC can override STANDBY");
    an answer sentence with content words that shares none with any passage sentence is not grounded;
  - the answer must be in the question's language (the Greek-script share of the words that are not names from
    the passages);
  - a chapter the answer names must be one of theirs.
The chapter / tour stop the answer comes from is added by THIS code (cite), never by the model (AM5 speaks "from the
manual, chapter N"). Pure Python + the model object: no process, no network, no file written.
"""
import os
import re
import sys
import time
import unicodedata

CANT = {"el": "Δεν βρίσκω απάντηση στο εγχειρίδιο.", "en": "I can't find that in the manual."}
MAX_SENT = 2
MAX_TOKENS = 160          # 2 Greek sentences are ~40 words = ~120 tokens in these tokenizers (AM4 measured: the
#                           longest kept answer was 72 tokens - the 2-sentence stop ends it first, 5c)
MAX_GEN_S = 10.0          # a longer generation is cut = this answer said from the passage (retrieval form)
SLOW_S = 8.0              # the last SLOW_N answers all slower = RETRIEVAL-ONLY mode (docs 5)
SLOW_N = 3
N_PASSAGES = 3
N_CTX = 3072              # 3 passages of <= 120 words + the instruction + the answer fit with room (Greek ~3 tokens / word)
MAX_PICK_SENT = 30        # answer mode "extract": at most this many numbered passage sentences are offered
MAX_PICK_WORDS = 45       # ... and two picked sentences longer than this together = only the first is said
MODES = ("generate", "extract")

SYSTEM = {
    "el": ("Είσαι το ομιλούν εγχειρίδιο του σταθμού Trygons C2. Απάντησε ΜΟΝΟ από τα αποσπάσματα του εγχειριδίου "
           "που δίνονται, στα ελληνικά, με μία ή δύο σύντομες ολόκληρες προτάσεις (με ρήμα) που θα ακουστούν. Γράψε τα ονόματα "
           "κουμπιών, λειτουργιών, οθονών και συναγερμών ακριβώς όπως στα αποσπάσματα, και κάθε αριθμό με τη "
           "μονάδα του ακριβώς όπως εκεί. Μην προσθέσεις τίποτα που δεν γράφουν τα αποσπάσματα. Μην αναφέρεις "
           "αριθμό κεφαλαίου. Αν τα αποσπάσματα δεν απαντούν στην ερώτηση, γράψε ακριβώς: " + CANT["el"]),
    "en": ("You are the talking manual of the Trygons C2 station. Answer ONLY from the manual passages given, in "
           "English, in one or two short complete sentences (with a verb) that will be spoken. Write button, mode, screen and alarm names "
           "exactly as in the passages, and every number with its unit exactly as there. Add nothing the passages "
           "do not say. Do not mention a chapter number. If the passages do not answer the question, reply "
           "exactly: " + CANT["en"]),
}
# answer mode "extract" (AM4-r): the model never writes a sentence - it PICKS one or two numbered passage sentences
SYSTEM_PICK = {
    "el": ("Είσαι το ομιλούν εγχειρίδιο του σταθμού Trygons C2. Σου δίνονται αριθμημένες προτάσεις του εγχειριδίου "
           "και μια ερώτηση. Διάλεξε τη μία ή τις δύο προτάσεις που απαντούν στην ερώτηση και γράψε ΜΟΝΟ τους "
           "αριθμούς τους, χωρισμένους με κόμμα (π.χ. 4 ή 4, 9). Μη γράψεις τίποτα άλλο. Αν καμία πρόταση δεν "
           "απαντά στην ερώτηση, γράψε ακριβώς: " + CANT["el"]),
    "en": ("You are the talking manual of the Trygons C2 station. You are given numbered manual sentences and a "
           "question. Pick the one or two sentences that answer the question and write ONLY their numbers, separated "
           "by a comma (e.g. 4 or 4, 9). Write nothing else. If no sentence answers the question, reply exactly: "
           + CANT["en"]),
}
LABEL = {"el": ("Αποσπάσματα του εγχειριδίου", "Κεφάλαιο", "Ξενάγηση", "Ερώτηση", "Προτάσεις του εγχειριδίου"),
         "en": ("Manual passages", "Chapter", "Guided tour", "Question", "Manual sentences")}


# --- text helpers -------------------------------------------------------------------------------------------------
def plain(text):
    """Lower case, accents removed, final sigma = sigma (the comparisons of the post-check)."""
    t = unicodedata.normalize("NFD", text)
    t = "".join(c for c in t if unicodedata.category(c) != "Mn")
    return t.lower().replace("ς", "σ")


SENT_RE = re.compile(r"(?<=[.!;·?])\s+(?=\S)")


def sentences(text):
    return [s.strip() for s in SENT_RE.split(" ".join((text or "").split())) if s.strip()]


def first_sentence(text):
    s = sentences(text)
    return s[0] if s else (text or "").strip()


def shortest_sentence(text, min_words=5):
    """The shortest full sentence of at least min_words words (else the first one) - the retrieval-only form."""
    s = sentences(text)
    full = [x for x in s if len(x.split()) >= min_words and _whole(x)]
    return min(full, key=len) if full else (s[0] if s else (text or "").strip())


def _whole(s):
    """A sentence that stands alone: starts like a sentence (a capital, a digit, a quote), brackets balanced."""
    c = s.lstrip("«\"'(")[:1]
    return bool(c) and (c.isupper() or c.isdigit()) and s.count("(") == s.count(")") and \
        s.count("«") == s.count("»")


# --- numbers + units ----------------------------------------------------------------------------------------------
UNITS = {}
for canon, words in {
        "kn": "kn kt kts knots knot κομβοι κομβους κομβων κομβο κομβου κομβοσ",
        "s": "s sec secs δευτ δευτερολεπτα δευτερολεπτο δευτερολεπτου δευτερολεπτων seconds second",
        "ms": "ms msec msecs millisecond milliseconds χιλιοστα-του-δευτερολεπτου",
        "min": "min mins λεπτα λεπτο λεπτου λεπτων λεπ minutes minute",
        "h": "h hr hrs ωρα ωρες ωρων ωρεσ hours hour",
        "m": "m μ μετρα μετρο μετρων meters metres metre meter",
        "cm": "cm εκατοστα", "mm": "mm χιλιοστα",
        "km": "km χιλιομετρα χιλιομετρο", "nm": "nm nmi nautical ναυτικα ναυτικο ναυτικων",
        "mi": "mi mile miles μιλια μιλι", "ft": "ft feet foot ποδια ποδι",
        "%": "% τοισ percent pct", "deg": "° μοιρεσ μοιρα μοιρων degrees degree deg",
        "m/s": "m/s", "km/h": "km/h kph", "mph": "mph", "deg/s": "°/s deg/s",
        "V": "v βολτ volt volts", "Hz": "hz", "fps": "fps", "GB": "gb", "MB": "mb", "kbit": "kbit kbps",
        "mAh": "mah", "A": "a αμπερ", "W": "w βατ watt watts", "kg": "kg κιλα κιλο"}.items():
    for w in words.split():
        UNITS[plain(w)] = canon                    # plain: "κόμβους" -> "κομβουσ" (final sigma), as the texts are

# number WORDS -> digits (AM4-r MED-1: every form, so "εβδομήντα δευτερόλεπτα" / "eighty seconds" / "twenty-five"
# are checked like digits; a bare "ένα" / "μία" / "one" / "half" is an article unless a unit follows it)
_EL_SMALL = {"δυο": 2, "τρια": 3, "τρεισ": 3, "τεσσερα": 4, "τεσσερισ": 4, "πεντε": 5, "εξι": 6, "επτα": 7, "εφτα": 7,
             "οκτω": 8, "οχτω": 8, "εννεα": 9, "εννια": 9, "δεκα": 10, "εντεκα": 11, "ενδεκα": 11, "δωδεκα": 12,
             "δεκατρια": 13, "δεκατρεισ": 13, "δεκατεσσερα": 14, "δεκατεσσερισ": 14, "δεκαπεντε": 15, "δεκαεξι": 16,
             "δεκαεπτα": 17, "δεκαεφτα": 17, "δεκαοκτω": 18, "δεκαοχτω": 18, "δεκαεννεα": 19, "δεκαεννια": 19,
             "εναμιση": 1.5, "μιαμιση": 1.5}
_EL_TENS = {"εικοσι": 20, "τριαντα": 30, "σαραντα": 40, "πενηντα": 50, "εξηντα": 60, "εβδομηντα": 70, "ογδοντα": 80,
            "ενενηντα": 90}
_EL_HUNDREDS = {}
for _i, _w in enumerate(("εκατο", "διακοσ", "τριακοσ", "τετρακοσ", "πεντακοσ", "εξακοσ", "επτακοσ", "οκτακοσ",
                         "εννιακοσ"), 1):
    for _end in ("", "ν", "ια", "ιεσ", "ιοι", "ιων", "ιουσ") if _w == "εκατο" else ("ια", "ιεσ", "ιοι", "ιων", "ιουσ"):
        _EL_HUNDREDS[_w + _end] = _i * 100
_EL_HUNDREDS.update({"εφτακοσια": 700, "οχτακοσια": 800, "εννεακοσια": 900})
_EL_THOUSAND = {"χιλια": 1000, "χιλιεσ": 1000, "χιλιοι": 1000, "χιλιων": 1000, "χιλιαδεσ": 1000}
_EN_SMALL = {"two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
             "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16,
             "seventeen": 17, "eighteen": 18, "nineteen": 19, "dozen": 12}
_EN_TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80,
            "ninety": 90}
_SMALL = dict(_EL_SMALL, **_EN_SMALL)
_TENS = dict(_EL_TENS, **_EN_TENS)
ARTICLES = {"ενα": 1, "ενασ": 1, "μια": 1, "μισο": 0.5, "μιση": 0.5, "μισοσ": 0.5, "one": 1, "half": 0.5}
NUM_WORDS = dict(_SMALL, **_TENS, **_EL_HUNDREDS, **_EL_THOUSAND, hundred=100, thousand=1000, **ARTICLES)
_JOIN = {"και", "and"}


def _wordval(w):
    """The value of ONE number word (compounds like "εικοσιπεντε" too), or None."""
    if w in NUM_WORDS:
        return NUM_WORDS[w]
    for t, tv in _TENS.items():
        if w.startswith(t) and w[len(t):] in _SMALL and _SMALL[w[len(t):]] < 10:
            return tv + _SMALL[w[len(t):]]
    return None


def _run_value(words):
    """The value of a run of number words: Greek hundreds are absolute (διακόσια = 200), "hundred" / "thousand"
    multiply what stands before them."""
    total, cur = 0, 0
    for w in words:
        v = _wordval(w)
        if w in ("hundred",):
            cur = (cur or 1) * 100
        elif w in ("thousand",) or w in _EL_THOUSAND:
            total += (cur or 1) * 1000
            cur = 0
        else:
            cur += v
    return total + cur


def digitize(t):
    """plain text -> the same with every number word (run) written in digits. A lone article word (ένα / μία / one /
    half) counts only when a known unit follows it."""
    parts = re.split(r"(\W+)", t)                      # words at even indexes, separators at odd ones
    out, i, n = [], 0, len(parts)
    while i < n:
        w = parts[i]
        if i % 2 == 1 or _wordval(w) is None:
            out.append(w)
            i += 1
            continue
        run, j = [w], i
        while j + 2 < n and parts[j + 1].strip() in ("", "-", "–") and (
                _wordval(parts[j + 2]) is not None or (parts[j + 2] in _JOIN and j + 4 < n and
                                                       _wordval(parts[j + 4]) is not None)):
            if parts[j + 2] in _JOIN:
                j += 2
                continue
            run.append(parts[j + 2])
            j += 2
        nxt = parts[j + 2] if j + 2 < n and parts[j + 1].strip() == "" else ""
        if all(x in ARTICLES for x in run) and nxt not in UNITS:
            out.append(w)                               # "ένα κουμπί": an article, not a number
            i += 1
            continue
        v = _run_value(run)
        out.append(("%g" % v) if v != int(v) else str(int(v)))
        i = j + 1
    return "".join(out)


def norm(text):
    """plain + the unit phrases ("per cent", "τοις εκατό") in one word + number words as digits."""
    t = plain(text)
    t = re.sub(r"\bper\s+cent\b", "percent", t)
    t = re.sub(r"\bτοισ\s+εκατο\b", "%", t)
    t = re.sub(r"\bχιλιοστα\s+του\s+δευτερολεπτου\b", "ms", t)
    return digitize(t)


NUM = r"(\d+(?:[.,]\d+)?)"
NUM_RE = re.compile(r"(?<![\w.,%°/])(-?)" + NUM + r"(?:\s*(?:-|–|\.\.\.?|…|εωσ|ωσ|to|until)\s*" + NUM +
                    r")?(?:\s*(%|°(?:/s)?)|\s*([^\W\d_]+(?:/[^\W\d_]+)?))?")
CHAPTER_RE = re.compile(r"(?:κεφαλαιο|κεφαλαιου|κεφ\.?|chapter|ch\.)\s*(\d+[a-z]?)")
ORDINAL_SUFFIX = {"ο", "η", "οσ", "ου", "ησ", "α", "εσ", "ων", "ουσ", "st", "nd", "rd", "th"}
DATE_RE = re.compile(r"(?<!\d)\d{4}-\d{2}-\d{2}(?!\d)")


def _tok_re(v):
    """The number `v` as it may stand in a normed text: "0.5" or "0,5", not inside another number / a word."""
    s = "%g" % abs(v)
    s = re.escape(s).replace(r"\.", "[.,]")
    return re.compile(r"(?<!\w)(?<!\d[.,])-?" + s + r"(?!\d)(?![.,]\d)")


def _f(x):
    return float(x.replace(",", "."))


def numbers(text, normed=False):
    """[(value, unit or None, following word or None, position)] of a text: digits (a range gives its unit to
    both ends), number words as digits (norm), the sign kept. Chapter references are left out. A following word
    that is no known unit comes back as `word` (the caller checks "number word" together)."""
    t = text if normed else norm(text)
    t = CHAPTER_RE.sub(lambda m: " " * len(m.group(0)), t)
    t = DATE_RE.sub(lambda m: " " * len(m.group(0)), t)         # "2026-11-04" is a date, not three numbers
    out = []
    for m in NUM_RE.finditer(t):
        sign = -1.0 if m.group(1) == "-" else 1.0
        unit, word = None, None
        if m.group(4):
            unit = UNITS.get(m.group(4), m.group(4))
        elif m.group(5):
            unit = UNITS.get(m.group(5))
            word = None if unit else m.group(5)
            glued = m.start(5) == m.end(3 if m.group(3) else 2)
            if glued and word is not None:
                if word in ORDINAL_SUFFIX:          # "7ο", "1η", "2ος": an ordinal = a bare number
                    word = None
                else:
                    continue                        # "13q2c", "v2", "x3b": an identifier, not a number
        for g in (m.group(2), m.group(3)):
            if g:
                out.append((sign * _f(g), unit, word, m.start()))
    return out


def chapters_named(text):
    return set(CHAPTER_RE.findall(plain(text)))


# --- names --------------------------------------------------------------------------------------------------------
CAPS_RE = re.compile(r"(?<![\w])([A-ZΑ-ΩΆΈΉΊΌΎΏΪΫ][A-ZΑ-ΩΆΈΉΊΌΎΏΪΫ0-9]+(?:[ /-][A-ZΑ-ΩΆΈΉΊΌΎΏΪΫ0-9]{2,})*)(?![\w])")
QUOTE_RE = re.compile(r"«([^»]{2,60})»|\"([^\"]{2,60})\"|“([^”]{2,60})”")
ALLOWED_CAPS = {"c2", "trygons", "pc", "ok"}        # the product's own name / the plain abbreviations
# a Capitalised (or capital) word used as the name of a thing: "the Reset button", "PC button", «κουμπί Reset»,
# «σελίδα Κάμερες» - a NAME whatever its spelling (AM4-r MED-2; no ALLOWED_CAPS exemption here)
_THING_EN = r"(?:button|key|page|tab|mode|alarm|lamp|panel|screen|switch|window|menu|setting)s?"
_THING_EL = (r"(?:κουμπί|κουμπιού|κουμπιά|πλήκτρο|πλήκτρου|σελίδα|σελίδας|σελίδες|καρτέλα|καρτέλας|λειτουργία|"
             r"λειτουργίας|συναγερμός|συναγερμού|συναγερμό|λυχνία|λυχνίας|πίνακας|πίνακα|οθόνη|οθόνης|διακόπτης|"
             r"διακόπτη|παράθυρο|μενού|ρύθμιση|ρύθμισης)")
_CAPW = r"[A-ZΑ-ΩΆΈΉΊΌΎΏΪΫ][\w-]*"
NAMED_RE = re.compile(r"(?<![\w])(" + _CAPW + r"(?:\s" + _CAPW + r"){0,3})\s+" + _THING_EN + r"(?![\w])")
NAMED_EL_RE = re.compile(r"(?<![\w])" + _THING_EL + r"\s+«?(" + _CAPW + r"(?:\s" + _CAPW + r"){0,3})")
_EN_FIRST = {"the", "a", "an", "this", "that", "every", "each", "any", "no", "its", "your", "my", "our", "one",
             "press", "use", "click", "hold", "open", "same", "other", "another", "next", "first", "second", "last",
             "left", "right", "new", "old", "only"}


def lang_terms(lang_dir=None):
    """The C2 names of the screens: every lang value written in CAPITALS of 2..5 words - buttons, modes, alarms,
    notices ("ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ", "HDG LOCK") - in plain form, so the answer's normal-case "κλείσιμο καταπακτής"
    is checked too. Both languages (an answer may name an English mode in a Greek sentence). A single capital word
    ("ΜΑΓΓΕΜΑ") is checked by the capitals rule only (in normal case it is an everyday word)."""
    lang_dir = lang_dir or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lang")
    out = set()
    for fn in ("el.txt", "en.txt"):
        try:
            with open(os.path.join(lang_dir, fn), encoding="utf-8") as f:
                lines = f.read().splitlines()
        except OSError:
            continue
        for line in lines:
            if "=" not in line or line.lstrip().startswith("#"):
                continue
            v = line.split("=", 1)[1].strip()
            letters = [c for c in v if c.isalpha()]
            if not letters or "{" in v or not all(c.isupper() for c in letters):
                continue
            words = re.findall(r"[^\W\d_]{2,}", plain(v))
            if 2 <= len(words) <= 5:              # one capital word alone is checked only when written in capitals
                out.add(" ".join(words))
    return out


def _wordy(text, normed=False):
    """normed text (plain, number words as digits) with only word characters, single spaces, padded (whole-word
    'in' tests)."""
    t = text if normed else norm(text)
    return " " + " ".join(re.findall(r"[\w%°]+(?:/[\w]+)?", t)) + " "


def names(text, terms=()):
    """The names an answer uses: words in capitals (+ their runs), quoted spans, known C2 terms (plain form), a
    Capitalised word standing as the name of a button / page / mode ... (AM4-r MED-2)."""
    out = set()
    for m in CAPS_RE.finditer(text):
        n = " ".join(plain(m.group(1)).split())
        if n not in ALLOWED_CAPS:
            out.add(n)
    for m in QUOTE_RE.finditer(text):
        out.add(" ".join(plain(next(g for g in m.groups() if g)).split()))
    for m in NAMED_RE.finditer(text):
        words = m.group(1).split()
        while words and plain(words[0]) in _EN_FIRST:     # "Press the Reset button": the name is "Reset"
            words = words[1:]
        if words:
            out.add(" ".join(plain(" ".join(words)).split()))
    for m in NAMED_EL_RE.finditer(text):
        out.add(" ".join(plain(m.group(1)).split()))
    w = _wordy(text)
    for t in terms:
        tw = " ".join(re.findall(r"[\w%°]+", t))
        if tw and (" " + tw + " ") in w:
            out.add(t)
    return out


# --- sentences, content words, negation (AM4-r MED-2) ---------------------------------------------------------------
STOP = set("""ο η το οι τα του τησ των τον την τη τουσ τισ και κι να θα σε στο στη στην στον στα στισ στουσ στησ στου
    με για απο που πωσ τι ποιο ποια ποιοσ ειναι αν οτι οταν αυτο αυτη αυτοσ αυτα μου σου μασ σασ πρεπει μπορει
    μπορειτε μπορουν εχει εχουν ειχε μετα πριν ενω οσο ωσ εωσ κατω πανω κατι καθε μονο παντα παντοτε ακομα ηδη
    πολυ λιγο ετσι οπωσ αλλα ομωσ επισησ τοτε εκει εδω αυτον αυτην σαν εαν
    a an the of to in on at for from by with and or is are was were be been being do does did it its this that
    these those what which who how when where why can could should would will shall may might must there here
    then than so if yes me us them they he she his her about into out up down get got have has had am any some
    just also always still only each every both all more most less very same other another such as while after
    before over under again further once per""".split()) | {"ο", "η", "το", "σε", "με", "να", "τη", "τα", "οι"}
NEG_RE = re.compile(r"(?<![\w])(?:δεν|δε|μην|μη|ποτε|οχι|κανενα|κανενασ|καμια|καμμια|ουτε|χωρισ|τιποτα|κανεισ|"
                    r"not|never|no|cannot|without|none|nobody|nothing|nor|neither)(?![\w])|n't")
GREEK_RE = re.compile(r"[\u0370-\u03ff\u1f00-\u1fff]")


NEG_WORDS = set("δεν δε μην μη ποτε οχι κανενα κανενασ καμια καμμια ουτε χωρισ τιποτα κανεισ not never no cannot "
                "without none nobody nothing nor neither".split())


def content_words(normed):
    """The words that carry meaning (no stop words, negations, units, numbers, 1-letter bits; "pc" / "dp" stay) of
    a normed text, in order."""
    return [w for w in re.findall(r"[^\W\d_]{2,}", normed) if w not in STOP and w not in UNITS and w not in NEG_WORDS]


def _same(a, b):
    """Two content words are the same word: equal, or the same first 5 letters (Greek endings, English -ed / -s)."""
    return a == b or (len(a) >= 5 and len(b) >= 5 and a[:5] == b[:5]) or (min(len(a), len(b)) >= 4 and (
        a.startswith(b) or b.startswith(a)))


def _shares(words, other):
    return sum(1 for w in words if any(_same(w, o) for o in other))


def has_negation(normed):
    return bool(NEG_RE.search(normed))


def negated_shared(ps, shared, window=2):
    """A negation word in the passage sentence `ps` standing up to `window` tokens BEFORE one of the `shared` content
    words - the manual negates the very thing the answer states ("the PC can NEVER override it" vs "the PC can
    override"; "ποτέ δεν προσθέτει" vs "προσθέτει"). Only before: Greek and English put the negation in front of
    the verb / noun; a negation about something else in the same sentence is not held against the answer."""
    toks = re.findall(r"[\w']+", ps)
    negs = [i for i, t in enumerate(toks) if t in NEG_WORDS or t.endswith("n't")]
    for i in negs:
        for j in range(i + 1, min(len(toks), i + window + 1)):
            if any(_same(toks[j], w) for w in shared):
                return True
    return False


def _passage_sentences(hits):
    """[(normed sentence, hit)] of the passages (chapter / heading / term count as a sentence of their own)."""
    out = []
    for h in hits:
        head = " ".join(x for x in (h.get("chapter", ""), h.get("heading", ""), h.get("term", "")) if x)
        if head:
            out.append((norm(head), h))
        for s in sentences(h.get("text") or ""):
            out.append((norm(s), h))
    return out


def _near_words(normed, pos, k=1):
    """The nearest k content word(s) on each side of the number at `pos` in the normed answer sentence (k = 1: a
    farther word must not vouch for the number - "reverse is capped at 20 % and forward at 5 %")."""
    before = content_words(normed[:pos])[-k:]
    m = NUM_RE.match(normed, pos)
    after = content_words(normed[m.end() if m else pos:])[:k]
    return before + after


def wrong_language(ans, lang, src_words):
    """The answer is not in the question's language: among its words that are NOT words of the passages (names
    like HDG LOCK are), the Greek-script letter share is < 1/2 for Greek or > 1/2 for English (AM4-r LOW-1)."""
    own = [w for w in re.findall(r"[^\W\d_]+", ans) if plain(w) not in src_words]
    letters = "".join(own)
    if len(letters) < 6:
        return False
    greek = len(GREEK_RE.findall(letters))
    share = greek / float(len(letters))
    return share < 0.5 if lang == "el" else share > 0.5


def postcheck(ans, hits, terms=(), lang=None):
    """[] = every name / number(+unit) / negation / chapter of the answer is in the passages (docs 5c); else the
    problems found."""
    src = " \n ".join("%s %s %s %s" % (h.get("chapter", ""), h.get("heading", ""), h.get("term", ""), h.get("text", ""))
                      for h in hits)
    srcn = norm(src)
    sw = _wordy(srcn, normed=True)
    probs = []
    for n in sorted(names(ans, terms)):
        nw = " ".join(re.findall(r"[\w%°]+", norm(n)))
        if nw and (" " + nw + " ") not in sw:
            probs.append("name not in the passages: %s" % n)
    have = numbers(srcn, normed=True)
    with_unit = {(v, u) for v, u, _w, _p in have if u}
    vals = {v for v, _u, _w, _p in have}
    psents = _passage_sentences(hits)
    for s in sentences(ans):
        sn = norm(s)
        for v, u, word, pos in numbers(sn, normed=True):
            if u is not None:
                if (v, u) not in with_unit:
                    probs.append("number + unit not in the passages: %g %s" % (v, u))
                    continue
            elif word is not None:                   # (_wordy drops the sign: "±1 / ±10" -> "1 10")
                if (" %s %s " % (("%g" % abs(v)).replace(".", ","), word)) not in sw and \
                        (" %g %s " % (abs(v), word)) not in sw:
                    probs.append("number with an unknown unit / word not in the passages: %g %s" % (v, word))
                    continue
            elif v not in vals:
                probs.append("number not in the passages: %g" % v)
                continue
            near = _near_words(sn, pos)
            if near:                                 # MED-2: the number in a passage SENTENCE with a word beside it
                tre = _tok_re(v)                     # (its unit was checked above against the whole source)
                ok = any(tre.search(ps) and _shares(near, content_words(ps)) for ps, _h in psents)
                if not ok:
                    probs.append("number not in a passage sentence with its words: %g %s (%s)" % (
                        v, u or word or "", " ".join(near)))
        cw = content_words(sn)
        if cw:
            scored = [(_shares(cw, content_words(ps)), ps) for ps, _h in psents]
            top = max((sc for sc, _ps in scored), default=0)
            best = [ps for sc, ps in scored if sc == top and sc > 0]
            if not best:
                probs.append("sentence not grounded in any passage sentence: %s" % s[:60])
            elif has_negation(sn):                   # a negated answer: a top passage sentence must negate too
                if not any(has_negation(ps) for ps in best):
                    probs.append("negation not in the passage sentence: %s" % s[:60])
            else:                                    # a plain answer: no top sentence may negate a shared word
                for ps in best:
                    shared = [w for w in cw if any(_same(w, o) for o in content_words(ps))]
                    if negated_shared(ps, shared):
                        probs.append("the passage sentence negates it: %s" % s[:60])
                        break
        elif has_negation(sn):
            probs.append("negation without a supporting passage sentence: %s" % s[:60])
    if lang in CANT and wrong_language(ans, lang, set(re.findall(r"[^\W\d_]+", plain(src)))):
        probs.append("not in the question's language (%s)" % lang)
    chs = {str(h.get("ch", "")).lower() for h in hits}
    for c in chapters_named(ans):
        if c not in chs:
            probs.append("chapter not among the passages: %s" % c)
    return probs


# --- the prompt ---------------------------------------------------------------------------------------------------
def _where(h, lab):
    return (lab[2] + ": " + (h.get("heading") or h.get("chapter") or "")) if h.get("ch") == "tour" else \
        "%s %s «%s»%s" % (lab[1], h.get("ch", ""), h.get("chapter", ""),
                          (" – " + " / ".join(x for x in (h.get("heading"), h.get("term")) if x))
                          if (h.get("heading") or h.get("term")) else "")


def prompt(q, hits, lang):
    lab = LABEL.get(lang, LABEL["en"])
    rows = []
    for i, h in enumerate(hits[:N_PASSAGES], 1):
        rows.append("[%d] %s\n%s" % (i, _where(h, lab), " ".join((h.get("text") or "").split())))
    user = "%s:\n\n%s\n\n%s: %s" % (lab[0], "\n\n".join(rows), lab[3], q.strip())
    return [{"role": "system", "content": SYSTEM.get(lang, SYSTEM["en"])}, {"role": "user", "content": user}]


def pick_sentences(hits, min_words=4):
    """Answer mode "extract": the numbered passage sentences the model may pick -> [(sentence, hit)]. Only WHOLE
    sentences (a capital / digit / quote first, brackets balanced - no lower-case clause fragments of the manual's
    "·"-lists) of at least min_words words."""
    out = []
    for h in hits[:N_PASSAGES]:
        for s in sentences(h.get("text") or ""):
            if len(s.split()) >= min_words and _whole(s):
                out.append((s, h))
    return out[:MAX_PICK_SENT]


def _ended(s):
    """A picked sentence spoken as one: a terminal mark at its end (the last sentence of a passage may lack it)."""
    s = s.strip()
    return s if s.endswith((".", "!", ";", "·", "?", ":")) else s + "."


def prompt_pick(q, hits, lang):
    lab = LABEL.get(lang, LABEL["en"])
    picks = pick_sentences(hits)
    rows, last = [], None
    for i, (s, h) in enumerate(picks, 1):
        if h is not last:
            rows.append("(%s)" % _where(h, lab))
            last = h
        rows.append("%d. %s" % (i, s))
    user = "%s:\n\n%s\n\n%s: %s" % (lab[4], "\n".join(rows), lab[3], q.strip())
    return [{"role": "system", "content": SYSTEM_PICK.get(lang, SYSTEM_PICK["en"])}, {"role": "user", "content": user}]


CANT_PLAIN = {k: plain(v).rstrip(".") for k, v in CANT.items()}
CANT_START = ("δεν βρισκω", "i can't find", "i cannot find", "δεν υπαρχει απαντηση", "the passages do not")


def clean(text):
    """The model's text -> at most MAX_SENT plain sentences (no markdown, no "Answer:" label)."""
    t = (text or "").replace("**", "").replace("__", "").replace("#", " ")
    t = re.sub(r"^\s*(απαντηση|απάντηση|answer)\s*:\s*", "", t.strip(), flags=re.I)
    t = re.sub(r"^\s*[-*•]\s*", "", t, flags=re.M)
    t = re.sub(r"^\s*\d{1,2}[.)]\s+", "", t, flags=re.M)     # a list number "1. " is not part of the answer
    t = re.sub(r"\[\d+\]", "", t)                       # passage markers are not spoken
    # an echoed passage label ("Κεφάλαιο 10 «ΣΥΝΑΓΕΡΜΟΙ» – ...", "Ξενάγηση: ...") is not an answer: dropped
    t = re.sub(r"^\s*(?:κεφάλαιο|κεφαλαιο|chapter)\s+\d+[A-Z]?\s*(?:«[^»]*»|\"[^\"]*\")\s*(?:[–-][^.\n]*)?[.:]?\s*",
               "", t, flags=re.I | re.M)
    s = [x for x in sentences(t) if not re.fullmatch(r"\d{1,2}[.)]?", x)]
    out = " ".join(s[:MAX_SENT]).strip()
    return out[:1].upper() + out[1:] if out[:1].islower() else out


def is_cant(text):
    p = plain(text).strip()
    return any(p.startswith(c) for c in CANT_START) or any(c in p for c in CANT_PLAIN.values())


def cite(hit, lang):
    if not hit:
        return ""
    if hit.get("ch") == "tour":
        return "%s: %s" % (LABEL.get(lang, LABEL["en"])[2], hit.get("heading") or hit.get("chapter") or "")
    return "%s %s «%s»" % (LABEL.get(lang, LABEL["en"])[1], hit.get("ch", ""), hit.get("chapter", ""))


def passage_of(hit, lang):
    """What the panel SHOWS with an answer: the full passage + its chapter (AM4-r: also after a failed check)."""
    if not hit:
        return None
    return {"text": " ".join((hit.get("text") or "").split()), "cite": cite(hit, lang), "id": hit.get("id", "")}


def source_hit(ans, hits):
    """The passage the answer's words come from most (its chapter is cited); the best one when nothing overlaps."""
    aw = set(re.findall(r"\w{4,}", plain(ans)))
    best, score = (hits[0] if hits else None), 0
    for h in hits:
        s = len(aw & set(re.findall(r"\w{4,}", plain(h.get("text") or ""))))
        if s > score:
            best, score = h, s
    return best


def bindings_env(py_dir):
    """Make the llama.cpp bindings importable (tools/get_llm.py's pip --target folder) + their CUDA DLLs findable.
    The bindings load their DLLs with the plain Windows search (PATH), so the folders go on PATH too."""
    dirs = []
    if py_dir:
        if py_dir not in sys.path:
            sys.path.insert(0, py_dir)
        dirs += [os.path.join(py_dir, "llama_cpp", "lib"), os.path.join(py_dir, "nvidia", "cuda_runtime", "bin")]
    for sp in list(sys.path):
        dirs.append(os.path.join(sp, "nvidia", "cublas", "bin"))       # the assistant Python's cuBLAS wheel
    if os.name != "nt":
        return
    for d in dirs:
        if os.path.isdir(d):
            try:
                os.add_dll_directory(d)
            except OSError:
                pass
            if d not in os.environ.get("PATH", ""):
                os.environ["PATH"] = d + os.pathsep + os.environ.get("PATH", "")


# --- the language models ------------------------------------------------------------------------------------------
class StubLlm:
    """Tests: no model. reply = a fixed text, "@first" (the best passage's first sentence), "@cant", or a function
    (messages -> text). Optional delay_s per answer (the slow / retrieval-only tests); finish "length" = the
    generation stopped at max_tokens (cut, AM4-r MED-3)."""
    kind = "stub"

    def __init__(self, reply="@first", delay_s=0.0, mode="gpu", finish="stop"):
        self.reply, self.delay_s, self.mode, self.layers, self.finish = reply, delay_s, mode, -1, finish
        self.calls = []

    def generate(self, messages, max_tokens=MAX_TOKENS, max_s=MAX_GEN_S, clock=time.monotonic):
        self.calls.append(messages)
        if self.delay_s:
            if self.delay_s >= max_s:
                return {"text": "", "tokens": 0, "cut": True, "first_ms": int(max_s * 1000)}
            time.sleep(self.delay_s)
        r = self.reply
        if callable(r):
            r = r(messages)
        elif r == "@first":
            body = messages[1]["content"].split("\n\n")[1].split("\n", 1)[1]
            r = first_sentence(body)
        elif r == "@cant":
            r = CANT["el"] if "Ερώτηση" in messages[1]["content"] else CANT["en"]
        return {"text": r, "tokens": len(r.split()), "cut": self.finish == "length", "first_ms": int(self.delay_s * 1000)}


class LlamaLlm:
    """llama.cpp through its Python bindings, IN this (the worker's) process: no exe, no server, no port. The model is
    a LOCAL .gguf file (a name / URL is refused - nothing is downloaded)."""
    kind = "llama"

    def __init__(self, model_path, layers=-1, n_ctx=3072, threads=None, mode="gpu"):
        if not isinstance(model_path, str) or not os.path.isabs(model_path) or not model_path.endswith(".gguf") \
                or not os.path.isfile(model_path):
            raise ValueError("the language model must be a local .gguf file: %r" % (model_path,))
        from llama_cpp import Llama                    # the bindings (tools/get_llm.py); never llama_cpp.server
        self.path, self.layers, self.mode, self.n_ctx = model_path, layers, mode, n_ctx
        self.llm = Llama(model_path=model_path, n_gpu_layers=layers, n_ctx=n_ctx, n_batch=512,
                         n_threads=threads or max(2, min(6, (os.cpu_count() or 4) - 2)), seed=0, verbose=False,
                         use_mmap=True)

    def generate(self, messages, max_tokens=MAX_TOKENS, max_s=MAX_GEN_S, clock=time.monotonic):
        """Streamed; stops at MAX_SENT sentences, max_tokens or max_s. cut = too slow (also a FIRST token later than
        max_s) OR stopped at max_tokens (finish_reason "length": the last sentence is unfinished - AM4-r MED-3);
        first_ms = the time to the first token (the prompt pass; the bindings cannot be interrupted inside it - the
        client's ANSWER_TIMEOUT_S bounds the worst case)."""
        t0 = clock()
        t_end = t0 + max_s
        out, n, cut, first = [], 0, False, None
        for ch in self.llm.create_chat_completion(messages=messages, temperature=0.0, top_p=1.0, top_k=1,
                                                  max_tokens=max_tokens, seed=0, stream=True):
            c0 = ch["choices"][0]
            d = c0.get("delta") or {}
            piece = d.get("content") or ""
            if piece:
                if first is None:
                    first = clock() - t0
                out.append(piece)
                n += 1
                if len(sentences("".join(out))) > MAX_SENT:
                    break
            if c0.get("finish_reason") == "length":
                cut = True
            if clock() > t_end:
                cut = True
                break
        return {"text": "".join(out), "tokens": n, "cut": cut, "first_ms": int(first * 1000) if first is not None
                else None}

    def close(self):
        try:
            self.llm.close()
        except Exception:
            pass
        self.llm = None


# --- the answer ---------------------------------------------------------------------------------------------------
def decide(raw, hits, lang, terms=(), cut=False, err=None, mode="generate"):
    """The model's raw text -> what is said: {"a", "src", "id", "cite", "passage", "problems", "raw", ...}. A cut
    generation keeps its COMPLETE sentences (post-checked like any answer); nothing complete = the retrieval form.
    Pure: the model comparison re-scores its stored raw texts with exactly this (tools/assistant_llm_eval.py
    rescore). mode "extract": raw = the numbers of the picked passage sentences (pick_sentences)."""
    if cut:
        whole = [s for s in sentences(raw or "") if s.rstrip().endswith((".", "!", ";", "·", "?"))]
        raw_used = " ".join(whole)
    else:
        raw_used = raw or ""
    out = {"raw": raw or "", "problems": [], "cut": bool(cut), "answer_mode": mode}
    if mode == "extract":
        return _decide_pick(raw_used, hits, lang, terms, out, err)
    txt = clean(raw_used)
    if not txt or err:
        h = hits[0]
        out.update(a=shortest_sentence(h.get("text", "")), src="retrieval", id=h.get("id", ""), cite=cite(h, lang),
                   passage=passage_of(h, lang), why=err or "the language model was too slow")
    elif is_cant(txt):
        out.update(a=CANT[lang], src="cant_find", model_text=txt, id="", cite="")
    else:
        probs = postcheck(txt, hits[:N_PASSAGES], terms, lang)
        if probs:                                    # AM4-r: can't find is SAID, the best passage is SHOWN (W105)
            h = hits[0]
            out.update(a=CANT[lang], src="passage", problems=probs, model_text=txt, id=h.get("id", ""),
                       cite=cite(h, lang), passage=passage_of(h, lang))
        else:
            h = source_hit(txt, hits[:N_PASSAGES])
            out.update(a=txt, src="llm", id=h.get("id", ""), cite=cite(h, lang), passage=passage_of(h, lang),
                       model_text=txt)
    return out


def _decide_pick(raw, hits, lang, terms, out, err):
    picks = pick_sentences(hits)
    if err or not (raw or "").strip():
        h = hits[0]
        out.update(a=shortest_sentence(h.get("text", "")), src="retrieval", id=h.get("id", ""), cite=cite(h, lang),
                   passage=passage_of(h, lang), why=err or "the language model was too slow")
        return out
    if is_cant(raw):
        out.update(a=CANT[lang], src="cant_find", model_text=raw.strip(), id="", cite="")
        return out
    # the picks: "4" / "4, 9" / "4 και 9", or the model echoing "4. <the sentence itself>" (accepted when the text
    # IS that sentence); any other words = not a pick: nothing is guessed
    body, echoed = raw, False
    lines = [ln.strip() for ln in raw.strip().splitlines() if ln.strip()]
    if lines and all(re.match(r"\d+[.)]\s+\S", ln) for ln in lines):
        nums = []
        for ln in lines:
            m = re.match(r"(\d+)[.)]\s+(.*)", ln)
            i, rest = int(m.group(1)), plain(m.group(2))[:40]
            if 1 <= i <= len(picks) and plain(picks[i - 1][0]).startswith(rest):
                nums.append(m.group(1))
        if nums and len(nums) == len(lines):
            body, echoed = ", ".join(nums), True
    ids = []
    for m in re.finditer(r"\d+", body):
        i = int(m.group(0))
        if 1 <= i <= len(picks) and i not in ids:
            ids.append(i)
    ids = ids[:MAX_SENT]
    letters = re.sub(r"(?<![\w])(?:και|and|η|or)(?![\w])", "", plain(body))
    letters = re.sub(r"[\d\s,.;:()\[\]-]+", "", letters)
    if not ids or letters:
        out.update(a=CANT[lang], src="passage", problems=["the model did not pick a sentence: %s" % raw[:60]],
                   model_text=raw.strip(), id=hits[0].get("id", ""), cite=cite(hits[0], lang),
                   passage=passage_of(hits[0], lang))
        return out
    if len(ids) == 2 and len(picks[ids[0] - 1][0].split()) + len(picks[ids[1] - 1][0].split()) > MAX_PICK_WORDS:
        ids = ids[:1]                               # two long manual sentences are too much to hear at once
    txt = " ".join(_ended(picks[i - 1][0]) for i in ids)
    h = picks[ids[0] - 1][1]
    out["echoed"] = echoed
    # verbatim manual sentences pass; kept as the backstop (a manual sentence's own "see chapter 9" is no problem)
    probs = [p for p in postcheck(txt, hits[:N_PASSAGES], terms, lang) if not p.startswith("chapter not")]
    if probs:
        out.update(a=CANT[lang], src="passage", problems=probs, model_text=txt, id=h.get("id", ""),
                   cite=cite(h, lang), passage=passage_of(h, lang))
    else:
        out.update(a=txt, src="extract", id=h.get("id", ""), cite=cite(h, lang), passage=passage_of(h, lang),
                   model_text=txt, picked=ids)
    return out


class Answerer:
    def __init__(self, llm=None, terms=(), clock=time.monotonic, retrieval_only=False, mode="generate"):
        self.llm = llm
        self.terms = set(terms)
        self.clock = clock
        self.retrieval_only = retrieval_only or llm is None
        self.why_ro = "" if llm is not None else "no language model"
        self.slow = []                                   # the last generation times (s)
        self.stick = True                                # False only for the model comparison (measures every answer)
        self.mode = mode if mode in MODES else "generate"

    def answer(self, q, lang, found):
        t0 = self.clock()
        lang = lang if lang in CANT else "en"
        hits = (found or {}).get("hits") or []
        r = {"q": q, "lang": lang, "problems": [], "gen_ms": 0, "tokens": 0, "id": "", "cite": "", "passage": None,
             "mode": "retrieval" if self.retrieval_only else getattr(self.llm, "mode", ""), "answer_mode": self.mode}
        if (found or {}).get("cant_find") or not hits:
            r.update(a=CANT[lang], src="cant_find")
        elif self.retrieval_only:
            h = hits[0]
            r.update(a=shortest_sentence(h.get("text", "")), src="retrieval", id=h.get("id", ""), cite=cite(h, lang),
                     passage=passage_of(h, lang), why=self.why_ro)
        else:
            g0 = self.clock()
            try:
                msgs = prompt_pick(q, hits, lang) if self.mode == "extract" else prompt(q, hits, lang)
                g = self.llm.generate(msgs)
            except Exception as e:                       # a broken model never leaves the question unanswered
                g = {"text": "", "tokens": 0, "cut": True, "err": str(e)[:120]}
            gen_s = self.clock() - g0
            r["gen_ms"], r["tokens"], r["first_ms"] = int(gen_s * 1000), g.get("tokens", 0), g.get("first_ms")
            self._timing(gen_s, g.get("cut"))
            r.update(decide(g.get("text", ""), hits, lang, self.terms, cut=g.get("cut"), err=g.get("err"),
                            mode=self.mode))
        r["ms"] = int((self.clock() - t0) * 1000)
        return r

    def _timing(self, gen_s, cut):
        """CPU too slow (the last SLOW_N generations all > SLOW_S, or cut) = RETRIEVAL-ONLY from now on."""
        self.slow = (self.slow + [gen_s if not cut else 1e9])[-SLOW_N:]
        if self.stick and len(self.slow) == SLOW_N and min(self.slow) > SLOW_S:
            self.retrieval_only = True
            self.why_ro = "the language model was too slow %d times (> %.0f s)" % (SLOW_N, SLOW_S)
