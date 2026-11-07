"""The talking manual's answer step (step AM4): the found passages -> 1-2 short spoken sentences, CHECKED.

    a = Answerer(llm, lang_terms)            llm = LlamaLlm(...) | StubLlm(...) | None (= retrieval only)
    a.answer(q, lang, found) -> {"a", "src", "mode", "cite", "id", "problems", "ms", "gen_ms", "tokens"}

src = what is said:
    "llm"        the model's answer passed the post-check
    "passage"    the model's answer FAILED the post-check (a name / number not in the passages) -> the best passage's
                 FIRST sentence instead (CLAUDE.md W, AM1-r M3)
    "retrieval"  no model / retrieval-only mode / the model too slow -> the best passage's SHORTEST full sentence
    "cant_find"  the search found nothing (index.Searcher's thresholds) OR the model said the passages don't answer
                 -> «Δεν βρίσκω απάντηση στο εγχειρίδιο.» / "I can't find that in the manual."

Grounding (docs/assistant_design.md 5): the model sees ONLY the question + the top passages (numbered, chapter +
heading) and a fixed instruction in the question's language; temperature 0; at most 2 sentences are kept.
POST-CHECK before anything is used: every NAME in the answer (a word in capitals, a «quoted» / "quoted" span, a
C2 name from the screens' capital texts) and every NUMBER - with its UNIT when it has one ("5 kn", "3 s", "50 m",
"30 %", "120 m", "10 min"; "πέντε δευτερόλεπτα" = "5 s") - must be in the passages; a chapter the answer names must
be one of theirs. The chapter / tour stop the answer comes from is added by THIS code (cite), never by the model.
Pure Python + the model object: no process, no network, no file written.
"""
import os
import re
import sys
import time
import unicodedata

CANT = {"el": "Δεν βρίσκω απάντηση στο εγχειρίδιο.", "en": "I can't find that in the manual."}
MAX_SENT = 2
MAX_TOKENS = 160          # 2 Greek sentences are ~40 words = ~120 tokens in these tokenizers
MAX_GEN_S = 10.0          # a longer generation is cut = this answer said from the passage (retrieval form)
SLOW_S = 8.0              # the last SLOW_N answers all slower = RETRIEVAL-ONLY mode (docs 5)
SLOW_N = 3
N_PASSAGES = 3
N_CTX = 3072              # 3 passages of <= 120 words + the instruction + the answer fit with room (Greek ~3 tokens / word)

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
LABEL = {"el": ("Αποσπάσματα του εγχειριδίου", "Κεφάλαιο", "Ξενάγηση", "Ερώτηση"),
         "en": ("Manual passages", "Chapter", "Guided tour", "Question")}


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
        "kn": "kn knots knot κομβοι κομβους κομβων κομβο κομβου",
        "s": "s sec secs δευτ δευτερολεπτα δευτερολεπτο δευτερολεπτου δευτερολεπτων seconds second",
        "ms": "ms",
        "min": "min mins λεπτα λεπτο λεπτου λεπτων λεπ minutes minute",
        "h": "h ωρα ωρες ωρων ωρεσ hours hour",
        "m": "m μ μετρα μετρο μετρων meters metres metre meter",
        "km": "km χιλιομετρα", "nm": "nm ναυτικα", "%": "% τοισ percent", "deg": "° μοιρεσ μοιρα μοιρων degrees deg",
        "V": "v βολτ volt volts", "Hz": "hz", "fps": "fps", "GB": "gb", "MB": "mb", "kbit": "kbit",
        "mAh": "mah", "A": "a αμπερ", "W": "w βατ"}.items():
    for w in words.split():
        UNITS[w] = canon
NUM_WORDS = {"el": {"δυο": 2, "τρια": 3, "τρεισ": 3, "τεσσερα": 4, "τεσσερισ": 4, "πεντε": 5, "εξι": 6, "επτα": 7,
                    "εφτα": 7, "οκτω": 8, "οχτω": 8, "εννεα": 9, "εννια": 9, "δεκα": 10, "δωδεκα": 12,
                    "δεκαπεντε": 15, "εικοσι": 20, "τριαντα": 30, "σαραντα": 40, "πενηντα": 50, "εξηντα": 60,
                    "εκατο": 100, "ενα": 1, "μια": 1, "μισο": 0.5},
             "en": {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8,
                    "nine": 9, "ten": 10, "twelve": 12, "fifteen": 15, "twenty": 20, "thirty": 30, "forty": 40,
                    "fifty": 50, "sixty": 60, "hundred": 100, "half": 0.5}}
_NUMW = {w: v for d in NUM_WORDS.values() for w, v in d.items()}
NUM = r"(\d+(?:[.,]\d+)?)"
NUM_RE = re.compile(r"(?<![\w.,])" + NUM + r"(?:\s*(?:-|–|\.\.\.?|…|εωσ|ωσ|to|until)\s*" + NUM +
                    r")?(?:\s*(%|°)|\s*([^\W\d_]+)\.?)?")
WORDNUM_RE = re.compile(r"(?<!\w)(" + "|".join(sorted(_NUMW, key=len, reverse=True)) + r")\s+([^\W\d_]+|%)")
CHAPTER_RE = re.compile(r"(?:κεφαλαιο|κεφαλαιου|κεφ\.?|chapter|ch\.)\s*(\d+[a-z]?)")


def _f(x):
    return float(x.replace(",", "."))


def numbers(text):
    """[(value, unit or None)] of a text: digits (a range gives its unit to both ends) and number WORDS followed by a
    unit ("πέντε δευτερόλεπτα"; a bare "ένα" is an article, never a number). Chapter references are left out."""
    t = CHAPTER_RE.sub(" ", plain(text))
    out = []
    for m in NUM_RE.finditer(t):
        unit = m.group(3) or UNITS.get(m.group(4) or "")
        unit = UNITS.get(unit, unit)
        for g in (m.group(1), m.group(2)):
            if g:
                out.append((_f(g), unit))
    for m in WORDNUM_RE.finditer(t):
        unit = UNITS.get(m.group(2))
        if unit:
            out.append((float(_NUMW[m.group(1)]), unit))
    return out


def chapters_named(text):
    return set(CHAPTER_RE.findall(plain(text)))


# --- names --------------------------------------------------------------------------------------------------------
CAPS_RE = re.compile(r"(?<![\w])([A-ZΑ-ΩΆΈΉΊΌΎΏΪΫ][A-ZΑ-ΩΆΈΉΊΌΎΏΪΫ0-9]+(?:[ /-][A-ZΑ-ΩΆΈΉΊΌΎΏΪΫ0-9]{2,})*)(?![\w])")
QUOTE_RE = re.compile(r"«([^»]{2,60})»|\"([^\"]{2,60})\"|“([^”]{2,60})”")
ALLOWED_CAPS = {"c2", "trygons", "pc", "ok"}        # the product's own name / the plain abbreviations


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


def _wordy(text):
    """plain text with only word characters, single spaces, padded (whole-word 'in' tests)."""
    return " " + " ".join(re.findall(r"[\w%°]+", plain(text))) + " "


def names(text, terms=()):
    """The names an answer uses: words in capitals (+ their runs), quoted spans, known C2 terms (plain form)."""
    out = set()
    for m in CAPS_RE.finditer(text):
        n = " ".join(plain(m.group(1)).split())
        if n not in ALLOWED_CAPS:
            out.add(n)
    for m in QUOTE_RE.finditer(text):
        out.add(" ".join(plain(next(g for g in m.groups() if g)).split()))
    w = _wordy(text)
    for t in terms:
        tw = " ".join(re.findall(r"[\w%°]+", t))
        if tw and (" " + tw + " ") in w:
            out.add(t)
    return out


def postcheck(ans, hits, terms=()):
    """[] = every name / number(+unit) / chapter of the answer is in the passages; else the problems found."""
    src = " \n ".join("%s %s %s %s" % (h.get("chapter", ""), h.get("heading", ""), h.get("term", ""), h.get("text", ""))
                      for h in hits)
    sw = _wordy(src)
    probs = []
    for n in sorted(names(ans, terms)):
        nw = " ".join(re.findall(r"[\w%°]+", n))
        if nw and (" " + nw + " ") not in sw:
            probs.append("name not in the passages: %s" % n)
    have = numbers(src)
    vals = {v for v, _u in have}
    for v, u in numbers(ans):
        if u is None:
            if v not in vals:
                probs.append("number not in the passages: %g" % v)
        elif (v, u) not in have:
            probs.append("number + unit not in the passages: %g %s" % (v, u))
    chs = {str(h.get("ch", "")).lower() for h in hits}
    for c in chapters_named(ans):
        if c not in chs:
            probs.append("chapter not among the passages: %s" % c)
    return probs


# --- the prompt ---------------------------------------------------------------------------------------------------
def prompt(q, hits, lang):
    lab = LABEL.get(lang, LABEL["en"])
    rows = []
    for i, h in enumerate(hits[:N_PASSAGES], 1):
        where = (lab[2] + ": " + (h.get("heading") or h.get("chapter") or "")) if h.get("ch") == "tour" else \
            "%s %s «%s»%s" % (lab[1], h.get("ch", ""), h.get("chapter", ""),
                              (" – " + " / ".join(x for x in (h.get("heading"), h.get("term")) if x))
                              if (h.get("heading") or h.get("term")) else "")
        rows.append("[%d] %s\n%s" % (i, where, " ".join((h.get("text") or "").split())))
    user = "%s:\n\n%s\n\n%s: %s" % (lab[0], "\n\n".join(rows), lab[3], q.strip())
    return [{"role": "system", "content": SYSTEM.get(lang, SYSTEM["en"])}, {"role": "user", "content": user}]


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
    (messages -> text). Optional delay_s per answer (the slow / retrieval-only tests)."""
    kind = "stub"

    def __init__(self, reply="@first", delay_s=0.0, mode="gpu"):
        self.reply, self.delay_s, self.mode, self.layers = reply, delay_s, mode, -1
        self.calls = []

    def generate(self, messages, max_tokens=MAX_TOKENS, max_s=MAX_GEN_S, clock=time.monotonic):
        self.calls.append(messages)
        if self.delay_s:
            if self.delay_s >= max_s:
                return {"text": "", "tokens": 0, "cut": True}
            time.sleep(self.delay_s)
        r = self.reply
        if callable(r):
            r = r(messages)
        elif r == "@first":
            body = messages[1]["content"].split("\n\n")[1].split("\n", 1)[1]
            r = first_sentence(body)
        elif r == "@cant":
            r = CANT["el"] if "Ερώτηση" in messages[1]["content"] else CANT["en"]
        return {"text": r, "tokens": len(r.split()), "cut": False}


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
        """Streamed; stops at MAX_SENT sentences, max_tokens or max_s (cut = too slow)."""
        t_end = clock() + max_s
        out, n, cut = [], 0, False
        for ch in self.llm.create_chat_completion(messages=messages, temperature=0.0, top_p=1.0, top_k=1,
                                                  max_tokens=max_tokens, seed=0, stream=True):
            d = ch["choices"][0].get("delta") or {}
            piece = d.get("content") or ""
            if piece:
                out.append(piece)
                n += 1
                if len(sentences("".join(out))) > MAX_SENT:
                    break
            if clock() > t_end:
                cut = True
                break
        return {"text": "".join(out), "tokens": n, "cut": cut}

    def close(self):
        try:
            self.llm.close()
        except Exception:
            pass
        self.llm = None


# --- the answer ---------------------------------------------------------------------------------------------------
def decide(raw, hits, lang, terms=(), cut=False, err=None):
    """The model's raw text -> what is said: {"a", "src", "id", "cite", "problems", "raw", ...}. A cut generation
    keeps its COMPLETE sentences (post-checked like any answer); nothing complete = the retrieval form. Pure: the
    model comparison re-scores its stored raw texts with exactly this (tools/assistant_llm_eval.py rescore)."""
    if cut:
        whole = [s for s in sentences(raw or "") if s.rstrip().endswith((".", "!", ";", "·", "?"))]
        raw_used = " ".join(whole)
    else:
        raw_used = raw or ""
    txt = clean(raw_used)
    out = {"raw": raw or "", "problems": [], "cut": bool(cut)}
    if not txt or err:
        h = hits[0]
        out.update(a=shortest_sentence(h.get("text", "")), src="retrieval", id=h.get("id", ""), cite=cite(h, lang),
                   why=err or "the language model was too slow")
    elif is_cant(txt):
        out.update(a=CANT[lang], src="cant_find", model_text=txt, id="", cite="")
    else:
        probs = postcheck(txt, hits[:N_PASSAGES], terms)
        if probs:
            h = hits[0]
            out.update(a=first_sentence(h.get("text", "")), src="passage", problems=probs, model_text=txt,
                       id=h.get("id", ""), cite=cite(h, lang))
        else:
            h = source_hit(txt, hits[:N_PASSAGES])
            out.update(a=txt, src="llm", id=h.get("id", ""), cite=cite(h, lang), model_text=txt)
    return out


class Answerer:
    def __init__(self, llm=None, terms=(), clock=time.monotonic, retrieval_only=False):
        self.llm = llm
        self.terms = set(terms)
        self.clock = clock
        self.retrieval_only = retrieval_only or llm is None
        self.why_ro = "" if llm is not None else "no language model"
        self.slow = []                                   # the last generation times (s)
        self.stick = True                                # False only for the model comparison (measures every answer)

    def answer(self, q, lang, found):
        t0 = self.clock()
        lang = lang if lang in CANT else "en"
        hits = (found or {}).get("hits") or []
        r = {"q": q, "lang": lang, "problems": [], "gen_ms": 0, "tokens": 0, "id": "", "cite": "",
             "mode": "retrieval" if self.retrieval_only else getattr(self.llm, "mode", "")}
        if (found or {}).get("cant_find") or not hits:
            r.update(a=CANT[lang], src="cant_find")
        elif self.retrieval_only:
            h = hits[0]
            r.update(a=shortest_sentence(h.get("text", "")), src="retrieval", id=h.get("id", ""), cite=cite(h, lang),
                     why=self.why_ro)
        else:
            g0 = self.clock()
            try:
                g = self.llm.generate(prompt(q, hits, lang))
            except Exception as e:                       # a broken model never leaves the question unanswered
                g = {"text": "", "tokens": 0, "cut": True, "err": str(e)[:120]}
            gen_s = self.clock() - g0
            r["gen_ms"], r["tokens"] = int(gen_s * 1000), g.get("tokens", 0)
            self._timing(gen_s, g.get("cut"))
            r.update(decide(g.get("text", ""), hits, lang, self.terms, cut=g.get("cut"), err=g.get("err")))
        r["ms"] = int((self.clock() - t0) * 1000)
        return r

    def _timing(self, gen_s, cut):
        """CPU too slow (the last SLOW_N generations all > SLOW_S, or cut) = RETRIEVAL-ONLY from now on."""
        self.slow = (self.slow + [gen_s if not cut else 1e9])[-SLOW_N:]
        if self.stick and len(self.slow) == SLOW_N and min(self.slow) > SLOW_S:
            self.retrieval_only = True
            self.why_ro = "the language model was too slow %d times (> %.0f s)" % (SLOW_N, SLOW_S)
