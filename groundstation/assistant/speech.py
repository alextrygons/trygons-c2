"""The speech normaliser of the talking manual (step AM5, CLAUDE.md W): what the station's voice SAYS for an
answer. Pure text -> text, no sound, no process, no file (the station's tts.py + sound.py make and play the WAV).

  spoken(info, lang) -> (text, kind)   the whole spoken answer: "from the manual, chapter N" + the answer
  normalize(text, lang, vocab)         the answer text as the voice should read it

Rules (owner I1.4, docs/assistant_design.md 6):
  * CAPITALS -> normal case: the screens' «ΚΑΤΑΠΑΚΤΗ» is read as «καταπακτή» - the accented form comes from the
    retrieved passages' own words (vocab_from), else the plain lower case;
  * acronyms are spoken like the "say." lines of the lang files (GPS = «τζι πι ες», 4G = «τέσσερα τζι», STANDBY =
    «στάντμπάι» ...; English: letters with spaces), an unknown Latin capital word is spelt letter by letter in
    Greek and lower-cased in English;
  * numbers keep their digits (the voices read them), their UNITS are spoken ("5 kn" = «5 κόμβοι», "50 %" =
    «50 τοις εκατό», "120 m" = «120 μέτρα»), ranges "10..120 m" = «10 έως 120 μέτρα», "->" = «προς», "<=" = «το
    πολύ», ">=" = «τουλάχιστον», "±" = «πλην ή συν», a " / " = a pause; a THOUSANDS group "1.500 m" / "1,500 m"
    (AM4-r3 LOW-d: the checker takes it as 1500, answer._f) is read as "1500 m", never as "one point five hundred";
  * the chapter is spoken with EVERY answer: «Από το εγχειρίδιο, κεφάλαιο 10.» / "From the manual, chapter 10.";
    two chapters (a pick from two passages, AM4-r2 LOW-a) = «κεφάλαια 10 και 12»; a tour stop = «Από την
    ξενάγηση.»;
  * a LONG pick (AM4-r2 review MED-3: whole manual sentences can run to 100+ words) is never read out: above
    LONG_WORDS words only «Βρήκα την απάντηση στο κεφάλαιο 10 – δείτε το κείμενο.» is said and the panel shows
    the full text (kind "long");
  * «Δεν βρίσκω απάντηση στο εγχειρίδιο.» is said as it is (kind "cant_find"); no answer text = nothing (kind "").
"""
import re
import unicodedata

LONG_WORDS = 50           # a spoken answer longer than this = the chapter intro only (the text is on the panel)

# acronym -> (Greek as the "say." lines spell it, English). The Greek spellings follow lang/el.txt's "say." lines
# where one exists (GPS = say.alarm.gps_lost, 4G = say.link.4g, STANDBY = say.mode.standby): test AS26 checks them.
SAID = {
    "GPS": ("τζι πι ες", "G P S"), "AIS": ("έι άι ες", "A I S"), "UAV": ("γιου έι βι", "U A V"),
    "UAVS": ("γιου έι βι", "U A Vs"), "DP": ("ντι πι", "D P"), "HDG LOCK": ("κλείδωμα πορείας", "heading lock"),
    "HDG": ("χέντινγκ", "heading"), "AUTO": ("ώτο", "auto"), "STANDBY": ("στάντμπάι", "standby"),
    "REMOTE": ("ριμόουτ", "remote"), "MANUAL": ("μάνιουαλ", "manual"), "MANEUVER": ("μανούβρα", "maneuver"),
    "TRAINING": ("τρέινινγκ", "training"), "USB": ("γιου ες μπι", "U S B"), "PC": ("πι σι", "P C"),
    "C2": ("σι δύο", "C two"), "SAVE": ("σέιβ", "save"), "KEEP": ("κιπ", "keep"), "HELM": ("χελμ", "helm"),
    "RF": ("αρ εφ", "R F"), "4G": ("τέσσερα τζι", "four G"), "5G": ("πέντε τζι", "five G"),
    "STARLINK": ("στάρλινκ", "Starlink"), "AI": ("έι άι", "A I"), "VHF": ("βι έιτς εφ", "V H F"),
    "CONFIRM": ("κονφέρμ", "confirm"), "ACCEPT": ("αξέπτ", "accept"), "REJECT": ("ριτζέκτ", "reject"),
    "TAKE": ("τέικ", "take"), "STOP": ("στοπ", "stop"), "OK": ("οκέι", "okay"), "AURIX": ("ώρικς", "Aurix"),
    "CPA": ("σι πι έι", "C P A"), "TCPA": ("τι σι πι έι", "T C P A"), "UTC": ("γιου τι σι", "U T C"),
    "ID": ("άι ντι", "I D"), "IP": ("άι πι", "I P"), "SOG": ("ταχύτητα εδάφους", "speed over ground"),
    "COG": ("πορεία εδάφους", "course over ground"), "REC": ("ρεκ", "rec"), "SCR": ("σκριν", "screen"),
    "AR": ("έι αρ", "A R"), "HUD": ("χαντ", "hud"), "LINK TIME": ("λινκ τάιμ", "link time"),
    "LINK": ("λινκ", "link"), "DRIVER DECISION": ("ντράιβερ ντισίζιον", "driver decision"),
    "ENGINE KILL": ("έντζιν κιλ", "engine kill"), "SPOT": ("σποτ", "spot"), "JOG": ("τζογκ", "jog"),
    "CRUISE": ("κρουζ", "cruise"), "TRIP": ("τριπ", "trip"), "CLEAR SLOWDOWN": ("κλίαρ σλόουντάουν", "clear slowdown"),
    "RESTORE POWER": ("ριστόρ πάουερ", "restore power"), "TALK": ("τοκ", "talk"), "ASK": ("ασκ", "ask"),
    "FWD": ("εμπρός", "forward"), "REV": ("όπισθεν", "reverse"), "N": ("νεκρά", "neutral"),
    "FTDI": ("εφ τι ντι άι", "F T D I"), "ELRS": ("ι ελ αρ ες", "E L R S"), "VRX": ("βι αρ εξ", "V R X"),
    "VTX": ("βι τι εξ", "V T X"), "MAVLINK": ("μάβλινκ", "MAVLink"), "LAND": ("λαντ", "land"), "RTL": ("αρ τι ελ", "R T L"),
    "GUIDED": ("γκάιντεντ", "guided"), "LOITER": ("λόιτερ", "loiter"), "EKF": ("ι κέι εφ", "E K F"),
    "BNO085": ("μπι εν όου 085", "B N O 085"), "TC375": ("τι σι 375", "T C 375"), "NMEA": ("εν εμ ι έι", "N M E A"),
    "SD": ("ες ντι", "S D"), "NV": ("εν βι", "N V"), "TTL": ("τι τι ελ", "T T L"), "RS485": ("αρ ες 485", "R S 485"),
    "RS232": ("αρ ες 232", "R S 232"), "CAN": ("καν", "CAN"), "PWM": ("πι ντάμπλγιου εμ", "P W M"),
    "DSPR": ("ντι ες πι αρ", "D S P R"), "CSA": ("σι ες έι", "C S A"), "EN": ("αγγλικά", "English"),
    "EL": ("ελληνικά", "Greek"), "NIGHT": ("νάιτ", "night"), "LAYOUT": ("λέιαουτ", "layout"), "BENCH": ("μπεντς", "bench"),
    "FIRST TEST": ("φερστ τεστ", "first test"), "GEAR FAULT": ("γκίαρ φολτ", "gear fault"), "HATCH FAULT": ("χατς φολτ", "hatch fault"),
    "FLY": ("φλάι", "fly"), "KPI": ("κέι πι άι", "K P I"), "GB": ("γκιγκαμπάιτ", "gigabytes"), "MB": ("μεγκαμπάιτ", "megabytes"),
    "CRC": ("σι αρ σι", "C R C"), "ADC": ("έι ντι σι", "A D C"), "CPU": ("σι πι γιου", "C P U"), "GPU": ("τζι πι γιου", "G P U"),
    "RAM": ("ραμ", "RAM"), "SSD": ("ες ες ντι", "S S D"), "LTE": ("ελ τι ι", "L T E"), "LAN": ("λαν", "LAN"), "WIFI": ("γουάι φάι", "wifi"),
}
# the Greek names of the Latin letters (an unknown capital Latin word is spelt out)
LETTERS_EL = {"A": "έι", "B": "μπι", "C": "σι", "D": "ντι", "E": "ι", "F": "εφ", "G": "τζι", "H": "έιτς", "I": "άι",
              "J": "τζέι", "K": "κέι", "L": "ελ", "M": "εμ", "N": "εν", "O": "όου", "P": "πι", "Q": "κιού", "R": "αρ",
              "S": "ες", "T": "τι", "U": "γιου", "V": "βι", "W": "ντάμπλγιου", "X": "εξ", "Y": "γουάι", "Z": "ζετ"}
# units after a number: unit -> (Greek, English)
UNITS = {"kn": ("κόμβοι", "knots"), "knots": ("κόμβοι", "knots"), "m/s": ("μέτρα το δευτερόλεπτο", "metres per second"),
         "km/h": ("χιλιόμετρα την ώρα", "kilometres per hour"), "m": ("μέτρα", "metres"), "km": ("χιλιόμετρα", "kilometres"),
         "nm": ("ναυτικά μίλια", "nautical miles"), "s": ("δευτερόλεπτα", "seconds"), "sec": ("δευτερόλεπτα", "seconds"),
         "ms": ("χιλιοστά του δευτερολέπτου", "milliseconds"), "min": ("λεπτά", "minutes"), "h": ("ώρες", "hours"),
         "%": ("τοις εκατό", "per cent"), "°": ("μοίρες", "degrees"), "deg": ("μοίρες", "degrees"), "V": ("βολτ", "volts"),
         "A": ("αμπέρ", "amperes"), "Hz": ("χερτζ", "hertz"), "kg": ("κιλά", "kilograms"), "cm": ("εκατοστά", "centimetres"),
         "mm": ("χιλιοστά", "millimetres"), "GB": ("γκιγκαμπάιτ", "gigabytes"), "MB": ("μεγκαμπάιτ", "megabytes"),
         "kbit/s": ("κιλομπίτ το δευτερόλεπτο", "kilobits per second"), "fps": ("καρέ το δευτερόλεπτο", "frames per second"),
         "W": ("βατ", "watts"), "mAh": ("μιλιαμπερώρια", "milliampere hours")}
_UNIT_RE = re.compile(r"(?<![\w.,])(-?\d+(?:[.,]\d+)?)\s?(" + "|".join(sorted((re.escape(u) for u in UNITS), key=len,
                                                                            reverse=True)) + r")(?![\w/])")
_RANGE_RE = re.compile(r"(\d+(?:[.,]\d+)?)\s*(?:\.\.\.?|…|–|-)\s*(\d+(?:[.,]\d+)?)(?=\s?(?:" +
                       "|".join(re.escape(u) for u in UNITS) + r")(?![\w/]))")
_ACRO_RE = re.compile(r"(?<![\w])(" + "|".join(sorted((re.escape(k) for k in SAID), key=len, reverse=True)) +
                      r")(?![\w])")
_CAP_LATIN_RE = re.compile(r"(?<![\w])([A-Z]{2,6})(?![\w])")
_CAP_GREEK_RE = re.compile(r"(?<![\w])([Α-ΩΆΈΉΊΌΎΏΪΫ]{2,})(?![\w])")
_FKEY_RE = re.compile(r"(?<![\w])F(\d{1,2})(?![\w])")
_THOUSANDS_RE = re.compile(r"(?<![\w.,])([1-9]\d{0,2})[.,](\d{3})(?![\d.,])")   # exactly 3 digits after the separator
GREEK_RE = re.compile(r"[Ͱ-Ͽἀ-῿]")


def strip_accents(s):
    t = unicodedata.normalize("NFD", s)
    return "".join(c for c in t if unicodedata.category(c) != "Mn")


def vocab_from(texts):
    """The accented lower-case forms of the passages' words: {ACCENTLESS UPPER: accented lower}. A capital word of
    the answer («ΚΑΤΑΠΑΚΤΗ») is read with the passage's own accent («καταπακτή»); several forms = the most frequent."""
    seen = {}
    for t in texts or ():
        for w in re.findall(r"[^\W\d_]{2,}", t or ""):
            if w.isupper() or not GREEK_RE.search(w):
                continue
            key = strip_accents(w).upper()
            low = w.lower()
            seen.setdefault(key, {})
            seen[key][low] = seen[key].get(low, 0) + 1
    return {k: max(v, key=lambda x: (v[x], x)) for k, v in seen.items()}


def _lower_el(w):
    w = w.lower()
    return (w[:-1] + "ς") if w.endswith("σ") and len(w) > 1 else w


def normalize(text, lang="el", vocab=None):
    """The answer text as the voice should read it (see the module doc)."""
    i = 0 if lang == "el" else 1
    t = (text or "").replace("«", " ").replace("»", " ").replace("\"", " ").replace("*", " ").replace("_", " ")
    t = t.replace("…", "...")
    t = re.sub(r"\[\d+\]", " ", t)
    t = _THOUSANDS_RE.sub(r"\1\2", t)            # "1.500 m" / "1,500 m" = 1500 (as answer._f reads it), spoken so
    # ranges before the unit, then the unit itself
    t = _RANGE_RE.sub(lambda m: "%s %s %s" % (m.group(1), "έως" if i == 0 else "to", m.group(2)), t)
    t = _UNIT_RE.sub(lambda m: "%s %s" % (m.group(1), UNITS[m.group(2)][i]), t)
    t = t.replace("+-", "±").replace("±", " %s " % ("πλην ή συν" if i == 0 else "plus or minus"))
    t = t.replace("<=", " %s " % ("το πολύ" if i == 0 else "at most")).replace(">=", " %s " % (
        "τουλάχιστον" if i == 0 else "at least"))
    t = t.replace("->", " %s " % ("προς" if i == 0 else "to")).replace("→", " %s " % ("προς" if i == 0 else "to"))
    t = re.sub(r"\s/\s", ", ", t)
    t = _FKEY_RE.sub(lambda m: ("εφ %s" if i == 0 else "F %s") % m.group(1), t)
    t = _ACRO_RE.sub(lambda m: SAID[m.group(1)][i], t)
    if i == 0:
        t = _CAP_LATIN_RE.sub(lambda m: " ".join(LETTERS_EL.get(c, c) for c in m.group(1)), t)
    else:
        t = _CAP_LATIN_RE.sub(lambda m: m.group(1).lower(), t)
    v = vocab or {}
    t = _CAP_GREEK_RE.sub(lambda m: v.get(strip_accents(m.group(1)).upper()) or _lower_el(m.group(1)), t)
    t = re.sub(r"\s+", " ", t).strip()
    t = re.sub(r"\s+([,.;:!?·])", r"\1", t)
    if t and t[-1] not in ".!?;":
        t += "."
    return t


# --- the chapter -----------------------------------------------------------------------------------------------------
_CH_RE = re.compile(r"(?:Κεφάλαιο|Chapter)\s+(\d+[A-Za-z]?)")
_TOUR = ("Ξενάγηση", "Tour", "Guided tour")      # answer.LABEL[lang][2]: the cite's label of a tour stop
INTRO = {"el": ("Από το εγχειρίδιο, κεφάλαιο %s.", "Από το εγχειρίδιο, κεφάλαια %s και %s.", "Από την ξενάγηση."),
         "en": ("From the manual, chapter %s.", "From the manual, chapters %s and %s.", "From the guided tour.")}
LONG = {"el": "Βρήκα την απάντηση στο κεφάλαιο %s – δείτε το κείμενο.",
        "en": "I found the answer in chapter %s – see the text."}
LONG_TOUR = {"el": "Βρήκα την απάντηση στην ξενάγηση – δείτε το κείμενο.",
             "en": "I found the answer in the guided tour – see the text."}


def chapters_of(cite):
    """The chapter numbers named in an answer's cite ("Κεφάλαιο 10 «..» + Κεφάλαιο 12 «..»"); "13A" -> "13 A"."""
    out = []
    for c in _CH_RE.findall(cite or ""):
        m = re.match(r"(\d+)([A-Za-z]?)", c)
        out.append((m.group(1) + " " + m.group(2).upper()).strip())
    return out


def intro(cite, lang):
    """The spoken chapter line of an answer (every spoken answer starts with it)."""
    lg = lang if lang in INTRO else "en"
    chs = chapters_of(cite)
    if len(chs) >= 2:
        return INTRO[lg][1] % (chs[0], chs[1])
    if chs:
        return INTRO[lg][0] % chs[0]
    if cite and cite.split(":")[0].strip() in _TOUR:
        return INTRO[lg][2]
    return ""


def spoken(info, lang=None):
    """-> (the whole spoken text, kind): kind "answer" (intro + the normalised answer), "long" (the intro-only line of
    a long pick, LONG_WORDS), "cant_find" (the can't-find sentence alone), "" (nothing to say)."""
    lang = lang or info.get("lang") or "el"
    lg = lang if lang in INTRO else "en"
    a = (info.get("a") or "").strip()
    if not a:
        return "", ""
    if info.get("src") in ("cant_find", "passage") or not info.get("cite"):
        return normalize(a, lg), "cant_find"
    cite = info.get("cite") or ""
    if len(a.split()) > LONG_WORDS:
        chs = chapters_of(cite)
        return (LONG[lg] % chs[0]) if chs else LONG_TOUR[lg], "long"
    texts = [(info.get("passage") or {}).get("text") or "", (info.get("passage2") or {}).get("text") or ""]
    texts += [h.get("text") or "" for h in ((info.get("found") or {}).get("hits") or [])]
    return (intro(cite, lg) + " " + normalize(a, lg, vocab_from(texts))).strip(), "answer"
