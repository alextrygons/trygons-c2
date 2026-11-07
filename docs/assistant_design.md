# Talking manual - design (step AM1, 2026-11-07)

Source of truth for the rules: CLAUDE.md section W "Talking manual". This file holds the architecture,
the choices and their reasons, the model comparison plan, the PC load budget and the tests. Owner spec:
docs/owner_findings_2026-11-06.md section I (I0 ground rules, I1 pipeline, I2 question list). Steps
AM2-AM6: PROGRESS.md. Open questions: PROGRESS.md "WAITING FOR OWNER" W81-W83.

Everything marked TO CONFIRM is a default the owner still confirms; MEASURE = a number found in a later
step on the owner's laptop and written here + in the manuals, never guessed in code.

## 0. Scope (binding)

- The operator (captain or copilot - e.g. while flying a UAV with both hands on the joystick) ASKS BY
  VOICE about the C2 itself ("πού κλείνω την καταπακτή;", "τι σημαίνει HATCH FAULT;") and HEARS a short
  answer (1-2 sentences) taken from the manual, with the full passage shown on the screen.
- STATION ONLY: groundstation/ + tools/ + docs. NO app/, hal/, gateway/ logic or AURIX change, no new
  frame, no gateway message, no TELEM field, no alarm bit.
- READ-ONLY: it never sends anything to the boat or the gateway, never changes a mode / setting /
  layout, never presses anything. It only listens (its own microphone), reads files, speaks and shows.
- Manual questions only (W82 default): it does NOT read live telemetry ("what mode am I in?" = the
  "can't find" answer or a pointer to the screen). Live answers = a possible later step with its own
  review.
- Fully OFFLINE: no internet, ever. Every model is a file in tools/bin/ (downloaded once by a pinned
  tools/get_*.py with sha256, never in git).

## 1. Architecture

```
 station process (tkinter, app.py)                     assistant worker (OWN process, below normal)
 ---------------------------------                     ------------------------------------------
 PTT: «ΕΡΩΤΗΣΗ» button / F9 / pedal (= F9)
   press  -> {"op":"listen"}          ---- stdin ---->  mic capture (own device setting)
   release-> {"op":"ask","lang"}      ---- JSON  ---->  1 STT  (Whisper)       -> question text
                                                        2 FIND (index + BM25)  -> top 3 passages
                                                        3 ANSWER (llama.cpp)   -> 1-2 sentences
                                                        3b post-check          -> answer / passage / can't find
 panel: question, answer, passage, chapter  <- stdout - {"op":"answer", "q", "a", "passages", "mode", "ms"}
 4 SPEAK: sound.py (assistant priority = lowest) -> tts.py (Piper / Windows voice) + speech normaliser
 6 LOG: groundstation/assistant/log/ (THIS PC only, text only)
```

- groundstation/assistant/ (package; groundstation/ is already excluded from the ADS build):
  - `client.py` - the station side: starts / stops / restarts the worker, PTT state machine, JSON
    lines, timeouts; the ONLY assistant module app.py imports.
  - `worker.py` - the worker process main (like gateway/vision/worker.py: JSON lines on stdin /
    stdout, ends when stdin closes, `low_priority()` = BELOW_NORMAL_PRIORITY_CLASS on Windows /
    nice 10 on Linux, limited CPU threads). Below normal, not IDLE: the answer has a latency target
    and most work runs on the GPU anyway; the station's windows keep normal priority.
  - `stt.py` (Whisper wrapper), `index.py` (manual -> sections, normalisation, BM25), `answer.py`
    (prompt, llama.cpp call, post-check, can't-find, retrieval-only), `speech.py` (speech normaliser),
    `gpu.py` (free GPU memory guard), `panel.py` (the slim panel), `page.py` (Tools page «Βοηθός»),
    `log.py` (local log).
- The worker loads the models ONCE at start (warm) and keeps them; it is started with the station when
  the setting `assistant` is on (default on for every station PC: W81) and the models exist; missing
  models = the button greyed + "models not installed" on the Tools page (never an error box).
- Speaking stays in the STATION process: the worker returns text only; the station speaks it through
  its own sound.py / tts.py, so the alarm priority lives in ONE scheduler (section 5).
- Crash / hang (no answer in 30 s - AM4: 60 s -, process gone) = station-local notice «ΒΟΗΘΟΣ ΕΚΤΟΣ» (the 13n1
  notice style, MasterLamps-free: never an alarm, never a sound) + restart after 5 s, at most 3 restarts
  in 10 min, then off until the station restarts (notice stays).

## 2. Read-only boundary (the proof, like academy AC1-AC3)

- No assistant module imports a network / serial module or a station command sender: `socket`,
  `asyncio` network use, `serial`, `gwclient`, `link`, `proto` writers, `stations`, `app` (the panel gets
  a narrow `view` object from app.py: root, theme, language, sound - never `app`).
- `client.py` may use `subprocess` (to start the worker) - the ONLY process door, like simguard.py;
  `worker.py` may start NO process (llama.cpp / Whisper run in-process through their Python bindings).
  AM1-r L4: client.py is also the ONLY module that may start the llama.cpp process: if AM4 picks the
  exe instead of the bindings, it is `llama-cli` started by client.py and spoken to over a pipe
  (stdin / stdout) - NEVER a TCP port, NEVER `llama-server` (AM4 extends AS1 / AS2 to check both names).
- Tests (AM2, extended every step): AS1 static imports of every assistant module; AS2 the transitive
  import graph through groundstation/ (nothing reaches link / gwclient / proto encoders / app.send);
  AS3 runtime: a whole question through the stub worker with `socket.socket`, `app.send*`,
  `gwclient.*.send*`, `link.*` patched to raise - the answer still comes; AS4 the network blocked
  (socket patched to raise) while the REAL models answer (N/A when the models are not downloaded).
- AM2-r (review of fb97ac2): MED-2 the worker-side blocker RECORDS every attempt (a line in a marker
  file + stderr; Python audit hook = also C-level `socket.__new__` / connect / bind / getaddrinfo /
  urllib / http.client) BEFORE refusing it, and AS3 / AS4 fail when the marker has a line - also when
  the library caught the OSError; a negative control (a script that tries twice and catches) proves
  the recording; `socket.gethostname` (a local OS query CTranslate2 makes at load, no traffic) is the
  one event allowed. AS3b = the same traps through the REAL station wiring (ap.asst, the screen-2
  button, App.ov_tick); traps there fire only for calls from an assistant code path (the station's
  own periodic sends go on). MED-3 THE LIMIT of the static proof, stated honestly: AS1 / AS2 cover
  groundstation/ + gateway/ only; the third-party libraries the worker imports (faster-whisper ->
  huggingface_hub -> httpx2 / httpcore2 / hf_xet / requests ...) contain network code BY DESIGN (the
  model download) and are not banned. AS2 walks the assistant Python's site-packages statically and
  LISTS the network modules reached (information, not pass / fail). The no-network proof of the
  worker therefore rests on the runtime blocker (AS3 / AS4), local model folders only (a name =
  refused, AS9) and the offline environment (HF_HUB_OFFLINE / TRANSFORMERS_OFFLINE, AS9). LOW-1: the
  view (app.AsstView) holds only a heard-question queue that App.ov_tick drains - never the App (AS6:
  a gc.get_referents walk). AM2-r2 MED-b: not even the loudhailer object any more - the horn state is a
  plain bool `view.horn` PUSHED by hailer.Hailer.push_horn (press / release / floor messages / unbind)
  and by App.ov_tick. AM2-r3 (review of 94ca987): the AS6 walk starts at the view AND at ap.asst (the
  client), goes into types (their dicts, methods, closures) and into the modules of
  groundstation/assistant/; it STOPS at the namespace of every OTHER module (counted in the test line):
  a function's __globals__ there is shared code that reaches the whole station by design, not a
  reference the assistant keeps - so the claim is "no App / loudhailer / link / Tk root reachable from
  the assistant's own objects", not "no edge skipped". Every place in hailer.py that changes the horn
  state (pressing / mine) calls push_horn() (AS6 static check; play_message's floor-not-given path was
  missing it). AS3b (AM2-r3 MED): (1) a callback that assistant code SCHEDULES (tkinter after /
  after_idle) is TAINTED - a sender reached from it counts as the assistant's, though no assistant
  frame is on the stack when it runs; (2) every sender call is counted per method + payload kind (the
  message's "t" / the frame type) and compared with the MOST of 3 windows without the assistant - any
  count above = FAIL (a periodic send may land once more in a window by its phase, so (2) alone can't
  see ONE extra send of a periodic kind - (1) does); mutation runs prove both: one send scheduled by
  assistant code through the station's own most frequent method + payload (caught by the taint) and
  four more scheduled by plain code (caught by the count). AM3-r3 (review of 25906e3 MED-1): the +1
  phase allowance exists ONLY for a method + kind the station sends on its own (baseline > 0); a kind
  with a baseline of 0 has no phase, so ONE send of a NEW kind - also from a thread the taint can't
  see - is caught by the count (mutation (c)). The station's start-up reads settle before the window.
- AM2-r2 LOW-1 (review of 25cb44f) - THE LIMIT of the runtime blocker, stated: it is PYTHON-level
  (socket patch + audit hook) and CANNOT see native code that opens sockets itself - CTranslate2
  (C++), hf_xet (Rust), any C extension calling the OS directly. Two OS-level checks therefore add to
  it: (1) AS4 lists every TCP / UDP endpoint the OS shows for the worker's PID (`netstat -ano`, no
  psutil) at ready, every ~1 s WHILE it answers and once AFTER the answers (AM2-r3) - any endpoint in
  any snapshot = FAIL (a socket opened and closed between two snapshots stays unseen: that is what the
  firewall rule is for); (2) a Windows Firewall
  OUTBOUND BLOCK rule for the assistant Python (rule "Trygons C2 assistant python - no network",
  program = tools/bin/assistant-python/python.exe), added by tools/get_whisper.py when run as an
  administrator (or `get_whisper.py --firewall`; netsh with an ARGUMENT LIST, no shell - AM2-r3);
  without the right it prints the ONE command for the owner (`netsh advfirewall firewall add rule
  name="..." dir=out action=block program="..." enable=yes`). The rule is read back with
  Get-NetFirewallRule + Get-NetFirewallApplicationFilter (AM2-r3: their Enabled / Direction / Action
  values are enum names, never translated - netsh's "show rule" text is localised on a Greek Windows):
  PASS only for a rule that is Enabled, Outbound, Block for exactly that program. Test AS4b prints its
  line also when the models are missing: present = PASS, absent = N/A with that command - never a PASS
  without it (W91). Ubuntu: the OS firewall by hand (documented, AS4b N/A - W93).

## 3. Speech -> text (step AM2)

Comparison (paper now, measured in AM2 on 30 recorded owner-style questions el + en, the laptop's mic
and a headset):

| | faster-whisper (CTranslate2) | whisper.cpp (ggml) |
|---|---|---|
| Form | Python package in its own env tools/bin/assistant-python/ (like vision-python) | one exe + .bin model, Python bindings or a pipe |
| GPU | CUDA 12 + cuDNN 9 DLLs (~600 MB extra download) | CUDA / Vulkan build, small |
| CPU fallback | int8, good | good (AVX2) |
| Greek | same Whisper weights = same accuracy | same |
| Initial prompt (term hints) | `initial_prompt` + `hotwords` | `--prompt` |
| VAD | built-in Silero VAD | built-in VAD in recent builds |

AM2 RESULT (2026-11-07, the owner's laptop: RTX A1000 6 GB, Windows 11). KEPT: faster-whisper
(tools/get_whisper.py: the embeddable Python 3.12.10 + faster-whisper 1.2.1 + CTranslate2 4.6.2 + the CUDA 12
cuBLAS 12.8.4.1 / cuDNN 9.8.0.87 wheels in tools/bin/assistant-python/; models at pinned Hugging Face
revisions, every file sha256-checked: large-v3-turbo (CTranslate2, 1.6 GB, GPU, int8_float16) and small
(0.48 GB, the CPU fallback, int8)). whisper.cpp was downloaded (b5130 cuBLAS 12.4 build + ggml
large-v3-turbo q5_0) but its downloaded exe was NOT allowed to run in the unattended session (the
permission guard), so it was NOT measured; it was deleted again and no download code for it was kept.
faster-whisper already meets the budget below, so the decision does not depend on it; whisper.cpp stays
an option if the CUDA wheels ever give trouble (it has no Python dependency).

Test speech = SYNTHETIC (no owner recordings yet): 20 Greek questions said by the Piper voice AND by the
Windows voice "Stefanos" + 10 English (Piper), 16 kHz. Word error rate, accents / case ignored (a strict
measure: one wrong ending = one error); time = from the release to the text, one question 1.5-3.5 s:

| Setting | el Piper | el Windows | en Piper | time per question |
|---|---|---|---|---|
| GPU large-v3-turbo, beam 1, hints in CAPITALS (first version) | 76 % | 52 % | 7 % | 1.2 s (first one 4.5 s: no warm-up) |
| GPU, beam 5, no hints | 47 % | 20 % | 7 % | 0.86 s |
| GPU, beam 5, hints in normal case (KEPT, as built in stt.py) | 45 % | 17 % | 7 % | 0.86 s (max 1.0 s) |
| CPU small, beam 5, hints (KEPT for the CPU) | 58 % | 34 % | 7 % | 5.0 s |
| CPU small, beam 1, hints | 54 % | 39 % | 10 % | 4.5 s (max 9.9 s with the el / en re-decode) |
| CPU large-v3-turbo | stopped after 7 min for 50 questions: > 8 s each - never on the CPU | | | |

Findings: (1) term hints written in CAPITALS (as the screens show them, without accents) make Whisper write
and mis-hear in capitals - the hints are normal-case words with accents; (2) beam 5 costs nothing on the
GPU and beam 1 saves only ~10 % on the CPU (and loses words): beam 5 everywhere; (3) a warm-up decode at load removes the slow first question; (4) Whisper's
language detection once took a Greek question for Polish: only el / en are accepted (else decoded again as
the likelier of the two); (5) the Piper Greek voice is hard to recognise (45 %) - a human voice is expected
near the Windows-voice figure; MEASURE again with the owner's own recorded questions (W84). The GPU time
(< 1 s) fits the 4 s budget of section 6; the CPU fallback does not (AM4's retrieval-only rule decides).
Language: the station language is only the tie-break; Whisper's detection (el / en) decides.
VRAM: the worker with large-v3-turbo loaded holds ~1 GB (nvidia-smi), nothing while it is stopped.

Term hints (as built, groundstation/assistant/hints.py): the Latin acronyms (HDG LOCK, DP, AUTO, STANDBY,
UAV ...), a short curated normal-case word list per language (καταπακτή, μάγγεμα, κλείδωμα πορείας, τιμόνι,
σχέση, μεγάφωνο ...) and the normal-case texts of the lang files (Tools page names, bar buttons, panel
words); the screens' all-capital Greek texts (mode bar, alarm titles) are NOT used (no accents, measured
worse). At most 700 characters per language (Whisper reads ~220 tokens of it), sent by client.py at init.

Microphone: its OWN setting (gs_settings `assistant_mic`, Tools page «Βοηθός»; groundstation/assistant/mic.py
lists the devices and captures at 16 kHz - audiodev.py's 8 kHz loudhailer path is not used), separate from the loudhailer `in_dev` / `out_dev`. None chosen = the Windows default
input ONLY for the assistant (it is not a boat function; the loudhailer's "never the default device"
rule is about the horn). Audio is never written to disk (RAM only, dropped after the answer).

PTT (W83 default): press-and-hold the framed «ΕΡΩΤΗΣΗ» button (screen 1 bottom bar, screen 2 bar 1 - AM2: bar 2 has no
room beside «ΟΜΙΛΙΑ» at 1366x768 / 125 %; bar 1 also exists in the camera view where a UAV is flown -,
with the "AI" badge) or hold F9 (gs_settings `assistant_key`; a USB foot pedal that sends F9 works the
same). Release = ask. Max 15 s per question (then asked automatically), < 0.4 s = ignored. NEVER on the
joystick (every stick button is used while flying a UAV), NEVER shared with the loudhailer «ΟΜΙΛΙΑ»:
the two use separate buttons, separate keys and separate devices, and while the horn is keyed by this
station the assistant does not listen (a question can never go out of the horn). F9 checked against
every existing binding (F11 / F12 / Space / A today).

## 4. Find (step AM3)

- Index = groundstation/assistant/index/manual_el.json + manual_en.json, built from MANUAL_EL.txt /
  MANUAL.txt: split by chapter (`N. TITLE`), sub-heading (`N.M` / upper-case lines / `---` rules) and
  paragraph; each passage = {id, chapter number + title, heading path, text, line range}. Passages
  60-250 words (long paragraphs split at sentences, short ones joined with the next under the same
  heading). Plus small "where is it" passages from the tour texts (academy/content/tour.json), marked as
  such (AM3-r LOW-1: the lang files are NOT indexed - term hints only).
- Stale check: the index stores the sha256 of the manual + tour.json + the parser; tools/check.sh FAILS when
  the committed index is stale (AM3-r MED-1: it no longer rebuilds it), the worker does not use a stale one.
- Search: Greek normalisation (lower case, accents and diaeresis removed, final sigma, Greek-Latin
  look-alikes for button names, a small stemmer: common endings -ος/-ου/-ο/-α/-ες/-ων/-ει/-ουμε ...),
  BM25 over words + the heading path weighted x2. Top 3 passages with their chapter names. A small
  offline multilingual embedding model (e.g. multilingual-e5-small, ~120 MB) ONLY if it measurably
  raises top-3 on the question list (AM3 measures BM25 alone first).
- Can't find: best BM25 score below a threshold (set on the question list's "not in the manual" items)
  = the can't-find answer, before any language model runs.

### 4a. As built (step AM3, measured)

- Code: groundstation/assistant/index.py (pure: parse, normalise, BM25, save / load / stale), tools/assistant_index.py
  (`build [--if-stale]`, `check`, `ask "..." [--lang]`, `eval [-v] [--report] [--gaps]`), the worker op `find` and
  `found` on every `heard`; the Tools page shows the best place (chapter / heading / term or the tour stop) or the
  can't-find sentence under the last question. No answer / voice yet (AM4 / AM5).
- Passages: el 554 (403 manual + 151 tour stops), en 823 (672 + 151). Units = paragraphs split at bullets and at
  lines that start a definition list ("  TERM   text": modes, alarms with their level letter, the trouble table);
  a TERM unit is its own passage and its term joins the heading path; other units are packed to MIN_WORDS 40,
  split above MAX_WORDS 120; < 5 words dropped; MANUAL_EL chapter 14 (the term table) left out (a translation
  list, it out-ranked the real answers).
- Search terms: the folded words minus both languages' stop words, Greek / English endings stripped, stems cut to
  6 (el) / 7 (en) characters, then the curated spoken-word table (index.SAID: what an operator SAYS -> the
  manual's word, e.g. drone -> UAV, joystick -> χειριστήριο, μηχανή -> κινητήρας, σταματάω -> STOP; "σήμα" is
  deliberately NOT mapped to "σύνδεση" because the manual's "σήμα AI" is the badge) + every pair of neighbouring
  terms. BM25 K1 1.2, B 0.5; chapter + heading + term x3; tour stops x0.9 (short texts are favoured by BM25).
  The parameters were chosen on the question list (a grid over MIN_WORDS / MAX_WORDS / HEAD_WEIGHT / B / STEM_LEN
  / TOUR_FACTOR / K1); the list is small, so the numbers below are a first measurement, not a guarantee.
- Can't find: best BM25 score < 3.0 OR the best passage covers < 35 % of the question's idf mass. The two do not
  separate cleanly on BM25 alone (an everyday word of the question - "κόστος", "ώρα", "boat" - matches some
  passage): the rest is AM4's job (the model answers only from the passages + the post-check).
- Measured (tools/assistant_index.py eval, question list v1, deterministic): el 145 questions (134 in the
  manual): top-1 82 = 61 %, top-3 103 = 77 %, false can't find 4; not in the manual: 9 / 11 can't find.
  en 48 (42): top-1 23 = 55 %, top-3 34 = 81 %, false can't find 0; 3 / 6. Weakest groups: engine (1 / 5 top-1),
  DP, screens, lost link; best: alarms (19 / 22), power limit. Search < 1 ms per question, index load 73 ms.
- An embedding model (multilingual-e5-small) was NOT tried in AM3: BM25 first (this section); AM4 decides whether
  the answer step needs better than top-3 77 % and measures an embedding model against these numbers then.
- Manual gaps found by the list (TEST_REPORT.md "Talking manual - FIND"): HATCH FAULT not explained (el + en), a
  stuck / half-open hatch, fuel tank / range, man overboard (el + en) = 6 entries, 4 topics.
- The AM3 figures above were IN-SAMPLE (every parameter chosen on the same list, loose labels): REPLACED by 4b.

### 4b. Review fixes (step AM3-r, review of 0feb416)

- MED-1 stale index: tools/check.sh runs `assistant_index.py check` (exit 1 = the gate FAILS; it never rebuilds
  quietly - rebuild by hand with `build` and commit the index with the manual change). The worker checks
  index.stale(lang) at init: a stale (or missing / broken) index is NOT loaded = every question of that language
  "can't find" + the reason, the ready message carries `index_err` -> the station notice asst.n_index (amber, Tools
  page line). Test AS14 (sources copied to a temp folder via TRYGONS_ASSISTANT_SRC, a manual changed, no rebuild ->
  `check` exit 1 STALE; the unchanged copy -> 0) + AS18 (worker: stale el = can't find, en still finds, the notice).
- MED-3 strict labels: a hit = a SPECIFIC expected phrase in the passage TEXT; a phrase found in >= 6 passages
  (LOOSE_N) or only in a heading / term does not count (every part of a long section shares its heading). Labels
  without a specific phrase were re-labelled from the manual text (el043 el060 el066 el070 el123 el134 el135, en001
  en002 en009 en019 en023 en027 en028 en032 en043); `eval` lists such a question as BROKEN. Tour-stop hits are
  counted apart (top-1 "via a tour stop"; whether they count as top-1 = W89).
- MED-2 held-out: tests/assistant/heldout.json = every 3rd question of each group (file order), frozen: el 42, en 10.
  The thresholds were re-tuned on the TUNING part only (grid MIN_SCORE 2..5 x MIN_COVER 0.25..0.5 x TOUR_FACTOR
  0.8..1.0: MIN_COVER 0.35 -> 0.30, the rest unchanged), then the held-out part was measured ONCE. Caveat: the
  held-out questions were part of the AM3 list, so the AM3 spoken-word table (SAID) and the parser were written
  with them in view - the held-out numbers are an upper bound until a fresh set (AM6).
- Measured (strict, deterministic):

  | | top-1 | top-3 | false can't find | not in the manual -> can't find |
  |---|---|---|---|---|
  | el in-sample (tuning, 96 in the manual) | 56 = 58 % (10 tour) | 75 = 78 % | 1 | 5 / 7 |
  | el HELD-OUT (38) | 24 = 63 % (4 tour) | 29 = 76 % | 1 | 4 / 4 |
  | en in-sample (33) | 15 = 45 % (1 tour) | 24 = 73 % | 0 | 2 / 5 |
  | en HELD-OUT (9) | 6 = 67 % | 8 = 89 % | 0 | 0 / 1 |

  AS17 floors sit just under these (tuning el 55 / 75 %, en 42 / 70 %; held-out el 60 / 73 %, en 55 / 75 %). The
  English held-out part is 9 questions: a rough figure.
- LOW-1: the lang files are NOT indexed (button names reach Whisper as term hints only, hints.py) and are no
  longer index sources. LOW-2: the worker searches only the question language's index (missing = can't find, never
  the other language). LOW-3: the assistant chapters (MANUAL_EL 17, MANUAL 22) are left out of the index
  (SKIP_CHAPTERS); the manuals no longer promise "never a guess" for the search alone (a wrong passage is
  possible - AM4's post-check is the guard). LOW-4: every split part of a long section has its OWN line range
  (per-word line numbers through the split), so [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ] (AM5) opens at the part, not at the section start.

### 4c. Review fixes (step AM3-r2, review of 9b12cd7)

- MED-1 tour labels: a passage of kind "tour" is ALWAYS a "tour" hit - also when it matched through an expected
  phrase in its text (el080, el109, en002 were counted as "manual" before). Expected phrases are counted on the
  MANUAL passages only (phrase_count / specific() / LOOSE_N never see a tour text). A question whose only phrase is
  in a tour text = a tour-only question (el135: expect [], its tour stop kept). `eval` reports top-1 as "from the
  manual" + "via a tour stop" apart (W95: recommended = reported separately, the floors stay on the total).
- MED-2 the false-"can't find" safety: MIN_COVER is per language, {"el": 0.30, "en": 0.33}. The English 0.30 of
  AM3-r refused only 2 / 6 out-of-manual questions (0.35 refused 3); swept on the English TUNING part only:
  0.31 .. 0.34 = 3 / 5 refused right, no false can't find, top-1 / top-3 unchanged -> 0.33 (inside the flat
  range; AM3-r3 LOW-3 measured again: 0.30 = 2 / 5, 0.35 = 3 / 5 refused right BUT 1 false can't find). AS17's
  English can't-find floor is back to 3 - NOTE (AM3-r3 LOW-4): this floor (FIND_MIN_CANT_OK en 3, checked on the
  WHOLE list) EQUALS the measured 3 / 6: there is NO margin, by the owner's choice W96 (a false can't find is the
  accepted price; a change to the English manual / index that loses ONE refusal fails AS17 at once and is looked
  at, never absorbed). BM25 is not the last guard: AM4's POST-CHECK (every name / number of the
  answer in the retrieved passages, else the retrieval-only sentence / can't find) is the BACKSTOP for a wrong
  passage that BM25 did not refuse; a false "can't find" is the accepted price of fewer wrong answers (W96).
- LOW: index.stale() also compares the committed passages with a fresh build (a hand-edited manual_*.json with the
  right hashes is stale; AS14); the tool's docstring no longer claims it reads the lang files; loose phrases next to
  specific ones removed (el032 "ΕΠΟΜΕΝΟ UAV" in 4 passages, el069, el090).
- Measured (strict, deterministic; tour hits now counted correctly):

  | | top-1 (manual + tour) | top-3 | false can't find | not in the manual -> can't find |
  |---|---|---|---|---|
  | el in-sample (tuning, 96) | 56 = 58 % (45 + 11) | 75 = 78 % | 1 | 5 / 7 |
  | el HELD-OUT (38) | 24 = 63 % (19 + 5) | 29 = 76 % | 1 | 4 / 4 |
  | en in-sample (33) | 15 = 45 % (13 + 2) | 24 = 73 % | 0 | 3 / 5 |
  | en HELD-OUT (9) | 6 = 67 % (6 + 0) | 8 = 89 % | 0 | 0 / 1 |

- The held-out numbers stay an upper bound (the SAID table / parser were written with the AM3 list in view): AM6
  needs a FRESH question set never seen while tuning (the owner's own recorded questions, W84, are the best source).

### 4d. Review fixes (step AM3-r3, review of 25906e3; tests / tools / docs only)

- MED-1 AS3b's count comparison: the +1 phase allowance (ALLOW_PHASE) applied to EVERY method + payload kind, also
  one the station never sends on its own (baseline 0) - so ONE send of a NEW kind from a path the taint can't see
  (a thread, a socket callback) passed. Now the allowance exists only for a kind with a baseline > 0; a kind with a
  baseline of 0 has no phase (one send = FAIL). Mutation (c) proves it: one send of a new kind (the station's most
  frequent sender method with "t" = "asst_mutation" / frame type 0xFE, unknown to proto.py and app/proto.h) made
  from a THREAD is caught by the count alone.
- LOW-1 the AS6 gc walk returns whether its todo list ran EMPTY (complete); AS6 asserts that, not only
  len(seen) < limit.
- LOW-2 the read-back Program test string is a raw string (no SyntaxWarning for "\p").
- LOW-3 the English MIN_COVER note: the flat tuning range is 0.31 .. 0.34 (not 0.32 .. 0.34) and 0.35 has 1 false
  can't find (index.py + 4c corrected, measured again).
- LOW-4 stated in 4c: the English can't-find floor 3 equals the measured 3 / 6 - no margin (W96).
- LOW-5 tools/get_whisper.py rule_blocks docstring: a literal %X% in our exe path reports the rule MISSING =
  fails safe.
- W103 / W104 asked (PROGRESS.md): a manual-only top-1 floor beside the total; INDEX_VERSION bumped (2 -> 3) now
  that stale()'s meaning changed in AM3-r2 (the passage comparison) without a bump - the index rebuilt + committed.

## 5. Answer (step AM4)

Candidates (Q4_K_M GGUF through llama.cpp; licences checked again at download time):

| Model | File | VRAM (4k ctx) | Greek | Licence (to verify) |
|---|---|---|---|---|
| Qwen2.5-7B-Instruct | ~4.7 GB | ~5.5 GB (does NOT fit beside Whisper + the HUD on 6 GB: partial offload) | good | Apache-2.0 |
| Meltemi-7B-Instruct (ILSP, Greek) | ~4.4 GB | ~5.2 GB (partial offload) | best Greek (made for it) | Apache-2.0 |
| Llama-Krikri-8B-Instruct (ILSP, Greek) | ~4.9 GB | ~5.8 GB (partial offload) | very good Greek | Llama 3.1 community |
| Llama-3.1-8B-Instruct | ~4.9 GB | ~5.8 GB | fair | Llama 3.1 community |
| Qwen2.5-3B-Instruct / Llama-3.2-3B-Instruct (the 3B option) | ~2 GB | ~2.6 GB (fits) | fair | Qwen research / Llama 3.2 community |

Comparison on the I2 question list (AM4): correctness (post-check pass + must-mention term present),
Greek quality (the owner reviews 20 answers per model, blind), time to the first sentence, tokens/s,
VRAM, the CPU-only time. The winner + its numbers go into this file and the manuals.

Grounded prompt: system text (fixed, in the question's language): "Answer ONLY from the passages below,
in 1-2 short spoken sentences, name the chapter; if the passages do not answer the question, reply
exactly: <can't-find sentence>." + the 3 passages + the question. Temperature 0, max 80 tokens.

Post-check (before anything is spoken): every button / mode / screen / alarm name in the answer (found
by the term list of section 3) must appear in the passages, the chapter named must be one of them, and
(AM1-r M3) every NUMBER in the answer together with its UNIT ("5 kn", "3 s", "50 m", "30 %", "120 m",
"10 min"; digits compared after normalising "," / "." and spoken number words) must appear in the
passages with that same unit; else the answer is replaced by the best passage's first sentence (retrieval-only form). The can't-find
sentence from the model is accepted as is.

GPU guard (gpu.py): before loading and before each answer, free GPU memory from NVML (nvidia-smi
fallback). Too little free memory (MEASURE: model + 0.8 GB margin, because the station's video decode,
the HUD and the academy's 3D sim also use it) = run on the CPU; CPU too slow (first sentence > 8 s on
the last 3 answers, MEASURE) = RETRIEVAL-ONLY mode: speak the best passage's shortest full sentence,
show the passage. The panel shows the mode (GPU / CPU / retrieval only).

### 5a. As built (step AM4)

- Route: llama.cpp through its PYTHON BINDINGS inside the worker (llama-cpp-python 0.3.36, the CUDA 12.4 Windows
  wheel of its GitHub release v0.3.36-cu124, 2026-10-01; size + sha256 = the release's own digest pinned in
  tools/get_llm.py; installed with `pip --target tools/bin/assistant/llm/py --no-deps`, its 4 small dependencies
  hash-locked in tools/assistant_requirements_llm.txt - the Whisper lock of section 10 is not touched). No exe, no
  llama-cli, never llama-server, never a port: the bindings ran in this unattended session (the downloaded
  whisper.cpp EXE of AM2 was not allowed to run; a DLL loaded by the already-used assistant Python was).
  AS23 checks every assistant module: no llama-server / llama-cli / --port / --host string in code, no
  llama_cpp.server / uvicorn / fastapi import, llama_cpp imported only in answer.py; AS2-style walk of the
  bindings lists the network modules their code can reach (asyncio via jinja2, requests in the grammar helper,
  urllib in the chat-format helper - never called by our path; the server sub-package needs uvicorn / fastapi,
  not installed). AS24 = the real model answering with the Python blocker + the worker PID's netstat endpoints
  (N/A when not downloaded).
- Code: groundstation/assistant/answer.py (prompt, post-check, can't find, retrieval-only, StubLlm / LlamaLlm),
  gpu.py (NVML through ctypes - the worker starts no process, so no nvidia-smi; a pure-Python GGUF header reader
  for layers / KV heads; plan()), worker.py (init "llm", op "ask_text", "answer" after every "heard", the guard
  before every answer), client.py (llm_cfg, stays THINKING until "answer", measures release -> answer text =
  answer_ms; ANSWER_TIMEOUT_S 30 -> 60 s: a CPU-only model reads the passages for many seconds before its first
  word), page.py (the answer line + where the model runs), app.py (AsstView.answers -> screen 2's message line).
- Prompt: the fixed instruction in the question's language (answer ONLY from the passages, 1-2 short COMPLETE
  sentences, names and numbers exactly as there, no chapter number, else exactly the can't-find sentence) + the top
  3 passages numbered with chapter / heading + the question; temperature 0, top_k 1, seed 0, streamed, at most 160
  tokens, stopped at 2 sentences, cut at MAX_GEN_S 10 s (= this answer in the retrieval form). The model never
  names the chapter: the CODE cites the passage the answer's words come from (cite).
- Post-check (answer.postcheck): names = words in capitals (+ runs: "HDG LOCK"), «quoted» / "quoted" spans and
  the screens' 2-5-word capital lang values in plain form ("κλείσιμο καταπακτής" too); numbers = digits (a range
  gives its unit to both ends) and number WORDS followed by a unit ("πέντε δευτερόλεπτα" = 5 s; a bare "ένα" is an
  article), units folded to one form (kn / κόμβοι, s / δευτερόλεπτα, m / μέτρα, min / λεπτά, %, °, V ...); a
  number WITH a unit must be in the passages with that unit, a bare number anywhere in them; a chapter the answer
  names must be one of theirs. Any miss = the best passage's FIRST sentence (src "passage"), the model's text kept
  in the log only. The model's can't-find sentence is accepted as it is.
- GPU guard (gpu.plan): need = the GGUF file + the KV cache for 3072 tokens (from its header) + 450 MB compute;
  free (NVML, AFTER Whisper is loaded) - MARGIN_MB 800 >= need = all layers on the GPU; else as many layers as fit
  ("gpu+cpu"); none = CPU. Before every answer: less than LOW_MB 300 free = reloaded with a new plan (the CPU).
  A failed GPU load is tried once on the CPU; a failed load at all = retrieval only (never a crash).
- Retrieval-only (no model / not installed / setting `assistant_llm` "off" / 3 answers in a row > SLOW_S 8 s or
  cut): the best passage's SHORTEST full sentence (>= 5 words, starts like a sentence, brackets balanced) as the
  owner's rule says. MEASURED on the tuning part (no model needed): the question's must-mention term is in that
  sentence for 21 / 96 Greek questions, in the FIRST sentence for 34 / 96, in the sentence sharing most words with
  the question for 44 / 96 (English 4 / 10 / 16 of 33) - W98 asks whether to switch to the best-overlap sentence.

### 5b. Model comparison (step AM4, measured 2026-11-07 on the owner's laptop: RTX A1000 6 GB, Windows 11)

Method: tools/assistant_llm_eval.py `run` - the REAL worker (Whisper large-v3-turbo loaded first, as at the station,
so the GPU guard sees the real free memory), typed questions (`ask_text`), the question list's TUNING part (the
held-out part never used for a choice), every candidate Q4_K_M + "none" (no model = the retrieval form). Correct =
the shown text holds the question's must-mention term and is not "can't find" (not-in-the-manual questions:
correct only as "can't find"); "model's own" = correct with the model's own sentences (post-check passed);
quality = an AUTOMATIC proxy only (Greek script / a Latin word only if in the passages, a complete sentence, <= 40
words) - the owner's blind review is W99 (docs/assistant_llm_review.md); time = question in -> answer out of the
worker (search + model; Whisper's ~0.9 s comes on top, see e2e below). CONDITIONS (stated honestly - the PC was in
use): the free GPU memory at load varied 3181-4886 MB (another program held 0.5-1.6 GB at times), the two 3B runs
overlapped a test_assistant run, qwen2.5-7b / llama3.1-8b ran only on the first 40 Greek + all 33 English tuning
questions (time) and partly beside tools/check.sh. The speeds of the partly-offloaded models are therefore
pessimistic; correctness does not depend on speed except through the 10 s cut.

Tuning part, all questions (Greek 96 in the manual + 7 not, English 33 + 5):

| model | lang | correct | model's own | post-check rejected | can't find right | false can't find | quality proxy | median / p90 ms | runs on (GPU layers) |
|---|---|---|---|---|---|---|---|---|---|
| none (retrieval form) | el | 21/96 = 22 % | - | - | 5/7 | 1 | - | 0 / 0 | - |
| qwen2.5-3b | el | 12/96 = 12 % | 8 % | 4/41 | 7/7 | 37 | 49 % | 2371 / 10072 | gpu (all) |
| llama3.2-3b | el | 21/96 = 22 % | 20 % | 4/49 | 7/7 | 40 | 87 % | 5187 / 9803 | gpu+cpu (23 of 28) |
| meltemi-7b | el | 22/96 = 23 % | 14 % | 4/40 | 7/7 | 27 | 89 % | 6929 / 10198 | gpu+cpu (24 of 32) |
| krikri-8b | el | 23/96 = 24 % | 17 % | 0/37 | 7/7 | 44 | 95 % | 7665 / 10214 | gpu+cpu (21 of 32) |
| none (retrieval form) | en | 4/33 = 12 % | - | - | 3/5 | 0 | - | 0 / 1 | - |
| qwen2.5-3b | en | 15/33 = 45 % | 42 % | 1/29 | 5/5 | 4 | 79 % | 3352 / 7098 | gpu (all) |
| llama3.2-3b | en | 22/33 = 67 % | 64 % | 2/35 | 3/5 | 0 | 100 % | 3223 / 5277 | gpu+cpu (23 of 28) |
| meltemi-7b | en | 13/33 = 39 % | 36 % | 3/21 | 4/5 | 1 | 89 % | 9499 / 10188 | gpu+cpu (24 of 32) |
| krikri-8b | en | 15/33 = 45 % | 39 % | 1/20 | 5/5 | 11 | 100 % | 7051 / 10130 | gpu+cpu (21 of 32) |
| qwen2.5-7b | en | 19/33 = 58 % | 58 % | 0/27 | 5/5 | 4 | 100 % | 5468 / 9416 | gpu+cpu (12 of 28) |
| llama3.1-8b | en | 21/33 = 64 % | 64 % | 0/30 | 5/5 | 2 | 100 % | 6371 / 9583 | gpu+cpu (13 of 32) |

The same first 40 Greek tuning questions for every model (39 in the manual + 1 not):

| model | correct | model's own | false can't find | quality proxy | median ms |
|---|---|---|---|---|---|
| none | 11/39 = 28 % | - | 1 | - | 0 |
| qwen2.5-3b | 6/39 = 15 % | 13 % | 14 | 65 % | 2070 |
| llama3.2-3b | 10/39 = 26 % | 26 % | 16 | 84 % | 4920 |
| meltemi-7b | 12/39 = 31 % | 18 % | 13 | 100 % | 6655 |
| krikri-8b | 11/39 = 28 % | 21 % | 20 | 100 % | 8432 |
| qwen2.5-7b | 12/39 = 31 % | 28 % | 8 | 30 % | 4687 |
| llama3.1-8b | 8/39 = 21 % | 18 % | 5 | 88 % | 10079 |

GPU memory: Whisper alone = ~1.1 GB (the "none" worker); the worker with a 3B model = 3.2-3.4 GB (Whisper + model +
KV + compute); with a 7-8B model 4.6-4.7 GB at 12-24 of its layers on the GPU. A 7-8B model NEVER fits completely
beside Whisper with the 800 MB margin on 6 GB (need 5.2-5.7 GB for the model alone, gpu.plan) - it always runs
partly on the CPU = 2.3-3 tokens/s, median 5-10 s per answer, often cut at 10 s.

FINDINGS: (1) English: a model clearly helps (llama3.2-3b 67 % vs the retrieval form's 12 %). (2) Greek: every
model says "can't find" to 27-44 of the 96 answerable questions (the search found a passage; W101) - Greek
correctness with any model is about the retrieval form's (22-31 %); the Greek-native models (Krikri, Meltemi) write
the best Greek (proxy 89-95 %, no post-check rejection) but are 2-3x slower here. (3) The post-check rejected 0-15 %
of the generated answers (a changed number / an invented name) - the guard works; it cannot catch a wrong but
grounded sentence (e.g. "Ο ΚΥΒΕΡΝΗΤΗΣ απαντά ΝΑΙ" for "how do I stop a launch"). (4) qwen2.5-3b writes poor Greek
(numbered lists, invented words: proxy 49 %).

CHOICE (tools/get_llm.py CHOSEN = paths.LLM_DEFAULT): **llama3.2-3b** (Meta Llama 3.2 3B Instruct, Llama 3.2
Community licence) - the only candidate that fits COMPLETELY on the GPU beside Whisper with the margin (need 2.7 GB;
all 28 layers when >= 3.5 GB is free), best English, Greek correctness = the larger models' within the noise of this
small list, and the only one inside the < 4 s target (e2e below). Krikri-8B (the best Greek wording) is the
alternative for a PC with >= 8 GB of GPU memory (`TRYGONS_ASSISTANT_LLM=krikri-8b`); W99 (the owner's blind Greek
review) may change the choice.

HELD-OUT (measured ONCE after the choice; the same worker / conditions): 

| model | lang | correct | model's own | post-check rejected | can't find right | false can't find | quality proxy | median / p90 ms | runs on |
|---|---|---|---|---|---|---|---|---|---|
| none (retrieval form) | el | 11/38 = 29 % | - | - | 4/4 | 1 | - | 1 / 2 | - |
| llama3.2-3b (CHOSEN) | el | 8/38 = 21 % | 16 % | 3/23 | 4/4 | 15 | 90 % | 4946 / 9230 | gpu (all 28) |
| krikri-8b | el | 10/38 = 26 % | 18 % | 0/16 | 4/4 | 12 | 94 % | 9408 / 10190 | gpu+cpu (12 of 32) |
| none (retrieval form) | en | 2/9 = 22 % | - | - | 0/1 | 0 | - | 1 / 2 | - |
| llama3.2-3b (CHOSEN) | en | 6/9 = 67 % | 67 % | 0/9 | 1/1 | 0 | 100 % | 3447 / 7102 | gpu (all 28) |
| krikri-8b | en | 4/9 = 44 % | 44 % | 0/6 | 1/1 | 3 | 100 % | 6294 / 10100 | gpu+cpu (12 of 32) |

(the held-out run overlapped the gateway test suite (CPU busy): its times are pessimistic - see e2e for the quiet
times). The held-out confirms the tuning picture: English 67 % with the model vs 22 % without; Greek no better than
the search's sentence (W102). The held-out set is small (38 + 9) and was part of the AM3 list (an upper bound,
docs 4b) - AM6 needs a fresh set.

End to end (`e2e`, release -> answer text, the real Whisper + llama3.2-3b, 10 Greek tuning questions spoken by the
Windows voice into the simulated microphone): two runs on the quiet PC
right after each other: run 2 = median **1.9 s**, max 2.6 s (Whisper 0.85-0.99 s of it; the model 0.7-1.7 s;
4 of the 10 were "can't find" in 1.6-1.8 s); run 1 (directly after the heavy comparison runs, the GPU at 72 °C) =
median 8.8 s, max 13.8 s - its first 6 questions took 8.5-13.8 s with Whisper alone at 4.1 s, its last 4 1.6-2.6 s.
So the < 4 s target (to the answer TEXT; the voice of AM5 adds its first sentence) is met on a quiet, warm GPU and
missed while the GPU is busy / throttled; AM5 measures again with the voice. GPU memory of the worker (Whisper +
llama3.2-3b, all on the GPU): 3650 MB (nvidia-smi 1349 -> 4999 MB of 6144). Recognition of the Windows voice was
imperfect ("καταβακτή", "εκτόξευσιντρόουν") - the search still found 6 of 10 (W84: real voices).
Answers seen in the e2e run that are WRONG but passed the post-check (grounded words, wrong meaning): el013 "Η
πύλη δίνει στο GPS του το UAV ..." (a launch-refusal question), el001 «ΚΑΤΑΠΑΚΤΗ» as a button name (the word is in
the passages as an ordinary word): the post-check is a guard against invented names / numbers, not a judge of
meaning - the chapter shown with every answer stays the operator's check (MANUAL ch. 22 / 17).

## 6. Speak (step AM5)

- Through the station's own sound.py SoundSystem: a new level ASSISTANT below FEEDBACK, ADVISORY,
  CAUTION, WARNING and every voice callout. Any alarm sound or callout request STOPS the assistant's
  voice at once (the player's stop) and the rest of the answer is dropped (the panel still shows it;
  [ΞΑΝΑ] replays it). An assistant answer never delays, queues ahead of or mutes an alarm. Tests in
  test_sound (AM5).
- TEXT ONLY (AM1-r M4): an answer that arrives while this PC's MasterLamps show lamps.on(WARNING) or
  while a voice callout is queued or sounding is NOT spoken at all - the panel shows it as text (the
  mode line says "text only - alarm"), [ΞΑΝΑ] speaks it later when nothing of that is left. The
  assistant never starts its voice into an active WARNING.
- tts.py with gs_settings `voice_engine` (Piper or the Windows voice); answers are NOT cached in
  sound_cache/ (one-off sentences; temporary WAV in the station's temp folder, deleted after playing).
- Speech normaliser (speech.py): upper-case words -> normal case ("ΚΑΤΑΠΑΚΤΗ" -> "καταπακτή"),
  acronyms spoken as in the "say." lang lines (HDG LOCK, DP, AUTO, UAV, AIS, GPS ...), arrows /
  symbols / units ("°", "kn", "%") spoken, chapter numbers as words.
- Target: the answer starts < 4 s after the release on the owner's laptop. Budget (MEASURE in AM4 /
  AM5): STT 0.8 s + find 0.05 s + LLM prompt 0.6 s + first sentence 1.2 s + TTS first sentence 0.4 s
  = ~3 s; the first sentence is spoken while the second is still generated (streaming).

## 7. Show + log (step AM5)

- Panel (the 13n1 look: theme.py fonts / 3 text sizes / square frames, draggable by the header,
  collapsible, position remembered in gs_settings): the RECOGNISED question (a mis-hearing is visible),
  the answer, the FULL passage + its chapter, [ΞΑΝΑ] (repeat), [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ] (opens MANUAL_EL.txt /
  MANUAL.txt at that place in a read-only text window), the "AI" badge, the mode (GPU / CPU / retrieval
  only). Screen 2: an overlay panel placed by the fit test (never over the boat keep-out circle, the
  S1 matrix still passes); screen 1: a small box in the lower right, hidden in MINIMAL declutter, never
  over the alarms / notices boxes or the centre. Later option: light the named button using
  academy/tour_shots/shots.json.
- Log on THIS PC only: groundstation/assistant/log/YYYY-MM-DD.jsonl (gitignored): question text,
  passage ids + scores, answer, mode, timings. Never audio, never sent anywhere (no gateway sync, no
  academy push).

## 8. PC load (owner's laptop: RTX A1000 6 GB laptop GPU, falls back to the CPU)

| Part | Idle (loaded) | While answering | Where |
|---|---|---|---|
| Whisper large-v3-turbo int8 | ~1 GB VRAM, ~0 CPU | GPU burst < 1 s | worker |
| LLM 3B Q4 / 7B Q4 | 2.6 GB / partial offload | GPU burst 1-3 s | worker |
| Index + BM25 | < 20 MB RAM | < 50 ms CPU | worker |
| TTS (Piper) | - | 1 core < 0.5 s | station (existing) |

- Idle = no CPU use (the worker blocks on stdin); the GPU memory stays allocated while the worker runs.
- Measured in AM4 / AM5 with tools/gateway_load.py's station counterpart (the station's frame time with
  4 camera views + the HUD, with and without an answer running): the screen-1 frame time must not grow
  by more than 10 % during an answer (MEASURE; else fewer threads / CPU STT).
- Never on the gateway PC (the worker refuses to start when gateway.json is present, like the academy),
  never on the boat.

## 9. Tests (per step; all in tools/check.sh)

- AM2: AS1-AS4 (section 2), PTT state machine (press / release / too short / too long / F9 / button /
  horn keyed = not listening), mic setting separate from the loudhailer, worker restart + «ΒΟΗΘΟΣ ΕΚΤΟΣ»
  notice (stub worker), term hints built from the lang files.
- AM3: index build + stale check, normalisation (accents, final sigma, case), retrieval top-1 / top-3 on
  the question list (thresholds set from the first measured run), can't-find on the "not in the manual"
  items.
- AM4: prompt + post-check (an invented button name = replaced by the passage sentence; AM1-r M3: an
  answer whose number / unit was altered, e.g. the passage says "5 kn" and the answer "15 kn" or
  "5 m", = replaced by the retrieval-only sentence), can't-find,
  GPU guard (fake NVML numbers), retrieval-only mode; model comparison = a report, not a pass / fail.
- AM5: alarm priority (a WARNING / CAUTION / callout stops the answer at once, never waits behind it;
  AM1-r M4: an answer arriving while lamps.on(WARNING) or a callout is queued / sounding = text on the
  panel only, no voice started, [ΞΑΝΑ] speaks it once that is over),
  speech normaliser, panel fit (S1 matrix + screen 1 boxes), "AI" badge present, log written locally
  and never contains audio.
- AM6: the full question list with the real models (N/A - never PASS - when the models are missing; the
  fast tests use a stub model), results in TEST_REPORT.md, the manual-gaps list.
- The question list tests/assistant/questions_el.json + _en.json (AM3 first version, AM6 full): 120-150
  questions as an operator would say them, several phrasings per question, grouped by situation (UAV
  flying, launch + hatch, alarms, modes, lost link, recording, loudhailer, roles, DP, engine, power
  limit), each with the expected section(s) + a must-mention term; some NOT in the manual (expect
  can't find). The owner reviews it (a W item in AM3).

## 10. Downloads (tools/bin only, never git)

- tools/get_whisper.py (AM2): the chosen Whisper runtime + model, pinned URL + sha256, into
  tools/bin/assistant/whisper/; tools/get_llm.py (AM4): llama.cpp (Python bindings in
  tools/bin/assistant-python/ or the pinned exe) + the chosen GGUF, pinned sha256, into
  tools/bin/assistant/llm/. Both found through gateway/toolpaths.py (new functions
  `assistant_python()`, `whisper_model()`, `llm_model()`), env overrides like the others.
- A download that fails its sha256 is deleted and reported; nothing is fetched at run time.
- AM2-r MED-4 (supply chain): the embeddable Python zip (sha256; its md5 = python.org's) and get-pip.py
  from a pinned pypa/get-pip COMMIT (sha256; never the moving bootstrap.pypa.io URL) are checked
  before use; pip installs ONLY `--require-hashes --only-binary :all: -r tools/assistant_requirements.txt`
  (+ `-r tools/assistant_requirements_cuda.txt`): every dependency pinned, every file hash of that
  version (pip-compile --generate-hashes style). `get_whisper.py --make-requirements` regenerates them
  from a working assistant Python (pip freeze --all + PyPI's sha256) - review the diff before a commit.
  setuptools stays in the lock (CTranslate2 needs it); pip itself comes from the hash-checked
  get-pip.py. Test AS13.
- AM2-r2 LOW-2 / LOW-3: an install lock marker tools/bin/assistant-python/trygons_install_lock.json =
  the sha256 of the requirements file(s) + the cuda choice + the bootstrap (Python version, zip sha256,
  get-pip.py sha256), written only after a successful install + import check; AM2-r3: missing /
  different / another bootstrap = the assistant Python is installed AFRESH from the checked zip (never
  pip over an old install: a package the new lock dropped would stay), and the marker is written only
  when `pip freeze --all` equals the locked set (pip / wheel aside; else FAILED, no marker). `--make-requirements` runs ONLY
  with TRYGONS_ASSISTANT_PYTHON set (never this Python by accident). Test AS13.

## 11. Open questions (PROGRESS.md WAITING FOR OWNER)

- W81 which PCs get it - default every station PC (captain + copilot), setting `assistant` on.
- W82 live telemetry answers - default NO (manual questions only).
- W87 (AM2-r) screen 2: «ΕΡΩΤΗΣΗ» on bar 1 instead of bar 2 (no room beside «ΟΜΙΛΙΑ»; also in the
  camera view) - recommended: accept. W88 (AM2-r): the S1 fit matrix with the button shown in IDLE
  counts as the fit proof (the label / size never change with the state) - recommended: yes.
  Screen 1 (AM2-r MED-1): «ΕΡΩΤΗΣΗ» = its own group right after the language / night / size group,
  >= 2 bar heights + AR / LAYOUT away from «ΟΜΙΛΙΑ» (test_driver I19, AS11). Screen 2 (AM2-r2 MED-a):
  «ΕΡΩΤΗΣΗ» = the FIRST item of bar 1 (left end, also in the camera view); «ΟΜΙΛΙΑ» is in the right
  part of bar 2; test_screen2 S1 checks in every layout of the matrix that they are >= 2 bar heights
  apart in x or y (measured: 549 px apart in x at 1366x768). W90: the minimum distance as part of W87.
- W90 (AM2-r2) screen 2's ask-to-talk minimum distance (>= 2 bar heights in x or y, S1) as part of
  W87 - recommended: yes. W91 (AM2-r2) the OS-level network block (firewall rule) wanted - recommended:
  yes, with the one-command fallback when the install has no administrator right.
- W92 (AM2-r3) the ask-to-talk minimum distance also in the camera view and with the bars docked at
  the top - recommended: yes (built as a test only: test_screen2 S1 checks both docks in all 24
  layouts, S25 the camera view at 1366x768 / 1920x1080 x 100-150 % with both docks). W93 (AM2-r3) on
  Ubuntu the OS firewall for the assistant Python stays manual and AS4b = N/A - recommended: accept.
- W83 push-to-talk - default the framed «ΕΡΩΤΗΣΗ» button (screen 1 bottom bar + screen 2 bar 2) +
  F9 (setting) + an optional USB foot pedal sending F9; never the joystick, never «ΟΜΙΛΙΑ».
- W84 (AM2) the owner's own recorded questions + the 2 new tour clips.
- W85 (AM1-r) the default microphone: AM2 uses the Windows default input when `assistant_mic` is
  not set (mic.py WAVE_MAPPER), while the loudhailer rule is "never the PC's default speaker / mic".
  Recommended: the assistant also needs an EXPLICIT choice - none chosen = OFFLINE "not configured"
  (the button greyed, the Tools page says so).
- W86 (AM1-r) a question about a SAFETY ACTION (STANDBY, e-stop, lost link, ...) with no manual hit:
  only "I can't find that in the manual", or also point to where to look? Recommended: yes, the panel
  also shows the chapter list of the manual (text only, never a guessed answer).
