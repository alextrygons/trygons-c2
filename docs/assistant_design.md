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
    `gpu.py` (free GPU memory guard), `show.py` (what the slim panel shows - the panel itself is drawn by app.py's
    overlay, AM5), `page.py` (Tools page «Βοηθός»), `qlog.py` (local log); the voice lives OUTSIDE the package in
    groundstation/asstvoice.py (sound.py reaches proto: the package must not import it, AS1 / AS2).
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

### 5c. Review fixes (step AM4-r, review of 3e1b1d3; station / tools / tests / docs only)

Where this section differs from 5a, THIS wins.

- MED-1 the post-check could be bypassed: a number followed by an UNKNOWN unit ("50 feet", "50 nautical miles",
  "3 milliseconds", "3 hrs", "20 per cent") was checked as a bare number (its value is in the passages: pass), the
  sign was dropped ("-3 s" = "3 s") and only a few number words were known ("eighty seconds", «εβδομήντα /
  τριακόσια δευτερόλεπτα» passed). Now (answer.norm / digitize / numbers): every number word of both languages is
  turned into digits first (2-19, the tens, the Greek hundreds διακόσια .. εννιακόσια, χίλια / thousand,
  "hundred" multiplying, hyphenated "twenty-five", compounds "εικοσιπέντε"; a bare "ένα" / "μία" / "one" / "half" is
  an article unless a known unit follows it); the sign is kept; the unit table has the review's spellings and more
  (feet / ft / πόδια, miles / mi / μίλια, nmi / nautical / ναυτικά, hrs, msec / milliseconds, per cent / τοις εκατό,
  cm, mm, kg, m/s, km/h, kph, mph, °/s ...; its keys in plain form - the earlier "κόμβους" never matched the
  final-sigma-folded text); a number followed by a word that is NO known unit must stand with that word in the
  passages ("50 fathoms" with "50 m" in the manual = refused: the stricter rule the review preferred, as a bigram so
  "οθόνη 1 του κυβερνήτη" copied from the manual still passes); a number glued to letters is an ordinal ("7ο", "1η"
  = a bare number) or an identifier ("13q2c", "v2": not a number); a date "2026-11-04" is not three numbers.
- MED-2 numbers were a SET over the three passages. Now every number of the answer must be in ONE passage SENTENCE
  together with the content word standing next to it in the answer (the nearest one on each side, stop words /
  units skipped; stems compared by their first 5 letters): the review's swapped probe "Reverse is capped at 20 % and
  forward at 5 %" is caught when the manual states the two caps in separate sentences ("The remote cap is 20 %
  forward. Reverse is capped at 5 %.") - a swap inside ONE manual sentence is NOT caught (a lexical check cannot;
  stated here and in the manuals; the extract mode of 5d makes it moot). AM4-r2 MED-3, said plainly: the number
  check looks ONLY at the one content word on each side of the number - a LEXICAL check of word adjacency, not of
  the claim: "the hatch closes in 3 s" and "the hatch opens in 3 s" both pass against a manual "the hatch closes in
  3 s" ("hatch" stands next to the number in that sentence). The negation rule below is lexical the same way (the
  negated WORD, not the meaning). In the extract mode (verbatim manual sentences) both are moot. Negation: an
  answer sentence with
  never / not / no / cannot / without / δεν / μην / ποτέ / όχι / χωρίς / κανένα ... needs a negation in a TOP
  supporting passage sentence (the one(s) sharing most content words); a plain answer sentence is refused when a
  top supporting sentence negates the very word it shares (a negation word up to 2 tokens BEFORE a shared word:
  "the PC can NEVER override it" vs "The PC can override STANDBY"; measured first with "anywhere in the sentence"
  = 13 of 35 English answers refused, most of them wrongly - the manual's sentences are full of "no other mode",
  "nothing else" clauses). An answer sentence with content words that shares none with any passage sentence is
  "not grounded" (refused). Names: a Capitalised word standing as the name of a thing - "the Reset button", "PC
  button", «κουμπί Reset», «σελίδα Κάμερες» (NAMED_RE / NAMED_EL_RE) - must be in the passages, and the
  ALLOWED_CAPS words (PC, OK, C2, Trygons) get no exemption there ("PC button" with no PC in the passages = refused).
- LOW-1 the language: the answer must be in the question's language - among its words that are NOT words of the
  passages (names like HDG LOCK are), the Greek-script letter share must be >= 1/2 for a Greek question and <= 1/2
  for an English one (an English answer to a Greek question = refused, never spoken).
- The review's rule (W105, default yes): a FAILED check now SAYS «Δεν βρίσκω απάντηση στο εγχειρίδιο.» and SHOWS
  the best passage with its chapter (every answer that rests on a passage carries `passage` {text, cite, id};
  the Tools page prints it under the answer, screen 2's message line says "see the passage on the Assistant page
  (chapter N)"); the passage's FIRST sentence is no longer spoken (it may not answer the question). The
  retrieval-only mode (no model / too slow) keeps the owner's rule (the shortest whole sentence). AM5 will speak
  "from the manual, chapter N" before every answer (the cite is the code's, never the model's).
- MED-3 LlamaLlm.generate: a generation that stopped at max_tokens (finish_reason "length") is CUT = its unfinished
  last sentence is dropped (decide keeps the whole ones, post-checked); the time to the FIRST token is measured
  (first_ms, in the answer + the eval rows) and a first token later than MAX_GEN_S counts as cut. The bindings'
  prompt pass cannot be interrupted from Python (the generator yields nothing until it is done): the client's
  ANSWER_TIMEOUT_S 60 s bounds the worst case (worker restart). MAX_TOKENS 160 stays: AM4's longest KEPT answer was
  72 tokens (Greek median 19), the 2-sentence stop ends every answer first; the 9 Greek cuts of the AM4 run were
  the 10 s time cut on the partly-offloaded model, not the token limit.
- MED-4 latency, honestly: the AM4 figure "median 1.9 s" was over ALL 10 spoken questions, 4 of which were "can't
  find" (a few tokens). Over the 6 answers the model ACTUALLY generated in that quiet run: median 2.3 s, max 2.6 s;
  the first run right after the heavy comparison: generated median 9.8 s, max 13.8 s. The comparison's medians
  (3.2-5.2 s) were on a busy PC with the model partly on the CPU. `assistant_llm_eval.py report` and `e2e` now
  print both medians (all / generated only). Re-measured 2026-11-07 after the fixes (10 spoken Greek tuning
  questions, the real Whisper; CONDITION: another program held 2.1 GB of the GPU, so only 17 of 28 layers were on
  the GPU - slower than AM4's all-28 run): generate: all 10 median 2.34 s, max 3.48 s; the 6 generated: median
  3.04 s, max 3.48 s. extract (5d): all 10 generated, median 1.86 s, max 2.74 s (Whisper 0.84-1.08 s of it).
- LOW-2 the model file: paths.LLM_PINNED (file name -> size, sha256 = tools/get_llm.py MODELS; AS23 keeps them
  equal) + paths.llm_verify: the worker computes the sha256 of the .gguf it is about to load (1.8 s for the 2 GB
  file, once per start, in the below-normal worker) and refuses a mismatch (size or digest) = retrieval only with
  the reason on the Tools page; a file NOT in the table is refused unless the per-PC setting
  `assistant_llm_unpinned` is true (W108); `get_llm.py --check --verify` checks every present file. The paths.py
  docstring's Ubuntu hint names the pinned faster-whisper==1.2.1 (the hash lock), get_llm.py's Ubuntu hint the
  pinned wheel version + the hash-locked requirements; the get_llm docstring's download size corrected (the chosen
  model + the wheel = ~2.5 GB, not 5.3).
- LOW-3 worker.guard: after the guard moved the model to the CPU (GPU short before an answer) it tries the GPU
  again when the free memory covers the plan's need + MARGIN_MB, at most every GPU_RETRY_S 120 s, never when the
  user's setting says "cpu"; all of it in RAM (cpu_forced_t / gpu_retry_t: a worker restart plans afresh from the
  real free memory - nothing wrong can persist). The client marks an answer whose question was interrupted by
  «ΟΜΙΛΙΑ» (pressed, or held while the answer was on its way) with `speak: False`, `no_speak_why: "horn"` - it is
  SHOWN, AM5 must never speak it (test AS22).
- Measured effect of the stricter post-check on the STORED AM4 tuning runs (rescore, llama3.2-3b, generate mode):
  rejected 4/49 -> 18/49 = 37 % of the Greek and 2/35 -> 9/35 = 26 % of the English generated answers; Greek
  correct 22 % -> 17 %, English 67 % -> 48 % (a rejected answer now counts as "can't find", not as the first
  sentence's luck). Among the new rejections are REAL errors the old check let through («Ναι, η μηχανή σβήνει αν
  είναι κάποιος στο σκάφος» - the manual says never in CREWED; «Ο μοχλός TWCS στο AUTO κάνει να αλλάξει το
  τιμόνι» - the manual says it never adds power) and paraphrases ("the drones can fly up to 120 m" vs the manual's
  "altitude 10..120 m"): W109.

### 5d. Answer mode "extract" vs "generate" (step AM4-r; the review's design, evaluated)

The review proposed: the model only PICKS passage sentences by number and they are spoken WORD FOR WORD. Built as
the per-PC setting `assistant_answer_mode` (client.DEFAULT_ANSWER_MODE; env TRYGONS_ASSISTANT_ANSWER; the Tools page
shows the mode): answer.prompt_pick numbers the WHOLE sentences (a capital / digit / quote first, brackets balanced,
>= 4 words) of the top 3 passages (<= 30) [CORRECTED in AM4-r2, 5e: in AM4-r the splitter cut a sentence at every
';' and '·' too, so a pick could be the FIRST HALF of a sentence whose "never / only / nothing but" clause followed
the ';' - 452 such picks over both indexes; the lower-case second halves were not offered. Now a sentence ends only at
. ! ? or the Greek question mark], the instruction
says "write ONLY the numbers of the one or two sentences that answer, e.g. 4 or 4, 9, else exactly the can't-find
sentence"; _decide_pick accepts "4" / "4, 9" / "4 και 9" / an echoed "4. <the sentence itself>"; any other text,
or numbers outside the list, = nothing is guessed (can't find said + the passage shown); two sentences longer than
MAX_PICK_WORDS 45 together = the first only; the post-check still runs as the backstop (a manual sentence's own "see
chapter 9" is allowed). Compared on the TUNING part with the real worker (`assistant_llm_eval.py run --answer
extract`; llama3.2-3b; the same stricter post-check for both; the held-out part untouched):

| llama3.2-3b, tuning | lang | correct | answered | precision (correct of answered) | false can't find (the model's own) | post-check rejected | quality proxy | median ms (question -> text) |
|---|---|---|---|---|---|---|---|---|
| generate (AM4) | el | 16/96 = 17 % | 38/96 = 40 % | 16/38 = 42 % | 58 (40) | 18/49 = 37 % | 27/31 = 87 % | 5187 (AM4's busy run) |
| **extract** | el | 35/96 = 36 % | 87/96 = 91 % | 35/87 = 40 % | 9 (1) | 8/97 = 8 % (the model wrote text instead of a number) | 79/89 = 89 % | 803 |
| generate (AM4) | en | 16/33 = 48 % | 25/33 = 76 % | 16/25 = 64 % | 8 (0) | 9/35 = 26 % | 26/26 = 100 % | 3223 |
| **extract** | en | 16/33 = 48 % | 33/33 = 100 % | 16/33 = 48 % | 0 (0) | 0/34 | 28/34 = 82 % (long sentences) | 427 |
| none (the search's shortest sentence) | el / en | 21/96 = 22 % / 4/33 = 12 % | 96 / 33 | 22 % / 12 % | 1 / 0 | - | - | 0 |

(extract ran with 17 of 28 layers on the GPU - another program held 2.1 GB; not-in-the-manual questions: extract
said can't find 5/7 el, 4/5 en; generate 7/7, 4/5.)

READ IT HONESTLY: "correct" = the must-mention term is in the spoken text, and the term is by construction in the
passage the search found - a verbatim sentence that contains the question's words is "correct" even when it does
not answer the question (CIRCULAR proxy; e.g. "πώς σταματάω την εκτόξευση" -> the picked sentence about the hatch
relay lamp). The human-style judgement of the picked sentences is the owner's (W99's blind sheet regenerated with the
extract answers as one more letter). What the numbers DO say: (1) extract never invents a WORD - a name, number or
negation word that is not in the manual cannot appear in a verbatim sentence (0 / 8 rejections were post-check
problems; the 8 Greek ones were the model writing text instead of a number = can't find) - BUT (AM4-r2) a CUT clause
can still mislead: in AM4-r the pick could be the first half of a "...; never ..." sentence (fixed, 5e), and an
off-target true sentence can be the answer to a question the manual does not answer (3 of 12 off-manual tuning
questions got one: W112, the score floor); (2) extract answers much more (91 / 100 %) and its
false "can't find" are few (9 / 0 vs 58 / 8); (3) Greek correctness doubles (36 vs 17 %) at the same precision (40 vs
42 %); English correctness is equal (48 %) at a LOWER precision (48 vs 64 %) - the model picks the keyword sentence,
not the answer, and says so for every question; (4) 4-6x faster (7 tokens median: the number); (5) the quality
proxy penalises only length (the manual's long sentences, > 40 words). Spoken, a manual sentence out of context can
be long or oblique ("Η ξενάγηση έχει κεφάλαιο «Το μαγγέμα UAV».") - the chapter is always shown.

CHOICE (W106 asks the owner to confirm): DEFAULT = "extract". Safer: no word that is not in the manual can be said,
and the lexical limits of 5c (a swap inside one sentence, the adjacent-word number check, the negated-word check)
cannot bite on verbatim text. Not worse: Greek better, English equal on correctness (lower precision = more
off-target TRUE manual sentences - a true sentence that does not answer the question is still a wrong answer to
the operator, which is why 5e adds a score floor). "generate" stays one setting away for a PC where English matters
most. Krikri / Meltemi were not run in this mode (they do not fit beside Whisper on 6 GB: W107). [AM4-r2 corrected
this paragraph: it said "nothing invented is possible" and "never a false fact" while the AM4-r picks could be a cut
half-sentence - see 5e.]

### 5e. Review fixes of AM4-r (step AM4-r2, review of 941755a; station / tools / tests / docs only)

Where this section differs from 5c / 5d, THIS wins.

- MED-1 CLAUSE FRAGMENTS. answer.sentences() split at ';' and '·' too, so a "whole" pick could be the FIRST HALF of a
  manual sentence and the clause after the ';' - often the "never / only / nothing but" part - was dropped (its
  lower-case start kept it out of the list): MANUAL.txt:1278 "TRAINING only from STANDBY by the PC;" lost "nothing
  but STANDBY until the outputs are armed". Over BOTH committed indexes 452 pickable pieces (201 el / 251 en) ended in
  ';' / '·'; 145 of them were followed by a lower-case clause with a negation or only / μόνο. Now a sentence ends ONLY
  at . ! ? or the Greek question mark ';' (a piece with more Greek than Latin letters before it, a capital / digit /
  quote after it); an English ';' and the Greek '·' (ano teleia) join the clause to its sentence; a piece after an
  abbreviation ("e.g.", "π.χ.", "κεφ.", "μέγ.", "...") that starts lower-case is joined too. pick_sentences never
  offers a piece that ends in ';' / '·' (a passage the index builder cut in the middle of a '·'-list), and offers a
  "* bullet" item without its bullet (a list item IS a manual sentence; before, 252 of them were silently out).
  Every ". ! ; · ?" test in the code (a cut generation's whole sentences, the spoken terminal mark) went through
  ends_sentence(). Side effect, stated: passage sentences are LONGER now, so (a) the 5c "number in ONE passage
  sentence with its neighbour" check is a little weaker across a former ';' (two clauses are one sentence), (b)
  the eval's "quality" proxy (<= 40 words) drops for English (joined sentences). Test AS25 runs over the WHOLE
  index: 3119 picks, none ends in ';' / '·', none starts lower-case, none is a cut clause (a pick whose piece did not
  end a sentence followed by a lower-case never / only / not clause), the reviewer's example is ONE pick; the
  mutation (the AM4-r splitter) fails it (hundreds). The split lives in answer.py, not in index.py - the index files
  were rebuilt + committed in this step only because MANUAL.txt / MANUAL_EL.txt changed (the gate's first run caught
  the stale index: every search "can't find").
- MED-2 NEGATION WINDOW. negated_shared looked 2 raw tokens after the negation: «Ο υπολογιστής μπορεί να παρακάμψει
  το STANDBY.» passed against «...δεν μπορεί ποτέ να το παρακάμψει» (μπορεί ποτέ να το = 4 tokens) and "The STANDBY
  button always wins and the PC can override it." passed because only the BEST-matching passage sentence ("The
  STANDBY panel button always wins.") was checked. Now the window counts CONTENT words (stop words, units, other
  negations - "μπορεί", "να", "το", "can", "ποτέ" - are skipped; NEG_MIN_SHARED 2 content words), and EVERY passage
  sentence ABOUT THE SAME THING is checked - the best match(es) as before, plus every sentence sharing >= 2 content
  words AND >= NEG_MIN_FRAC 40 % of the answer sentence's. Measured on the stored llama3.2-3b generate runs (tuning):
  "any sentence sharing one word" refused 11 English / 8 Greek answers the old rule kept, most of them TRUE ("Hold
  the HATCH CLOSE button." vs "without a station holding HELM nothing can switch it"); the 2 + 40 % rule refuses 4 /
  3 and both probes. Both probes are tests (AS19). Generate re-scored with it: el correct 17 % (16/96, as before),
  rejected 20/49 = 41 % (37 %); en correct 13/33 = 39 % (48 %), answered 61 % (76 %), rejected 14/35 = 40 % (26 %) -
  the honest price of the window + the longer sentences. In the EXTRACT mode a verbatim pick is EXEMPT from "another
  passage sentence negates a shared word" (the pick IS the manual's text; the rule refused 6 / 34 English and 8 / 97
  Greek true manual sentences in the first AM4-r2 run) - like the "see chapter 9" exemption; and a '/'-joined capital
  term of the manual ("ΚΙΤΡΙΝΟ/ΠΟΡΤΟΚΑΛΙ") is now found in the passages (it refused a verbatim el sentence).
- MED-3 docs: 5c now says the number / negation checks are LEXICAL (the adjacent word, not the claim).
- LOW-a two picks from DIFFERENT passages: the cite names BOTH chapters ("Κεφάλαιο 16 «...» + Κεφάλαιο 12 «...»"),
  the answer carries `passage2`, the Tools page prints both passages. LOW-b thousands separators: "1,500 m" /
  "1.500 m" = 1500 m (a group of exactly 3 digits after the separator, 1-3 digits before, not "0."), never 1.5 m;
  "1500" also matches "1,500" in a passage sentence; "0.125 s" stays a decimal. LOW-c a CUT extract output ("1" when
  the generation stopped at max_tokens / the time) is the pick - the whole-sentence filter is for generate only.
  LOW-d paths.llm_verify caches (abs path, size, mtime_ns) -> sha256 per process: the GPU guard's CPU / GPU reloads
  hash nothing again ("sha256 verified (cached)"); a changed file is hashed (AS23). LOW-e AS24 requires the model's
  OWN answer (src llm / extract; a "passage" fallback = FAIL); AS21 adds garbage outputs ("asdf", "7.", a wrong echo
  "1. Something else.", "I think 1 is right", "one", "...", a fragment of a pick, "1 Πατήστε ...") = can't find + the
  passage, an empty output = the retrieval form.
- W106 RE-MEASURED (extract, llama3.2-3b, the real worker, the SAME stricter post-check; 17 of 28 layers on the GPU as
  in 5d):

| llama3.2-3b extract | part | correct | answered | precision | false can't find | post-check rejected | can't find right | quality (<= 40 words) | median ms |
|---|---|---|---|---|---|---|---|---|---|
| AM4-r (5d, cut clauses) | el tuning | 35/96 = 36 % | 91 % | 40 % | 9 | 8 % | 5/7 | 89 % | 803 |
| **AM4-r2** | el tuning | 41/96 = 43 % | 90/96 = 94 % | 41/90 = 46 % | 6 (the model's own 1) | 5/97 = 5 % (text instead of a number) | 5/7 | 74/92 = 80 % | 828 |
| AM4-r (5d) | en tuning | 16/33 = 48 % | 100 % | 48 % | 0 | 0 % | 4/5 | 82 % | 427 |
| **AM4-r2** | en tuning | 15/33 = 45 % | 33/33 = 100 % | 45 % | 0 | 0 % | 4/5 | 24/34 = 71 % | 454 |
| **AM4-r2 HELD-OUT, once** | el | 18/38 = 47 % | 34/38 = 89 % | 18/34 = 53 % | 4 (1) | 3/37 = 8 % | 4/4 | 29/34 = 85 % | 891 |
| **AM4-r2 HELD-OUT, once** | en | 7/9 = 78 % | 9/9 | 78 % | 0 | 0 | 1/1 | 7/9 | 505 |

  (The must-mention proxy is still circular - 5d. The Greek gain comes from the longer sentences holding more of the
  question's words, not from a better model; the English loss (16 -> 15) is one question whose joined sentence
  changed the pick. "quality" fell because joined sentences are longer than 40 words more often.)
- W112 THE SCORE FLOOR. Extract still answered 3 of the 12 off-manual tuning questions with a true but off-target
  sentence: el016 «η καταπακτή κόλλησε στη μέση» (a manual gap, best BM25 score 7.10), el143 (6.07), en047 "what mode
  am I in right now" (6.79, live telemetry - W82). Their scores lie INSIDE the range of the correct answers (Greek
  correct from 5.31 up, 10th percentile 7.78; English from 11.6): a floor is a weak separator. Built: EXTRACT_FLOOR
  7.2 (answer.py; client.DEFAULT_EXTRACT_FLOOR; the per-PC setting `assistant_extract_floor`, 0 = off; the worker's
  "extract_floor"; eval `run --floor`): below it nothing is picked, «Δεν βρίσκω ...» is said and the best passage is
  STILL shown. Effect (computed from the stored scores, no re-run needed): tuning - refuses all 3 off-manual answers;
  costs 3 of 41 correct Greek answers (and 5 wrong ones: 90 -> 82 answered), no English one; held-out (once) - the
  off-manual questions were refused already; costs 0 Greek and 1 of 7 correct English answers (34 -> 32 / 9 -> 8
  answered). Set just above the highest of 3 cases: W112 asks the owner (recommendation: ON at 7.2, the W102 rule;
  re-tune on the W84 real questions). Floor table (tuning): 6.5 refuses 1 of 3, costs 3 el correct; 7.0 refuses 2,
  costs 3; 7.2 refuses 3, costs 3; 8.0 refuses 3, costs 5 el.
- test_driver "HUD not rendered in the rig" (AM4-r note): CAUSE FOUND in AM4-r2 - the 4 failures (I4 / I12 / I19 /
  I23) appear when test_driver.py runs under a Python WITHOUT Pillow (the Windows Store `python` 3.15 on this PC has
  no PIL: the HUD renderer needs it, so nothing is drawn); with the gate's C:\msys64\ucrt64\bin\python.exe (Pillow
  installed) test_driver passed 24/24 today. Not a code problem.

### 5f. Review fixes of AM4-r2 (step AM4-r3, review of 0ef4bc4; station / tools / tests / docs only)

Where this section differs from 5c / 5d / 5e, THIS wins.

- MED-1 THE EXCEPTION THAT FOLLOWS. A verbatim pick is the manual's own words but could leave the NEXT sentence
  out - the one that limits it: «... είναι βοηθήματα.» -> «ΔΕΝ είναι για ναυσιπλοΐα.», the AUTO RETURN conditions
  -> "NEVER out of REMOTE MANUAL / MANEUVER", "... the files." -> "NEVER edit them." (the reviewer counted 105 el /
  140 en such picks; with this step's own definition 128 / 158). Now answer.pick_sentences offers such a sentence
  only TOGETHER with the limiting sentence(s) that follow it - one unit of byte-identical CONSECUTIVE manual text
  (answer._units: the pieces are joined with the text between them, a leading bullet taken off the unit only) - or
  not at all: answer.limits() = the next sentence has a LIMITING word (NEG_WORDS + only / μόνο / except / unless /
  εκτός / exclusively / solely, LIMIT_RE) AND shares >= PICK_LIMIT_SHARED 2 content words with the unit, OR is a
  SHORT one (<= PICK_LIMIT_SHORT 10 words) that OPENS with the limiting word («ΔΕΝ είναι για ναυσιπλοΐα.», "Never
  edit them.": no subject of its own = it speaks about the sentence before). The joining continues while the next
  sentence limits the growing unit; a unit over PICK_JOIN_WORDS 60 words is NOT offered (neither the statement nor
  the pair - the limiting sentence alone stays offered; the passage is shown anyway), a limiting piece that is
  itself cut (ends in ';' / '·') makes the unit impossible = not offered. Measured over both committed indexes:
  186 el / 189 en pickable statements are followed by a limiting sentence (this definition, the short-opener rule
  included); at cap 60: 116 / 119 joined units offered (21 / 34 of them longer than speech.LONG_WORDS 50 = shown
  whole, spoken as the chapter intro only - W113), 70 / 70 statements NOT offered (the pair too long); cap 45: 86 /
  73 joined, 100 / 116 not offered; cap 100: 164 / 181 joined (69 / 96 over 50 words), 22 / 8 not offered; no cap:
  175 / 189 joined (80 / 104 over 50), 11 / 0 not offered (cut pieces) - W116 asks the owner about the cap.
  The exemption of 5e ("a verbatim pick is not refused because ANOTHER manual sentence negates a word of it") is
  KEPT - the exception that follows is now INSIDE the pick, so the exemption no longer hides it; the exemption
  still protects a true sentence from an unrelated negated one elsewhere in the passage. Test AS25 runs over the
  WHOLE index: every pick is a byte-identical substring of its passage, NO pick is followed by a limiting sentence
  about the same thing that it leaves out (0 of 2987 picks), the reviewer's «... είναι βοηθήματα. ΔΕΝ είναι για
  ναυσιπλοΐα.» is one pick; the mutation (the AM4-r2 pick list, nothing joined) leaves 290 exceptions out = FAILS.
  Unit tests AS21: the pair, the chain, the cap, the short «ΔΕΝ ...», an unrelated negation that does NOT travel,
  the same verbatim sentence of 5e now comes WITH its following «Χωρίς HELM κανείς δεν πατά ...».
- MED-2 THE "SAME THING" RULE PER CLAUSE (generate mode). "In AUTO the boat follows the waypoint route at the cruise
  speed of each leg and the PC can override the STANDBY button." passed: the 40 % was measured against the WHOLE
  sentence's 12 content words, so the negating "The PC can never override it." (2 shared) was not "about the same
  thing"; and "The PC is never allowed, in any mode or situation, to override it." escaped the 2-content-word
  window. Now answer.postcheck judges the whole sentence AND each of its CLAUSES with >= NEG_CLAUSE_MIN 3 content
  words (answer.clauses: split at and / but / or / και / κι / αλλά / όμως and at , ; · - never inside "1,500"; a
  2-word fragment like "such as: where do I close the hatch?" is no claim of its own and is judged only as part of
  its sentence - the first try judged every clause and refused the tour text's own first sentence, AS11): a passage
  sentence is about the same thing when it shares >= NEG_MIN_SHARED 2 content words that are >= NEG_MIN_FRAC 40 %
  of the unit's content words OR of the NEGATING passage sentence's own; a negated clause needs a negated top
  passage sentence of its own; and negated_shared skips a comma INSERTION after the negation (", in any mode or
  situation," = one hop; a comma without a closing comma is not skipped). The reviewer's probes (el + en, the joined
  sentence, the insertion, both together) are AS19 tests; true two-clause sentences pass. The stored llama3.2-3b
  GENERATE tuning runs re-scored (`rescore`) with the final rule: el correct 17/96 = 18 % (5e: 17 %), rejected 21/49
  -> 21/56 = 38 % of the generated rows (5e: 41 %); en correct 12/33 = 36 % (5e: 39 %), rejected 15/35 = 43 % (40 %)
  - one more TRUE English answer refused by the lexical rule, one Greek one back (the W109 price; moot in the
  default extract mode). The checks stay LEXICAL (5c).
- MED-3 / LOW-a LONG PICKS. AM5 made a pick over speech.LONG_WORDS 50 intro-only when SPOKEN (W113). Now a SHORTER
  way is OFFERED where one exists: a single piece longer than LONG_PICK_WORDS 50 that is a GLUED "* bullet" list
  (the manual's list items have no full stops, so the index builder's sentence split kept them as one "sentence")
  is offered item by item (answer._ITEM_RE, each item byte-identical manual text, the limiting rule applies among
  the items, items joined with their " * " between them); a genuinely long sentence (the 112-word HUD description,
  comma lists) stays ONE pick - shown whole, spoken as the intro. Before: 55 el / 83 en single picks over 50
  words; now 35 / 61 (+ the joined units over 50 of MED-1), 0 of them a glued list (AS25).
- LOW-b THE FLOOR SETTING + A GATE. client.py's comment said "0 = none, the measured default" (read as "0 is the
  default"): the DEFAULT is 7.2; a `null` / missing `assistant_extract_floor` = the default (client._floor,
  answer.Answerer: None / "" = EXTRACT_FLOOR), only an explicit 0 switches the floor off (MANUAL / MANUAL_EL say
  so). The floor was tuned on BM25 scores that MOVE with every manual / parser change (the IDF of every word): a
  new gate `python tools/assistant_index.py floor` (tools/check.sh after the eval; test AS29) keeps the frozen
  separation of tests/assistant/floor.json - on the TUNING part only: the known off-manual questions under the
  floor (el 7 / en 5 ids), the known in-manual questions at least FLOOR_MARGIN 0.5 over it (83 / 31 ids), the
  COUNT of in-manual tuning questions over it (86 / 32; a fall by more than FLOOR_DRIFT 2 fails), and the code's
  floor (answer.EXTRACT_FLOOR = client.DEFAULT_EXTRACT_FLOOR) = the tuned one in the file. A failure names the
  question that crossed and says how to re-tune (W117: `assistant_llm_eval.py run --answer extract --floor F` on
  the tuning part, set the two constants, `floor --write` freezes the new separation; never on the held-out
  questions). ONE floor for both languages (AM4-r4 LOW-2 corrected this paragraph: the file holds one "floor" and
  the code one constant; per-language floors are NOT supported - the English in-manual scores start at 7.0, the W112
  note stands; AS29 checks the file has no per-language floor). AS29 proves the four mutations fail.
- LOW-c paths.llm_verify's cache key = (abs path, size, mtime_ns, st_ino, st_ctime_ns): a file replaced by another
  of the same size and mtime (a copy with `cp -p`) has another inode / change time and is hashed again. AM4-r4
  LOW-5, stated: on Windows st_ctime_ns is the file's CREATION time (not the inode change time), so a file replaced
  in place keeps it; the cache is per PROCESS (a dict in paths.py), so the first load of every worker start hashes
  the file again anyway (~2 s) - the key only saves the GPU guard's reloads inside one run.
- LOW-d "1.500 m" / "1,500 m" SPOKEN. 5e made the checker read "1.500" as 1500 (a thousands group), so an English
  generate answer "1.500 m" for the manual's "1500 m" passes - and the voice would have read "one point five
  hundred metres". speech.normalize now turns a thousands group (1-3 digits, a separator, EXACTLY 3 digits, no more
  digits) into the plain digits "1500" before the TTS, in both languages and both answer modes (the Greek manual's
  own "1.500 m" is read right too); "0.125 s" / "1.5 s" / "2,5 s" stay decimals (AS26).
- LOW-e AS24 (the real model, network blocked) no longer depends on the floor: it asserts first that each question
  scores >= EXTRACT_FLOOR + 2 (14.9 el / 12.8 en today) - a floor "can't find" can never look like a model failure.
- PILLOW EARLY. groundstation/test_driver.py and tests/acceptance/fresh_build.py stop at once with "this Python has
  no Pillow - run with C:\msys64\ucrt64\bin\python.exe (tools/check.sh) / set PYTHON=... (RUN_ACCEPTANCE.bat)" when
  PIL is missing (the AM4-r gate lost hours to misleading HUD failures; fresh_build runs every clone suite with
  sys.executable).
- THE "REJECTED" DIVISORS 97 / 37 vs 96 / 38 (5e's table): explained, not a mistake - "post-check rejected" is
  counted over EVERY row the model actually generated for (src llm / extract / passage), the off-manual questions
  INCLUDED (the model answered 2 of the 7 off-manual Greek tuning questions = 95 in-manual + 2 = 97; held-out: 37 of
  the 38 in-manual rows generated + 0 of the 4 off-manual = 37), while "correct" / "answered" count the in-manual
  questions (96 / 38). The report's column says so now.
- OWNER QUESTIONS of the review -> W116 (the join cap 60 / the verbatim exemption kept with the exception inside the
  pick), W117 (who re-tunes the floor = the gate above, on the tuning part; recommended default: the step that
  changes a manual re-tunes when the gate fails), W118 (a 100+ word sentence as a spoken answer = NO: intro only,
  as AM5 built it - W113 keeps the 50).

#### Re-measured (AM4-r3, llama3.2-3b, the real worker; tuning part for both modes, held-out ONCE at the end)

The real worker (`assistant_llm_eval.py run`), 17 of 28 layers on the GPU (another program held 2.1 GB, as in 5d /
5e), the floor 7.2 IN FORCE for extract (5e's AM4-r2 row was measured without it and the floor's cost computed from
the stored scores), the joined picks of MED-1, the per-clause check of MED-2 for generate; "rejected" = of every row
the model generated for, off-manual questions included:

| llama3.2-3b | part | correct | answered | precision | false can't find (model's own) | post-check rejected | can't find right | quality (<= 40 words) | median ms |
|---|---|---|---|---|---|---|---|---|---|
| **extract** (5e AM4-r2: 43 % / 94 % / 46 % / 6 / 5 % / 5/7) | el tuning | 38/96 = 40 % | 80/96 = 83 % | 38/80 = 48 % | 16 (10) | 6/86 = 7 % | 7/7 | 65/80 = 81 % | 820 |
| **extract** (5e: 45 % / 100 % / 45 % / 0 / 0 / 4/5) | en tuning | 16/33 = 48 % | 32/33 = 97 % | 16/32 = 50 % | 1 (1) | 0/32 | 5/5 | 17/32 = 53 % | 446 |
| generate (5e re-scored: 17 % / rejected 41 %) | el tuning | 16/96 = 17 % | 36/96 = 38 % | 16/36 = 44 % | 60 (39) | 21/56 = 38 % | 7/7 | 31/35 = 89 % | 1313 |
| generate (5e re-scored: 39 % / 40 %) | en tuning | 12/33 = 36 % | 19/33 = 58 % | 12/19 = 63 % | 14 (0) | 15/35 = 43 % | 4/5 | 20/20 | 1200 |
| **extract HELD-OUT, once** (5e: 47 % / 89 % / 53 % / 4 / 8 % / 4/4) | el | 14/38 = 37 % | 30/38 = 79 % | 14/30 = 47 % | 8 (3) | 5/35 = 14 % | 4/4 | 26/30 = 87 % | 3089 (the PC busy) |
| **extract HELD-OUT, once** (5e: 7/9) | en | 7/9 = 78 % | 8/9 = 89 % | 7/8 = 88 % | 1 (1) | 0/8 | 1/1 | 7/8 | 487 |
| generate HELD-OUT, once | el | 2/38 = 5 % | 11/38 = 29 % | 2/11 = 18 % | 27 (13) | 14/25 = 56 % | 4/4 | 10/11 | 1347 |
| generate HELD-OUT, once | en | 5/9 = 56 % | 8/9 = 89 % | 5/8 = 62 % | 1 (0) | 1/9 | 1/1 | 8/8 | 3425 |

READ IT HONESTLY: (1) the floor now REALLY refuses every off-manual tuning question (7/7 el, 5/5 en; 5e: 5/7, 4/5)
and costs what 5e computed - Greek extract correct 43 -> 40 % on the tuning part, answered 94 -> 83 %, false
"can't find" 6 -> 16 (the floor's 10 + the model's own); held-out Greek 47 -> 37 % (34 -> 30 answered, false can't
find 4 -> 8): the floor was set on 3 cases and takes true answers with it (W112 stands: re-tune on the W84 real
questions; the gate of LOW-b keeps the separation it was tuned on). (2) The joined picks did not lower precision
(48 / 50 % vs 46 / 45 %) but the "quality" proxy (<= 40 words) fell for English (71 -> 53 %): a statement + its
exception is longer; 30 of the 112 kept picks (both languages, tuning) are over 40 words - the voice says the
chapter intro for those over 50 (W113 / W118). (3) The post-check rejected 6 Greek extract picks (7 %):
the model wrote text instead of a number, as before - not a problem of the picks. (4) Generate with the per-clause
rule: Greek 17 % (as before), English 36 % (5e re-scored 39 %; 5d 48 %), rejected 38 / 43 %; held-out Greek
generate 5 % (2/38; 56 % of its generated answers rejected) - the lexical checks refuse most of what a 3B model
writes in Greek: one more reason for the extract default (W106). (5) Latency: the GPU was shared, as in 5e; the
held-out Greek extract median 3.1 s came from a busy PC (the heavy generate run ended just before it), the tuning
runs 0.8 / 0.45 s. The must-mention proxy stays circular (5d); the FRESH question set of AM6 is still needed.

### 5g. Review fixes of AM4-r3 (step AM4-r4, review of f3b0964; station / tools / tests / docs only)

Where this section differs from 5c-5f, THIS wins.

- MED-1 THE LIMIT RULE WAS CIRCULAR AND INCOMPLETE. AS25 proved "no pick leaves its limiting sentence out" with the
  SAME answer.limits() that builds the picks, inside ONE passage only. The reviewer counted what it missed: (a) the
  PASSAGE BOUNDARY - a pick that ends its passage while the next passage of the same chapter opens with the limit
  (19 en / 10 el; el:267 «... μετά από 5 s.» -> «όχι συναγερμός, τίποτα άλλο δεν αλλάζει.», en:189 "tab Bench." ->
  "Only the laptop on the USB cable ..."); (b) a following sentence that opens with a PRONOUN and carries the limit
  but shares < 2 content words (42 en; "YES is pre-selected, so the trigger alone = a fast ACCEPT." -> "It counts
  only when ... shown >= 0.5 s and only as a NEW press"); (c) a limit TWO sentences later sharing >= 3 words (19 en /
  23 el) and an "Except ..." sentence BEFORE the pick. Now answer.pick_units builds the units over the CHAPTER text:
  index.following() gives every search hit its "after" = the next FOLLOWING_N 2 manual passages of the same chapter
  (consecutive line ranges; the worker's found hits carry them, AS22), the passage and its following passages are
  ONE text for the sentence split (so a sentence the index builder cut at the boundary - «... κρατά τη θέση του·»
  + «για 3 s ...» - is WHOLE again, no cut piece any more) and the chain of limiting sentences runs on into the
  next passage; a unit that reaches into it shows that passage as passage2 (the same chapter cited once; its text
  counts for the post-check). answer.limits(): + a sentence opening with a PRONOUN (PRONOUN_OPENERS: it / this /
  that / these / those / they / αυτό / αυτή / αυτός / αυτά ... / έτσι) that carries a limiting word limits the
  sentence before it whatever the shared words; the shared-word count uses limit_words() = a SMALLER stop list
  (LIMIT_STOP: "always / only / every / μόνο / πάντα" stay content words). limits2(): a limiting sentence TWO
  sentences after the unit sharing >= PICK_LIMIT_SHARED2 3 content words takes the sentence between along.
  excepts(): a sentence BEFORE the pick opening with except / unless / εκτός that shares >= 2 content words begins
  the unit. The cap PICK_JOIN_WORDS 60 refuses the WHOLE unit (the statement is not offered alone - never a cut; the
  limiting sentence alone still is). A defined TERM passage (a definition-list line, often without a full stop) is
  never chained to (index.following stops at a term on either side): the first build of this step glued consecutive
  alarm definitions into one "sentence" and the post-check then refused 3 verbatim picks (el054 / el089 / el098 in
  the first re-measure run) - found and fixed before the final run. Measured over both committed indexes (manual
  passages): 2178 picks with the AM4-r3 rules on the chapter text -> 2014 with the AM4-r4 rules (the look-ahead 2
  alone costs 155 picks, the pronoun rule 7, the stop list 6, "Except" before 0); 205 joined units, 84 units
  reaching into the following passage (among them the formerly CUT boundary sentences made whole: before, they
  were not offered at all). AS25
  now has its OWN wider rule (I_* inside the test: own fold / stop list / limit-word list / stemming; only the
  numeral table answer.digitize is shared, so «Δύο σταθμοί» / «στις ΔΥΟ οθόνες» is not a shared content word), over
  the chapter text, 1-2 sentences ahead, the pronoun opener, "Except" before: REMAINING MISSES = 0 (both languages;
  a miss would have to be listed + justified in tests/assistant/as25_misses.json - an unlisted or stale entry
  fails). The AM4-r2 list misses > 200, the AM4-r3 rules > 40 (the reviewer's cases) = the mutations fail. Tests
  AS21 (the pronoun / look-ahead / "Except" / boundary / cut-at-'·' / cap / no-"after" / term-stop cases), AS25
  (14 checks).
- MED-2 THE 2-WORD CLAUSE RELAXATION. NEG_CLAUSE_MIN 3 (AM4-r3, for AS11's tour sentence) let "In AUTO the boat
  follows the waypoint route ... of each leg and the PC overrides it." pass against "The PC, the gateway, the copilot
  station and the shore station can never override it." (the 2-word clause "the PC overrides it" was not judged).
  Now a 2-content-word clause IS judged when ALL its content words (100 %) are shared with a NEGATING passage
  sentence; a clause ending in '?' is a question = no claim (the tour text's "such as: where do I close the hatch?"
  stays: AS11). "The PC overrides it." stays refused. AS19 probes.
- MED-3 THE HELD-OUT DROP, HONESTLY. 5f / TEST_REPORT attributed the Greek held-out drop 18 -> 14 correct and the
  false can't-finds 4 -> 8 to the floor. Per question (the AM4-r2 run log vs the stored AM4-r3 rows, el held-out, 38
  in-manual): 7 LOST, 3 GAINED. Of the 7 lost: 0 by the FLOOR (the 3 held-out questions under 7.2 - el063 5.20,
  el066 2.88, el117 6.85 - were WRONG already in AM4-r2: the floor turned 2 wrong extract answers + 1 can't-find into
  can't-finds, cost 0 correct); 1 by the JOINED PICKS / length cap (el080: the same first sentence, now a longer
  unit, and the second pick dropped by MAX_PICK_WORDS 45 - the must term went with it); 2 by the POST-CHECK (el077,
  el098: src passage - the model wrote text instead of a number); 4 by the MODEL RUN (el025, el101, el108, el122: a
  different sentence picked from a renumbered list; the 3 gained - el003, el041, el050 - the same way). The false
  can't-find rise 4 -> 8: the floor +2 (el063, el117), the post-check +2. So the drop was mostly run-to-run variance
  of a 3B model on a renumbered list plus the length cap, NOT the floor. STATED PLAINLY: the held-out set has now
  been used THREE times (the AM3-r choice check, the 5e floor choice / 5f reporting, and this step's re-measure):
  its numbers are OPTIMISTIC and are no longer a held-out measurement; a FRESH set (the owner's W89 / W84 questions)
  is for AM6.
- LOW-1 `assistant_index.py floor --write` is REFUSED while any off-manual TUNING question scores >= the floor (they
  are listed; floor_blockers); `--force` writes anyway and records them in floor.json "off_manual_over" (AS29: none
  today, a floor 2 points lower would be refused).
- LOW-2 one floor (above, 5f corrected). LOW-3 answer.floor_value / client._floor: a bool (True = 1.0 before), a
  NEGATIVE number (max(0, -1) = 0 switched the floor off silently), NaN or garbage = the DEFAULT 7.2; only an
  explicit 0 = off (AS21; MANUAL / MANUAL_EL). LOW-4 _decide_pick drops a picked id whose text is already INSIDE
  another chosen pick (the model picked the joined unit AND its limiting sentence: the unit alone, never said
  twice; AS21). LOW-5 above (5f LOW-c).
- GENERATE MODE = BEST EFFORT, NON-DEFAULT. The reviewer's remaining bypasses (double negation, a negation after the
  word, unless / only / μόνο, nominalised forms) cannot be made lexically airtight. Done where cheap: an answer
  clause with only / unless / except / μόνο / εκτός needs a top supporting passage sentence with a limiting word
  (like the negation rule), and the BEST passage sentence putting only / unless / except directly before a shared
  content word (limited_shared: the first content word after it, never across a comma - «ΚΟΚΚΙΝΟ μόνο για
  συναγερμούς, ΚΙΤΡΙΝΟ για προσοχή» restricts συναγερμούς, not κίτρινο) refuses a plain answer ("the captain closes
  the hatch" vs "ONLY the captain closes the hatch"); in extract a verbatim pick is exempt from it like the negation
  rule. The stored llama3.2-3b GENERATE tuning runs re-scored: el 17/96 = 18 % correct (as before), rejected 22/56
  = 39 % (38 %); en 11/33 = 33 % (36 %), rejected 16/35 = 46 % (43 %) - one more TRUE English answer refused by the
  lexical rule. DOCUMENTED as such: the Tools page shows a CAUTION line under the model while "generate" is chosen
  (lang asst.answer_mode.generate_warn), MANUAL / MANUAL_EL say it is best effort and not the default; W122
  recommends it stays off by default. The rule question "picks crossing passage boundaries" = YES (MED-1).
- RE-MEASURED (llama3.2-3b extract, the real worker, 17 of 28 layers on the GPU as in 5d-5f, the floor 7.2 in
  force; tuning part + held-out). Said plainly: this step ran the held-out part TWICE - the first run (test_results/
  assistant_llm/am4r4/run_extract.log: el tuning 33 % / held-out 34 %) exposed the TERM-passage gluing bug (3
  verbatim picks refused by the post-check) and was superseded by the final run after the fix; the held-out set has
  therefore been used FOUR times in all (MED-3 counts the earlier two) and its numbers are optimistic:

| llama3.2-3b extract | part | correct | answered | precision | false can't find (floor / post-check) | post-check rejected | can't find right | quality (<= 40 words) | median ms |
|---|---|---|---|---|---|---|---|---|---|
| AM4-r3 (5f) | el tuning | 38/96 = 40 % | 83 % | 48 % | 16 (10 / 6) | 6/86 = 7 % | 7/7 | 81 % | 820 |
| **AM4-r4** | el tuning | 33/96 = 34 % | 80/96 = 83 % | 33/80 = 41 % | 16 (10 / 6) | 6/86 = 7 % | 7/7 | 63/80 = 79 % | 785 |
| AM4-r3 (5f) | en tuning | 16/33 = 48 % | 97 % | 50 % | 1 (1 / 0) | 0/32 | 5/5 | 53 % | 446 |
| **AM4-r4** | en tuning | 15/33 = 45 % | 31/33 = 94 % | 15/31 = 48 % | 2 (1 / 1) | 1/32 = 3 % | 5/5 | 17/31 = 55 % | 447 |
| AM4-r3 (5f) | el held-out | 14/38 = 37 % | 79 % | 47 % | 8 (3 / 5) | 5/35 = 14 % | 4/4 | 87 % | 3089 (busy) |
| **AM4-r4** | el held-out | 16/38 = 42 % | 32/38 = 84 % | 16/32 = 50 % | 6 (3 / 3) | 3/35 = 9 % | 4/4 | 28/32 = 88 % | 806 |
| AM4-r3 (5f) | en held-out | 7/9 = 78 % | 89 % | 88 % | 1 (1 / 0) | 0/8 | 1/1 | 88 % | 487 |
| **AM4-r4** | en held-out | 7/9 = 78 % | 8/9 = 89 % | 7/8 = 88 % | 1 (1 / 0) | 0/8 | 1/1 | 7/8 | 476 |

  PER QUESTION against the stored AM4-r3 rows (test_results/assistant_llm/am4r3_bak): el tuning 38 -> 33 = 9 lost /
  4 gained - of the 9 lost, 7 are the MODEL picking another sentence from a renumbered, shorter list (the proxy
  counted the old pick "correct"), 2 the joined unit / the MAX_PICK_WORDS 45 cap (the same first sentence, a longer
  unit, the second pick dropped), 0 the floor, 0 the post-check; the gains: 2 post-check rejections that are gone
  (the term fix), 2 other picks. en tuning 16 -> 15 (2 lost: another pick / a changed unit; 1 gained). el held-out
  14 -> 16 (3 gained: 2 post-check rejections gone, 1 another pick; 1 lost: an echo "1. <text>" the pick parser does
  not accept). The REJECTIONS are now the model's, not the picks': 4 of the 6 Greek tuning rejections and 1 of the
  3 held-out ones are the model echoing the prompt's EXAMPLE "4, 9" with a list shorter than 4 (shorter lists =
  the cap's refusals; W123 c: AM6 may put example numbers inside the list's length - measured on the fresh set,
  never here), the others an echoed "1. <text>" the parser could not match; ONE English verbatim pick (en030) was
  refused by the lexical number rule - the manual's own list number "1." glued to the end of a sentence ("...
  reverse 5 -> 10 -> 15): 1.") = the price of the backstop on a true sentence, stated. Latency unchanged (785 /
  447 ms; held-out Greek 806 ms on a quiet PC vs 5f's 3089 ms on a busy one). Honest reading: the wider limit rule
  costs 7.5 % of the picks and ~5 points of the circular Greek "correct" proxy on the tuning part (through shorter
  lists and the 3B model's echoes), gains 5 points on the (over-used) held-out part, keeps precision (41-50 %) and
  brings every following exception the reviewer found INTO the spoken text; the fresh AM6 set decides.

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

### 7a. As built (step AM5: speak + show; station / tools / tests / docs only, NO app / hal / gateway change)

- Code (the station side; the worker is unchanged): `groundstation/assistant/speech.py` (pure: the speech
  normaliser + `spoken(info, lang)` = the chapter intro + the normalised answer), `show.py` (pure: cut, the manual
  place of a passage from its line range, `demo_answer` for pictures / tests), `qlog.py` (the local log),
  `groundstation/asstvoice.py` (the station's voice: TTS off the Tk thread, sound.py at the lowest level, the
  temporary WAV, the voice outcome), `groundstation/manualwin.py` ([ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ]: the manual read-only at the
  lines), `sound.py` (level `ASSISTANT` = FEEDBACK - 1, `Scheduler.busy / stop_assistant`, `SoundSystem.assistant_*`),
  `app.py` (`panel_assistant`, `asst_on_answer` speaks, `asst_repeat`, `asst_open_manual`, the auto-fold),
  `page.py` ([ΞΑΝΑ] / [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ] + the voice line), `hailer._asst_horn` (stops the voice directly),
  `driver.py` (the answer 12 s on screen 1's notices). The assistant package still imports no sound / tts / app
  module (AS1 / AS2): sound.py reaches proto through alerts, so the speaking stays in `asstvoice.py` outside the
  package, which only CALLS `speech.spoken` and `qlog.write`.
- The voice: `tts.synth` (Piper / the Windows voice per `voice_engine`) makes ONE WAV per answer in the system
  temp folder (`trygons_asst_voice/`, never sound_cache/, never the log folder), at the sound setting's volume;
  the Tk tick plays it through `SoundSystem.assistant_speak` and deletes it 0.5 s after its end (a file still
  open is tried again at the next tick; a station crash's leftovers older than 1 h are removed at the next start).
  The TTS run (a subprocess inside tts.py) is a helper thread; between its end and the play the alarm / callout /
  horn checks run AGAIN (an alarm that came during the TTS = text only, the WAV removed unplayed).
- Alarm priority as built: `Scheduler.request` at level ASSISTANT never queues (busy = "busy" -> text only), has no
  repeat rule, never interrupts; any other request while it plays interrupts it (`_start(interrupt=True)` =
  `player.stop()`), the item is never put back; `SoundSystem.assistant_blocked()` = "alarm" (lamps.on(WARNING) or
  a repeating warning not acknowledged = `sched.active`) / "callout" (`sched.busy()`: something plays, waits or
  is due). The panel's voice line says which: Ειπώθηκε / Μόνο κείμενο – ενεργός συναγερμός / ... φωνητική
  ειδοποίηση / ... το μεγάφωνο / Η φωνή κόπηκε ... / Η φωνή σταμάτησε ... / χωρίς φωνή για τη γλώσσα / ο ήχος
  κλειστός. `asstvoice.AsstVoice.repeat()` = [ΞΑΝΑ] (also an answer that was text only; never while the horn is
  held).
- Long picks (AM4-r2 review MED-3): above `speech.LONG_WORDS` 50 words the voice says only «Βρήκα την απάντηση
  στο κεφάλαιο N – δείτε το κείμενο.» (kind "long"); the message line cuts the answer at `show.LINE_MAX` 160 minus
  the cite's length (min 80), the panel's answer line at 320, screen 1's notice at 120, the panel's passage at 700
  characters (a passage is <= 120 words, so nearly always whole); `show.cut` cuts at a word with "…".
- The panel "assistant" (screen 2, `self.ov.panel`, corner tuple = placed by the fit test like the relays): appears
  once a question was heard (also while THINKING), AI badge + question + answer (bold) + voice line with the cite +
  the passage(s) with their chapters + [ΞΑΝΑ] [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ] [ΚΕΙΜΕΝΟ ▾/▸]. AUTO-FOLD: when every panel is up on a
  small window (test: 1366x768 @ 125 %, 1366x658 @ 150 % with fill_everything) the layout did not fit with the full
  passage open - `App.render` then gives way step by step (`asst_auto_fold` 1 the passage folded, 2 the answer cut
  short + no cite, 3 the header only) and lays out again until it fits (measured: 1366x768 @ 125 % = 1, 1366x658 @
  150 % = 3, 1920x1080 = whole); [ΚΕΙΜΕΝΟ ▸] or a click on the collapsed header opens it anyway (`asst_force_open`:
  the overlay then only shrinks the text), a new answer tries the whole panel again. The guided tour has
  the stop s2.assistant (tools/demo_screens.py `force_assistant` puts a canned `show.demo_answer` on the station
  for the picture: no model, no TTS).
- [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ]: `show.manual_place(info)` = (lang, first line, last line) from the passage id matched in the
  answer's `found.hits` (every manual passage carries its line range since AM3-r LOW-4; a tour stop has none ->
  the button grey); `manualwin.open_at` = one Toplevel with a read-only Text of the whole manual, the lines
  highlighted and scrolled into view.
- The log `qlog.write(info, voice=why)`: one JSON line per answer (q, lang, a, src, cite, id, mode, ms, gen_ms,
  answer_ms, problems, picked, passage_id / cite, the 3 hit ids + scores, the voice outcome); never the passage
  texts or any audio. Written from `asstvoice` when the voice decision is final (spoken / long / alarm / callout /
  horn / no_voice / sound_off / failed).
- Not done (later items): lighting the named button through academy/tour_shots/shots.json (the optional idea of
  I1.5), the W85 explicit-microphone rule and the W86 chapter list on a safety question (both still WAITING).
- Tests: AS26 (the normaliser against the lang files' say-lines), AS27 (alarm priority + the voice rules + the WAV
  + the log, with a recording player and a fake TTS), AS28 (the panel, the manual window, the fit matrix with the
  auto-fold, the notices, the screenshots docs/screens/assistant/screen2_answer_*.png + screen1_answer_el.png).

### 7b. Review fixes of AM5 (step AM5-r, review of 5f15787; station / tools / tests / docs only, NO app / hal / gateway logic change)

- MED-1 (the temporary WAV leak, `asstvoice.py`): every WAV the station asks for is in `AsstVoice.files` from the
  moment its name is chosen until it is really gone. `stop()` (a new question, «ΟΜΙΛΙΑ», the station closing) takes
  a result the TTS thread already finished (the up-to-200 ms gap before the tick) and deletes it; a thread that
  finishes AFTER a stop / close sees the changed generation and deletes its own file; a delete that failed (the
  player still holds the file) stays on the list and is tried again at every tick - `_play` no longer overwrites
  the one `tmp` slot (the old file was lost when its delete had failed); `close()` deletes every owed file AND every
  `asst_<this pid>_*.wav` left in the folder; `_clean_stale` at the next start removes a crashed station's files
  older than STALE_S = 300 s (was 1 h: a clip lives at most TTS_MAX_S + its length + 0.5 s; two stations on one PC
  never share a name - the pid). Tests AS27 (k)-(n): stop in the gap, a late result after stop, a failed delete
  retried under a new answer, close with a stray file.
- MED-2 (`speech.py`): a plain "<" / ">" is spoken («λιγότερο από» / "less than", «πάνω από» / "more than"; the
  manuals say "< 250 ms", "> 1.5 kn", "> 30 %"), "≤ ≥" like "<= >=", the unit "%/s" («τοις εκατό ανά
  δευτερόλεπτο» / "percent per second"); a "<placeholder>" (trygons-gw-<σκάφος>) is read as its word. AS26 + the
  new AS30: EVERY pickable sentence of both indexes (answer.pick_sentences = what extract may say) with a plain
  "<" / ">" or "%/s" (el 6 / en 14) is spoken with the words and no "<" / ">" is left.
- MED-3 (the read-only proof covers the station side): `asstvoice.py` + `manualwin.py` (groundstation/, outside the
  package) are in AS1 (static imports: nothing banned, no process, never `import app`) and AS2 (the transitive
  module-level graph: nothing banned but the codec `proto` that alerts.py reads - proto imports `struct` + `calendar`
  only, checked; the function-level imports of the reached station modules - sound.py's Callouts -> hud, tour_texts
  -> academymenu -> gwclient / subprocess - are listed as the LIMIT: the station's own callouts and tour, never
  called by the voice; a static check that asstvoice.py touches the sound system only through `sched` +
  `assistant_*`). Runtime: AS27 runs the whole voice cycle under the station's sender traps + a blocked socket
  (nothing called), AS28 opens the manual window under them. The voice gets a NARROW view: `asstvoice.VoicePort`
  (`horn_held()` / `lang()` = the App-free AsstView's bound methods, `sound()` = the SoundSystem through a
  WEAK reference set by `App._build_sound`; the SoundSystem itself holds the App for its lamps / telemetry, so a
  strong reference would reach it) instead of the App; AS6 walks gc.get_referents from the voice too (no App;
  the negative control - a strong sound reference - is found).
- LOW-1 (`speech.py`): the letter N by its context (never a fixed meaning): "F / N / R" = «εμπρός, νεκρά, όπισθεν»,
  a lone N in a sentence about the gear («σχέση», gear, neutral, a lone F / R) = «νεκρά» / "neutral", after a
  coordinate number («37.82 N») or in a compass / heading sentence = «βόρεια» / "north", else the letter («εν» /
  "N"; "n/N" stays); KNOWN Latin capital words of the screens are said as words (SAID: OFF «οφ», ON, YES, NO, NEVER
  «ποτέ», RESET, START, WARNING ... - never spelt letter by letter; unknown ones still are, XYZ); "fwd" / "rev" =
  forward / reverse; a negative range "-10..-1 m" = «μείον 10 έως μείον 1 μέτρα», a leading minus = «μείον» /
  "minus"; negations survive (ΔΕΝ -> δεν, NEVER -> «ποτέ» / "never", NOT -> "not"). AS26.
- LOW-2 (`qlog.py`): a day's file above MAX_FILE_BYTES 5 MB continues in `YYYY-MM-DD_2.jsonl`, `_3` ...; files older
  than KEEP_DAYS 30 (by the date in the name) are deleted at the next write, oldest first; only the log's own
  `*.jsonl` names are touched. AS31.
- LOW-3: the lang texts say what is true - the assistant never RECORDS the microphone to disk (the question stays in
  memory), its own voice is a temporary clip deleted after playing; `asst.v_failed` points to the log and the log
  line carries the reason (`detail`: "tts made no file (engine ..., lang ...)", an exception's type + message, "tts
  run longer than 8 s"); the panel keeps it in `AsstVoice.detail`.
- LOW-4: `sound.read_wav(path, breathe=None)` / `sound.gain(samples, vol, breathe=None)` run their pure-Python
  loops (8-bit -> 16-bit, stereo -> mono, the linear resampling, the gain) in chunks of CHUNK 8192 samples and call
  `breathe()` between them - the TTS thread passes time.sleep(0), so a long clip never holds the interpreter for
  the whole loop (fast paths: a 16-bit mono clip at RATE is not touched, a gain of 1.0 returns the array itself).
  The W114 auto-fold takes ONE step per `App.render` (one layout per frame): the next step runs in the following
  idle frame (`_fold_render`, `after_idle`, never two queued), never three layouts inside one render call; unfold =
  the next answer (`asst_on_answer` resets the fold), [ΚΕΙΜΕΝΟ ▸] or a click on the collapsed header. AS27 (q),
  AS28 (one step per render, the frames after it reach the fit at 1366x658 @ 150 %).
- Owner questions (PROGRESS.md): W119 the assistant speaks while only a CAUTION lamp is lit (recommended: yes -
  only a WARNING lamp / a sounding or queued callout silence it, as built); W120 a temporary TTS WAV in %TEMP% under
  "no audio persisted" (recommended: yes - deleted at once, stale cleanup 5 min); W121 the voice is dropped when
  the TTS takes longer than TTS_MAX_S 8 s (built now: text only + [ΞΑΝΑ]).

### 7c. Review fixes of AM5-r (step AM5-r2, review of 9516ecb; station / tools / tests / docs only, NO app / hal / gateway logic change)

- MED-1 (SAFETY MISREADING, `speech.py`): the AM5-r rule ran the "digit N = north" regex BEFORE the gear context and
  took "heading" / «πορεία» as a north context, so REAL manual sentences were spoken wrong: "Gear pulses F 1900 N 1500
  R 1100" -> "F 1900 north 1500", "Throttle demand 0 N every step" -> "north", «Γκάζι 0, N, πορεία σταθερή.» ->
  «βόρεια», "Set N before the heading turn" -> north, "TELEM2 group N" -> north in both languages. The CONSERVATIVE
  rule now (owner-recommended, W124): a lone N is NEVER «βόρεια» / "north" except (a) a DECIMAL coordinate followed
  by N and then a longitude with E / W or ° (`_N_COORD_RE` / `_N_COORD_DEG_RE`: "37.82 N 23.60 E" - the E / W is
  spoken too, «ανατολικά» / "east"; the ° may already be the spoken unit) and (b) an N right after the word north /
  βόρεια (`_N_AFTER_NORTH_RE`); then sentence by sentence (`_SENT_RE`): "group N" / «ομάδα N» = the letter FIRST
  (`_N_GROUP_RE`), a sentence about the gear (`_N_GEAR_CTX`: σχέση / νεκρά / gear / neutral / a lone CAPITAL F or R /
  γκάζι / throttle / ρελαντί / idle / παλμοί / pulses / shift) = «νεκρά» / "neutral", else the letter («εν» / "N").
  "heading" / «πορεία» / compass / πυξίδα are NO north context any more (`_N_NORTH_CTX` is gone). Measured over
  EVERY pickable sentence of both indexes with a lone N (el 12 / en 14): only the one coordinate sentence (the demo's
  Saronic Gulf) says north; "TELEM2 group N" is the letter in both languages, the gear pulses / REV | N | FWD / «ΠΙΣΩ
  | N | ΜΠΡΟΣΤΑ» / "the gear servo at N" are neutral, "key N" / "part N" / "record N" the letter. Tests: AS26 (every
  reviewer sentence + the two north exceptions + «Πορεία N» = the letter), the new AS32 over the whole index: the
  spoken text of a sentence with a lone N has no more north / βόρεια words than the sentence itself unless
  `speech.has_coordinate` holds, and the named sentences are checked by content.
- LOW-1 (`asstvoice.py`): the TTS_MAX_S rule also when a RESULT is taken - `tick()` drops a finished clip whose
  request is older than TTS_MAX_S (the Tk thread stalled, the TTS thread delivered late): "slow", nothing played,
  the file swept. `make_t0` = when the voice was asked for (speak()), not when the thread began.
- LOW-2: ONE TTS thread at a time (`thread` / `_busy()`): a new question while a TTS run is alive puts its job in
  `queued` (the newest only - an older waiting job is replaced; the running one's result is dropped by the
  generation); `tick()` starts the waiting job once the thread ended (or drops it when the generation moved or its
  8 s passed). The TTS subprocess itself is bounded: `tts.synth(..., timeout=TTS_TIMEOUT_S)` = TTS_MAX_S + 2 s
  (Piper / PowerShell killed by subprocess.run's timeout) - a hung voice program can no longer hold the thread for
  tts.synth's default 300 s. Never a pile-up of TTS processes.
- LOW-3: `making` (the file the TTS thread writes) is cleared ONLY by that thread itself when it is done (under the
  lock) - `stop()` and the slow drop leave it, so `_live()` keeps the sweep off a file still being written (and off
  its `<path>.tmp`, sound.write_wav's half-written name); `close()` and `_clean_stale` remove `*.wav.tmp` too.
- LOW-4: `_lock` - the thread's "is this generation still wanted + hand the result over" and the Tk thread's "take
  the result" / "drop it (stop, slow)" are atomic; `gen` changes only under it.
- LOW-5 (`qlog.py`): the docstring states the limits (no limit on the parts per day; prune trusts the PC clock) and
  `prune` deletes at most MAX_PRUNE_PER_RUN 5 of the oldest files per run (a clock jumped > 31 days ahead loses a
  few files per answer, never the whole log at once - a limit, not a protection). AS31.
- Nit: the Greek voice lines write «ΞΑΝΑ» / «ΟΜΙΛΙΑ» with the guillemets (asst.v_slow and the other asst.v_* lines).
- AS27 asserts that the whole voice cycle imports no `hud` / `gwclient` / `academymenu` (sys.modules before / after:
  the function-level imports of sound.py listed as AS2's limit are really never reached by the voice).
- Owner questions (PROGRESS.md): W124 the conservative N rule (a lone N is the letter unless a coordinate pair /
  after "north"; recommended: confirm); W125 the lang text asst.note / asst.v_alarm now say exactly what the code
  does - the MASTER WARNING lamp («ΓΕΝΙΚΟΣ ΣΥΝΑΓΕΡΜΟΣ») and a sounding / queued callout make the answer text only,
  a MASTER CAUTION alone does not (W119's answer, nothing new decided; the manuals said WARNING already).

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
- W105-W109 (AM4-r): a failed check = can't find said + the passage shown (default yes); the answer mode default
  "extract" (5d; confirm); Krikri / Meltemi in the extract mode only on a >= 8 GB GPU; unpinned model files
  refused unless `assistant_llm_unpinned`; the stricter number / negation rules' false "can't find" accepted.
- W122 (AM4-r4): the "generate" answer mode stays a best-effort, NON-default mode (a CAUTION line on the Tools page
  while chosen; its lexical checks cannot be made airtight) - recommended: keep extract the default, generate off by
  default. W123 (AM4-r4): the held-out question set has been used three times (choice check, floor choice /
  reporting, the AM4-r4 re-measure) = optimistic numbers; the look-ahead-2 limit rule costs ~8 % of the manual picks
  (2178 -> 2014) for the exceptions it brings in; with shorter lists the 3B model more often echoes the prompt's
  example "4, 9" (= can't find) - accept, and judge both (and a prompt example inside the list's length) on the
  FRESH AM6 set (W84 / W89)?
- W86 (AM1-r) a question about a SAFETY ACTION (STANDBY, e-stop, lost link, ...) with no manual hit:
  only "I can't find that in the manual", or also point to where to look? Recommended: yes, the panel
  also shows the chapter list of the manual (text only, never a guessed answer).
