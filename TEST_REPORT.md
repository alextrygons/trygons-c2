# Trygons C2 – acceptance test report

Date: 2026-10-23. Code under test: commit `e385f86` (Parts 0–3), tested with the tools in
`tests/acceptance/` (run them again: `tests\acceptance\RUN_ACCEPTANCE.bat [full]`, MANUAL chapter 19).
Rule for this test: no product code was changed – findings are reported with a reproduction and a
proposed fix. Big outputs (logs, coverage pages, soak data) are in `test_results/` (not in git).

## 00000000000000000. Harbour autonomy - the AURIX side (step HX2, 2026-11-12, FIRMWARE)

app/harbour.c + smallboat.c + link.c (CLAUDE.md Z "Step HX2 AS BUILT", docs/harbour_design.md 2.7). Part of the next flash gate.
- Sim HB1-HB13 (`autopilot_sim hb`, also in every "all"): entry refusals one by one with their why, START + CONFIRM (CREWED /
  UNCREWED) on the checked route leg, the speed cap (route / command / harb_max_kn / 8 kn ceiling / proposal; PARAM ceilings), the
  heading (60 deg, the edge rule, 2 s refused = HOLD), the corridor (back at 2 kn, HOLD beyond +5 m, idle at once, gear N, AUTO kept),
  the 8 deg/s slew, the lease (1000 / 600 / 3000->1000 ms, STOP flag, another kind), GPS / compass, every exit, RESUME on the same
  leg, the lost link (grace 0 = today's kill 2.8 s; grace 60 = kill at 63.0 s; each cut fault at once; link back = latch cleared;
  CREWED never), HARBOUR_STAT 50 frames / 10 s + log event 32, FIRST TEST refused + kind 4 inert. Firmware stage: all 405 / bridge 404 /
  --can 397 / --can-fast 397 passed, 0 failed; TriCore incl. big boat + FIRST TEST; STACK CHECK OK (x3); CPU1 RAM 5.0 %.
- Storm 200 episodes (run_storm.py, 10 workers, 60 simulated h): every invariant PASS, self-test PASS; harbour I85 16 / I86 53 098 /
  I89 44 071 / I91 29 / I92 36 252 checks, 0 fails (30 % harbour episodes from their own hash; a run >= 200 episodes that does not
  reach them FAILS). Mutations (tests/acceptance/storm/mutate_harbour.sh, HARB_MUTATE): 1 cap removed -> I86 (seed 3), 2 corridor
  HOLD removed -> I89 (seed 48), 3 lease HOLD removed -> I92 (seed 7), 4 slew removed -> HB6, 5 grace removed -> I85 (seed 12).
- Findings: (1) storm I86 seed 123: AUTO's rough-water slow-down LIFTED a target below nerv_min_kn (5 kn) to 5 kn - a speed RISE
  above the leg (every AUTO, not only harbour) - fixed (a slow-down never raises). (2) sim GA2 (76 800 baud): HARBOUR_STAT on the
  gateway port made a TELEMETRY gap 260 ms - the frame is now sent only while a harbour route is stored / harbour autonomy is on.
  (3) HB4: at 4 kn the boat overshoots the corridor edge rule (CORRIDOR HOLD is the backstop) - for HX5 / W146. (4) a cap below the
  in-gear idle speed (~1.5 kn) only reaches idle: the planner must STOP (HX5). (5) the drift cut vs a coast-down from 5 kn (W147).
- Station: groundstation/test_harbour_proto.py HP0a-HP0d PASS (mirrors); station stage: test_lang L2 (confirm.12 key, added)
  fixed, SR8 (NVENC screen recording) = the known environmental failure.
- Serial opus review (tools/api_review.py, review_417a0d8.txt): no HIGH; MED-1 (a harbour route could be driven by plain AUTO /
  a DP TRIP without the harbour limits) FIXED - refused (HB13); MED-2 (steering left paused after RESUME) - auto_mode releases it
  every RUN step (HB10 moving), now also explicitly in harb_begin; MED-3 (HOLD needs HELM) = as designed: every AURIX command is
  HELM-only (R7), the gateway forwards a copilot's HOLD on the HELM's behalf without ACCEPT (proto.is_driving_cmd) - note.
  LOWs (grace clamp at use, a cap below the in-gear idle, storm oracle details) = notes (docs/harbour_design.md 2.7).

## 0000000000000000. Talking manual - the prompt default (W129, step AM4-r8, 2026-11-08) + the prefix / Tools-label fix of the one-line places

Station / tools / tests / docs only (docs/assistant_design.md 5k; no app / hal / gateway logic change).

W129 (the owner's YES of 2026-11-08): the extract prompt DEFAULT is "fewshot" now (client.DEFAULT_PROMPT; the per-PC setting
`assistant_prompt` stays as the override). NOT RE-MEASURED: the fresh set below ran ONCE with the OLD default and is not re-run
(the task's rule) - the fewshot figures stay the TUNING-part ones (el 45 % / en 61 % correct, no "no pick" rows, first pick = 4 in
3 %); the new default is measured once on the owner's W84 recordings (or a second fresh set) when they come. The reviewer's L3: the
worked example had no echo rule - answer.FEWSHOT_ECHO: an echo of the worked example ("2" and nothing else, its sentence / question
written back without that text in the offered list, "2" + its figures 30 and 5) = NO pick («Δεν βρίσκω» + the passage, "echoed the
worked example"); "2, 3" / "3" / "2. <the real sentence 2>" / "4, 9" stay picks. Cost on the stored fewshot tuning runs: 0 of 115
outputs were a lone "2" (first pick = 2 in 11 / 83 el + 2 / 32 en, the same share as the default prompt's 7 / 72 + 2 / 32).

The opus review of 37d85dd (ONE safety MED in two parts + MED-2, no HIGH): (1) with a PREFIX on screen 2's message line (the W79
hint «Κεντράρετε το χειριστήριο ...», the loudhailer's «ΜΙΛΑ: ...») draw_bar did not recognise the answer line and fell back to the
OLD end-cut = the limit lost exactly while a hint was on - fixed by show.fit_message (the prefix kept apart in app.msg_prefix and
drawn whole, the answer line whole / the fallback / a plain answer's pixel end-cut in the room left; < 60 px beside the prefix = the
prefix alone); (2) the Tools window's message label was a THIRD one-line place that clipped - app.asst_tools_msg (the label's own
font vs its width, re-fitted on <Configure>, width unknown = the fallback when limited); MED-2: the message was built from the
CHARACTER-cut answer first (a single limiting sentence over 160 characters lost its last clause by characters and a wide bar then
showed the cut line "whole") - a limiting answer now reaches the one-line places UNCUT and the pixel rule decides. LOWs: a can't-find
is not "limited" («Δεν βρίσκω» holds «δεν»), driver.asst_notice_line before the first frame = the fallback when limited, the 124 / 146
refused-statement figures dated (AM4-r5, 2026-11-08). The manuals' claim "a limiting answer is never end-cut" is now TRUE for all
THREE one-line places (ch. 22 / 17 say so). Measured (AS34 AM4-r8 part): 5085 prefix checks (416 joined units + 2 reviewer examples x
W79 / «ΜΙΛΑ» / both x 300 / 450 / 700 / 1300 px + the 2560-px window flex): the limit in the drawn line 389, a fallback 4696, 0
lost; the mutation (the old end-cut of prefix + line) loses the limit 4696 times. MED-2: 239 single limiting sentences > 160
characters x 5 rooms = 1195 checks: whole 74 / fallback 1121 / 0 lost; the mutation (the character-cut line) "fit" a wide bar without
its last clause 243 times. AS28 drives the real station (the bar's drawn text with / without the W79 prefix, the Tools label at 300 /
1300 / 2560 px and with no width, a can't find not limited); AS21 the 4-message fewshot prompt + every echo form + the mutation (under
variant "default" the lone "2" IS a pick). Gate tools/check.sh: everything passed (sim all 386 / bridge 385 / can 378 x 2, TriCore, stack, memory, FAT32 2/2, NV 4/4,
DroneCAN 19/19, charts 6/6, lang 6/6, boats 12/12, screen 2 28/28, tracks 12/12, loudhailer 22 + 8, index current + floor gate 7 /
86 + 5 / 32, assistant 36 passed 0 failed 1 N/A (AS4b, W91) incl. AS4 / AS24 real models with the NEW default prompt, sound 17,
avoidance 15, OS 12, setup 53/53, UAV 42 + 15, gateway 155/155) EXCEPT screen recording SR8 (9/10): the real-ffmpeg h264_nvenc
recording made 1 segment in 12 s again - re-run alone: the same; the SAME environmental failure as in AM6 (screenrec.py /
test_screenrec.py untouched since d96295e; NVENC on this PC today), not this commit. RUN_ACCEPTANCE quick (a local clone of this
commit): 6/7 PASS (ADS excludes + iLLD, storm 200 episodes 0 fails incl. the I59 / I63 key cases, time wrap, day at sea, gateway
storm I80-I84); the fresh build itself: every sub-step PASS (MediaMTX, START_DEMO from the clone, no PC paths) except
groundstation\test_driver.py ONCE 23 of 24 (the acceptance log keeps only the summary line, not the test's name) - re-run alone
in the SAME clone and in the worktree (LOOP RULE 3): 24 / 24 both = a one-off flake of that run (test_driver.py is untouched by
this commit), not this commit.

## 000000000000000. Talking manual - the FINISH (step AM6, 2026-11-08) + the pixel display fix (step AM4-r7) - the FRESH question set measured ONCE

Station / tools / tests / docs only (docs/assistant_design.md 12 + 5j; no app / hal / gateway logic change). The figures of
AM3-AM5 below were all seen while tuning (the held-out set seven times) and are optimistic; THESE are the honest ones.

THE FRESH SET: tests/assistant/fresh_el.json (76 operator questions: 68 in the manual + 8 not) and fresh_en.json (31: 25 + 6),
written on 2026-11-08 after every tuning choice (search factors, synonyms, MIN_COVER, the floor 7.2, the prompt) on topics and
phrasings not in questions_<lang>.json; 0 broken labels at the retrieval check, nothing tuned after it; run ONCE end to end
through the real worker (`assistant_llm_eval.py run --set fresh --answer extract`; llama3.2-3b, the DEFAULT prompt, floor 7.2;
the whole model on the GPU this time: 3651 MB).

| fresh set | SEARCH top-1 | top-3 | search can't find right / false | ANSWER correct | answered | precision | can't find right | false can't find (model's own / no pick / post-check) | first pick = 4 | median ms (p90 / max) |
|---|---|---|---|---|---|---|---|---|---|---|
| el (68 + 8) | 38/68 = 56 % | 55/68 = 81 % | 5/8 / 1 | 28/68 = 41 % | 55/68 = 81 % | 28/55 = 51 % | 7/8 | 13 (4 / 9 / 0) | 7/36 = 19 % | 481 (826 / 3406) |
| en (25 + 6) | 9/25 = 36 % | 16/25 = 64 % | 2/6 / 0 | 13/25 = 52 % | 20/25 = 80 % | 13/20 = 65 % | 6/6 | 5 (2 / 0 / 2) | 1/10 = 10 % | 272 (389 / 421) |

Reading: Greek holds the (optimistic) held-out level (41 % correct, precision 51 % = half of the answers given are a true manual
sentence of the right chapter that does not answer; the "correct" proxy = the picked text holds the question's term, circular,
no human judgement - W99 open), English falls to 52 % on a small set whose search top-1 is only 36 %; 9 of the 13 Greek false
can't-finds are NO PICK rows of the default prompt, 0 by the floor / post-check; latency 0.5 s / 0.3 s median from the typed
question to the answer text (the speech recognition ~1 s and the voice ~1 s come on top: 2-3 s to the first spoken word on a
quiet PC, MANUAL ch. 22). W128 position bias on this set: 19 % of the Greek first picks are the example's 4 (the tuning runs
71-76 %); the top first number here is 1.

THE W128 VARIANTS (TUNING part only, never the fresh set): (a) the prompt "fewshot" (no digit example; one worked example turn
with real manual sentences; setting `assistant_prompt`, default still "default" = W129): llama3.2-3b el 43/96 = 45 % correct /
answered 86 % / precision 52 % / no pick 0 / false can't find 13 / 571 ms / first pick = 4 in 3 % (the default prompt in AM4-r5:
34 % / 75 % / 46 % / 14 of 86 / 24 / 824 ms / 76 %); en 20/33 = 61 % / 97 % / 62 % / 0 / 1 / 319 ms / 0 % (48 % / 97 % / 50 % /
0 / 1 / 451 ms / 59 %) - better on every count, one run each, the 3B churn is +-6 of 96. (b) Krikri-8B extract (21 of 32 layers
on the GPU, 4629 MB, the default prompt): el 49/96 = 51 % / 78 % / 65 % / no pick 2 / false can't find 21 (19 the model's own) /
1048 ms (p90 1569) / first pick 4 in 6 %; en 19/33 = 58 % / 88 % / 66 % / 950 ms - more precise, refuses more, twice the time
and 4.6 GB of the 6 GB card: an option for a >= 8 GB GPU only (W107).

AM4-r7 (the opus review of b1a0138, ONE safety MED, no HIGH): the AM4-r6 limit-keeping cut counted CHARACTERS while both
one-line displays cut by PIXELS (the bar's f.measure to the flex width, the HUD's font shrink to 8 px + the window-edge crop):
«... είναι βοηθήματα. ΔΕΝ είναι για ναυσιπλοΐα.» (111 characters) lost its limit on both screens. Fix: show.fit_line = the whole
line only when the renderer's own pixel measure says it fits, else (a joined unit or any answer with a limiting word) the
fallback line (new short lang key asst.answer_limited_short), never an end-cut; plain answers keep a pixel end-cut
(show.pixel_cut); app.asst_line + draw_bar (screen 2) and driver.asst_notice_line (screen 1, hud.R("notices") at f_norm).
AS34 pixel part: 3780 checks (416 joined units + the 2 review examples; 1366x768 + 1920x1080 x 3 text sizes + 3 bar widths): 0
lose the limit, 20 fit whole, 3760 show a fallback (2930 the short one pixel-cut: the 1366-px notice box is ~278 px); the
mutation (the character cut) overflows the real room 1370 times. The reviewer's LOWs L1 / L2 / L3 / L5 = notes in docs 5j (the cap).

MANUAL GAPS for the owner (docs/assistant_design.md 12; W89 open): HATCH FAULT not explained, a stuck / half-open hatch, fuel
(tank / consumption), man overboard, engine maintenance (oil / service), camera care (salt), live readings (engine temperature /
hours: W82 by design); thin places (a table line or a long paragraph only): what STANDBY does, the reverse power-limit steps,
the hat switch, ΑΒΕΒΑΙΗ ΑΦΙΞΗ, the generator's first step / cycle, the dead-man relay.

Deliverables of AM6: MANUAL ch. 22 / MANUAL_EL ch. 17 "WHAT IT CAN AND CANNOT DO" (can / cannot, the figures above in plain
words, headset + foot pedal, the PTT key, privacy, the limits list), PARTS.md "Shore PC" (headset with a boom microphone
recommended, optional USB foot pedal sending F9), the tour stops s1.btn_ask / s2.assistant / tools.assistant (exist, voice_pending
= W84), `assistant_index.py eval --set fresh`, `assistant_llm_eval.py run --set fresh --prompt fewshot` (+ the first-pick
statistic in every summary). Tests: AS34 (pixel part + fit_line + the station wiring), test_lang 6/6 (the new lang key); the
index rebuilt + committed (the manuals changed; the floor gate unchanged 7 / 86 + 5 / 32; the main list's eval unchanged el 60 /
78 %, en 50 / 76 %). Gate tools/check.sh: all suites passed except SR8 (screen recording, 9/10: 1 NVENC segment in 12 s; the same on the pristine HEAD b1a0138 in a temporary worktree = environmental); RUN_ACCEPTANCE quick: 7/7 PASS (ADS excludes + iLLD, fresh build from the clone incl. START_DEMO and MediaMTX this time, storm 200 episodes 0 fails incl. the I59 / I63 key cases, time wrap, day at sea, gateway storm I80-I84). OPEN for the owner: W84, W89, W97, W99 (his four), W129 (the prompt
default).

## 00000000000000. Talking manual - review fixes of AM4-r4 (step AM4-r5, 2026-11-08) - the LAST fix round of the limit / pick logic

Station / tools / tests / docs only (docs/assistant_design.md 5h). The AM4-r4 chain of limiting sentences ran on: every
next limit was judged against the GROWING unit and the chain never stopped at a heading, so sections were glued (en:277
HELM / role swap + the generator = 211 words) and 216 en / 190 el units (this step re-measured 212 / 192) fell over the
60-word cap and were REFUSED - key safety statements among them (en:220 "CONFIRM only answers a request from the PC",
en:185 "Only STANDBY, TRAINING and HEADING LOCK", en:299 "Only the HELM station's relay commands", en:235 the gear
through NEUTRAL, el:122 «Ξεκινά ΜΟΝΟ με ΕΝΑΡΞΗ AUTO + CONFIRM»). Now every candidate limit is judged against the PICK +
the ONE sentence before it, the chain stops at a heading (index.following: same chapter + heading + term; W126 = NO
across a heading), a definition split over passages of the same term IS chained (the UNCREWED HDG LOCK block, SCR), and
a chain over 60 words falls back to the statement + its FIRST limit marked "more" = «Υπάρχουν και άλλοι περιορισμοί –
δείτε το εγχειρίδιο» spoken at the end + a line on the panel / Tools page (W126 = YES), else nothing (never a cut). A
sentence that OPENS with a limiting word counts at ANY length, a pronoun limit counts two ahead too, a two-ahead limit
must RESTRICT a shared word, a pronoun-opening sentence is never offered alone (LOW-2; 3 el / 65 en such picks gone).
Measured over both indexes: offered picks 2014 -> 2070, joined units 76 / 127 -> 137 / 193, "more" units 40 el / 45 en,
statements still refused (the statement + its first limit alone over 60 words) 192 / 212 -> 124 / 146; the curated list
tests/assistant/limits_must.json (the reviewer's 23 sentences) 0 misses + 2 documented limits (en:189 across a heading,
en:87 a pronoun with a pronoun antecedent). AS25 is a TRULY separate implementation now (own splitter / fold / lists /
stemming / following rule, the following rule checked equal to index.following): 0 misses over both indexes; the AM4-r2
list misses > 200, the AM4-r3 / AM4-r4 limit shape misses >= 8 of the curated list, the fallback off >= 3. The prompt
example: letters "N or N, M" (the review's proposal) and digits the list cannot reach "40 or 40, 41" were MEASURED and
are worse with the 3B model (Greek 0 % / 14 % correct: it copies them), no example 22 % (it writes text); the example
"4 or 4, 9" stays and an EXACT echo of it is NO pick (the 5g "post-check rejected" of those rows = NO PICK, counted
apart now; in AM4-r4 6 of 9 Greek echoes were ACCEPTED as confident picks, and the AM4-r4 English held-out "correct"
en009 WAS such an echo). answer.sentences: an abbreviation only as a whole token ("much." is no "ch."), "e.g. 3840"
and "1. black box" joined.

RE-MEASURED (llama3.2-3b extract, 17 of 28 layers on the GPU, the floor 7.2; the tuning part FOUR times in this step
for the prompt choice, the held-out part with the kept prompt once - but the two superseded prompt variants ran it too
= the held-out set is now used SEVEN times in all, its numbers optimistic; the fresh AM6 set is the real measurement):

| llama3.2-3b extract | part | correct | answered | precision | false can't find (floor / no pick / post-check) | post-check rejected | no pick (exact "4, 9" echoes) | can't find right | median ms |
|---|---|---|---|---|---|---|---|---|---|
| AM4-r5 (AM4-r4: 34 % / 83 % / 41 % / 16) | el tuning | 34 % (33/96) | 75 % | 46 % | 24 (10 / 14 / 0) | 0/86 | 14/86 (11) | 7/7 | 824 |
| AM4-r5 (AM4-r4: 45 % / 94 % / 48 % / 2) | en tuning | 48 % (16/33) | 97 % | 50 % | 1 (1 / 0 / 0) | 0/32 | 0/32 | 5/5 | 451 |
| AM4-r5 HELD-OUT (AM4-r4: 42 % / 84 % / 50 % / 6) | el | 42 % (16/38) | 87 % | 48 % | 5 (3 / 1 / 1) | 1/35 | 1/35 (0) | 4/4 | 821 |
| AM4-r5 HELD-OUT (AM4-r4: 78 % / 89 % / 88 % / 1) | en | 67 % (6/9) | 78 % | 86 % | 2 (1 / 1 / 0) | 0/8 | 1/8 (1) | 1/1 | 487 |

Per question vs the stored AM4-r4 rows: el tuning 6 lost / 6 gained (every one the model picking another sentence
from a renumbered list), the 11 Greek echoes now say can't find (answered 83 -> 75 %, precision 41 -> 46 %); en tuning
+1; el held-out 1 / 1; en held-out -1 = the echo en009. The AM4-r3 -> AM4-r4 Greek loss (40 -> 34 %) is NOT explained by
the echoes (<= 1 point): it is the run-to-run churn of a 3B model on renumbered lists (+-6 of 96 between runs of the
same code). NEW FINDING (W128, for AM6): the example BIASES the pick - in every stored extract run the model's FIRST
picked number is the example's "4" (AM4-r5 el tuning 66 of 86 rows = 76 %, 22 of the 33 "correct" answers among them;
el held-out 71 %; en tuning 59 %; AM4-r3 / AM4-r4 the same 75-76 % in Greek): the circular proxy counts a positional
pick "correct" when sentence 4 of the best passage happens to hold the must term, so it says little about the Greek
picking quality of the 3B model; every alternative prompt measured here was worse (letters 0 %, 40/41 14 %, none 22 %)
- AM6 decides on the fresh set (another wording / few-shot, a bigger Greek model, a non-positional judgement). Tests
AS21 (89), AS25 (17, own rule + curated list), AS26 (+ the marker), AS32 (the coordinate rule over every manual
sentence), AS24 (the Greek hatch question now echoes "4, 9" = an honest no pick accepted for it; a second Greek
question proves the extract answer); test_lang 6/6; the index rebuilt + committed (index.py + the manuals changed; the
floor gate unchanged 7 / 86 + 5 / 32). W126-W128.

## 0000000000000. Talking manual - review fixes of AM4-r3 (step AM4-r4, 2026-11-08) - CORRECTS the AM4-r3 reading of the held-out drop below

Station / tools / tests / docs only (docs/assistant_design.md 5g). The "no pick leaves its limiting sentence out" rule
was circular (AS25 used the same answer.limits() that builds the picks) and incomplete (one passage only): the
reviewer counted picks whose limit opens the NEXT passage of the chapter (19 en / 10 el), limits that open with a
pronoun ("It counts only when ...": 42 en) and limits two sentences later (19 en / 23 el). Now the pick units are built
over the CHAPTER text (the passage + its following passages = one text: a sentence the index builder cut at the
boundary is whole again, the chain crosses the boundary and the following passage is shown as passage2), the limit is
looked for up to TWO sentences ahead, a pronoun-opening limit counts whatever the shared words, an "Except ..." before
the pick begins the unit; the cap refuses the whole unit. AS25 checks it with its OWN independent wider rule: 0
remaining misses over both indexes (the AM4-r2 / AM4-r3 lists miss > 200 / > 40). Cost: 2178 -> 2014 manual picks
offered (the look-ahead 2 = 155 of them), 84 units reach into the following passage; a TERM passage (a definition
line) is never chained to. Generate: a 2-word clause fully shared
with a negating sentence is judged ("... and the PC overrides it." refused), '?' clauses are no claim; only / unless /
except added where cheap; generate is DOCUMENTED as a best-effort NON-default mode (a CAUTION line on the Tools page;
W122). floor --write refused while an off-manual tuning question reaches the floor; one floor for both languages; a
bool / negative floor setting = the default; a pick inside another chosen pick is dropped.

THE HELD-OUT DROP OF AM4-r3, HONESTLY (el held-out, 38 in-manual, AM4-r2 18 correct -> AM4-r3 14): 7 lost, 3 gained.
Lost by the FLOOR: 0 (the 3 held-out questions under 7.2 were wrong already); by the joined picks / length cap: 1;
by the post-check: 2; by the MODEL RUN (another sentence picked from a renumbered list): 4 (the 3 gained the same
way). The false can't-finds 4 -> 8: +2 the floor, +2 the post-check. The AM4-r3 text below ("the floor's cost") is
corrected by this. THE HELD-OUT SET HAS NOW BEEN USED THREE TIMES (AM3-r check, AM4-r2 floor choice / AM4-r3
reporting, this re-measure): its numbers are optimistic; the fresh AM6 set (W84 / W89) is the real measurement (W123).

RE-MEASURED (llama3.2-3b extract, 17 of 28 layers on the GPU, the floor 7.2 in force; the held-out part was run TWICE
in this step - the first run found the term-passage gluing bug and was superseded - so the set is now used FOUR times):

| llama3.2-3b extract | part | correct | answered | precision | false can't find (floor / post-check) | rejected | can't find right | median ms |
|---|---|---|---|---|---|---|---|---|
| AM4-r4 (AM4-r3: 40 % / 83 % / 48 % / 16 / 7 %) | el tuning | 34 % (33/96) | 83 % | 41 % | 16 (10 / 6) | 6/86 = 7 % | 7/7 | 785 |
| AM4-r4 (AM4-r3: 48 % / 97 % / 50 % / 1 / 0) | en tuning | 45 % (15/33) | 94 % | 48 % | 2 (1 / 1) | 1/32 = 3 % | 5/5 | 447 |
| AM4-r4 HELD-OUT (AM4-r3: 37 % / 79 % / 47 % / 8 / 14 %) | el | 42 % (16/38) | 84 % | 50 % | 6 (3 / 3) | 3/35 = 9 % | 4/4 | 806 |
| AM4-r4 HELD-OUT (AM4-r3: 78 % / 89 % / 88 % / 1 / 0) | en | 78 % (7/9) | 89 % | 88 % | 1 (1 / 0) | 0/8 | 1/1 | 476 |

Per question (vs the stored AM4-r3 rows): el tuning 38 -> 33 = 9 lost (7 the model picking another sentence from a
shorter renumbered list, 2 the joined unit / length cap, 0 the floor, 0 the post-check) / 4 gained; el held-out 14 -> 16
(2 post-check rejections gone, 1 other pick; 1 lost). The rejections are now the MODEL's: 5 of 9 Greek ones are the
prompt's example "4, 9" echoed back with a list shorter than 4 (W123 c), the rest an unmatched "1. <text>" echo; ONE
English verbatim pick (en030) was refused by the lexical number rule (the manual's list number "1." glued to the end of
the sentence). The wider limit rule costs 7.5 % of the picks and ~5 points of the circular Greek proxy on the tuning
part, gains 5 on the over-used held-out part, keeps precision and brings every following exception into the spoken
text. Generate (stored runs re-scored with the new clause / only rules): el 18 %, rejected 39 %; en 33 %, rejected
46 % (one more true answer refused). Tests AS19 (60), AS21 (69), AS22 (+ after), AS25 (14, its own rule), AS29 (12).

## 000000000000. Talking manual - review fixes of AM4-r2 (step AM4-r3, 2026-11-08) - CORRECTS the AM4-r2 extract figures below

Station / tools / tests / docs only (docs/assistant_design.md 5f). A verbatim pick could leave the NEXT manual sentence
out - the one that limits it («... είναι βοηθήματα.» -> «ΔΕΝ είναι για ναυσιπλοΐα.»; 186 el / 189 en pickable
statements are followed by such a sentence): now it is offered only TOGETHER with it (one unit of consecutive manual
text, chained; over 60 words = not offered; test AS25 over the WHOLE index: 0 of 2987 picks leave one out, the AM4-r2
list leaves 290 out = FAIL). The generate post-check's negation rule works PER CLAUSE and skips a comma insertion
(the reviewer's "In AUTO ... and the PC can override the STANDBY button" / "never allowed, in any mode or situation,
to override it" are AS19 tests). A glued "* bullet" list over 50 words is offered item by item (single picks over 50
words 55 el / 83 en -> 35 / 61). A null `assistant_extract_floor` = the default 7.2 (only 0 = off) and a GATE
(`tools/assistant_index.py floor`, check.sh, AS29) fails when the BM25 scores move away from the separation the floor
was tuned on (tests/assistant/floor.json). "1.500 m" is spoken as 1500. AS24 asserts its questions score >= floor + 2.
test_driver.py / fresh_build.py fail early without Pillow. Re-measured (llama3.2-3b, the floor 7.2 IN FORCE - the
AM4-r2 row below was measured without it; "rejected" = of every row the model generated for, off-manual included):

| llama3.2-3b | part | correct | answered | precision | false can't find | rejected | can't find right | median ms |
|---|---|---|---|---|---|---|---|---|
| extract (AM4-r2: 43 % / 94 % / 46 % / 6 / 5/7) | el tuning | 40 % (38/96) | 83 % | 48 % | 16 | 6/86 = 7 % | 7/7 | 820 |
| extract (AM4-r2: 45 % / 100 % / 45 % / 0 / 4/5) | en tuning | 48 % (16/33) | 97 % | 50 % | 1 | 0 | 5/5 | 446 |
| generate | el tuning | 17 % | 38 % | 44 % | 60 | 21/56 = 38 % | 7/7 | 1313 |
| generate | en tuning | 36 % | 58 % | 63 % | 14 | 15/35 = 43 % | 4/5 | 1200 |
| extract HELD-OUT (once; AM4-r2 47 % / 89 % / 53 % / 4) | el | 37 % (14/38) | 79 % | 47 % | 8 | 5/35 = 14 % | 4/4 | 3089 (busy PC) |
| extract HELD-OUT (once) | en | 78 % (7/9) | 89 % | 88 % | 1 | 0 | 1/1 | 487 |
| generate HELD-OUT (once) | el | 5 % (2/38) | 29 % | 18 % | 27 | 14/25 = 56 % | 4/4 | 1347 |
| generate HELD-OUT (once) | en | 56 % (5/9) | 89 % | 62 % | 1 | 1/9 | 1/1 | 3425 |

Honestly: the floor now refuses EVERY off-manual tuning question (7/7, 5/5) and takes true Greek answers with it
(tuning 43 -> 40 %, held-out 47 -> 37 %, false can't find 6 -> 16 / 4 -> 8) - set on 3 cases (W112: re-tune on the
real questions; the new gate keeps the tuned separation). The joined picks kept precision (48 / 50 %) but are longer
(English <= 40-word proxy 71 -> 53 %; the voice says the chapter intro above 50 words, W113 / W118). Generate stays
far behind in Greek (17 % / held-out 5 %: the lexical checks refuse most of a 3B model's Greek). The AM4-r2 divisors
97 / 37 vs 96 / 38 were the off-manual rows the model answered (explained, not an error). Tests AS19 (54), AS21 (54),
AS25 (11), AS26, AS29.

## 00000000000. Talking manual - review fixes of AM4-r (step AM4-r2, 2026-11-07) - CORRECTS the AM4-r extract figures below

Station / tools / tests / docs only (docs/assistant_design.md 5e). The AM4-r "extract" picks were NOT always whole
sentences: the splitter cut at ';' and '·' too, so a pick could be the first half of a manual sentence whose "never /
only / nothing but" clause followed (452 such picks over both indexes, 145 with a negation / only clause after them).
Fixed: a sentence ends only at . ! ? or the Greek question mark; test AS25 proves it over the WHOLE index (3119 picks,
0 clause fragments; the old splitter fails the test). The negation check now skips stop words («δεν μπορεί ποτέ να
το παρακάμψει») and looks at every passage sentence about the same thing (the reviewer's two probes are tests);
thousands separators ("1,500 m" = 1500 m); two picks from two passages cite both chapters; a cut "1" is the pick;
the model file's sha256 is cached per process (a guard reload hashes nothing again); AS24 requires the model's own
answer. Re-measured (llama3.2-3b, extract, the same stricter check):

| extract | part | correct | answered | precision | false can't find | can't find right | median ms |
|---|---|---|---|---|---|---|---|
| AM4-r2 | el tuning | 43 % (AM4-r 36 %) | 94 % | 46 % | 6 | 5/7 | 828 |
| AM4-r2 | en tuning | 45 % (48 %) | 100 % | 45 % | 0 | 4/5 | 454 |
| AM4-r2 HELD-OUT (once) | el | 47 % | 89 % | 53 % | 4 | 4/4 | 891 |
| AM4-r2 HELD-OUT (once) | en | 78 % (7/9) | 100 % | 78 % | 0 | 1/1 | 505 |

The proxy stays circular (a verbatim sentence holding the question's words is "correct"). Extract still answered 3 of 12
off-manual tuning questions with a true but off-target sentence (best scores 6.07-7.10, inside the in-manual range):
a SCORE FLOOR 7.2 (setting assistant_extract_floor; the passage is still shown below it) refuses all 3 at the cost of
3 of 41 correct Greek tuning answers and 1 of 7 correct English held-out answers - W112 asks the owner (default ON).
Generate mode re-scored with the stricter negation rule: el 17 % correct / 41 % rejected, en 39 % / 40 % (was 48 % /
26 %). test_driver's "HUD not rendered" failures of the AM4-r gate: a Python without Pillow (the Windows Store
python); with the gate's msys Python 24/24. Tests AS19 (48 checks), AS21 (45), AS23, AS24, AS25.

## 0000000000. Talking manual - ANSWER, review fixes (step AM4-r, 2026-11-07) - CORRECTS the AM4 figures below

Station / tools / tests / docs only. The post-check of 3e1b1d3 could be bypassed (an unknown unit "50 feet", the sign
"-3 s", number words "εβδομήντα δευτερόλεπτα", a number re-attached to another noun, a negation added / dropped, a
Capitalised "Reset button", an answer in the other language): fixed (docs/assistant_design.md 5c; test AS19 = 42
checks incl. every reviewer probe). A FAILED check now SAYS "can't find" and SHOWS the best passage with its chapter
(W105). The stored AM4 tuning runs re-scored with the stricter check (llama3.2-3b, "generate"): rejected 37 % of the
Greek / 26 % of the English generated answers (was 8 % / 6 %); correct Greek 17 % (was 22 %), English 48 % (was 67 %)
- a rejected answer is now "can't find", and among the new rejections are real errors the old check let through.

The review's extractive design, measured (5d; the same stricter post-check; the must-mention proxy is CIRCULAR -
a verbatim sentence holding the question's words counts as correct even when it does not answer):

| llama3.2-3b, tuning | lang | correct | answered | precision | false can't find | post-check rejected | median ms |
|---|---|---|---|---|---|---|---|
| generate | el | 17 % | 40 % | 42 % | 58 | 37 % | 5187 (busy PC) |
| **extract** (DEFAULT now, W106) | el | 36 % | 91 % | 40 % | 9 | 8 % (text instead of a number) | 803 |
| generate | en | 48 % | 76 % | 64 % | 8 | 26 % | 3223 |
| **extract** | en | 48 % | 100 % | 48 % | 0 | 0 % | 427 |

Latency, honestly (MED-4): AM4's "1.9 s median" was over 10 spoken questions of which 4 were "can't find"; over the 6
answers the model ACTUALLY generated: 2.3 s median / 2.6 s max (quiet PC), 9.8 s / 13.8 s in the run right after the
heavy comparison. Re-measured today (10 spoken Greek questions, real Whisper, another program holding 2.1 GB of the
GPU = 17 of 28 layers on it): generate 3.04 s median / 3.48 s max over its 6 generated (2.34 s over all 10); extract
1.86 s / 2.74 s over all 10 (every one generated). The model file's sha256 is verified before every load (1.8 s).
Tests: test_assistant AS19-AS24 (AS24 = the real model, network blocked). Owner items W105-W109.

## 000000000. Talking manual - ANSWER (step AM4, 2026-11-07)

Station / tools / tests / docs only (no app / hal / gateway change). llama.cpp in the assistant worker through its
Python bindings (no exe, no server, no port); a grounded prompt; a post-check of every name and every number + unit
(else the manual's first sentence); the model's "can't find"; a GPU memory guard (GPU / GPU+CPU / CPU) and a
retrieval-only mode. Model comparison with `tools/assistant_llm_eval.py` through the real worker (Whisper loaded):
the TUNING part for the choice, the HELD-OUT part measured once. Full tables + conditions: docs/assistant_design.md 5b.

| tuning part | Greek correct (96) | Greek false "can't find" | English correct (33) | median ms (search + model) | runs on (6 GB GPU, Whisper loaded) |
|---|---|---|---|---|---|
| none (the search's sentence) | 22 % | 1 | 12 % | 0 | - |
| qwen2.5-3b | 12 % | 37 | 45 % | 2371 / 3352 | GPU |
| **llama3.2-3b (chosen)** | 22 % | 40 | 67 % | 5187 / 3223 (busy PC) | GPU+CPU 23/28 in that run (another program held GPU memory); all 28 with >= 3.5 GB free |
| meltemi-7b | 23 % | 27 | 39 % | 6929 / 9499 | GPU+CPU 24/32 |
| krikri-8b | 24 % | 44 | 45 % | 7665 / 7051 | GPU+CPU 21/32 |
| qwen2.5-7b (first 40 el) | 31 % of 39 | 8 | 58 % | 4687 / 5468 | GPU+CPU 12/28 |
| llama3.1-8b (first 40 el) | 21 % of 39 | 5 | 64 % | 10079 / 6371 | GPU+CPU 13/32 |

HELD-OUT (once): llama3.2-3b Greek 8/38 = 21 % (the search's sentence 11/38 = 29 %), English 6/9 = 67 % (2/9);
Krikri Greek 10/38, English 4/9. Not-in-the-manual questions: "can't find" 4/4 el, 1/1 en for the models.
Post-check rejections: 0-15 % of the generated answers. End to end (release -> answer TEXT, real Whisper + llama3.2-3b,
10 spoken Greek questions): median 1.9 s, max 2.6 s on a quiet PC (a first run right after the heavy runs: median
8.8 s, max 13.8 s); worker GPU memory 3.65 GB. Tests: test_assistant AS19-AS24 (AS24 = the real model with the
network blocked: PASS on this PC). Owner questions W98-W102 (retrieval sentence, blind Greek review
docs/assistant_llm_review.md, the GPU margin, the Greek "can't find" rate, Greek with or without the model).

## 00000000. Talking manual - FIND, second review fixes (step AM3-r2, 2026-11-07) - REPLACES the AM3-r figures

Tools / tests / docs only. A tour passage is now ALWAYS a "tour" hit (phrases in tour texts were counted as manual
hits), phrases are counted on manual passages only (el135 = tour-only), MIN_COVER per language (el 0.30, en 0.33 -
tuned on the English TUNING part only, so English out-of-manual questions are refused as often as at 0.35).

| `tools/assistant_index.py eval` | in the manual | top-1 (manual + tour) | top-3 | false "can't find" | not in the manual -> "can't find" |
|---|---|---|---|---|---|
| Greek in-sample (tuning) | 96 | 56 = 58 % (45 + 11) | 75 = 78 % | 1 | 5 / 7 |
| Greek HELD-OUT | 38 | 24 = 63 % (19 + 5) | 29 = 76 % | 1 | 4 / 4 |
| Greek all | 134 | 80 = 60 % (64 + 16) | 104 = 78 % | 2 | 9 / 11 |
| English in-sample (tuning) | 33 | 15 = 45 % (13 + 2) | 24 = 73 % | 0 | 3 / 5 |
| English HELD-OUT | 9 | 6 = 67 % (6 + 0) | 8 = 89 % | 0 | 0 / 1 |
| English all | 42 | 21 = 50 % (19 + 2) | 32 = 76 % | 0 | 3 / 6 |

Manual chapters only: Greek top-1 64 / 134 = 48 %, English 19 / 42 = 45 %. Now refused in English: "what do I do if
someone falls overboard". Still not refused by BM25 (AM4's post-check is the backstop): "η καταπακτή κόλλησε στη
μέση", "τι ώρα δύει ο ήλιος", "what does HATCH FAULT mean?", "how much does the boat cost", "what mode am I in right
now". AS17: can't find right el >= 8, en >= 3. A FRESH question set (never seen while tuning) is needed in AM6.

## 0000000. Talking manual - FIND, review fixes (step AM3-r, 2026-11-07) - REPLACES the AM3 figures below

Station only (no app / hal / gateway change). The AM3 figures below were IN-SAMPLE (thresholds chosen on the same
list) and used LOOSE labels (a phrase in many passages or a heading-only match counted as a hit). Now:
- strict hit = a passage whose TEXT holds a SPECIFIC expected phrase (found in 1..5 passages; >= 6 = loose, a
  heading / term match alone = no hit) or one of the question's accepted tour stops (counted apart). 16 labels
  re-written from the manual text (el043 el060 el066 el070 el123 el134 el135, en001 en002 en009 en019 en023 en027
  en028 en032 en043; el134 / el135 / en043 = tour stops only, because the assistant chapters are no longer indexed).
- held-out set = tests/assistant/heldout.json (every 3rd question of each group, frozen before tuning). Re-tuned on
  the TUNING part only (MIN_COVER 0.35 -> 0.30; MIN_SCORE 3.0, TOUR_FACTOR 0.9 kept), then held-out measured once.
  Caveat: the held-out questions were in the AM3 list, so the spoken-word table SAID was written with them in view
  = an upper bound until a fresh set (AM6). English held-out = 9 questions: rough.

| `tools/assistant_index.py eval` | in the manual | top-1 (via a tour stop) | top-3 | false "can't find" | not in the manual -> "can't find" |
|---|---|---|---|---|---|
| Greek in-sample (tuning) | 96 | 56 = 58 % (10) | 75 = 78 % | 1 | 5 / 7 |
| Greek HELD-OUT | 38 | 24 = 63 % (4) | 29 = 76 % | 1 | 4 / 4 |
| Greek all | 134 | 80 = 60 % (14) | 104 = 78 % | 2 | 9 / 11 |
| English in-sample (tuning) | 33 | 15 = 45 % (1) | 24 = 73 % | 0 | 2 / 5 |
| English HELD-OUT | 9 | 6 = 67 % (0) | 8 = 89 % | 0 | 0 / 1 |
| English all | 42 | 21 = 50 % (1) | 32 = 76 % | 0 | 2 / 6 |

Manual chapters only (tour-stop hits not counted): Greek top-1 66 / 134 = 49 %, English 20 / 42 = 48 %.
Greek per group (top-1 / top-3 of n): alarms 17 / 20 of 22, modes 11 / 15 of 20, UAV 9 / 13 of 17, launch + hatch
9 / 13 of 14, joystick 7 / 8 of 10, screens 3 / 5 of 9, recording 4 / 5 of 7, loudhailer 5 / 6 of 7, lost link 4 / 4
of 6, power limit 4 / 5 of 5, roles 3 / 3 of 5, DP 2 / 3 of 5, engine 1 / 2 of 5, assistant 1 / 2 of 2.
Not refused by BM25 (AM4 must refuse them): "η καταπακτή κόλλησε στη μέση", "τι ώρα δύει ο ήλιος", "what does HATCH
FAULT mean?", "how much does the boat cost", "what mode am I in right now", "what do I do if someone falls
overboard". False can't find: "μπορεί το drone να ρίξει κάτι;", "ποιος μπορεί να κάνει μανούβρα".
AS17 floors (just under the honest numbers): tuning el 55 / 75 %, en 42 / 70 %; held-out el 60 / 73 %, en 55 / 75 %;
false can't find el <= 3, en <= 1; can't find right el >= 8, en >= 2.
Other fixes: a stale committed index FAILS tools/check.sh (`assistant_index.py check`) and is never used by the
worker (can't find + the notice asst.n_index; AS14 mutation: a manual changed, no rebuild -> exit 1); the question
language's index only (LOW-2); the lang files no index source (LOW-1); the assistant chapters MANUAL_EL 17 / MANUAL
22 not indexed (LOW-3); a line range per split part (LOW-4; 0 wrong ranges over 1048 manual passages).

## 000000. Talking manual - FIND (step AM3, 2026-11-07)

Station only (no app / hal / gateway change). `python tools/assistant_index.py eval` (deterministic; also in
tools/check.sh) on the question list v1 `tests/assistant/questions_el.json` + `questions_en.json` (written from the
manuals + the tour; the owner reviews it = W89). A hit = a retrieved passage holding one of the question's expected
manual phrases (or one of its accepted tour stops). BM25 alone, no language model, no embedding model.

| | questions | in the manual | top-1 | top-3 | false "can't find" | not in the manual -> "can't find" |
|---|---|---|---|---|---|---|
| Greek | 145 | 134 | 82 (61 %) | 103 (77 %) | 4 | 9 / 11 |
| English | 48 | 42 | 23 (55 %) | 34 (81 %) | 0 | 3 / 6 |

Greek per group (in the manual: top-1 / top-3 of n): alarms 19 / 20 of 22, power limit 4 / 5 of 5, loudhailer 5 / 6
of 7, joystick 7 / 8 of 10, launch + hatch 9 / 13 of 14, UAV 9 / 13 of 17, modes 12 / 16 of 20, recording 4 / 5 of 7,
roles 3 / 3 of 5, lost link 3 / 3 of 6, DP 2 / 3 of 5, screens 3 / 5 of 9, engine 1 / 1 of 5, assistant 1 / 2 of 2.
Search < 1 ms per question, index load 73 ms (554 Greek / 823 English passages incl. 151 tour stops).
Not refused by BM25 (AM4's grounded answer + post-check must refuse them): "η καταπακτή κόλλησε στη μέση",
"τι ώρα δύει ο ήλιος", "what does HATCH FAULT mean?", "how much does the boat cost", "what mode am I in right now".

Manual gaps (the manual itself is silent - for the owner, W89): 6 entries, 4 topics
- el015 / en005 "τι σημαίνει HATCH FAULT;" / "what does HATCH FAULT mean?": the HATCH FAULT notice (cause, what to
  do) is not explained in MANUAL_EL / MANUAL (only "την καταπακτή υγιή" / "the hatch healthy").
- el016 "η καταπακτή κόλλησε στη μέση": a stuck / half-open hatch (switch timeout) is not described for the operator.
- el141 "πόσα λίτρα βενζίνη έχει το ρεζερβουάρ": fuel tank / range - nothing in the manual.
- el144 / en048 "τι κάνω αν πέσει άνθρωπος στη θάλασσα" / "what do I do if someone falls overboard": no operator
  procedure (a safety action - W86).

## 00000. Step 13b - review fixes for 11g + step 13 (2026-10-29)

Two independent safety reviews each of commits 3ad1e95 (step 11g) and 1c0ddc6 (step 13); every finding verified in the
code, the real defects fixed with a sim test each (mutation-checked: each test fails with its fix reverted):
DP17 (DP TRIP after an AUTO RETURN = the stored program, own copy; AUTO START CONFIRM in DP changes nothing), AR6
(rough-water memory kept on an AUTO RETURN in AUTO), PW5 / PW6 (dead paddlewheel), CN28 (CPU1 neutral after the
throttle node's return), RB22 (DroneCAN relay ON burst cut -> OFF, SHUTOFF ON after a warm reset, reboot around a warm
reset = unknown), CN26 (full_us). Storm: new I59 (a DP TRIP drives its own copy of the stored program, also with an
AUTO RETURN latched - the generator now puts an AUTO RETURN + STANDBY before some TRIP starts), I58 stronger (the
CONFIRM must answer CONF_AUTO_RETURN; speed / radius / action / final STOP checked; E_PC_RETURN has its own draw), I47
(monitor trip), I51 (every changed gear-node command), I52 (baseline from just before the tripping monitor step).
**I52 note:** the cross-core race (core 0's TX flush running while CPU1 trips) can't happen in the single-threaded
storm - it is covered by review only; on the board TXBCR can't retract a frame already in transmission, so 1 stale
ArrayCommand is the hardware allowance (overwritten by CPU1's safe frame within 20 ms).

## 0000. Step 14 - final check: full acceptance + 24 h soak (2026-10-29)

Code under test: commit `77f00bb` (every step DONE; fresh clone from GitHub). Run:
`python tests/acceptance/run_acceptance.py full` (NOT shortened), 145 min, summary kept in
`test_results/acceptance_summary_step14_full.txt`. No app/, hal/, gateway or groundstation logic was changed.

**What "24 h soak" means here (said exactly):** the AURIX soak is 24 SIMULATED hours (`storm.exe --soak 24 24 --log`,
1 726 102 control steps, 173 boots, with the core-0 log producer + core-2 logger) - it runs in 0.3 min of PC time;
in addition 10 more 24 h simulated soaks were run with seeds 1-10 (240 h more, every one "all invariants held",
0 overruns, 0 log drops). The REAL-TIME part is the 60 min system soak (simulator + gateway + 4 test cameras +
vision + stations). A 24 h REAL-TIME system soak was NOT run (not feasible in this session); it stays an open item
for the gateway PC before the first long uncrewed trip.

| Part | Result |
|---|---|
| ADS excludes / iLLD sources | PASS (4 configurations) |
| Fresh clone from GitHub + every suite (tools/check.sh) | FAIL on the first run, see F14-1..F14-3. Inside the gate: sim 300/300, bridge 299/299 (1 N/A), DroneCAN servos `all --can` 292/292 and `--can-fast` 292/292 (8 N/A each), TriCore + iLLD + writer checks, FAT32 2/2, NV 4/4, DroneCAN 19/19, charts 5/5, lang 5/5, boats 10/10, screen 2 15/15, sound 10/10, avoidance 14/14, **academy 50/50, practical 8/8**, OS 11/12 (F14-1), gateway 111/114 (E1-E3, PC load), e2e 10/10, logs 3/3, test_driver 18/21 (I4 / I12 / I19, W10), START_DEMO from the clone PASS |
| Fault storm 2000 episodes (588 simulated h, ~35 % DroneCAN episodes) | FAIL on the first run: 14 seeds on I29 / I32 / I36 / I50 (F14-2). All 14 reproduce with `--replay SEED`; analysed: storm oracle gaps, no product bug. After the oracle fix: **2000 episodes, 52 invariants (I1-I58 as built) 0 fails**; self-test still catches its planted faults |
| 32-bit time wraparound | PASS |
| 24 h AURIX soak with logging (simulated, seed 24) | PASS - all invariants, 0 overruns, 0 log drops, longest step 15 ms PC time (Windows scheduling; 99.9 % < 100 us) |
| System soak 60 min (real time) | PASS - AURIX step max 490 us (PC), 0 overruns, 0 log drops; every memory / handle trend "steady" |
| Day at sea, AURIX part | PASS 15 / 15 |
| Day at sea, whole system | PASS 12 / 12 (29.5 min; 244 / 245 files synced equal, 1 re-queued after a change) |
| Gateway storm I14, I15, I17, I20, I21, I26 (120 episodes) | PASS |
| Station storm I16 (300 s) | PASS, self-test caught the planted frame |
| Coverage | PASS (smallboat.c 98 % lines / 87 % branches) |

Re-runs alone (PC load): gateway section E 6/6 PASS; test_driver 18/21 again (I4 / I12 / I19 = the live-HUD tests,
the known W10 / W11 finding - two pythonw processes of the owner on this PC, not touched); test_osdep 12/12 in this tree.

### Findings of step 14

- **F14-1 (packaging, real, fixed in this commit - not product logic).** `.gitattributes` has `gateway/** text eol=lf`,
  so a FRESH CLONE gets `gateway/windows/run_gateway.bat` with LF line endings (this working tree had CRLF from an
  older checkout, so the gate passed here). cmd.exe can mis-handle `goto` / labels in an LF-only batch file - the
  Windows gateway's restart loop could break. Caught by gateway/test_osdep.py OS12 ("Windows restart loop") only in
  the clone. Reproduction: `git clone <repo> x && python x/gateway/test_osdep.py` -> OS12 FAIL. Fix: `.gitattributes`
  line `gateway/windows/*.bat text eol=crlf` (after `gateway/**`, so it wins); the index stays LF, checkouts get CRLF.
- **F14-2 (storm oracle gaps, test code only, fixed in tests/acceptance/storm/storm.c).** Only seen with 2000 episodes
  (the per-step gate runs 200). Each reproduces with `test_results\storm.exe --replay SEED` on the old storm.c:
  - I29 seeds 330, 1970: "killed 6.0 s after the loss at open sea" - the kill was the OD-3 UNCREWED throttle-node-loss
    kill at once (`ll.node_kill`), which I27 already exempts and I29 did not. Correct product behaviour (CLAUDE.md T OD-3).
  - I32 seeds 206, 769, 1124, 1779: "generator active with the gear at -1" - on DroneCAN a node loss sends no gear
    frame (gear HOLD, OA-2); the gear servo stands at N (the switch confirms), the generator is already STOPPING
    (non-advisory alarm). I32 now counts HOLD with the simulated gear servo at N as N.
  - I36 seed 493: "engine relay 3 switched on with throttle 8.5 %" - relay 3 is SHUTOFF, ALWAYS allowed since step 11b;
    the oracle still applied the IGNITION conditions to it. (The same message as the step-11c seed 108 finding.)
  - I36 seed 1445: "a relay event changed the driving: CAN NODE LOST" - the alarm came from a storm DroneCAN node
    power loss 0.7 s earlier (lost 1.5 s after its last 1 Hz NodeStatus), not from the relay event; CAN NODE LOST is
    now not judged in I36 while a storm servo-node event runs (like the timer alarms of word 1).
  - I50 seeds 352, 651, 685, 830, 1166, 1670: "a node silent 1840 ms but not lost" - a chip / watchdog reset or a
    frozen core 0 came within the 1.5 s; the oracle counted the silence from before the reset, the new boot's node
    table starts at the boot (and STANDBY + the "not seen yet" interlock hold meanwhile). The silence is now counted
    from the boot and not while core 0 is frozen.
  After the fix all 14 seeds hold and the 2000-episode storm passes 0 fails.
- **F14-3 (known, not new).** test_driver I4 / I12 / I19 (live HUD; W10 / W11) and gateway E1-E3 under the gate's load
  (alone 6/6) - unchanged from steps 11c-13.

Open for the owner (nothing blocks code): a 24 h REAL-TIME system soak on the gateway PC itself; the board / bench
items of FIRST_TEST.md and the WAITING FOR OWNER list in PROGRESS.md.

## 000. Academy (steps 15-22) - final report (2026-10-29)

The academy of docs/ACADEMY_SPEC.md (parts A-J) is complete in the code; what is left needs the owner on site.

| Part | Step | Commit | What | Tests |
|---|---|---|---|---|
| design + "no boat link" | 15 | 5b41988 | academy/, simguard.py = the only door (simulator on this PC / files), banner, content format, levels model | AC1-AC14 |
| A accounts | 16 | 241ab4b | accounts + logbook on the shore PC, instructor view, signed levels file -> gateway (next login only), backup | AC15-AC20, gateway AL1-AL4 |
| B guided tour | 17 | d93e452 | every screen item (read from the station's own sources), Greek voice | AC21-AC24 |
| C + D lessons, quizzes | 18 | 5d82ebb | 5 lessons checked on the real simulator, 5 pools / 35 questions (choice, picture, sound) | AC25-AC29 |
| E scenarios | 19 | e38ef2e | 10 emergencies (simulator faults + academy-drawn hazards), reaction time, replay + debrief | AC30-AC33 |
| F video lessons | 20 | 0035e13 | lesson editor from saved events (trim, blur, questions), player with the HUD redrawn | AC34-AC37 |
| G + J exams, practical test, online | 21 | 02470c2 | exams + cool-down, 2FA, renewal / currency, certificate PDF, practical test (watch only + CONFIRM_ONLY 0x30), online rules | AC38-AC43, practical P1-P8, sim CO1-CO3 |
| H 3D sim interface + I manual / tests / screenshots | 22 | step 22 | academy/sim3d.py + window, docs/academy_sim_interface.md, MANUAL ch. 20 / MANUAL_EL ch. 15, docs/screens/academy/ (18 pictures) | AC44-AC50 |

Safety result: the academy never reaches a boat link (static + transitive + runtime proof, AC1-AC3); the only AURIX
change of the whole academy is the confirm-only request 0x30 that changes nothing on the boat (sim CO1-CO3, storm
event E_PC_CONFONLY, practical P1-P8); a level goes up only through the practical test (examiner 2FA + onboard
CONFIRM), never through an exam, a 3D mission, a quiz or anything online (AC38, AC40, AC41, AC46); the 3D simulator
interface never replaces the simulated AURIX and never takes a command (AC44-AC47); chart and drone photos never reach
a safety check (AC11, AC48). Every spec I rule is mapped to its tests (AC50). Gate at step 22: academy 50/50,
practical 8/8, every other suite of tools/check.sh as in PROGRESS.md row 22.

Open (owner, nothing blocks code): W30 the real Daskalio test area (on site with the instructor tool; the practical
test refuses the placeholder), W33-W35 / W39 content and defaults review (Greek texts, scores, time limits), and the 3D
world itself (a Godot project: docs/academy_sim_interface.md section 7 is its checklist).

## 00. Crew profiles, lost link, MANEUVER, roles, two screens, boats (2026-10-25, commits 4e3a1f8 .. this one)

Code under test: commit `988a537` (sections 1-5 of the task + the fix below). Full acceptance run
`tests\acceptance\RUN_ACCEPTANCE.bat full --short-soak` (= `run_acceptance.py full --short-soak`).
**The soak was SHORTENED** (AURIX soak 4 simulated hours instead of 24; system soak 15 min instead of
60). **The full 24 h soak must be run before the first uncrewed trip** (`RUN_ACCEPTANCE.bat full`).

| Part | Result |
|---|---|
| ADS excludes / iLLD sources | PASS (boats/ excluded in all 4 configurations) |
| Fresh clone from GitHub, every suite (tools/check.sh) | PASS (23.5 min): sim 188 + bridge 187 (1 N/A), gateway 100/100, boats 5/5, language 5/5, sound 10/10, charts, planner 14/14, FAT32 |
| Fault storm, 2000 episodes, 588 simulated hours | PASS: I1-I13, I19, I20 and the new I22-I25, I27-I31 - 0 failures (see below for the first run) |
| 32-bit time wraparound | PASS |
| AURIX soak - SHORTENED, 4 simulated hours with logging | PASS: all invariants, 286 498 control steps, longest 567 us (PC), 0 overruns, 0 log drops |
| System soak - SHORTENED, 15 min real time | PASS (AURIX step max 265 us on the PC, 0 overruns, 0 log drops) |
| Day at sea, AURIX part | PASS 15 / 15 |
| Day at sea, whole system | 11 / 12 on the first run: the script still answered the avoidance proposal from the old "driver screen" station, now a copilot - refused by the new rule (only the captain's screen 1 answers). Script fixed (tests/acceptance/day_at_sea.py) so the captain answers; **rerun: PASS 12 / 12** (28 min) |
| Gateway storm I14, I15, I17, I20, I21, **I26** | PASS (I26: 8 262 checks) |
| Station storm **I16** (new rule: copilot actions only after the captain's ACCEPT, in the captain's envelope) | PASS, 300 s: 263 frames, 142 driving (85 captain's own, 5 copilot after ACCEPT), 0 violations; self-test catches a planted frame |
| Coverage | PASS (smallboat.c 97 % lines / 87 % branches, link.c 89 % / 78 %) |

The system parts first failed at once in the clean worktree: `sim\autopilot_sim.exe` is not in git and
the runner didn't build it. **Fixed:** run_acceptance.py now builds the simulator first. The three parts
were then run on the same code: results above.

New invariants (storm.c, numbering after I21):

| | Invariant | Checks (2000 episodes) |
|---|---|---|
| I22 | UNCREWED: the onboard joystick has no effect (no MANUAL / HDG LOCK, CONFIRM only from the shore) | 16.7 M |
| I23 | the profile CREWED / UNCREWED never changes outside STANDBY (stopped); never CREWED -> UNCREWED without a power-up | 23 037 |
| I24 | MANEUVER never commands throttle above idle | 286 880 |
| I25 | MANEUVER gear changes only at idle | 286 880 |
| I26 | MANEUVER / REMOTE refused for users without a manual licence (gateway storm + gateway test CR1) | 8 262 |
| I27 | ENGINE KILL only after idle + neutral | 1 630 kills |
| I28 | lost link near shore / missing or stale shore info: kill at once (once idle + neutral) | 1 194 |
| I29 | lost link at open sea: no kill before the grace time, kill after it unless the link returned | 12 959 |
| I30 | never an ENGINE KILL started in CREWED | 1 630 |
| I31 | the kill stays latched until a reset on board | 2.85 M |

25 % of the storm episodes start UNCREWED; the storm adds joystick plug / unplug, shore CONFIRM,
MANEUVER requests, HELM drop / return and shore-info changes.

**Finding (fixed, commit 988a537): the first 2000-episode run failed I10 once (seed 389).** AUTO
START was asked while in MANEUVER; then the gateway went quiet, HELM was lost, the boat went to STANDBY,
and the CONFIRM pressed in the same 50 ms step started AUTO from STANDBY (two allowed transitions in one
step, but the request should not survive a STANDBY caused by a fault). The same was possible from
REMOTE before. Fix: entering STANDBY cancels any pending CONFIRM request; sim test MV4. Other storm
findings during the work (all fixed before their commits): a CREWED joystick that failed in STANDBY
could switch the boat to UNCREWED (now only a power-up without a joystick gives UNCREWED); a profile
change UNCREWED -> CREWED kept the outputs armed with a held stick (now the outputs re-arm).

**ADS (TASKING) build:** not run in this session - the headless ADS builder can't run while the ADS
IDE is open on this PC, and ADS's own make needs the IDE. Checked instead: the TriCore gcc compile of
every firmware file (app/, hal/, Cpu0/1/2, the big-boat and FIRST TEST variants) with -Wall -Wextra:
**0 errors, 0 warnings**; every Ifx* function the firmware needs is built in all 4 configurations; no
new C file was added (the ADS makefiles stay valid). **To do on the PC: Project -> Build in ADS**, then
`python tools/mapreport.py "TriCore Debug (TASKING)/TRYGONS_C2.map"`.
Memory (expected, from the objects; tools/mapreport.py --objects): core 0 150.7 KB / 240 KB (62.8 %),
CPU1 12.1 KB (5.0 %, limit 60 %), core 2 78.1 KB / 96 KB (81.3 %), **LMU blocks 0 / 1 / 2: 53.9 KB / 64 KB
(84.2 %) each - unchanged** (the black box; nothing new was placed in the LMU; the new data - joystick
probe, shore joystick, lost-link and shore info - is in core 0's RAM).

Still to be checked on the real boat / PC: the engine-kill relay (not chosen yet - no uncrewed operation
before it is fitted and bench-tested), the shore joystick's CONFIRM button mapping, the boat list with the
real Tailscale addresses, screen 2 on a 1600 px laptop (it needs ~1840 px in Greek), the screenshots in
docs/screens (not regenerated; they still show the old separate map window).

## 0. Status after the fixes (2026-10-23, commits 6a35b25 .. 8054948)

Every finding of section 2 was worked on, in the order of section 5 (one commit each, with a sim /
gateway test that fails without the fix where that is possible). The rest of this report (sections
1-5) is the ORIGINAL test result, kept as it was.

| ID | Status | Commit | What was done |
|---|---|---|---|
| T11 | fixed | 6a35b25 | iLLD library + .cproject in git (.gitignore rule anchored); fresh clone compiles for TriCore |
| R1 | fixed (rule kept) | 5768d2e | STANDBY: stick (MANUAL) / gyro (REMOTE) steers until idle + neutral; STANDBY closing rate 100 %/s |
| T2 | fixed | d90d6ac | HDG LOCK / AUTO / REMOTE / TRAINING refused while the stick is held ("centre the joystick") |
| R3 | fixed (bench test open) | 5449c11 | GTM TOM servo PWM written; hal_safe_outputs drives idle / neutral itself |
| R5 | fixed | cf896af | NaN / inf / out-of-range GPS fix = rejected fix (also the second GPS, DR inputs) |
| R4 | fixed | 9d6ab75 | monitor watches the heartbeat only after core 0 runs; 3 s boot timeout |
| R2 | fixed | 1312250 | big-boat build feeds the CPU1 monitor |
| T1 | fixed | a38afbc | STARLINK never auto-selected while a normal channel works (0 in 500 storm runs) |
| (ADS) | fixed | 2da572a | ADS build excludes every non-firmware folder; check in check.sh + RUN_ACCEPTANCE |
| T3 | fixed where safe | affa89f | ~60 s re-trust with a second GPS or paddlewheel; without: old rule (a slow along-course drag can't be told from a speed error) |
| T4 | fixed | 8b9e68a | START_DEMO builds the simulator on a fresh copy |
| R6 | fixed | d8381b5 | navreq heading needs a cap, max 150 deg off the leg, no stale HDG LOCK heading |
| R9 | fixed | 147642c | outputs arm only with the joystick seen and centred |
| R11 | fixed | 6b47fae | alarm ACK clears only alarms whose cause is gone |
| R10 | fixed (driver contract) | c2873db | CPU2 watchdog serviced after every SD call; hal_sd_* <= 200 ms contract |
| R8 | fixed (verify map) | f6bf7df | shared data explicitly in core 0's DSPR; mapreport checks it |
| T5 | fixed | e180125 | recording writes every packet: no false stall on slow streams |
| R20 | fixed | 5dd443b | "AURIX alive" only from a valid frame |
| R21 | fixed | 8c97be9 | camera config checked; NaN distance counts as in the path |
| R7 | documented (design) | 88ebc43 | trust boundary written down: driving changes always need on-board checks |
| R12 | fixed | (with each fix) | tests C3b, C3c, C3d, R5, NV9, R11, F0, F1b, R17, R18 |
| R13 | fixed | 4b1fb6f | sim test hook not in the firmware (checked by the gate) |
| R14 | fixed | e50c722 | no fault-steer setting turns the steering off at once (OFF = centre) |
| R15 | not a finding | - | - |
| R16 | fixed | d76338e | corrupted log queue length resynced |
| R17 | fixed | 99067b5 | oversized telemetry counted, test shows the headroom (7 bytes) |
| R18 | fixed | bed41fe | link stops reading after 64 frames per step |
| R19 | fixed | cf896af | second GPS range check (with R5) |
| R22 | fixed | 6f43185 | monitor seqlock |
| T7 | fixed | 6440a2d | dead "synced" flag removed |
| T12 | fixed | 4a7536b | secrets.json / *.key / *.pem git-ignored |
| coverage servo.c | fixed | 8054948 | clamp corner-case test; found + fixed: NaN pulse was FULL on a reversed servo |
| T6 | open (watch) | - | MediaMTX handle trend: check in a long soak on the Pi |
| T8, T9, T10 | info | - | unchanged (spec wording / operation notes) |

New worst reaction times (fault storm, 2000 episodes): STANDBY -> idle + neutral **1.41 s** (was 2.16 s;
the 2.88 s below was a press during a core-0 reboot with the outputs already safe), onboard joystick
**0.09 s**, remote data loss **0.55 s**, CPU1 monitor **0.21 s**. Still open without hardware: everything in
section 4 (board, Pi, boat) - including the bench test of the new servo PWM (R3) and the FatFs driver
that must keep the SD time bound (R10).

**Acceptance test after the fixes** (`RUN_ACCEPTANCE.bat quick`, commit f44f237, cloned locally because
the fixes were not pushed yet): **6 / 6 PASS** - ADS build excludes; fresh build 13 / 13 (simulator
162 + 161 tests, charts, avoidance, gateway F1 / SH / VI, driver screen, START_DEMO from the clone);
fault storm 200 episodes (all invariants, held-stick NOTE 0); 32-bit time wrap; day at sea (AURIX part)
15 / 15; gateway storm (I14, I15 0 STARLINK switches, I17, I20, I21).

## 1. Summary (10 lines)

1. **The control software (AURIX) is in good shape in the simulator.** 2000 random "bad days"
   (588 simulated hours) and a 24-hour soak broke none of the safety rules that are checked on the
   boat side (I1–I13, I18–I20), and the checks were proven to catch planted violations.
2. The STANDBY button won in 0.0 s (mode) / at most 2.9 s to idle + neutral; the onboard joystick
   took over within 0.09 s; remote-data loss gave idle + neutral within 0.55 s; the CPU1 monitor
   reacted within 0.21 s – every time.
3. **One CRITICAL point (R1) needs your decision:** on STANDBY from MANUAL or REMOTE the steering
   stops at once while the throttle is still coming down (up to ~3.5 s). CLAUDE.md says "idle +
   neutral first, then steering off". Either the code or the rule has to change.
4. **Five HIGH points, all for the real board:** the driver library for the board is missing on
   GitHub (T11, a one-line .gitignore fix); the big-boat build would trip the CPU1 monitor
   (R2), the real "force safe outputs" does not drive the servos yet (R3), the monitor can trip
   during a slow boot (R4), a NaN GPS value would pass all GPS checks (R5).
5. **MEDIUM:** the gateway can pick STARLINK while a normal channel works (after a channel trial,
   or in MANUAL fallback); after STANDBY with the stick held, HDG LOCK can be engaged and the held
   stick then does not take over; GPS trust comes back slowly (~7 min) after a spoof at speed;
   START_DEMO fails on a fresh clone until the simulator was built by hand; and 7 review points.
6. The day at sea (part A 15 / 15, part B 12 / 12) and the gateway storms behaved as specified,
   apart from these points.
7. The real board is not tested at all: hal_tc375.c is still mostly stubs (no sensors, PWM, UARTs,
   buttons, joystick, flash, SD card) – see section 4.
8. The camera obstacle detection is licensed correctly (Apache-2.0 / CC BY 4.0) and works on the PC
   (~2 fps on 2 cores); its speed on the Pi 5 is NOT known yet.
9. Coverage is high on the AURIX code (smallboat.c 97 % of lines); the gaps (servo.c branches, the
   big boat, the real router / UART code) are listed in 3.7.
10. **Fix order:** R1 decision, T11 now (5 min), R3 + R5 + R4 before the first powered bench test, then the MEDIUM
    gateway / joystick points, then the rest (section 5).

## 2. All findings (CRITICAL first)

ID: R = independent code review (subagent, code reading, `tests/acceptance/code_review.md`),
T = found by the tests here. "checked" = I confirmed it in the code or by a test.

| ID | Severity | Where | Finding (plain) | Proposed fix |
|---|---|---|---|---|
| R1 | CRITICAL (checked: code + sim tests expect it) | app/smallboat.c:459-475, 1810 | On STANDBY from MANUAL / REMOTE the steering stops in the same step while the engine still pushes for up to ~3.5 s (throttle ramp + 1 s gear wait). The rule says idle + neutral first. | Hold the last outboard angle (or gyro hold) until idle + neutral, then steering off – or write the MANUAL exception into CLAUDE.md. |
| T11 | HIGH (build, test; not a safety problem) | .gitignore line 6 | The Infineon driver library (778 files in Libraries/iLLD/TC3xx/Tricore/) is not on GitHub: the ignore rule `TriCore*/` for the ADS build folder also matches "Tricore" (git on Windows ignores case). A fresh clone can't build the board firmware; tools/check.sh fails there. Repro: `fresh_build.py` -> `Ifx_Types.h: No such file`. | Change the rule to `/TriCore*/` (only the build folder at the top), then `git add Libraries/iLLD` and commit. |
| R2 | HIGH (checked) | Cpu0_Main.c (big boat), app/monitor.c | The big-boat build never sends the CPU1 heartbeat: the monitor trips after 200 ms and the rudder can never move. No test. | Publish the heartbeat (and safe throttle/gear values) from autopilot_step; add a big-boat + monitor sim test. |
| R3 | HIGH | hal/hal_tc375.c:167-176 | The real "force safe outputs" only sets a flag; idle / neutral pulses reach the servos only when core 0 writes again – never if core 0 is frozen. Small race too. | Write idle / neutral directly into the PWM hardware in hal_safe_outputs; re-check the flag after every write; bench test. |
| R4 | HIGH | Cpu1_Main.c:196-199, app/monitor.c:97-107 | CPU1 waits only 1 ms for core 0 at start; the heartbeat timeout counts before the first heartbeat. A slow boot (> 200 ms) trips the monitor and the boat won't start. | Check the heartbeat only after the first one, with a separate boot timeout (e.g. 2 s). |
| R5 | HIGH (checked) | app/dr.c:277-397 | A GPS fix is never checked for NaN / impossible values; a NaN passes every integrity test and spoils the position while GPS still looks trusted. | Reject fixes that are not finite or out of range (and count them as rejected); sim test with a garbage fix. |
| T1 | MEDIUM (test) | gateway/channels.py | STARLINK chosen while 4G / RF works: (a) a channel trial that runs out goes back to STARLINK (AUTO and MANUAL); (b) MANUAL fallback when the chosen channel works again but is not "stable" yet. 33 of 120 random 2-hour runs. Repro: `storm_gateway.py --part channels --seed 7`. | On a trial revert re-decide (unless the previous setting was an explicit STARLINK choice); in fallback prefer the healthy chosen channel over STARLINK. |
| T2 | MEDIUM (test) | app/smallboat.c (HDG LOCK button, STANDBY "ignored until centred") | STANDBY pressed with the stick not centred → the stick is ignored until centred; HDG LOCK (or AUTO / REMOTE) can still be engaged, and the held stick then does NOT take over. 97 times in 2000 episodes. Repro: storm seed 836 / 12. | Engage HDG LOCK / AUTO / REMOTE only with the joystick centred (like the hand-back), or let a deflection count once the stick was centred after STANDBY. |
| T3 | MEDIUM (test) | app/dr.c (GPS re-trust) | After a GPS spoof ends at ~7 kn, GPS is trusted again only after ~6.7 min (dead reckoning 106 → 177 m meanwhile); at 3.6 kn after 60 s. Safe (AUTO keeps going on DR / holds), but slow. Repro: `STORM_SLAM=1.6 STORM_DEBUG=1 storm.exe --dayatsea`. | Re-check the acceptance window (fix vs DR within the DR uncertainty) and the N-consistent-fixes rule at speed. |
| T4 | MEDIUM (test) | demo/demo.py build_sim() | START_DEMO fails on a fresh clone until the simulator was built by hand: `cmd /c sim/build_sim.bat` (MSYS2 Python joins with "/") is not understood by cmd. | Use `sim\build_sim.bat`. |
| R6 | MEDIUM | app/link.c:647-668, smallboat.c:239-270 | A navigation HEADING from the Pi is accepted with any value; an accepted HDG LOCK heading can still be applied up to 3 s after entering HDG LOCK. | Limit the deviation from the track, require a cap with a heading, apply HDG_SET only if it arrived while already in HDG LOCK. |
| R7 | MEDIUM | app/link.c:686-819 | The AURIX trusts the station id / "local" flag sent by the Pi (a faulty Pi could act as a boat station). CONFIRM still protects AUTO / REMOTE / power. | Document it as design, or keep a fixed list of local station ids on the AURIX. |
| R8 | MEDIUM | app/monitor.c, logq.c | Memory placement of the shared monitor data / log queues (cache) is not fixed; unverified until the first ADS build. | Place them in non-cached memory; check with tools/mapreport.py after the first build. |
| R9 | MEDIUM | app/smallboat.c:1567 | If no joystick is detected at boot, the outputs arm without the stick check. | Wait for the joystick (or a "no joystick" setting) before arming. |
| R10 | MEDIUM | Cpu2_Main.c:47-52 | CPU2 watchdog 500 ms; a slow SD-card write could reset the whole chip. | Bounded SD writes, kick the watchdog between chunks. |
| R11 | MEDIUM | app/link.c:530-534 | ALARM_ACK from any HELM station clears every alarm; link-lost alarms are not raised again while the link is still down. | Clear only alarms whose cause is gone; restrict ACK from remote stations. |
| R12 | MEDIUM | tests | No tests for the error paths of R2, R4, R5, R6, R11, R16. | Add sim tests. |
| T5 | LOW (test) | gateway/recorder.py | The recording stall check (file growth within 10 s) restarts recording on very low-bitrate streams (seen with the demo's test pictures). | Use the MediaMTX byte counter or a longer stall time. |
| T6 | LOW (test) | MediaMTX (third party) | 30-min soak: MediaMTX handles 227 → 289 (weak upward trend, R² 0.64); memory steady. | Watch it in a longer soak on the Pi; restart MediaMTX daily if it grows. |
| T7 | LOW (test) | gateway/recorder.py | The `"synced"` flag in event.json is never set to True (dead condition). | Remove it or set it when the shore confirms. |
| R13 | LOW | smallboat.c:168-173 | The sim test hook sb_test_drive is compiled into the firmware. | `#ifdef SIM`. |
| R14 | LOW | smallboat.c:471-474 | With fault_steer ≠ GYRO a compass loss while moving turns the steering off at once. | Remove the option or centre instead. |
| R15 | – (not confirmed) | CLAUDE.md | Review said OBSTACLE AHEAD is missing from the exception list – CLAUDE.md already lists it. | none |
| R16 | LOW | app/logq.c:53-66 | A corrupted length in the log queue breaks it for good. | Check the length, resync. |
| R17 | LOW | app/link.c:246-370 | Telemetry is 193 of 200 bytes; one more field = telemetry silently never sent. | Static assert / test on the size. |
| R18 | LOW | app/link.c:1059 | > 64 frames in one step: further bytes overwrite unparsed frames. | Stop reading at 64 frames. |
| R19 | LOW | app/link.c:792-800 | GW_GPS2 lat / lon not range-checked. | Range-check. |
| R20 | LOW | gateway/aurix.py:56-58 | "AURIX alive" updated on any byte (also noise). | Only on a good frame. |
| R21 | LOW | gateway/vision/geometry.py | No guard for a zero field of view; a NaN distance hides an obstacle. | Validate the camera config; treat NaN as "in the path". |
| R22 | LOW | smallboat.c:1882, monitor.c:91 | Monitor data written field by field (a false trip just after a limit change is possible but unlikely). | Seqlock, or write the limits before the throttle. |
| T12 | LOW (review, checked) | .gitignore | `gateway/secrets.json` (real sync key / passwords on the Pi) is not git-ignored; nothing secret was ever committed (whole history searched), but one `git add .` on the Pi would. | Add `gateway/secrets.json` and `*.key` to .gitignore. |
| T8 | INFO | spec wording | I1 "never above the power limit": by design the throttle may be above a just-lowered limit while it comes down at 40 %/s (CPU1 allows 2.5 s). Tested that way. | Say so in CLAUDE.md. |
| T9 | INFO | operation | Raising the power in HDG LOCK needs the stick centred for CONFIRM = throttle to zero for a moment. | Mention in the MANUAL (power limit chapter). |
| T10 | INFO | gateway/stations.py | The driver-ACCEPT list (DRIVING_CMDS) does not include the remote-driving request or power raise; both need the onboard CONFIRM anyway. | Confirm this is intended. |

## 3. Results per part

### 3.1 Fresh build from GitHub (4.1)
`fresh_build.py`: clone from GitHub (`e385f86`) into an empty folder, then only the steps in
MANUAL chapters 3, 4 and 4A. **12 of 13 steps PASS; 1 FAIL = finding T11.**

| step | result | detail |
|---|---|---|
| git clone | PASS | 1 s |
| sim\build_sim.bat | PASS | BUILD OK |
| autopilot_sim all | PASS | 146 passed, 0 failed |
| autopilot_sim all --bridge | PASS | 145 passed, 0 failed, 1 not applicable |
| python tools\get_mediamtx.py | PASS | MediaMTX v1.21.2 into the clone's tools\bin |
| sh tools/check.sh (the gate) | **FAIL** | everything passes (sim 146 + 145, charts 5, avoidance 14, gateway 82) **except the TriCore compile of hal_tc375.c and Cpu0/1/2_Main.c: `Ifx_Types.h: No such file`** – T11 |
| groundstation test_e2e / test_logs / test_driver | PASS | 10 / 3 / 8 passed |
| START_DEMO from the clone | PASS | all 5 ports up, STOP_DEMO stopped everything (here the simulator was already built by the step above; on a clone where nobody built it, T4) |
| paths that only exist on this PC | PASS | none found |
| not in git but made by documented tools | PASS | charts (tools/chartprep.py), vision model (tools/get_vision_model.py), vision Python (tools/get_vision_python.py) |

**T11:** `.gitignore` line 6 `TriCore*/` is meant for the ADS build folder "TriCore Debug (TASKING)",
but git on Windows ignores letter case, so it also hides `Libraries/iLLD/TC3xx/Tricore/` –
778 files of the Infineon driver library are not on GitHub. The board firmware can't be built from
a clone (and would be lost with this PC).

### 3.2 A day at sea (4.2)
Two parts (the simulator server can't be told sea state, compass glitches or a GPS spoof from
outside, and Part 4 may not change the simulator): part A in the C harness (simulated time),
part B through the whole system in real time.

**Part A (AURIX, `storm.exe --dayatsea`, ~50 simulated minutes): 15 / 15 PASS, invariants held all day.**

| time | what happened | what the system did | expected | result |
|---|---|---|---|---|
| 3 s | power-up with the joystick held | armed 0, throttle 0.0 %, gear N | outputs not armed, idle, neutral | PASS |
| 4 s | joystick centred, neutral confirmed | armed 1, power limit 30 / 5 % | armed; power limit 30 / 5 | PASS |
| 281 s | TRAINING (PC, calm water), result accepted | started 1, result reached (code 0), accepted 1, mode 0 | calibration runs, idle + neutral, accept -> STANDBY | PASS |
| 341 s | HEADING LOCK, joystick throttle, +1 +10 -1 -10 | mode 2, 7.0 kn, target +1 +11 +10 +0 | mode HDG LOCK, speed > 3 kn, target +1 +11 +10 0 | PASS |
| 358 s | power limit raised 30 -> 50 -> 75 % | without CONFIRM 30 %; then 50 %, 75 % | no CONFIRM: stays 30; each step needs CONFIRM | PASS |
| 360 s | AUTO program (6 waypoints, 6-16 kn) started with CONFIRM | program 0, start 0, CONFIRM 1, mode 3 | program accepted, AUTO after CONFIRM | PASS |
| 450 s | AUTO on leg 1 | mode 3, state 0, 8.0 kn (target 8.0 kn) | running, about the leg speed (limit 75 %) | PASS |
| 956 s | sea state 4, slamming on the fast leg | impacts 9, lowest cap 30 %, power limit 75 %, state 0 | impacts, throttle reduced in steps, power limit kept | PASS |
| 1209 s | GPS lost for 4 min | DR 1, after 4 min mode 3, uncertainty 44 m; GPS trusted again 1 | dead reckoning, AUTO continues (slower), GPS trusted again | PASS |
| 1270 s | GPS spoof attempt (800 m jump) | rejected 1, alarm 1, recovered 1, mode 3 | rejected, GPS SUSPECT, dead reckoning, recovers | PASS |
| 1276 s | compass glitch 0.8 s | mode 3, new compass alarm 0 | no reaction (below the 2 s timeout) | PASS |
| 1280 s | joystick takeover in AUTO, then resume | MANUAL 1, SAVE 1, resume 0 + CONFIRM 1 -> mode 3 | MANUAL at once + SAVE; resume = PC + CONFIRM | PASS |
| 1296 s | remote driving, 2 s link dropout | request 0, CONFIRM 1, driving 1; LOST after 0.30 s, data back -> still LOST/idle 1, STANDBY 1 | LOST within 0.4 s: idle + neutral + gyro hold, never resumes, STANDBY after 5 s | PASS |
| 2966 s | arrival at the last waypoint (STOP) | resume 0 + CONFIRM 1, arrived 1, throttle 0.0 %, gear N, mode 0 | ARRIVED, idle + neutral | PASS |
| 2966 s | STANDBY | mode 0 | STANDBY | PASS |

**Part B (whole system, `day_at_sea.py`, ~30 min real time: simulator + gateway + 4 cameras + AIS + vision + channels + sync + captain / driver screens): 12 / 12 PASS.**

| t | gateway time | what happened | what the system did | expected | result |
|---|---|---|---|---|---|
| 23 s | 10:00:22 | HEADING LOCK, crossing ship | proposal "239500001: CPA 0.01 nm in 3.3 min. Steer 180 and slow to 5 k"; accepted -> locked heading 180 | proposal on the driver screen, ACCEPT changes the heading | PASS |
| 45 s | 10:00:44 | AUTO program (6 waypoints, past the 1.5 m shoal) | upload 0, driver asked True, AUTO running | accepted by the depth check, AUTO after ACCEPT + CONFIRM | PASS |
| 188 s | 10:03:07 | AIS crossing ship during AUTO | request applied yes (planner SLOW), back to the leg yes | slow down, route astern (or DRIVER DECISION slow), then rejoin | PASS |
| 191 s | 10:03:10 | vision blip + kayak without AIS, STAGE 3 off | blip filtered True; OBSTACLE AHEAD yes, SAVE yes, no cap True | blip ignored; alarm + SAVE, no effect on the boat | PASS |
| 200 s | 10:03:19 | kayak without AIS, STAGE 3 on (driver ACCEPT) | speed cap 0.0 kn applied yes, throttle 24.0 -> 0.6 % | AUTO slows / stops, never more throttle | PASS |
| 211 s | 10:03:31 | 4G fails -> RF; RF fails -> STARLINK | RF True, STARLINK True, data saver + no sync True, banner flag True | RF first, then STARLINK emergency: saver, no sync, banner | PASS |
| 228 s | 10:03:47 | 4G returns | back on 4G True (after 15.0 s stable), sync allowed True | automatic switch back, sync resumes | PASS |
| 437 s | 10:07:17 | GPS lost for 4 min | dead reckoning yes, mode after 4 min AUTO, GPS trusted again yes | DR, AUTO continues, GPS back | PASS |
| 452 s | 10:07:31 | remote driving, 2 s dropout | request 0, REMOTE yes; LOST after 0.38 s; data back -> still LOST + idle True; STANDBY yes | idle + neutral + gyro hold, never resumes, STANDBY after 5 s | PASS |
| 454 s | 10:07:33 | joystick takeover during AUTO, resume | MANUAL yes, SAVE request yes -> event (new or merged) True, resumed yes | MANUAL at once + SAVE; resume = PC + CONFIRM | PASS |
| 1780 s | 10:29:40 | arrival at the last waypoint | arrived yes, mode STANDBY, throttle 0.0 %, gear 0 | STOP: idle + neutral | PASS |
| 1801 s | 10:30:00 | sync to the shore receiver | 181 files confirmed: 180 checksums equal, 0 different [], 1 changed after the confirmation (queued again), 0 not found on shore; 4 saved events on the SSD; queue {} | every confirmed file identical on shore | PASS |

(The first run showed a FAIL at the arrival; it was the test reading an old telemetry frame - fixed in the test tool, the boat was idle + neutral.)

### 3.3 Soak (4.3)
- **AURIX, 24 simulated hours with logging** (`storm.exe --soak 24 24 --log`): all invariants held,
  1,725,520 control steps, 225 reboots (resets in the schedule), 0 core-0 overruns, 0 log queue drops,
  1.38 million log records (251 MB) written.
- **Control step time with / without logging (PC, 2 h):** longest 498 / 167 µs, average 2 / 0 µs,
  99.9 % below 100 µs – of a 50,000 µs budget (I18). The TC375 must be measured on the board.
- **32-bit time wraparound:** started 3 min before 2^32 ms, ran 10 min across it: all invariants held,
  11,997 control steps (expected ~12,000), 7 reboots, 0 overruns.
- **System soak, 30 min real time** (gateway + 4 cameras via MediaMTX + recording + vision + AIS +
  sync + channel changes, 31 SAVEs, 25 channel changes, 115 mode presses): memory steady for the
  gateway (+7 MB/h, startup), simulator, recorders, vision; MediaMTX handles +54/h (T6); 193 ring
  files deleted on time; 228 files synced, backlog max 28.5 MB, 0 at the end; AURIX 0 overruns,
  0 log drops; vision ACTIVE at the end (0–2 fps on the busy PC, 1.2 GB private memory).
  Real time only: 24 h of gateway time must be run on the Pi (`soak_system.py --minutes 1440`).

### 3.4 Fault storm – safety invariants (4.4)
`run_storm.py --episodes 2000`: **2000 episodes, 588 simulated hours, 3.9 min on this PC (10 workers).**
Self-test first: 3 planted violations (power raised without CONFIRM, a forbidden mode change, a
throttle written above the limit) were all caught.

| invariant | checks | fails | worst reaction |
|---|---|---|---|
| I1 throttle ≤ power limit (coming down at the closing rate allowed) / remote cap | 41,518,013 | 0 | – |
| I2 30 % / 5 % after every power-up / reset | 25,714 | 0 | – |
| I3 gear only at idle; idle → neutral → gear → throttle | 277,431 | 0 | – |
| I4 STANDBY always wins | 15,600 | 0 | 2.88 s to idle + neutral |
| I5 the onboard joystick wins | 7,398 | 0 | 0.09 s |
| I6 PC / remote never take control from the joystick | 6,130 | 0 | – |
| I7 power raise / AUTO / REMOTE only with CONFIRM within 10 s | 5,854 | 0 | – |
| I8 CONFIRM alone never changes the mode | 21,009 | 0 | – |
| I9 idle + neutral commanded before / with steering off | 206,934 | 0 | – (see R1: *reached* is not checked) |
| I10 only allowed mode transitions | 37,073 | 0 | – |
| I11 disarmed at power-up until neutral + centred | 13,276,473 | 0 | – |
| I12 remote data old / missing → idle + neutral, never resumes | 177,312 | 0 | 0.55 s |
| I13 slamming / caps never increase the throttle | 41,528,945 | 0 | – |
| I14 avoidance / programs never over shallow water, never across a bow (gateway storm) | 23,631 | 0 | – |
| I15 STARLINK never for sync / never auto-selected while a normal channel works | 864,418 | 33 | – (T1) |
| I16 driving actions only with driver ACCEPT / single-operator (station storm + self-test) | 54 forwarded / 17 driving | 0 | – |
| I17 un-synced saved events never deleted | 1,486 | 0 | – |
| I18 logging never delays the control step | 2 × 143,410 steps | 0 | longest 498 µs (PC) |
| I19 CPU1 monitor safe state on every injected violation | 14,288 | 0 | 0.21 s |
| I20 navigation / vision caps never steer, never raise the throttle (C + gateway) | 41.5 M + 1.29 M | 0 | – |
| I21 vision crash / overload never releases a cap by itself | 864,000 | 0 | – |

AUTO was entered 1122 times (always PC + CONFIRM), REMOTE 3687 times. The first run showed 20
failing seeds; 19 were mistakes in my own checks (fixed in the test tools, never in the product)
and one was the real finding T2. Every failing seed can be replayed: `test_results\storm.exe --replay SEED`.

### 3.5 Vision (Part 2)
- Licence: eWaSR code + weights Apache-2.0, MaSTr1325 CC BY 4.0 – allowed (checked twice:
  by me and by the reviewer).
- Model: ewasr_resnet18 (official ONNX, 512 × 384). PC speed: 115–190 ms per picture on 2
  threads (with "denormal as zero"; 1500 ms without), 7.8 fps on the MaSTr1325 pictures, ~2 fps in
  the gateway (limited by max_fps). 99.9 % of pixels right on 100 MaSTr1325 pictures (but the
  model was trained on them – optimistic).
- False blips: single-frame detections never became obstacles (gateway tests VI2, day at sea B, the
  vision storm). Worker crash → OFFLINE notice, restart, no mode change, an active cap held until the
  driver's CLEAR (VI7, VR4, storm I21).
- **Not known: the speed on the Pi 5** (expect several hundred ms per picture; the INT8 model is
  ready to try).

### 3.6 Independent code review (4.5)
A separate subagent with no history read CLAUDE.md, MANUAL.txt, WIRING.md and the code:
1 CRITICAL, 4 HIGH, 7 MEDIUM, 10 LOW (one, R15, not confirmed). All are in the table in section 2;
the full review (with the hal_tc375.c stub list) is `tests/acceptance/code_review.md`.
Secrets: none in the git history; add `gateway/secrets.json` to .gitignore (T12); make sure the demo /
test users never exist on the real gateway.

### 3.7 Coverage (4.6)
Measured with gcov (C: simulator suites with both joysticks, the fault storm, day at sea A, time
wrap) and coverage.py (Python: every gateway section, avoid, the gateway / station storms, the
ground-station suites). Full tables: `test_results/coverage_summary.txt`.

**AURIX code (app/), lines / branches:** smallboat.c 97 % / 88 %, training.c 99 / 90, dr.c 100 / 91,
joystick.c 100 / 93, slam.c 100 / 99, monitor.c 100 / 88, powerlim.c 98 / 78, link.c 89 / 78,
logger.c 93 / 75, autopilot.c (big boat) 91 / 68, servo.c 100 / **60**, modes.c 100 / 69.

**Gateway / ground station (lines):** avoid.py 96 %, shallow.py 95, sync.py 94, recorder.py 90,
stations.py 89, gateway.py 88, ais.py 88, vision/manager.py 83, channels.py 80, aurix.py 62,
groundstation/app.py 70.

**Safety-relevant code that no test runs (the gaps):**
- `app/servo.c`: every line runs, but only 60 % of its branches – the clamp's corner cases
  (e.g. reverse at the limit, limit lowered during reverse) are not all exercised. Add unit tests.
- `app/autopilot.c` (big boat): `manual_loop`, `autopilot_manual`, `autopilot_load_calibration`,
  `autopilot_set_outboard_config` – and no big-boat test runs with the CPU1 monitor (see R2).
- `app/link.c`: `send_prog_wp`, `link_param_info`, `link_param_count` (reading a program / the
  parameter list back to the PC).
- `app/powerlim.c`: `plim_result_name` (text only).
- `gateway/channels.py`: the real router driver (`_ssh`, `apply`, `status`, `read`) – only the
  simulated network is tested; the RutOS router must be tested on the bench.
- `gateway/gateway.py`: `_serial` / `reader` – the real UART to the AURIX (only TCP in the sim);
  `gateway/aurix.py` 62 % for the same reason (and R20).
- `gateway/vision/manager.py`: `throttle_bits` (Pi throttling / heat check – PC has none),
  `_front_url` (camera switch while vision runs).
- `gateway/vision/worker.py`: runs in its own Python and is not measured; its parts are covered by
  the VR tests (real model) and the evaluation tool, not line by line.
- Ground-station tools never run by a test: logview.py, log2csv.py, replay.py, tlgreplay.py,
  program.py (45 %) – not safety-relevant, but program.py loads waypoint files (the AURIX validates
  them again).

## 4. What can't be tested without the real board, the Pi and the boat
- **The board:** hal_tc375.c is mostly stubs – compass, gyro / BNO085, GPS (NMEA), throttle / gear /
  steering PWM, panel buttons (STANDBY!), joystick ADC, both UARTs, the USB bridge input, DFLASH,
  SD card (list: code_review.md). Real timing of the 20 Hz loop, the watchdogs, the CPU1 monitor
  latch with real PWM (R3), boot timing (R4), cache / memory placement (R8), the TASKING build.
- **The Pi 5:** vision speed and temperature, MediaMTX + 4 cameras + recording + sync together over
  hours, the real router (RutOS) and Starlink, the dAISy HAT, the real SSD.
- **The boat:** the outboard / steering motor model, sea behaviour, slamming levels, the real
  compass / GPS noise, TRAINING on the water, AIS reception range, camera views, the lookout.

## 5. Suggested order for fixing
1. **R1** – decide: hold the steering until idle + neutral, or change the rule (it is about safety).
2. **Now, 5 minutes:** T11 – fix the .gitignore line and commit the driver library, so the
   firmware can be built from GitHub.
3. **Before the first powered bench test:** R3 (safe outputs drive the PWM), R5 (GPS NaN / range
   checks + test), R4 (boot vs heartbeat), R2 (big boat heartbeat, if the big boat is used), and
   hal_get_button_presses / hal_read_joystick (STANDBY and CONFIRM must work on the board).
4. **Before sea trials:** T2 (engage autopilot only with a centred stick), R6 (limit navreq heading),
   R9 (arm only with a joystick seen), R11 (alarm ACK), R10 (SD watchdog), R8 (check the map file).
5. **Gateway:** T1 (STARLINK selection), T5, R20, R21; T4 (START_DEMO on a fresh clone).
6. **Then:** T3 (GPS re-trust time), R7, R12 tests, the LOW points, the coverage gaps.
After each fix: `sh tools/check.sh`, then `tests\acceptance\RUN_ACCEPTANCE.bat`.

## TR2 - trend panel «ΤΑΣΕΙΣ» (data + graphs), 2026-11-10
Station only (groundstation/trends.py, trendpanel.py, tools/trend_load.py); tests groundstation/test_trends.py TG1-TG4 +
TG6-TG10 (10 / 10 pass) + test_screen2 S29 (= TG5, S1 matrix with the panel open at 6 graphs, 48 layouts: the panel adds
no problem; 6 layouts of the filled 1366x658 window with the bars docked at the top do not fit even without it - not the
panel's). CPU (tools/trend_load.py, a full 60-minute ring, 6 graphs open, 2 windows x 3 text sizes x every zoom):
```
sampler: 3.0 us per frame (limit 50)
1920x1080 small     2 min: draw 5.1 ms (data 2.0)  10 min: draw 5.6 ms (data 3.0)  30 min: draw 6.6 ms (data 4.1)  60 min: draw 7.9 ms (data 5.5)
1920x1080 normal    2 min: draw 5.0 ms (data 2.0)  10 min: draw 5.5 ms (data 3.2)  30 min: draw 6.2 ms (data 4.5)  60 min: draw 7.3 ms (data 5.5)
1920x1080 large     2 min: draw 4.3 ms (data 2.0)  10 min: draw 5.0 ms (data 3.0)  30 min: draw 6.1 ms (data 4.5)  60 min: draw 7.4 ms (data 5.8)
1366x768 small      2 min: draw 4.1 ms (data 2.0)  10 min: draw 4.6 ms (data 3.3)  30 min: draw 5.7 ms (data 4.4)  60 min: draw 7.1 ms (data 5.8)
1366x768 normal     2 min: draw 4.1 ms (data 2.0)  10 min: draw 4.6 ms (data 3.0)  30 min: draw 5.7 ms (data 4.5)  60 min: draw 7.0 ms (data 5.9)
1366x768 large      2 min: draw 4.4 ms (data 2.1)  10 min: draw 4.7 ms (data 3.2)  30 min: draw 5.4 ms (data 4.2)  60 min: draw 6.4 ms (data 5.9)
worst redraw 7.9 ms (limit 10), worst data part 5.9 ms
TREND LOAD: OK
```
Limits (design 9): sampler <= 50 us per frame, panel redraw <= 10 ms (the cold-cache worst case = a redraw with new data,
at most 2 Hz); the unit test uses 3x looser bounds. The panel adds no work to screen 1. Assistant index: rebuilt because
tour.json gained the stop s2.trends (voice_pending); the extract-floor gate is unchanged (7 / 86 and 4 / 33 frozen).
Side fix found by the gate: driver.answer() called a callback without an argument when the ADS-B / mob / mark questions
passed a function with none (TypeError in the tour-picture demo run) - now tolerant.

## TR4 - acceleration graphs, screen-1 strip, debrief / academy replay, manuals, 2026-11-10
Station / tools / tests / docs only (NO app / hal / gateway-logic change). New: graph G9 acceleration (three axes, each its own
colour + pattern, average + peak), trendgraph.py + trendreplay.py (pure), the optional strip at screen 1's left edge (box
"trends", off by default, hidden in MINIMAL, HUD layout version 4), the same graphs in logview.py (a pane) and in the academy
scenario replay + video-lesson player, MANUAL.txt ch. 26 / MANUAL_EL.txt ch. 21, tour stops s2.trends (text) + s1.trends
(voice_pending), screenshots docs/screens/trends/ (made by tools/demo_screens.py --tour from the running demo). Tests:
test_trends TJ1-TJ5 (26 / 26 pass), test_driver I26 (25 / 25 pass), TG6 / TG7 now cover trendgraph.py + trendreplay.py.
CPU (tools/trend_load.py, quiet PC, a full 60-minute ring, 6 default graphs, 2 windows x 3 text sizes x every zoom; every cell
the lower median of 2 passes - OS noise bursts showed up as single 9-11 ms cells next to 5-6 ms neighbours in the
single-pass runs; the redraw = a refresh with new data, the panel's own cache cleared):
```
sampler: 6.5 us per frame (limit 50)
1920x1080 small     2 min: draw 5.3 ms (data 3.5)  10 min: draw 5.9 ms (data 5.1)  30 min: draw 5.8 ms (data 6.4)  60 min: draw 5.3 ms (data 7.6)
1920x1080 normal    2 min: draw 5.3 ms (data 3.8)  10 min: draw 6.4 ms (data 6.0)  30 min: draw 5.8 ms (data 7.2)  60 min: draw 5.6 ms (data 8.5)
1920x1080 large     2 min: draw 5.3 ms (data 3.8)  10 min: draw 5.4 ms (data 5.6)  30 min: draw 5.4 ms (data 7.2)  60 min: draw 5.4 ms (data 8.9)
1366x768 small      2 min: draw 5.0 ms (data 3.9)  10 min: draw 5.3 ms (data 5.9)  30 min: draw 5.8 ms (data 7.6)  60 min: draw 5.5 ms (data 8.9)
1366x768 normal     2 min: draw 4.9 ms (data 3.7)  10 min: draw 4.7 ms (data 5.5)  30 min: draw 6.0 ms (data 7.0)  60 min: draw 4.9 ms (data 8.8)
1366x768 large      2 min: draw 4.3 ms (data 3.7)  10 min: draw 4.5 ms (data 5.6)  30 min: draw 4.6 ms (data 6.8)  60 min: draw 5.8 ms (data 8.8)
detector: observe 6.1 us per frame (limit 50), judging all values 0.531 ms avg / 1.093 p99 / 3.678 max per 2 Hz sample (limit 1)
screen-1 strip: 5.32 ms per 2 Hz refresh (lines + ops, limit 10), 0.015 ms per frame while the strip is the same (ops reused, 2010 ops; limit 2)
worst redraw 6.4 ms (limit 10), worst data part 8.9 ms; the FIRST draw after opening / a zoom change (every cache empty) worst 19.0 ms
TREND LOAD: OK
```
What the numbers needed: the TR3 run printed OVER A LIMIT (10-15 ms) in noisy runs; run quiet it was 9.2-11.4 ms at the worst
cell (the extra acceleration values and the detector had made it slower than TR2's 7.9). Optimised, no limit relaxed:
(1) the decimation keeps COMPLETE pixel columns on the absolute time grid in a cache (trends.Trends._colc: a refresh only
works on the newest column; `clean` = one NaN test per sample set instead of one per column), (2) the panel draws as of the
start of the current column step (<= 1 column = 2 px behind the clock; the newest value is always in the label; the click
maps through the same axis: TG3 updated to check exactly that and the bound), (3) screen 1's strip ops are reused between
frames (0.014 ms per frame, 5 ms per half-second refresh; limits 2 ms / 10 ms). The FIRST draw after opening the panel or
changing the zoom (every cache empty) is 7-9 ms per zoom; the very first draw of a process is ~13-20 ms (Tk / Python warm-up):
one frame. Screen 1's frame time with the strip: +0.014 ms (the median-frame limit is +2 ms).
Assistant index: rebuilt (the manuals + tour.json changed); extract floor gate: the new chapter moved the off-manual tuning
question el016 to 7.28 >= the frozen 7.25, so the floor was re-tuned 7.25 -> 7.3 by the documented procedure
(answer.EXTRACT_FLOOR + client.DEFAULT_EXTRACT_FLOOR + the AS test constant + `floor --write`; frozen separation el 7 below /
86 above, en 4 / 33 as before). Retrieval numbers (assistant_index.py eval): el all top-1 83 / 135 = 61 %, top-3 77 %,
held-out top-1 64 % / top-3 77 %; en all top-1 53 %, top-3 77 %, held-out 78 % / 100 % (9 questions); AS17 still FAILS in
the gate (a known open failure; its thresholds were NOT touched).
Gate (tools/check.sh, 2026-11-10, one run): only the known items failed - AS17 (open), AC31 (s11_mob_search; passes alone) and
gateway F9 (KEEP 5': 0.2 min in the ring, timing under load; passes alone); everything else passed.

## CL2 - flaky tests under load (2026-11-10)
Harness-timing fixes only (docs/cleanup_report.md CL2): test_driver bounded event pump + condition waits (I25, I10, I19, I23), gateway VR5 waits for the worker's asynchronous sample writes. Failure rates (tools/loadgen.py, 20 CPUs): test_driver 0 busy 1/3 -> 0/3, 4 busy 5/5 -> 0/5, 20 busy: the 900 s stall at I16 gone, only I23 (a real 60 ms / 150 ms render-time threshold) still fails 2/2; gateway VR 4 busy 0/5 (VR5 was 1/3 once), 20 busy = vision pauses by design (CPU guard), not a flake. No assertion or threshold changed.

CL2-c (2026-11-11): remaining known flakes - AC31 s11_mob_search (load; passes alone), gateway A2 (SD holes under load; passes on the re-run alone, W50), VR5 / LH2 / LH20 / F1 fixed in the harness. Known / env: AS17 (W84 recordings), SR8 (NVENC). check.sh needs > 45 min in one run.

## X2 - real ArduPilot SITL behind the real router (2026-11-11)

How to run: `python tools/get_sitl.py` (pinned download into tools/bin/sitl/, sha256-checked), then
`python tools/run_sitl.py` (or `--bg`; log in test_results/sitl/, 45 min limit). Without SITL every test is N/A.
tools/check.sh runs it only when `tools/get_sitl.py --check` passes. Details + measured answers:
docs/uav_design.md section 14.

Run on the owner's PC (Windows 11, native Cygwin SITL, no WSL), ArduCopter V4.7.3-beta1 (the rolling Windows
'CopterStable' build; the version-pinned 4.7.2 Linux build is pinned but was not run here): 3 suites, ~9 min,
**11 passed, 5 failed, 0 N/A**.

| Test | Result | What it showed |
|---|---|---|
| SI1 registration (+ the parameter file) | PASS | good copter registered; FS_GCS_ENABLE 1 + GUID_TIMEOUT 10 refused, the notice names both |
| SI2 GPS warm start | PASS | GPS_RTCM_DATA (MGA-INI time + position) delivered; the effect on a real u-blox = bench only |
| SI3 route + launch sequence | **FAIL** | the mission upload never completes: ArduCopter answers MISSION_COUNT with MISSION_REQUEST (id 40), which the router's codec / allow.py FROM_DRONE do not have |
| SI4 mission flown | **FAIL** | same cause (no mission on the drone) |
| SI5 in-flight mission change | **FAIL** | same cause |
| SI6 link loss = hold | PASS | 20 s radio loss: same mode, 30.0 m kept, no RTL / LAND |
| SI7 manual flying + give back | PASS | 8.0 m/s forward, 0.1 m/s 6 s after centring, altitude kept; give back = BRAKE -> LOITER |
| SI8 GPS lost in the air | PASS | EKF failsafe -> ALT_HOLD (~28 s), altitude held with RC ignored, no landing |
| SI9 allow-list vs ArduPilot | PASS | 17 servo / relay / gripper / winch / mount / parachute / RTL / LAND / force / termination commands never left the router; drone unchanged |
| SI10 battery minimum | PASS | manager NAV_LAND refused; the router's rule landed + disarmed it; no ArduPilot landing before |
| SI11 SITL .tlog via pymavlink | PASS | ~2600 packets, in order, no denied command / PARAM_SET |
| SI12 never a parameter write | PASS | 85 SAFETY parameters unchanged after the flights |
| SI13 no-GPS climb fallback | **FAIL** | ALT_HOLD + ARM + TAKEOFF param3=1 accepted, NO climb, the DISARM then refused (MAV_RESULT 4) - stays armed on deck |
| SI14a six copters: registration | PASS | 5 registered, FENCE_ACTION 1 refused naming it |
| SI14b six copters: launch in series | **FAIL** | the mission upload (as SI3) |
| SI14c six copters: hold | PASS | 5 flying, every radio cut 15 s, all held |

Findings for the owner: W138 (no-GPS fallback does not climb and cannot be disarmed), W139 (MISSION_REQUEST:
allow.py change needed = SERIAL safety review; blocks every real launch), W140 (the SITL / drone ArduPilot
version). Parameter-file findings (built into docs/uav_params.param): telemetry stream rates needed (the router
never asks for streams), SERVO17-32 default -1 refused, an enabled fence blocks the no-GPS ARM.
The fast tests: test_uav UA41 NEW (the file vs the registry's rules, get_sitl pins / refusal, toolpaths, the
N/A path of the runner).
Note: tools/check.sh's Python (MSYS2 ucrt64) has no pymavlink, so in the gate the SITL suite reports N/A even with
SITL installed; the run above used the Python that has pymavlink (`python tools/run_sitl.py`). Fast tests after
the change: test_uav 48/48, test_uavui 16/16, test_lang 6/6, strays selftest OK, assistant index rebuilt (floor
gate unchanged 7 / 86 + 4 / 33).

## UAV-r1 - MISSION_REQUEST, no launch without GPS, the force DISARM in the slot (2026-11-11)

Owner W139 + W138 (gateway/uav + tests + docs only; NO app / hal / AURIX change; docs/uav_design.md 15). Serial
safety review in 2 rounds (opus, tools/api_review.py: review_5f038f0.txt 1 HIGH / 3 MED, review_3451b68.txt 2 MED -
all fixed, the notes in docs 15).
SITL (ArduCopter V4.7.3-beta1, Windows, `python tools/run_sitl.py`, final code): **18 passed, 0 failed, 0 N/A**
(log test_results/sitl/sitl_20261111-180313.log). SI3 / SI4 / SI5 / SI14b (the mission upload) PASS; SI13 = no GPS
= no launch (stopped "no GPS fix in 10 s with the hatch open", never armed, slot not launchable, PREPARE refused
125); SI15a = a copter stuck armed on deck (ALT_HOLD take-off accepted, no climb): it reports landed state TAKEOFF,
the normal DISARM is refused by the allow-list ("not on the ground"), the launch's FORCED DISARM in the slot ->
disarmed; SI15b = in flight the force DISARM refused by the router (over ELRS, and over a still-heard USB link at
20 m). One SI5 failure on an intermediate run (MISSION_ACK 13 INVALID_SEQUENCE over the radio bridge) passed when
re-run alone; the router now treats that ack as not fatal (ArduPilot goes on asking; the read-back still checks).
SITL found further real blockers beyond W139, fixed here: ArduCopter 4.7 DENIES MISSION_START with param1 / param2 !=
0 (MAV_RESULT 2) -> MISSION_SET_CURRENT then MISSION_START [0, 0] (+ MIS_RESTART 0 SAFETY if-present); a
DO_CHANGE_SPEED item is read back with frame 0 (mission.same accepts that for DO items only); ArduCopter streams no
EXTENDED_SYS_STATE (the router asks for it at 2 Hz on the slot cable). Two X2 test-harness bugs fixed (SI3 / SI14b
compared a bool with "DONE"; SI14c now waits until the copters loiter at their last point).
Fast tests: gateway test_uav 49/49 (UA3 / UA16 / UA17 / UA32 / UA34 updated, UA42 NEW with mutations), storm
`--part uav` 60 episodes I80-I84 PASS (I84 with its own in-slot record, I81 with its own GPS record; 22 force
DISARMs + 31 ARMs left the router; the in_slot-always-True mutation = I84 fails, the gps_ok-always-True mutation =
16 I81 fails), groundstation test_uavui 16/16, test_lang 6/6; academy 53/56: AC22 / AC54 (the tour does not
cover the X1 ".tlog" export button, uav:tlog) and AC55 (the tour screenshots stale) FAIL - pre-existing since X1, not
touched here. Assistant index rebuilt (the manuals changed), check / eval / floor gate pass (7 / 86 + 4 / 33).

## AM7 - licence-clean language model + the scaled extract floor (2026-11-11)

Station / tools / tests / docs only (no app / hal / gateway logic change; docs/assistant_design.md 5n, docs/licences.md).
Measured on the owner's laptop (RTX A1000 6 GB, nothing else on the GPU), the real worker with Whisper loaded first, answer
mode extract, the fewshot prompt, the post-check, the scaled floor (7.40 el / 7.33 en). Tuning part (el 96 + 7, en 34 + 4):

| model (licence) | el correct / precision | en correct / precision | el median ms | worker GPU MB |
|---|---|---|---|---|
| llama3.2-3b (Llama 3.2 Community, RESTRICTED - the old default) | 44 % / 50 % | 53 % / 55 % | 571 | 3651 |
| **qwen3-4b (Apache-2.0) - CHOSEN** | **47 % / 64 %** | **59 % / 65 %** | 1315 | 4225 |
| ministral3-3b (Apache-2.0) | 46 % / 59 % | 62 % / 68 % | 628 | 3729 |
| phi4-mini (MIT) | 8 % / 29 % (writes text, no pick) | 53 % / 58 % | 739 | 4273 |
| phi3.5-mini (MIT) | 29 % / 40 % | 50 % / 55 % | 1645 | 4721 |
| qwen2.5-1.5b (Apache-2.0) | 7 % / 39 % | 47 % / 52 % | 699 | 2435 |
| eurollm-1.7b (Apache-2.0) | no output (0 tokens; retrieval fallback) | - | - | 2511 |
| meltemi-7b (Apache-2.0, 24 of 32 layers) | 19 % / 38 % | 44 % / 50 % | 2200 | 4691 |
| qwen2.5-7b (Apache-2.0, 21 of 28 layers) | 38 % / 49 % | 59 % / 65 % | 6450 | 4623 |

Can't find right 7/7 + 4/4 for every model. FRESH set ONCE (qwen3-4b): el correct 30/68 = 44 %, answered 74 %, precision 60 %,
can't find right 7/8, 1455 ms median; en correct 5/25 = 20 %, answered 40 %, precision 50 %, can't find right 6/6. WORSE THAN
BEFORE in English (AM6 llama3.2-3b with the old default prompt: 52 %): 7 English + 6 Greek answers were a lone "2" = the fewshot
worked example echoed = refused as no pick (safe: «can't find» + the passage) - not tuned after seeing the fresh set (W142). e2e
(10 spoken Greek questions): release -> answer text median 2.5 s, max 3.3 s.

The floor: BM25 scores creep with the passage count (Greek median +3.1 % for N 540 -> 615 over 8 manual versions, el016 +2.8 %).
MIN_SCORE 3.0 and the extract floor 7.2 are now in the units of the corpus they were tuned on (N 540 / 810) and multiplied by
idf(df 5, N) / idf(df 5, N_ref) >= 1: today 7.40 / 7.33 (higher than the hand-raised 7.3) and MIN_SCORE 3.085 / 3.054. Backtest:
el016 refused at all 8 versions, 0 in-manual tuning questions lost; the main list's retrieval and the fresh set's retrieval
unchanged except AS17's Greek can't-find 7 -> 8 of 10 (el140 refused again) - AS17 PASSES (it measures typed retrieval; it never
needed the W84 recordings). Gate `assistant_index.py floor`: el 7 off-manual under / 86 in-manual over, en 4 / 33 = PASS.
Tests run alone: AS14, AS16-AS25, AS29 PASS (AS24 = the real qwen3-4b with the network blocked), test_lang 6/6.

## CL4 rest-2 - tools/check.sh split in stages (2026-11-12)

`sh tools/check.sh` = every stage in sequence + a summary (a missing / crashed / timed-out stage = GATE: FAILED); `--list`, `--stage <name>` = one stage alone under a 45-minute watchdog (TRYGONS_STAGE_LIMIT_S 2700). Measured on the owner's laptop (one stage at a time, nothing else running):

| stage | contents | seconds | result |
|---|---|---|---|
| docs | progress archive, python syntax, assistant index / eval / floor | 39 | OK |
| firmware | sim all 392 + bridge 391 (1 N/A) + can 384 + can-fast 384 (8 N/A each), TriCore compile, writer checks, stackcheck (selftest PASS incl. ST28), memory, FAT32 2, NV 4, DroneCAN 19, api_review | 141 | OK |
| station | chart 6, lang 6, boats 12, screen 2 29, look 26, track 12, screenrec 10, yesno 7, benchstack 7, hailer 8, trends 26, sound 17, uavui 16, sar 32, adsbui 9 | 747 | OK |
| academy | assistant 36 (1 N/A), tour pictures, academy 56/56, practical 8/8 | 910 | OK |
| gateway-misc | avoid 15, osdep 12, setup 53, loudhailer 22, uav 48 (1 N/A: pymavlink), SITL N/A (no pymavlink), adsb 14, sar demo OK | 553 | OK |
| gateway | test_gateway.py | 1517 | 163 of 164 in the stage; G6 failed once in-stage (release code 103 instead of 7, timing), section G alone PASS (W50 rule); earlier stage run: section D crashed in 3 s once, passes alone |

Sum about 57 min for the full gate; no stage over 25 min. Flakes seen in the FIRST (partly stale-tree) stage runs and passing alone / on the repeat (W50 rule, not weakened): academy AC31 s11_mob_search (failed 2x inside the stage, passes alone 2x and in the full-gate run), gateway-misc UA18a (event loop 594 ms, passes alone), gateway section D (crashed in 3 s once, passes alone). Real fixes found by the split runs: lang keys bench.stack_core0-2 (dynamic use) restored, test_screen2 S11 allows the product name "Mission Planner" (X1 button), tests/assistant/floor.json re-frozen (passage count only; floor unchanged 7.2), tour stop uav.tlog (voice_pending) + pictures, stackcheck POLLED_ONLY also from core1_main / core2_main (MED-A, selftest ST28 mutation).

## HX3 - harbour sensors behind an interface, simulated radar + LiDAR (2026-11-12)

gateway/test_harbour_sensors.py HS1-HS10 (stage gateway-misc): 10 passed. Measured against the world truth: radar
range 299.4 m for 299.8 m (bias < 0.5 m), bearing 314.98 deg for 315.0 (relative to the bow), speed 2.03 for 2.0 m/s;
range sd 5.41 for the modelled 5.50 m, bearing sd 1.02 for 1.0 deg, sweep drop 2.5 % for 2 %; nothing inside the 25 m
blind zone or beyond 3000 m; LiDAR kayak at (19.99, 5.01) for (20, 5) m; a 0.3 m swimmer never seen at 10 m (under
the -7 deg FOV), Pd 0.61 at 25 m; the filter keeps 0 of the hull / water / spray returns; degraded = noise x2.8,
detections 481 -> 226; clutter 1.1 -> 24.6 plots per sweep; the worker process: OK in 0.6 s, LOST 1.0 s after a
"lost" fault, skew DEGRADED 0.3 s, a crash restarted in 1.7 s, a hang in 4.2 s (test limits: restart_s 1, silent_s 3),
below-normal priority (0x4000). Load (tools/gateway_load.py --harbour, 60 s): radar worker 0.3 %, LiDAR worker
0.6 %, the gateway-side manager 1.4 % of one core; latency p99 312 ms radar / 54 ms LiDAR (300 / 50 ms of it modelled).
HX3-r (review fixes): HS1-HS10 pass again; swimmer at 25 m 244 raw / 244 after the worker's filter (400 scans), kayak 287 / 287; a worker with a missing Python = LOST "cannot start" in 0.1 s. Round 2: a 0.12 m paddleboard kept in 287 / 287 scans (272 marked surface); a pose 1 s old = DEGRADED. UAV tests alone after the stage: 49 passed (UA22 = the known load flake). Stage runs: docs OK (36 s); gateway-misc 553 s: harbour 10/10, UA22 failed once in the stage while the station stage ran in parallel (see the re-run); station: SR8 NVENC = the known environmental failure (every other station suite passed, 817 s); gateway stage OK 164 / 164 (1409 s).

## HX4 - harbour perception: the fusion tracker + the static map (2026-11-13)

gateway/test_harbour_tracker.py HT1-HT16 (stage gateway-misc): 16 passed, the HX3 simulators in-process, seeded.
Single vessel: 1.1 m position error, velocity (-0.03, 1.90) for (0, 2) m/s; default harbour (40 s): ferry AIS-linked
(MMSI) 3.2 m, fishing boat 0.1 m, the 3 moored boats static (0.8 / 2.0 / 0.8 m), 7 quay tracks static in the chart
layer; crossing vessels keep their ids; head-on vy -1.93 (truth -2), overtaken vy 1.01 (1); static map: moored boats
accumulated (3440 LiDAR cells), the chart layer, save / load, decayed after 6 h; kayak = small, swimmer = person,
buoy = buoy, a flat 0.12 m board = person + surface, steer_ok in 50 / 41 / 50 / 50 of the last 50 ticks; sea clutter
never confirmed; a radar ghost kept + ghost_suspect; LiDAR rain: the swimmer stays steer_ok, the clutter flag (30 new
tracks / s) - 32 clutter tracks confirmed (obstacles, conservative); camera-only and AIS-only tracks never steer_ok
(+ the conflicts camera_without_range / ais_without_range); LiDAR lost = U2, a radar skew = U11, both silent =
range_lost U1 + the track coasting (not steer_ok), deleted after delete_s; an AIS position 40 m off = U6; timing p99
0.61 ms (in-process), determinism, read only (sockets trapped: 0), mutations (camera / AIS as a range source, no
fresh-hit rule) all caught. Load (tools/gateway_load.py --harbour, 60 s): tracker in the manager's tick p99 9.9 ms
(budget 30), 47 tracks, the manager 4.3 % of one core.
HX4-r (review of a154416): HT1-HT17 pass (17 / 17); HT17 = an AIS fix jumping onto a moored boat keeps both range
tracks in place, a batch without heading = input rejected + no pose, a batch 1 h off = refused (time_jump 1) and the
manager's monotonic mapping, a static map of another origin refused, a camera swimmer 12 m in front of a moored boat
at 22 m = a camera-only track + camera_without_range, flags before the first cycle = range_lost + "tracker not
started", two rafted boats 3 m apart = 2 tracks.
Round 2 (review of 858168e): 17 / 17; HT17 adds the AIS report kept at its position (U6) after a jump, a coasting AIS
ferry not handed to a moored boat 20 m away, a 5 s clock step refused + U11 "clock". UA22 (UAV battery landing) failed
in both gateway-misc runs AND alone AND on the pre-HX4 commit 693fcfd (worktree) - environmental, not this step.


## HX7 - harbour scenario library END TO END on the real simulator (2026-11-13)
tests/harbour/run_scenarios.py: every episode = the real autopilot_sim (UNCREWED, speed 1) + the real gateway modules
(simulated radar + LiDAR, tracker, planner, route check) + the captain's station (HELM, route upload with the harbour flag,
START, the shore stick's CONFIRM); one process per episode in a kill-on-close job; 4 episodes in parallel (6 in parallel
stalled the machine: a 300 s episode limit killed the whole batch twice - use --jobs 4).
Library `all` (H1-H20 + H3n, 21 episodes, 624 s wall): 18 PASS, 3 XFAIL. H1 empty exit, H2 arrival at a quay, H3 wide
channel, H3n narrow channel (the planner holds ~30 m before +-17 m moorings, 11-14 m hull gap), H5 / H6 crossing from
starboard / port, H7 overtaking a slow boat, H9 kayak, H10 swimmer, H11 buoy, H12 / H13 one sensor lost (<= 2.5 kn),
H14 both lost / H16 GPS spoofed / H20 planner crash = HOLD in time, H15 link lost = STANDBY then stopped (the kill after
the grace is NOT waited for: the run is 131 s), H17 camera-only / H18 AIS-only ghost = slows, never a turn.
XFAIL (documented FINDING HX7-2, shown on every run, a fix turns them into a "remove the xfail mark" FAIL): H4 head-on,
H8 fast overtaker from astern, H19 a vessel that does not give way - the planner HOLDs (SOG 0) 25-30 m before / in front
of the vessel (U7 / U5) and a vessel that ignores the boat runs through the standing boat (hull overlap -4.2 / -6.3 /
-6.0 m). The boat never moved into them; it stops in their path (no sidestep to the corridor edge, no reverse).
FINDING HX7-1: before any START the AURIX's entry_ok is False (needs the planner lease; the gateway opens the planner's
pre-start window only when a START passes it), the station's START button is enabled only on entry_ok -> the real button
can never be pressed first (the rig sends a first START: NACK 6, then True).
Storm `storm --episodes 30 --seed 1` (seeded random worlds, 75 s each, the common criteria; 668 s wall): first run 24 PASS,
6 FAIL = 4 starts refused by the planner's STOP at the start (the boat never moved: now the explicit correct outcome "E
START refused...") + 2 unfinished routes at the time limit (C6 now accepts RUN / SLOW at the limit); the 6 re-run: see PROGRESS.
Not run: the demo rehearsal (desktop), the full check.sh gate, an independent review.

HX7 review round 1 (sonnet, tools/api_review.py, e5cdaed): HIGH "a boat that never leaves passes" CONFIRMED and fixed: the
first full runs were partly VACUOUS (H3 / H5 / H7 / H11 ... held at x = 2 m from a "planner cycle late" U4 HOLD while 4
episodes shared the PC and still "passed"). Now P1 (RUN / SLOW seen + >= 25 m), P2 (every fault fired), P3 (the object came
within 40 m), arrival at the route's end, C2 against EVERY object with its own limit, hull circle = len / 2, the link-lost
check reads ll_state + throttle (the old hard-coded True is gone; today's grace is 0: kill >= 3 s after the loss), a HOLD
from a host-load transient in the first 25 m is answered by the captain's RESUME once the planner is clear (counted in the
episode's events). LOAD SENSITIVITY (finding for the owner): with 3-4 episodes in parallel the gateway's planner / tracker
cycle goes late (U4, "tracker overload: cycle 312 ms > 30 ms") and the boat correctly STOPs - run the stage on a quiet PC,
library 2 jobs, storm 3. After the last fix only H1 / H3 / H5 / H11 were re-run (4 PASS, jobs 2, 246 s); the FULL library
and storm were NOT re-run with the preconditions (the last full run, jobs 3, before the RESUME fix: 13 PASS, 4 XFAIL
(H4 / H8 / H19 + ...), the rest = load HOLDs). The numbers above (18 PASS, 624 s) came from the pre-review criteria.
