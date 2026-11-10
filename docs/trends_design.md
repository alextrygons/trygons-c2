# Trend panel «ΤΑΣΕΙΣ» - design (step TR1, 2026-11-10)

Owner spec: docs/owner_findings_2026-11-09.md section L (L1-L4, binding). CLAUDE.md section Y = the short
rules. Steps: TR2 data + graphs, TR3 silent highlight, TR4 screen-1 strip + debrief / academy + docs.
Defaults marked TO CONFIRM are asked in PROGRESS.md W136. Implementers build FROM this file; a step may
rename within its own commit and updates this file + CLAUDE.md Y.

## 0. Ground rules

- STATION ONLY: groundstation/ (+ academy/ replay, tools/, docs). NO app / hal / gateway / AURIX change,
  no new frame, no new TELEM2 field (TELEM2 is FULL), no alarm bit.
- READ-ONLY: the trend modules never send anything (boat, gateway, other stations), never change a mode,
  setting outside their own `trends_*` keys, alarm, lamp or sound. They get decoded dicts IN, draw OUT.
- SILENT: a highlight is never an alarm: no sound.py call, no MasterLamps / set_local, not in the alarm
  list / panel, nothing to acknowledge. Real alarms are untouched and always win.
- Modules (targets): `groundstation/trends.py` (pure: value table, sampler, ring), `trenddetect.py` (pure:
  statistics, commanded-change filter, highlight state), `trendpanel.py` (screen-2 panel drawing + click),
  `trendlog.py` (the local log). hud.py draws the screen-1 strip from the same model. None of them can reach a
  sender: section 8.

## 1. Values (existing telemetry only)

Telemetry: TELEMETRY 0x80 (`proto.decode_telemetry`) + TELEM2 0x8B (`proto.decode_telem2`), 10 Hz
from the AURIX (link.c TELEM_PERIOD_MS 100), forwarded at full rate by the gateway (stations.py "tm" /
"t2"); gw_status 1 Hz. The station may receive fewer frames (busy link): the sampler never assumes 10 Hz.

| id | Name el / en (lang `trend.<id>`) | Source field (decoded) | Unit | Graph | Detect |
|----|------|------|------|------|------|
| sog | ΤΑΧΥΤΗΤΑ / SPEED | TELEM `est_sog` (DR / GPS; in DP the GPS value, proto) | kn | G1 | mean + spread |
| thr | ΓΚΑΖΙ / THROTTLE | TELEM `throttle` (= link.c `s->thr.out_pct`, signed %) | % | G1 | none (commanded) |
| hdg | ΠΟΡΕΙΑ / HEADING | TELEM `heading` (compass; `compass_ok` false = gap) | deg | G2 | none (TO CONFIRM) |
| rot | ΣΤΡΟΦΗ / RATE OF TURN | TELEM `yaw_rate` | deg/s | G2 | spread only |
| xte | ΑΠΟΚΛΙΣΗ / CROSS-TRACK | station-computed: `driver.waypoint_info` -> `hud.xte` (est position vs the leg start seen by this station and `wp_lat` / `wp_lon`); only in AUTO / DP TRIP with `wp_set`, else gap. The AURIX's own `autop.leg.xte_m` is in the black box only (logfields `xte`) | m (+ = right) | G3 | mean + spread |
| roll | ΚΛΙΣΗ / ROLL | TELEM2 group H `att_roll` (None when `att_age_ms` stale -> gap) | deg | G4 | mean + spread |
| pitch | ΔΙΑΓΩΓΗ / PITCH (running angle: trim result) | TELEM2 group H `att_pitch` | deg | G4 | mean + spread |
| slam_g | ΚΡΟΥΣΕΙΣ / IMPACT PEAK | TELEM2 group `slam_peak` (= the largest \|vertical acceleration\| since the last telemetry, link.c max-hold, BNO085) | g | G5 | mean |
| slam_pm | ΚΡΟΥΣΕΙΣ/ΛΕΠΤΟ / IMPACTS PER MIN | TELEM2 `slam_rate` (impacts in the last 60 s, slam.h `rate_pm`) | 1/min | G5 | mean |
| depth | ΒΑΘΟΣ / DEPTH (chart) | `charts.depth(est_lat, est_lon)` 1 Hz as driver.py `depth_cache` (offline chart, NO depth sounder; no charts / no position = gap) | m | G6 | none (TO CONFIRM) |
| bank_v | ΜΠΑΤΑΡΙΑ / BATTERY | TELEM2 group L `bank_v` (30 s filter; None = no voltage data = gap) | V | G7 | mean |
| link_ms | ΚΑΘΥΣΤΕΡΗΣΗ / ROUND TRIP | gw_status `stations[you].rtt_ms` (this station's round trip, as driver.py `latency_ms`) | ms | G8 | mean + spread |
| link_loss | ΑΠΩΛΕΙΕΣ / LOST | station-computed: telemetry frames missed over the last 10 s from the `t_ms` steps (expected 100 ms; a boat restart = rebase) in % ; the active channel's gw_status `channels[].loss_pct` shown in the click readout | % | G8 | mean (up only) |

- "throttle vs speed" (L1): G1 draws sog + thr together; the SOG mean-down highlight is judged only while
  the throttle is steady (filter F2) = "same throttle, less speed".
- slamming: vertical peaks + impacts / min exist (above). NOT engine vibration (owner).
- FIRMWARE ITEM FW-TR (listed, NOT done, NOT scheduled): longitudinal / lateral acceleration (and a raw
  vertical value at a usable rate) are NOT in the telemetry (`accel_z` exists only in BENCH_DIAG 0x8D,
  USB, bench mode). Carrying them needs a NEW frame (TELEM2 is full) in app/link.c = FIRMWARE: its own step
  with its own SERIAL safety review (LOOP RULE 6). Until then G5 shows only slam_g / slam_pm. Owner: W136.

## 2. Sampler + ring buffer

- One sample every 0.5 s (2 Hz) on the station clock, taken in `App.on_telem`'s path (cheap: field copies
  only). Value = the newest received value in that 0.5 s, except slam_g = the MAX of the frames in it.
  No frame for a value in the slot, telemetry age > 3 s, or the field None / non-finite = NaN = GAP.
- Ring per value: `array('f')` of N = 7200 (60 min = the largest zoom) + one shared `array('d')` time ring.
  13 values x 7200 x 4 B + 7200 x 8 B ~= 432 KB per boat; cleared on a boat change / disconnect
  (`clear_telemetry`) and on a boat restart (t_ms back). RAM only, never on disk (the debrief uses the logs).
- Lines are never drawn across a NaN (a gap > 1 s = a break); "no data yet" = the label «—» only, never a
  flat or interpolated line.

## 3. Look (L2)

- Background: the panel's own dark fill `TREND_BG = "#05080a"` (alpha 0.85) in BOTH themes ("matrix" look);
  thin lines 1.25 px with a 3 px glow (same colour, alpha 0.25); grid `#14202a`; text `#c8d4dc`.
- Palette `theme.TREND` (per value) + pattern `theme.TREND_DASH` (Tk / Pillow dash tuples):

| id | colour | pattern | id | colour | pattern |
|----|--------|---------|----|--------|---------|
| sog | #00e0c0 | solid | slam_g | #f4f4f4 | solid |
| thr | #d0b0ff | dash (6,3) | slam_pm | #a0a0a0 | dot (2,3) |
| hdg | #ff80c0 | solid | depth | #4060ff | solid |
| rot | #e8e8a0 | dot (2,3) | bank_v | #80ffff | solid |
| xte | #8090ff | solid | link_ms | #ffc0e0 | solid |
| roll | #00a0ff | dash (6,3) | link_loss | #90c060 | dashdot (6,3,2,3) |
| pitch | #a0ff40 | solid | | | |

  Rules (test TG4): every pair of trend colours >= 55 apart (RGB Euclid), every trend colour >= 70 from
  every meaning colour (WARN, CAUTION, GOOD, ADVISORY, MAGENTA, YELLOW, HUD_YELLOW, REC, FROZEN, WARN_DARK,
  NIGHT.*, MODE_COLOURS.*); no hue in 340..58 deg with saturation > 0.25 (= no red / amber / yellow-amber);
  inside one graph the lines differ in PATTERN (colour-blind safe) and in HLS lightness >= 0.1 OR pattern.
  Measured with the table: closest pair 57 (slam_g / link_ms), closest meaning 72 (roll / ADVISORY).
- Highlight colour = `theme.WARN` (only here and only for a highlight: CLAUDE.md colour rule, section Y).
- NIGHT (settings "night"): every line = `theme.NIGHT["main"]` (first line of a graph) or `NIGHT["dim"]`
  (second line), same patterns; a highlighted line = `NIGHT["warn"]`, width 2.5 px, label outlined.
  Identity = pattern + the label. Test: in night every drawn line colour is in {main, dim, warn}.
- Each line's right end: value + unit + name («12.4 kn ΤΑΧΥΤΗΤΑ»), at the 3 text sizes (theme.px info
  level x text_scale). Time axis: last 10 min by default, zoom 2 / 10 / 30 / 60 min (buttons in the header;
  setting `trends_zoom`). Drawing decimates to <= 1 point per pixel column, keeping each column's MIN and
  MAX (a slam spike is never lost).
- Click a point (screen 2 only) = an outlined readout: the time (HH:MM:SS, UTC + local as the track) + every
  value at that sample (NaN = «—») + the mode; Esc / a second click closes it.

## 4. Layout

Screen 2 (13n1 rules, overlay.Overlay): panel "trends" (header «ΤΑΣΕΙΣ»), draggable, collapsible
(s2_collapsed), position saved (s2_panel_pos), the 3 text sizes; COLLAPSED by default (the S1 matrix stays
as today until a user opens it). Open: width max(260 px, 0.22 W), graphs G1..G8 each shown / hidden by the
user (setting `trends_show`, default G1, G2, G4, G5, G7, G8 = 6), at most 6 at once; graph height
max(3 info lines, 48 px). Placed by the fit test (a corner tuple like "relays"), never over the boat
keep-out circle, the mode bar, STANDBY, the alarm panel or the two bars; if 6 graphs do not fit, the panel
shows fewer (down to 2, the user's order), then folds to its header. Test TG5 = test_screen2 S29: the S1
matrix (every size x scaling x language) with the panel OPEN at 6 graphs, docked bars top and bottom, the
camera view: nothing clipped, no overlap, keep-out respected.

Screen 1 (TR4): hud box "trends" = (0.095, 0.235, 0.150, 0.395): the left edge strip between the throttle
bar and the picture's free centre (x1 = 0.245 < 0.25), under mirror_l, above "relays". OFF by default
(setting `hud_trends`), hidden in MINIMAL, at most 3 graphs (setting `hud_trends_show`, default G1, G4, G5),
no click, no zoom (follows `trends_zoom`). HUD_LAYOUT_VERSION 4 adds only "trends". It draws its own dark
panel (theme ALPHA hud_panel) = the one exception from "on the video all information GREEN" (owner L2:
own colour per value). Test TJ1 (test_driver I26, I19 style): no overlap with any DEFAULT_LAYOUT box,
LOGO_BOX, the free centre 0.25..0.75 x 0.22..0.75 or the bottom bar; inside the left 0.25.

## 5. Detection (TR3) - own recent normal vs the last 30 s

Per detected value, on every 2 Hz sample:
- BASE = the samples in [t - 630 s, t - 30 s] since the last rebase; needs >= 240 valid (2 min) else no
  judgement. RECENT = the last 30 s (60 samples), needs >= 45 valid.
- Robust statistics: median m and spread s = 1.4826 x MAD; s_eff = max(s_BASE, s_min).
- MEAN change: |m_R - m_B| >= k_m x s_eff (k_m = 4) AND >= abs_min. Direction ↑ / ↓.
- SPREAD (rhythm / oscillation) change: s_R / s_eff >= 2.5 (↑) or, only when s_B >= 2 s_min,
  s_R / s_B <= 0.4 (↓).
- ON after the condition held continuously 10 s; OFF when the statistic stays below 0.6 x its threshold for
  20 s (hysteresis); shown at least 30 s; COOLDOWN 5 min per value + kind + direction after it ends. One
  highlight per value at a time (mean wins over spread).
- BASE statistics recomputed every 5 s (sorted copy, <= 1200 samples), RECENT every sample.

| id | s_min | abs_min | mean | spread | judged in modes |
|----|-------|---------|------|--------|-----------------|
| sog | 0.2 kn | 1.0 kn | ↑↓ | ↑↓ | AUTO, HDG LOCK |
| rot | 0.5 deg/s | - | - | ↑↓ | AUTO, HDG LOCK |
| xte | 2 m | 10 m | ↑↓ | ↑ | AUTO, DP TRIP |
| roll | 0.5 deg | 3 deg | ↑↓ | ↑↓ | AUTO, HDG LOCK |
| pitch | 0.3 deg | 1.5 deg | ↑↓ | ↑↓ | AUTO, HDG LOCK |
| slam_g | 0.1 g | 0.3 g | ↑↓ | - | every mode but STANDBY / DP / BENCH |
| slam_pm | 1 /min | 3 /min | ↑↓ | - | every mode but STANDBY / DP / BENCH |
| bank_v | 0.05 V | 0.3 V | ↑↓ | - | every mode |
| link_ms | 20 ms | 150 ms | ↑↓ | ↑ | every mode |
| link_loss | 1 % | 5 % | ↑ | - | every mode |

Not judged at all (display only, TO CONFIRM W136): thr (commanded), hdg (commanded / steered; rot carries
the yawing), depth (position-driven; SHALLOW WATER has its own alarm). In MANUAL / REMOTE / MANEUVER /
TRAINING / BENCH / FIRST TEST the driver commands sog, rot, xte, roll, pitch every second: not judged.

## 6. Commanded-change filter

A trigger REBASES the listed values: their BASE restarts after the settle time (so the old level is never
compared with the commanded new one; with the 2-min BASE minimum nothing is judged for >= settle + 2 min)
and any highlight of them ends silently (logged "off", why "rebase").

| # | Trigger (from the decoded telemetry / gw_status) | Values rebased | Settle |
|---|------|------|------|
| F1 | mode change (`mode`), DP sub-state change (`dp_sub`), STANDBY / `stopping` | sog rot xte roll pitch slam_g slam_pm | 20 s |
| F2 | throttle step: \|thr - mean(thr, previous 10 s)\| > 5 %, or \|d thr / dt\| > 2 %/s (also `throttle_demand`) | sog pitch slam_g slam_pm | 20 s after it is steady |
| F3 | AUTO waypoint change (`auto_wp`), target heading change \|d `target`\| > 5 deg (HEADING_ADJ, HDG LOCK set), an avoidance / nav heading (`nav_kind` heading), AUTO RETURN start (`prog_returning`) | rot xte roll sog | 30 s |
| F4 | speed cap / set change: `speed_set_kn`, `wp_kn`, power limit (`power_fwd`), navreq cap (`nav_cap10`), slamming cap (`slam_cap`), `speed_limited` | sog pitch slam_g slam_pm | 20 s |
| F5 | gear change (`gear`) | sog rot roll pitch slam_g slam_pm | 20 s |
| F6 | shore stick override start / end (TELEM2 D `ov_hand`) | rot xte roll | 30 s |
| F7 | generator start / stop (`gen_active`), a relay change of the engine group | bank_v | 60 s |
| F8 | link: active channel change, data saver on / off, HELM change, reconnect, station telemetry mode change | link_ms link_loss | 30 s |
| F9 | a gap > 5 s in a value | that value | 0 s |
| F10 | boat restart (t_ms back), boat change, disconnect | all (rings cleared) | - |

## 7. Highlight + log

- Highlight: the line + its right-end label turn `theme.WARN` (NIGHT: `NIGHT["warn"]`, 2.5 px) and an
  OUTLINED label «ΤΑΣΗ: <ΟΝΟΜΑ> ↑» / «↓» (mean) or «ΤΑΣΗ: <ΟΝΟΜΑ> ∿↑» / «∿↓» (spread) at the top of its
  graph (outline 1 px WARN, transparent fill, never a filled box); lang `trend.hl_mean_up` ... A collapsed
  panel shows nothing (its header gets NO colour - silent). Screen-1 strip the same inside its box.
- Log (THIS PC only, never sent): `client`-style folder `groundstation/trends/log/YYYY-MM-DD.jsonl`
  (gitignored; a TRYGONS_TEST run = a temp folder; 5 MB per file, 30 days, as assistant qlog) + the station
  event line `App.event` «ΤΑΣΗ: ΤΑΧΥΤΗΤΑ ↓ 22.1 → 18.3 kn». One JSON line per on / off:
  `{"ts": "2026-11-09T12:34:56.5Z", "boat": "rf49d", "ev": "on", "val": "sog", "kind": "mean", "dir": "down",
  "base_m": 22.1, "base_s": 0.4, "rec_m": 18.3, "rec_s": 0.5, "stat": 9.5, "thr": 4.0, "mode": "AUTO",
  "dur_s": null, "why": null}`; "off" adds dur_s and why ("calm" / "rebase" / "gap" / "disconnect").

## 8. Read-only proof (like academy AC1-AC3 / assistant AS1-AS3)

- TG6 static: no trend module imports socket, asyncio, gwclient, link, stations, aurix, proto (the app
  passes decoded dicts), sound, alerts' writers, or calls `app.send*` / `link.send*` (AST scan).
- TG7 transitive import graph of the trend modules: no sender module reachable.
- TG8 runtime: a running station (TRYGONS_TEST) with every sender (App.send*, gwclient write, socket
  connect / send) patched to FAIL + counted; 10 min of synthetic telemetry incl. highlights, clicks, zoom,
  night, screen-1 strip: 0 sender calls by trend code (AS3b method: counts against the same window
  without the panel + taint of callbacks scheduled by trend code), MasterLamps / sound unchanged.
- TG9 narrow view: the panel holds no App reference (gc walk like AS6): it gets `TrendView` (settings
  get / put of `trends_*` only, the canvas, the theme).

## 9. CPU load

`tools/trend_load.py` (TR2; TR4 adds the strip): a synthetic 10 Hz feed with the ring full (60 min) and 6
graphs open at 1920x1080 and 1366x768, every text size; measures sampler us per frame, detection ms per
tick, panel redraw ms, screen-1 frame time with / without the strip (test_driver I23's measure). Limits:
sampler <= 50 us average, detection <= 1 ms per tick for all values, redraw <= 10 ms, redraw at most 2 Hz
and only while open + visible, screen-1 median frame time + <= 2 ms. Numbers in TEST_REPORT.md; the unit
tests use 3x looser bounds (loaded PCs).

## 10. Tests per step

TR2 (groundstation/test_trends.py, tools/check.sh): TG1 ring (size, wrap, NaN gaps, clear on restart /
boat change), TG2 sampler (every source field + unit from real decoded frames: `proto.decode_telemetry` /
`decode_telem2` of encoded frames; slam_g max-hold; stale > 3 s = gap; None att / bank_v = gap; xte only in
AUTO / DP TRIP), TG3 drawing (no segment across a gap, min / max decimation keeps a spike, labels at the
right end, zoom 2 / 10 / 30 / 60, click readout values + time), TG4 palette rules + night colours + patterns
per graph, TG5 = test_screen2 S29 layout matrix, TG6-TG9 read-only proof, TG10 CPU bounds.
TR3 (test_trends.py): TH1 a step in sog / roll / pitch / slam / bank_v / link -> ON after 10 s, not
before; TH2 a spread change (oscillation added / removed) -> ∿↑ / ∿↓; TH3 every trigger F1-F9 -> NO
highlight for a commanded change of the size of TH1's step (one case each, incl. a full simulated AUTO
program with waypoint turns + a throttle staircase = 0 highlights); TH4 hysteresis (a statistic hovering at
the threshold = at most one on / off pair), minimum 30 s shown, cooldown 5 min; TH5 too little BASE /
RECENT = nothing; TH6 gaps (F9) and missing data; TH7 silent (no sound call, MasterLamps rows / lamps /
ack unchanged, alarm panel unchanged, collapsed header uncoloured); TH8 log format, folder, size / age
limits, never sent; TH9 a real WARNING active at the same time: alarm panel + sound as without the panel;
TH10 the modes table (not judged in MANUAL / REMOTE / MANEUVER / DP ...); TH11 sim replay: a recorded
`autopilot_sim server` AUTO run = 0 highlights, the same with the SIM_FAULT slam = a slam_g / slam_pm ↑.
TR4: TJ1 screen-1 strip (test_driver I26 overlap rules, off by default, MINIMAL hidden, layout v4
migration, night, frame time); TJ2 debrief: logview.py "Trends" from a .tlg (black-box fields est_sog,
thr_out, yaw_rate, xte, slam_peak, slam_rate, bank_v at 20 Hz -> the same rings at 2 Hz; roll / pitch /
depth / link not in the log = gap); TJ3 academy replays (scenario replay .jsonl, video-lesson hud.jsonl)
through the same model, files only (simguard rules, AC1-AC3 still pass); TJ4 MANUAL.txt + MANUAL_EL.txt
chapter, lang keys el + en (test_lang: no English left in the Greek UI), tour stop s2.trends
(voice_pending), screenshots docs/screens/trends/ (tools/demo_screens.py).
