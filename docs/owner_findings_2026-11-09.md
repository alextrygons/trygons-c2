## L. Trend panel ("ΤΑΣΕΙΣ") - owner 2026-11-09 - steps TR1-TR4 - STATION ONLY (groundstation/ + docs + academy replay); NO app / hal / gateway logic change; read-only (never sends anything to the boat)
Purpose: the driver / copilot sees at a glance how the trip is going: scrolling time graphs of key values, and a silent highlight when a value changes its rhythm or average unexpectedly. Looks like a "matrix" instrument (thin glowing lines on dark).

L1 Values (from the EXISTING telemetry only; a value the telemetry does not carry yet (e.g. raw acceleration at a usable rate) = listed in TR1 as a separate FIRMWARE item with its own serial safety review, not done here):
- speed (SOG) + throttle vs speed (same throttle, less speed = hint);
- heading + rate of turn; cross-track error in AUTO;
- roll + pitch (IMU) - pitch = the boat's running angle, so the result of the outboard TRIM is visible (pitch together with speed and throttle);
- ACCELEROMETER values (owner 2026-11-09): vertical acceleration peaks (g) = slamming, plus slamming impacts per minute; longitudinal / lateral acceleration where the telemetry has them. NOT engine vibration (owner: not wanted);
- depth; battery voltage; link quality (round-trip time, lost packets).

L2 Look (owner decisions 2026-11-09)
- Each value has ITS OWN COLOUR; the colours also differ in brightness / line pattern (dash, dot) so they stay distinguishable for colour-blind crew and in NIGHT mode (dim red: then pattern + label only). Not red, not the CAUTION amber for normal lines.
- Dark background, thin glowing lines, value + unit + name at each line's right end, time axis (default last 10 min, zoom 2 / 10 / 30 / 60 min).
- Screen 2: a slim side strip (13n1 rules: draggable, collapsible, 3 text sizes, layout saved per PC); 4-6 small graphs stacked, each value can be shown / hidden. Screen 1: optional tiny strip at the EDGE only (never the centre). Captain AND copilot.
- Click a point = time + all values there; the same graphs in the debrief (logview / track) and in the academy replays.

L3 Silent highlight ("ΤΑΣΗ") - owner: red, with the name, silent
- Each value compared with its OWN recent normal (e.g. last 10 min) vs the last ~30 s: a clear change of average or rhythm (spread / oscillation) = highlight.
- EXPECTED changes are not highlights: compared against what was COMMANDED / the mode (a heading change at an AUTO waypoint, speed after a throttle change, DP manoeuvres, STANDBY).
- Minimum duration before highlighting, a cooldown per value, hysteresis (no flicker).
- Highlight = the line + its label turn RED and «ΤΑΣΗ: <ΟΝΟΜΑ> ↑/↓» appears in the panel as an OUTLINED label (not a filled alarm box). SILENT: no sound, no master lamp, not in the alarm list, nothing to acknowledge; logged in the station log. The CLAUDE.md colour rule is updated: "red = alarms, STANDBY, SAVE, and the trend highlight INSIDE the trend panel only (outlined, labelled ΤΑΣΗ)" (owner 2026-11-09).
- Real alarms stay as they are and always win.

L4 Tests + docs
- Tests: highlight on a step / oscillation change, NO highlight on commanded changes, cooldown / hysteresis, night mode, missing data (gap, never a fake line), never sends anything (read-only proof like the academy / assistant), CPU load of the panel measured (never slows screen 1).
- MANUAL.txt + MANUAL_EL.txt chapter, tour stop, screenshots docs/screens/trends/.

Steps (each its own commit + independent review in parallel, via the API review tool (API1) when it works; 2-fix-round cap; Sonnet for working agents except TR1 design = Opus):
TR1 design (CLAUDE.md section + docs/trends_design.md: values, colours, detection method + thresholds, tests).
TR2 data + graphs (ring buffer from telemetry, the panel, colours, zoom, click).
TR3 silent highlight (detection, commanded-change filter, red outlined ΤΑΣΗ labels, log).
TR4 screen-1 edge strip, debrief / academy replay, MANUAL / tour / screenshots.
