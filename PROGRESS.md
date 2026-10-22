# PROGRESS

Task: "Two parts: (1) three safety fixes, (2) AIS ships, depth charts and
collision avoidance" (fixes F1-F3, sections A-E, tests, final report, push).
If interrupted: say "continue from PROGRESS.md" - continue with the first
section that is not DONE below, without redoing finished work.

## Status: ALL SECTIONS DONE (pushed to GitHub)

| Section | What | State | Commit |
|---|---|---|---|
| Baseline | state before the work | DONE | 15c7f25 |
| F1 | driver confirmation enforced in the gateway, single-operator mode | DONE | ec19c59 |
| F2 | slamming RED = STEP (default) / LIMIT_RESET, no power-limit drop | DONE | ac29ce1 |
| F3 | black box off CPU1 (DSPR2 + LMU), slamming in the 20 Hz record, mapreport per core, CPU1 > 60 % fails | DONE | e309f03 |
| A | AIS receiver (receive only), stale / removal, RECEIVER SILENT, online feed (never STARLINK), second GPS option | DONE | dff86f5 |
| B | offline depth charts (EMODnet, GEBCO fill, OpenSeaMap marks), map layers, "Not for navigation" note | DONE | 4fe2dfa |
| C | shallow-water checks (programs, AUTO start, predicted position), GW_NAVREQ to the AURIX | DONE | dc28498 |
| D | collision avoidance CPA / TCPA: AUTO automatic, HDG LOCK proposals, MANUAL alarm, DRIVER DECISION, gateway lost | DONE | 8f51c18 |
| E | simulated AIS traffic, AIS list tab, driver HUD collision line, proposal dialog | DONE | 554cebd |
| Report | final report given, git push done | DONE | 554cebd |

## Tests (last full run: tools/check.sh -> GATE: OK)

- sim `autopilot_sim all` and `all --bridge`: exit 0
- TriCore compile OK; memory: core 0 51 %, CPU1 5 % (limit 60 %), core 2 81 %, LMU blocks ~84 %
- gateway 74 passed (incl. F1, AIS, SH, CA end to end), planner (test_avoid.py) 14, charts 6
- ground station: e2e 10, logs 3, driver 7

## Next

Nothing left of this task. Possible later work:
- check the black-box placement in the real ADS / TASKING build (map file + tools/mapreport.py)
- tune the collision thresholds at sea (end-to-end tests use smaller distances)

## Open problems / uncertain

- AIS does not show small boats, jet skis, swimmers or kayaks: lookout + cameras stay essential.
- At the 30 % power limit (~7 kn) a fast crossing ship can only be avoided by a "best available" route (DRIVER DECISION alarm too); random-traffic soak: closest ship 0.17 nm.
- Slow GPS drags (<= 0.2 m/s) are not caught within 10 min without the second GPS.
- dAISy HAT takes the Pi's serial port: the AURIX link then needs a USB-UART adapter.
- Charts are free data, NOT for navigation; near the coast "no data" = shallow (some legs refused).
- .cproject has a harmless line added by ADS (not committed).
