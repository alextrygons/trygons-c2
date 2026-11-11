# Owner findings 2026-11-08

## J. UAV: real ArduPilot SITL tests + standard flight logs (owner 2026-11-08) - steps X1-X2 - gateway/uav + tests + tools + docs; NO app / hal change; camera drones only (scope of section V unchanged)

X1 Flight logs in a standard format + Mission Planner as the service tool
- The router (gateway/uav/router_worker.py) writes, per drone and per flight, the MAVLink stream it RECEIVES from that drone as a standard .tlog (8-byte big-endian microsecond timestamp + raw MAVLink 2 packet, the format Mission Planner / MAVExplorer / UAV Log Viewer read), next to the existing per-drone flight log; same event id / file naming as the flight log; rotated / size-capped like the other logs; never uploaded automatically.
- Also the commands the C2 SENT to that drone (after allow.py) in the same .tlog, so a debrief shows both sides; denied messages are NOT written as packets (only counted, as today).
- Export from the station: the existing flight-log export also offers ".tlog (Mission Planner)". A test opens a written .tlog with pymavlink (mavutil) and checks every packet + timestamp order; a SITL-made .tlog (X2) is checked the same way.
- MANUAL.txt + MANUAL_EL.txt + FIRST_TEST.md: Mission Planner (or QGroundControl) = the drones' WORKSHOP tool only: firmware, calibration, parameters, log analysis - on the bench over USB, NEVER connected to the gateway / router during an operation (a second GCS would bypass the allow-list and the one-pilot rule). How to open the .tlog (Mission Planner "Telemetry Logs", MAVExplorer offline).

X2 Real ArduPilot SITL in the tests (answers the "TO CONFIRM in SITL" items, e.g. W60 RC_OPTIONS, W68 ALT_HOLD with RC ignored)
- tools/get_sitl.py: pinned ArduCopter SITL (version = the drones' ArduPilot version - TO CONFIRM by the owner, default the latest stable 4.x) into tools/bin (never in git), sha256 pinned, found through gateway/toolpaths.py. Linux native; on Windows the prebuilt SITL binaries if available, else WSL - document what works on the owner's big PC.
- docs/uav_params.param = the REQUIRED ArduPilot parameter set for C2 drones (the owner rules in parameter form): no RTL / no LAND on GCS or RC loss (failsafes = hold / LOITER-style, never land), battery failsafe = LAND only at the absolute minimum (C2 rule "land only when the battery is finished"), RC_OPTIONS as needed, fence / altitude limits matching the C2 checks, sysid per slot. The registry check (H1 "Register") compares each drone's live parameters with this file and refuses a drone that differs on a SAFETY parameter (list which ones count) - a notice names the parameter.
- SITL test suite (tests/acceptance or gateway tests, own runner, background with a log): 1 and 6 SITL copters behind the REAL router + allow.py: register (incl. the parameter check: a SITL copter with a wrong failsafe parameter is refused), mission upload from a C2 route, launch sequence with the simulated hatch, warm-start GPS path, link loss = hold (never RTL / land), battery-minimum landing, manual assisted flying (setpoints, let go = hold), «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ», in-flight mission change, the allow-list denies every non-allowed message (incl. any payload / release / servo / relay command) with a real ArduPilot on the other side.
- Each "TO CONFIRM in SITL" item in docs/uav_design.md / PROGRESS.md: answered with the measured result (or "still bench only" with why).
- simdrone.py stays for the fast tests; SITL tests = N/A (never PASS) when SITL is not installed; check.sh runs them only when present.
- Written to TEST_REPORT.md; MANUAL note "how to run the SITL tests".

Steps (each its own commit + independent review in parallel; PROGRESS.md LOOP RULES incl. the 2-fix-round cap; allow.py changes = SERIAL review per W110):
X1 flight logs (.tlog) + Mission Planner service-tool notes.
X2 ArduPilot SITL + required parameter file + registry parameter check + SITL test suite.
