# Demo 2026-11-11 (tag demo-2026-11-11) - notes

1. Start: double-click DEMO_SAR.bat (the normal demo + the presenter's guide window); stop: STOP_DEMO.bat. MANUAL.txt ch. 25 / MANUAL_EL.txt ch. 20 list the 8 presenter steps. Plain START_DEMO.bat is unchanged (the helicopter then crosses by itself).
2. The guide only listens; every YES / NO is answered by the presenter (mouse or shore stick trigger). Its one action: "Send the helicopter" (a SIMULATED aircraft).
3. Load demo\demo_mission.json on screen 2 for the AUTO step; the launch needs the demo boat's relay table (relay 8 = hatch) pushed and taken (onboard CONFIRM) - try the whole run once before the presentation.
4. Needs: msys Python with tkinter + Pillow, MediaMTX + ffmpeg in tools/bin (python tools/get_mediamtx.py), the built simulator (the demo builds it).
5. Known failure: AS17 (assistant, Greek can't-find 7/10: el140 scores 3.003 vs the 3.0 search threshold since the longer manual chapters) - NOT tuned, owner decision.
6. Known environment failure: SR8 (NVENC encoder probe on this PC). Known flakes that pass alone: VR2, AC31, LH2, AS28, one test_driver test.
7. Not done in PF6: no tour stop for MOB / search / mark / ADS-B; no screenshots in docs/screens/sar/ (they need the running demo with the stations; tools/demo_screens.py was not run).
8. Owner decisions pending (PROGRESS.md WAITING FOR OWNER): W130-W134 (SAR stand-off 30 m, MOB drift estimate, ADS-B limits + the Windows readsb build, UAV camera mounts, keys M / K), W84 / W89 / W97 / W99 (talking manual), W80 (ADS clean build), W110 / W111 (MAVLink allow-list).
9. Simulated only: the person, the helicopter, the UAVs and their video; estimated positions are marked "not for navigation".
10. Rollback: git checkout demo-2026-11-11 gives exactly this version.
