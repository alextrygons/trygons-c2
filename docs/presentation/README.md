# Trygons C2 - presentation pack

Made by `python tools/demo_screens.py --presentation` on 2026-11-06 from the running demo (simulated boat, gateway,
cameras, AIS traffic, the UAV magazine with simulated drones). Every picture is the REAL ground station
(screen 1 = the driver's camera screen with the HUD, screen 2 = navigation & control). Greek (`_el`) and
English (`_en`) versions. The MP4 clips are recordings of the station's own windows (Windows PrintWindow
pictures - never the rest of the desktop - piped into ffmpeg), H.264 `-preset veryfast` as REC SCREEN, at most
1280 wide, 10 fps.

| File | What it shows |
|---|---|
| [01_screen1_auto_el.png](01_screen1_auto_el.png) | Screen 1 in AUTO: the camera video with the green HUD, the route ribbon and the waypoint (AR), speed, heading tape, the bottom bar |
| [01_screen1_auto_en.png](01_screen1_auto_en.png) | Screen 1 in AUTO: the camera video with the green HUD, the route ribbon and the waypoint (AR), speed, heading tape, the bottom bar |
| [02_screen2_auto_el.png](02_screen2_auto_el.png) | Screen 2 in AUTO: the map with the route, the boat in the centre, the AIS traffic, UAV 1 in the air, the mode bar (AUTO lit) and the corner panels |
| [02_screen2_auto_en.png](02_screen2_auto_en.png) | Screen 2 in AUTO: the map with the route, the boat in the centre, the AIS traffic, UAV 1 in the air, the mode bar (AUTO lit) and the corner panels |
| [clip1_auto_screen1.mp4](clip1_auto_screen1.mp4) | AUTO on screen 1 (25 s) |
| [03_screen2_collision_el.png](03_screen2_collision_el.png) | Screen 2 in HDG LOCK: the DEMO FERRY on a collision course - COLLISION RISK, the traffic panel with CPA / TCPA |
| [03_screen2_collision_en.png](03_screen2_collision_en.png) | Screen 2 in HDG LOCK: the DEMO FERRY on a collision course - COLLISION RISK, the traffic panel with CPA / TCPA |
| [04_screen1_collision_el.png](04_screen1_collision_el.png) | Screen 1: the collision line to the ferry on the HUD and the alarm |
| [04_screen1_collision_en.png](04_screen1_collision_en.png) | Screen 1: the collision line to the ferry on the HUD and the alarm |
| [05_screen1_proposal_el.png](05_screen1_proposal_el.png) | HDG LOCK: the gateway's avoidance proposal on the captain's screen 1 - YES on top, NO underneath (the trigger answers with the selection) |
| [05_screen1_proposal_en.png](05_screen1_proposal_en.png) | HDG LOCK: the gateway's avoidance proposal on the captain's screen 1 - YES on top, NO underneath (the trigger answers with the selection) |
| [clip2_collision_proposal.mp4](clip2_collision_proposal.mp4) | The ferry and the avoidance proposal, the captain answers YES (25 s) |
| [clip3_joystick_override.mp4](clip3_joystick_override.mp4) | Joystick override in AUTO: the onboard stick moved -> MANUAL at once, the autopilot steps back (screen 2, 25 s) |
| [06_screen2_manual_el.png](06_screen2_manual_el.png) | Screen 2 after the joystick override: MANUAL lit on the mode bar |
| [06_screen2_manual_en.png](06_screen2_manual_en.png) | Screen 2 after the joystick override: MANUAL lit on the mode bar |
| [07_screen2_uav_video_el.png](07_screen2_uav_video_el.png) | Screen 2's camera view with UAV 1's video, its green values and the buttons of the cameras / UAVs |
| [07_screen2_uav_video_en.png](07_screen2_uav_video_en.png) | Screen 2's camera view with UAV 1's video, its green values and the buttons of the cameras / UAVs |
| [clip4_uav_video.mp4](clip4_uav_video.mp4) | UAV 1's video on screen 2 (20 s) |
| [08_tools_uav_magazine_el.png](08_tools_uav_magazine_el.png) | Tools -> UAV magazine: six slots, REGISTER, PREPARE LAUNCH, FLY UAV, flight logs |
| [08_tools_uav_magazine_en.png](08_tools_uav_magazine_en.png) | Tools -> UAV magazine: six slots, REGISTER, PREPARE LAUNCH, FLY UAV, flight logs |
| [09_screen2_loudhailer_el.png](09_screen2_loudhailer_el.png) | The loudhailer: TALK held on screen 2's bar 2 (press and hold) |
| [09_screen2_loudhailer_en.png](09_screen2_loudhailer_en.png) | The loudhailer: TALK held on screen 2's bar 2 (press and hold) |
| [clip5_loudhailer.mp4](clip5_loudhailer.mp4) | The loudhailer: TALK pressed and held, let go (20 s) |
| [10_mode_bar_el.png](10_mode_bar_el.png) | The mode bar: every mode on one line, the active one inverted, BATTLE SHORT at the end |
| [10_mode_bar_en.png](10_mode_bar_en.png) | The mode bar: every mode on one line, the active one inverted, BATTLE SHORT at the end |
| [11_battle_short_el.png](11_battle_short_el.png) | BATTLE SHORT (demo theatre only: every code is refused, nothing is sent anywhere) |
| [11_battle_short_en.png](11_battle_short_en.png) | BATTLE SHORT (demo theatre only: every code is refused, nothing is sent anywhere) |
| [clip6_battle_short.mp4](clip6_battle_short.mp4) | BATTLE SHORT: an authorisation code typed -> INVALID AUTHORISATION CODE (red flash + shake), every time |

Notes
- The front camera is the demo's sea video (MaSTr1325 pictures, CC BY 4.0 - demo/media/ATTRIBUTION.txt) when
  `tools/get_vision_samples.py` was run, else the test pictures.
- The joystick override is the CREWED rule: the onboard stick moved = MANUAL at once (the joystick always wins).
- BATTLE SHORT is demo theatre only: no code exists, nothing is sent to the boat or the gateway.
- AIS does not show small craft (kayaks, swimmers): the crew keeps watch.
- Regenerate: `python tools/demo_screens.py --presentation` (Windows; the demo stops and starts again).
