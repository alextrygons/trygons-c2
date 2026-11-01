# Ground station screenshots (review)

Made from the demo by `python tools/demo_screens.py` on 2026-10-30.

| File | What it shows |
|---|---|
| [driver_day_el.png](driver_day_el.png) | driver HUD, day, Greek: AUTO mission, AR overlays (horizon, route ribbon, waypoint, predicted path, velocity vector), AIS threat DEMO FERRY |
| [driver_day_en.png](driver_day_en.png) | driver HUD, day, English: AUTO mission, AR overlays (horizon, route ribbon, waypoint, predicted path, velocity vector), AIS threat DEMO FERRY |
| [driver_alarm_el.png](driver_alarm_el.png) | driver HUD with active alarms (GPS LOST, OBSTACLE AHEAD): MASTER CAUTION lit, alarm bar |
| [driver_alarm_en.png](driver_alarm_en.png) | the same in English |
| [driver_night_el.png](driver_night_el.png) | driver HUD, NIGHT mode (dim red), el |
| [driver_night_en.png](driver_night_en.png) | driver HUD, NIGHT mode (dim red), en |
| [driver_minimal_el.png](driver_minimal_el.png) | driver HUD, declutter MINIMAL (speed, heading, mode, gear, alarms) |
| [driver_edit_layout_el.png](driver_edit_layout_el.png) | driver HUD, edit-layout mode (drag / resize / hide every box) |
| [driver_hidpi_en.png](driver_hidpi_en.png) | step 13l: screen 1 in a 3840 x 2400 window (high-DPI laptop) = the picture composed at 1920 x 1200 (render_max_px) that is shown x2; before the fix only a 200 x 150 corner was drawn and the windows froze. Made by groundstation/test_driver.py I23 (TRYGONS_SHOT_DIR) |
| [screen2_acked_el.png](screen2_acked_el.png) | step 13m: screen 2 after ACK ALARM - the alarm rows steady (this PC silenced), the boat's ack sent. Made by groundstation/test_screen2.py S16 (TRYGONS_SHOT_DIR) |
| [screen2_disconnected_el.png](screen2_disconnected_el.png) | step 13m: screen 2 after a disconnect - no old alarm rows, no NO DATA, "not connected" (S16) |
| [driver_disconnected_el.png](driver_disconnected_el.png) | step 13m: screen 1 with no boat - "not connected" in the mode box instead of the red NO DATA alarm (S16) |
| [screen2_linktime_el.png](screen2_linktime_el.png) | step 13q1: screen 2 with LINK TIME 30 s - the amber banner "ΧΡΟΝΟΣ ΧΩΡΙΣ ΣΥΝΔΕΣΗ 30 s (ΠΑΡΑΚΑΜΨΗ)" and, in the "Link" panel, the value with - / + (HELM captain only). Made by groundstation/test_screen2.py S17 (TRYGONS_SHOT_DIR) |
| [driver_linktime_el.png](driver_linktime_el.png) | step 13q1: screen 1 with LINK TIME 30 s - the same banner in the notices (S17) |
| [screen2_dark_el.png](screen2_dark_el.png) | screen 2 (navigation & control), dark theme, el: the map with the boat in the centre, the four corner panels, STANDBY, the logo |
| [screen2_light_en.png](screen2_light_en.png) | screen 2 (navigation & control), light theme, en: the map with the boat in the centre, the four corner panels, STANDBY, the logo |
| [screen2_dark_en.png](screen2_dark_en.png) | screen 2 (navigation & control), dark theme, en: the map with the boat in the centre, the four corner panels, STANDBY, the logo |
| [tools_connection_el.png](tools_connection_el.png) | tools window, connection page, Greek |
| [tools_sound_el.png](tools_sound_el.png) | tools window, sound page, Greek |
| [about_el.png](about_el.png) | Help -> About, el: logo, version, commit, licences / credits |
| [about_en.png](about_en.png) | Help -> About, en: logo, version, commit, licences / credits |
| [splash_el.png](splash_el.png) | the start-up screen of START_DEMO (Greek), while the gateway starts |
