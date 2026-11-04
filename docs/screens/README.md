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
| [driver_maneuver_el.png](driver_maneuver_el.png) | step 13q2: screen 1 in MANEUVER - the drive box shows ΤΗΛΕΧΕΙΡΙΣΜΟΣ and ΜΑΝΟΥΒΡΑ (the running one lit, the other one a straight switch), no RELEASE any more (owner 2026-10-31). Made by groundstation/test_screen2.py S4 (TRYGONS_SHOT_DIR) |
| [tools_remote_el.png](tools_remote_el.png) | step 13q2: Tools -> the shore-joystick page without the RELEASE button, the new help (UNCREWED one press with the stick centred + lever neutral; CREWED onboard CONFIRM; to stop choose another mode); step 13p: + the YES / NO answers on the stick (trigger, thumb, hat; the stick Y on / off at this station) (S4) |
| [driver_question_no_el.png](driver_question_no_el.png) | step 13p (owner 2026-10-31 D): a question on screen 1 in the LOWER part, above the bottom bar - the ONE vertical Y/N layout: the question, [ΝΑΙ] on top, [ΟΧΙ] underneath (full width, framed), here ΟΧΙ selected (= inverted: the stick pulled back / the hat down), the countdown and the stick's keys below. Made by groundstation/test_screen2.py S20 (TRYGONS_SHOT_DIR); driver_slam_ask_el.png shows the same box with ΝΑΙ pre-selected |
| [screen2_own_confirm_el.png](screen2_own_confirm_el.png) | step 13p: the single operator's own confirmation in the same vertical layout (ΝΑΙ – ΑΠΟΣΤΟΛΗ on top, ΟΧΙ – Άκυρο underneath); the shore stick's trigger answers it too (S20) |
| [screen2_geofence_el.png](screen2_geofence_el.png) | step 13q2c: screen 2, UNCREWED, after a geofence breach in HDG LOCK - the new WARNING row "ΕΚΤΟΣ ΓΕΩΦΡΑΧΤΗ: STANDBY" in the alarms panel; the HDG LOCK button of the control panel sends the HELM captain's mode command when UNCREWED (owner 2026-11-01). Made by groundstation/test_screen2.py S18 (TRYGONS_SHOT_DIR) |
| [driver_geofence_el.png](driver_geofence_el.png) | step 13q2c: screen 1 with the same GEOFENCE alarm in the alarms box (S18) |
| [screen2_dark_el.png](screen2_dark_el.png) | screen 2 (navigation & control), dark theme, el: the map with the boat in the centre, the four corner panels, STANDBY, the logo |
| [screen2_light_en.png](screen2_light_en.png) | screen 2 (navigation & control), light theme, en: the map with the boat in the centre, the four corner panels, STANDBY, the logo |
| [screen2_dark_en.png](screen2_dark_en.png) | screen 2 (navigation & control), dark theme, en: the map with the boat in the centre, the four corner panels, STANDBY, the logo |
| [tools_connection_el.png](tools_connection_el.png) | tools window, connection page, Greek |
| [tools_sound_el.png](tools_sound_el.png) | tools window, sound page, Greek |
| [about_el.png](about_el.png) | Help -> About, el: logo, version, commit, licences / credits |
| [about_en.png](about_en.png) | Help -> About, en: logo, version, commit, licences / credits |
| [splash_el.png](splash_el.png) | the start-up screen of START_DEMO (Greek), while the gateway starts |
| [screen2_modebar_el.png](screen2_modebar_el.png) | step 13n1 (owner 2026-10-31 E1): screen 2 with the MODE BAR at the top centre (STANDBY · ΚΛΕΙΔΩΜΑ ΠΟΡΕΙΑΣ · AUTO · ΤΗΛΕΧΕΙΡΙΣΜΟΣ · ΜΑΝΟΥΒΡΑ · DP · ΧΕΙΡΟΚΙΝΗΤΟ · BATTLE SHORT; the active mode white / inverted, not allowed = grey), square framed buttons, STANDBY the only red button, the white CONNECTED lamp, the two bottom bars. Made by groundstation/test_screen2.py S21 (TRYGONS_SHOT_DIR) |
| [screen2_bars_el.png](screen2_bars_el.png) | step 13n1: the two thin bottom bars (bar 1: heading −10 −1 value +1 +10, AUTO pause / resume / skip, ΔΙΑΔΡΟΜΗ / ΜΕΓ. ΤΑΧΥΤΗΤΑ / ΓΕΩΦΡΑΧΤΗΣ; bar 2: nearest ship + CPA, messages, zoom, ΣΚΑΦΟΣ, Εργαλεία, Aa, ΕΠΑΝΑΦΟΡΑ ΑΡΧΙΚΩΝ) with the ROUTE sub-panel open just above bar 1 (S22) |
| [driver_bottom_bar_el.png](driver_bottom_bar_el.png) | step 13n1: screen 1 with the ONE thin bottom bar in groups (ΕΛ/EN · ΝΥΧΤΑ · ΟΛΑ · ΓΕΜΙΣΜΑ · Aa | AR · ΕΜΠΟΔΙΑ AI | ΔΙΑΤΑΞΗ | ΕΓΓΡΑΦΗ · SAVE | ώρα + αποστολή) and the heading tape above it, every information green on the video (S21) |
| (13o: no new picture) | step 13o (owner 2026-10-31 F): the gear is «Σχέση» in the status panel of screen2_modebar_el.png (remade by test_screen2 with TRYGONS_SHOT_DIR). The new Tools - Sound row "Φωνή: Piper (AI, εκτός σύνδεσης) / Φωνή των Windows" + "Δείγματα φωνής" are Tk widgets: no picture (the desktop is not visible to the test PC) |
