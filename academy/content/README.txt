academy/content/ - the Academy's lessons, quizzes, scenarios, exams and limits
==========================================================================

Every file here is JSON with comments as "_help" / "_note..." keys (the same way as boats/*.json).
Edit them with any text editor; academy/test_academy.py (AC10) checks every file:
  * "kind" (lesson | quiz | scenario | exam | limits | test_area) and a "_help" comment;
  * every text a trainee sees in BOTH languages: "<name>_el" (Greek, the default) + "<name>_en";
  * lessons and scenarios: "runs_on": "sim" (the academy never drives a real boat);
  * quizzes: "answer" = the index (0, 1, 2 ...) of the right choice.

The files whose name starts with "example_" are examples of the format (step 15). The real
content comes with the steps 18 (lessons, quizzes), 19 (scenarios) and 21 (exams); the example
lesson / quiz / scenario files are gone (only the example exam remains until step 21).

Lessons (step 18, lessons/*.json): each step = do_el / do_en (what to do), hint_el / hint_en, and
"check" = the SIMULATOR state that proves it, e.g. {"f": "mode_name", "eq": "HDG_LOCK"} or
{"f": "target", "delta": 10, "tol": 0.5}; hint_s, fail_s (time limit), hold_s, at_start
("gps_fail" / "gps_ok"). The field names and every rule: academy/lesson.py. A new lesson also needs
its trainee script in test_academy.py (LESSON_SCRIPTS, AC28 runs it on the real simulator).
Quizzes (quizzes/*.json): type choice (default), picture (show s1 / s2 / panel + area) or sound
(sound warning / caution / advisory); every question has why_el / why_en (academy/quiz.py).

Scenarios (step 19, scenarios/*.json): setup + prep steps (like a lesson), then "event" (after_s;
faults = SIMULATOR faults compass / gps_spoof / slam / joy_break / gps_fail, hazard = ais / shallow /
swimmer drawn by the academy, show = an alarm the station would show, say = what the trainee sees,
wait_for, clear_after_s), "react" (any = the right reactions, good_s, max_s), "forbid", observe_s,
outcome.min_m, "after" (untimed follow-up steps), right_el / right_en + debrief_el / debrief_en.
Every rule: academy/scenario.py. A new scenario also needs its trainee script in test_academy.py
(SCENARIO_SCRIPTS, AC31 runs it on the real simulator).

Video lessons (step 20) are NOT here: they are made by the Lesson editor from the boat's saved
events and kept in academy/data/videolessons/<id>/ (lesson.json kind "videolesson" + the small blurred
copies + hud.jsonl) - never in git, because real trip video shows people and number plates. Every
rule: academy/videolesson.py (check_doc).

Chart photos (docs/daskalio/): for the test area picture, the demo map and the 3D sim ONLY.
A photo is NOT a navigation chart - never used by any safety check (docs/daskalio/README.txt).
