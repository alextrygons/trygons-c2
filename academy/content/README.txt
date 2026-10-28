academy/content/ - the Academy's lessons, quizzes, scenarios, exams and limits
==========================================================================

Every file here is JSON with comments as "_help" / "_note..." keys (the same way as boats/*.json).
Edit them with any text editor; academy/test_academy.py (AC10) checks every file:
  * "kind" (lesson | quiz | scenario | exam | limits | test_area) and a "_help" comment;
  * every text a trainee sees in BOTH languages: "<name>_el" (Greek, the default) + "<name>_en";
  * lessons and scenarios: "runs_on": "sim" (the academy never drives a real boat);
  * quizzes: "answer" = the index (0, 1, 2 ...) of the right choice.

The files whose name starts with "example_" are examples of the format (step 15). The real
content comes with the steps 18 (lessons, quizzes), 19 (scenarios) and 21 (exams).

Chart photos (docs/daskalio/): for the test area picture, the demo map and the 3D sim ONLY.
A photo is NOT a navigation chart - never used by any safety check (docs/daskalio/README.txt).
