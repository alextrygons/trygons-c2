# Academy - the interface for a future 3D simulator (Godot)

ACADEMY_SPEC part H, step 22. **There is no 3D world yet.** This file describes what the academy already
does and what a future 3D simulator (planned: Godot) must do to plug in. The code is `academy/sim3d.py`
(`INTERFACE_VERSION = 1`) and the window `academy/sim3d_app.py`; the tests are academy/test_academy.py
AC44-AC50.

## 1. The rules (binding: CLAUDE.md section S, docs/ACADEMY_SPEC.md)

1. **The 3D simulator never replaces the simulated AURIX.** Every lesson, scenario and mission drives
   this repository's `sim/autopilot_sim` on **127.0.0.1:5760** (the academy's `simguard.SimLink`), with its
   own unchanged safety code (STANDBY, joystick priority, power limit, CONFIRM, gear only at idle ...).
   The 3D world is a **view** of that boat and a **mission scorer** - nothing else.
2. **Files only.** The academy and the 3D simulator talk through files in `academy/data/sim3d/`
   (gitignored). The academy opens no new socket and starts no new process (D2 / D3: its only processes
   stay the simulator and ffmpeg). The trainee or instructor starts the 3D program by hand.
3. **No command ever comes from the 3D simulator.** The academy reads only its `hello.json` (present +
   version) and its mission results. A result carrying a level, a role, an expiry or a command is refused.
   A mission result is logged; it **never changes a level** (levels go up only through the practical test).
4. **Every file the academy reads goes through `simguard`** (`check_recording` for JSON, `check_image` for
   photos): an ordinary file - never a COM port, a device, a URL or a folder - and nothing at all on a
   gateway PC (`gateway.json` present).
5. **Drone photos and the Daskalio chart photos are pictures, never a chart.** They give the 3D view its
   picture / terrain. No safety check (shallow water, route, geofence, collision) ever reads them: those
   use only `charts/` (tools/chartprep.py). The terrain manifest carries `"not_for_navigation": true`;
   AC11 / AC48 prove no safety code mentions `docs/daskalio` or `sim3d`.
6. A future step that wants the 3D world to supply the **physics** (waves, wind, the hull) to the
   simulated AURIX is a **separate, reviewed step**: it would change `sim/` (the boat model), never `app/`,
   and would need its own tests and the owner's review.

## 2. The folder `academy/data/sim3d/`

| File | Written by | What |
|---|---|---|
| `settings.json` | academy | `{"backend": "builtin" \| "godot3d"}` - the lesson simulator chosen in the window |
| `hello.json` | 3D sim | `{"kind": "sim3d_hello", "version": 1, "t": <epoch s>, "name": "..."}`, rewritten about every second |
| `live.jsonl` | academy | the simulated boat at 5 Hz while a lesson / scenario runs with the 3D backend in use |
| `missions/<attempt>.json` | academy | a mission package (one per attempt) |
| `results/<attempt>.json` | 3D sim | the result of that attempt |
| `results/done/` | academy | results already read (moved there, never read twice) |
| `terrain/<area>/manifest.json` + `*.jpg` | academy | imported drone photos (no metadata, at most 2048 px) |

Write every file as `name.tmp` and rename it (the academy does the same), so the reader never sees half a file.

## 3. The backend choice and `hello.json`

* `builtin` (default, always available): autopilot_sim's own world, the academy's 2D pictures.
* `godot3d`: the same autopilot_sim **plus** the 3D view. Used only while `hello.json` is fresh: `kind` =
  `sim3d_hello`, `version` = `INTERFACE_VERSION` (1), and `t` within `HELLO_MAX_AGE_S` (5 s) of the
  academy PC's clock. Otherwise the academy uses `builtin` and says why ("no hello.json", "stale", "speaks
  interface 2").

## 4. `live.jsonl` (academy -> 3D view)

A new file for every lesson / scenario run. Line 1 = header, then one sample per line at 5 Hz:

```json
{"kind": "sim3d_live", "version": 1, "source": "lesson", "id": "l01_hdg_lock", "t0": 1761700000.123}
{"t": 0.0, "mode": "HDG_LOCK", "hdg": 91.2, "sog": 4.5, "cog": 90.8, "thr": 22.0, "lat": 37.8201235,
 "lon": 23.6, "alm": 0, "alm2": 0, "gear": 1, "outboard": -3.5, "rot": 0.4}
```

`t` = seconds since the run started; `mode` = the AURIX mode name (`groundstation/proto.py MODES`);
`hdg` / `cog` degrees; `sog` knots; `thr` throttle %; `lat` / `lon` the simulator's position estimate;
`alm` / `alm2` the two alarm words; `gear` (0 N, 1 F, 2 R as in the telemetry); `outboard` the outboard angle
(deg); `rot` the yaw rate (deg/s). A field is left out when the telemetry has not sent it yet. Read the file
like `tail -f`; a cut last line is normal while it is written.

## 5. Missions (`academy/content/missions/*.json`)

Commented content files (`_help`), kind `mission`, `runs_on` `"sim"`, both languages:

| Field | Meaning |
|---|---|
| `id` | plain name, unique |
| `level` | OBSERVER / DRIVER / CAPTAIN (the list is grouped by it) |
| `title_el` / `title_en`, `brief_el` / `brief_en` | what the trainee reads |
| `world` | the terrain area (`terrain/<world>/`), e.g. `daskalio_demo` |
| `goals` | list: `reach` / `keep_clear` (`n_m`, `e_m` metres from the START position, `radius_m`), `heading` (`deg`, `tol_deg`), `mode` (a mode name), `time_limit` (`s`) |
| `pass_score` | 1..100 |
| `max_attempts` | 0 = no limit; else the next attempt is refused |
| `lesson` | optional: the simulator lesson whose own checks run too |

`sim3d.check_mission` checks every file (AC45). Goals are relative to where the simulated boat is at the
start, so a mission does not depend on the simulator's origin.

### The mission package `missions/<attempt>.json` (academy -> 3D sim)

```json
{"kind": "sim3d_mission", "version": 1, "attempt": "ann_m01_leave_bay_20261029-101500_123",
 "user": "ann", "lang": "el", "issued": 1761725700.123, "mission": { ...the content file... },
 "aurix": {"host": "127.0.0.1", "port": 5760, "_note": "the simulated AURIX: the 3D sim only shows it"},
 "terrain": "terrain/daskalio_demo/manifest.json"}
```

Writing it counts an attempt (logbook kind `mission_start`).

### The result `results/<attempt>.json` (3D sim -> academy)

```json
{"kind": "sim3d_result", "version": 1, "attempt": "ann_m01_leave_bay_20261029-101500_123",
 "mission": "m01_leave_bay", "status": "passed", "score": 85, "duration_s": 212.4}
```

`status` = `passed` / `failed` / `aborted`; `score` 0..100. The academy ("Collect results") accepts it only
for an attempt it issued, for the same mission, once; `passed` in the logbook = status passed AND score >=
the mission's `pass_score`. Refused (and shown): unknown / forged attempt, another mission, a score out of
range, another version, an unknown status, any `level` / `licence` / `role` / `expiry` / `command(s)` field.
Logbook kind `mission` (score, status, passed, duration, topic = the mission id = the instructor view's
weak areas).

## 6. Drone photos -> terrain (`sim3d.import_photos`)

The window's "Import drone photos": JPEG / PNG / TIFF photos of the owner's DJI drone. Read per photo:
EXIF GPS (latitude, longitude, altitude) and the DJI XMP tags `RelativeAltitude`, `FlightYawDegree`,
`GimbalPitchDegree`, `GimbalYawDegree`. A photo without a GPS position is skipped (said). The copy in
`terrain/<area>/` is at most 2048 px and has **no metadata** (no EXIF, no XMP, no comment). The manifest:

```json
{"kind": "sim3d_terrain", "version": 1, "area": "daskalio_demo", "not_for_navigation": true,
 "_note": "drone photos for the 3D TRAINING view only - NOT a navigation chart ...",
 "photos": [{"file": "DJI_0001.jpg", "lat": 37.82, "lon": 23.6, "alt_m": 35.5, "rel_alt_m": 60.1,
             "yaw_deg": -12.5, "gimbal_pitch_deg": -90.0, "w": 2048, "h": 1365}], "updated": 1761725700.1}
```

A nadir photo (gimbal pitch -90) with its height and yaw is enough for the 3D world to drape it over a flat
sea / a rough terrain. Building a height map (photogrammetry) is the 3D project's own work.

## 7. What the Godot project must do (checklist for whoever builds it)

1. Rewrite `hello.json` every second while running.
2. Read `live.jsonl` and draw the boat (position, heading, outboard angle, throttle, mode, alarms).
3. Read a new `missions/<attempt>.json`, draw the world from its `terrain` manifest and the goals.
4. Score the goals from the live samples; at the end write `results/<attempt>.json` (atomic rename).
5. Never open the simulator's port, never write any other academy file, never show a photo as a chart.

The academy side is complete and tested (AC44-AC50); nothing in the academy needs to change for a first
3D viewer.
