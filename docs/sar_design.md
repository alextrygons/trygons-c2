# Search and rescue tools - design (step PF1)

Owner spec: docs/owner_findings_2026-11-08b.md section K (K0-K5). CLAUDE.md section X is the short
source of truth; this file is what the implementing steps (PF3 MOB, PF2 search patterns, PF4 mark
from video, PF5 ADS-B, PF6 demo) build from. Defaults marked TO CONFIRM are PROGRESS.md W130-W134.

## 0. Ground rules (K0) - how they are kept

- NO app / hal / AURIX change, NO gateway/uav/allow.py change, no new AURIX frame, no TELEM2 field,
  no new alarm bit. Every SAR tool only CREATES a boat route, a UAV mission, a marker or a warning.
- A boat route = the EXISTING path: station `prog_upload` (PROG_* frames) -> gateway prog_check
  (shallow 114, geofence 120, <= 50 waypoints, max_leg_kn) -> the AURIX's program_validate -> AUTO
  START + the CONFIRM rules (the captain's YES; a copilot's = the captain's ACCEPT, as today).
- A UAV mission = the EXISTING `uav_route {target, wps}` (uavcheck 123 / 124, launch rules for a
  magazine drone, the in-flight change rules for a flying one) and `uav_launch_prepare` (the
  captain's YES). Nothing starts by itself.
- Estimated positions (marked from video, MOB drift) are drawn as ESTIMATES: a dashed circle +
  «ΕΚΤΙΜΗΣΗ – όχι για ναυσιπλοΐα» / "ESTIMATE - not for navigation".
- Look: 13n1 (framed buttons, 3 sizes, slim draggable panels, vertical Y/N = ynbox / yesno.Selector,
  answerable with the shore stick), red only for the MOB WARNING alarm. Greek default (lang el + en).

## 1. Shared markers (all of K2 / K3 / K4 finds)

- Gateway module `gateway/sar.py` (class SarStore): the ONE copy of every marker of this boat,
  state/sar_markers.json (written by a helper thread), every change in the audit log (kind "sar").
- Marker = {id, kind "mob" | "point" | "find", n (MOB 1, POINT 3 ...), name, lat, lon, t_utc,
  r_m (uncertainty radius, 0 = boat GPS), est (bool), person (bool: a person in the water ->
  stand-off rule), src {station, camera | "UAV-n", u, v}, still (file name or null), closed (null |
  {t_utc, by, why "found" | "deleted"}), drift {set_deg, kn, conf} | null, rev}.
- NEW station -> gateway JSON message `sar_mark {op, m}`: op "add" | "move" | "rename" | "person" |
  "close" | "delete"; any crew role (captain or copilot), never a VIEWER = the NEW result code 128
  "SAR: not allowed" (stations.GW_RESULTS + lang result.128).
  The gateway answers `sar_ack {ok, id, code}` and sends `sar_markers {rev, list}` to every station
  on every change + on login; gw_status "sar" {rev, n_open} lets a station re-ask (`sar_get {}`).
- NEW `sar_still {id, jpg_b64}` (<= 40 KB because one station line is 64 KB; the station's crop of the clicked spot) -> stored in
  <data>/sar/<id>.jpg, kept with the SAVE event of that moment (recorder event folder link).
- No gateway / link down: the station keeps a LOCAL marker (groundstation/sar.py LocalSar, RAM +
  the file groundstation/sar/local.jsonl, gitignored; TRYGONS_TEST = a temp folder) and sends it as "add" when the gateway
  comes back (same id = idempotent). The MOB alarm never waits for the gateway.
- Debrief: every marker change goes into the station log + the gateway audit + the open SAVE
  event's event.json "sar" list; tlg / logview untouched (AURIX log unchanged).

## 2. MOB (K2, step PF3)

Flow (station `groundstation/mob.py`, pure model + app.py wiring):
1. Press = the framed «MOB» button (screen 1 bottom bar beside the REC group; screen 2 bar 1) or the
   key `mob_key` (default M, gs_settings; never in a text field; never on the joystick). Captain AND
   copilot, single operator, no ACCEPT (it changes no driving).
2. AT ONCE, from the telemetry sample of the press time (app.t, its rx time; NOT a later one): the
   position = est_lat / est_lon (pos_source, unc_m -> r_m); no position = the marker at the last
   known position with r_m = unc + age x SOG and the text «ΘΕΣΗ ΑΒΕΒΑΙΗ».
3. LocalSar marker MOB n + `sar_mark add` (kind "mob", person true).
4. WARNING alarm on THIS station: alerts.MasterLamps.set_local("mob_<n>", text, WARNING) + voice
   callout «Άνθρωπος στη θάλασσα» / "Man overboard" (sound.py callout, alarm repeat rules; ACK
   silences, the marker stays). Every OTHER station raises the same local WARNING when the marker
   arrives in `sar_markers` (one per marker id).
5. SAVE: the existing `save {id}` message (gateway recorder.trigger kind "mob", pre_s / post_s as
   a normal SAVE) + the station's screen part (screenrec, same id) - as the SAVE button does.
6. The MOB line (screen 1 notices box top line + screen 2 banner / "sar" panel): MOB n, distance +
   bearing from the boat now, elapsed mm:ss; drift ESTIMATE only when TELEM current_conf is good
   (DR learned current set_deg / drift_kn; there is NO wind data today -> leeway 0, say «χωρίς
   άνεμο») else «χωρίς εκτίμηση παρασυρσης».
   Drift: P(t) = P0 + v_c t; r(t) = sqrt(r0^2 + (sigma_v t)^2), sigma_v = max(0.1 kn, 0.5 |v_c|)
   (TO CONFIRM W131). Drawn as a dashed line + circle, ESTIMATE text.
7. Offers (one vertical Y/N box at a time, the CAPTAIN answers - captain_confirm / the gateway's
   ask_captain for a copilot; single operator = own question):
   - «ΕΠΙΣΤΡΟΦΗ ΣΤΟ MOB» -> the stand-off route (section 3) through prog_upload, then AUTO START
     (its normal CONFIRM). Refused text when already inside the stand-off distance.
   - «UAV ΠΑΝΩ ΑΠΟ ΤΟ MOB» -> `uav_route {target "next" | slot, wps [{lat, lon, alt 30}]}` (+ the
     normal `uav_launch_prepare` for a magazine drone); a flying UAV = the in-flight change.
   - «ΜΟΤΙΒΟ ΕΡΕΥΝΑΣ» -> the PF2 panel with the MOB (or its drift point now) as datum.
8. Marker drag = `move`, «ΒΡΕΘΗΚΕ» = `close {why found}` (time kept), alarm cleared on every station.
- CREWED text "someone fell off the boat", UNCREWED "a person seen in the water" (same function).

## 3. Stand-off rule (K0; every route to a person)

- A boat route whose last point is a marker with person = true (MOB, a marked point set as "person
  in the water", a SAR find) ENDS at the approach point A, never at the marker M:
  A = M + D x u, D = sar_standoff_m (default 30 m, TO CONFIRM W130), u = the unit vector
  - from M toward the DOWN-current side (bearing = the current's set_deg) when current_conf is good
    (the boat then approaches heading up-current and stops down-current of the person), else
  - from M back toward the boat's present position (straight approach).
- Route = [boat now (start), A] with final_action PROG_END_STOP (= idle + neutral + STANDBY at the
  last waypoint, the AURIX's existing arrival stop; never PROG_END_HOLD), leg speed <= sar_approach_kn
  (default 5 kn, TO CONFIRM W130) and the last waypoint action WP_ACT_SLOW.
- Distance boat -> M <= D + arrival radius = NO route (text «Μέσα στην απόσταση ασφαλείας –
  χειροκίνητη προσέγγιση»). The final approach is manual by the captain; text on screen + manual:
  «ΚΙΝΔΥΝΟΣ ΕΛΙΚΑΣ – τελική προσέγγιση χειροκίνητα, μηχανή στη νεκρά κοντά στο άτομο».
- Pure function groundstation/sar_geo.py standoff_route(boat, marker, current, cfg) -> wps | why.

## 4. Search patterns (K1, step PF2) - IAMSAR Vol. II geometry

Pure module `groundstation/sar_geo.py` (flat-earth local ENU around the datum, metres; bearings
true, clockwise; a point = datum + (r @ brg)). Inputs: datum, S (track spacing m), C (first leg
course: default the current's set_deg reversed = into the current when known, else typed), size,
speed, (UAV) altitude. Turns to STARBOARD (IAMSAR standard) unless "port" is chosen.

- Expanding square (SS): leg k = 1..n has length L_k = ceil(k/2) x S (S, S, 2S, 2S, 3S, 3S ...);
  course of leg k = C + 90 (k-1) (right turns). Waypoints: datum, then the cumulative ends -> n + 1
  points. Total length = S x sum ceil(k/2) = S x (floor(n/2)(floor(n/2)+1) + (n odd ? ceil(n/2) : 0)).
  Size given as legs n, or a half-side R -> n = 2 floor(R/S) + 1 (the square covered = (n_max S)^2).
- Sector search (VS): radius R, 9 legs of length R, 120 deg starboard turns; the corner points lie
  on the circle at C, C+60, C+120, C+180, C+240, C+300. Waypoints (10): datum, R@C, R@(C+60),
  datum, R@(C+240), R@(C+300), datum, R@(C+120), R@(C+180), datum. Total 9R. Track spacing at the
  rim = R (60 deg chord), at R/2 = R/2. Second search option: the same with C + 30.
- Parallel track (PS) over a W (across) x Lg (along C) rectangle, corner start: tracks n = ceil(W/S),
  the first S/2 inside the edge; search legs along C of length Lg - S (S/2 in from each end),
  cross legs of length S, alternating C / C+180. Waypoints 2n. Total = n (Lg - S) + (n - 1) S.
- Creeping line (CS) = parallel track with the search legs ACROSS the long side (legs along C - 90
  for a starboard turn, C + 90 for port - as built, PF2-r; advance along C) - same formulas with W and Lg swapped (cheap: PF2 if time allows).
- Shown before sending: numbered legs, total length, time = total / speed, area covered ~ total x S,
  waypoint count. Boat: > 50 waypoints = refused «σπάστε το» with the count + the option "cut at 50"
  (says which leg it ends on); never silently cut. UAV: alt per waypoint (default 50 m), end =
  LOITER_UNLIM over the last point (mission.py, unchanged); uavcheck decides.
- Target box (the "Send route" box, uavroute.py): ΣΚΑΦΟΣ / ΕΠΟΜΕΝΟ UAV / a specific UAV.
- Boat route through prog_upload with final_action PROG_END_STOP (PF2-r: the boat arrives at the last
  waypoint and STOPS - idle + neutral + STANDBY; HOLD was dropped: it keeps the last course with no end
  point, so a route cut at 50 could run on; the YES text says it) unless the datum is a person and the pattern ends at it
  (SS ends far away; VS ends AT the datum -> the last point replaced by the stand-off point, section 3).

## 5. Mark a point from the video (K3, step PF4)

- Click on screen 1's video / the copilot camera view / a UAV video (or the hat / key `mark_key`
  default K with the cursor on the video) -> `groundstation/sar_mark.py`:
  boat camera = gateway vision.camera (height_m, tilt_down_deg, hfov / vfov, yaw_deg; the same
  calibration hudar.py uses, from gw_status vision "camera") + TELEM2 group H roll / pitch +
  heading + the boat position; geometry -> bearing rel + distance d. AS BUILT (PF4-r LOW): the geometry is a
  SEPARATE exact inverse of the AR overlay's pinhole model hudar.Cam.project (groundstation/sar_mark.py
  inverse()), NOT gateway/vision/geometry.py locate() - the planned import was not used.
  UAV camera = the drone's altitude above the water h (relative alt), its ATTITUDE roll / pitch / yaw
  (router_worker.py adds roll / pitch to the drone record next to hdg - router only, NOT allow.py)
  and its camera mount: gateway.json uav slots[].camera {tilt_down_deg, hfov_deg, vfov_deg}
  (fixed mount, TO CONFIRM W133; no gimbal command anywhere).
- Distance from the depression angle alpha below the horizon: d = h / tan(alpha);
  bearing = heading + yaw_cam + atan((u - 0.5) / fx).
- Uncertainty (1 sigma; the circle drawn at 2 sigma):
  sigma_alpha^2 = sigma_att^2 + sigma_cal^2 + (sigma_pix / fy)^2  (rad)
  sigma_d = (h / sin^2 alpha) x sigma_alpha  (~ d^2 / h x sigma_alpha for small alpha)
  sigma_lat = d x sigma_psi, sigma_psi^2 = sigma_hdg^2 + sigma_cal^2 + (sigma_pix / fx)^2
  r_m = 2 x sqrt(sigma_d^2 + sigma_lat^2 + unc_pos^2)  (unc_pos = TELEM unc_m or the UAV GPS eph)
  Defaults: sigma_att 1.0 deg boat / 2.0 deg UAV, sigma_cal 0.5 deg, sigma_hdg 2 deg, sigma_pix
  1.5 px at the video's resolution.
- Refused (with the reason, nothing made): alpha <= 0.3 deg (at / above the horizon), d > max_range
  (boat 1000 m, UAV 1500 m), sigma_d / d > 0.35 («πολύ αβέβαιο»), no calibration, attitude data older
  than 1 s, no position.
- PF4 AS BUILT (measured refinements of the formulas above, groundstation/sar_mark.py): sigma_alpha^2 also holds
  the roll error x the click's sideways offset (sigma_att^2 (1 + x^2), x = (u - 0.5) / fx: big in a 120 deg UAV
  view); sigma_d = half of h / tan(alpha - s) - h / tan(alpha + s) (equal to h / sin^2 alpha x s for small s,
  honest when s is a large part of alpha); the refusal limit is sigma_d / d > 0.35 (was 0.5 here: at 0.5 the
  true place was inside the circle in only 52-75 % of the far UAV cases, selection bias at the edge). Measured
  (test_sar MK4, 12000 trials, bands = the ESTIMATED range): boat 10-20 m error 2 m / circle 5 m / inside 97 %,
  20-35 m 4 / 12 / 86 %, 35-50 m 9 / 25 / 88 %; UAV at 50 m 50-150 m 8 / 21 / 93 %, 150-400 m 47 / 132 / 84 %,
  400-900 m 133 / 281 / 81 %. A boat camera (3 m high) gives a useful position only to about 50 m.
- PF4-r AS BUILT (review of d34ab45, MED-3 / MED-4): sigma_psi^2 also holds the roll error x the click's VERTICAL
  offset (sigma_att^2 y^2, y = (v - 0.5) / fy: the roll turns an off-centre point sideways); r_m = circle_k x the
  1-sigma (circle_k 4.0, was 2): measured in the MODEL over the WHOLE field of view (MK4: boat +-40 deg, UAV
  +-60 deg, 12000 trials) the true place is inside in 93-100 % per band (boat 10-20 / 20-35 / 35-50 m: 100 / 98 /
  99 %, circle 11 / 24 / 49 m; UAV at 50 m 50-150 / 150-300 / 300-900 m: 100 / 97 / 93 %, circle 49 / 193 / 414 m;
  MK4 requires >= 92 %). MODEL only - not a real lens (said in MANUAL ch. 23 / MANUAL_EL ch. 18). Freshness: the
  boat's attitude age = the telemetry sample's age at the station + att_age_ms, the UAV's = att_age / alt_age + the
  record's age at the station (uav_store receive time), > 1 s = refused (MK11); the VIDEO delay (0.2-0.5 s typ.) is
  not known to the station: the attitude at the click is used (stated in the manuals). tilt_down_deg of a UAV mount
  range-checked -30..90; the still cap 40 KB on both sides. The gateway's sar_still writes in a helper thread, and
  Recorder.note_sar only queues - the recorder worker adds it to the open events and copies the still without
  Recorder.lock (13n4b; SA5). A delete is said explicitly (sar_markers "deleted", SarStore.deleted, kept in the
  state file): a station deletes a sent point ONLY then - a point missing from an empty / restarted gateway list is
  handed in again (SA6, MK8).
- PF4 AS BUILT, data: the router adds roll / pitch / att_age to the drone record (ATTITUDE, router_worker.py only;
  allow.py unchanged), the manager adds the fixed mount "camera" per slot; station message sar_still {id, jpg_b64}
  (<= 40 KB: one station line is 64 KB) -> <data>/sar/<id>.jpg + marker "still" + every OPEN SAVE event's
  event.json "sar" list (recorder.note_sar, also every marker change); sar_ack {what "still"}. Click = screen 1's
  free video (driver.mark_at) / screen 2's camera view / the key `mark_key` (K) under the mouse; the menu on a marker
  asks a vertical YES / NO before route / UAV / delete; a point deleted on another station disappears here.
- Result = `sar_mark add` kind "point" (est true, r_m, src) + `sar_still` (a 160 x 120 crop around
  the click). Actions on the marker: route there (stand-off if "person"), UAV there, PF2 datum,
  rename, delete. Accuracy measured in the sim on known points (numbers into MANUAL).

## 6. ADS-B (K4, step PF5) - receive only

- gateway/adsb_worker.py = OWN low-priority process: reads readsb / dump1090 SBS (TCP 30003,
  127.0.0.1) or Beast (30005); gateway/adsb.py parses (SBS MSG,1..8 / Beast 0x1a framing, DF17
  position via the receiver's decoded SBS only - no CPR decoding of our own), keeps targets
  (stale 60 s, removed 180 s), filters alt <= adsb_max_alt_ft (3000 ft) and range <= adsb_max_km
  (20 km) (TO CONFIRM W132). readsb itself = tools/bin/ (pinned, tools/get_readsb.py, Ubuntu:
  the distro package; Windows: dump1090 build TO CONFIRM). gw_status "adsb" {state RX | NO_RX |
  NO_DATA, targets [{icao, cs, lat, lon, alt_ft, gs_kn, trk, vs_fpm, age}], warn [...]}.
- CPA per (aircraft, flying UAV): relative position / velocity in ENU (aircraft from SBS, UAV from
  its GLOBAL_POSITION_INT velocity), t_cpa = -(r.v)/|v|^2 clipped 0..300 s; horizontal d_h and
  vertical d_v at t_cpa. CAUTION: d_h < 1000 m AND d_v < 150 m within 120 s; WARNING: d_h < 500 m
  AND d_v < 100 m within 60 s (TO CONFIRM W132). UNKNOWN altitude (owner W137, 2026-11-10, step AB-r):
  an aircraft without an altitude and / or a UAV whose altitude is unknown (missing / non-finite /
  stale) = CO-ALTITUDE (d_v taken as 0): the horizontal CPA alone gives CAUTION / WARNING, the text
  says «ύψος άγνωστο» / "altitude unknown", HOLD / DESCEND is proposed as for any conflict (the
  unknown side at the known side's altitude; both unknown = DESCEND) - replaces "unknown = CAUTION
  only, no DESCEND". Notices / lamps station-local
  (MasterLamps.set_local "adsb_<icao>_<slot>") + voice «Αεροσκάφος κοντά στο UAV 3»; never a boat
  alarm bit.
- Proposal (vertical Y/N, that UAV's pilot or the captain): HOLD = the existing `uav_hold {uav}`;
  DESCEND = a `uav_route` re-upload of the remaining points at alt_safe (default 30 m) through the
  in-flight change rules. NEVER automatic; no answer = nothing.
- No receiver / no frames 10 s = notice «ADS-B: χωρίς δέκτη» / «χωρίς δεδομένα», the map says
  NO DATA (never an empty "clear" sky). Manual: not every aircraft transmits ADS-B.
- Sim: gateway/simadsb.py = an SBS server on a local port with scripted aircraft (a helicopter
  crossing the UAV area at 500 ft); gateway.json "adsb": {"sim": true}.
- Map: symbol by track, callsign, altitude, speed, 2 min track (mapview THEMES "adsb").

## 7. Demo scenario (K5, step PF6)

demo/sar_demo.py (+ a "SAR DEMO" entry in the demo launcher): boat in AUTO on a route ->
MOB pressed (script presses via the station's own button handler) -> return offer (waits for the
presenter's YES) -> UAV over the MOB -> an expanding square for a second (simulated) person ->
mark from the UAV video (simcam draws a figure at a known point) -> simulated helicopter (ADS-B
CAUTION / WARNING, the HOLD proposal answered by the presenter). Every YES waits. Academy lesson +
scenario "MOB and search" (simulator only, academy/uavsim + the boat sim; simguard rules).

AS BUILT (PF6): DEMO_SAR.bat -> demo.py start --sar; the guide window demo/sar_demo.py only listens (read-only gateway user) and never answers a question: the presenter does. The helicopter starts on the guide's cue (simadsb, TRYGONS_SIM_ADSB_CUE); the "second person" is a disc in UAV 1's simulated picture (TRYGONS_SIM_PERSON). Academy: l06_mob_search, s11_mob_search. Tests demo/test_sar_demo.py SD1-SD5.

## 8. Tests per step

- PF3 MOB (groundstation/test_sar.py MB1..): marker = the boat position of the PRESS sample (a later
  telemetry sample never moves it); WARNING + callout + SAVE message sent; no gateway = local marker
  + alarm, sent later once (idempotent id); offers need the captain (copilot -> ask_captain;
  single operator own); the return route ends at the stand-off point with PROG_END_STOP and passes /
  fails prog_check like a hand route; inside the stand-off = no route; key M not in a text field;
  dead camera still works; gateway SarStore (test_gateway section SA1..): add / move / close /
  VIEWER refused 128 / broadcast to every station / restart keeps the file.
- PF2 patterns (test_sar.py SP1..): SS / VS / PS (/ CS) leg lengths, turn angles, spacing vs the
  formulas of section 4 (incl. port turns, wrap of bearings); total length / time / waypoint count;
  > 50 = refused / cut says which leg; a pattern route fails the same shallow / geofence checks as a
  hand route (gateway prog_check); UAV mission altitude per point + uavcheck refusals; nothing sent
  without the captain's YES.
- PF4 mark (test_sar.py MK1..): known-point geometry boat camera + UAV camera (forward and inverse:
  project a known point to (u, v), mark it, error < r_m); horizon / range / uncertainty refusals;
  the marker reaches every station; still stored; nothing moves without the captain.
- PF5 ADS-B (gateway/test_adsb.py AB1..): SBS + Beast parsing (bad lines counted), stale / removed,
  filters, CPA geometry (head-on, crossing, diverging, vertical separation), CAUTION / WARNING
  levels, the proposal never acts by itself (no uav_hold / uav_route without the answer), NO_RX /
  NO_DATA notice, the worker in its own process + a job (strays).
- PF6: the demo script runs end to end against the sim with scripted answers (test_sar SD1), the
  academy lesson on the simulator (academy AC57..), screenshots docs/screens/sar/.
- Every step: tools/check.sh, `git diff` of app/ hal/ gateway/uav/allow.py EMPTY.
