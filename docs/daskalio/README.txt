docs/daskalio/ - photos of depth charts of the Daskalio area (Ormos Dhaskalio, Attica)
=====================================================================================

What they are
  IMG_0516.jpg .. IMG_0520.jpg = phone photos of a Garmin chart plotter screen showing the
  depth contours of Ormos Dhaskalio / Akra Aspro (taken by the owner). The originals were
  HEIC files (iPhone); they were converted to JPEG (step 11f, 2026-10-28): the long side
  at most 4032 pixels (enough to read the depth numbers), the photo metadata (EXIF, incl.
  any GPS position of the phone) removed. The originals are NOT in git.

THE RULE (CLAUDE.md section T "Chart photos", docs/dronecan_design.md section 11)
  These photos are for the ACADEMY TEST AREA, the DEMO MAP and the 3D SIM ONLY.

  A PHOTO IS NOT A NAVIGATION CHART. It is NEVER used for the shallow-water check
  (gateway/shallow.py, groundstation/chart.py), the collision / route checks, the
  geofence or any other safety check - unless it has first been georeferenced AND
  checked against an official chart, and that is a separate, reviewed step.

  The safety checks use only charts/ (tools/chartprep.py: EMODnet / GEBCO / OpenSeaMap
  data) with "no data = shallow". A photo of a screen can be distorted, out of date,
  taken at an angle and has no position reference - never steer by it.

  Not for navigation.
