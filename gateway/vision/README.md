# Camera obstacle detection (eWaSR) on the gateway

The gateway looks at the active front camera (FRONT-1, or FRONT-2 when it has
switched over) and finds obstacles on the water. **It never steers and never
asks for more throttle.** At most it asks the AURIX for a lower speed (STAGE 3,
off by default). Everything else is a display, a log and an alarm.

## Licence (checked 2026-10-22)

| what | licence | use here |
|---|---|---|
| eWaSR code (github.com/tersekmatija/eWaSR) | Apache-2.0 | allowed, also commercial; keep the licence and notices |
| eWaSR trained weights (release 0.1.0) | Apache-2.0 (README "License and usage") | allowed; `models/LICENSE-eWaSR.txt` is copied next to the model |
| MaSTr1325 dataset (test pictures, demo video) | CC BY 4.0 | allowed, also commercial, **with attribution**: Bovcon, Muhovic, Pers, Kristan, "The MaSTr1325 dataset for training deep USV obstacle detection models", IROS 2019 |

The model file and the dataset are downloaded by the tools below, never stored in git.

## The model

- **ewasr_resnet18**: the lightest published eWaSR variant (ResNet-18 backbone, no IMU input).
  The official ONNX file of release 0.1.0 (241 MB, sha256 `15e520fc5f7e910c9367556db9a9c1107bdc9c74e7982b3bf89c7050509411d7`).
- Input `image`: 1 x 3 x **384 x 512** (fixed in the published file), RGB, /255, ImageNet mean / std.
  The camera sub-stream (stream2) is scaled to 512 x 384 by ffmpeg. The input size can't be changed
  without exporting the model again from PyTorch (its resize steps have fixed sizes); the
  configurable part is the frame rate (`max_fps`) and the threads.
- Output `prediction`: 1 x 3 x 96 x 128 class scores: 0 obstacle, 1 water, 2 sky.
- Speed (ONNX Runtime, CPU, 2 threads): this PC about 120-190 ms per picture. **Important**:
  the session option `session.set_denormal_as_zero` (in worker.py) - without it the same model
  takes ~1500 ms on x86 (tiny "denormal" weights). **The gateway PC (ThinkPad L390 Yoga,
  i5-8265U) must be measured** (`tools/gateway_load.py`, gateway/README.md section 21). An INT8 (quantised) version took 75 ms instead of 115 ms here and
  gave the same class for 99.8 % of the pixels on 4 test pictures - not used yet (it needs
  calibration pictures from your own camera; INT8 helps most on small CPUs).

## The steps (worker.py, then manager.py)

1. ffmpeg reads `rtsp://127.0.0.1:8554/cam_front1_lo` (the sub-stream through MediaMTX),
   scales to 512 x 384, at most `2 x max_fps` frames / s. A reader thread keeps **only the
   newest frame** - older ones are dropped, never queued.
2. The model -> class map 96 x 128 and the obstacle probability.
3. Obstacle blobs: connected obstacle cells whose lower edge or side **touches water**
   (things on / in the water); the bottom 15 % of the picture is ignored (own bow, own wake);
   a confidence per blob (mean obstacle probability); blobs wider than 60 % of the picture = the
   shore (ignored as obstacles, still in the overlay).
4. The horizon: from the boat's IMU (BNO085 roll / pitch, TELEM2 group H) when the AURIX sends
   it, else the water / sky boundary in the picture, else only the camera tilt.
5. Bearing (camera field of view) and a **rough** distance from how far below the horizon the
   waterline is (camera height). Always shown as a range and "(rough)".
6. Path corridor: +-25 m around the path ahead (heading + turn rate), at least 150 m or 60 s.
7. Persistence: an obstacle counts only after **3 processed frames in a row** (wakes, foam,
   glare are filtered); forgotten after 2 misses.
8. Stages (each change logged; STAGE 3 ON needs the driver screen's ACCEPT):
   - STAGE 1 (ON): overlay + logging only
   - STAGE 2 (ON): obstacle in the corridor in a driving mode -> alarm OBSTACLE AHEAD
     (advisory: no effect on the boat) -> automatic video SAVE
   - STAGE 3 (OFF until after sea trials): AUTO slows to 3 kn (idle when nearer than 40 m);
     HEADING LOCK: a slow-down proposal on the driver screen; MANUAL / REMOTE: alarm only.
     Only a speed cap through the navigation request (validated by the AURIX like the AIS
     avoidance). A cap that was active stays while vision is not working, until the driver
     clears it (driver screen: CLEAR SLOWDOWN).

## Resource protection

- Own process, `IDLE` priority (Windows) / `nice 19` (Linux), only on the last `threads` cores
  (default 2 of 4), ONNX Runtime with `threads` threads, no spinning.
- Paused when the gateway's event loop runs late (> 150 ms), the CPU is above 90 %, the CPU is
  above 78 °C or thermal-throttles (Ubuntu: the hottest thermal zone + the Intel throttle
  counters, gateway/osdep.py; Windows: CPU % and loop lag only); resumed after 10 s OK.
  Status PAUSED.
- Below `min_fps` (1.0) -> VISION DEGRADED. Crash / no model / no camera -> VISION OFFLINE and a
  restart (5 s, 10 s, ... 60 s). A notice on the screens and in the audit log, never an alarm.

## Files

| file | what |
|---|---|
| worker.py | the separate process (numpy + onnxruntime) |
| manager.py | on the gateway: health, pausing, tracks, stages, the speed cap, logging, samples |
| geometry.py | horizon, bearing, rough distance, corridor (pure Python) |
| evaluate.py | run the model on a video / folder -> test_results/vision_eval/report.md |
| models/ | ewasr_resnet18.onnx + LICENSE-eWaSR.txt (`python tools/get_vision_model.py`) |
| data/ | MaSTr1325 (`python tools/get_vision_samples.py`, also makes demo/media/sea.mp4) |

Logs: `<SSD>/vision/detections-YYYYMMDD.jsonl` (every confirmed obstacle: time, bearing, rough
distance, confidence, stage action), `<SSD>/vision/worker.log`. Samples (frame .ppm + mask
.pgm, off by default, `vision.samples`): `<SSD>/vision/samples/`, oldest deleted above
`max_mb`, none while the SSD is low / full - for a later own Saronic dataset.

## On the gateway PC (Ubuntu 24.04 LTS or Windows 11, CPU only - no NVIDIA)

1. ffmpeg (already there for the recording: Ubuntu `sudo apt install ffmpeg`, Windows MSYS2 /
   the PATH, gateway/toolpaths.py).
2. The worker's Python: Ubuntu `pip install onnxruntime==1.20.1 numpy==2.1.3` in the gateway's
   Python (x86-64 wheels); Windows `python tools/get_vision_python.py` (an own embedded Python
   in tools/bin/vision-python/, found by toolpaths.vision_python()).
3. Model: `python3 tools/get_vision_model.py` (241 MB into gateway/vision/models/).
4. Nothing else to start: the gateway service / task starts the worker itself (`vision.enabled`),
   at the lowest priority (Ubuntu nice 19, Windows IDLE class) on its own 2 cores.
5. Check: the captain screen, Cameras tab, "VISION ACTIVE x fps", and run
   `python tools/gateway_load.py` once on the gateway PC (CPU / RAM / fps with 4 cameras).
   Below 1 fps it shows DEGRADED: lower `max_fps`, or the INT8 model, or the upgrade below.

Later upgrade (not implemented): a USB NPU or an OAK-D camera (eWaSR
exports to its .blob directly) would run the model many times faster; fine-tuning on own
Saronic recordings (the samples above) would make it fit these waters.

## On the PC (demo, tests)

`python tools\get_vision_python.py` (a separate Windows Python with onnxruntime in
tools\bin\vision-python - MSYS2's onnxruntime package can't be imported), then
`get_vision_model.py` and `get_vision_samples.py`. Evaluation:

    tools\bin\vision-python\python.exe gateway\vision\evaluate.py gateway\vision\data\mastr1325\images --masks gateway\vision\data\mastr1325\masks --max 100
    tools\bin\vision-python\python.exe gateway\vision\evaluate.py my_recording.ts --fps 2
