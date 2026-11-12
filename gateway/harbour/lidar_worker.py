"""The lidar worker = its OWN process (docs/harbour_design.md 5, step HX3): python gateway/harbour/lidar_worker.py
--cfg '<harbour.lidar JSON>' [--world <file>]. The loop + the protocol: worker.py. Simulated (driver "sim") until the
owner chooses the sensor (W148); a real driver name = the skeleton in sensors.py (OFFLINE "not chosen")."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from harbour import worker  # noqa: E402

if __name__ == "__main__":
    sys.exit(worker.run("lidar"))
