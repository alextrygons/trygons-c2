"""Harbour autonomy on the gateway (CLAUDE.md Z, docs/harbour_design.md).

HX3: the sensor interface (sensors.py), the sensor workers (radar_worker.py / lidar_worker.py = OWN processes,
worker.py the shared loop), the simulated harbour world + radar + LiDAR (simworld.py, simradar.py, simlidar.py)
and the gateway side (manager.py: HarbourSensors). No tracker (HX4), no planner (HX5), no UI (HX6).

RULE: nothing in this package imports aurix.py or sends anything to the AURIX (gateway/test_harbour_sensors.py HS9).
"""
