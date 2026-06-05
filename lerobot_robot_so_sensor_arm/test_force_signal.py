"""
Run this script, then make contact with the force sensor.
Prints timestamp, force, and d_force to stdout.
Ctrl+C to stop. Redirect to a file to save:
    uv run python test_force_signal.py > force_log.csv
"""

import time
import sys
from lerobot_robot_so_sensor_arm.force_sensor import ForceSensor

SENSOR_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34338323531351800132-if00"

sensor = ForceSensor(port=SENSOR_PORT)
sensor.connect()

print("t_s,force,d_force")

last_force = 0.0
start = time.perf_counter()

try:
    while True:
        t = time.perf_counter() - start
        force = sensor.read()
        d_force = force - last_force
        last_force = force
        print(f"{t:.4f},{force:.4f},{d_force:.4f}")
        sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    sensor.disconnect()
