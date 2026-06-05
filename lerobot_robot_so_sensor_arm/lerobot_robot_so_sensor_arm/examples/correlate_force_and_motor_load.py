"""
Teleoperate the arm while logging gripper force sensor, motor current, and motor load to
Rerun for correlation analysis.

Run from the lerobot_2026 directory (where the uv venv lives):
    uv run python /path/to/lerobot_haptic_feedback/lerobot_robot_so_sensor_arm/lerobot_robot_so_sensor_arm/examples/correlate_force_and_motor_load.py
"""

import csv
import time
from pathlib import Path

import rerun as rr

from lerobot_robot_so_sensor_arm import SOSensorArmConfig, SOSensorArm
from lerobot_teleoperator_feedback_leader import FeedbackLeaderConfig, FeedbackLeader

LOG_PATH = Path(__file__).parent / "force_motor_log.csv"

ROBOT_PORT   = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF263835-if00"
SENSOR_PORT  = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34338323531351800132-if00"
LEADER_PORT  = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AA9024519-if00"
GIMBAL_PORT  = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

robot  = SOSensorArm(SOSensorArmConfig(id="my_so_sensor_arm",     port=ROBOT_PORT,  sensor_port=SENSOR_PORT))
teleop = FeedbackLeader(FeedbackLeaderConfig(id="my_feedback_leader", port=LEADER_PORT, feedback_port=GIMBAL_PORT))

rr.init("gripper_force_correlation", spawn=True)

teleop.connect()
robot.connect()

LOOP_PERIOD = 1.0 / 60

print(f"Teleoperating — logging to {LOG_PATH}  Ctrl+C to stop.")
with open(LOG_PATH, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t", "force", "current_mA", "load"])
    t_start = time.monotonic()
    try:
        while True:
            t0 = time.monotonic()

            obs = robot.get_observation()
            teleop.send_feedback(obs)
            action = teleop.get_action()
            robot.send_action(action)

            force      = float(obs["sensor.force"])
            current_mA = float(obs["gripper.present_current"]) * 6.1 * 5
            load_val   = float(obs["gripper.present_load"])

            rr.log("gripper/force_sensor",     rr.Scalars(force))
            rr.log("gripper/motor_current_mA", rr.Scalars(current_mA))
            rr.log("gripper/motor_load",       rr.Scalars(load_val))
            rr.log("gimbal/torque_command",    rr.Scalars(teleop.last_torque))

            writer.writerow([f"{time.monotonic() - t_start:.4f}", f"{force:.3f}", f"{current_mA:.1f}", f"{load_val:.1f}"])

            elapsed = time.monotonic() - t0
            remaining = LOOP_PERIOD - elapsed
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.disconnect()
        robot.disconnect()
