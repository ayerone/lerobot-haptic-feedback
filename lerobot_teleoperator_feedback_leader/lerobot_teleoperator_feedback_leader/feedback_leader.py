import logging
import math

import draccus

from lerobot.teleoperators.so_leader import SO101Leader
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS

from .config_feedback_leader import FeedbackLeaderConfig
from .feedback_motor import FeedbackMotor, GimbalCalibration

logger = logging.getLogger(__name__)


class GripFeedbackController:
    SENSOR_DEADBAND_THRESHOLD = 2
    FORCE_LIMIT_THRESHOLD = 30
    TELEOP_EFFECTOR_TOO_OPEN_THRESHOLD = 15
    JAW_OPEN_SCALAR = 0.01

    FORCE_SETPOINT = 20.0
    AUTO_GRIP_P_GAIN = 0.01
    AUTO_GRIP_DEADBAND = 4.0
    AUTO_GRIP_BREAK_THRESHOLD = 3.0
    GRIP_SPRING_SCALAR = 1.0
    MAX_INTENTIONAL_SQUEEZE = 2.0

    def __init__(self):
        self._grip_clamp_position: float | None = None
        self._auto_grip_pos: float | None = None
        self._tightest_pos: float | None = None
        self._auto_grip_broken: bool = False

    @property
    def grip_clamp_position(self) -> float | None:
        return self._grip_clamp_position

    @property
    def auto_grip_pos(self) -> float | None:
        return self._auto_grip_pos

    def _grip_spring_torque(self, gimbal_pos: float, reference: float) -> float:
        # Negative torque pushes the gimbal toward opening when closed past reference.
        displacement = max(0.0, reference - gimbal_pos)
        return -self.GRIP_SPRING_SCALAR * math.sqrt(displacement)

    def compute(self, force: float, gimbal_pos: float, gripper_pos: float) -> float:
        # Break auto-grip if user opens teleop significantly beyond current target
        if self._auto_grip_pos is not None and gimbal_pos > self._auto_grip_pos + self.AUTO_GRIP_BREAK_THRESHOLD:
            self._auto_grip_pos = None
            self._tightest_pos = None
            self._auto_grip_broken = True

        # Above force limit: activate/update auto-grip with P control
        if force > self.FORCE_LIMIT_THRESHOLD:
            if self._grip_clamp_position is None:
                self._grip_clamp_position = gimbal_pos
            if self._auto_grip_pos is None and not self._auto_grip_broken:
                self._auto_grip_pos = gripper_pos
                self._tightest_pos = gripper_pos
            if self._auto_grip_pos is not None and abs(force - self.FORCE_SETPOINT) > self.AUTO_GRIP_DEADBAND:
                self._auto_grip_pos += self.AUTO_GRIP_P_GAIN * (force - self.FORCE_SETPOINT)
                self._tightest_pos = min(self._tightest_pos, self._auto_grip_pos)
                self._auto_grip_pos = min(self._auto_grip_pos, self._tightest_pos)  # redundant; kept for clarity if blend is reintroduced
            if self._auto_grip_pos is not None and gimbal_pos < self._auto_grip_pos:
                self._auto_grip_pos = max(gimbal_pos, self._tightest_pos - self.MAX_INTENTIONAL_SQUEEZE)
            return self._grip_spring_torque(gimbal_pos, gripper_pos)

        # Normal gripping: activate/update auto-grip with P control
        if force > self.SENSOR_DEADBAND_THRESHOLD:
            self._grip_clamp_position = None
            if self._auto_grip_pos is None and not self._auto_grip_broken:
                self._auto_grip_pos = gripper_pos
                self._tightest_pos = gripper_pos
            if self._auto_grip_pos is not None and abs(force - self.FORCE_SETPOINT) > self.AUTO_GRIP_DEADBAND:
                self._auto_grip_pos += self.AUTO_GRIP_P_GAIN * (force - self.FORCE_SETPOINT)
                self._tightest_pos = min(self._tightest_pos, self._auto_grip_pos)
                self._auto_grip_pos = min(self._auto_grip_pos, self._tightest_pos)  # redundant; kept for clarity if blend is reintroduced
            if self._auto_grip_pos is not None and gimbal_pos < self._auto_grip_pos:
                self._auto_grip_pos = max(gimbal_pos, self._tightest_pos - self.MAX_INTENTIONAL_SQUEEZE)
            return self._grip_spring_torque(gimbal_pos, gripper_pos)

        # No contact: reset clamp and break latch
        self._grip_clamp_position = None
        self._auto_grip_broken = False
        if self._auto_grip_pos is not None:
            return self._grip_spring_torque(gimbal_pos, self._auto_grip_pos)

        # Manual mode: apply jaw spring if teleop is too far open
        error = gimbal_pos - gripper_pos
        if error > self.TELEOP_EFFECTOR_TOO_OPEN_THRESHOLD:
            return self.JAW_OPEN_SCALAR * error
        return 0.0


class FeedbackLeader(SO101Leader):
    config_class = FeedbackLeaderConfig
    name = "feedback_leader"

    def __init__(self, config: FeedbackLeaderConfig):
        super().__init__(config)

        self._gimbal_position = 0
        self._grip_controller = GripFeedbackController()
        self.gimbal_calibration: GimbalCalibration | None = None
        self.gimbal_calibration_fpath = HF_LEROBOT_CALIBRATION / TELEOPERATORS / self.name / f"{self.id}.gimbal.json"
        self._load_gimbal_calibration()
        self.feedback_motor = FeedbackMotor(
            port=config.feedback_port,
            calibration=self.gimbal_calibration
        )

    @property
    def feedback_features(self) -> dict[str, type]:
        return { "sensor.force": float, "gripper.pos": float }

    @property
    def is_connected(self) -> bool:
        return super().is_connected and self.feedback_motor.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.feedback_motor.connect()

        if not self.feedback_motor.is_calibrated:
            self.calibrate_gimbal()

        super().connect(calibrate=calibrate)

        logger.info(f"{self.name} Connected")

    def calibrate_gimbal(self) -> None:
        self.feedback_motor.disable_torque()
        range_min, range_max = self.feedback_motor.record_range_of_motion()
        self.gimbal_calibration = GimbalCalibration(
            range_min=range_min,
            range_max=range_max,
        )

        self._save_gimbal_calibration()
        self.feedback_motor.write_calibration(self.gimbal_calibration)
        self.feedback_motor.enable_torque()

    def calibrate(self) -> None:
        logger.info("Calibrating the BASE ARM")
        super().calibrate()

        logger.info(f"\nCalibrating the GIMBAL MOTOR: {self.feedback_motor}")
        self.calibrate_gimbal()
        logger.info("Finished gimbal calibration")

    def _load_gimbal_calibration(self) -> None:
        fpath = self.gimbal_calibration_fpath
        if not fpath.is_file():
            return
        with open(fpath) as f, draccus.config_type("json"):
            self.gimbal_calibration = draccus.load(GimbalCalibration, f)

    def _save_gimbal_calibration(self) -> None:
        fpath = self.gimbal_calibration_fpath
        with open(fpath, "w") as f, draccus.config_type("json"):
            draccus.dump(self.gimbal_calibration, f, indent=4)

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        so_action = super().get_action()

        # Clip at robot_jaw_max_angle so a full gimbal rotation maps to a narrower gripper range, gaining resolution.
        self._gimbal_position = self.feedback_motor.read()
        max_angle = self.feedback_motor.robot_jaw_max_angle

        if self._grip_controller.auto_grip_pos is not None:
            to_send = self._grip_controller.auto_grip_pos
        else:
            to_send = min(self._gimbal_position, max_angle)
            if self._grip_controller.grip_clamp_position is not None:
                to_send = max(to_send, self._grip_controller.grip_clamp_position)

        so_action["gripper.pos"] = to_send

        return so_action

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, float]) -> None:
        torque = self._grip_controller.compute(
            feedback["sensor.force"], self._gimbal_position, feedback["gripper.pos"]
        )
        self.feedback_motor.write(torque)

    @check_if_not_connected
    def disconnect(self) -> None:
        super().disconnect()
        self.feedback_motor.disconnect()
