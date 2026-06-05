import logging
import math
from dataclasses import dataclass

import draccus

from lerobot.teleoperators.so_leader import SO101Leader
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS

from .config_feedback_leader import FeedbackLeaderConfig
from .feedback_motor import FeedbackMotor, GimbalCalibration

logger = logging.getLogger(__name__)


@dataclass
class FeedbackCommand:
    value: float
    vibrate: bool = False
    center: float = 0.0


class GripFeedbackController:
    SENSOR_DEADBAND_THRESHOLD = 2
    VIBRATION_ONSET_FORCE = 5
    FORCE_LIMIT_THRESHOLD = 30
    GRIP_FEEDBACK_SCALAR = 1 / 30
    CONTACT_DERIVATIVE_SCALAR = 3
    DERIVATIVE_DECAY = 0.9
    GIMBAL_RESTORE_SCALAR = 0.05
    TELEOP_EFFECTOR_TOO_OPEN_THRESHOLD = 15
    JAW_OPEN_SCALAR = 0.01

    def __init__(self):
        self._last_force = 0.0
        self._derivative_envelope = 0.0
        self._grip_clamp_position: float | None = None

    @property
    def grip_clamp_position(self) -> float | None:
        return self._grip_clamp_position

    def _compute_vibration_magnitude(self, force: float) -> float:
        d_force = force - self._last_force
        self._last_force = force
        self._derivative_envelope = max(max(0.0, d_force), self._derivative_envelope * self.DERIVATIVE_DECAY)
        steady_term = self.GRIP_FEEDBACK_SCALAR * math.sqrt(max(0.0, force - self.VIBRATION_ONSET_FORCE))
        derivative_term = self.CONTACT_DERIVATIVE_SCALAR * self._derivative_envelope
        return steady_term + derivative_term

    def compute(self, force: float, gimbal_pos: float, gripper_pos: float) -> FeedbackCommand:
        # Above force limit: clamp gripper position and vibrate with gimbal restore
        if force > self.FORCE_LIMIT_THRESHOLD:
            if self._grip_clamp_position is None:
                self._grip_clamp_position = gimbal_pos
            magnitude = self._compute_vibration_magnitude(force)
            gimbal_drift = self._grip_clamp_position - gimbal_pos
            restore = -self.GIMBAL_RESTORE_SCALAR * gimbal_drift if gimbal_drift > 0 else 0.0
            return FeedbackCommand(value=magnitude, vibrate=True, center=restore)

        # Normal gripping: vibrate without restore
        if force > self.SENSOR_DEADBAND_THRESHOLD:
            self._grip_clamp_position = None
            magnitude = self._compute_vibration_magnitude(force)
            return FeedbackCommand(value=magnitude, vibrate=True)

        # No contact: reset state, apply jaw spring if teleop is too far open
        self._grip_clamp_position = None
        self._last_force = 0.0
        self._derivative_envelope = 0.0
        error = gimbal_pos - gripper_pos
        if error > self.TELEOP_EFFECTOR_TOO_OPEN_THRESHOLD:
            return FeedbackCommand(value=self.JAW_OPEN_SCALAR * error)
        return FeedbackCommand(value=0.0)


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
        to_send = min(self._gimbal_position, max_angle)

        if self._grip_controller.grip_clamp_position is not None:
            to_send = max(to_send, self._grip_controller.grip_clamp_position)

        so_action["gripper.pos"] = to_send

        return so_action

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, float]) -> None:
        cmd = self._grip_controller.compute(
            feedback["sensor.force"], self._gimbal_position, feedback["gripper.pos"]
        )
        if cmd.vibrate:
            self.feedback_motor.vibrate(cmd.value, center=cmd.center)
        else:
            self.feedback_motor.write(cmd.value)

    @check_if_not_connected
    def disconnect(self) -> None:
        super().disconnect()
        self.feedback_motor.disconnect()
