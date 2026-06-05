import logging
import math

import draccus

from lerobot.teleoperators.so_leader import SO101Leader
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS

from .config_feedback_leader import FeedbackLeaderConfig
from .feedback_motor import FeedbackMotor, GimbalCalibration

logger = logging.getLogger(__name__)


class FeedbackLeader(SO101Leader):
    config_class = FeedbackLeaderConfig
    name = "feedback_leader"

    CURRENT_TORQUE_SCALAR = 0.3
    CURRENT_LOCKOUT_THRESHOLD = 2.0  # counts; zero torque below this to prevent idle oscillation
    SIGNAL_SMOOTH_ALPHA = 0.2   # EMA on input signals; ~2 Hz cutoff at 60 Hz
    TORQUE_SMOOTH_ALPHA = 0.3   # EMA on output torque; ~14 dB rejection at 20 Hz
    GRIPPER_VELOCITY_K = 0.05   # gripper velocity gate: weight = 1 / (1 + k * |v|)

    def __init__(self, config: FeedbackLeaderConfig):
        super().__init__(config)

        self._gimbal_position = 0
        self.last_torque: float = 0.0
        self._smooth_current: float = 0.0
        self._smooth_load: float = 0.0
        self._smooth_torque: float = 0.0
        self._prev_gripper_pos: float | None = None
        self.gimbal_calibration: GimbalCalibration | None = None
        self.gimbal_calibration_fpath = HF_LEROBOT_CALIBRATION / TELEOPERATORS / self.name / f"{self.id}.gimbal.json"
        self._load_gimbal_calibration()
        self.feedback_motor = FeedbackMotor(
            port=config.feedback_port,
            calibration=self.gimbal_calibration
        )

    @property
    def feedback_features(self) -> dict[str, type]:
        return { "gripper.present_current": float, "gripper.present_load": float, "gripper.pos": float }

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
        so_action["gripper.pos"] = min(self._gimbal_position, self.feedback_motor.robot_jaw_max_angle)

        return so_action

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, float]) -> None:
        α = self.SIGNAL_SMOOTH_ALPHA
        self._smooth_current += α * (feedback["gripper.present_current"] - self._smooth_current)
        self._smooth_load    += α * (feedback["gripper.present_load"]    - self._smooth_load)

        gripper_pos = feedback["gripper.pos"]
        if self._prev_gripper_pos is None:
            gripper_vel = 0.0
        else:
            gripper_vel = (gripper_pos - self._prev_gripper_pos) * 60  # units/s at ~60 Hz
        self._prev_gripper_pos = gripper_pos

        gripper_vel_weight = 1.0 / (1.0 + self.GRIPPER_VELOCITY_K * abs(gripper_vel))

        if self._smooth_current * gripper_vel_weight < self.CURRENT_LOCKOUT_THRESHOLD:
            raw_torque = 0.0
        else:
            # Sign from load, magnitude from current attenuated by gripper speed.
            # Fast gripper movement → likely motion current, not contact force → reduce torque.
            raw_torque = -math.copysign(
                self.CURRENT_TORQUE_SCALAR * gripper_vel_weight * self._smooth_current,
                self._smooth_load
            )

        self._smooth_torque += self.TORQUE_SMOOTH_ALPHA * (raw_torque - self._smooth_torque)

        self.last_torque = self._smooth_torque
        self.feedback_motor.write(self._smooth_torque)
        try:
            import rerun as rr
            rr.log("gimbal/torque_command", rr.Scalars(self._smooth_torque))
        except Exception:
            pass

    @check_if_not_connected
    def disconnect(self) -> None:
        super().disconnect()
        self.feedback_motor.disconnect()
