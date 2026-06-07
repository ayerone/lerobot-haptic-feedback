
from dataclasses import dataclass, field
from lerobot.robots.config import RobotConfig
from lerobot.cameras import CameraConfig


@RobotConfig.register_subclass("so_feedback_follower")
@dataclass
class SOFeedbackFollowerConfig(RobotConfig):
    # motor port
    port: str

    disable_torque_on_disconnect: bool = True

    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    use_degrees: bool = False
