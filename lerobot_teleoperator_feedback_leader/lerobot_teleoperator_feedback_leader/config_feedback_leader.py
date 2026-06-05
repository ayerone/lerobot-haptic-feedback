from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("feedback_leader")
@dataclass
class FeedbackLeaderConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str

    # Port to connect the logsplitter
    feedback_port: str

    # Whether to use degrees for angles
    use_degrees: bool = False
