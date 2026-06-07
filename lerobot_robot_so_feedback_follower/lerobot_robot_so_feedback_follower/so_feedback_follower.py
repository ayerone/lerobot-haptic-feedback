
import logging

from lerobot.processor import RobotObservation
from lerobot.robots.so_follower import SO101Follower
from lerobot.utils.decorators import check_if_not_connected
from .config_so_feedback_follower import SOFeedbackFollowerConfig

logger = logging.getLogger(__name__)

class SOFeedbackFollower(SO101Follower):
    config_class = SOFeedbackFollowerConfig
    name = "so_feedback_follower"

    def __init__(self, config: SOFeedbackFollowerConfig):
        super().__init__(config)
        self.config = config

    def get_observation(self) -> RobotObservation:
        obs_dict = super().get_observation()
        obs_dict["gripper.present_current"] = float(self.bus.read("Present_Current", "gripper"))
        obs_dict["gripper.present_load"]    = float(self.bus.read("Present_Load",    "gripper"))
        return obs_dict
