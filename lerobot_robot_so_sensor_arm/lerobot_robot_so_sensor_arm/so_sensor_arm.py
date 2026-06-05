
from functools import cached_property
import logging

from lerobot.processor import RobotObservation

# from lerobot.robots.robot import Robot
from lerobot.robots.so_follower import SO101Follower
from lerobot.utils.decorators import check_if_not_connected
from .config_so_sensor_arm import SOSensorArmConfig
from .force_sensor import ForceSensor

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

logger = logging.getLogger(__name__)

class SOSensorArm(SO101Follower):
    config_class = SOSensorArmConfig
    name = "so_sensor_arm"

    def __init__(self, config: SOSensorArmConfig):
        super().__init__(config)
        self.config = config
        self.sensor = ForceSensor(
            port=self.config.sensor_port,
        )

    @property
    def _sensor_ft(self) -> dict[str, type]:
        return {
            "sensor.force": float,
        }
    
    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return { **super().observation_features, **self._sensor_ft }
    #    return {**self._motors_ft, **self._cameras_ft, **self._sensor_ft}
    
    @property
    def is_connected(self) -> bool:
        return super().is_connected and self.sensor.is_connected()
    
    def connect(self, calibrate: bool = True) -> None:
        super().connect(calibrate=calibrate)

        logger.info("connecting to sensor")
        self.sensor.connect()
        # logger.info("connected to sensor")
        # sensor generally does not require calibration
        # if not self.sensor.is_calibrated and calibrate:
        #     logger.info(
        #         "Mismatch between calibration values in the SENSOR and the calibration file or no calibration file found"
        #     )
        #     self.sensor.calibrate()
        
        return
    
    def calibrate(self) -> None:
        super().calibrate()
        self.sensor.calibrate() # dummy method; this sensor requires no calibration
        return

    def get_observation(self) -> RobotObservation:
        obs_dict = super().get_observation()
        # TODO: time the sensor read operation
        obs_dict["sensor.force"] = self.sensor.read()
        return obs_dict
    
    def disconnect(self) -> None:
        super().disconnect()
        self.sensor.disconnect()
        logger.info(f"{self} disconnected.")
        return


