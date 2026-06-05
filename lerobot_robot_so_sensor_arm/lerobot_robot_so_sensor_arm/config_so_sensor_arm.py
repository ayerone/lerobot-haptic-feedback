
from dataclasses import dataclass, field
from lerobot.robots.config import RobotConfig
# from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.cameras import CameraConfig


@RobotConfig.register_subclass("so_sensor_arm")
@dataclass
class SOSensorArmConfig(RobotConfig):
    '''
    Mostly a copy of SOFollowerRobotConfig from the lerobot library, added "sensor_port" because
    in this project, the analogue force sensor is connected to a separate arduino, and this
    will be its com port.

    Since this is a @dataclass, adding a property here without a default value will make that 
    property a required argument to instantiate the class, e.g. 
        config = SOSensorArmConfig(port="/dev/ttyACM0", sensor_port="/dev/ttyACM2")
    
    '''
    # motor port
    port: str

    # sensor port
    sensor_port: str

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False

