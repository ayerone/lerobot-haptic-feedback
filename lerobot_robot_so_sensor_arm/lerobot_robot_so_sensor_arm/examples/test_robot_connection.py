
from lerobot_robot_so_sensor_arm import SOSensorArmConfig, SOSensorArm

so_follower_port = "/dev/YOUR_ROBOT_SERIAL_PORT" # example: "/dev/ttyACM0"
force_sensor_port = "/dev/YOUR_SENSOR_SERIAL_PORT" # example: "/dev/ttyACM2"

# Note: Consider reusing a robot id that you already have a calibration for
config = SOSensorArmConfig(id="so_with_sensor", port=so_follower_port, sensor_port=force_sensor_port)
arm = SOSensorArm(config)

arm.connect() # will run calibration; provide argument calibration=False to skip

obs = arm.get_observation()
print(obs)

arm.disconnect()
