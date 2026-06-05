#!/usr/bin/env bash

# export robot_port=/your/robot/port
# export sensor_port=/your/sensor/port
# export teleop_port=/your/teleop/port
# export HF_USER=your-huggingface-user-name # This one could already be set in your environment

# Error if robot_port, sensor_port, teleop_port, or HF_USER environment variables are not set
set -u

lerobot-record \
    --robot.type=so_sensor_arm \
    --robot.port=$robot_port \
    --robot.sensor_port=$sensor_port \
    --robot.id=my_so_sensor_arm \
    --robot.cameras="{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30} }" \
    --teleop.type=so101_leader \
    --teleop.port=$teleop_port \
    --teleop.id=my_so101_leader \
    --display_data=true \
    --dataset.repo_id="${HF_USER}/force_sensor_test" --dataset.num_episodes=1 --dataset.single_task="Testing the force sensor observation."
