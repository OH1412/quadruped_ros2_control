#!/usr/bin/env bash
set -euo pipefail

cd /home/xzx/unitree/unitree_actuator_sdk/ros2_ws
colcon build --packages-select unitree_command_publisher
source install/setup.bash
ros2 run unitree_command_publisher unitree_command_publisher
