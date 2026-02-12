#!/usr/bin/env bash
set -euo pipefail

cd /home/xzx/unitree/unitree_actuator_sdk/ros2_ws
colcon build --packages-select unitree_m8010_controller
source install/setup.bash
ros2 run unitree_m8010_controller m8010_controller \
  --ros-args -p ports:="[/dev/ttyUSB2,/dev/ttyUSB3]"
