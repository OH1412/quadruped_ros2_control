# unitree_m8010_controller

ROS2 (Humble) Python node for Unitree GO-M8010-6 initialization, feedback, and control.

## Features
- Stage 1 (first 1s): send zero commands, read feedback, compute per-motor average offset once.
- Stage 2: publish `/joint_states` continuously and apply `offset` to `q_des` when sending commands.
- Reads 12 motors across two serial ports.

## Build
```bash
cd /home/xzx/unitree/unitree_actuator_sdk/ros2_ws
colcon build --packages-select unitree_m8010_controller
source install/setup.bash
```

## Run
```bash
ros2 run unitree_m8010_controller m8010_controller \
  --ros-args -p ports:="[/dev/ttyUSB2,/dev/ttyUSB3]"
```

## Topics
- Publishes: `/joint_states` (`sensor_msgs/JointState`)
- Subscribes: `/unitree_command` (`unitree_motor_msgs/UnitreeCommand`)

## Notes
Set `UNITREE_ACTUATOR_SDK_ROOT=/home/xzx/unitree/unitree_actuator_sdk` if SDK import fails.
