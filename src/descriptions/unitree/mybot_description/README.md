# MyBot Description

This repository contains the URDF model of MyBot quadruped robot.

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

## 1. Build

```bash
cd ~/quadruped_ros2_control
colcon build --packages-up-to mybot_description --symlink-install
```

## 2. Visualize the robot

To visualize and check the configuration of the robot in rviz, simply launch:

```bash
source ~/quadruped_ros2_control/install/setup.bash
ros2 launch mybot_description visualize.launch.py
```

## 3. Launch ROS2 Control

### 3.1 Mujoco Simulator

* OCS2 Quadruped Controller
  ```bash
  source ~/quadruped_ros2_control/install/setup.bash
  ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=mybot_description
  ```

## 4. Configuration Files

### Robot Model
- `xacro/robot.xacro` - Main robot description
- `xacro/const.xacro` - Physical constants and dimensions
- `xacro/leg.xacro` - Leg structure definition

### Control Configuration
- `config/robot_control.yaml` - PD gains and control parameters
- `config/ocs2/task.info` - OCS2 controller task weights
- `config/ocs2/gait.info` - Gait patterns and timing
- `config/ocs2/reference.info` - Reference trajectories

## 5. Customization Guide

To adapt this description for your specific robot:

1. **Update physical parameters** in `xacro/const.xacro`:
   - Body dimensions, mass, inertia
   - Leg lengths, joint ranges
   - Motor torque limits

2. **Adjust control gains** in `config/robot_control.yaml`:
   - PD gains (kp, kd) for each joint

3. **Tune OCS2 parameters** in `config/ocs2/`:
   - `task.info`: Swing height, trajectory weights
   - `gait.info`: Gait cycle timing
   - `reference.info`: Default joint positions

4. **Replace meshes** in `meshes/` folder with your robot's visual models
