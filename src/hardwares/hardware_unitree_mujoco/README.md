# Hardware Unitree Mujoco

This package contains the hardware interface to control the Unitree robot or Mujoco simulation using ROS 2 topics only (DDS-free in the hardware plugin). 

In theory, it also can communicate with real robot, but it is not tested yet. You can use go2 simulation in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco). In this simulation, I add foot force sensor support.

*[x] **[2025-01-16]** Add odometer states for simulation. 

## 1. Interfaces (ROS topic IO)

Required hardware interfaces:

* command:
  * joint position
  * joint velocity
  * joint effort
  * KP
  * KD
* state (ROS topics):
  * joint effort
  * joint position
  * joint velocity
  * imu sensor
    * linear acceleration
    * angular velocity
    * orientation
  * foot force sensor
  * odometry

Default ROS topics:
* state:
  * /joint_states (sensor_msgs/JointState)
  * /imu (sensor_msgs/Imu)
  * /foot_force (std_msgs/Float32MultiArray, order: FL, RL, FR, RR)
  * /odometry (nav_msgs/Odometry)
* command:
  * /joint_command (sensor_msgs/JointState: position, velocity, effort)
  * /joint_kp (std_msgs/Float32MultiArray)
  * /joint_kd (std_msgs/Float32MultiArray)

These topic names can be overridden via ros2_control hardware parameters.

## 2. DDS<->ROS bridge (for Mujoco/real robot DDS sources)

If your simulator/robot still publishes Unitree DDS (e.g., unitree_mujoco), run the bridge executable to convert DDS to ROS topics:

```bash
ros2 run hardware_unitree_mujoco unitree_dds_ros_bridge \
  --ros-args -p network_interface:=lo -p domain:=1
```

This allows the ROS-only hardware interface to keep working without DDS.

## 3. Build

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

Build Command:
```bash
cd ~/ros2_ws
colcon build --packages-up-to hardware_unitree_mujoco --symlink-install
```

## 4. Config topics
Example ros2_control hardware params (in xacro):
```xml
<hardware>
  <plugin>hardware_unitree_mujoco/HardwareUnitree</plugin>
  <param name="state_joint_topic">/joint_states</param>
  <param name="state_imu_topic">/imu</param>
  <param name="state_foot_force_topic">/foot_force</param>
  <param name="state_odometry_topic">/odometry</param>
  <param name="command_joint_topic">/joint_command</param>
  <param name="command_kp_topic">/joint_kp</param>
  <param name="command_kd_topic">/joint_kd</param>
</hardware>
```