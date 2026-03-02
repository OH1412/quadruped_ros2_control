# Hardware Unitree Mujoco

本包是 quadruped_ros2_control 工程中面向 Unitree 机器人 / MuJoCo 仿真的 ros2-control 硬件接口实现，通过纯 ROS 2 话题完成状态 / 指令交互，使控制器与底层 DDS / SDK 解耦。

理论上也可以与真实机器人直接通信，但尚未充分测试。对于仿真，可直接配合 [unitree_mujoco](https://github.com/legubiao/unitree_mujoco) 中的 Go2 仿真使用，该仿真已增加足端力传感器支持。

* [x] **[2025-01-16]** 为仿真增加里程计（odometry）状态输出

## 1. 接口说明（ROS 话题 IO）

所需 ros2-control 硬件接口：

* command（写入）：
  * 关节位置（joint position）
  * 关节速度（joint velocity）
  * 关节力矩（joint effort）
  * KP 增益
  * KD 增益
* state（通过 ROS 话题读取）：
  * 关节力矩（joint effort）
  * 关节位置（joint position）
  * 关节速度（joint velocity）
  * IMU 传感器：
    * 线加速度（linear acceleration）
    * 角速度（angular velocity）
    * 姿态四元数（orientation）
  * 足端力传感器（foot force sensor）
  * 里程计（odometry）

默认 ROS 话题映射：

* state：
  * `/joint_states`（sensor_msgs/JointState）
  * `/imu`（sensor_msgs/Imu）
  * `/foot_force`（std_msgs/Float32MultiArray，顺序：FL，RL，FR，RR）
  * `/odometry`（nav_msgs/Odometry）
* command：
  * `/joint_command`（sensor_msgs/JointState：position / velocity / effort）
  * `/joint_kp`（std_msgs/Float32MultiArray）
  * `/joint_kd`（std_msgs/Float32MultiArray）

以上话题名均可通过 ros2_control 硬件参数重映射。

## 2. DDS 与 ROS 的桥接（用于 MuJoCo / 真实机器人 DDS 源）

如果模拟器 / 机器人仍然通过 Unitree DDS 发布数据（例如 unitree_mujoco），可以运行桥接节点将 DDS 转换为 ROS 话题：

```bash
ros2 run hardware_unitree_mujoco unitree_dds_ros_bridge \
  --ros-args -p network_interface:=lo -p domain:=1
```

这样，硬件插件内部只需订阅 / 发布 ROS 话题即可，不再直接依赖 DDS。

## 3. 编译

测试环境：
* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble

编译命令：

```bash
cd ~/ros2_ws
colcon build --packages-up-to hardware_unitree_mujoco --symlink-install
```

## 4. 话题配置示例

在 xacro / URDF 中配置 ros2_control 硬件插件时，可以如下指定话题名：

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