# Unitree Guide Controller

本包是 quadruped_ros2_control 工程中基于 Unitree Guide 的 ros2-control 控制器，实现对 Unitree 系列四足机器人（含仿真）的基础步态与模式切换控制。原始 Unitree Guide 项目见：https://github.com/unitreerobotics/unitree_guide。

由于本实现使用 KDL 进行运动学与动力学计算，与官方实现存在一定差异，在某些工况下可能略显不稳定。

测试环境：

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

[![](http://i1.hdslb.com/bfs/archive/310e6208920985ac43015b2da31c01ec15e2c5f9.jpg)](https://www.bilibili.com/video/BV1aJbAeZEuo/)

## 1. 接口说明

所需 ros2-control 硬件接口：

* command（写入）：
    * 关节位置（joint position）
    * 关节速度（joint velocity）
    * 关节力矩（joint effort）
    * KP 增益
    * KD 增益
* state（读取）：
    * 关节力矩（joint effort）
    * 关节位置（joint position）
    * 关节速度（joint velocity）
    * IMU 传感器（imu sensor）：
        * 线加速度（linear acceleration）
        * 角速度（angular velocity）
        * 姿态四元数（orientation）

## 2. 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_guide_controller
```

## 3. 启动

### 3.1 MuJoCo 仿真

> 提示：在启动本控制器前，需要先根据 https://github.com/legubiao/unitree_mujoco 启动 Unitree MuJoCo C++ 仿真。

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller mujoco.launch.py pkg_description:=go2_description
```

### 3.2 Gazebo Classic 11（ROS2 Humble）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo_classic.launch.py pkg_description:=go2_description
```

### 3.3 Gazebo Harmonic（ROS2 Jazzy）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo.launch.py pkg_description:=go2_description
```