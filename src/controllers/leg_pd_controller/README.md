# Leg PD Controller

本包是 quadruped_ros2_control 工程中的基础腿部 PD 控制器，通过在仅支持力矩控制的硬件接口（如 Gazebo ros2_control）上提供位置 / 速度 / KP / KD 接口，使其它基于位置控制的 ros2-control 控制器也可以在“仅力矩接口”的仿真 / 硬件上复用。

测试环境：
* Ubuntu 24.04
  * ROS2 Jazzy

## 1. 接口说明

本控制器对外提供的关节接口：
* 关节位置（joint position）
* 关节速度（joint velocity）
* 关节力矩（joint effort）
* KP 增益
* KD 增益

底层所需的硬件接口：
* command：
  * 关节力矩（joint effort）
* state：
  * 关节位置（joint position）
  * 关节速度（joint velocity）

## 2. 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to leg_pd_controller
```

## 3. 运行

可在 Gazebo 仿真中与 Go1 / A1 等机器人模型配合使用，示例参考：`../../descriptions/quadruped_gazebo`。