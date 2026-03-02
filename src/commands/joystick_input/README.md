# Joystick Input

本包是 quadruped_ros2_control 工程中的手柄控制输入节点，用于订阅 joystick 相关话题，将手柄按键/摇杆转换为统一的 `control_input_msgs/Input` 控制指令。

该节点会监听手柄相关话题，并发布 `control_input_msgs/Input` 消息。

测试环境：
* Ubuntu 24.04
  * ROS2 Jazzy
  * Logitech F310 游戏手柄

### 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to joystick_input
```

### 运行

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch joystick_input joystick.launch.py
```

## 1. 与 Unitree Guide 控制器的配合使用

### 1.1 模式切换

* 被动模式（Passive）：`LB + B`
* 固定站立（Fixed Stand）：`LB + A`
  * 自由站立（Free Stand）：`LB + X`
  * 小跑（Trot）：`LB + Y`
  * 摆动测试（SwingTest）：`LT + B`
  * 平衡模式（Balance）：`LT + A`

### 1.2 运动控制

* 摇杆 / 按键：控制机器人移动与姿态（具体映射由控制器配置）
* 某些按键组合可用于重置速度或紧急停机（可在代码/launch 中自定义）