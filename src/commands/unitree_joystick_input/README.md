# Unitree Joystick Input Node

本包是 quadruped_ros2_control 工程中用于对接 Unitree 官方无线遥控器的输入节点，基于 `unitree_sdk2` 将遥控器数据桥接到 ROS 2。

该节点会监听 Unitree 无线遥控器的底层话题，并发布 `unitree_go::msg::dds_::WirelessController_` 消息。

> 使用前请先通过 `ifconfig` 确认用于连接机器人/模拟器的网络网卡名称，并在 launch 文件中修改相应参数。

测试环境：
* Ubuntu 24.04
  * ROS2 Jazzy

### 编译
```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_joystick_input --symlink-install
```

### 运行
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_joystick_input joystick.launch.py
```

## 1. 各控制器的按键映射

### 1.1 Unitree Guide Controller
* 被动模式（Passive）：`select`
* 固定下蹲（Fixed Down）：`start`
* 固定站立（Fixed Stand）：`start`
  * 自由站立（Free Stand）：`right + X`
  * 小跑（Trot）：`right + Y`
  * 摆动测试（SwingTest）：`right + B`
  * 平衡模式（Balance）：`right + A`

### 1.2 OCS2 Quadruped Controller
* 被动模式（Passive）：`select`
* OCS2 模式：`start`
  * 站立（Stance）：`start`
  * 小跑（trot）：`right + X`
  * 站立小跑（standing trot）：`right + Y`
  * 飞行小跑（flying_trot）：`right + B`

### 1.3 RL Quadruped Controller
* 被动模式（Passive）：`select`
* 固定下蹲（Fixed Down）：`start`
* 固定站立（Fixed Stand）：`start`
  * RL 控制模式：`right + X`