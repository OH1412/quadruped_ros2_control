# Keyboard Input

本包是 quadruped_ros2_control 工程中的键盘控制输入节点，用于从键盘读取按键并发布 control_input_msgs/Input 消息，统一驱动不同控制器（Unitree Guide / OCS2 / RL 等）。

该节点会读取键盘输入，并发布 `control_input_msgs/Input` 消息。

测试环境：
* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble

### 编译
```bash
cd ~/ros2_ws
colcon build --packages-up-to keyboard_input
```

### 运行
```bash
source ~/ros2_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

## 1. 与 Unitree Guide 控制器的配合使用

### 1.1 模式切换
* 被动模式（Passive）：数字键 1
* 固定站立（Fixed Stand）：数字键 2
  * 自由站立（Free Stand）：数字键 3
  * 小跑（Trot）：数字键 4
  * 摆动测试（SwingTest）：数字键 5
  * 平衡模式（Balance）：数字键 6

### 1.2 运动控制
* `WASD` / `IJKL`：前后左右/姿态调整（具体含义视控制器配置而定）
* `Space`：将速度指令清零
* `7` 或 `Shift+7`（在终端里会生成 `&`）：在小跑（Trot）模式下触发原地踏步（Wave All）