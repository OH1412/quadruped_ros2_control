# Unitree Feedback Reader

本包是 quadruped_ros2_control 工程中的低层反馈读取节点，基于 Unitree Actuator SDK 从电机总线读取力矩 / 速度 / 位置等信息，必要时发送使能命令，并在终端打印调试信息、同时发布 `sensor_msgs/JointState` 供上层控制器与可视化使用。

## 帧格式（16 字节）

- 头部（Header）：`0xFD 0xEE`
- 模式字节（Mode byte）：
  - 位 0–3：电机 ID
  - 位 4–6：状态位
- 负载（Payload）：
  - `tau_fbk`（int16）——力矩反馈原始值
  - `omega_fbk`（int16）——角速度反馈原始值
  - `theta_fbk`（int32）——角度反馈原始值
  - `temp`（int8）——温度
  - `merror + force`（uint16）——错误标志与力值编码
- CRC：对字节 0–13 做 CRC16-CCITT（多项式 0x1021，初始值 0xFFFF）

物理量换算：

- $\tau = \tau_{fbk} / 256$（力矩，单位 N·m）
- $\omega = (\omega_{fbk} / 256) \times 2\pi$（角速度，单位 rad/s）
- $\theta = (\theta_{fbk} / 32768) \times 2\pi$（角度，单位 rad）

## 编译

建议在 quadruped_ros2_control 所在工作空间中直接编译：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select unitree_feedback_reader
```

如果你仍在 Unitree 官方的 unitree_actuator_sdk 工程内使用本包，可参考原始路径：

```bash
cd /home/xzx/unitree/unitree_actuator_sdk/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select unitree_feedback_reader
```

## 运行

在对应工作空间下加载安装环境后运行节点：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run unitree_feedback_reader feedback_reader
```

或在 unitree_actuator_sdk 的 ROS2 工作空间中：

```bash
source /home/xzx/unitree/unitree_actuator_sdk/ros2_ws/install/setup.bash
ros2 run unitree_feedback_reader feedback_reader
```

## 话题（Topics）

- 发布 `sensor_msgs/JointState` 到 `/joint_states`（可通过参数修改）

## 参数（Parameters）

- `ports`：串口设备列表，默认值 `[/dev/ttyUSB0, /dev/ttyUSB1]`
- `bus0_ids`：挂在 `/dev/ttyUSB0` 上的电机 ID，默认 `[1..6]`
- `bus1_ids`：挂在 `/dev/ttyUSB1` 上的电机 ID，默认 `[7..12]`
- `baudrate`：串口波特率，默认 `4000000`
- `print_rate_hz`：终端打印频率（Hz），默认 `50`
- `joint_state_topic`：JointState 发布话题名，默认 `/joint_states`
- `joint_names`：12 个关节名称列表
- `show_extra`：若为 `true`，在终端额外打印 tau / temp / merror / force 等信息
- `send_rate_hz`：发送命令循环频率（Hz），默认 `200`
- `enable_kp`，`enable_kd`，`enable_q`：使能阶段使用的 KP / KD / 目标位置
- `enable_duration_sec`：保持使能阶段的时间（秒），之后发送零命令进入纯反馈模式

## 说明（Notes）

- 依赖 `unitree_actuator_sdk` 的 Python 绑定（`lib/unitree_actuator_sdk*.so`）。
- 节点启动后会先发送使能命令，使电机进入工作状态，然后发送零命令以触发持续的反馈数据流。
