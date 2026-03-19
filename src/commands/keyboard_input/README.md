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

### 1.3 与 Unitree Guide 控制器（unitreeguide）配合使用 — 详细说明

- 启动要点：

  - 启动仿真或真实机器人驱动并确保 `joint_states` / IMU / `tf` 等话题正常发布。
  - 启动 `unitree_guide_controller` 的 launch 文件或通过 `controller_manager` 加载控制器。
    ```bash
    # 示例（根据包内 launch 名称替换）
    source ~/ros2_ws/install/setup.bash
    ros2 launch unitree_guide_controller <launch_file>.launch.py

    # 或使用 controller_manager 的 spawner 加载
    ros2 run controller_manager spawner unitree_guide_controller --controller-manager /controller_manager
    ```
- 键盘节点与话题映射：

  - `keyboard_input` 发布消息类型：`control_input_msgs/Input`。
  - `unitreeguide` 在不同项目中可能订阅不同话题（例如 `cmd_input` / `cmd_vel` / 自定义 topic），请在对应的 `launch`/参数中确认 topic 名称或在包内 README 查看示例。
  - 若需要，把 `keyboard_input` 的输出 remap 到控制器订阅的话题：
    ```bash
    ros2 run keyboard_input keyboard_input __ros2:=<your_namespace>
    ```

    （常用做法：在 launch 文件中使用 `<remap>` 将 `keyboard_input` 的发布 remap 到控制器订阅的主题。）
- 常见操作与安全建议：

  - 上电 / 启动后先保持机器人静止，确认传感器数据正常再切换到行走模式。
  - 使用 `Space` 键作为紧急清零指令；真实硬件上仍需配合硬件急停。
  - 在切换控制器（从手动到自动，或从 unitreeguide 切换到 OCS2）前，先停止当前控制器，或先把速度指令置零并确认机器人处于稳态。

### 1.4 与 OCS2 控制器（ocs2_quadruped_controller）配合使用 — 详细说明

- 作用与先决条件：

  - OCS2 是基于 NMPC 的高级控制器，依赖高频、准确的状态估计（IMU、关节角、里程计/基座位姿）。
  - 启动前请确保状态估计节点、`joint_states`、传感器话题、以及 `/tf` 都稳定可用。
- 启动顺序示例：

  - 在仿真中：
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch ocs2_quadruped_controller <launch_file>.launch.py
    ```
  - 将 `keyboard_input` 或其他输入节点的输出 remap/连接到 OCS2 的命令接口（参见 OCS2 launch 中的 topic 配置）。
- 使用与调试建议：

  - 初次使用时在仿真中充分验证参数（MPC 预测步长、权重、约束、接触模型等）。
  - 将控制器置于 `configured` / `activated` 前，确认机器人处于安全姿态（四脚接触、姿态接近平衡）。
  - 通过 `ros2 topic echo`、`ros2 service call /controller_manager/list_controllers`、以及 OCS2 日志观察控制器状态与误差。
- 切换与回退策略：

  - 若 OCS2 在真实机器人上出现不稳定，立即切换回 `unitreeguide` 或关断输出并进入安全模式。
  - 推荐在 launch 中配置参数文件与快速切换脚本，以便在紧急情况下快速回退。

### 1.5 OCS 特殊按键映射说明（基于源码行为）

本项目中 `keyboard_input` 发布的 `control_input` 对 OCS2 的含义与 `unitreeguide` 不完全相同，以下为源码中的实际映射（已在控制器源码中确认）：

- 数字键（`1` 到 `0`）的行为：

  - `1`：进入被动/停止（OCS2 中 `command == 1` 会触发 FSM 切换到 PASSIVE）。
  - `2` 及以上：OCS2 将把收到的 `command` 值用于选取 gait 列表中的模式，选取规则为 `gait_index = command - 2`（例如按 `2` -> 选择 `gait_list[0]`，按 `3` -> `gait_list[1]`，以此类推）。

  * 按键 `1`：被动 / 停止（OCS 切到 PASSIVE）
  * 按键 `2`：stance（gait_list[0]）
  * 按键 `3`：trot（gait_list[1]）
  * 按键 `4`：standing_trot（gait_list[2]）
  * 按键 `5`：flying_trot（gait_list[3]）
  * 按键 `6`：pace（gait_list[4]）
  * 按键 `7` 或 `Shift+7`（`&`）：standing_pace（gait_list[5]）
  * 按键 `8`：dynamic_walk（gait_list[6]）
  * 按键 `9`：static_walk（gait_list[7]）
  * 按键 `0`：amble（gait_list[8]）
- 连续轴（`WASD` / `IJKL`）与 OCS2：

  - `keyboard_input` 会持续发布 `lx/ly/rx/ry` 值，但 OCS2 的 NMPC 主体通常不直接采用这些原始轴值作为最终关节命令；在本代码中，OCS2 更多将 `control_input.command` 用于离散的 gait/模式切换，而速度/姿态目标的具体映射需在 OCS2 的参数与参考管理器中配置。
- 结论与建议：

  - 如果你在使用 OCS2 时希望通过键盘实时修改期望前进速度或姿态，请在 OCS2 的 launch/config 中确认或添加将 `control_input` 的 `lx/ly/rx/ry` 映射为 MPC 的参考（或修改 `CtrlComponent` / `ReferenceManager` 的处理逻辑）。
  - 若只是切换 gait，请使用数字键 `2` 开始对应索引切换，使用 `1` 回退到被动。

---

如需，我可以把上述启动流程写成示例 launch（带 remap）或提供一个用于快速切换的 `ros2` 命令脚本（含安全停机与 controller_manager 操作）。
