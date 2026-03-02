# Quadruped ROS2 Control

基于 ros2-control 的四足机器人控制与仿真工程，支持 Mujoco、Gazebo Classic 与 Gazebo Harmonic，多种控制器（传统、MPC、强化学习）以及真实 Unitree Go2 机器人部署。

> Tested on Ubuntu 22.04 (ROS2 Humble) & Ubuntu 24.04 (ROS2 Jazzy)

---

## 1. 仓库总览

本仓库是一个完整的 ROS2 工作空间，主要目录结构如下（仅列出核心模块）：

- src/
  - commands/：上位机命令与键盘控制节点（如 keyboard_input）
  - controllers/：各类 ros2-control 控制器
    - unitree_guide_controller/：基于 Unitree Guide 的控制器
    - ocs2_quadruped_controller/：基于 OCS2 & legged_control 的 NMPC 控制器
    - rl_quadruped_controller/：基于强化学习的控制器
  - descriptions/：机器人模型（URDF/SRDF），包含 Unitree / DeepRobotics / Xiaomi 等多种机型
  - drivers/：传感器与外设驱动
  - estimations/：状态估计相关组件
  - hardwares/：ros2-control 硬件接口与 Gazebo / MuJoCo 插件
    - gz_quadruped_hardware/：Gazebo Harmonic ros2-control 插件
    - hardware_unitree_mujoco/：Unitree MuJoCo 仿真硬件接口
  - libraries/：仿真与工具库
    - gz_quadruped_playground/：基于 Gazebo 的环境与感知仿真
    - grid_map_* / terrain_analysis_* 等环境感知与代价地图组件
  - ocs2_ros2/：OCS2 相关库与 ROS2 接口

- mybot/：基于 Go2 的自定义 MuJoCo 机器人模型
  - README.md：mybot 使用说明
  - QUICK_REFERENCE.md / INTERFACE_ANALYSIS.md：接口与参数详细分析

- build/、install/、log/：colcon 构建输出与日志

更多每个子包的详细说明，可参见对应子目录下的 README。

---

## 2. 功能与特性

- 多控制器框架
  - Unitree Guide 控制器：兼容官方 unitree_guide 接口，支持 Go2 等机型
  - OCS2 Quadruped Controller：基于 NMPC 的鲁棒行走控制，支持地形感知与 MPC/被动模式切换
  - RL Quadruped Controller：支持从 MuJoCo 仿真到真实机器人部署的强化学习控制

- 多仿真后端
  - MuJoCo：高精度动力学仿真，支持自定义 mybot 机型
  - Gazebo Classic：与 ROS2 Humble 集成的经典仿真
  - Gazebo Harmonic (ros-gz)：新一代仿真平台，配套 gz_quadruped_hardware 插件

- 多机器人模型支持
  - Unitree：Go1 / Go2 / A1 / Aliengo / B2
  - Xiaomi：Cyberdog
  - DeepRobotics：Lite3 / X30
  - Anybotics：Anymal C

- 真实机器人部署
  - 已支持真实 Unitree Go2 机器人（含硬件接口与部署流程）

---

## 3. 开发环境与依赖

- 操作系统
  - Ubuntu 22.04 + ROS2 Humble
  - Ubuntu 24.04 + ROS2 Jazzy（建议用于 Gazebo Harmonic）

- 基础依赖
  ```bash
  cd ~/ros2_ws
  rosdep install --from-paths src --ignore-src -r -y
  ```

- 典型第三方依赖（按需选择）
  - MuJoCo / unitree_mujoco（用于高保真仿真）
  - Gazebo Classic / Gazebo Harmonic
  - Pinocchio（OCS2 控制器依赖，不要使用 rosdep 安装的版本）
  - libtorch（RL 控制器依赖，可选 CPU / CUDA 版本）

各控制器与插件的额外依赖，请参考对应子目录下的 README。

---

## 4. 快速上手（Quick Start）

以下假设本仓库位于 `~/ros2_ws/src/quadruped_ros2_control`，并在 `~/ros2_ws` 下进行构建。

### 4.1 基础编译

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-up-to \
    unitree_guide_controller \
    go2_description \
    keyboard_input \
  --symlink-install
```

---

### 4.2 MuJoCo 仿真或真实 Unitree 机器人

**重要 RMW 说明（CycloneDDS 冲突）**

- CycloneDDS ROS2 RMW 可能与 unitree_sdk2 冲突：
  - 若在不使用 `sudo` 的情况下无法启动 Unitree MuJoCo 仿真，则说明存在冲突，不能直接使用 `hardware_unitree_mujoco`。
  - 建议：卸载 CycloneDDS，改用 FastDDS，或参考 unitree_ros2 的配置说明自行编译 cyclone dds 并指定 RMW。

**1）编译 Unitree 硬件接口**

```bash
cd ~/ros2_ws
colcon build --packages-up-to hardware_unitree_mujoco
```

**2）在 Unitree MuJoCo 中使用 mybot 模型**

1. 按照 unitree_mujoco 仓库说明，先能正常运行 Go2 仿真。
2. 将本仓库的 mybot 目录复制到 unitree_mujoco 仓库中，与 go2 目录同级：
   - `.../unitree_mujoco/unitree_robots/go2`
   - `.../unitree_mujoco/unitree_robots/mybot`
3. 将 mybot 所需的 `.obj` / `.stl` 模型文件放入 `mybot/assets` 中（可从 Pangolin 团队处获取）。
4. 详细配置与参数说明参见 [mybot/README.md](mybot/README.md)、[mybot/QUICK_REFERENCE.md](mybot/QUICK_REFERENCE.md) 与 [mybot/INTERFACE_ANALYSIS.md](mybot/INTERFACE_ANALYSIS.md)。

**3）启动 ros2-control 与键盘控制**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller mujoco.launch.py

source ~/ros2_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

![mujoco](.images/mujoco.png)

---

### 4.3 Gazebo Classic 仿真（ROS2 Humble）

**1）安装 Gazebo Classic 及插件**

```bash
sudo apt-get install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
```

**2）编译 Leg PD 控制器**

```bash
cd ~/ros2_ws
colcon build --packages-up-to leg_pd_controller
```

**3）启动仿真与控制器**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo_classic.launch.py

source ~/ros2_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

![gazebo classic](.images/gazebo_classic.png)

---

### 4.4 Gazebo Harmonic 仿真（ROS2 Jazzy & Humble）

> ROS2 Humble 用户请参考 [src/hardwares/gz_quadruped_hardware](src/hardwares/gz_quadruped_hardware) 中的说明。

**1）安装 Gazebo Harmonic 与 ros-gz**

```bash
sudo apt-get install ros-jazzy-ros-gz
```

**2）编译 Gazebo Playground**

```bash
cd ~/ros2_ws
colcon build --packages-up-to gz_quadruped_playground --symlink-install
```

**3）启动仿真与控制器**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo.launch.py

source ~/ros2_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

![gazebo](.images/gazebo.png)

---

## 5. 控制器与关键子包简介

- Unitree Guide Controller
  - 位置 / 速度 / 力矩 + KP/KD 控制接口
  - 依赖 KDL 做运动学与动力学计算，与官方 unitree_guide 行为略有差异
  - 详细说明见 [src/controllers/unitree_guide_controller/README.md](src/controllers/unitree_guide_controller/README.md)

- OCS2 Quadruped Controller
  - 基于 OCS2 与 legged_control 的 NMPC 控制器
  - 支持 MPC / 被动模式切换，并已支持地面真值估计
  - 详细说明见 [src/controllers/ocs2_quadruped_controller/README.md](src/controllers/ocs2_quadruped_controller/README.md)

- RL Quadruped Controller
  - 使用 libtorch 运行强化学习策略，可从 MuJoCo 仿真迁移到真实机器人
  - 详细说明见 [src/controllers/rl_quadruped_controller/README.md](src/controllers/rl_quadruped_controller/README.md)

- Gazebo Quadruped Hardware & Playground
  - Gazebo Harmonic ros2-control 插件：见 [src/hardwares/gz_quadruped_hardware](src/hardwares/gz_quadruped_hardware)
  - Gazebo Playground：见 [src/libraries/gz_quadruped_playground](src/libraries/gz_quadruped_playground)

- Robot Descriptions
  - 各机器人 URDF/SRDF：见 [src/descriptions](src/descriptions) 及其子目录 README

---

## 6. 真实 Unitree Go2 机器人

- 已支持真实 Go2 机器人控制：
  - 含硬件接口、控制器与部署示例
  - 具体使用方式可参考 go2_description 与相关控制器 README

演示视频（Real Unitree Go2 Robot）：

[![](http://i0.hdslb.com/bfs/archive/7d3856b3c5e5040f24990d3eab760cf8ba4cf80d.jpg)](https://www.bilibili.com/video/BV1QpZaY8EYV/)

---

## 7. Roadmap / Todo

- [x] **[2025-02-23]** Gazebo Playground
  - [x] OCS2 controller for Gazebo Simulation
  - [x] Refactor FSM and Unitree Guide Controller
- [x] **[2025-03-30]** Real Go2 Robot Support
- [ ] OCS2 Perceptive Locomotion Demo

---

## 8. 进一步探索

- 更多机器人模型
  - 见 [src/descriptions](src/descriptions)

- 更多控制器
  - OCS2 Quadruped Controller：见 [src/controllers/ocs2_quadruped_controller](src/controllers/ocs2_quadruped_controller)
  - RL Quadruped Controller：见 [src/controllers/rl_quadruped_controller](src/controllers/rl_quadruped_controller)

- 传感器与环境仿真
  - Gazebo Quadruped Playground：见 [src/libraries/gz_quadruped_playground](src/libraries/gz_quadruped_playground)

- 自定义机器人 mybot
  - 见 [mybot/README.md](mybot/README.md)

---

## 9. 参考文献与相关项目

### Conference Paper

[1] Liao, Qiayuan, et al. "Walking in narrow spaces: Safety-critical locomotion control for quadrupedal robots with duality-based optimization." In *2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2723–2730. IEEE, 2023.

### Miscellaneous

[1] Unitree Robotics. *unitree_guide: An open source project for controlling the quadruped robot of Unitree Robotics, and it is also the software project accompanying 《四足机器人控制算法--建模、控制与实践》 published by Unitree Robotics*. Online: https://github.com/unitreerobotics/unitree_guide

[2] Qiayuan Liao. *legged_control: An open-source NMPC, WBC, state estimation, and sim2real framework for legged robots*. Online: https://github.com/qiayuanl/legged_control

[3] Ziqi Fan. *rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm*. 2024. Online: https://github.com/fan-ziqi/rl_sar