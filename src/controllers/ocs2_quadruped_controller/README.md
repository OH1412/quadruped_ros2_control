# OCS2 Quadruped Controller

本包是 quadruped_ros2_control 工程中的 NMPC 四足控制器，基于 [legged_control](https://github.com/qiayuanl/legged_control) 与 [ocs2_ros2](https://github.com/legubiao/ocs2_ros2)，提供鲁棒的模型预测控制并支持多种商用 / 自研四足平台。

测试环境：

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

更新记录：

* [x] **[2025-01-16]** 增加对地面真值估计器（ground truth estimator）的支持
* [x] **[2025-03-15]** 控制器支持在被动模式与 MPC 模式之间切换

[![](http://i0.hdslb.com/bfs/archive/e758ce019587032449a153cf897a543443b64bba.jpg)](https://www.bilibili.com/video/BV1UcxieuEmH/)

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
    * IMU 传感器：
        * 线加速度（linear acceleration）
        * 角速度（angular velocity）
        * 姿态四元数（orientation）
    * 足端力传感器（feet force sensor）

## 2. 编译

### 2.1 依赖准备（OCS2 + Pinocchio）

在安装 OCS2 相关包之前，需要先根据官方文档安装 [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)。**不要使用 rosdep 安装的 pinocchio 版本。**

安装好 Pinocchio 后，在工作空间中克隆并安装 ocs2_ros2：

```bash
cd ~/ros2_ws/src
git clone https://github.com/legubiao/ocs2_ros2

cd ocs2_ros2
git submodule update --init --recursive

cd ..
rosdep install --from-paths src --ignore-src -r -y
```

### 2.2 编译 OCS2 四足控制器

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_quadruped_controller --symlink-install
```

## 3. 启动与使用

### 3.1 支持的机器人描述包

* Unitree
    * go2_description
    * go1_description
    * a1_description
    * aliengo_description
    * b2_description
* Xiaomi
    * cyberdog_description
* DeepRobotics
    * lite3_description
    * x30_description
* Anybotics
    * anymal_c_description

### 3.2 关于 OCS2 生成的共享库

OCS2 四足控制器依赖基于 C++ 自动微分的共享库。首次启动控制器时，会自动编译相应的 OCS2 模型并生成 `.so` 文件，你可能会在终端看到类似输出：

```text
[gazebo-5] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_libcppadcg_tmp-27918274.so
[gazebo-5] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_libcppadcg_tmp-27918274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_lib.so
[gazebo-5] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_libcppadcg_tmp-94918274.so
[gazebo-5] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_libcppadcg_tmp-94918274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_lib.so
[gazebo-5] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_libcppadcg_tmp-83618274.so
[gazebo-5] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_libcppadcg_tmp-83618274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_lib.so
```

这一步可能需要几分钟时间，完成后重启控制器，机器人即可正常站立。

共享库输出路径由机器人描述包下 `config/ocs2` 目录中的 `task.info` 文件配置，其中 `modelFolderCppAd` 字段若不是以 `/` 开头，则会被视为“相对于当前用户 Home 目录的相对路径”。

### 3.3 键盘模式切换

* 键盘 `1`：被动模式（Passive Mode）
* 键盘 `2`：OCS2 MPC 模式
    * 再次按 `2`：站立（stance）
    * 键盘 `3`：小跑（trot）
    * 键盘 `4`：站立小跑（standing_trot）
    * 键盘 `5`：飞行小跑（flying_trot）

### 3.4 启动控制器

#### MuJoCo 仿真

> 提示：在启动控制器前，需要先根据 https://github.com/legubiao/unitree_mujoco 启动 Unitree MuJoCo C++ 仿真。

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=go2_description
```

#### Gazebo 仿真

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_quadruped_controller gazebo.launch.py pkg_description:=go2_description
```