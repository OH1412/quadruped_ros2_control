# Robot Descriptions

本目录包含 quadruped_ros2_control 工程中所有支持的机器人描述文件（URDF / SRDF），涵盖多款商用与自研四足平台，并提供从 URDF 到 MuJoCo / Gazebo 模型的转换流程。

已支持的机器人品牌与机型：

* Unitree
  * [Go1](unitree/go1_description/)
  * [Go2](unitree/go2_description/)
  * [A1](unitree/a1_description/)
  * [Aliengo](unitree/aliengo_description/)
  * [B2](unitree/b2_description/)
* Xiaomi
  * [Cyberdog](xiaomi/cyberdog_description/)
* Deep Robotics
  * [Lite 3](deep_robotics/lite3_description/)
  * [X30](deep_robotics/x30_description/)
* Anybotics
  * [Anymal C](anybotics/anymal_c_description/)

## 1. 从 URDF 转换为 MuJoCo 模型

大致步骤如下：

1. 安装 [MuJoCo](https://github.com/google-deepmind/mujoco)
2. 将网格文件转换为 MuJoCo 支持的格式（如 STL）。
3. 调整 URDF 中的 mesh 引用，使其与转换后的网格文件匹配。
   * 例如从 `.dae` 转为 `.stl` 时，网格缩放比例可能发生变化，需要在 URDF 中同步修改 `scale`。
4. 使用 `xacro` 生成最终 URDF：

   ```bash
   xacro robot.xacro > ../urdf/robot.urdf
   ```

5. 使用 MuJoCo 提供的工具（或命令行 `compile`）将 URDF 转换为 MuJoCo XML 模型：

   ```bash
   compile robot.urdf robot.xml
   ```

## 2. Gazebo 仿真依赖

### 2.1 Gazebo Harmonic（ROS2 Jazzy 主测，兼容 Humble）

Gazebo Harmonic 仿真部分主要在 ROS2 Jazzy 上测试，对 ROS2 Humble 也做了适配（包名略有差异）。

* 安装 Gazebo Harmonic：

  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```

* 安装 Gazebo 的 ros2-control 支持：

  ```bash
  sudo apt-get install ros-jazzy-gz-ros2-control
  ```

* 编译 Leg PD 控制器：

  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to leg_pd_controller
  ```

### 2.2 Gazebo Classic 11（ROS2 Humble）

Gazebo Classic（Gazebo 11）仿真主要在 ROS2 Humble 上测试。

* 安装 Gazebo Classic：

  ```bash
  sudo apt-get install ros-humble-gazebo-ros
  ```

* 安装 Gazebo 的 ros2-control 支持：

  ```bash
  sudo apt-get install ros-humble-gazebo-ros2-control
  ```

* 编译 Leg PD 控制器：

  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to leg_pd_controller
  ```