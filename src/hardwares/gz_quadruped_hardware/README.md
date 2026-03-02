# Gazebo Quadruped ROS2 Control Plugin

本包是 quadruped_ros2_control 工程中用于 Gazebo Harmonic 的 ros2-control 插件，基于官方 [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) 进行修改，以更好支持四足机器人仿真。

## 编译

1. 在 Ubuntu 22.04 上安装 Gazebo Harmonic：

  参考官方文档：https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu

2. 安装 ros-gz 集成：

  ```bash
  sudo apt-get install ros-humble-ros-gzharmonic
  ```

3. 在工作空间内编译本插件：

  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to gz_quadruped_hardware --symlink-install
  ```