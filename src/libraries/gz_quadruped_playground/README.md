# Gazebo Quadruped Playground

本包是 quadruped_ros2_control 工程中的 Gazebo 仿真场景与传感器集合，用于在多种环境（如空场、仓库等）下测试不同控制器、感知与导航算法。

测试环境：

* Ubuntu 24.04（ROS2 Jazzy）

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to gz_quadruped_playground --symlink-install
```

## 启动仿真

* 搭配 Unitree Guide 控制器：

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py
  ```

  使用仓库场景：

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py world:=warehouse
  ```

* 搭配 OCS2 四足控制器：

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py controller:=ocs2
  ```

  使用仓库场景：

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py controller:=ocs2 world:=warehouse
  ```

## SLAM 测试

### 录制 rosbag

```bash
cd ~/ros2_ws
ros2 bag record /rgbd_d435/points /rgbd_d435/depth_image /scan/points /imu_sensor_broadcaster/imu /odom /tf /tf_static /joint_states
```

### Fast LIO

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch gz_quadruped_playground fast_lio.launch.py
```

## 相关资料

* Gazebo 里程计发布插件：
  https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html#details
* Gazebo Intel RealSense D435 RGBD 相机模型：
  https://app.gazebosim.org/OpenRobotics/fuel/models/Intel%20RealSense%20D435