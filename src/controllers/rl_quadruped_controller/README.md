# RL Quadruped Controller

本包是 quadruped_ros2_control 工程中的强化学习四足控制器，负责加载并运行基于 libtorch 的策略网络，实现从仿真到实机的 RL 控制。

[![](http://i0.hdslb.com/bfs/archive/9886e7f9ed06d7f880b5614cb2f4c3ec1d7bf85f.jpg)](https://www.bilibili.com/video/BV1QP1pYBE47/)

测试环境：

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

## 2. 编译

### 2.1 安装 libtorch

可以根据需要选择带 CUDA 的 libtorch 版本，但必须选择 **C++11 ABI** 版本。libtorch 的安装位置可以自定义，只需在 `~/.bashrc` 中正确配置环境变量。

```bash
cd ~/CLionProjects/
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
```

```bash
cd ~
rm -rf libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
echo 'export Torch_DIR=~/libtorch' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib' >> ~/.bashrc
```

### 2.2 编译控制器

```bash
cd ~/ros2_ws
colcon build --packages-up-to rl_quadruped_controller --symlink-install
```

## 3. 启动

### 3.1 MuJoCo 仿真

> 提示：在启动本控制器前，需要先根据 https://github.com/legubiao/unitree_mujoco 启动 Unitree MuJoCo C++ 仿真。

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rl_quadruped_controller mujoco.launch.py pkg_description:=go2_description
```

### 3.2 Gazebo Classic 11（ROS2 Humble）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rl_quadruped_controller gazebo_classic.launch.py pkg_description:=a1_description
```

### 3.3 Gazebo Harmonic（ROS2 Jazzy）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rl_quadruped_controller gazebo.launch.py pkg_description:=go2_description
```