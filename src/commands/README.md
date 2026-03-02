# Control Command Inputs

本目录包含 quadruped_ros2_control 工程中的所有控制命令输入节点（键盘、手柄、无线遥控等），为各类控制器提供统一的控制指令来源。

子包概览：

- keyboard_input：从键盘读取输入并发布 control_input_msgs/Input 消息
- joystick_input：从有线/无线手柄读取输入并发布 control_input_msgs/Input 消息
- unitree_joystick_input：从 Unitree 官方遥控器读取输入，并桥接为 ROS 2 控制器可用的消息

这些命令输入节点通常与以下控制器配合使用：

- unitree_guide_controller
- ocs2_quadruped_controller
- rl_quadruped_controller