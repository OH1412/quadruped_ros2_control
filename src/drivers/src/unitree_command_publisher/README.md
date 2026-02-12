# unitree_command_publisher

纯发布 `UnitreeCommand` 的 ROS2 Python 包，不订阅任何话题。

## Build
```bash
cd /home/xzx/unitree/unitree_actuator_sdk/ros2_ws
colcon build --packages-select unitree_command_publisher
source install/setup.bash
```

## Run
```bash
ros2 run unitree_command_publisher unitree_command_publisher
```

## 参数示例
```bash
ros2 run unitree_command_publisher unitree_command_publisher --ros-args \
  -p publish_rate_hz:=50 \
  -p q_des:="[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]" \
  -p dq_des:="[0,0,0,0,0,0,0,0,0,0,0,0]" \
  -p tau_ff:="[0,0,0,0,0,0,0,0,0,0,0,0]" \
  -p kp:=2.0 -p kd:=0.05
```
