# MuJoCo 与 ROS2 控制器接口分析

## 通信架构概览

```
ROS2 控制器        DDS 网络           MuJoCo 仿真器
    ↓              (LCM/Fast-DDS)        ↓
 LowCmd 发送 ←→ unitree_sdk2_bridge.h ←→ 关节控制
 LowState 接收                          传感器反馈
```

## 详细通信流程

### 1. 控制命令传输 (Controller → Simulator)

**路径**：ROS2 Controller → DDS Message → unitree_sdk2_bridge.h → MuJoCo Control

```cpp
// unitree_sdk2_bridge.h - run() 方法中的关键代码
void run() {
    // 这是 1000Hz 的控制线程回调
    {
        std::lock_guard<std::mutex> lock(lowcmd->mutex_);
        for (int i(0); i < num_motor_; i++) {
            auto& m = lowcmd->msg_.motor_cmd()[i];  // 接收 ROS2 控制命令
            
            // 控制算法：应用 PD 控制
            mj_data_->ctrl[i] = m.tau()                    // 前馈力矩
                              + m.kp() * (m.q() - mj_data_->sensordata[i])
                              + m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }
}
```

**关键参数映射**：
- `m.tau()`：目标力矩 (τ_target)
- `m.q()`：目标位置 (rad)
- `m.dq()`：目标速度 (rad/s)
- `m.kp()`：位置增益
- `m.kd()`：速度增益

### 2. 反馈信号传输 (Simulator → Controller)

**路径**：MuJoCo 传感器 → unitree_sdk2_bridge.h → DDS Message → ROS2 Controller

```cpp
// 低层状态反馈 (12个电机)
for (int i(0); i < num_motor_; i++) {
    lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];                    // 位置
    lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];     // 速度
    lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2*num_motor_]; // 力矩
}

// IMU 反馈 (如果存在)
lowstate->msg_.imu_state().quaternion()[0-3] = 姿态四元数
lowstate->msg_.imu_state().rpy() = 欧拉角(计算得出)
lowstate->msg_.imu_state().gyroscope()[0-2] = 角速度
lowstate->msg_.imu_state().accelerometer()[0-2] = 加速度

// 脚端接触力反馈 (Go2 特有)
lowstate->msg_.foot_force()[0-3] = 四个脚的接触力
```

## 关键接口映射

### 控制命令接收
```
LowCmd Message (ROS2 → MuJoCo)
├─ motor_cmd[12]
│  ├─ tau(): 力矩命令 (N⋅m)
│  ├─ q(): 位置命令 (rad)
│  ├─ dq(): 速度命令 (rad/s)  
│  ├─ kp(): 位置增益
│  └─ kd(): 速度增益
└─ wireless_controller (遥控器输入)
```

### 状态反馈发送
```
LowState Message (MuJoCo → ROS2)
├─ motor_state[12]
│  ├─ q(): 关节角度 (rad)
│  ├─ dq(): 关节速度 (rad/s)
│  └─ tau_est(): 估计的力矩 (N⋅m)
├─ imu_state
│  ├─ quaternion: 姿态
│  ├─ rpy: 欧拉角
│  ├─ gyroscope: 角速度
│  └─ accelerometer: 线加速度
├─ foot_force[4] (Go2)
└─ joystick (遥控器状态)
```

## 我的修改是否会出问题？

### ✅ **安全** - 不会造成接口破损

**原因分析**：

1. **关节编号不变**
   - 修改前后都是 12 个电机 (4腿 × 3关节)
   - 关节顺序完全相同：FL_hip/thigh/calf, FR_hip/thigh/calf, RL_hip/thigh/calf, RR_hip/thigh/calf
   - DDS 消息中的 motor_cmd[12] 和 motor_state[12] 索引完全相同

2. **传感器编号不变**
   - `sensordata[]` 数组的关节位置/速度/力矩索引不变
   - IMU 传感器索引不变：`dim_motor_sensor_ + 0~9`
   - 脚部接触力索引不变：`dim_motor_sensor_ + 16~19`
   - 框架位置索引不变：`dim_motor_sensor_ + 10~15`

3. **控制算法兼容**
   ```cpp
   // 这个公式对所有关节参数变化都通用
   mj_data_->ctrl[i] = m.tau()                              // 不依赖关节参数
                      + m.kp() * (m.q() - 当前位置)       // 只看位置差
                      + m.kd() * (m.dq() - 当前速度);     // 只看速度差
   ```

### ⚠️ **需要注意的问题**

#### 问题1: 关节限制变化
- **旧值**：膝关节 -2.7227 ~ -0.83776 rad
- **新值**：膝关节 -2.7 ~ -0.6 rad
- **影响**：更严格的范围限制，MuJoCo 会自动硬限制超出范围的命令
- **ROS2 端需要验证**：所有目标位置命令必须在新范围内

**检查方法**：
```bash
# 检查控制器发送的命令范围是否超出新限制
# 膝关节旧范围：-2.7227 ~ -0.83776
# 膝关节新范围：-2.7 ~ -0.6
# 外展关节旧范围：-1.0472 ~ 1.0472
# 外展关节新范围：-0.48 ~ 0.48
```

#### 问题2: 质量和惯性改变
- **影响**：重力补偿、动力学计算会改变
- **表现**：机器人的自然摆摆动周期会改变
- **ROS2 端不需要改动**：只影响仿真，不影响 DDS 接口

#### 问题3: 脚部位置变化
```
旧的脚部位置：
- FL_foot pos="0 0 -0.213"

新的脚部位置：
- FL_foot pos="0.0015191 -0.00024999 -0.22467"
```
- **影响**：接触点位置，影响步态仿真精度
- **ROS2 端不需要改动**：只影响仿真中的触地检测

## 兼容性验证清单

### ✅ 必须验证的项目

```
[?] 1. 运动范围验证
    检查 ROS2 控制器是否发送超出新范围的命令
    - 外展关节：需在 [-0.48, 0.48] rad
    - 膝关节：需在 [-2.7, -0.6] rad
    
[?] 2. 控制器性能测试
    a) 直接位置控制 - 验证新关节范围下的收敛性
    b) 力矩控制 - 验证新电机参数下的响应
    c) 步态生成 - 验证在新身体参数下的稳定性
    
[?] 3. 传感器反馈验证
    a) 关节位置、速度、力矩是否正确反馈
    b) IMU 数据是否正确计算
    c) 脚部接触力是否按新位置计算
    
[?] 4. 通信性能测试
    检查 DDS 消息是否能稳定以 1000Hz 传输
```

### 启动命令示例

```bash
# 启动 MuJoCo 仿真器（使用更新后的 mybot）
cd /home/rc_kfs/unitree_mujoco/simulate
./unitree_mujoco -i 1 -n lo -r mybot -s ../unitree_robots/mybot/scene.xml

# 在另一个终端启动 ROS2 控制器
ros2 launch mybot_description gazebo_rl_control.launch.py
```

## 问题诊断指南

### 症状1: 关节命令被硬限制截断
```
症状：控制器发送命令，但机器人不按预期移动
原因：可能发送了超出新范围的命令
解决：检查 ROS2 层的命令发生器，添加范围验证
```

### 症状2: 仿真与实物差异大
```
症状：仿真中步态不稳定，实物正常
原因：新的质量和惯性改变了动力学
解决：调整 ROS2 控制器中的增益系数 (kp, kd, tau)
```

### 症状3: DDS 通信超时
```
症状：仿真器和控制器无法通信
原因：通常不是本修改导致，但可验证：
检查命令：
  ros2 topic list
  ros2 topic hz /rt/lowcmd
  ros2 topic hz /rt/lowstate
```

## 建议的测试顺序

1. **第一步**：启动仿真，观察默认站立姿态是否合理
2. **第二步**：运行简单的循环命令（如膝关节摆动）
3. **第三步**：运行步态控制器，观察 4 条腿的协调性
4. **第四步**：比较实物数据（如有）与仿真数据

## 总结

| 项目 | 旧版本 | 新版本 | 影响 |
|------|--------|--------|------|
| DDS 接口 | 不变 | 不变 | ✅ 兼容 |
| 关节编号 | 12个 | 12个 | ✅ 兼容 |
| 关节顺序 | 相同 | 相同 | ✅ 兼容 |
| 控制算法 | PD+FF | PD+FF | ✅ 兼容 |
| 关节限制 | 宽松 | 严格 | ⚠️ 需验证 |
| 质量参数 | 旧值 | 新值 | ⚠️ 需调参 |
| 仿真精度 | 低 | 高 | ✅ 改进 |

**结论**：不会破坏 DDS 通信接口，但需要验证控制器命令是否在新范围内，以及调整控制增益以适应新的动力学模型。
