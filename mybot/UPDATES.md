# mybot MuJoCo 仿真更新说明

## 概述
基于 `mybot_description` 的 URDF 文件，对 MuJoCo 仿真模型进行了重大更新和改进。

## 主要改进

### 1. 网格文件升级
- **旧版本**：使用分割的多个 OBJ 文件（如 base_0.obj, base_1.obj 等）
- **新版本**：使用完整的 STL 文件，提供更准确的几何表示
  - body.STL（机身）
  - 每个关节配置单独的 STL 文件（FL/FR/RL/RR_hip/thigh/calf/foot）

### 2. 机器人参数更新

#### 机身 (base_link)
- **质量**：5.3036 kg（之前：6.921 kg）
- **惯性张量**：根据 URDF 数据调整
  - ixx: 0.012634
  - iyy: 0.017259
  - izz: 0.017164
- **重心位置**：(-0.075044, 0.0013153, 0.023062)

#### 关节配置范围

| 关节类型 | 旧范围 | 新范围 |
|--------|-------|-------|
| 外展关节 (abduction) | -1.0472 ~ 1.0472 | -0.48 ~ 0.48 |
| 前腿髋关节 (front_hip) | -1.5708 ~ 3.4907 | -1.44 ~ 1.44 |
| 后腿髋关节 (back_hip) | -0.5236 ~ 4.5379 | -1.48 ~ 1.48 |
| 膝关节 (knee) | -2.7227 ~ -0.83776 | -2.7 ~ -0.6 |

#### 电机控制范围
- **旧值**：-23.7 ~ 23.7 N⋅m（膝关节：-45.43 ~ 45.43）
- **新值**：统一为 -33.5 ~ 33.5 N⋅m（符合 mybot_description）

#### 各腿关节质量和惯性数据
所有腿部关节（hip、thigh、calf、foot）的质量和惯性张量已从 URDF 更新：
- **髋关节 (hip)**：0.63494 ~ 0.6138 kg
- **大腿 (thigh)**：1.107 ~ 1.107021 kg  
- **小腿 (calf)**：0.292 ~ 0.292240 kg
- **脚 (foot)**：0.0335 kg

#### 腿部位置更新
优化了所有腿部关节的位置，提高了仿真的准确性：

**前腿 (Front)**
- FL_hip: (0.0946, 0.062, -0.071)
- FR_hip: (0.0946, -0.062, -0.071)

**后腿 (Rear)**
- RL_hip: (-0.2146, 0.062, -0.071)
- RR_hip: (-0.2146, -0.062, -0.071)

### 3. 碰撞和接触参数优化
- **脚部摩擦系数**：从 0.4 更新为 0.6
- **整体摩擦系数**：从 0.4 更新为 0.6

### 4. 关键帧初始化
更新了默认站立姿态的关节角度，使其与 URDF 定义的初始状态更一致。

## 文件结构
```
mybot/
├── mybot.xml           # 更新后的 MuJoCo 模型定义
├── scene.xml           # 场景文件（包含 mybot.xml）
├── assets/             # 网格文件目录
│   ├── body.STL        # 新增
│   ├── FL_hip.STL      # 新增
│   ├── FL_thigh.STL    # 新增
│   ├── FL_calf.STL     # 新增
│   ├── FL_foot.STL     # 新增
│   ├── FR_hip.STL      # 新增
│   ├── FR_thigh.STL    # 新增
│   ├── FR_calf.STL     # 新增
│   ├── FR_foot.STL     # 新增
│   ├── RL_hip.STL      # 新增
│   ├── RL_thigh.STL    # 新增
│   ├── RL_calf.STL     # 新增
│   ├── RL_foot.STL     # 新增
│   ├── RR_hip.STL      # 新增
│   ├── RR_thigh.STL    # 新增
│   ├── RR_calf.STL     # 新增
│   ├── RR_foot.STL     # 新增
│   └── [旧 OBJ 文件]   # 保留以向后兼容
└── README.md
```

## 兼容性说明
- 旧的 OBJ 文件仍保留在 assets 目录中，确保向后兼容
- MuJoCo XML 结构保持不变，仅更新参数和网格引用
- 与 ROS2 控制系统的兼容性得到改进

## 建议后续步骤
1. 在 MuJoCo 仿真器中测试更新后的模型
2. 验证关节范围和运动学约束
3. 如需进一步微调，可参考 mybot_description/urdf/robot.urdf

## 参考资源
- URDF 定义：`mybot_description/urdf/robot.urdf`
- 网格文件来源：`mybot_description/meshes/`
