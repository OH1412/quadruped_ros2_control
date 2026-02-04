# mybot 快速参考指南

## 主要改进总结

### ✅ 已完成
1. **STL 网格文件集成**
   - 从 mybot_description 复制了 17 个 STL 文件到 assets/
   - 单个文件表示每个零件，几何精度更高

2. **物理参数更新**
   - 机身质量从 6.921 kg → 5.3036 kg
   - 所有关节的质量和惯性数据已从 URDF 同步
   - 摩擦系数优化：0.4 → 0.6

3. **运动学约束优化**
   - 外展关节范围：-1.0472 ~ 1.0472 → **-0.48 ~ 0.48 rad**
   - 前腿髋关节范围：-1.5708 ~ 3.4907 → **-1.44 ~ 1.44 rad**
   - 后腿髋关节范围：-0.5236 ~ 4.5379 → **-1.48 ~ 1.48 rad**
   - 膝关节范围：-2.7227 ~ -0.83776 → **-2.7 ~ -0.6 rad**

4. **电机参数统一**
   - 所有关节统一为 -33.5 ~ 33.5 N⋅m 的控制范围

5. **足端位置精确化**
   - 所有四个脚的位置坐标已根据 URDF 数据调整
   - 改善了步态仿真的准确性

## 文件对照

| 部件 | 旧方案（OBJ） | 新方案（STL） |
|------|-------------|-------------|
| 机身 | base_0/1/2/3/4.obj | body.STL |
| 髋关节 | hip_0/1.obj | FL/FR/RL/RR_hip.STL |
| 大腿 | thigh_0/1/mirror_0/1.obj | FL/FR/RL/RR_thigh.STL |
| 小腿 | calf_0/1/mirror_0/1.obj | FL/FR/RL/RR_calf.STL |
| 脚 | foot.obj | FL/FR/RL/RR_foot.STL |

## 使用方式

### 在 MuJoCo 仿真器中加载
```bash
# 使用 mjvisualize（如果安装了 MuJoCo）
mjvisualize /path/to/unitree_mujoco/unitree_robots/mybot/scene.xml

# 或者在 Python 中
import mujoco
model = mujoco.MjModel.from_xml_path('unitree_robots/mybot/mybot.xml')
```

### Python 控制示例
```python
import mujoco
import mujoco.viewer

# 加载模型和数据
model = mujoco.MjModel.from_xml_path('unitree_robots/mybot/mybot.xml')
data = mujoco.MjData(model)

# 获取关节索引
joint_names = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
]

# 设置关节角度（弧度）
for i, name in enumerate(joint_names):
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    data.ctrl[i] = 0.0  # 设置目标角度

# 运行仿真
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

## 关键参数对照表

### 腿部关节默认姿态
```
qpos: [0 0 0.1 1 0 0 0  -0.48  -0.72  -1.65  -0.15   0.72  -1.65  0.5   0.72  -1.65  -0.5   0.72  -1.65]
      ↑ x y z 姿态(四元数)    ↑FL     ↑FR      ↑RL      ↑RR
```

### 关节编号
```
0-2:   基座平移 (x, y, z)
3-6:   基座旋转 (四元数)
7-9:   前左腿 (hip, thigh, calf)
10-12: 前右腿 (hip, thigh, calf)
13-15: 后左腿 (hip, thigh, calf)
16-18: 后右腿 (hip, thigh, calf)
```

## 故障排除

### 问题：网格文件找不到
**解决方案**：确保 assets/ 目录包含所有 STL 文件，并且 mybot.xml 中的 meshdir 设置正确
```xml
<compiler angle="radian" meshdir="assets" autolimits="true" />
```

### 问题：关节限制错误
**解决方案**：检查 mybot.xml 中的默认类定义和关节范围设置

### 问题：碰撞穿模
**解决方案**：验证碰撞几何体定义，特别是足端接触点的位置

## 版本信息
- **mybot_description 版本**：基于 ROS2 四足机器人控制框架
- **MuJoCo 格式**：兼容 MuJoCo 2.x
- **更新日期**：2026-02-04

## 相关链接
- MuJoCo 官网：https://mujoco.org
- 原始 URDF：`mybot_description/urdf/robot.urdf`
- 网格数据来源：`mybot_description/meshes/`
