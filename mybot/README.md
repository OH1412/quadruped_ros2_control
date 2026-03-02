# MyBot 机器人模型

mybot 是 quadruped_ros2_control 工程中基于 Unitree Go2 的自定义 MuJoCo 机器人模型，用于在 Unitree MuJoCo 仿真器中进行高精度动力学仿真和控制算法验证。更详细的参数和接口说明见 QUICK_REFERENCE.md 与 INTERFACE_ANALYSIS.md。

## 文件结构

- `mybot.xml` - 机器人模型定义（骨骼、关节、传感器、执行器等 MuJoCo 元素）
- `scene.xml` - 场景文件（引入 mybot.xml 并定义地面、光源等环境）
- `assets/` - 3D 网格文件（.stl / .obj 格式，用于机身与各关节外形）

## 如何使用

### Python版本

编辑 `simulate_python/config.py`：
```python
ROBOT = "mybot"  # 改为 "mybot"
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml"
```

### C++版本

编辑 `simulate/config.yaml`：
```yaml
robot: "mybot"  # 改为 "mybot"
robot_scene: "scene.xml"
```

然后运行模拟器即可加载mybot模型。

## 修改模型

如果你需要修改mybot的外形、质量、关节范围等，编辑 `mybot.xml` 文件。

主要修改点包括：
- `<default class="mybot">` - 机器人默认参数
- `<inertial>` - 质量和惯性参数
- `<geom>` - 几何和碰撞参数
- `<joint>` - 关节范围和参数
- `<motor>` - 电机控制范围

如果需要修改3D外观，编辑或替换 `assets/` 文件夹中的 .obj 文件。
