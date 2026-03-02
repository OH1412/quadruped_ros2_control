//
// Created by tlab-uav on 25-2-27.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H

// CtrlComponent 是 Unitree Guide 控制器内部各子模块的聚合体，
// 主要被各个 FSMState 持有引用，用来访问：
//   - robot_model_    ：四足机器人动力学/运动学模型（QuadrupedRobot），
//   - estimator_      ：状态估计器（基于 IMU / 关节测量 / 足端位置等），
//   - balance_ctrl_   ：质心 + 姿态的平衡控制器，根据期望加速度/角速度算足端支撑力，
//   - wave_generator_ ：步态相位生成器（决定哪条腿支撑、哪条腿摆动）。
//
// UnitreeGuideController 在 on_configure/on_activate 中创建并填充这些指针，
// 然后在不同的 FSM 状态（如 StateTrotting、StateFreeStand）中通过 CtrlComponent
// 来访问这些公共资源，实现“多个状态共享一套模型/估计/控制”的结构。

#include <unitree_guide_controller/gait/WaveGenerator.h>

#include "BalanceCtrl.h"
#include "Estimator.h"

struct CtrlComponent {
    // 机器人模型：提供前向/逆向运动学、力矩求解等接口
    std::shared_ptr<QuadrupedRobot> robot_model_;
    // 状态估计器：根据传感器数据估计机体位姿、速度、足端位置等
    std::shared_ptr<Estimator> estimator_;
    // 平衡控制器：根据期望质心加速度、姿态变化，求解足端支撑力
    std::shared_ptr<BalanceCtrl> balance_ctrl_;
    // 步态相位生成器：管理触地/摆动序列和相位（WAVE_ALL/STANCE_ALL/SWING_ALL）
    std::shared_ptr<WaveGenerator> wave_generator_;

    CtrlComponent() = default;
};
#endif //CTRLCOMPONENT_H
