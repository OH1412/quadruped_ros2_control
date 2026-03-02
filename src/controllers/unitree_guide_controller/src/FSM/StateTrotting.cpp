//
// Created by tlab-uav on 24-9-18.
//

#include "unitree_guide_controller/FSM/StateTrotting.h"

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>

// Trotting 状态：
//   - 通过 getUserCmd() 将手柄/键盘输入映射为机体目标线速度 v_cmd_body_ 和偏航角速度 d_yaw_cmd_；
//   - 通过 calcCmd() 积分/限幅得到期望机体位置 pcd_ 和速度 vel_target_（全局系），以及目标偏航角 yaw_cmd_；
//   - 利用 gait_generator_ 根据目标速度和 yaw_cmd_ 生成足端摆动/支撑轨迹；
//   - 在 calcTau() 中调用 BalanceCtrl 计算支撑腿所需的足端力，再通过 QuadrupedRobot 反算关节力矩；
//   - 在 calcQQd() 中根据目标足端轨迹求解目标关节角/角速度 q_goal/qd_goal；
//   - 在 calcGain() 中根据当前腿是否支撑/摆动设置对应的 PD 增益；
//   - 最终通过 ctrl_interfaces_.joint_*_command_interface_ 写回 ros2_control。
StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component) : FSMState(FSMStateName::TROTTING, "trotting",
                                                                       ctrl_interfaces),
                                                              estimator_(ctrl_component.estimator_),
                                                              robot_model_(ctrl_component.robot_model_),
                                                              balance_ctrl_(ctrl_component.balance_ctrl_),
                                                              wave_generator_(ctrl_component.wave_generator_),
                                                              gait_generator_(ctrl_component) {
    // 步态高度（足端摆动轨迹在垂直方向的最大高度）
    gait_height_ = 0.08;
    // 质心位置/速度 PD 增益
    Kpp = Vec3(70, 70, 70).asDiagonal();
    Kdp = Vec3(10, 10, 10).asDiagonal();
    // 机体姿态（角度） PD 增益
    kp_w_ = 780;
    Kd_w_ = Vec3(70, 70, 70).asDiagonal();
    // 摆动腿足端轨迹的 PD 增益
    Kp_swing_ = Vec3(400, 400, 400).asDiagonal();
    Kd_swing_ = Vec3(10, 10, 10).asDiagonal();

    // 用户指令映射到线速度/角速度时的限幅范围
    v_x_limit_ << -0.4, 0.4;
    v_y_limit_ << -0.3, 0.3;
    w_yaw_limit_ << -0.5, 0.5;
    // 控制周期 dt
    dt_ = 1.0 / ctrl_interfaces_.frequency_;
}

// 进入 trotting 状态：
//   - 以当前机体位置/偏航角为参考初始化 pcd_、yaw_cmd_；
//   - 清零目标速度，重启 gait_generator_；
//   - 将 control_inputs_.command 清零，避免立即切回其它状态。
void StateTrotting::enter() {
    pcd_ = estimator_->getPosition();
    // 机体高度设为“机体到前腿足端的 z 距离”对应的站立高度
    pcd_(2) = -estimator_->getFeetPos2Body()(2, 0);
    v_cmd_body_.setZero();
    yaw_cmd_ = estimator_->getYaw();
    Rd = rotz(yaw_cmd_);
    w_cmd_global_.setZero();

    ctrl_interfaces_.control_inputs_.command = 0;
    gait_generator_.restart();
}

// 每个控制周期：更新估计状态、读取用户指令、生成足端轨迹、求解力矩/关节目标，并设置步态相位。
void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    // 当前机体位置/速度（全局系）
    pos_body_ = estimator_->getPosition();
    vel_body_ = estimator_->getVelocity();

    // 机体旋转 B2G（Body 到 Global）以及其转置 G2B
    B2G_RotMat = estimator_->getRotation();
    G2B_RotMat = B2G_RotMat.transpose();

    // 1) 根据用户输入得到机体目标速度/偏航角速度
    getUserCmd();
    // 2) 根据当前速度和目标速度积分/限幅得到目标位置 pcd_ 和 vel_target_
    calcCmd();

    // 3) 根据目标速度与偏航角速度设置步态（相位+接触模式），生成足端期望轨迹
    gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
    gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);

    // 4) 计算支撑腿所需足端力 -> 关节力矩；摆动腿使用足端 PD 轨迹追踪
    calcTau();
    // 5) 根据足端目标位置/速度反算关节位置/速度目标
    calcQQd();

    // 6) 根据是否需要“迈步”决定 wave_generator_ 的整体状态（全支撑/全摆动）
    if (checkStepOrNot()) {
        wave_generator_->status_ = WaveStatus::WAVE_ALL;
    } else {
        wave_generator_->status_ = WaveStatus::STANCE_ALL;
    }

    // 7) 设置每个关节对应的 PD 增益（摆动腿/支撑腿不同）
    calcGain();
}

void StateTrotting::exit() {
    // 退出时将所有腿状态设为 SWING_ALL，便于安全过渡
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

// 根据 control_inputs_.command 判断是否切换到其它状态
FSMStateName StateTrotting::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::TROTTING;
    }
}

// 将摇杆输入映射为机体坐标系下的目标线速度 v_cmd_body_ 和偏航角速度 d_yaw_cmd_
void StateTrotting::getUserCmd() {
    /* Movement */
    v_cmd_body_(0) = invNormalize(ctrl_interfaces_.control_inputs_.ly, v_x_limit_(0), v_x_limit_(1));  // 反转前进方向
    v_cmd_body_(1) = -invNormalize(ctrl_interfaces_.control_inputs_.lx, v_y_limit_(0), v_y_limit_(1));  // 反转左右方向
    v_cmd_body_(2) = 0;

    /* Turning */
    d_yaw_cmd_ = -invNormalize(ctrl_interfaces_.control_inputs_.rx, w_yaw_limit_(0), w_yaw_limit_(1));
    // 一阶低通滤波，避免转向指令过于突变
    d_yaw_cmd_ = 0.9 * d_yaw_cmd_past_ + (1 - 0.9) * d_yaw_cmd_;
    d_yaw_cmd_past_ = d_yaw_cmd_;
}

// 根据当前状态与期望速度/偏航角，更新目标机体位置 pcd_ 和速度 vel_target_
void StateTrotting::calcCmd() {
    /* Movement */
    // 将机体坐标系下的期望速度变换到世界坐标
    vel_target_ = B2G_RotMat * v_cmd_body_;

    // 对目标速度做限幅，防止一步内变化过大
    vel_target_(0) =
            saturation(vel_target_(0), Vec2(vel_body_(0) - 0.2, vel_body_(0) + 0.2));
    vel_target_(1) =
            saturation(vel_target_(1), Vec2(vel_body_(1) - 0.2, vel_body_(1) + 0.2));

    // 目标位置 pcd_ 由上一时刻积分得到，并在当前位置附近做限幅
    pcd_(0) = saturation(pcd_(0) + vel_target_(0) * dt_,
                         Vec2(pos_body_(0) - 0.05, pos_body_(0) + 0.05));
    pcd_(1) = saturation(pcd_(1) + vel_target_(1) * dt_,
                         Vec2(pos_body_(1) - 0.05, pos_body_(1) + 0.05));

    vel_target_(2) = 0;

    /* Turning */
    // 积分偏航角速度，得到目标偏航角 yaw_cmd_
    yaw_cmd_ = yaw_cmd_ + d_yaw_cmd_ * dt_;
    Rd = rotz(yaw_cmd_);
    // 只使用 z 轴的角速度作为机体期望朝向的控制目标
    w_cmd_global_(2) = d_yaw_cmd_;
}

// 计算机体级别的 PD 控制输出 -> 足端支撑力 -> 关节力矩
void StateTrotting::calcTau() {
    // 机体位置/速度误差
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;

    // 期望质心线加速度 dd_pcd 和角速度变化 d_wbd
    Vec3 dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    Vec3 d_wbd = kp_w_ * rotMatToExp(Rd * G2B_RotMat) +
                 Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());

    // 对线加速度与角速度变化做限幅，避免过大控制量
    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3, 3));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3, 3));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-5, 5));

    d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-40, 40));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));

    const Vec34 pos_feet_body_global = estimator_->getFeetPos2Body();
    // BalanceCtrl 根据期望质心加速度/角速度与足端在机体下的位置，
    // 求解每条支撑腿所需的足端力（全局系），其中 contact_ 决定哪些腿在支撑
    Vec34 force_feet_global =
            -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet_body_global, wave_generator_->contact_);


    Vec34 pos_feet_global = estimator_->getFeetPos();
    Vec34 vel_feet_global = estimator_->getFeetVel();

    // 对于摆动腿，用足端 PD 让足端跟踪 gait_generator_ 给出的目标轨迹
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            force_feet_global.col(i) =
                    Kp_swing_ * (pos_feet_global_goal_.col(i) - pos_feet_global.col(i)) +
                    Kd_swing_ * (vel_feet_global_goal_.col(i) - vel_feet_global.col(i));
        }
    }

    // 将足端力从全局系转换到机体系
    Vec34 force_feet_body_ = G2B_RotMat * force_feet_global;

    std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    // 逐腿调用 QuadrupedRobot 的 getTorque()，将足端力映射成关节力矩，并写回 command 接口
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_body_.col(i), i);
        for (int j = 0; j < 3; j++) {
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }
}

// 根据足端目标位置/速度，反解得到各关节的期望角度和角速度，并写入 command 接口
void StateTrotting::calcQQd() {
    const std::vector<KDL::Frame> pos_feet_body = robot_model_->getFeet2BPositions();

    Vec34 pos_feet_target, vel_feet_target;
    for (int i(0); i < 4; ++i) {
        pos_feet_target.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_);
        vel_feet_target.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_);
    }

    Vec12 q_goal = robot_model_->getQ(pos_feet_target);
    Vec12 qd_goal = robot_model_->getQd(pos_feet_body, vel_feet_target);
    for (int i = 0; i < 12; i++) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
}

// 根据当前腿是否支撑/摆动，给每个关节设置不同的关节空间 PD 增益
void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            // swing gain（摆动腿增益较大，追踪足端轨迹）
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(3);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2);
            }
        } else {
            // stable gain（支撑腿增益较小，避免刚度过大影响接触）
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(0.8);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(0.8);
            }
        }
    }
}

// 判断是否需要迈步（更新 gait_generator 状态），主要依据：
//   - 用户给的机体目标速度是否足够大；
//   - 机体位置/速度误差是否超过某些阈值；
//   - 偏航角速度命令是否足够大。
bool StateTrotting::checkStepOrNot() {
    if (fabs(v_cmd_body_(0)) > 0.03 || fabs(v_cmd_body_(1)) > 0.03 ||
        fabs(pos_error_(0)) > 0.08 || fabs(pos_error_(1)) > 0.08 ||
        fabs(vel_error_(0)) > 0.05 || fabs(vel_error_(1)) > 0.05 ||
        fabs(d_yaw_cmd_) > 0.20) {
        return true;
    }
    return false;
}
