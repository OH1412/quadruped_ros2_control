//
// Created by tlab-uav on 24-9-6.
//

#ifndef QUADRUPEDCONTROLLER_H
#define QUADRUPEDCONTROLLER_H

// UnitreeGuideController 是本工程中“Unitree Guide 风格”的高层控制器：
//   - 订阅 /control_input（键盘/手柄/Unitree 遥控器）得到用户指令；
//   - 结合 QuadrupedRobot 模型、状态估计器、WaveGenerator（步态相位生成器）；
//   - 通过一组有限状态机 FSMState（站立、下蹲、自由站立、Small tests、trotting 等）
//     生成每条腿的期望足端轨迹与关节目标；
//   - 最终通过 ros2_control 的 command interfaces 写出：
//       关节力矩 torque、期望关节角 q_d / qd_d，以及对应的 PD 增益 kp/kd；
//   - 下游可以直接将这些命令发到硬件，也可以通过 leg_pd_controller 等中间层再做力矩计算。

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_common/FSM/FSMState.h>
#include <controller_common/FSM/StatePassive.h>
#include <controller_common/FSM/StateFixedDown.h>
#include <controller_common/common/enumClass.h>

#include "control/CtrlComponent.h"
#include "FSM/StateBalanceTest.h"
#include "FSM/StateFixedStand.h"
#include "FSM/StateFreeStand.h"
#include "FSM/StateSwingTest.h"
#include "FSM/StateTrotting.h"

namespace unitree_guide_controller {
    // 保存所有可能的 FSM 状态实例，便于在不同模式之间切换
    struct FSMStateList {
        std::shared_ptr<FSMState> invalid;      // 无效状态（占位）
        std::shared_ptr<StatePassive> passive;  // 被动模式：不发力矩
        std::shared_ptr<StateFixedDown> fixedDown;   // 下蹲固定姿态
        std::shared_ptr<StateFixedStand> fixedStand; // 站立固定姿态
        std::shared_ptr<StateFreeStand> freeStand;   // 自由站立（有平衡控制）
        std::shared_ptr<StateTrotting> trotting;     // 小跑步态

        std::shared_ptr<StateSwingTest> swingTest;   // 摆腿测试
        std::shared_ptr<StateBalanceTest> balanceTest; // 平衡测试
    };

    // 顶层 ros2_control 控制器插件，负责：
    //   - 声明/绑定硬件 command & state interfaces；
    //   - 管理 FSM 状态机和 CtrlComponent（机器人模型/估计/步态生成等）；
    //   - 在 update() 中按当前状态调用相应状态的 run()，并根据指令切换状态。
    class UnitreeGuideController final : public controller_interface::ControllerInterface {
    public:
        UnitreeGuideController() = default;

        // 声明需要的 command interfaces，例如：
        //   <command_prefix>/<joint_name>/{effort,position,velocity,kp,kd}
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        // 声明需要的 state interfaces，例如：
        //   <joint_name>/{position,velocity,effort}, <imu_name>/<imu_interface>
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // 控制主循环：
        //   - 调用 CtrlComponent 中的 robot_model / wave_generator / estimator 更新内部状态；
        //   - 再调用当前 FSM 状态的 run() 生成关节命令；
        //   - 依据 current_state_->checkChange() 决定是否切换到下一个状态。
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // 生命周期回调：声明参数、初始化 CtrlComponent 与估计器/步态生成器等
        controller_interface::CallbackReturn on_init() override;

        // 创建话题订阅（/control_input、/robot_description），构造 QuadrupedRobot、WaveGenerator 等
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // 绑定 ros2_control 注入的 command/state interfaces 到 CtrlInterfaces，初始化 FSM 状态机
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        // 聚合结构：
        //   - ctrl_component_：包含机器人模型、估计器、平衡控制器、步态相位生成器等；
        //   - ctrl_interfaces_：保存所有与硬件交互的 loaned interfaces 以及用户指令等共享数据。
        CtrlComponent ctrl_component_;
        CtrlInterfaces ctrl_interfaces_;

    protected:
        // 由参数服务器读取的硬件/传感器配置
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::string imu_name_;        // IMU 传感器名称
        std::string base_name_;       // 机体（base link）名称
        std::string command_prefix_;  // 若非空，则 command 接口前会加上该前缀（用于链式控制）
        std::vector<std::string> imu_interface_types_;
        std::vector<std::string> feet_names_; // 足端 link 名称（FR/FL/RR/RL）

        // FR FL RR RL 对应的默认站立和下蹲关节角（弧度制）
        std::vector<double> stand_pos_ = {
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3
        };

        std::vector<double> down_pos_ = {
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4
        };

        // 用于 fixed stand / down 模式的 PD 增益
        double stand_kp_ = 80.0;
        double stand_kd_ = 3.5;

        // 订阅：
        //   /control_input  —— 来自键盘/手柄/遥控器的高层指令
        //   /robot_description —— 机器人 URDF，用于构造 QuadrupedRobot 模型
        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

        // 将不同接口名映射到 CtrlInterfaces 中对应的 loaned command interface 数组
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        // 当前 FSM 模式、状态名，以及当前/下一状态指针
        FSMMode mode_ = FSMMode::NORMAL;
        std::string state_name_;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;

        // 可选：用于统计控制循环频率
        std::chrono::time_point<std::chrono::steady_clock> last_update_time_;
        double update_frequency_;

        // 根据 FSMStateName 返回对应的状态实例
        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        // 将 state interface 名称映射到 CtrlInterfaces 中对应的状态接口数组
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };
    };
}


#endif //QUADRUPEDCONTROLLER_H
