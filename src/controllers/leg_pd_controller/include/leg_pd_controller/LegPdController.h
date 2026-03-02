//
// Created by tlab-uav on 24-9-19.
//

#ifndef LEGPDCONTROLLER_H
#define LEGPDCONTROLLER_H

// 该控制器是一个基于 ros2_control 的“链式”PD 控制器（ChainableControllerInterface）。
// 作用：
//   - 上游控制器（如步态规划、MPC、RL 控制器）通过本类导出的 reference interfaces
//     写入每个关节的期望位置 q_d、期望速度 dq_d、前馈力矩 tau_ff 以及增益 Kp、Kd；
//   - 本控制器从硬件 state interfaces 读取当前关节位置 q 和速度 dq；
//   - 在 update_and_write_commands() 中按如下公式计算力矩并写入 effort command：
//       tau = tau_ff + Kp * (q_d - q) + Kd * (dq_d - dq)
//   - 因为只向下游硬件导出 effort 接口，所以在硬件看来本控制器就是一个纯力矩控制层。

#include <controller_interface/chainable_controller_interface.hpp>
#include "realtime_tools/realtime_buffer.h"
#include <controller_interface/controller_interface.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"


namespace leg_pd_controller {
    // 预留的数据类型（例如以后可以通过话题订阅期望值），当前实现未直接使用
    using DataType = std_msgs::msg::Float64MultiArray;

    class LegPdController final : public controller_interface::ChainableControllerInterface {
    public:
        // 生命周期：声明参数并分配缓存（关节名、state/reference 接口类型等）
        controller_interface::CallbackReturn on_init() override;

        // 告诉 ros2_control 本控制器需要哪些 command interfaces：
        //   对每个关节只申请 joint_name + "/effort" 力矩接口
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        // 告诉 ros2_control 本控制器需要哪些 state interfaces：
        //   由参数 state_interfaces 决定（通常为 position 与 velocity）
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // on_configure：根据关节数分配 reference_interfaces_，为上游控制器提供 q_d/dq_d/tau_ff/Kp/Kd 五类引用
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // on_activate：将 loaned interfaces 绑定到内部引用数组，后续 update 中直接使用
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        // 链式模式开关（本控制器当前对是否 chained 没有特殊处理，直接返回 true）
        bool on_set_chained_mode(bool chained_mode) override;

        // 控制主循环：在每个控制周期内读取期望值和实际值，计算 PD 力矩并写入 effort command
        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        // 向上游控制器导出 reference interfaces：
        //   controller_name/joint_name/{position,velocity,effort,kp,kd}
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

        // 预留：如果以后希望通过订阅话题更新 reference_interfaces_，可在此实现
        #ifdef ROS2_CONTROL_VERSION_LT_3
        controller_interface::return_type update_reference_from_subscribers() override;
        #else
        controller_interface::return_type update_reference_from_subscribers(
            const rclcpp::Time &time, const rclcpp::Duration &period);
        #endif


        // ====== 期望量（由上游控制器通过 reference interfaces 写入） ======

        // 前馈力矩 tau_ff[i]
        std::vector<double> joint_effort_command_;
        // 期望关节位置 q_d[i]
        std::vector<double> joint_position_command_;
        // 期望关节速度 dq_d[i]
        std::vector<double> joint_velocities_command_;
        // 位置误差增益 Kp[i]
        std::vector<double> joint_kp_command_;
        // 速度误差增益 Kd[i]
        std::vector<double> joint_kd_command_;

        // 关节名称列表，对应上面所有数组的索引 i
        std::vector<std::string> joint_names_;

        // 通过参数配置的状态接口类型（例如 {"position", "velocity"}）
        std::vector<std::string> state_interface_types_;
        // 通过参数配置的 reference 接口类型（当前实现中主要使用 position/velocity/effort/kp/kd）
        std::vector<std::string> reference_interface_types_;

        // 若以后通过话题订阅期望值，可用实时 buffer 保存最近一次消息
        realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>> rt_buffer_ptr_;

        // ====== 与硬件交互的 loaned interfaces（由 ros2_control 注入） ======

        // 对每个关节的 effort command 接口引用，用于写入最终力矩
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
        joint_effort_command_interface_;
        // 对每个关节 position state 接口的引用，用于读取当前 q
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
        joint_position_state_interface_;
        // 对每个关节 velocity state 接口的引用，用于读取当前 dq
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
        joint_velocity_state_interface_;

        // 将 state interface 名称（"position" / "velocity"）映射到对应的接口数组，
        // 便于在 on_activate 中按接口名分发 LoanedStateInterface
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &joint_position_state_interface_},
            {"velocity", &joint_velocity_state_interface_}
        };
    };
}


#endif //LEGPDCONTROLLER_H
