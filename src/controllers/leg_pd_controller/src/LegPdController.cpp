//
// Created by tlab-uav on 24-9-19.
//

#include "leg_pd_controller/LegPdController.h"

namespace leg_pd_controller {
    using config_type = controller_interface::interface_configuration_type;

    // on_init：
    //   - 从参数服务器读取 "joints"、"reference_interfaces"、"state_interfaces" 三个参数；
    //   - 根据关节数分配期望量缓存（q_d, dq_d, tau_ff, Kp, Kd）。
    controller_interface::CallbackReturn LegPdController::on_init() {
        try {
            // joints: 关节名称列表，决定控制的自由度
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
            // reference_interfaces: 导出的 reference 接口类型（当前实现主要使用 position/velocity/effort/kp/kd）
            reference_interface_types_ =
                    auto_declare<std::vector<std::string> >("reference_interfaces", reference_interface_types_);
            // state_interfaces: 需要订阅的状态接口类型（例如 position、velocity）
            state_interface_types_ = auto_declare<std::vector<
                std::string> >("state_interfaces", state_interface_types_);
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        const size_t joint_num = joint_names_.size();
        // 为每个关节分配期望量缓存，初始为 0
        joint_effort_command_.assign(joint_num, 0);
        joint_position_command_.assign(joint_num, 0);
        joint_velocities_command_.assign(joint_num, 0);
        joint_kp_command_.assign(joint_num, 0);
        joint_kd_command_.assign(joint_num, 0);

        return CallbackReturn::SUCCESS;
    }

    // 告诉 ros2_control：本控制器需要哪些 command interfaces
    controller_interface::InterfaceConfiguration LegPdController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size());
        // 对每个关节，仅申请 "<joint_name>/effort" 力矩命令接口
        for (const auto &joint_name: joint_names_) {
            conf.names.push_back(joint_name + "/effort");
        }

        return conf;
    }

    // 告诉 ros2_control：本控制器需要哪些 state interfaces（例如 position / velocity）
    controller_interface::InterfaceConfiguration LegPdController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                // 构造 "<joint_name>/<interface_type>" 形式的状态接口名称
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }
        return conf;
    }

    // on_configure：分配 reference_interfaces_ 缓冲区
    // 约定：每个关节占用 5 个 double，对应 q_d, dq_d, tau_ff, Kp, Kd（具体布局由上层使用时约定）
    controller_interface::CallbackReturn LegPdController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        reference_interfaces_.resize(joint_names_.size() * 5, std::numeric_limits<double>::quiet_NaN());
        return CallbackReturn::SUCCESS;
    }

    // on_activate：把 ros2_control 注入的 loaned interfaces 绑定到内部引用数组
    controller_interface::CallbackReturn LegPdController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        joint_effort_command_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        // 1) 记录每个关节的 effort command interface（写输出力矩用）
        for (auto &interface: command_interfaces_) {
            joint_effort_command_interface_.emplace_back(interface);
        }

        // 2) 按接口名（position/velocity）把 state interfaces 分类放入对应数组
        for (auto &interface: state_interfaces_) {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    // 当前未对 chained_mode 做特殊处理，直接允许开启链式模式
    bool LegPdController::on_set_chained_mode(bool /*chained_mode*/) {
        return true;
    }

    // 每个控制周期调用：
    //   1. 检查关节数量与内部数组尺寸是否一致；
    //   2. 对每个关节根据 q_d/dq_d/tau_ff/Kp/Kd 以及当前 q/dq 计算 PD 力矩；
    //   3. 通过 joint_effort_command_interface_ 写回到硬件 effort 命令接口。
    controller_interface::return_type LegPdController::update_and_write_commands(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        if (joint_names_.size() != joint_effort_command_.size() ||
            joint_names_.size() != joint_kp_command_.size() ||
            joint_names_.size() != joint_position_command_.size() ||
            joint_names_.size() != joint_position_state_interface_.size() ||
            joint_names_.size() != joint_velocity_state_interface_.size() ||
            joint_names_.size() != joint_effort_command_interface_.size()) {
            std::cout << "joint_names_.size() = " << joint_names_.size() << std::endl;
            std::cout << "joint_effort_command_.size() = " << joint_effort_command_.size() << std::endl;
            std::cout << "joint_kp_command_.size() = " << joint_kp_command_.size() << std::endl;
            std::cout << "joint_position_command_.size() = " << joint_position_command_.size() << std::endl;
            std::cout << "joint_position_state_interface_.size() = " << joint_position_state_interface_.size() <<
                    std::endl;
            std::cout << "joint_velocity_state_interface_.size() = " << joint_velocity_state_interface_.size() <<
                    std::endl;
            std::cout << "joint_effort_command_interface_.size() = " << joint_effort_command_interface_.size() <<
                    std::endl;

            throw std::runtime_error("Mismatch in vector sizes in update_and_write_commands");
        }

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            // PD 控制律：
            //   tau[i] = tau_ff[i]
            //            + Kp[i] * (q_d[i]  - q[i])
            //            + Kd[i] * (dq_d[i] - dq[i])
            const double torque =
                // 前馈力矩 tau_ff
                joint_effort_command_[i]
                // 位置误差项 Kp * (q_d - q)
                + joint_kp_command_[i] * (
                    joint_position_command_[i] - joint_position_state_interface_[i].get().get_value())
                // 速度误差项 Kd * (dq_d - dq)
                + joint_kd_command_[i] * (
                    joint_velocities_command_[i] - joint_velocity_state_interface_[i].get().get_value());

            // 将计算得到的力矩写入对应关节的 effort command 接口
            joint_effort_command_interface_[i].get().set_value(torque);
        }

        return controller_interface::return_type::OK;
    }

    // 导出 reference interfaces：
    //   对每个关节导出 5 个接口：position / velocity / effort / kp / kd
    //   上游控制器可以通过这些接口写入 q_d, dq_d, tau_ff, Kp, Kd
    std::vector<hardware_interface::CommandInterface> LegPdController::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;

        int ind = 0;
        std::string controller_name = get_node()->get_name();
        for (const auto &joint_name: joint_names_) {
            std::cout << joint_name << std::endl;
            reference_interfaces.emplace_back(controller_name, joint_name + "/position", &joint_position_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/velocity",
                                              &joint_velocities_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/effort", &joint_effort_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/kp", &joint_kp_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/kd", &joint_kd_command_[ind]);
            ind++;
        }

        return reference_interfaces;
    }

    // 目前未实现通过话题订阅更新 reference_interfaces_，因此这里直接返回 OK
    // 如需从订阅者更新期望值，可在此函数中读取 rt_buffer_ptr_ 并写入 joint_*_command_
#ifdef ROS2_CONTROL_VERSION_LT_3
    controller_interface::return_type LegPdController::update_reference_from_subscribers() {
        return controller_interface::return_type::OK;
    }
#else
    controller_interface::return_type LegPdController::update_reference_from_subscribers(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        return controller_interface::return_type::OK;
    }
#endif
}

// 将 LegPdController 注册为 pluginlib 插件，使其可以被 ros2_control 加载
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(leg_pd_controller::LegPdController, controller_interface::ChainableControllerInterface);
