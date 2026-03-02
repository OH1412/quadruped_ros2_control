//
// Created by tlab-uav on 24-9-6.
//

#include "unitree_guide_controller/UnitreeGuideController.h"

#include <unitree_guide_controller/gait/WaveGenerator.h>
#include "unitree_guide_controller/robot/QuadrupedRobot.h"

namespace unitree_guide_controller
{
    using config_type = controller_interface::interface_configuration_type;

    // 声明需要的 command interfaces：
    //   - 如果 command_prefix_ 非空：<command_prefix_>/<joint_name>/<interface_type>
    //   - 否则：<joint_name>/<interface_type>
    // 其中 interface_type 一般包括 {effort, position, velocity, kp, kd}，
    //   最终在 on_activate() 中按名字映射到 ctrl_interfaces_.joint_*_command_interface_
    controller_interface::InterfaceConfiguration UnitreeGuideController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    // 声明需要的 state interfaces：
    //   - 对于每个关节：<joint_name>/<state_interface_type>（如 position / velocity / effort）
    //   - 对于 IMU：<imu_name_>/<imu_interface_type>
    // 这些接口在 on_activate() 里被分类放入 ctrl_interfaces_.joint_*_state_interface_ 或 imu_state_interface_
    controller_interface::InterfaceConfiguration UnitreeGuideController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        return conf;
    }

    // 主循环：
    //   1. 调用 robot_model_->update() 更新关节/刚体状态；
    //   2. 调用 wave_generator_->update() 更新步态相位（哪条腿支撑/摆动）；
    //   3. 调用 estimator_->update() 更新机体位姿、速度、足端等估计；
    //   4. 根据当前 FSMMode：
    //        - NORMAL：当前状态 run() 生成新的关节命令，并根据 checkChange() 判断是否切换状态；
    //        - CHANGE：执行当前状态 exit()，切到 next_state_，再执行其 enter()，然后回到 NORMAL。
    controller_interface::return_type UnitreeGuideController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // auto now = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_diff = now - last_update_time_;
        // last_update_time_ = now;
        //
        // // Calculate the frequency
        // update_frequency_ = 1.0 / time_diff.count();
        // RCLCPP_INFO(get_node()->get_logger(), "Update frequency: %f Hz", update_frequency_);

        // 若还没收到 /robot_description 创建 QuadrupedRobot，则不做控制
        if (ctrl_component_.robot_model_ == nullptr)
        {
            return controller_interface::return_type::OK;
        }

        ctrl_component_.robot_model_->update();
        ctrl_component_.wave_generator_->update();
        ctrl_component_.estimator_->update();

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    // on_init：
    //   - 从参数服务器读取关节名、接口类型、IMU/base 名称、足端名等；
    //   - 读取/声明站立、下蹲姿态和对应的 PD 增益；
    //   - 创建 Estimator 实例（其他组件在 on_configure/on_activate 里创建）。
    controller_interface::CallbackReturn UnitreeGuideController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // imu sensor / 机体/命名空间等配置
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            feet_names_ =
                auto_declare<std::vector<std::string>>("feet_names", feet_names_);

            // pose parameters：默认站立/下蹲关节角与 PD 增益
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            // 创建状态估计器，后续各 FSM 状态从 ctrl_component_.estimator_ 读取机体状态
            ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    // on_configure：
    //   - 订阅 /control_input，将键盘/手柄/遥控器命令写入 ctrl_interfaces_.control_inputs_；
    //   - 订阅 /robot_description，根据 URDF 创建 QuadrupedRobot 模型和 BalanceCtrl；
    //   - 创建 WaveGenerator（指定步态周期和占空比）。
    controller_interface::CallbackReturn UnitreeGuideController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // Handle message
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        // /robot_description 采用 transient_local QoS，确保晚加入时仍能收到一次完整 URDF
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                    ctrl_interfaces_, msg->data, feet_names_, base_name_);
                ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_component_.robot_model_);
            });

        // 步态相位生成器：
        //   - 周期 0.45s，占空比 0.5，四条腿相位为 {0, 0.5, 0.5, 0}
        // 从参数读取步态设置
        double gait_period = auto_declare<double>("gait_period", 0.45);
        double gait_duty = auto_declare<double>("gait_duty", 0.5);
        std::vector<double> gait_phases = auto_declare<std::vector<double>>("gait_phases", {0.0, 0.5, 0.5, 0.0});
        ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(gait_period, gait_duty, Vec4(gait_phases[0], gait_phases[1], gait_phases[2], gait_phases[3]));

        return CallbackReturn::SUCCESS;
    }

    // on_activate：
    //   - 将 ros2_control 注入的 loaned interfaces 整理填入 ctrl_interfaces_；
    //   - 创建各个 FSM 状态实例（passive/fixed stand/trotting 等）；
    //   - 设置初始状态为 PASSIVE 并调用其 enter()。
    controller_interface::CallbackReturn
    UnitreeGuideController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // 1) 绑定 command interfaces：根据接口名（字符串末尾）放入不同数组
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // 2) 绑定 state interfaces：IMU 单独处理，其余按接口名放入 map
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // 3) 创建各个 FSM 状态实例
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.swingTest = std::make_shared<StateSwingTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_);

        // 4) 初始化 FSM：从 PASSIVE 状态开始
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> UnitreeGuideController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::FREESTAND:
            return state_list_.freeStand;
        case FSMStateName::TROTTING:
            return state_list_.trotting;
        case FSMStateName::SWINGTEST:
            return state_list_.swingTest;
        case FSMStateName::BALANCETEST:
            return state_list_.balanceTest;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_guide_controller::UnitreeGuideController, controller_interface::ControllerInterface);
