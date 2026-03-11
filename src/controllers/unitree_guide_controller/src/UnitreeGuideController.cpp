//
// Created by tlab-uav on 24-9-6.
//

#include "unitree_guide_controller/UnitreeGuideController.h"
#include <memory>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <unitree_guide_controller/gait/WaveGenerator.h>
#include "unitree_guide_controller/robot/QuadrupedRobot.h"
#include "unitree_guide_controller/visualize/FootTrajectoryVisualization.h"

namespace unitree_guide_controller
{
    using config_type = controller_interface::interface_configuration_type;

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

            ctrl_component_.wave_generator_->setStatusFromFSM(current_state_->state_name);
            
            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        // publish monitoring topics every cycle
        {
            // current body: split into pos, vel, yaw
            Vec3 cur_pos = ctrl_component_.estimator_->getPosition();
            Vec3 cur_vel = ctrl_component_.estimator_->getVelocity();
            double cur_yaw = ctrl_component_.estimator_->getYaw();

            std_msgs::msg::Float32MultiArray pos_msg;
            pos_msg.data.resize(3);
            for (int i = 0; i < 3; ++i) pos_msg.data[i] = cur_pos(i);
            body_pos_pub_->publish(pos_msg);

            std_msgs::msg::Float32MultiArray vel_msg;
            vel_msg.data.resize(3);
            for (int i = 0; i < 3; ++i) vel_msg.data[i] = cur_vel(i);
            body_vel_pub_->publish(vel_msg);

            std_msgs::msg::Float32MultiArray yaw_msg;
            yaw_msg.data.resize(1);
            yaw_msg.data[0] = cur_yaw;
            body_yaw_pub_->publish(yaw_msg);

            // desired body
            Vec3 des_pos(0, 0, 0), des_vel(0, 0, 0);
            double des_yaw = 0.0;
            Vec34 des_feet_pos; des_feet_pos.setZero();
            Vec34 foot_forces; foot_forces.setZero();
            if (auto trotting = std::dynamic_pointer_cast<StateTrotting>(current_state_)) {
                des_pos = trotting->getPcd();
                des_vel = trotting->getVelTarget();
                des_yaw = trotting->getYawCmd();
                des_feet_pos = trotting->getPosFeetGlobalGoal();
                foot_forces = trotting->getForceFeetGlobal();
            } else if (auto balance = std::dynamic_pointer_cast<StateBalanceTest>(current_state_)) {
                des_pos = balance->getPcd();
            }

            std_msgs::msg::Float32MultiArray pos_cmd_msg;
            pos_cmd_msg.data.resize(3);
            for (int i = 0; i < 3; ++i) pos_cmd_msg.data[i] = des_pos(i);
            body_pos_cmd_pub_->publish(pos_cmd_msg);

            std_msgs::msg::Float32MultiArray vel_cmd_msg;
            vel_cmd_msg.data.resize(3);
            for (int i = 0; i < 3; ++i) vel_cmd_msg.data[i] = des_vel(i);
            body_vel_cmd_pub_->publish(vel_cmd_msg);

            std_msgs::msg::Float32MultiArray yaw_cmd_msg;
            yaw_cmd_msg.data.resize(1);
            yaw_cmd_msg.data[0] = des_yaw;
            body_yaw_cmd_pub_->publish(yaw_cmd_msg);

            // current foot positions
            Vec34 cur_feet = ctrl_component_.estimator_->getFeetPos();
            std_msgs::msg::Float32MultiArray fp_msg;
            fp_msg.data.resize(12);
            for (int leg = 0; leg < 4; ++leg) {
                for (int axis = 0; axis < 3; ++axis) {
                    fp_msg.data[leg * 3 + axis] = cur_feet(axis, leg);
                }
            }
            foot_pos_pub_->publish(fp_msg);

            // desired foot positions
            std_msgs::msg::Float32MultiArray fpcmd_msg;
            fpcmd_msg.data.resize(12);
            for (int leg = 0; leg < 4; ++leg) {
                for (int axis = 0; axis < 3; ++axis) {
                    fpcmd_msg.data[leg * 3 + axis] = des_feet_pos(axis, leg);
                }
            }
            foot_pos_cmd_pub_->publish(fpcmd_msg);

            // foot forces
            std_msgs::msg::Float32MultiArray ff_msg;
            ff_msg.data.resize(12);
            for (int leg = 0; leg < 4; ++leg) {
                for (int axis = 0; axis < 3; ++axis) {
                    ff_msg.data[leg * 3 + axis] = foot_forces(axis, leg);
                }
            }
            foot_force_pub_->publish(ff_msg);

            // update foot trajectory visualization (only during trotting state)
            if (foot_vis_) {
                if (auto trotting = std::dynamic_pointer_cast<StateTrotting>(current_state_)) {
                    Vec34 cur_feet = trotting->getFeetPositionGlobal();
                    VecInt4 contacts = trotting->getContact();
                    foot_vis_->update(cur_feet, contacts);
                }
            }
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            feet_names_ =
                auto_declare<std::vector<std::string>>("feet_names", feet_names_);

            // pose parameters
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);
            // gait / wave parameters
            gait_period_ = auto_declare<double>("gait_period", gait_period_);
            gait_duty_ = auto_declare<double>("gait_duty", gait_duty_);
            auto gait_phases = auto_declare<std::vector<double>>("gait_phases",
                                                                    std::vector<double>{gait_bias_(0), gait_bias_(1), gait_bias_(2), gait_bias_(3)});
            if (gait_phases.size() == 4) {
                gait_bias_(0) = gait_phases[0];
                gait_bias_(1) = gait_phases[1];
                gait_bias_(2) = gait_phases[2];
                gait_bias_(3) = gait_phases[3];
            } else {
                RCLCPP_WARN(get_node()->get_logger(),
                            "gait_phases parameter should have size 4, using defaults instead");
            }
            // choose sim or real kp/kd set
            const bool use_sim = auto_declare<bool>("use_sim_kp_kd", false);
            ctrl_interfaces_.use_sim_kp_kd_ = use_sim;
            if (use_sim) {
                // use simulation-friendly kp/kd (unscaled)
                stand_kp_ = 80.0; // sim default from commented options
                stand_kd_ = 3.5; // sim default from commented options
            }
            RCLCPP_INFO(get_node()->get_logger(), "use_sim_kp_kd: %s", use_sim ? "true" : "false");

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

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

        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                    ctrl_interfaces_, msg->data, feet_names_, base_name_);
                ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_component_.robot_model_);
            });

        // intermediate-variable publishers
        foot_force_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/foot_force", 10);
        body_pos_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/body_pos", 10);
        body_vel_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/body_vel", 10);
        body_yaw_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/body_yaw", 10);
        body_pos_cmd_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/body_pos_cmd", 10);
        body_vel_cmd_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/body_vel_cmd", 10);
        body_yaw_cmd_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/body_yaw_cmd", 10);
        foot_pos_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/foot_pos", 10);
        foot_pos_cmd_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/guide/foot_pos_cmd", 10);

        // visualization helper (uses its own publisher internally)
        foot_vis_ = std::make_unique<visualize::FootTrajectoryVisualization>(get_node());

            // construct wave generator using configured gait parameters
        ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(gait_period_, gait_duty_, gait_bias_);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
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

        // assign state interfaces
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

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.swingTest = std::make_shared<StateSwingTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_);

        // Initialize FSM
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
