//
// Created by biao on 24-9-9.
//

#include "hardware_unitree_mujoco/HardwareUnitree.h"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <unordered_map>
using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    joint_torque_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    imu_states_.assign(10, 0);
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);

    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }


    if (const auto show_foot_force_param = info.hardware_parameters.find("show_foot_force"); show_foot_force_param !=
        info.hardware_parameters.end())
    {
        show_foot_force_ = show_foot_force_param->second == "true";
    }
    if (const auto param = info.hardware_parameters.find("state_joint_topic");
        param != info.hardware_parameters.end())
    {
        state_joint_topic_ = param->second;
    }
    if (const auto param = info.hardware_parameters.find("state_imu_topic");
        param != info.hardware_parameters.end())
    {
        state_imu_topic_ = param->second;
    }
    if (const auto param = info.hardware_parameters.find("state_foot_force_topic");
        param != info.hardware_parameters.end())
    {
        state_foot_force_topic_ = param->second;
    }
    if (const auto param = info.hardware_parameters.find("state_odometry_topic");
        param != info.hardware_parameters.end())
    {
        state_odometry_topic_ = param->second;
    }
    if (const auto param = info.hardware_parameters.find("command_joint_topic");
        param != info.hardware_parameters.end())
    {
        command_joint_topic_ = param->second;
    }
    if (const auto param = info.hardware_parameters.find("command_kp_topic");
        param != info.hardware_parameters.end())
    {
        command_kp_topic_ = param->second;
    }
    if (const auto param = info.hardware_parameters.find("command_kd_topic");
        param != info.hardware_parameters.end())
    {
        command_kd_topic_ = param->second;
    }
    if (const auto param = info.hardware_parameters.find("unitree_command_topic");
        param != info.hardware_parameters.end())
    {
        unitree_command_topic_ = param->second;
    }

    // Single ROS node for all topic IO (states in, commands out).
    io_node_ = std::make_shared<rclcpp::Node>("unitree_ros_topic_io");
    executor_.add_node(io_node_);

    // Cache latest state messages for read() to map into ros2_control interfaces.
    joint_state_sub_ = io_node_->create_subscription<sensor_msgs::msg::JointState>(
        state_joint_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
            latest_joint_state_ = std::move(msg);
        });
    imu_sub_ = io_node_->create_subscription<sensor_msgs::msg::Imu>(
        state_imu_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg)
        {
            latest_imu_msg_ = std::move(msg);
        });
    foot_force_sub_ = io_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        state_foot_force_topic_, rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            latest_foot_force_ = std::move(msg);
        });
    odom_sub_ = io_node_->create_subscription<nav_msgs::msg::Odometry>(
        state_odometry_topic_, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg)
        {
            latest_odom_ = std::move(msg);
        });

    // Publish outgoing command topics for a ROS-only actuator layer.
    joint_cmd_pub_ = io_node_->create_publisher<sensor_msgs::msg::JointState>(
        command_joint_topic_, rclcpp::SensorDataQoS());
    kp_pub_ = io_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        command_kp_topic_, rclcpp::SensorDataQoS());
    kd_pub_ = io_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        command_kd_topic_, rclcpp::SensorDataQoS());
    unitree_cmd_pub_ = io_node_->create_publisher<control_input_msgs::msg::UnitreeCommand>(
        unitree_command_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
        rclcpp::get_logger("unitree_hardware"),
        "ROS topic IO configured: joint=%s imu=%s foot_force=%s odom=%s cmd_joint=%s kp=%s kd=%s",
        state_joint_topic_.c_str(), state_imu_topic_.c_str(), state_foot_force_topic_.c_str(),
        state_odometry_topic_.c_str(), command_joint_topic_.c_str(), command_kp_topic_.c_str(),
        command_kd_topic_.c_str());


    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareUnitree::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    // export imu sensor state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
    }

    // export foot force sensor state interface
    if (info_.sensors.size() > 1)
    {
        for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
        }
    }

    // export odometer state interface
    if (info_.sensors.size() > 2)
    {
        // export high state interface
        for (uint i = 0; i < info_.sensors[2].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[2].name, info_.sensors[2].state_interfaces[i].name, &high_states_[i]);
        }
    }


    return
        state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareUnitree::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }
    return command_interfaces;
}

return_type HardwareUnitree::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Update cached ROS messages before mapping into state interfaces.
    executor_.spin_some();

    if (latest_joint_state_)
    {
        const auto& msg = *latest_joint_state_;
        if (!msg.name.empty())
        {
            // Name-based mapping to keep joint order stable.
            std::unordered_map<std::string, size_t> index_by_name;
            index_by_name.reserve(msg.name.size());
            for (size_t i = 0; i < msg.name.size(); ++i)
            {
                index_by_name[msg.name[i]] = i;
            }

            int ind = 0;
            for (const auto& joint_name : joint_interfaces["position"])
            {
                const auto it = index_by_name.find(joint_name);
                if (it != index_by_name.end() && it->second < msg.position.size())
                {
                    joint_position_[ind] = msg.position[it->second];
                }
                ++ind;
            }

            ind = 0;
            for (const auto& joint_name : joint_interfaces["velocity"])
            {
                const auto it = index_by_name.find(joint_name);
                if (it != index_by_name.end() && it->second < msg.velocity.size())
                {
                    joint_velocities_[ind] = msg.velocity[it->second];
                }
                ++ind;
            }

            ind = 0;
            for (const auto& joint_name : joint_interfaces["effort"])
            {
                const auto it = index_by_name.find(joint_name);
                if (it != index_by_name.end() && it->second < msg.effort.size())
                {
                    joint_effort_[ind] = msg.effort[it->second];
                }
                ++ind;
            }
        }
        else
        {
            // Fallback to index-based mapping when names are missing.
            const size_t count = std::min<size_t>(joint_position_.size(), msg.position.size());
            for (size_t i = 0; i < count; ++i)
            {
                joint_position_[i] = msg.position[i];
            }
            const size_t vcount = std::min<size_t>(joint_velocities_.size(), msg.velocity.size());
            for (size_t i = 0; i < vcount; ++i)
            {
                joint_velocities_[i] = msg.velocity[i];
            }
            const size_t ecount = std::min<size_t>(joint_effort_.size(), msg.effort.size());
            for (size_t i = 0; i < ecount; ++i)
            {
                joint_effort_[i] = msg.effort[i];
            }
        }
    }

    // IMU states from ROS topic
    if (latest_imu_msg_)
    {
        imu_states_[0] = latest_imu_msg_->orientation.w; // w
        imu_states_[1] = latest_imu_msg_->orientation.x; // x
        imu_states_[2] = latest_imu_msg_->orientation.y; // y
        imu_states_[3] = latest_imu_msg_->orientation.z; // z
        imu_states_[4] = latest_imu_msg_->angular_velocity.x;
        imu_states_[5] = latest_imu_msg_->angular_velocity.y;
        imu_states_[6] = latest_imu_msg_->angular_velocity.z;
        imu_states_[7] = latest_imu_msg_->linear_acceleration.x;
        imu_states_[8] = latest_imu_msg_->linear_acceleration.y;
        imu_states_[9] = latest_imu_msg_->linear_acceleration.z;
    }

    // Foot force vector (FL, RL, FR, RR) from ROS topic.
    if (latest_foot_force_ && latest_foot_force_->data.size() >= 4)
    {
        foot_force_[0] = latest_foot_force_->data[0];
        foot_force_[1] = latest_foot_force_->data[1];
        foot_force_[2] = latest_foot_force_->data[2];
        foot_force_[3] = latest_foot_force_->data[3];
    }

    if (show_foot_force_)
    {
        RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"), "foot_force(): %f, %f, %f, %f", foot_force_[0], foot_force_[1], foot_force_[2],
                    foot_force_[3]);
    }

    // Odometer states mapped from nav_msgs/Odometry.
    if (latest_odom_)
    {
        high_states_[0] = latest_odom_->pose.pose.position.x;
        high_states_[1] = latest_odom_->pose.pose.position.y;
        high_states_[2] = latest_odom_->pose.pose.position.z;
        high_states_[3] = latest_odom_->twist.twist.linear.x;
        high_states_[4] = latest_odom_->twist.twist.linear.y;
        high_states_[5] = latest_odom_->twist.twist.linear.z;
    }

    // RCLCPP_INFO(get_logger(), "high state: %f %f %f %f %f %f", high_states_[0], high_states_[1], high_states_[2],
    //             high_states_[3], high_states_[4], high_states_[5]);

    return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Publish joint commands and gains for a ROS-only actuator interface.
    if (joint_cmd_pub_)
    {
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.name = joint_interfaces["position"];
        cmd_msg.position = joint_position_command_;
        cmd_msg.velocity = joint_velocities_command_;
        cmd_msg.effort = joint_torque_command_;
        joint_cmd_pub_->publish(cmd_msg);
    }
    if (kp_pub_)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(joint_kp_command_.begin(), joint_kp_command_.end());
        kp_pub_->publish(msg);
    }
    if (kd_pub_)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(joint_kd_command_.begin(), joint_kd_command_.end());
        kd_pub_->publish(msg);
    }
    // Publish combined UnitreeCommand message
    if (unitree_cmd_pub_)
    {
        control_input_msgs::msg::UnitreeCommand ucmd;
        // Copy vectors into fixed-size arrays
        for (size_t i = 0; i < 12 && i < joint_position_command_.size(); ++i) ucmd.q_des[i] = joint_position_command_[i];
        for (size_t i = 0; i < 12 && i < joint_velocities_command_.size(); ++i) ucmd.dq_des[i] = joint_velocities_command_[i];
        for (size_t i = 0; i < 12 && i < joint_torque_command_.size(); ++i) ucmd.tau_ff[i] = joint_torque_command_[i];
        for (size_t i = 0; i < 12 && i < joint_kp_command_.size(); ++i) ucmd.kp[i] = joint_kp_command_[i];
        for (size_t i = 0; i < 12 && i < joint_kd_command_.size(); ++i) ucmd.kd[i] = joint_kd_command_[i];
        unitree_cmd_pub_->publish(ucmd);
    }
    return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareUnitree, hardware_interface::SystemInterface)
