//
// Created by biao on 24-9-9.
//


#ifndef HARDWAREUNITREE_H
#define HARDWAREUNITREE_H

#include "hardware_interface/system_interface.hpp"
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <mutex>

#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <unitree_motor_msgs/msg/unitree_command.hpp>

class HardwareUnitree final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    std::vector<double> imu_states_;
    std::vector<double> foot_force_;
    std::vector<double> high_states_;

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };


    void lowStateMessageHandle(const void *messages);

    void highStateMessageHandle(const void *messages);

    void remoteWirelessHandle(const void *messages);

    unitree_go::msg::dds_::LowCmd_ low_cmd_{}; // default init
    unitree_go::msg::dds_::LowState_ low_state_{}; // default init
    unitree_go::msg::dds_::SportModeState_ high_state_{}; // default init

    std::string network_interface_ = "lo";
    int domain_ = 1;
    bool show_foot_force_ = false;

    // ROS topic names
    std::string state_joint_topic_ = "/joint_states";
    std::string state_imu_topic_ = "/imu";
    std::string state_foot_force_topic_ = "/foot_force";
    std::string state_odometry_topic_ = "/odometry";
    std::string command_joint_topic_ = "/joint_commands";
    std::string command_kp_topic_ = "/kp_commands";
    std::string command_kd_topic_ = "/kd_commands";
    std::string unitree_command_topic_ = "/unitree_commands";

    // ROS node and publishers/subscribers
    std::shared_ptr<rclcpp::Node> io_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr kp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr kd_pub_;
    rclcpp::Publisher<unitree_motor_msgs::msg::UnitreeCommand>::SharedPtr unitree_cmd_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr foot_force_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // Latest received messages
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;
    std_msgs::msg::Float32MultiArray::SharedPtr latest_foot_force_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_;

    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> low_state_subscriber_;
    std::mutex low_state_mutex_;
    bool low_state_received_ = false;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_state_subscriber_;
    std::mutex high_state_mutex_;
    bool high_state_received_ = false;

    bool imu_linear_accel_in_g_ = false;
};

#endif //HARDWAREUNITREE_H
