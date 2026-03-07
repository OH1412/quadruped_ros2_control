#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "crc32.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;

class UnitreeDdsRosBridge : public rclcpp::Node
{
public:
    UnitreeDdsRosBridge()
    : rclcpp::Node("unitree_dds_ros_bridge")
    {
        declare_parameter<std::string>("network_interface", "lo");
        declare_parameter<int>("domain", 1);
        declare_parameter<std::vector<std::string>>("joint_names", defaultJointNames());

        declare_parameter<std::string>("state_joint_topic", "/joint_states");
        declare_parameter<std::string>("state_imu_topic", "/imu");
        declare_parameter<std::string>("state_foot_force_topic", "/foot_force");
        declare_parameter<std::string>("state_odometry_topic", "/odometry");

        declare_parameter<std::string>("command_joint_topic", "/joint_commands");
        declare_parameter<std::string>("command_kp_topic", "/kp_commands");
        declare_parameter<std::string>("command_kd_topic", "/kd_commands");

        declare_parameter<int>("command_publish_rate_hz", 500);

        network_interface_ = get_parameter("network_interface").as_string();
        domain_ = get_parameter("domain").as_int();
        joint_names_ = get_parameter("joint_names").as_string_array();

        state_joint_topic_ = get_parameter("state_joint_topic").as_string();
        state_imu_topic_ = get_parameter("state_imu_topic").as_string();
        state_foot_force_topic_ = get_parameter("state_foot_force_topic").as_string();
        state_odometry_topic_ = get_parameter("state_odometry_topic").as_string();

        command_joint_topic_ = get_parameter("command_joint_topic").as_string();
        command_kp_topic_ = get_parameter("command_kp_topic").as_string();
        command_kd_topic_ = get_parameter("command_kd_topic").as_string();

        command_publish_rate_hz_ = get_parameter("command_publish_rate_hz").as_int();
        if (command_publish_rate_hz_ <= 0)
        {
            command_publish_rate_hz_ = 500;
        }

        if (joint_names_.empty())
        {
            joint_names_ = defaultJointNames();
        }

        cmd_position_.assign(joint_names_.size(), 0.0);
        cmd_velocity_.assign(joint_names_.size(), 0.0);
        cmd_effort_.assign(joint_names_.size(), 0.0);
        cmd_kp_.assign(joint_names_.size(), 0.0);
        cmd_kd_.assign(joint_names_.size(), 0.0);

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            state_joint_topic_, rclcpp::SensorDataQoS());
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            state_imu_topic_, rclcpp::SensorDataQoS());
        foot_force_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            state_foot_force_topic_, rclcpp::SensorDataQoS());
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            state_odometry_topic_, rclcpp::SensorDataQoS());

        joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            command_joint_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                onJointCommand(std::move(msg));
            });
        kp_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            command_kp_topic_, rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                onKpCommand(std::move(msg));
            });
        kd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            command_kd_topic_, rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                onKdCommand(std::move(msg));
            });

        // DDS setup: bridge LowState/HighState into ROS topics and send LowCmd back.
        ChannelFactory::Instance()->Init(domain_, network_interface_);
        low_cmd_publisher_ =
            std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
        low_cmd_publisher_->InitChannel();

        low_state_subscriber_ =
            std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
        low_state_subscriber_->InitChannel(
            [this](auto&& PH1)
            {
                lowStateMessageHandle(std::forward<decltype(PH1)>(PH1));
            },
            1);

        high_state_subscriber_ =
            std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(TOPIC_HIGHSTATE);
        high_state_subscriber_->InitChannel(
            [this](auto&& PH1)
            {
                highStateMessageHandle(std::forward<decltype(PH1)>(PH1));
            },
            1);

        initLowCmd();

        // Periodic command publisher to DDS using latest ROS command topics.
        const auto period = std::chrono::milliseconds(1000 / command_publish_rate_hz_);
        cmd_timer_ = create_wall_timer(period, [this]() { publishLowCmd(); });

        RCLCPP_INFO(get_logger(),
                    "DDS<->ROS bridge ready. state_joint=%s imu=%s foot_force=%s odom=%s cmd_joint=%s kp=%s kd=%s",
                    state_joint_topic_.c_str(), state_imu_topic_.c_str(), state_foot_force_topic_.c_str(),
                    state_odometry_topic_.c_str(), command_joint_topic_.c_str(), command_kp_topic_.c_str(),
                    command_kd_topic_.c_str());
    }

private:
    static std::vector<std::string> defaultJointNames()
    {
        return {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        };
    }

    void initLowCmd()
    {
        low_cmd_.head()[0] = 0xFE;
        low_cmd_.head()[1] = 0xEF;
        low_cmd_.level_flag() = 0xFF;
        low_cmd_.gpio() = 0;

        for (int i = 0; i < 20; ++i)
        {
            low_cmd_.motor_cmd()[i].mode() = 0x01;
            low_cmd_.motor_cmd()[i].q() = 0;
            low_cmd_.motor_cmd()[i].kp() = 0;
            low_cmd_.motor_cmd()[i].dq() = 0;
            low_cmd_.motor_cmd()[i].kd() = 0;
            low_cmd_.motor_cmd()[i].tau() = 0;
        }
    }

    void lowStateMessageHandle(const void* messages)
    {
        low_state_ = *static_cast<const unitree_go::msg::dds_::LowState_*>(messages);

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = now();
        joint_msg.name = joint_names_;
        joint_msg.position.resize(joint_names_.size());
        joint_msg.velocity.resize(joint_names_.size());
        joint_msg.effort.resize(joint_names_.size());

        const size_t count = std::min<size_t>(joint_names_.size(), low_state_.motor_state().size());
        for (size_t i = 0; i < count; ++i)
        {
            joint_msg.position[i] = low_state_.motor_state()[i].q();
            joint_msg.velocity[i] = low_state_.motor_state()[i].dq();
            joint_msg.effort[i] = low_state_.motor_state()[i].tau_est();
        }
        joint_state_pub_->publish(joint_msg);

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.w = low_state_.imu_state().quaternion()[0];
        imu_msg.orientation.x = low_state_.imu_state().quaternion()[1];
        imu_msg.orientation.y = low_state_.imu_state().quaternion()[2];
        imu_msg.orientation.z = low_state_.imu_state().quaternion()[3];
        imu_msg.angular_velocity.x = low_state_.imu_state().gyroscope()[0];
        imu_msg.angular_velocity.y = low_state_.imu_state().gyroscope()[1];
        imu_msg.angular_velocity.z = low_state_.imu_state().gyroscope()[2];
        imu_msg.linear_acceleration.x = low_state_.imu_state().accelerometer()[0];
        imu_msg.linear_acceleration.y = low_state_.imu_state().accelerometer()[1];
        imu_msg.linear_acceleration.z = low_state_.imu_state().accelerometer()[2];
        // const double gravity = 9.80665;
        // imu_msg.linear_acceleration.x /= gravity;
        // imu_msg.linear_acceleration.y /= gravity;
        // imu_msg.linear_acceleration.z /= gravity;
        imu_pub_->publish(imu_msg);

        std_msgs::msg::Float32MultiArray foot_msg;
        foot_msg.data.resize(4);
        foot_msg.data[0] = low_state_.foot_force()[0];
        foot_msg.data[1] = low_state_.foot_force()[1];
        foot_msg.data[2] = low_state_.foot_force()[2];
        foot_msg.data[3] = low_state_.foot_force()[3];
        foot_force_pub_->publish(foot_msg);
    }

    void highStateMessageHandle(const void* messages)
    {
        high_state_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(messages);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base";
        odom_msg.pose.pose.position.x = high_state_.position()[0];
        odom_msg.pose.pose.position.y = high_state_.position()[1];
        odom_msg.pose.pose.position.z = high_state_.position()[2];
        odom_msg.twist.twist.linear.x = high_state_.velocity()[0];
        odom_msg.twist.twist.linear.y = high_state_.velocity()[1];
        odom_msg.twist.twist.linear.z = high_state_.velocity()[2];
        odom_pub_->publish(odom_msg);
    }

    void onJointCommand(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        if (!msg->name.empty())
        {
            std::unordered_map<std::string, size_t> index_by_name;
            index_by_name.reserve(msg->name.size());
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                index_by_name[msg->name[i]] = i;
            }
            for (size_t j = 0; j < joint_names_.size(); ++j)
            {
                const auto it = index_by_name.find(joint_names_[j]);
                if (it == index_by_name.end())
                {
                    continue;
                }
                const size_t idx = it->second;
                if (idx < msg->position.size())
                {
                    cmd_position_[j] = msg->position[idx];
                }
                if (idx < msg->velocity.size())
                {
                    cmd_velocity_[j] = msg->velocity[idx];
                }
                if (idx < msg->effort.size())
                {
                    cmd_effort_[j] = msg->effort[idx];
                }
            }
        }
        else
        {
            const size_t count = std::min(joint_names_.size(), msg->position.size());
            for (size_t i = 0; i < count; ++i)
            {
                cmd_position_[i] = msg->position[i];
            }
            const size_t vcount = std::min(joint_names_.size(), msg->velocity.size());
            for (size_t i = 0; i < vcount; ++i)
            {
                cmd_velocity_[i] = msg->velocity[i];
            }
            const size_t ecount = std::min(joint_names_.size(), msg->effort.size());
            for (size_t i = 0; i < ecount; ++i)
            {
                cmd_effort_[i] = msg->effort[i];
            }
        }
    }

    void onKpCommand(std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        const size_t count = std::min(joint_names_.size(), msg->data.size());
        for (size_t i = 0; i < count; ++i)
        {
            cmd_kp_[i] = msg->data[i];
        }
    }

    void onKdCommand(std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        const size_t count = std::min(joint_names_.size(), msg->data.size());
        for (size_t i = 0; i < count; ++i)
        {
            cmd_kd_[i] = msg->data[i];
        }
    }

    void publishLowCmd()
    {
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            low_cmd_.motor_cmd()[i].mode() = 0x01;
            low_cmd_.motor_cmd()[i].q() = static_cast<float>(cmd_position_[i]);
            low_cmd_.motor_cmd()[i].dq() = static_cast<float>(cmd_velocity_[i]);
            low_cmd_.motor_cmd()[i].kp() = static_cast<float>(cmd_kp_[i]);
            low_cmd_.motor_cmd()[i].kd() = static_cast<float>(cmd_kd_[i]);
            low_cmd_.motor_cmd()[i].tau() = static_cast<float>(cmd_effort_[i]);
        }

        low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                    (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        low_cmd_publisher_->Write(low_cmd_);
    }

    std::string network_interface_;
    int domain_ = 1;
    std::vector<std::string> joint_names_;
    int command_publish_rate_hz_ = 500;

    std::string state_joint_topic_;
    std::string state_imu_topic_;
    std::string state_foot_force_topic_;
    std::string state_odometry_topic_;
    std::string command_joint_topic_;
    std::string command_kp_topic_;
    std::string command_kd_topic_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr foot_force_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr kp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr kd_sub_;

    rclcpp::TimerBase::SharedPtr cmd_timer_;

    std::vector<double> cmd_position_;
    std::vector<double> cmd_velocity_;
    std::vector<double> cmd_effort_;
    std::vector<double> cmd_kp_;
    std::vector<double> cmd_kd_;

    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};
    unitree_go::msg::dds_::SportModeState_ high_state_{};

    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> low_state_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_state_subscriber_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnitreeDdsRosBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
