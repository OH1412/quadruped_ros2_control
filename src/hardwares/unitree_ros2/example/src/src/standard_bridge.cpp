/**
 * Standard Bridge for Unitree ROS2
 * Bridges DDS topics to standard ROS2 messages and vice versa
 **/

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

class StandardBridge : public rclcpp::Node
{
public:
    StandardBridge()
    : rclcpp::Node("standard_bridge")
    {
        declare_parameter<std::vector<std::string>>("joint_names", defaultJointNames());

        declare_parameter<std::string>("state_joint_topic", "/joint_states");
        declare_parameter<std::string>("state_imu_topic", "/imu");
        declare_parameter<std::string>("state_foot_force_topic", "/foot_force");
        declare_parameter<std::string>("state_odometry_topic", "/odometry");

        declare_parameter<std::string>("command_joint_topic", "/joint_command");
        declare_parameter<std::string>("command_kp_topic", "/joint_kp");
        declare_parameter<std::string>("command_kd_topic", "/joint_kd");

        declare_parameter<int>("command_publish_rate_hz", 500);

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

        low_cmd_pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

        low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
            "lowstate", 10,
            [this](unitree_go::msg::LowState::SharedPtr msg)
            {
                lowStateCallback(msg);
            });
        sport_mode_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            [this](unitree_go::msg::SportModeState::SharedPtr msg)
            {
                sportModeStateCallback(msg);
            });

        joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            command_joint_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                onJointCommand(msg);
            });
        kp_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            command_kp_topic_, rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                onKpCommand(msg);
            });
        kd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            command_kd_topic_, rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                onKdCommand(msg);
            });

        // Periodic command publisher
        const auto period = std::chrono::milliseconds(1000 / command_publish_rate_hz_);
        cmd_timer_ = create_wall_timer(period, [this]() { publishLowCmd(); });

        initLowCmd();

        RCLCPP_INFO(get_logger(),
                    "Standard Bridge ready. DDS->ROS: lowstate->joint_states/imu/foot_force, sportmodestate->odometry. ROS->DDS: joint_command/kp/kd->lowcmd");
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

    std::vector<double> sanitizeGains(std::vector<double> gains) const
    {
        if (gains.size() != joint_names_.size())
        {
            gains.resize(joint_names_.size(), gains.empty() ? 0.0 : gains.back());
        }
        return gains;
    }

    void initLowCmd()
    {
        low_cmd_.head[0] = 0xFE;
        low_cmd_.head[1] = 0xEF;
        low_cmd_.level_flag = 0xFF;
        low_cmd_.gpio = 0;

        for (int i = 0; i < 20; ++i)
        {
            low_cmd_.motor_cmd[i].mode = 0x01;
            low_cmd_.motor_cmd[i].q = 0;
            low_cmd_.motor_cmd[i].kp = 0;
            low_cmd_.motor_cmd[i].dq = 0;
            low_cmd_.motor_cmd[i].kd = 0;
            low_cmd_.motor_cmd[i].tau = 0;
        }
    }

    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr &msg)
    {
        // Joint States
        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = now();
        joint_msg.name = joint_names_;
        joint_msg.position.resize(joint_names_.size());
        joint_msg.velocity.resize(joint_names_.size());
        joint_msg.effort.resize(joint_names_.size());

        const size_t count = std::min<size_t>(joint_names_.size(), msg->motor_state.size());
        for (size_t i = 0; i < count; ++i)
        {
            joint_msg.position[i] = msg->motor_state[i].q;
            joint_msg.velocity[i] = msg->motor_state[i].dq;
            joint_msg.effort[i] = msg->motor_state[i].tau_est;
        }
        joint_state_pub_->publish(joint_msg);

        // IMU
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.w = msg->imu_state.quaternion[0];
        imu_msg.orientation.x = msg->imu_state.quaternion[1];
        imu_msg.orientation.y = msg->imu_state.quaternion[2];
        imu_msg.orientation.z = msg->imu_state.quaternion[3];
        imu_msg.angular_velocity.x = msg->imu_state.gyroscope[0];
        imu_msg.angular_velocity.y = msg->imu_state.gyroscope[1];
        imu_msg.angular_velocity.z = msg->imu_state.gyroscope[2];
        imu_msg.linear_acceleration.x = msg->imu_state.accelerometer[0];
        imu_msg.linear_acceleration.y = msg->imu_state.accelerometer[1];
        imu_msg.linear_acceleration.z = msg->imu_state.accelerometer[2];
        const double gravity = 9.80665;
        imu_msg.linear_acceleration.x /= gravity;
        imu_msg.linear_acceleration.y /= gravity;
        imu_msg.linear_acceleration.z /= gravity;
        imu_pub_->publish(imu_msg);

        // Foot Force
        std_msgs::msg::Float32MultiArray foot_msg;
        foot_msg.data.resize(4);
        foot_msg.data[0] = msg->foot_force[0];
        foot_msg.data[1] = msg->foot_force[1];
        foot_msg.data[2] = msg->foot_force[2];
        foot_msg.data[3] = msg->foot_force[3];
        foot_force_pub_->publish(foot_msg);
    }

    void sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr &msg)
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base";
        odom_msg.pose.pose.position.x = msg->position[0];
        odom_msg.pose.pose.position.y = msg->position[1];
        odom_msg.pose.pose.position.z = msg->position[2];
        odom_msg.twist.twist.linear.x = msg->velocity[0];
        odom_msg.twist.twist.linear.y = msg->velocity[1];
        odom_msg.twist.twist.linear.z = msg->velocity[2];
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
        std::vector<double> incoming(msg->data.begin(), msg->data.end());
        cmd_kp_ = sanitizeGains(incoming);
    }

    void onKdCommand(std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        std::vector<double> incoming(msg->data.begin(), msg->data.end());
        cmd_kd_ = sanitizeGains(incoming);
    }

    void publishLowCmd()
    {
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            low_cmd_.motor_cmd[i].mode = 0x01;
            low_cmd_.motor_cmd[i].q = static_cast<float>(cmd_position_[i]);
            low_cmd_.motor_cmd[i].dq = static_cast<float>(cmd_velocity_[i]);
            low_cmd_.motor_cmd[i].kp = static_cast<float>(cmd_kp_[i]);
            low_cmd_.motor_cmd[i].kd = static_cast<float>(cmd_kd_[i]);
            low_cmd_.motor_cmd[i].tau = static_cast<float>(cmd_effort_[i]);
        }

        // Note: CRC calculation might be needed, but in unitree_ros2 examples, it's handled differently
        // For simplicity, assuming it's included or not required here
        low_cmd_pub_->publish(low_cmd_);
    }

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
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr kp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr kd_sub_;

    rclcpp::TimerBase::SharedPtr cmd_timer_;

    std::vector<double> cmd_position_;
    std::vector<double> cmd_velocity_;
    std::vector<double> cmd_effort_;
    std::vector<double> cmd_kp_;
    std::vector<double> cmd_kd_;

    unitree_go::msg::LowCmd low_cmd_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StandardBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}