#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_motor_msgs/msg/unitree_command.hpp"

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace unitree_hw_interface
{

    class UnitreeHwInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeHwInterface)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        void start_executor();
        void stop_executor();

        void start_bus_threads();
        void stop_bus_threads();

        void bus_thread(size_t bus_index);
        void apply_realtime();
        void set_low_latency(const std::string &device);

        void command_callback(const unitree_motor_msgs::msg::UnitreeCommand::SharedPtr msg);

        hardware_interface::HardwareInfo info_;

        std::string port_bus0_ = "/dev/ttyUSB0";
        std::string port_bus1_ = "/dev/ttyUSB1";
        uint32_t baudrate_ = 4000000;
        size_t timeout_us_ = 2000;

        std::unique_ptr<SerialPort> bus0_;
        std::unique_ptr<SerialPort> bus1_;

        std::array<double, 12> cmd_q_{};
        std::array<double, 12> cmd_dq_{};
        std::array<double, 12> cmd_tau_{};
        std::array<double, 12> cmd_kp_{};
        std::array<double, 12> cmd_kd_{};

        std::array<double, 12> state_q_{};
        std::array<double, 12> state_dq_{};
        std::array<double, 12> state_tau_{};

        std::mutex data_mutex_;

        std::atomic<bool> running_{false};
        std::thread bus_threads_[2];

        std::mutex cycle_mutex_;
        std::condition_variable cycle_cv_;
        std::condition_variable done_cv_;
        uint64_t cycle_id_{0};
        uint64_t bus_cycle_[2]{0, 0};
        uint64_t bus_done_[2]{0, 0};

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<unitree_motor_msgs::msg::UnitreeCommand>::SharedPtr sub_;
        std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        std::thread executor_thread_;
    };

} // namespace unitree_hw_interface
