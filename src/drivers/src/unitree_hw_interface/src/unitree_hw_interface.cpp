#include "unitree_hw_interface/unitree_hw_interface.hpp"

#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <functional>
#include <linux/serial.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"

namespace unitree_hw_interface
{

    hardware_interface::CallbackReturn UnitreeHwInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        info_ = info;

        if (info_.joints.size() != 12)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.count("serial_bus0") > 0)
        {
            port_bus0_ = info_.hardware_parameters.at("serial_bus0");
        }
        if (info_.hardware_parameters.count("serial_bus1") > 0)
        {
            port_bus1_ = info_.hardware_parameters.at("serial_bus1");
        }
        if (info_.hardware_parameters.count("baudrate") > 0)
        {
            baudrate_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("baudrate")));
        }
        if (info_.hardware_parameters.count("timeout_us") > 0)
        {
            timeout_us_ = static_cast<size_t>(std::stoul(info_.hardware_parameters.at("timeout_us")));
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> UnitreeHwInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(12 * 3);

        for (size_t i = 0; i < 12; ++i)
        {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_q_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_dq_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_tau_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> UnitreeHwInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(12 * 5);

        for (size_t i = 0; i < 12; ++i)
        {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_q_[i]);
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_dq_[i]);
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &cmd_tau_[i]);
            command_interfaces.emplace_back(info_.joints[i].name, "kp", &cmd_kp_[i]);
            command_interfaces.emplace_back(info_.joints[i].name, "kd", &cmd_kd_[i]);
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn UnitreeHwInterface::on_activate(
        const rclcpp_lifecycle::State &)
    {
        if (!rclcpp::ok())
        {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
        }

        node_ = rclcpp::Node::make_shared("unitree_hw_interface");
        sub_ = node_->create_subscription<unitree_motor_msgs::msg::UnitreeCommand>(
            "unitree_command", rclcpp::SystemDefaultsQoS(),
            std::bind(&UnitreeHwInterface::command_callback, this, std::placeholders::_1));

        start_executor();

        set_low_latency(port_bus0_);
        set_low_latency(port_bus1_);

        bus0_ = std::make_unique<SerialPort>(port_bus0_, 16, baudrate_, timeout_us_, BlockYN::NO);
        bus1_ = std::make_unique<SerialPort>(port_bus1_, 16, baudrate_, timeout_us_, BlockYN::NO);

        running_.store(true);
        start_bus_threads();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn UnitreeHwInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        stop_bus_threads();
        stop_executor();

        bus0_.reset();
        bus1_.reset();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type UnitreeHwInterface::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        std::unique_lock<std::mutex> lock(cycle_mutex_);
        uint64_t target_cycle = cycle_id_;
        done_cv_.wait(lock, [&]()
                      { return !running_.load() || (bus_done_[0] >= target_cycle && bus_done_[1] >= target_cycle); });

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UnitreeHwInterface::write(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        {
            std::lock_guard<std::mutex> lock(cycle_mutex_);
            ++cycle_id_;
        }
        cycle_cv_.notify_all();

        std::unique_lock<std::mutex> lock(cycle_mutex_);
        uint64_t target_cycle = cycle_id_;
        done_cv_.wait(lock, [&]()
                      { return !running_.load() || (bus_done_[0] >= target_cycle && bus_done_[1] >= target_cycle); });

        return hardware_interface::return_type::OK;
    }

    void UnitreeHwInterface::start_executor()
    {
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]()
                                       { executor_->spin(); });
    }

    void UnitreeHwInterface::stop_executor()
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (executor_thread_.joinable())
        {
            executor_thread_.join();
        }
        executor_.reset();
        sub_.reset();
        node_.reset();
    }

    void UnitreeHwInterface::start_bus_threads()
    {
        bus_threads_[0] = std::thread(&UnitreeHwInterface::bus_thread, this, 0);
        bus_threads_[1] = std::thread(&UnitreeHwInterface::bus_thread, this, 1);
    }

    void UnitreeHwInterface::stop_bus_threads()
    {
        running_.store(false);
        cycle_cv_.notify_all();

        if (bus_threads_[0].joinable())
        {
            bus_threads_[0].join();
        }
        if (bus_threads_[1].joinable())
        {
            bus_threads_[1].join();
        }
    }

    void UnitreeHwInterface::bus_thread(size_t bus_index)
    {
        apply_realtime();

        std::unique_ptr<SerialPort> *port = (bus_index == 0) ? &bus0_ : &bus1_;
        size_t index_start = (bus_index == 0) ? 0 : 6;
        size_t index_end = (bus_index == 0) ? 6 : 12;
        unsigned short id_base = (bus_index == 0) ? 1 : 7;

        while (running_.load())
        {
            uint64_t cycle = 0;
            {
                std::unique_lock<std::mutex> lock(cycle_mutex_);
                cycle_cv_.wait(lock, [&]()
                               { return !running_.load() || cycle_id_ > bus_cycle_[bus_index]; });
                if (!running_.load())
                {
                    break;
                }
                cycle = cycle_id_;
                bus_cycle_[bus_index] = cycle;
            }

            for (size_t motor_index = index_start; motor_index < index_end; ++motor_index)
            {
                MotorCmd cmd;
                MotorData data;

                cmd.motorType = MotorType::GO_M8010_6;
                data.motorType = MotorType::GO_M8010_6;
                cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                cmd.id = static_cast<unsigned short>(id_base + (motor_index - index_start));

                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    cmd.q = static_cast<float>(cmd_q_[motor_index]);
                    cmd.dq = static_cast<float>(cmd_dq_[motor_index]);
                    cmd.tau = static_cast<float>(cmd_tau_[motor_index]);
                    cmd.kp = static_cast<float>(cmd_kp_[motor_index]);
                    cmd.kd = static_cast<float>(cmd_kd_[motor_index]);
                }

                (*port)->sendRecv(&cmd, &data);

                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    state_q_[motor_index] = data.q;
                    state_dq_[motor_index] = data.dq;
                    state_tau_[motor_index] = data.tau;
                }
            }

            {
                std::lock_guard<std::mutex> lock(cycle_mutex_);
                bus_done_[bus_index] = cycle;
            }
            done_cv_.notify_all();
        }
    }

    void UnitreeHwInterface::apply_realtime()
    {
        mlockall(MCL_CURRENT | MCL_FUTURE);

        sched_param params;
        params.sched_priority = 99;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    }

    void UnitreeHwInterface::set_low_latency(const std::string &device)
    {
        int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0)
        {
            return;
        }

        serial_struct serinfo;
        if (ioctl(fd, TIOCGSERIAL, &serinfo) == 0)
        {
            serinfo.flags |= ASYNC_LOW_LATENCY;
            ioctl(fd, TIOCSSERIAL, &serinfo);
        }
        close(fd);
    }

    void UnitreeHwInterface::command_callback(const unitree_motor_msgs::msg::UnitreeCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < 12; ++i)
        {
            cmd_q_[i] = msg->q_des[i];
            cmd_dq_[i] = msg->dq_des[i];
            cmd_tau_[i] = msg->tau_ff[i];
            cmd_kp_[i] = msg->kp[i];
            cmd_kd_[i] = msg->kd[i];
        }
    }

} // namespace unitree_hw_interface

PLUGINLIB_EXPORT_CLASS(unitree_hw_interface::UnitreeHwInterface, hardware_interface::SystemInterface)
