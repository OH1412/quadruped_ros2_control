//
// Created by qiayuan on 2021/11/15.
//

#include "ocs2_quadruped_controller/estimator/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <memory>
#include <utility>

namespace ocs2::legged_robot
{
    StateEstimateBase::StateEstimateBase(CentroidalModelInfo info,
                                         CtrlInterfaces& ctrl_component,
                                         rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ctrl_component_(ctrl_component),
          info_(std::move(info)),
          rbd_state_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)), node_(std::move(node))
    {
        // contact mode: 0 = original threshold, 1 = GMO+fusion
        if (!node_->has_parameter("contact_mode")) {
            node_->declare_parameter("contact_mode", contact_mode_);
        }
        contact_mode_ = node_->get_parameter("contact_mode").as_int();

        if (contact_mode_ == 1) {
            gmo_detector_ = std::make_unique<GMOContactDetector>(info_, ctrl_component_, node_);
        }

        if (!node_->has_parameter("odom_topic")) {
            node_->declare_parameter("odom_topic", odom_topic_);
        }
        odom_topic_ = node_->get_parameter("odom_topic").as_string();

        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    }

    void StateEstimateBase::updateJointStates()
    {
        const size_t size = ctrl_component_.joint_effort_state_interface_.size();
        vector_t joint_pos(size), joint_vel(size);

        for (int i = 0; i < size; i++)
        {
            joint_pos(i) = ctrl_component_.joint_position_state_interface_[i].get().get_value();
            joint_vel(i) = ctrl_component_.joint_velocity_state_interface_[i].get().get_value();
        }

        rbd_state_.segment(6, info_.actuatedDofNum) = joint_pos;
        rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = joint_vel;
    }

    void StateEstimateBase::updateContact()
    {
        const size_t size = ctrl_component_.foot_force_state_interface_.size();
        if (contact_mode_ == 1 && gmo_detector_)
        {
            // prepare joint torques and rbd state
            vector_t joint_torques(size * 3); // conservative size; actual mapping may differ
            for (size_t i = 0; i < ctrl_component_.joint_effort_state_interface_.size(); ++i)
            {
                joint_torques(i) = ctrl_component_.joint_effort_state_interface_[i].get().get_value();
            }
            // simple dt estimate: not available here, pass small dt
            double dt = 0.001;
            gmo_detector_->update(rbd_state_, joint_torques, dt);
            for (int i = 0; i < (int)size; i++)
            {
                contact_flag_[i] = gmo_detector_->getContactProb(i) > 0.5;
            }
        }
        else
        {
            for (int i = 0; i < (int)size; i++)
            {
                contact_flag_[i] = ctrl_component_.foot_force_state_interface_[i].get().get_value() >
                    feet_force_threshold_;
            }
        }
    }

    void StateEstimateBase::updateImu()
    {
        quat_ = {
            ctrl_component_.imu_state_interface_[0].get().get_value(),
            ctrl_component_.imu_state_interface_[1].get().get_value(),
            ctrl_component_.imu_state_interface_[2].get().get_value(),
            ctrl_component_.imu_state_interface_[3].get().get_value()
        };

        angular_vel_local_ = {
            ctrl_component_.imu_state_interface_[4].get().get_value(),
            ctrl_component_.imu_state_interface_[5].get().get_value(),
            ctrl_component_.imu_state_interface_[6].get().get_value()
        };

        linear_accel_local_ = {
            ctrl_component_.imu_state_interface_[7].get().get_value(),
            ctrl_component_.imu_state_interface_[8].get().get_value(),
            ctrl_component_.imu_state_interface_[9].get().get_value()
        };

        // orientationCovariance_ = orientationCovariance;
        // angularVelCovariance_ = angularVelCovariance;
        // linearAccelCovariance_ = linearAccelCovariance;

        const vector3_t zyx = quatToZyx(quat_) - zyx_offset_;
        const vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat_), angular_vel_local_));

        updateAngular(zyx, angularVelGlobal);
    }

    void StateEstimateBase::initPublishers()
    {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
    }

    void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel)
    {
        rbd_state_.segment<3>(0) = zyx;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }

    void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel)
    {
        rbd_state_.segment<3>(3) = pos;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry& odom) const
    {
        rclcpp::Time time = odom.header.stamp;
        odom_pub_->publish(odom);

        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header = odom.header;
        pose.pose.pose = odom.pose.pose;
        pose_pub_->publish(pose);
    }
} // namespace legged
