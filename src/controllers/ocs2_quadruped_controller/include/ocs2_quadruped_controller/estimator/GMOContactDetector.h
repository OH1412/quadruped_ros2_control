// GMOContactDetector.h
#pragma once

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <controller_common/CtrlInterfaces.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
// #include <ocs2_legged_robot/gait/GaitSchedule.h>
// #include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

namespace ocs2::legged_robot
{
class GMOContactDetector
{
public:
    GMOContactDetector(const CentroidalModelInfo& info, CtrlInterfaces& ctrl,
                       rclcpp_lifecycle::LifecycleNode::SharedPtr node);

    // call once per control step before updateContact
    void resetIfNeeded(double dt);

    // update with latest rbd_state, joint torques and compute per-foot contact probabilities
    void update(const vector_t& rbd_state, const vector_t& joint_torques, double dt);

    // get fused contact probability for foot i (0..N-1)
    double getContactProb(size_t foot_index) const;

    // inject helpers (optional) for strict computations
    void setHelpers(ocs2::CentroidalModelRbdConversions* rbdConv,
                    ocs2::PinocchioEndEffectorKinematics* eeKinematics);

private:
    CentroidalModelInfo info_;
    CtrlInterfaces& ctrl_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    // simple internal storage
    std::vector<vector_t> residuals_; // per-leg residual mapped to foot force space
    std::vector<double> contact_prob_;

    // GMO state (integral residual)
    vector_t p0_;             // initial generalized momentum
    vector_t integral_term_;  // accumulated integral term
    vector_t prev_r_;         // previous residual r_{k-1}
    bool initialized_ = false;

    // parameters
    double K_I = 1.0; // integral gain
    double force_sigmoid_k_ = 1.0;
    double vel_sigmoid_k_ = 1.0;
    double prob_threshold_ = 0.5;
    vector_t prev_p_; // previous generalized momentum

    // optional helpers
    ocs2::CentroidalModelRbdConversions* rbd_conversions_ptr_ = nullptr;
    ocs2::PinocchioEndEffectorKinematics* ee_kinematics_ptr_ = nullptr;
//     // optional gait information (injected by CtrlComponent / StateEstimateBase)
//     std::shared_ptr<ocs2::legged_robot::GaitSchedule> gait_schedule_ptr_ = nullptr;
//     double current_time_ = 0.0;

// public:
//     // inject gait schedule so GMO can query mode/phase
//     void setGaitSchedule(const std::shared_ptr<ocs2::legged_robot::GaitSchedule>& gaitSchedule) { gait_schedule_ptr_ = gaitSchedule; }
//     // update current time (seconds) so GMO can compute phase-based contact prior
//     void setCurrentTime(double t) { current_time_ = t; }
};
} // namespace ocs2::legged_robot
