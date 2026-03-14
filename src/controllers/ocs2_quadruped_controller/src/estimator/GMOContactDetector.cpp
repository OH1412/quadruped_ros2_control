// GMOContactDetector.cpp
#include "ocs2_quadruped_controller/estimator/GMOContactDetector.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

namespace ocs2::legged_robot
{
    GMOContactDetector::GMOContactDetector(const CentroidalModelInfo& info, CtrlInterfaces& ctrl,
                                       rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : info_(info), ctrl_(ctrl), node_(std::move(node))
{
    const size_t nFeet = info_.numThreeDofContacts;
    residuals_.assign(nFeet, vector_t::Zero(3));
    contact_prob_.assign(nFeet, 0.0);
    prev_p_.setZero(info_.stateDim);
    p0_.setZero(info_.stateDim);
    integral_term_.setZero(info_.stateDim);
    prev_r_.setZero(info_.stateDim);
    initialized_ = false;

    if (node_->has_parameter("gmo_ki")) K_I = node_->get_parameter("gmo_ki").as_double();
    if (node_->has_parameter("gmo_force_sigmoid_k")) force_sigmoid_k_ = node_->get_parameter("gmo_force_sigmoid_k").as_double();
    if (node_->has_parameter("gmo_vel_sigmoid_k")) vel_sigmoid_k_ = node_->get_parameter("gmo_vel_sigmoid_k").as_double();
    if (node_->has_parameter("gmo_prob_threshold")) prob_threshold_ = node_->get_parameter("gmo_prob_threshold").as_double();
}


void GMOContactDetector::resetIfNeeded(double dt)
{
    (void)dt;
}

void GMOContactDetector::update(const vector_t& rbd_state, const vector_t& joint_torques, double dt)
{
    // Full GMO-style integral residual implementation.
    //  p_k = normalizedMomentum * mass
    //  integral_term += (beta_k + r_{k-1}) * dt
    //  r_k = K_I * (p_k - p0 - integral_term)

    vector_t p = centroidal_model::getNormalizedMomentum(rbd_state, info_) * info_.robotMass;

    if (!initialized_)
    {
        p0_ = p;
        integral_term_.setZero();
        prev_r_.setZero();
        prev_p_ = p;
        initialized_ = true;
    }

    // If RBD helpers are available, compute a better beta_k using rnea-based calls
    vector_t beta = vector_t::Zero(info_.stateDim);
    if (rbd_conversions_ptr_ && ee_kinematics_ptr_)
    {
        // convert rbd_state -> centroidal state
        const vector_t centroidal_state = rbd_conversions_ptr_->computeCentroidalStateFromRbdModel(rbd_state);
        // zero input
        const vector_t zeroInput = vector_t::Zero(static_cast<long>(info_.inputDim));
        const vector_t zeroJointAcc = vector_t::Zero(static_cast<long>(info_.actuatedDofNum));
        // rnea with current velocities -> C(q,qd)*qd + g
        const vector_t rnea_with_v = rbd_conversions_ptr_->computeRbdTorqueFromCentroidalModel(centroidal_state, zeroInput, zeroJointAcc);
        // rnea with zero velocities -> g (approx)
        vector_t centroidal_state_v0 = centroidal_state;
        centroidal_model::getNormalizedMomentum(centroidal_state_v0, info_).setZero();
        const vector_t rnea_v0 = rbd_conversions_ptr_->computeRbdTorqueFromCentroidalModel(centroidal_state_v0, zeroInput, zeroJointAcc);

        // approximate C*qdot - g = rnea_with_v - 2*rnea_v0
        vector_t coriolis_minus_gravity = rnea_with_v - 2.0 * rnea_v0;

        // place measured joint torques into actuated segment and add the coriolis-gravity term (project if sizes match)
        const size_t actuated = info_.actuatedDofNum;
        if (joint_torques.size() >= actuated)
        {
            // measured joint torques go into actuated joint positions of beta
            beta.segment(6, static_cast<long>(actuated)) = joint_torques.segment(0, static_cast<long>(actuated));
        }
        // add coriolis_minus_gravity into corresponding entries if sizes match
        if (coriolis_minus_gravity.size() >= beta.size())
        {
            beta += coriolis_minus_gravity.segment(0, static_cast<long>(beta.size()));
        }
    }
    else
    {
        // Fallback: previous crude approximation
        const size_t actuated = info_.actuatedDofNum;
        if (joint_torques.size() >= actuated)
        {
            beta.segment(6, static_cast<long>(actuated)) = joint_torques.segment(0, static_cast<long>(actuated));
        }
    }

    // integrate residual according to GMO discrete rule
    integral_term_ += (beta + prev_r_) * dt;

    // compute residual r_k
    vector_t r_k = K_I * (p - p0_ - integral_term_);

    // store for next step
    prev_r_ = r_k;
    prev_p_ = p;

    // Map residual to per-foot force using end-effector Jacobians (least-squares solve of J^T f ~= r_k)
    const size_t nFeet = info_.numThreeDofContacts;
    if (ee_kinematics_ptr_)
    {
        // build centroidal state for Jacobian evaluation if possible
        vector_t centroidal_state = vector_t::Zero(static_cast<long>(info_.stateDim));
        try {
            centroidal_state = rbd_conversions_ptr_ ? rbd_conversions_ptr_->computeCentroidalStateFromRbdModel(rbd_state)
                                                    : centroidal_model::getNormalizedMomentum(rbd_state, info_);
        } catch(...) {
            centroidal_state = vector_t::Zero(static_cast<long>(info_.stateDim));
        }

        const auto posApprox = ee_kinematics_ptr_->getPositionLinearApproximation(centroidal_state);
        const double lambda = 1e-6;
        for (size_t i = 0; i < nFeet && i < posApprox.size(); ++i)
        {
            const matrix_t& J = posApprox[i].dfdx; // 3 x stateDim
            matrix_t A = J * J.transpose(); // 3x3
            A.diagonal().array() += lambda;
            vector_t rhs = J * r_k; // 3x1
            vector3_t f = A.ldlt().solve(rhs);

            double fz = static_cast<double>(f(2));
            if (fz < 0.0) fz = 0.0;

            double p_force = 1.0 / (1.0 + std::exp(-force_sigmoid_k_ * (fz - 5.0)));
            double vz = centroidal_model::getBaseLinearVelocity(rbd_state, info_)(2);
            double p_vel = 1.0 / (1.0 + std::exp(-vel_sigmoid_k_ * (vz - 0.01)));
            double p_phase = 0.5;
            double p_contact = 1.0 - (1.0 - p_force) * (1.0 - p_vel) * (1.0 - p_phase);
            contact_prob_[i] = p_contact;
        }
    }
    else
    {
        // fallback: vertical proxy
        double fz_proxy = std::max(0.0, r_k.size() > 2 ? r_k(2) : 0.0);
        for (size_t i = 0; i < nFeet; ++i)
        {
            double fz = fz_proxy / static_cast<double>(nFeet);
            double p_force = 1.0 / (1.0 + std::exp(-force_sigmoid_k_ * (fz - 5.0)));
            double vz = centroidal_model::getBaseLinearVelocity(rbd_state, info_)(2);
            double p_vel = 1.0 / (1.0 + std::exp(-vel_sigmoid_k_ * (vz - 0.01)));
            double p_phase = 0.5;
            double p_contact = 1.0 - (1.0 - p_force) * (1.0 - p_vel) * (1.0 - p_phase);
            contact_prob_[i] = p_contact;
        }
    }
}

void GMOContactDetector::setHelpers(ocs2::CentroidalModelRbdConversions* rbdConv,
                                   ocs2::PinocchioEndEffectorKinematics* eeKinematics)
{
    rbd_conversions_ptr_ = rbdConv;
    ee_kinematics_ptr_ = eeKinematics;
}

double GMOContactDetector::getContactProb(size_t foot_index) const
{
    if (foot_index >= contact_prob_.size()) return 0.0;
    return contact_prob_[foot_index];
}

} // namespace ocs2::legged_robot
