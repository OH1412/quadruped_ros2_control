//
// Created by tlab-uav on 24-9-16.
//

#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include <memory>

#include "unitree_guide_controller/common/mathTypes.h"
class QuadrupedRobot;

class BalanceCtrl {
public:
    // constructor takes robot model and an optional flag indicating whether
    // simulation-style (unscaled) kp/kd gains are used. When running in sim the
    // effective gravity should be the real value (-9.81), otherwise we boost it
    // to -12.0 to compensate for modeling errors on the real robot.
    explicit BalanceCtrl(const std::shared_ptr<QuadrupedRobot>& robot,
                         bool use_sim_kp_kd = false);

    ~BalanceCtrl() = default;

    /**
     * Calculate the desired feet end force
     * @param ddPcd desired body acceleration
     * @param dWbd desired body angular acceleration
     * @param rot_matrix current body rotation matrix
     * @param feet_pos_2_body feet positions to body under world frame
     * @param contact feet contact
     * @return
     */
    Vec34 calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix,
               const Vec34 &feet_pos_2_body, const VecInt4 &contact);

private:
    void calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM);

    void calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM);

    void calConstraints(const VecInt4 &contact);

    void solveQP();

    Mat12 G_, W_, U_;
    Mat6 S_;
    Mat3 Ib_;
    Vec6 bd_;
    Vec3 g_, pcb_;
    Vec12 F_, F_prev_, g0T_;
    double mass_, alpha_, beta_, friction_ratio_;
    Eigen::MatrixXd CE_, CI_;
    Eigen::VectorXd ce0_, ci0_;
    Eigen::Matrix<double, 6, 12> A_;
    Eigen::Matrix<double, 5, 3> friction_mat_;
};


#endif //BALANCECTRL_H
