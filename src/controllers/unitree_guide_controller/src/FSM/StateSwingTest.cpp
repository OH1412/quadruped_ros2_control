//
// Created by biao on 24-9-12.
//

#include <iostream>
#include "unitree_guide_controller/FSM/StateSwingTest.h"

#include <unitree_guide_controller/control/CtrlComponent.h>

#include "unitree_guide_controller/common/mathTools.h"

StateSwingTest::StateSwingTest(CtrlInterfaces &ctrl_interfaces,
                            CtrlComponent &ctrl_component)
    : FSMState(
          FSMStateName::SWINGTEST, "swing test",
          ctrl_interfaces),
      robot_model_(ctrl_component.robot_model_) {
    _xMin = -0.15;
    _xMax = 0.10;
    _yMin = -0.15;
    _yMax = 0.15;
    _zMin = -0.05;
    _zMax = 0.20;
    // default to leg 1 (joints indices 3,4,5) for swing test
    test_leg_ = 1;  // 0-based leg index (0..3)
}

void StateSwingTest::enter() {
    // set small gains for the three joints of the test_leg_, large gains elsewhere
    for (int i = 0; i < 12; ++i) {
        int leg = i / 3;
        bool is_test = (leg == test_leg_);
        double kp = ctrl_interfaces_.use_sim_kp_kd_
                        ? (is_test ? 3.0 : 180.0)
                        : (is_test ? 3.0 / 40.0 : 180.0 / 40.0);
        double kd = ctrl_interfaces_.use_sim_kp_kd_
                        ? (is_test ? 2.0 : 5.0)
                        : (is_test ? 2.0 / 40.0 : 5.0 / 40.0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd);
    }

    Kp = KDL::Vector(20, 20, 50);
    Kd = KDL::Vector(5, 5, 20);

    init_joint_pos_ = robot_model_->current_joint_pos_;
    init_foot_pos_ = robot_model_->getFeet2BPositions();

    target_foot_pos_ = init_foot_pos_;
    // use test_leg_ index for front reference
    fr_init_pos_ = init_foot_pos_[test_leg_];
    fr_goal_pos_ = fr_init_pos_;
}

void StateSwingTest::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    if (ctrl_interfaces_.control_inputs_.ly > 0) {
        fr_goal_pos_.p.x(invNormalize(ctrl_interfaces_.control_inputs_.ly, fr_init_pos_.p.x(),
                                      fr_init_pos_.p.x() + _xMax, 0, 1));
    } else {
        fr_goal_pos_.p.x(invNormalize(ctrl_interfaces_.control_inputs_.ly, fr_init_pos_.p.x() + _xMin,
                                      fr_init_pos_.p.x(), -1, 0));
    }
    if (ctrl_interfaces_.control_inputs_.lx > 0) {
        fr_goal_pos_.p.y(invNormalize(ctrl_interfaces_.control_inputs_.lx, fr_init_pos_.p.y(),
                                      fr_init_pos_.p.y() + _yMax, 0, 1));
    } else {
        fr_goal_pos_.p.y(invNormalize(ctrl_interfaces_.control_inputs_.lx, fr_init_pos_.p.y() + _yMin,
                                      fr_init_pos_.p.y(), -1, 0));
    }
    if (ctrl_interfaces_.control_inputs_.ry > 0) {
        fr_goal_pos_.p.z(invNormalize(ctrl_interfaces_.control_inputs_.ry, fr_init_pos_.p.z(),
                                      fr_init_pos_.p.z() + _zMax, 0, 1));
    } else {
        fr_goal_pos_.p.z(invNormalize(ctrl_interfaces_.control_inputs_.ry, fr_init_pos_.p.z() + _zMin,
                                      fr_init_pos_.p.z(), -1, 0));
    }

    positionCtrl();
    torqueCtrl();
}

void StateSwingTest::exit() {
}

FSMStateName StateSwingTest::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::SWINGTEST;
    }
}

void StateSwingTest::positionCtrl() {
    // only modify the target foot corresponding to the selected leg
    target_foot_pos_[test_leg_] = fr_goal_pos_;
    target_joint_pos_ = robot_model_->getQ(target_foot_pos_);
    int base = test_leg_ * 3;
    ctrl_interfaces_.joint_position_command_interface_[base].get().set_value(target_joint_pos_[test_leg_](0));
    ctrl_interfaces_.joint_position_command_interface_[base + 1].get().set_value(target_joint_pos_[test_leg_](1));
    ctrl_interfaces_.joint_position_command_interface_[base + 2].get().set_value(target_joint_pos_[test_leg_](2));
}

void StateSwingTest::torqueCtrl() const {
    const KDL::Frame fr_current_pos = robot_model_->getFeet2BPositions(test_leg_);

    const KDL::Vector pos_goal = fr_goal_pos_.p;
    const KDL::Vector pos0 = fr_current_pos.p;
    const KDL::Vector vel0 = robot_model_->getFeet2BVelocities(test_leg_);

    const KDL::Vector force0 = Kp * (pos_goal - pos0) + Kd * -vel0;
    // compute torque for the selected leg
    KDL::JntArray torque0 = robot_model_->getTorque(force0, test_leg_);

    int base = test_leg_ * 3;
    for (int j = 0; j < 3; j++) {
        ctrl_interfaces_.joint_torque_command_interface_[base + j].get().set_value(torque0(j));
    }
}
