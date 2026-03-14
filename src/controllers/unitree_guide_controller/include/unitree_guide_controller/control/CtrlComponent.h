//
// Created by tlab-uav on 25-2-27.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <unitree_guide_controller/gait/WaveGenerator.h>

#include "BalanceCtrl.h"
#include "Estimator.h"

struct CtrlComponent {
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<BalanceCtrl> balance_ctrl_;
    std::shared_ptr<WaveGenerator> wave_generator_;
    
    // Contact-mode configuration (shared single source)
    int contact_mode_ = 0; // 0 = phase-based, 1 = force-threshold-based
    double contact_front_threshold_ = -25.0;
    double contact_rear_threshold_ = -30.0;
    double contact_all_threshold_ = 0.0;

    CtrlComponent() = default;
};
#endif //CTRLCOMPONENT_H
