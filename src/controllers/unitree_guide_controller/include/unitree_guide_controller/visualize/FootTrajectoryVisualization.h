#ifndef FOOT_TRAJECTORY_VISUALIZATION_H
#define FOOT_TRAJECTORY_VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

// use mathTypes.h for Vec34 and VecInt4
#include "unitree_guide_controller/common/mathTypes.h"

namespace unitree_guide_controller {
namespace visualize {

class FootTrajectoryVisualization
{
public:
    explicit FootTrajectoryVisualization(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

    /**
     * Update stored traces and publish current trajectories.
     * @param cur 3x4 matrix of current foot coords in robot frame
     * @param contact 4-vector (0=swing,1=contact); on contact the corresponding line is cleared
     */
    void update(const Vec34 &cur, const VecInt4 &contact);

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    // one sequence of points per leg
    std::array<std::vector<geometry_msgs::msg::Point>, 4> lines_;
};

} // namespace visualize
} // namespace unitree_guide_controller

#endif // FOOT_TRAJECTORY_VISUALIZATION_H
