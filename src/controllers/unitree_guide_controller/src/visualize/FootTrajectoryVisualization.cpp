#include "unitree_guide_controller/visualize/FootTrajectoryVisualization.h"
#include <geometry_msgs/msg/point.hpp>

namespace unitree_guide_controller {
namespace visualize {

FootTrajectoryVisualization::FootTrajectoryVisualization(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
    : node_(node)
{
    pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/guide/foot_trajectory", 1);
}

void FootTrajectoryVisualization::update(const Vec34 &cur,
                                         const VecInt4 &contact)
{
    visualization_msgs::msg::MarkerArray arr;
    for (int leg = 0; leg < 4; ++leg) {
        if (contact(leg) > 0) {
            // clear trajectory when foot touches ground
            lines_[leg].clear();
        } else {
            geometry_msgs::msg::Point p;
            p.x = cur(0, leg);
            p.y = cur(1, leg);
            p.z = cur(2, leg);
            lines_[leg].push_back(p);
        }

        visualization_msgs::msg::Marker m;
        m.header.frame_id = "base";
        m.header.stamp = node_->now();
        m.ns = "foot_traj";
        m.id = leg;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.005;
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
        m.points = lines_[leg];
        arr.markers.push_back(m);
    }
    pub_->publish(arr);
}

} // namespace visualize
} // namespace unitree_guide_controller
