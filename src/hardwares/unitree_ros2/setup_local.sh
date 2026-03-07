#!/bin/bash
echo "Setup unitree ros2 simulation environment"
export ROS_DOMAIN_ID=1
source /opt/ros/humble/setup.bash
source $HOME/quadruped_ros2_control/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'


