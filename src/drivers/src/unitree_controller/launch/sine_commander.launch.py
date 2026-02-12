from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="unitree_controller",
                executable="sine_commander",
                name="unitree_sine_commander",
                output="screen",
            )
        ]
    )
