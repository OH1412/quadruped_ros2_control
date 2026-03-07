import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

package_controller = "ocs2_quadruped_controller"
package_hardware = "hardware_unitree_mujoco"

def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    joint_names = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                   'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                   'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                   'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control.yaml",
        ]
    )

    rviz_config_file = os.path.join(get_package_share_directory(package_controller), "config", "visualize_ocs2.rviz")

    # 1) Livox driver (MID360) - try multiple likely locations
    livox_launch_path = None
    try:
        livox_share = get_package_share_directory('livox_ros_driver2')
    except Exception:
        livox_share = None

    candidates = []
    if livox_share:
        # Non-standard folder used by some repos
        candidates.append(os.path.join(livox_share, 'launch_ROS2', 'msg_MID360_launch.py'))
        # Standard launch folder
        candidates.append(os.path.join(livox_share, 'launch', 'msg_MID360_launch.py'))

    for p in candidates:
        if os.path.exists(p):
            livox_launch_path = p
            break

    if livox_launch_path is None:
        raise FileNotFoundError('Cannot find Livox MID360 launch file. Tried:\n' + '\n'.join(candidates))

    start_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path),
        # Pass-through args if needed in the future
        # Currently msg_MID360_launch.py usually does not consume use_sim_time
        launch_arguments={}.items(),
    )

    # Start DDS↔ROS bridge to convert Mujoco/real-robot DDS to ROS topics
    unitree_bridge = Node(
        package=package_hardware,
        executable='unitree_dds_ros_bridge',
        name='unitree_dds_ros_bridge',
        parameters=[
            {'network_interface': 'lo'},  # Use localhost for Mujoco simulation
            {'domain': 1},
            {'state_imu_topic': '/livox/imu'},  # Publish IMU on /livox/imu
        ],
        output='screen',
        # Allow disabling the bridge via launch arg
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
    )

    # Optional Standard Bridge to convert DDS topics to standard ROS2 interfaces
    standard_bridge = Node(
        package='unitree_ros2_example',
        executable='standard_bridge',
        name='standard_bridge',
        parameters=[
            {'network_interface': 'lo'},
            {'domain': 1},
            {'joint_names': joint_names},
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_standard_bridge')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    ocs2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_quadruped_controller", "--controller-manager", "/controller_manager"]
    )

    return [
        start_livox,     # Start Livox driver first so IMU/LiDAR topics are ready
        unitree_bridge,  # Start bridge to publish DDS data as ROS topics
        standard_bridge,  # Optional standard bridge for ROS-friendly topics
        rviz,
        robot_state_publisher,
        controller_manager,
        joint_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[ocs2_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='go2_description',
        description='package for robot description'
    )

    # Control whether to start the Unitree DDS↔ROS bridge
    enable_bridge = DeclareLaunchArgument(
        'enable_bridge',
        default_value='false',
        description='Enable Unitree DDS↔ROS bridge (true/false)'
    )

    # Control whether to start the Standard Bridge
    enable_standard_bridge = DeclareLaunchArgument(
        'enable_standard_bridge',
        default_value='false',
        description='Enable Standard Bridge for DDS to ROS2 topics (true/false)'
    )

    return LaunchDescription([
        pkg_description,
        enable_bridge,
        enable_standard_bridge,
        OpaqueFunction(function=launch_setup),
    ])
