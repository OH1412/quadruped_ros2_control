import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control.yaml",
        ]
    )

    controller_parameters = [robot_controllers]
    if context.launch_configurations.get('use_sim_kp_kd', 'false').lower() == 'true':
        controller_parameters.append(
            os.path.join(
                get_package_share_directory('unitree_guide_controller'),
                'config',
                'use_sim_kp_kd.yaml',
            )
        )

    # optionally append contact-mode overrides based on launch args and robot description
    contact_mode = context.launch_configurations.get('contact_mode', '0')
    if contact_mode == '1':
        pkg_desc = context.launch_configurations.get('pkg_description', '')
        if 'go2' in pkg_desc:
            controller_parameters.append(
                os.path.join(
                    get_package_share_directory('unitree_guide_controller'),
                    'config',
                    'contact_mode_go2.yaml',
                )
            )
        elif 'mybot' in pkg_desc:
            controller_parameters.append(
                os.path.join(
                    get_package_share_directory('unitree_guide_controller'),
                    'config',
                    'contact_mode_mybot.yaml',
                )
            )
        else:
            controller_parameters.append(
                os.path.join(
                    get_package_share_directory('unitree_guide_controller'),
                    'config',
                    'contact_mode_default.yaml',
                )
            )

    rviz_config_file = os.path.join(get_package_share_directory(package_description), "config", "visualize_urdf.rviz")

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
        parameters=controller_parameters,
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

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_guide_controller", "--controller-manager", "/controller_manager"],
    )

    return [
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
                on_exit=[unitree_guide_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='go2_description',
        description='package for robot description'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim_kp_kd',
        default_value='false',
        description='If true, use simulation kp/kd parameters (mujoco)'
    )

    contact_mode_arg = DeclareLaunchArgument(
        'contact_mode',
        default_value='0',
        description='Contact detection mode: 0=phase-based, 1=force-threshold-based'
    )

    return LaunchDescription([
        pkg_description,
        use_sim_arg,
        contact_mode_arg,
        OpaqueFunction(function=launch_setup),
    ])
