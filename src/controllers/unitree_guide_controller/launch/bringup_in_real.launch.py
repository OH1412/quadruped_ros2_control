import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def _make_livox_include():
    try:
        livox_share = get_package_share_directory('livox_ros_driver2')
    except Exception:
        livox_share = None

    candidates = []
    if livox_share:
        candidates.append(os.path.join(livox_share, 'launch_ROS2', 'msg_MID360_launch.py'))
        candidates.append(os.path.join(livox_share, 'launch', 'msg_MID360_launch.py'))

    for path in candidates:
        if os.path.exists(path):
            return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))

    searched = '\n'.join(candidates) if candidates else '  (livox_ros_driver2 package not found)'
    raise FileNotFoundError('Cannot find Livox MID360 launch file. Tried:\n' + searched)


def generate_launch_description():
    pkg_description_arg = DeclareLaunchArgument(
        'pkg_description',
        default_value='mybot_description',
        description='Package providing the robot description (xacro/URDF)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    use_fast_livo_arg = DeclareLaunchArgument(
        'use_fast_livo',
        default_value='true',
        description='Launch Fast-LIVO odometry node'
    )

    use_sim_kp_kd_arg = DeclareLaunchArgument(
        'use_sim_kp_kd',
        default_value='false',
        description='When true use simulation kp/kd parameters in controller'
    )

    pkg_description = LaunchConfiguration('pkg_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_fast_livo = LaunchConfiguration('use_fast_livo')
    use_sim_kp_kd = LaunchConfiguration('use_sim_kp_kd')

    fast_livo_dir = get_package_share_directory('fast_livo')
    fast_livo_launch_path = os.path.join(fast_livo_dir, 'launch', 'mapping_avia.launch.py')

    livox_launch = _make_livox_include()

    fast_livo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_livo_launch_path),
        launch_arguments={'use_rviz': 'true'}.items(),
        condition=IfCondition(use_fast_livo),
    )

    imu_converter = Node(
        package='livox_imu_converter',
        executable='imu_converter_node',
        name='livox_imu_converter',
        output='screen',
        parameters=[
            {'input_topic': '/livox/imu'},
            {'output_topic': '/imu'},
            {'odom_topic': '/aft_mapped_to_init'},
        ],
    )

    mujoco_launch_path = os.path.join(
        get_package_share_directory('unitree_guide_controller'),
        'launch',
        'mujoco.launch.py'
    )
    unitree_mujoco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mujoco_launch_path),
        launch_arguments={'pkg_description': pkg_description, 'use_sim_kp_kd': use_sim_kp_kd}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(pkg_description_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_fast_livo_arg)
    ld.add_action(use_sim_kp_kd_arg)
    ld.add_action(livox_launch)
    ld.add_action(TimerAction(period=2.0, actions=[fast_livo_launch]))
    ld.add_action(TimerAction(period=4.0, actions=[imu_converter]))
    ld.add_action(unitree_mujoco)

    return ld
