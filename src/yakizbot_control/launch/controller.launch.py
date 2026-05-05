"""
controller.launch.py
─────────────
Launches:

1. Robot State Publisher
2. Joint State Bridge 
3. wheel_odom_node
4. simple_controller
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ─────────────────────────────
    # Launch Arguments
    # ─────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.0425"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.28"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    # ─────────────────────────────
    # URDF via xacro
    # ─────────────────────────────
    pkg_path = get_package_share_directory('yakizbot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'yakizbot.urdf.xacro')

    robot_description = xacro.process_file(xacro_file).toxml()

    # ─────────────────────────────
    # Robot State Publisher
    # ─────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen'
)

    # ─────────────────────────────
    # ADD: Joint State Bridge (IMPORTANT)
    # ─────────────────────────────
    joint_state_bridge = Node(
        package='yakizbot_bringup',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        output='screen'
    )


    # ─────────────────────────────
    # Simple Controller
    # ─────────────────────────────
    simple_controller = Node(
        package='yakizbot_control',
        executable='simple_controller',
        name='simple_controller',
        output='screen',
        parameters=[
        {
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation
        }
]   
    )

    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_bridge,   
        simple_controller
    ])