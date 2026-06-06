from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro
 


def generate_launch_description():

    # =========================
    # Launch Args
    # =========================
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

    # =========================
    # Robot Description (URDF)
    # =========================
    pkg_path = get_package_share_directory("yakizbot_description")
    xacro_file = os.path.join(pkg_path, "urdf", "yakizbot.urdf.xacro")

    robot_description = xacro.process_file(xacro_file).toxml()

    # =========================
    # Robot State Publisher
    # =========================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
                "publish_frequency": 50.0
            }
        ]
    )

    # =========================
    # Simple Controller
    # =========================
    simple_controller = Node(
        package="yakizbot_control",
        executable="simple_controller",
        name="simple_controller",
        output="screen",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            "use_sim_time": use_sim_time
        }]
    )
    
    # =========================
    # EKF Localization
    # =========================
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("yakizbot_localization"),
                "config",
                "ekf.yaml"
            )
        ]
    )

    # =========================
    # IMU Filter (Madgwick)
    # =========================

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{
        "use_mag": False,
        "world_frame": "enu",
        "publish_tf": False,
        "gain": 0.1,
        "use_sim_time": use_sim_time
        }],
        remappings=[
            ("imu/data_raw", "/imu/data"),
            ("imu/data", "/imu/data_filtered")
]
)
    
    # =========================
    # RViz
    # =========================
    rviz_config = os.path.join(
        get_package_share_directory("yakizbot_localization"),
        "rviz",
        "yakizbot.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config]
    )



    # =========================
    # Launch
    # =========================
    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        robot_state_publisher,
        simple_controller,
        imu_filter_node,
        robot_localization_node,
        rviz_node
    ])