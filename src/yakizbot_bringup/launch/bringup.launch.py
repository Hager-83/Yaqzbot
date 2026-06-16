from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition, UnlessCondition


import os
import xacro   # FIX: required for proper parsing


def generate_launch_description():

    # =========================
    # Launch Args
    # =========================
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    map_name_arg = DeclareLaunchArgument(
    "map_name",
    default_value="house"
    )

    map_name = LaunchConfiguration("map_name")


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
    # Robot Description (URDF) FIXED
    # =========================
    pkg_path = get_package_share_directory("yakizbot_description")
    xacro_file = os.path.join(pkg_path, "urdf", "yakizbot.urdf.xacro")

    # FIX: safer xacro processing (prevents silent RViz failure)
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # =========================
    # Robot State Publisher
    # =========================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time,
                "publish_frequency": 50.0
            }
        ]
    )

    # =========================
    # Simple Controller
    # =========================
    wheel_controller_node = Node(
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
    # EKF Localization
    # =========================
    controller_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("yakizbot_localization"),
                "config",
                "ekf.yaml"
            ),
            {"use_sim_time": use_sim_time}                        # FIX
        ]
    )

    localization_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("yakizbot_localization"),
            "launch",
            "global_localization.launch.py"
        )
    ),
    condition=UnlessCondition(use_slam),
    launch_arguments={"map_name": map_name}.items()
    )

    
    kinect_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yakizbot_mapping"),
                "launch",
                "kinect.launch.py"
            )
        )
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yakizbot_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_name": map_name
        }.items(),
        condition=IfCondition(use_slam)
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
        use_slam_arg,
        map_name_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        robot_state_publisher,
        wheel_controller_node,
        imu_filter_node,
        controller_node,
        localization_node,
        kinect_node,
        slam_node,
        rviz_node
    ])