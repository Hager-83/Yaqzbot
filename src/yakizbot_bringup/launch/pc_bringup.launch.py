from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition, UnlessCondition

import os


def generate_launch_description():

    # =========================
    # Launch Args
    # =========================
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="house"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam     = LaunchConfiguration("use_slam")
    map_name     = LaunchConfiguration("map_name")

    # =========================
    # Localization (AMCL)
    # =========================
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

    # =========================
    # SLAM
    # =========================
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
            "map_name":     map_name
        }.items(),
        condition=IfCondition(use_slam)
    )

    # =========================
    # Navigation
    # =========================
    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yakizbot_navigation"),
                "launch",
                "navigation.launch.py"
            )
        ),
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
        localization_node,
        slam_node,
        navigation_node,
        rviz_node,
    ])