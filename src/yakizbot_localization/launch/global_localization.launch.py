import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # =========================
    # Launch Arguments
    # =========================
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="house",          # Change to your default map
        description="Name of the map folder inside yakizbot_mapping/maps/"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=PathJoinSubstitution([
            get_package_share_directory("yakizbot_localization"),
            "config",
            "amcl.yaml"
        ]),
        description="Full path to AMCL config file"
    )

    # =========================
    # Configuration
    # =========================
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    # Path to the map.yaml file
    map_path = PathJoinSubstitution([
    get_package_share_directory("yakizbot_mapping"),
    "maps",
    PythonExpression(["'", map_name, "' + '.yaml'"]) 
])

    lifecycle_nodes = ["map_server", "amcl"]

    # =========================
    # Nodes
    # =========================
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
    ])