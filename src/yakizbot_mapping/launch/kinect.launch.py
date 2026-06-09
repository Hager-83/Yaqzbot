from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    kinect_driver = Node(
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2_node",
        parameters=[PathJoinSubstitution([
            FindPackageShare("yakizbot_mapping"),
            "config",
            "kinect_driver.yaml"
        ])],
        output="screen"
    )

    depth_to_scan = Node(
    package='depthimage_to_laserscan',
    executable='depthimage_to_laserscan_node',
    name='depthimage_to_laserscan',
    parameters=[{
        'scan_height': 1,
        'range_min': 0.45,
        'range_max': 4.0,
        'output_frame': 'kinect_depth_optical_frame',
        'use_sim_time': use_sim_time, 
    }],
    remappings=[
        ('depth',             '/depth/image_raw'),
        ('depth_camera_info', '/depth/camera_info'),
        ('scan',               '/scan')
    ],
    output="screen"
)

    return LaunchDescription([
        declare_use_sim_time,
        kinect_driver,
        depth_to_scan,
    ])