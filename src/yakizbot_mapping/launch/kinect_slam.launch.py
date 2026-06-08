from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition                          # FIX: import needed
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_slam = LaunchConfiguration('use_slam')

    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='true'
    )

    kinect_driver = Node(
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2_node",
        namespace="kinect",
        parameters=[os.path.join(
            get_package_share_directory("yakizbot_bringup"),
            "config",
            "kinect_driver.yaml"
        )],
        output="screen"
    )

    # FIX: indentation was broken in original
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        namespace='kinect',
        remappings=[
            ('depth',            '/depth/image_raw'),
            ('depth_camera_info','/depth/camera_info'),
            ('scan',             '/scan'),
        ],
        parameters=[{
            'scan_height':    10,
            'range_min':      0.45,
            'range_max':      4.5,
            'output_frame':   'kinect_link',
        }],
        output='screen'
    )

    # FIX: condition added — slam_launch only starts when use_slam:=true
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        condition=IfCondition(use_slam)                            # FIX
    )

    return LaunchDescription([
        declare_use_slam,
        kinect_driver,
        depth_to_scan,
        slam_launch,
    ])