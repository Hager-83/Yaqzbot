import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('yakizbot_patrol'),
        'config', 'patrol_points.yaml'
    )

    patrol_node = Node(
        package='yakizbot_patrol',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([patrol_node])