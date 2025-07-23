from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloud_fuser',
            executable='realsense_filter_node',
            name='realsense_filter_node',
            output='screen'
        )
    ])

