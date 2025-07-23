from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloud_fuser',
            executable='cloud_fuser_v3',
            name='cloud_fuser_v3',
            output='screen'#,
            #arguments=['--ros-args', '--log-level', 'DEBUG'],
        ),
    ])

