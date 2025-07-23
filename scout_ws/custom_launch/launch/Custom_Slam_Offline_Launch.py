from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Elevate to bring up CAN interface
        #ExecuteProcess(
        #    cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '500000'],
        #    output='screen'
        #),

        # Launch Livox Driver
        ExecuteProcess(
            cmd=['ros2', 'launch', 'livox_ros_driver2', 'rviz_HAP_launch.py'],
            output='screen'
        ),

        # Launch Scout Base
        ExecuteProcess(
            cmd=['ros2', 'launch', 'scout_base', 'scout_base.launch.py'],
            output='screen'
        ),

        # Launch Scout Description
        ExecuteProcess(
            cmd=['ros2', 'launch', 'scout_description', 'scout_base_description.launch.py'],
            output='screen'
        ),

        # Launch PointCloud to LaserScan
        ExecuteProcess(
            cmd=['ros2', 'launch', 'pointcloud_to_laserscan', 'sample_pointcloud_to_laserscan_launch.py'],
            output='screen'
        ),

        # Launch SLAM Toolbox
        ExecuteProcess(
            cmd=['ros2', 'launch', 'slam_toolbox', 'offline_launch.py'],
            output='screen'
        ),
    ])

