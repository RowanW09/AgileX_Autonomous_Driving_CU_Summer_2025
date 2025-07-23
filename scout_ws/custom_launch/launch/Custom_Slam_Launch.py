from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_param_file_path = '/home/scoutmini2/scout_ws/src/slam_toolbox/config/mapper_params_online_async.yaml'
    return LaunchDescription([
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
            cmd=['ros2', 'launch', 'pointcloud_to_laserscan', 'sample_pointcloud_to_laserscan_Livox_Launch.py'],
            output='screen'
        ),
        
        # imu conversion
        ExecuteProcess(
    	    cmd=['ros2', 'launch', 'robot_localization', 'ekf.launch.py'],
    	    output='screen'
	),
	
	# Slam
        ExecuteProcess(
            cmd=[
                'ros2', 'launch',
                get_package_share_directory('slam_toolbox') + '/launch/online_async_launch.py',
                'use_sim_time:=false',
                'autostart:=true',
                f'params_file:={slam_param_file_path}'
                #'slam_toolbox.__log_level:=debug',
            ],
            output='screen'
        ),
    ])

