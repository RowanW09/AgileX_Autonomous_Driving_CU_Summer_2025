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
        # Launch RealSense Driver
	ExecuteProcess(
    	    cmd=[
            'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
            'enable_rgbd:=true',
            'enable_sync:=true',
            'align_depth.enable:=true',
            'enable_color:=true',
            'enable_depth:=true',
            'pointcloud.enable:=true',
            'tf_publish_rate:=50.0'
    	    ],
   	     output='screen'
	),
	
	# Launch RealSense Filter using ExecuteProcess
        #ExecuteProcess(
        #    cmd=[
        #        'ros2', 'launch', 'cloud_fuser', 'realsense_filter_launch.py'
        #    ],
        #    output='screen'
        #),
        
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
        # Launch cloud fuser
        ExecuteProcess(
            cmd=['ros2', 'launch', 'cloud_fuser', 'cloud_fuser_launch.py'],
            output='screen'
        ),
        
        # Launch PointCloud to LaserScan (RealSense & Livox)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'pointcloud_to_laserscan', 'sample_pointcloud_to_laserscan_ALL_launch.py'],
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
