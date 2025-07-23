from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    slam_param_file_path = '/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async_RS.yaml'

    return LaunchDescription([
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
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'cloud_fuser', 'realsense_filter_launch.py'
            ],
            output='screen'
        ),
	
	# Launch Livox Driver (IMU)
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
            cmd=['ros2', 'launch', 'pointcloud_to_laserscan', 'sample_pointcloud_to_laserscan_realsense_launch.py'],
            output='screen'
        ),
        
        # imu conversion
        ExecuteProcess(
    	    cmd=['ros2', 'launch', 'robot_localization', 'ekf.launch.py'],
    	    output='screen'
	),

        # Launch SLAM Toolbox
        ExecuteProcess(
            cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            f'params_file:={slam_param_file_path}'
            ],
            output='screen'
        ),
    ])

