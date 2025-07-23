from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
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
            'pointcloud.enable:=true'
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
            cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=false'],
            output='screen'
        ),

        # Launch Nav2
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                'use_sim_time:=false',
                'autostart:=true',
                'params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml'
            ],
            output='screen'
        ),

        # Launch Nav2 Collision Monitor
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_collision_monitor', 'collision_monitor_node.launch.py',
                'use_sim_time:=false'
            ],
            output='screen'
        ),

        # Launch RViz
        ExecuteProcess(
            cmd=[
                'rviz2', '-d',
                '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
            ],
            output='screen'
        ),
    ])

