from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_file_path = '/home/scoutmini2/Desktop/Full_Map.yaml'
    #map_file_path = '/home/scoutmini2/Desktop/Map_CAMP_2c_Edited.yaml'
    rviz_config_path = '/home/scoutmini2/scout_ws/src/navigation2/nav2_bringup/rviz/nav2_default_view.rviz'
    nav2_param_file_path = '/home/scoutmini2/scout_ws/src/navigation2/nav2_bringup/params/nav2_Livox_params.yaml'

    return LaunchDescription([
        # Livox LiDAR
        ExecuteProcess(
            cmd=['ros2', 'launch', 'livox_ros_driver2', 'rviz_HAP_launch.py'],
            output='screen'
        ),

        # Scout base node
        ExecuteProcess(
            cmd=['ros2', 'launch', 'scout_base', 'scout_base.launch.py'],
            output='screen'
        ),

        # Scout base description
        ExecuteProcess(
            cmd=['ros2', 'launch', 'scout_description', 'scout_base_description.launch.py'],
            output='screen'
        ),

        # PointCloud to LaserScan
        ExecuteProcess(
            cmd=['ros2', 'launch', 'pointcloud_to_laserscan', 'sample_pointcloud_to_laserscan_Livox_Launch.py'],
            output='screen'
        ),
        
        # imu conversion
        ExecuteProcess(
    	    cmd=['ros2', 'launch', 'robot_localization', 'ekf.launch.py'],
    	    output='screen'
	),

        # Nav2 localization (with saved map)
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'localization_launch.py',
                'use_sim_time:=false',
                f'map:={map_file_path}',
                'autostart:=true'#,
                #f'params_file:={nav2_param_file_path}'
            ],
            output='screen'
        ),

        # Nav2 navigation
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                'use_sim_time:=false',
                'map_subscribe_transient_local:=true'#,
                #f'params_file:={nav2_param_file_path}'
            ],
            output='screen'
        ),

        # Collision Monitor
        ExecuteProcess(
            cmd=['ros2', 'launch', 'nav2_collision_monitor', 'collision_monitor_node.launch.py', 'use_sim_time:=false'],
            output='screen'
        ),

        # RViz2 with Nav2 config
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),
    ])

