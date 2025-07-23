from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_param_file_path = '/home/scoutmini2/scout_ws/src/navigation2/nav2_bringup/params/nav2_Livox_params.yaml'
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

        # Launch IMU Conversion
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robot_localization', 'ekf.launch.py',
                'use_sim_time:=false',
            ],
            output='screen'
        ),

        # Launch SLAM Toolbox
        ExecuteProcess(
            cmd=[
                'ros2', 'launch',
                get_package_share_directory('slam_toolbox') + '/launch/online_async_launch.py',
                'use_sim_time:=false',
                f'params_file:={slam_param_file_path}'
                #'slam_toolbox.__log_level:=debug',
            ],
            output='screen'
        ),
        
        # Launch Nav2
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                'use_sim_time:=false',
                'autostart:=true',
                f'params_file:={nav2_param_file_path}'
            ],
            output='screen'
        ),

        # Launch Nav2 Collision Monitor
        ExecuteProcess(
            cmd=[
                'ros2', 'launch',
                'nav2_collision_monitor', 'collision_monitor_node.launch.py',
                'use_sim_time:=false'
            ],
            output='screen'
        ),

        # Launch RViz
        ExecuteProcess(
            cmd=[
                'rviz2', '-d',
                get_package_share_directory('nav2_bringup') + '/rviz/nav2_default_view.rviz'
            ],
            output='screen'
        ),
    ])
