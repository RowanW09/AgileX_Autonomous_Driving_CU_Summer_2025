Each of the following section are located in /home/scout_ws/src.

### cloud_fuser
Custom Python node to fuse Livox LiDAR and RealSense RGB-D point clouds into a unified point cloud topic for enhanced perception.

### custom_launch
Contains custom launch files that simplify or automate the startup of various ROS 2 nodes (e.g., SLAM, Nav2, LiDAR, sensor fusion).

### m-explore-ros2
Lightweight frontier-based exploration tool that allows the robot to autonomously map unknown areas by navigating to frontier regions.

### navigation2
Full ROS 2 Navigation Stack (Nav2), including global and local planners, controllers, recovery behaviors, and behavior trees.

### navigation_msgs
Message definitions used by Nav2 and other navigation-related packages (e.g., NavigateToPose, costmaps, path messages).

### robot_state_publisher
ROS 2 node that publishes the transform (TF) tree of the robot from its URDF, enabling other nodes to understand its physical configuration.

### scout_ros2
ROS 2 driver and description for the AgileX Scout Mini UGV. Includes URDF, CAN interface, and base controller integration.

### slam_toolbox
SLAM Toolbox package for 2D simultaneous localization and mapping. Used for real-time map building and localization.

### ugv_sdk
Low-level SDK provided by AgileX for CAN-based communication with the Scout Mini’s base, including velocity control and diagnostics.
