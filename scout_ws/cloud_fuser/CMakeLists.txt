cmake_minimum_required(VERSION 3.5)
project(cloud_fuser)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)    # for tf2_eigen
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# include PCL headers
include_directories(${PCL_INCLUDE_DIRS})

# --- V2 node (legacy) ---
add_executable(cloud_fuser_v2
  cloud_fuser/cloud_fuser_node_V2.cpp
)
ament_target_dependencies(cloud_fuser_v2
  rclcpp
  sensor_msgs
  tf2_ros
  tf2
  tf2_sensor_msgs
  message_filters
  pcl_conversions
)
target_link_libraries(cloud_fuser_v2
  ${PCL_LIBRARIES}
)

# --- V3 node (optimized) ---
add_executable(cloud_fuser_v3
  cloud_fuser/cloud_fuser_node_V3.cpp
)
ament_target_dependencies(cloud_fuser_v3
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_sensor_msgs
  tf2_eigen
  message_filters
  pcl_conversions
)
target_link_libraries(cloud_fuser_v3
  ${PCL_LIBRARIES}
  tf2_eigen::tf2_eigen
)

# install both executables
install(TARGETS
  cloud_fuser_v2
  cloud_fuser_v3
  DESTINATION lib/${PROJECT_NAME}
)

# optional Python support
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/setup.py")
  ament_python_install_package(${PROJECT_NAME})
endif()

# install any launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
