cmake_minimum_required(VERSION 3.5)
project(ouster_pointcloud_slice)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(pointcloud_slice_node src/pointcloud_slice.cpp)
ament_target_dependencies(pointcloud_slice_node
  rclcpp sensor_msgs pcl_ros
)
target_link_libraries(pointcloud_slice_node ${PCL_LIBRARIES})

install(TARGETS pointcloud_slice_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/ouster_configurator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
