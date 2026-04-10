cmake_minimum_required(VERSION 3.14)
project(pointcloud_crop_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_executable(${PROJECT_NAME}_node
  src/pointcloud_crop_filter_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_eigen
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
