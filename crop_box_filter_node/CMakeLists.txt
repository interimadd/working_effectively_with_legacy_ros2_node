cmake_minimum_required(VERSION 3.14)
project(pointcloud_crop_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_library(${PROJECT_NAME}_component SHARED
  src/pointcloud_crop_filter_node.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_eigen
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "pointcloud_crop_filter::PointCloudCropFilterNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_eigen
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pointcloud_crop_filter_node
    test/test_pointcloud_crop_filter_node.cpp
  )
  target_link_libraries(test_pointcloud_crop_filter_node
    ${PROJECT_NAME}_component
  )
  ament_target_dependencies(test_pointcloud_crop_filter_node
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2_ros
  )
endif()

ament_package()
