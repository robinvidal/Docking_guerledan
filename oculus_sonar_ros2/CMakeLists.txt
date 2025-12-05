cmake_minimum_required(VERSION 3.8)
project(oculus_sonar_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(oculus_driver REQUIRED)  

add_executable(oculus_sonar_node
  src/oculus_sonar_node.cpp
)

target_include_directories(oculus_sonar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(oculus_sonar_node
  oculus_driver
)

ament_target_dependencies(oculus_sonar_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  oculus_sonar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
