cmake_minimum_required(VERSION 3.8)
project(uav_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(uav_bridge_node src/uav_bridge_node.cpp)
add_executable(mock_mavros_pose_node src/mock_mavros_pose_node.cpp)
add_executable(fcu_link_monitor_node src/fcu_link_monitor_node.cpp)
add_executable(failsafe_manager_node src/failsafe_manager_node.cpp)
add_executable(flight_state_machine_node src/flight_state_machine_node.cpp)

ament_target_dependencies(uav_bridge_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(mock_mavros_pose_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(fcu_link_monitor_node
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(failsafe_manager_node
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(flight_state_machine_node
  rclcpp
  std_msgs
)

install(TARGETS
  uav_bridge_node
  mock_mavros_pose_node
  fcu_link_monitor_node
  failsafe_manager_node
  flight_state_machine_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
