rosbuild_add_executable(
  data_robot_node
  DataRobotNode.cpp
)

target_link_libraries(
  data_robot_node
  data_robot_application_services
  data_robot_message_endpoints
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
)
