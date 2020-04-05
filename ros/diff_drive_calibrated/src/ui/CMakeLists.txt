add_executable(
  diff_drive_node
  DiffDriveNode.cpp
)

target_link_libraries(
  diff_drive_node
  ${catkin_LIBRARIES}
  diff_drive_application_services
  diff_drive_message_endpoints
  diff_drive_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  diff_drive_core
)
