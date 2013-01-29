rosbuild_add_executable(
  differential_drive_node
  DiffDriveNode.cpp
)

target_link_libraries(
  differential_drive_node
  differential_drive_application_services
  differential_drive_message_endpoints
  differential_drive_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  differential_drive_core
)
