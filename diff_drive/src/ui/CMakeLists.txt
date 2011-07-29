rosbuild_add_executable(
  diff_drive_i2c_node
  DiffDriveI2CNode.cpp
)

target_link_libraries(
  diff_drive_i2c_node
  i2c_core
  diff_drive_application_services
  diff_drive_message_endpoints
  # Important that core comes after application_services due to direction of dependencies
  diff_drive_core
)
