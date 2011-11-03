include_directories(
#  "message_endpoints/test_doubles"
#  "data_repositories/test_doubles"
  "core"
)

# rosbuild_add_gtest will automatically run tests but may segfault with threads.
#rosbuild_add_gtest(
rosbuild_add_executable(
  data_robot_tests
  TestRunner.cpp
  ./core/EncoderCountsReaderTests.cpp
#  ./core/TwistConverterTests.cpp
#  ./core/BaseModelTests.cpp
#  ./application_services/BaseModelSetupServiceTests.cpp
# ./application_services/OdometryReportingServiceTests.cpp
#  ./application_services/TwistCommandServiceTests.cpp
#
)

rosbuild_add_gtest_build_flags(data_robot_tests)

# Link the libraries
target_link_libraries(
  data_robot_tests
#  diff_drive_application_services
#  diff_drive_message_endpoints
#  diff_drive_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
)
