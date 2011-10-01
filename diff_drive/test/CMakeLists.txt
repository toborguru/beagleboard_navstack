include_directories(
  "message_endpoints/test_doubles"
)

#rosbuild_add_executable(
# rosbuild_add_gtest will automatically run tests but may segfault with threads.
#rosbuild_add_gtest(
rosbuild_add_executable(
  diff_drive_tests
  TestRunner.cpp
  ./core/EncoderCountsReaderTests.cpp
  ./core/BaseModelTests.cpp
#  ./application_services/OdometryReportingServiceTests.cpp
#  ./message_endpoints/OdometryEndpointTests.cpp
#  ./message_endpoints/EncoderCountEndpointTests.cpp
)

rosbuild_add_gtest_build_flags(diff_drive_tests)

# Link the libraries
target_link_libraries(
  diff_drive_tests
#  diff_drive_application_services
#  diff_drive_message_endpoints
#  i2c_core
  # Important that core comes after application_services due to direction of dependencies
  diff_drive_core
)
