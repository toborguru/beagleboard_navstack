include_directories(
  "message_endpoints/test_doubles"
  "data_repositories/test_doubles"
  "core"
)

#rosbuild_add_executable(
# rosbuild_add_gtest will automatically run tests but may segfault with threads.
#rosbuild_add_gtest(
rosbuild_add_executable(
  diff_drive_tests
  TestRunner.cpp
  ./core/BaseModelTests.cpp
  ./core/OdometryIntegratorTests.cpp
  ./core/TwistConverterTests.cpp
  ./application_services/BaseModelSetupServiceTests.cpp
  ./application_services/OdometryReportingServiceTests.cpp
  ./application_services/TwistCommandServiceTests.cpp
)

#rosbuild_add_executable(
# rosbuild_add_gtest will automatically run tests but may segfault with threads.
#rosbuild_add_gtest(
rosbuild_add_executable(
  diff_drive_ros_tests
  TestRunner.cpp
  ./message_endpoints/OdometryEndpointTests.cpp
  ./message_endpoints/TickVelocityEndpointTests.cpp
  ./message_endpoints/TwistEndpointTests.cpp
  ./message_endpoints/EncoderCountsEndpointTests.cpp
  ./data_repositories/BaseModelRepositoryTests.cpp
)

rosbuild_add_gtest_build_flags(diff_drive_tests)
rosbuild_add_gtest_build_flags(diff_drive_ros_tests)

# Link the libraries
target_link_libraries(
  diff_drive_tests
  diff_drive_application_services
  diff_drive_message_endpoints
  # Important that core comes after application_services due to direction of dependencies
  diff_drive_core
)

# Link the libraries
target_link_libraries(
  diff_drive_ros_tests
  diff_drive_application_services
  diff_drive_message_endpoints
  diff_drive_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  diff_drive_core
)
