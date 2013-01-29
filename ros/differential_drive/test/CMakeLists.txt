include_directories(
  "message_endpoints/test_doubles"
  "data_repositories/test_doubles"
  "core"
)

#rosbuild_add_executable(
# rosbuild_add_gtest will automatically run tests but may segfault with threads.
#rosbuild_add_gtest(
rosbuild_add_executable(
  differential_drive_tests
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
  differential_drive_ros_tests
  TestRunner.cpp
  ./message_endpoints/EncoderCountsSubscriberEndpointTests.cpp
  ./message_endpoints/OdometryPublisherEndpointTests.cpp
  ./message_endpoints/MovementStatusPublisherEndpointTests.cpp
  ./message_endpoints/TickVelocityPublisherEndpointTests.cpp
  ./message_endpoints/TwistSubscriberEndpointTests.cpp
  ./data_repositories/BaseModelRepositoryTests.cpp
)

rosbuild_add_gtest_build_flags(differential_drive_tests)
rosbuild_add_gtest_build_flags(differential_drive_ros_tests)

# Link the libraries
target_link_libraries(
  differential_drive_tests
  differential_drive_application_services
  differential_drive_message_endpoints
  # Important that core comes after application_services due to direction of dependencies
  differential_drive_core
)

# Link the libraries
target_link_libraries(
  differential_drive_ros_tests
  differential_drive_application_services
  differential_drive_message_endpoints
  differential_drive_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  differential_drive_core
)
