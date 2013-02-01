include_directories(
  "message_endpoints/test_doubles"
  "data_repositories/test_doubles"
)

#rosbuild_add_executable(
# rosbuild_add_gtest will automatically run tests but may segfault with threads.
# Seems to be working fine for me...
rosbuild_add_gtest(
  test/differential_drive_tests
  TestRunner.cpp
  ./core/BaseModelTests.cpp
  ./core/OdometryIntegratorTests.cpp
  ./core/TwistConverterTests.cpp
  ./application_services/BaseModelSetupServiceTests.cpp
  ./application_services/OdometryReportingServiceTests.cpp
  ./application_services/TwistCommandServiceTests.cpp
)
# If using rosbuild_add_executable uncomment this line
#rosbuild_add_gtest_build_flags(test/differential_drive_tests)

# Link the libraries
target_link_libraries(
  test/differential_drive_tests
  differential_drive_application_services
  differential_drive_message_endpoints
  # Important that core comes after application_services due to direction of dependencies
  differential_drive_core
)

# Using rostest to launch a ROS environment
rosbuild_add_executable(
  test/differential_drive_ros_tests
  TestRunner.cpp
  ./message_endpoints/EncoderCountsSubscriberEndpointTests.cpp
  ./message_endpoints/OdometryPublisherEndpointTests.cpp
  ./message_endpoints/MovementStatusPublisherEndpointTests.cpp
  ./message_endpoints/TickVelocityPublisherEndpointTests.cpp
  ./message_endpoints/TwistSubscriberEndpointTests.cpp
  ./data_repositories/BaseModelRepositoryTests.cpp
)
rosbuild_add_gtest_build_flags(test/differential_drive_ros_tests)
# Make sure rostest launches test/mynode.test during "make test"
rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/differential_drive.test)

# Link the libraries
target_link_libraries(
  test/differential_drive_ros_tests
  differential_drive_application_services
  differential_drive_message_endpoints
  differential_drive_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  differential_drive_core
)
