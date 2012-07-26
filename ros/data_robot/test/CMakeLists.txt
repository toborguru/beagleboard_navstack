include_directories(
  "message_endpoints/test_doubles"
#  "data_repositories/test_doubles"
  "core"
)

# rosbuild_add_gtest will automatically run tests but may segfault with threads.
#rosbuild_add_gtest(
rosbuild_add_executable(
  data_robot_tests
  TestRunner.cpp
  ./core/BaseTelemetryReaderTests.cpp
  ./core/RollOverHelpersTests.cpp
  ./core/BumpersProcessorTests.cpp
  ./application_services/TickVelocityCommandServiceTests.cpp
)

rosbuild_add_gtest_build_flags(data_robot_tests)

# Link the libraries
target_link_libraries(
  data_robot_tests
  data_robot_application_services
#  data_robot_message_endpoints
#  data_robot_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
)


#rosbuild_add_executable(
# rosbuild_add_gtest will automatically run tests but may segfault with threads.
#rosbuild_add_gtest(
rosbuild_add_executable(
  data_robot_ros_tests
  TestRunner.cpp
  ./message_endpoints/TickVelocityEndpointTests.cpp
  ./message_endpoints/EncoderCountsEndpointTests.cpp
)

rosbuild_add_gtest_build_flags(data_robot_ros_tests)

# Link the libraries
target_link_libraries(
  data_robot_ros_tests
#  data_robot_application_services
  data_robot_message_endpoints
#  data_robot_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
)


#rosbuild_add_executable(
# rosbuild_add_gtest will automatically run tests but may segfault with threads.
#rosbuild_add_gtest(
rosbuild_add_executable(
  data_robot_bb_tests
  TestRunner.cpp
  ./message_endpoints/I2CBusEndpointTests.cpp
)

rosbuild_add_gtest_build_flags(data_robot_bb_tests)

# Link the libraries
target_link_libraries(
  data_robot_bb_tests
#  data_robot_application_services
  data_robot_message_endpoints
#  data_robot_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
)

