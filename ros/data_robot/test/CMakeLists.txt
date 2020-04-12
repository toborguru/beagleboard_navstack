include_directories(
  "message_endpoints/test_doubles"
)

add_gtest(
  data_robot_tests
  TestRunner.cpp
  ./core/BaseTelemetryReaderTests.cpp
  ./core/RollOverHelpersTests.cpp
  ./core/BumpersProcessorTests.cpp
  ./application_services/TickVelocityCommandServiceTests.cpp
)

# I am specifically causing a variety of overflow conditions in the RollOverHelpers test.
add_compile_flags(data_robot_tests "-Wno-overflow")

# Link the libraries
target_link_libraries(
  data_robot_tests
  data_robot_application_services
#  data_robot_message_endpoints
#  data_robot_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
)

# rosbuild_add_gtest will automatically run tests but may segfault with threads.
# This ^ only seems to apply for ROS executables, like this one
#add_executable(
add_gtest(
  data_robot_ros_tests
  TestRunner.cpp
  ./message_endpoints/TickVelocitySubscriberEndpointTests.cpp
  ./message_endpoints/EncoderCountsPublisherEndpointTests.cpp
)
# If using rosbuild_add_executable uncomment this line
#rosbuild_add_gtest_build_flags(data_robot_ros_tests)
#rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/ros_message_endpoints.test)

# Link the libraries
target_link_libraries(
  data_robot_ros_tests
#  data_robot_application_services
  data_robot_message_endpoints
#  data_robot_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
)

# We only want to run this test on the actual robot.
#rosbuild_add_executable(
add_gtest(
  data_robot_bb_tests
  TestRunner.cpp
  ./message_endpoints/I2CBusRequestProcessorEndpointTests.cpp
)
#rosbuild_add_gtest_build_flags(data_robot_bb_tests)

# Link the libraries
target_link_libraries(
  data_robot_bb_tests
#  data_robot_application_services
  data_robot_message_endpoints
#  data_robot_data_repositories
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
)

