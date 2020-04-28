cmake_minimum_required(VERSION 2.8.3)
project(data_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
	message_generation
  diff_drive_calibrated
  tf
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bumpers.msg
  PowerState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_robot
  CATKIN_DEPENDS roscpp std_msgs diff_drive_calibrated message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  "src/core"
  "src/core/bus_requests"
  "src/core/data_repository_interfaces"
  "src/core/listener_interfaces"
  "src/core/message_endpoint_interfaces"
  "src/core/utils"
  "src/data_repositories"
  "src/message_endpoints"
  "src/application_services"
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
#add_subdirectory(test)

add_dependencies(data_robot_application_services ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(data_robot_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(data_robot_data_repositories ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(data_robot_message_endpoints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/ui/DataRobotNode.cpp)

add_executable(
  bumper_wander
  src/ui/BumperWander.cpp
)

add_executable(
  static_command
  src/ui/StaticCommand.cpp
)

add_executable(
  figure_eights
  src/ui/FigureEights.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bumper_wander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(figure_eights ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  data_robot_application_services
  data_robot_data_repositories
  data_robot_message_endpoints
  # Important that core comes after application_services due to direction of dependencies
  data_robot_core
  ${catkin_LIBRARIES}
)

target_link_libraries(bumper_wander
  ${catkin_LIBRARIES}
)

target_link_libraries(static_command
  ${catkin_LIBRARIES}
)

target_link_libraries(figure_eights
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bumper_wander
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS static_command
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS figure_eights
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_data_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
