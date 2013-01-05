# Create the library
rosbuild_add_library(
  data_robot_data_repositories
  BumperIndexesRepository.cpp
)

target_link_libraries(
  data_robot_data_repositories
)

