# Create the library
rosbuild_add_library(
  diff_drive_data_repositories
  BaseModelRepository.cpp
)

target_link_libraries(
  diff_drive_data_repositories
)

