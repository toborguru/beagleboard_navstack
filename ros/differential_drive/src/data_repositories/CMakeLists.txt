# Create the library
rosbuild_add_library(
  differential_drive_data_repositories
  BaseModelRepository.cpp
#  MovementStatusParametersRepository.cpp
)

target_link_libraries(
  differential_drive_data_repositories
)

