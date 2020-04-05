# Create the library
add_library(
  diff_drive_data_repositories
  DifferentialParametersRepository.cpp
#  BaseModelRepository.cpp
#  MovementStatusParametersRepository.cpp
)

target_link_libraries(
  diff_drive_data_repositories
)

