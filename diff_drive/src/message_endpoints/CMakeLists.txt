# Create the library
add_library(
  diff_drive_message_endpoints
  OdometryEndpoint.cpp
  EncoderCountEndpoint.cpp
)

target_link_libraries(
  diff_drive_message_endpoints
)

