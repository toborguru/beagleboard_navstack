# Create the library
add_library(
  diff_drive_message_endpoints
  EncoderCountsEndpoint.cpp
  MovementStatusEndpoint.cpp
  OdometryEndpoint.cpp
  TickVelocityEndpoint.cpp
  TwistEndpoint.cpp
)

target_link_libraries(
  diff_drive_message_endpoints
)

