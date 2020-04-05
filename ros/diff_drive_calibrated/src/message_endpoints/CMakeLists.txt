# Create the library
add_library(
  diff_drive_message_endpoints
  EncoderCountsSubscriberEndpoint.cpp
  MovementStatusPublisherEndpoint.cpp
  OdometryPublisherEndpoint.cpp
  TickVelocityPublisherEndpoint.cpp
  TwistSubscriberEndpoint.cpp
)

target_link_libraries(
  diff_drive_message_endpoints
)

