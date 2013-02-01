# Create the library
rosbuild_add_library(
  differential_drive_message_endpoints
  AsyncSpinnerWrapper.cpp
  EncoderCountsSubscriberEndpoint.cpp
  MovementStatusPublisherEndpoint.cpp
  OdometryPublisherEndpoint.cpp
  TickVelocityPublisherEndpoint.cpp
  TwistSubscriberEndpoint.cpp
)

target_link_libraries(
  differential_drive_message_endpoints
)

