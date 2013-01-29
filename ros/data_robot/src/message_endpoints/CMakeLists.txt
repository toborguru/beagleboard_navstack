# Create the library
rosbuild_add_library(
  data_robot_message_endpoints
  BumpersPublisherEndpoint.cpp
  EncoderCountsPublisherEndpoint.cpp
  PowerStatePublisherEndpoint.cpp
  TickVelocitySubscriberEndpoint.cpp
  I2CBusRequestProcessorEndpoint.cpp
  Crc8.c
  i2c-api.c
)

target_link_libraries(
  data_robot_message_endpoints
)

