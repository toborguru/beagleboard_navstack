#add_definitions( -DLOG_MESSAGE=0 )

# Create the library
add_library(
  data_robot_message_endpoints
  BumpersPublisherEndpoint.cpp
  EncoderCountsPublisherEndpoint.cpp
  PowerStatePublisherEndpoint.cpp
  TickVelocitySubscriberEndpoint.cpp
  I2CBusRequestProcessorEndpoint.cpp
  i2c_api.c
)
