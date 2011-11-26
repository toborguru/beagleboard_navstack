# Create the library
add_library(
  data_robot_message_endpoints
  BumpersEndpoint.cpp
  EncoderCountsEndpoint.cpp
  TickVelocityEndpoint.cpp
  I2CBusEndpoint.cpp
  Crc8.c
  i2c-api.c
)

target_link_libraries(
  data_robot_message_endpoints
)

