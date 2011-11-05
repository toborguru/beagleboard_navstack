# Create the core library
add_library (
  data_robot_core
  BusRequest.cpp
  ReadEncodersRequest.cpp
  MotorVelocityRequest.cpp
  EncoderCountsReader.cpp
  RollOverHelpers.c
)
