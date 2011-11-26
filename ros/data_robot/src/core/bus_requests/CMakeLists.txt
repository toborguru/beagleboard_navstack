# Create the core library
add_library (
  data_robot_core
  BusRequest.cpp
  BaseCommandRequest.cpp
  MotorVelocityRequest.cpp
  ReadEncodersRequest.cpp
  ReadFrontTelemetryRequest.cpp
)
