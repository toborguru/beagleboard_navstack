# Create the core library
add_library (
  data_robot_core
  BumpersProcessor.cpp
  EncoderCountsReader.cpp
  FrontTelemetryReader.cpp
  TickVelocityForwarder.cpp
  bus_requests/BusRequest.cpp
  bus_requests/BaseCommandRequest.cpp
  bus_requests/MotorVelocityRequest.cpp
  bus_requests/ReadEncodersRequest.cpp
  bus_requests/ReadFrontTelemetryRequest.cpp
  utils/RollOverHelpers.c
)
