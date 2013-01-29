# Create the core library
rosbuild_add_library (
  data_robot_core
  BumpersProcessor.cpp
  BaseTelemetryReader.cpp
  EncoderCountsProcessor.cpp
  FrontTelemetryReader.cpp
  PowerStateProcessor.cpp
  TickVelocityForwarder.cpp
  BusRequest.cpp
  bus_requests/BaseCommandRequest.cpp
  bus_requests/MotorVelocityRequest.cpp
  bus_requests/ReadBaseTelemetryRequest.cpp
  bus_requests/ReadFrontTelemetryRequest.cpp
  utils/RollOverHelpers.c
)
