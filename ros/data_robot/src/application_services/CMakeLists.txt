# Create the library
add_library(
  data_robot_application_services
  BumpersProcessorSetupService.cpp
  EncoderCountsReportingService.cpp
  FrontTelemetryReportingService.cpp
  TickVelocityCommandService.cpp
)
