# Create the library
add_library(
  data_robot_application_services
  BaseTelemetryReportingService.cpp
  BumpersProcessorSetupService.cpp
  FrontTelemetryReportingService.cpp
  TickVelocityCommandService.cpp
)
