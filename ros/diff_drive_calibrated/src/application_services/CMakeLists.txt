# Create the library
add_library(
  diff_drive_application_services
  ParametersSetupService.cpp
  OdometryReportingService.cpp
  TwistCommandService.cpp
)
