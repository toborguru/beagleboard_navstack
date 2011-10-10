# Create the library
add_library(
  diff_drive_application_services
  BaseModelSetupService.cpp
  OdometryReportingService.cpp
  TwistCommandService.cpp
)
