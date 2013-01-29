# Create the library
rosbuild_add_library(
  differential_drive_application_services
  BaseModelSetupService.cpp
  OdometryReportingService.cpp
  TwistCommandService.cpp
)
