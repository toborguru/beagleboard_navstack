# Create the core library
add_library(
  diff_drive_core
  OdometryIntegrator.cpp
  TwistConverter.cpp
  BaseModel.cpp
)
