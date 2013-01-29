# Create the core library
rosbuild_add_library(
  differential_drive_core
  OdometryIntegrator.cpp
  TwistConverter.cpp
  BaseModel.cpp
)
