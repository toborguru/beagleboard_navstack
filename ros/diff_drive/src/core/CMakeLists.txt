# Create the core library
rosbuild_add_library(
  diff_drive_core
  OdometryIntegrator.cpp
  TwistConverter.cpp
  BaseModel.cpp
)
