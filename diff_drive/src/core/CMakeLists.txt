# Create the core library
add_library(
  diff_drive_core
  OdometryIntegrater.cpp
  TwistConverter.cpp
  BaseModel.cpp
)
