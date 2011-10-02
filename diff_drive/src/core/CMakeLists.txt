# Create the core library
add_library(
  diff_drive_core
  EncoderCountsReader.cpp
  TwistReader.cpp
  BaseModel.cpp
)
