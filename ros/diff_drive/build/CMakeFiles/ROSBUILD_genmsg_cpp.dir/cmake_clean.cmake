FILE(REMOVE_RECURSE
  "../src/diff_drive/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/diff_drive/TickVelocity.h"
  "../msg_gen/cpp/include/diff_drive/EncoderCounts.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)