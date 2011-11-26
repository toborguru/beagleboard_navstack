FILE(REMOVE_RECURSE
  "../src/diff_drive/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/diff_drive/msg/__init__.py"
  "../src/diff_drive/msg/_TickVelocity.py"
  "../src/diff_drive/msg/_EncoderCounts.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
