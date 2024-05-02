# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "src/CMakeFiles/camera_kinova_cpp_autogen.dir/AutogenUsed.txt"
  "src/CMakeFiles/camera_kinova_cpp_autogen.dir/ParseCache.txt"
  "src/camera_kinova_cpp_autogen"
  )
endif()
