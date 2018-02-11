# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "AnimeshConfig.h.in"
  "../include/AnimeshConfig.h"
  )

# Add the binary tree to the search path for inlcude files
file(GLOB SOURCES "*.cpp")

# Building the executable
add_executable(Animesh ${SOURCES})

target_link_libraries(Animesh libVectorAngle )

target_compile_features(Animesh PUBLIC cxx_std_11)
set_target_properties( Animesh PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
