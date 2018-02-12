file(GLOB SOURCES "*.cpp")

# Need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(VectorAngle STATIC ${SOURCES})

target_include_directories(VectorAngle PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> ${EIGEN3_INCLUDE_DIR})