file(GLOB SOURCES "*.cpp")

# Need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# The test executable
add_executable(testGraph ${SOURCES})

# Required libaries
target_link_libraries(testGraph  Graph VectorAngle ${GTEST_LIBRARIES}  ${CMAKE_THREAD_LIBS_INIT} )

# Stash it
install(TARGETS testGraph DESTINATION bin)

add_test( NAME TestGraph COMMAND testGraph)
