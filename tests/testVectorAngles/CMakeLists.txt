file(GLOB SOURCES "*.cpp")

# Need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# The test executable
add_executable(testVectorAngles ${SOURCES})

# Required libaries
target_link_libraries(testVectorAngles VectorAngle ${GTEST_LIBRARIES}  ${CMAKE_THREAD_LIBS_INIT} )

# Stash it
install(TARGETS testVectorAngles DESTINATION bin)

add_test( VectorAngleTest bin/testVectorAngles)
