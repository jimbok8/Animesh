file(GLOB SOURCES "src/*.cpp")

add_library(VectorAngle STATIC ${SOURCES})


# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(
	VectorAngle PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	${EIGEN3_INCLUDE_DIR}
	PRIVATE src)

# This makes the project importable from the build directory
export(
	TARGETS VectorAngle
	FILE VectorAngleLibraryConfig.cmake)


# Every library has unit tests, of course

# GTest requires pthreads
find_package( Threads REQUIRED )
find_package( GTest REQUIRED )

# Include specific tests
include_directories("${GTEST_INCLUDE_DIRS}")

file( GLOB TEST_SOURCES "tests/*.cpp" )
add_executable(
	testVectorAngles
    ${TEST_SOURCES})

target_link_libraries(
	testVectorAngles VectorAngle ${GTEST_LIBRARIES}  ${CMAKE_THREAD_LIBS_INIT} )

add_test( NAME VectorAngleTest COMMAND testVectorAngles)


# Need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# Stash it
install(TARGETS testVectorAngles DESTINATION bin)

