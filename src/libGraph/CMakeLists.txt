file(GLOB SOURCES "src/*.cpp")

add_library(Graph STATIC ${SOURCES})

target_include_directories(
	Graph PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	${EIGEN3_INCLUDE_DIR}
	PRIVATE src)

# This makes the project importable from the build directory
export(
	TARGETS Graph
	FILE GraphLibraryConfig.cmake)


# Every library has unit tests, of course

# GTest requires pthreads
find_package( Threads REQUIRED )
find_package( GTest REQUIRED )

# Include specific tests
include_directories("${GTEST_INCLUDE_DIRS}")

file( GLOB TEST_SOURCES "tests/*.cpp" )
add_executable(
	testGraph
    ${TEST_SOURCES})

target_link_libraries(
	testGraph Graph ${GTEST_LIBRARIES}  ${CMAKE_THREAD_LIBS_INIT} )

add_test( NAME TestGraphZeroLengthVectorsShouldThrow COMMAND testGraph --gtest_filter=ZeroLengthVectorsShouldThrow)
add_test( NAME TestGraphNormalSetCorrectlyFromConstructor COMMAND testGraph --gtest_filter=TestNormalSetCorrectlyFromConstructor)
add_test( NAME TestGraphPointSetCorrectlyFromConstructor COMMAND testGraph --gtest_filter=TestPointSetCorrectlyFromConstructor)
add_test( NAME TestGraphPointsAndNormalsWithDifferentSizeShouldThrow COMMAND testGraph --gtest_filter=TestPointsAndNormalsWithDifferentSizeShouldThrow)
add_test( NAME TestGraphNeighboursIncorrectlySizedShouldThrow COMMAND testGraph --gtest_filter=TestNeighboursIncorrectlySizedShouldThrow)
add_test( NAME TestGraphSizeIsCorrectlySet COMMAND testGraph --gtest_filter=TestSizeIsCorrectlySet)

# Stash it
install(TARGETS testGraph DESTINATION bin)

# Need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
