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

add_test( NAME GraphTest COMMAND testGraph)

# Stash it
install(TARGETS testGraph DESTINATION bin)

# Need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
