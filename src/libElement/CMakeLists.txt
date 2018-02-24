file(GLOB SOURCES "src/*.cpp")

add_library(Element STATIC ${SOURCES})

target_include_directories(
	Element PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	${EIGEN3_INCLUDE_DIR}
	PRIVATE src)

# This makes the project importable from the build directory
export(
	TARGETS Element
	FILE ElementLibraryConfig.cmake)

# Need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Every library has unit tests, of course

# GTest requires pthreads
# find_package( Threads REQUIRED )
# find_package( GTest REQUIRED )

# Include specific tests
# include_directories("${GTEST_INCLUDE_DIRS}")

file( GLOB TEST_SOURCES "tests/*.cpp" )
add_executable(
	testElement
    ${TEST_SOURCES})

target_link_libraries(
	testElement 
	Element gtest gmock)


# Stash it
install(TARGETS testElement DESTINATION bin)

add_test( NAME TestElementZeroLengthVectorsShouldThrow COMMAND testGraph --gtest_filter=TestElement.ZeroLengthVectorsShouldThrow)
add_test( NAME TestElementNormalSetCorrectlyFromConstructor COMMAND testGraph --gtest_filter=TestElement.NormalSetCorrectlyFromConstructor)
add_test( NAME TestElementLocationSetCorrectlyFromConstructor COMMAND testGraph --gtest_filter=TestElement.LocationSetCorrectlyFromConstructor)

