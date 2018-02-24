file(GLOB SOURCES "src/*.cpp")

add_library(Field STATIC ${SOURCES})

target_include_directories(
	Field PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	${EIGEN3_INCLUDE_DIR}
	PRIVATE src)

target_link_libraries( Field 
	Element 
	Graph)

# This makes the project importable from the build directory
export(
	TARGETS Field
	FILE FieldLibraryConfig.cmake)

# Need Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})



# Tests
file( GLOB TEST_SOURCES "tests/*.cpp" )

add_executable(
	testField
    ${TEST_SOURCES})

# target_link_libraries(
# 	testField 
# 	Field 
# 	Element
# 	Graph
# 	gtest gmock
# 	${CMAKE_THREAD_LIBS_INIT} )

target_link_libraries(
	testField
	Field
	Graph 
	gtest gmock
	${CMAKE_THREAD_LIBS_INIT} )

add_test( NAME TestFieldDataTangentVectorIsPerpendicularToNormal COMMAND testField --gtest_filter=TestFieldData.TangentVectorIsPerpendicularToNormal)
add_test( NAME TestFieldDataTangentVectorIsUnit COMMAND testField --gtest_filter=TestFieldData.TangentVectorIsUnit )
add_test( NAME TestFieldDataKIsZero COMMAND testField --gtest_filter=TestFieldData.KIsZero  )
# add_test( NAME  COMMAND testField --gtest_filter=  )



# Stash it
install(TARGETS testField DESTINATION bin)

