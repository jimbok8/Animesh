# Need spdlog
set(Fgt_DIR "${PROJECT_SOURCE_DIR}/third_party/build/fgt/lib/cmake/fgt")
set(SPDLOG_ROOT "/usr/local/include/spdlog")
find_package(spdlog REQUIRED)

add_library(
		Surfel SHARED
		src/Surfel.cpp include/Surfel/Surfel.h
		src/Pixel.cpp include/Surfel/Pixel.h
		src/PixelInFrame.cpp include/Surfel/PixelInFrame.h
		src/FrameData.cpp include/Surfel/FrameData.h
		src/Surfel_Compute.cpp include/Surfel/Surfel_Compute.h
		src/Surfel_IO.cpp include/Surfel/Surfel_IO.h
)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(
		Surfel
		PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
		${EIGEN3_INCLUDE_DIR}
		PRIVATE include/Surfel
)

target_link_libraries(
		Surfel
		DepthMap
		Geom
		GeomFileUtils
		Properties
)

# Every library has unit tests, of course
add_executable(
		testSurfel
		tests/main.cpp
		tests/TestSurfel.h tests/TestSurfel.cpp
)

target_link_libraries(
		testSurfel
		Surfel
		gtest
		gmock
)

add_test(
		NAME TestSurfel.NameOfTest
		COMMAND testSurtfel --gtest_filter=TestSurfel.NameOfTest
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Stash it
install(
		TARGETS testSurfel
		DESTINATION bin
)

