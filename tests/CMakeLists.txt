# GTest requires pthreads
find_package( Threads REQUIRED )


# We need GTest for testing
enable_testing()
find_package(GTest REQUIRED)

# Include specific tests
include_directories("${GTEST_INCLUDE_DIRS}")

# Include all tests 
add_subdirectory( testVectorAngles )