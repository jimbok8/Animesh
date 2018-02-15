cmake_minimum_required (VERSION 3.8 FATAL_ERROR)

project( Animesh
         VERSION 0.1 
         LANGUAGES CXX CUDA)


# include_directories(${CMAKE_CURRENT_SOURCE_DIR})


# Write targets to bin directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

set(CMAKE_CXX_STANDARD 11)

# Enable Test 
enable_testing( )

# Add the binary tree to the search path for inlcude files
add_subdirectory( main )
add_subdirectory( libVectorAngle )
add_subdirectory( libGraph )


# We need OpenCV 
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11
#target_link_libraries(CopyMe3D ${OpenCV_LIBS})
    
# And freenect2
#find_package(freenect2 REQUIRED)
#include_directories(${freenect2_INCLUDE_DIRS})
#target_link_libraries(CopyMe3D ${freenect2_LIBRARIES})


# And CUDA
#find_package( CUDA REQUIRED )
#include_directories(${CUDA_INCLUDE_DIRS})
#set(CMAKE_CUDA_FLAGS "-g -G -O0")


# And libpng
#find_package( PNG REQUIRED )
#include_directories(${PNG_INCLUDE_DIRECTORIES})
	
# Add TSDF
#include_directories( ${CMAKE_SOURCE_DIR}/../libsdf/include)
#target_link_libraries(CopyMe3D ${CMAKE_SOURCE_DIR}/../libsdf/library/libsdf.a ${CUDA_LIBRARIES} ${PNG_LIBRARIES})


