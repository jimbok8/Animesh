file(GLOB SOURCES "*.cpp")

add_executable(testVectorAngles ${SOURCES})

target_link_libraries(testVectorAngles ${GTEST_LIBRARIES})

install(TARGETS testVectorAngles DESTINATION bin)