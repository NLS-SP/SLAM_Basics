#build the test cases.
include_directories(${GTEST_INCLUDE_DIRS})


set(TEST_BINARY ${PROJECT_NAME}_test)

add_executable(${TEST_BINARY}
        main.cpp
        color-test.cpp
        buffer_test.cpp
        texture-test.cpp
        framebuffer-test.cpp)

target_link_libraries(${TEST_BINARY} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} ${THIRD_PARTY} ${PROJECT_NAME})
add_test(${TEST_BINARY} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TEST_BINARY})