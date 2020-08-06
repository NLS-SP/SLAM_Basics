project(BASIC_ICP)
include_directories("/usr/local/include/eigen3")

include_directories(include)
add_executable(BASIC_ICP src/ICP.cpp)