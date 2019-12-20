project(3DMap)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARIES})

add_executable(Map_Build mapBuilt.cpp)
target_link_libraries(Map_Build ${OCTOMAP_LIBRARIES})
