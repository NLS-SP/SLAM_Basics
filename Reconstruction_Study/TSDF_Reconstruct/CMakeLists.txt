cmake_minimum_required(VERSION 2.8)
project(TSDF_Reconstruct)

include_directories("/usr/local/include/eigen3")

include_directories(include)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

############################
### JSON CPP
############################
find_package(JSONCPP REQUIRED)
#pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES} /usr/local/lib/libjsoncpp.dylib)

file(GLOB SOURCE_FILES src/*.cpp)
add_executable(TSDF_Representation ${SOURCE_FILES})
target_link_libraries(TSDF_Representation ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})

