project(CERES_Study)

include_directories(include)
include_directories(CERES_Study)

set(OpenCV_DIR /usr/local/opencv3.4.1/share/OpenCV)
find_package(OpenCV 3.4.1 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.1/include)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.1/lib)

add_executable(hello_world hello_world.cpp)
target_link_libraries(hello_world ${CERES_LIBRARIES} glog::glog ${GFLAGS_LIBRARIES})

add_executable(hellow_world_numeric_diff hello_world_numeric_diff.cpp)
target_link_libraries(hellow_world_numeric_diff ${CERES_LIBRARIES} glog::glog)

add_executable(hello_world_analytic_diff hello_world_analytic_diff.cpp)
target_link_libraries(hello_world_analytic_diff ${CERES_LIBRARIES} glog::glog)

add_executable(hello_power hello_power.cpp)
target_link_libraries(hello_power ${CERES_LIBRARIES} glog::glog ${GFLAGS_LIBRARIES})

add_executable(hello_curve_fitting hello_curve_fitting.cpp)
target_link_libraries(hello_curve_fitting ${CERES_LIBRARIES} glog::glog ${GFLAGS_LIBRARIES})

add_executable(hello_robust_fitting hello_robust_fitting.cpp)
target_link_libraries(hello_robust_fitting ${CERES_LIBRARIES} glog::glog ${GFLAGS_LIBRARIES})

add_executable(hello_bundle hello_bundle.cpp bal_problem.cpp)
target_link_libraries(hello_bundle ${CERES_LIBRARIES} glog::glog ${GFLAGS_LIBRARIES})

add_executable(hello_ceres_pnp hello_ceres_pnp.cpp)
target_link_libraries(hello_ceres_pnp ${CERES_LIBRARIES} glog::glog ${GFLAGS_LIBRARIES} ${OpenCV_LIBS})