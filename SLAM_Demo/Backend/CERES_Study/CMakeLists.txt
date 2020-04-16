project(CERES_Study)

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

add_executable(hello_bundle hello_bundle.cpp)
target_link_libraries(hello_bundle ${CERES_LIBRARIES} glog::glog ${GFLAGS_LIBRARIES})
