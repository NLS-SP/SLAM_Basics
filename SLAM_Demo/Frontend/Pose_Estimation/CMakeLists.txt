project(Pose_Estimation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE DEBUG)

add_executable(opcv_poseEstimation2d2d OpenCV_poseEstimation_2d2d.cpp)
target_link_libraries(opcv_poseEstimation2d2d ${OpenCV_LIBS})

add_executable(opcv_poseEstimation2d3d Opencv_poseEstimation_2d3d.cpp)
target_link_libraries(opcv_poseEstimation2d3d ${OpenCV_LIBS} ${THIRD_PARTY_LIBS})

add_executable(opcv_poseEstimation3d3d OpenCV_poseEstimation_3d3d.cpp)
target_link_libraries(opcv_poseEstimation3d3d ${OpenCV_LIBS})