cmake_minimum_required(VERSION 2.8)
project(Rescursive_study)

set(CMAKE_MACOSX_RPATH 1)

add_executable(maze_test maze_test.cpp)
add_executable(hanoi_problem hanoi_problem.cpp)
add_executable(hanoi_problem_stack hanoi_problem_stack.cpp)
add_executable(checkboard_separation checkboard_separation.cpp)

