cmake_minimum_required(VERSION 2.8)
project(Thread_Study)

set(CMAKE_CXX_STANDARD 14)

add_executable(hello_thread hello_thread.cpp)

add_executable(hello_joinable hello_joinable.cpp)

add_executable(hello_accumulate_detach hello_accumulate_detach.cpp)

add_executable(hello_mutex_object hello_mutex_object.cpp)

add_executable(hello_hierarchical_mutex hello_hierarchical_mutex.cpp)

add_executable(hello_condition_variable hello_condition_variable.cpp)

add_executable(hello_condition_queue hello_condition_queue.cpp)