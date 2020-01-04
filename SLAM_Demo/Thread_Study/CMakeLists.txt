cmake_minimum_required(VERSION 2.8)
project(Thread_Study)

set(CMAKE_CXX_STANDARD 14)

add_executable(machine_thread machine_thread.cpp)
add_executable(thread_function thread_function.cpp)
add_executable(hello_detach hello_detach.cpp)
add_executable(hello_function_object hello_function_object.cpp)

add_executable(hello_lambda_function hello_lambda_function.cpp)

add_executable(hello_function_parameter hello_function_parameter.cpp)