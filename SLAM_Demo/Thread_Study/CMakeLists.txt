cmake_minimum_required(VERSION 2.8)
project(Thread_Study)

set(CMAKE_CXX_STANDARD 14)

#set(BOOST_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED TRUE)
FIND_PACKAGE(Boost COMPONENTS system thread REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(hello_thread hello_thread.cpp)

add_executable(hello_thread_construct hello_thread_construct.cpp)

add_executable(hello_move_in_thread hello_move_in_thread.cpp)

add_executable(hello_mutex_example hello_mutex_example.cpp)

add_executable(hello_time_mutex hello_time_mutex.cpp)

add_executable(hello_joinable hello_joinable.cpp)

add_executable(hello_accumulate_detach hello_accumulate_detach.cpp)

add_executable(hello_mutex_object hello_mutex_object.cpp)

add_executable(hello_hierarchical_mutex hello_hierarchical_mutex.cpp)

add_executable(hello_condition_variable hello_condition_variable.cpp)

add_executable(hello_condition_queue hello_condition_queue.cpp)

add_executable(hello_async hello_async.cpp)

add_executable(hello_async_parameter hello_async_parameter.cpp)

add_executable(hello_async_multi_task hello_async_multi_task.cpp)

add_executable(hello_threadPool example_threadPool.cpp threadPool.cpp)
target_link_libraries(hello_threadPool ${Boost_LIBRARIES} ${BOOST_THREAD_LIBRARY})

add_executable(parallel_sort parallel_sort.cpp)

add_executable(parallel_accumulate parallel_accumulate.cpp)