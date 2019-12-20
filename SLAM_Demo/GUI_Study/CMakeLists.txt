cmake_minimum_required(VERSION 2.8)
project(GUI_VERSION)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

add_executable(guiWindow mainWindow.cpp)
target_link_libraries(guiWindow Qt5::Core)
target_link_libraries(guiWindow Qt5::Widgets)