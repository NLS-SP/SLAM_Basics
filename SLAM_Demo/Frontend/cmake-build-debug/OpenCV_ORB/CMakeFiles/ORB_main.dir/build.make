# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug

# Include any dependencies generated for this target.
include OpenCV_ORB/CMakeFiles/ORB_main.dir/depend.make

# Include the progress variables for this target.
include OpenCV_ORB/CMakeFiles/ORB_main.dir/progress.make

# Include the compile flags for this target's objects.
include OpenCV_ORB/CMakeFiles/ORB_main.dir/flags.make

OpenCV_ORB/CMakeFiles/ORB_main.dir/ORB_main.cpp.o: OpenCV_ORB/CMakeFiles/ORB_main.dir/flags.make
OpenCV_ORB/CMakeFiles/ORB_main.dir/ORB_main.cpp.o: ../OpenCV_ORB/ORB_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object OpenCV_ORB/CMakeFiles/ORB_main.dir/ORB_main.cpp.o"
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/OpenCV_ORB && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ORB_main.dir/ORB_main.cpp.o -c /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/OpenCV_ORB/ORB_main.cpp

OpenCV_ORB/CMakeFiles/ORB_main.dir/ORB_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ORB_main.dir/ORB_main.cpp.i"
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/OpenCV_ORB && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/OpenCV_ORB/ORB_main.cpp > CMakeFiles/ORB_main.dir/ORB_main.cpp.i

OpenCV_ORB/CMakeFiles/ORB_main.dir/ORB_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ORB_main.dir/ORB_main.cpp.s"
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/OpenCV_ORB && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/OpenCV_ORB/ORB_main.cpp -o CMakeFiles/ORB_main.dir/ORB_main.cpp.s

# Object files for target ORB_main
ORB_main_OBJECTS = \
"CMakeFiles/ORB_main.dir/ORB_main.cpp.o"

# External object files for target ORB_main
ORB_main_EXTERNAL_OBJECTS =

OpenCV_ORB/ORB_main: OpenCV_ORB/CMakeFiles/ORB_main.dir/ORB_main.cpp.o
OpenCV_ORB/ORB_main: OpenCV_ORB/CMakeFiles/ORB_main.dir/build.make
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_dnn.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_highgui.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_ml.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_objdetect.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_shape.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_stitching.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_superres.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_videostab.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_viz.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_calib3d.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_features2d.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_flann.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_photo.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_video.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_videoio.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_imgcodecs.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_imgproc.3.4.8.dylib
OpenCV_ORB/ORB_main: /usr/local/opencv3.4.8/lib/libopencv_core.3.4.8.dylib
OpenCV_ORB/ORB_main: OpenCV_ORB/CMakeFiles/ORB_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ORB_main"
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/OpenCV_ORB && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ORB_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
OpenCV_ORB/CMakeFiles/ORB_main.dir/build: OpenCV_ORB/ORB_main

.PHONY : OpenCV_ORB/CMakeFiles/ORB_main.dir/build

OpenCV_ORB/CMakeFiles/ORB_main.dir/clean:
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/OpenCV_ORB && $(CMAKE_COMMAND) -P CMakeFiles/ORB_main.dir/cmake_clean.cmake
.PHONY : OpenCV_ORB/CMakeFiles/ORB_main.dir/clean

OpenCV_ORB/CMakeFiles/ORB_main.dir/depend:
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/OpenCV_ORB /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/OpenCV_ORB /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/OpenCV_ORB/CMakeFiles/ORB_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : OpenCV_ORB/CMakeFiles/ORB_main.dir/depend
