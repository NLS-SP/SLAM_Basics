# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.13.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.13.4/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/build

# Include any dependencies generated for this target.
include CMakeFiles/HelloPangolinOffScreen.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/HelloPangolinOffScreen.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/HelloPangolinOffScreen.dir/flags.make

CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.o: CMakeFiles/HelloPangolinOffScreen.dir/flags.make
CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.o: ../helloPangolinOffScreen.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.o -c /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/helloPangolinOffScreen.cpp

CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/helloPangolinOffScreen.cpp > CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.i

CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/helloPangolinOffScreen.cpp -o CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.s

# Object files for target HelloPangolinOffScreen
HelloPangolinOffScreen_OBJECTS = \
"CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.o"

# External object files for target HelloPangolinOffScreen
HelloPangolinOffScreen_EXTERNAL_OBJECTS =

HelloPangolinOffScreen: CMakeFiles/HelloPangolinOffScreen.dir/helloPangolinOffScreen.o
HelloPangolinOffScreen: CMakeFiles/HelloPangolinOffScreen.dir/build.make
HelloPangolinOffScreen: /usr/local/lib/libpangolin.dylib
HelloPangolinOffScreen: /usr/local/lib/libGLEW.dylib
HelloPangolinOffScreen: /usr/local/lib/libpng.dylib
HelloPangolinOffScreen: /usr/lib/libz.dylib
HelloPangolinOffScreen: /usr/local/lib/libjpeg.dylib
HelloPangolinOffScreen: /usr/local/lib/libtiff.dylib
HelloPangolinOffScreen: CMakeFiles/HelloPangolinOffScreen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable HelloPangolinOffScreen"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HelloPangolinOffScreen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/HelloPangolinOffScreen.dir/build: HelloPangolinOffScreen

.PHONY : CMakeFiles/HelloPangolinOffScreen.dir/build

CMakeFiles/HelloPangolinOffScreen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/HelloPangolinOffScreen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/HelloPangolinOffScreen.dir/clean

CMakeFiles/HelloPangolinOffScreen.dir/depend:
	cd /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/build /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/build /Users/gatsby/CV_Program/SLAM_Demo/Frontend/Viewer/build/CMakeFiles/HelloPangolinOffScreen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/HelloPangolinOffScreen.dir/depend
