# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/HelloPangolin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/HelloPangolin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/HelloPangolin.dir/flags.make

CMakeFiles/HelloPangolin.dir/helloPangolin.o: CMakeFiles/HelloPangolin.dir/flags.make
CMakeFiles/HelloPangolin.dir/helloPangolin.o: ../helloPangolin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/HelloPangolin.dir/helloPangolin.o"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HelloPangolin.dir/helloPangolin.o -c /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/helloPangolin.cpp

CMakeFiles/HelloPangolin.dir/helloPangolin.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HelloPangolin.dir/helloPangolin.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/helloPangolin.cpp > CMakeFiles/HelloPangolin.dir/helloPangolin.i

CMakeFiles/HelloPangolin.dir/helloPangolin.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HelloPangolin.dir/helloPangolin.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/helloPangolin.cpp -o CMakeFiles/HelloPangolin.dir/helloPangolin.s

# Object files for target HelloPangolin
HelloPangolin_OBJECTS = \
"CMakeFiles/HelloPangolin.dir/helloPangolin.o"

# External object files for target HelloPangolin
HelloPangolin_EXTERNAL_OBJECTS =

HelloPangolin: CMakeFiles/HelloPangolin.dir/helloPangolin.o
HelloPangolin: CMakeFiles/HelloPangolin.dir/build.make
HelloPangolin: /usr/local/lib/libpangolin.dylib
HelloPangolin: /usr/local/lib/libGLEW.dylib
HelloPangolin: /usr/local/lib/libavcodec.dylib
HelloPangolin: /usr/local/lib/libavformat.dylib
HelloPangolin: /usr/local/lib/libavutil.dylib
HelloPangolin: /usr/local/lib/libswscale.dylib
HelloPangolin: /usr/local/lib/libavdevice.dylib
HelloPangolin: /usr/local/lib/libpng.dylib
HelloPangolin: /usr/lib/libz.dylib
HelloPangolin: /usr/local/lib/libjpeg.dylib
HelloPangolin: /usr/local/lib/libtiff.dylib
HelloPangolin: /usr/local/lib/libIlmImf.dylib
HelloPangolin: /usr/local/lib/libzstd.dylib
HelloPangolin: CMakeFiles/HelloPangolin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable HelloPangolin"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HelloPangolin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/HelloPangolin.dir/build: HelloPangolin

.PHONY : CMakeFiles/HelloPangolin.dir/build

CMakeFiles/HelloPangolin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/HelloPangolin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/HelloPangolin.dir/clean

CMakeFiles/HelloPangolin.dir/depend:
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/cmake-build-debug /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/cmake-build-debug /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/cmake-build-debug/CMakeFiles/HelloPangolin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/HelloPangolin.dir/depend

