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
include Viewer/CMakeFiles/HelloPangolin.dir/depend.make

# Include the progress variables for this target.
include Viewer/CMakeFiles/HelloPangolin.dir/progress.make

# Include the compile flags for this target's objects.
include Viewer/CMakeFiles/HelloPangolin.dir/flags.make

Viewer/CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.o: Viewer/CMakeFiles/HelloPangolin.dir/flags.make
Viewer/CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.o: ../Viewer/helloPangolin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Viewer/CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.o"
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/Viewer && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.o -c /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/helloPangolin.cpp

Viewer/CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.i"
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/Viewer && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/helloPangolin.cpp > CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.i

Viewer/CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.s"
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/Viewer && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer/helloPangolin.cpp -o CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.s

# Object files for target HelloPangolin
HelloPangolin_OBJECTS = \
"CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.o"

# External object files for target HelloPangolin
HelloPangolin_EXTERNAL_OBJECTS =

Viewer/HelloPangolin: Viewer/CMakeFiles/HelloPangolin.dir/helloPangolin.cpp.o
Viewer/HelloPangolin: Viewer/CMakeFiles/HelloPangolin.dir/build.make
Viewer/HelloPangolin: /usr/local/lib/libpangolin.dylib
Viewer/HelloPangolin: /usr/local/lib/libGLEW.dylib
Viewer/HelloPangolin: /usr/local/lib/libavcodec.dylib
Viewer/HelloPangolin: /usr/local/lib/libavformat.dylib
Viewer/HelloPangolin: /usr/local/lib/libavutil.dylib
Viewer/HelloPangolin: /usr/local/lib/libswscale.dylib
Viewer/HelloPangolin: /usr/local/lib/libavdevice.dylib
Viewer/HelloPangolin: /usr/local/lib/libpng.dylib
Viewer/HelloPangolin: /usr/lib/libz.dylib
Viewer/HelloPangolin: /usr/local/lib/libjpeg.dylib
Viewer/HelloPangolin: /usr/local/lib/libtiff.dylib
Viewer/HelloPangolin: /usr/local/lib/libIlmImf.dylib
Viewer/HelloPangolin: Viewer/CMakeFiles/HelloPangolin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable HelloPangolin"
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/Viewer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HelloPangolin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Viewer/CMakeFiles/HelloPangolin.dir/build: Viewer/HelloPangolin

.PHONY : Viewer/CMakeFiles/HelloPangolin.dir/build

Viewer/CMakeFiles/HelloPangolin.dir/clean:
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/Viewer && $(CMAKE_COMMAND) -P CMakeFiles/HelloPangolin.dir/cmake_clean.cmake
.PHONY : Viewer/CMakeFiles/HelloPangolin.dir/clean

Viewer/CMakeFiles/HelloPangolin.dir/depend:
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Viewer /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/Viewer /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/cmake-build-debug/Viewer/CMakeFiles/HelloPangolin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Viewer/CMakeFiles/HelloPangolin.dir/depend
