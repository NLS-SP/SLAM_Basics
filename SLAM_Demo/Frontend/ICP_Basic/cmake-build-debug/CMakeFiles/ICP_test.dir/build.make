# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_SOURCE_DIR = /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/ICP_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ICP_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ICP_test.dir/flags.make

CMakeFiles/ICP_test.dir/src/ICP_test.cpp.o: CMakeFiles/ICP_test.dir/flags.make
CMakeFiles/ICP_test.dir/src/ICP_test.cpp.o: ../src/ICP_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ICP_test.dir/src/ICP_test.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ICP_test.dir/src/ICP_test.cpp.o -c /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/src/ICP_test.cpp

CMakeFiles/ICP_test.dir/src/ICP_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ICP_test.dir/src/ICP_test.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/src/ICP_test.cpp > CMakeFiles/ICP_test.dir/src/ICP_test.cpp.i

CMakeFiles/ICP_test.dir/src/ICP_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ICP_test.dir/src/ICP_test.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/src/ICP_test.cpp -o CMakeFiles/ICP_test.dir/src/ICP_test.cpp.s

# Object files for target ICP_test
ICP_test_OBJECTS = \
"CMakeFiles/ICP_test.dir/src/ICP_test.cpp.o"

# External object files for target ICP_test
ICP_test_EXTERNAL_OBJECTS =

ICP_test: CMakeFiles/ICP_test.dir/src/ICP_test.cpp.o
ICP_test: CMakeFiles/ICP_test.dir/build.make
ICP_test: CMakeFiles/ICP_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ICP_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ICP_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ICP_test.dir/build: ICP_test

.PHONY : CMakeFiles/ICP_test.dir/build

CMakeFiles/ICP_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ICP_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ICP_test.dir/clean

CMakeFiles/ICP_test.dir/depend:
	cd /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/cmake-build-debug /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/cmake-build-debug /Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/ICP_Basic/cmake-build-debug/CMakeFiles/ICP_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ICP_test.dir/depend
