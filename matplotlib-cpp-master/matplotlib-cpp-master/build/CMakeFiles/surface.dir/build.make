# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/build

# Include any dependencies generated for this target.
include CMakeFiles/surface.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/surface.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/surface.dir/flags.make

CMakeFiles/surface.dir/examples/surface.cpp.o: CMakeFiles/surface.dir/flags.make
CMakeFiles/surface.dir/examples/surface.cpp.o: ../examples/surface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/surface.dir/examples/surface.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/surface.dir/examples/surface.cpp.o -c /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/examples/surface.cpp

CMakeFiles/surface.dir/examples/surface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/surface.dir/examples/surface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/examples/surface.cpp > CMakeFiles/surface.dir/examples/surface.cpp.i

CMakeFiles/surface.dir/examples/surface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/surface.dir/examples/surface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/examples/surface.cpp -o CMakeFiles/surface.dir/examples/surface.cpp.s

# Object files for target surface
surface_OBJECTS = \
"CMakeFiles/surface.dir/examples/surface.cpp.o"

# External object files for target surface
surface_EXTERNAL_OBJECTS =

bin/surface: CMakeFiles/surface.dir/examples/surface.cpp.o
bin/surface: CMakeFiles/surface.dir/build.make
bin/surface: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
bin/surface: CMakeFiles/surface.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/surface"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/surface.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/surface.dir/build: bin/surface

.PHONY : CMakeFiles/surface.dir/build

CMakeFiles/surface.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/surface.dir/cmake_clean.cmake
.PHONY : CMakeFiles/surface.dir/clean

CMakeFiles/surface.dir/depend:
	cd /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/build /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/build /home/sia/workspace/PatrolRob_bag/matplotlib-cpp-master/matplotlib-cpp-master/build/CMakeFiles/surface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/surface.dir/depend

