# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /home/remco/cmake-install/bin/cmake

# The command to remove a file.
RM = /home/remco/cmake-install/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/build

# Include any dependencies generated for this target.
include CMakeFiles/object_plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/object_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/object_plugin.dir/flags.make

CMakeFiles/object_plugin.dir/object_plugin.cc.o: CMakeFiles/object_plugin.dir/flags.make
CMakeFiles/object_plugin.dir/object_plugin.cc.o: ../object_plugin.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/object_plugin.dir/object_plugin.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_plugin.dir/object_plugin.cc.o -c /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/object_plugin.cc

CMakeFiles/object_plugin.dir/object_plugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_plugin.dir/object_plugin.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/object_plugin.cc > CMakeFiles/object_plugin.dir/object_plugin.cc.i

CMakeFiles/object_plugin.dir/object_plugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_plugin.dir/object_plugin.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/object_plugin.cc -o CMakeFiles/object_plugin.dir/object_plugin.cc.s

# Object files for target object_plugin
object_plugin_OBJECTS = \
"CMakeFiles/object_plugin.dir/object_plugin.cc.o"

# External object files for target object_plugin
object_plugin_EXTERNAL_OBJECTS =

libobject_plugin.so: CMakeFiles/object_plugin.dir/object_plugin.cc.o
libobject_plugin.so: CMakeFiles/object_plugin.dir/build.make
libobject_plugin.so: CMakeFiles/object_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libobject_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/object_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/object_plugin.dir/build: libobject_plugin.so

.PHONY : CMakeFiles/object_plugin.dir/build

CMakeFiles/object_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/object_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/object_plugin.dir/clean

CMakeFiles/object_plugin.dir/depend:
	cd /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/build /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/build /home/remco/catkin_ws/src/delta_robot_simulation/delta_robot_plugin/build/CMakeFiles/object_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/object_plugin.dir/depend

