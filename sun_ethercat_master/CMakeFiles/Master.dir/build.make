# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/asusrobot/Documents/Federico-De_Simone

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/asusrobot/Documents/Federico-De_Simone

# Include any dependencies generated for this target.
include master/CMakeFiles/Master.dir/depend.make

# Include the progress variables for this target.
include master/CMakeFiles/Master.dir/progress.make

# Include the compile flags for this target's objects.
include master/CMakeFiles/Master.dir/flags.make

master/CMakeFiles/Master.dir/Master.cpp.o: master/CMakeFiles/Master.dir/flags.make
master/CMakeFiles/Master.dir/Master.cpp.o: master/Master.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/asusrobot/Documents/Federico-De_Simone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object master/CMakeFiles/Master.dir/Master.cpp.o"
	cd /home/asusrobot/Documents/Federico-De_Simone/master && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Master.dir/Master.cpp.o -c /home/asusrobot/Documents/Federico-De_Simone/master/Master.cpp

master/CMakeFiles/Master.dir/Master.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Master.dir/Master.cpp.i"
	cd /home/asusrobot/Documents/Federico-De_Simone/master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/asusrobot/Documents/Federico-De_Simone/master/Master.cpp > CMakeFiles/Master.dir/Master.cpp.i

master/CMakeFiles/Master.dir/Master.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Master.dir/Master.cpp.s"
	cd /home/asusrobot/Documents/Federico-De_Simone/master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/asusrobot/Documents/Federico-De_Simone/master/Master.cpp -o CMakeFiles/Master.dir/Master.cpp.s

master/CMakeFiles/Master.dir/Master.cpp.o.requires:

.PHONY : master/CMakeFiles/Master.dir/Master.cpp.o.requires

master/CMakeFiles/Master.dir/Master.cpp.o.provides: master/CMakeFiles/Master.dir/Master.cpp.o.requires
	$(MAKE) -f master/CMakeFiles/Master.dir/build.make master/CMakeFiles/Master.dir/Master.cpp.o.provides.build
.PHONY : master/CMakeFiles/Master.dir/Master.cpp.o.provides

master/CMakeFiles/Master.dir/Master.cpp.o.provides.build: master/CMakeFiles/Master.dir/Master.cpp.o


# Object files for target Master
Master_OBJECTS = \
"CMakeFiles/Master.dir/Master.cpp.o"

# External object files for target Master
Master_EXTERNAL_OBJECTS =

master/Master: master/CMakeFiles/Master.dir/Master.cpp.o
master/Master: master/CMakeFiles/Master.dir/build.make
master/Master: SOEM/libsoem.a
master/Master: master/CMakeFiles/Master.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/asusrobot/Documents/Federico-De_Simone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Master"
	cd /home/asusrobot/Documents/Federico-De_Simone/master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Master.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
master/CMakeFiles/Master.dir/build: master/Master

.PHONY : master/CMakeFiles/Master.dir/build

master/CMakeFiles/Master.dir/requires: master/CMakeFiles/Master.dir/Master.cpp.o.requires

.PHONY : master/CMakeFiles/Master.dir/requires

master/CMakeFiles/Master.dir/clean:
	cd /home/asusrobot/Documents/Federico-De_Simone/master && $(CMAKE_COMMAND) -P CMakeFiles/Master.dir/cmake_clean.cmake
.PHONY : master/CMakeFiles/Master.dir/clean

master/CMakeFiles/Master.dir/depend:
	cd /home/asusrobot/Documents/Federico-De_Simone && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/asusrobot/Documents/Federico-De_Simone /home/asusrobot/Documents/Federico-De_Simone/master /home/asusrobot/Documents/Federico-De_Simone /home/asusrobot/Documents/Federico-De_Simone/master /home/asusrobot/Documents/Federico-De_Simone/master/CMakeFiles/Master.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : master/CMakeFiles/Master.dir/depend

