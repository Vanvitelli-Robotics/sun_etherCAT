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
include sun_scheduling/CMakeFiles/sun_scheduling.dir/depend.make

# Include the progress variables for this target.
include sun_scheduling/CMakeFiles/sun_scheduling.dir/progress.make

# Include the compile flags for this target's objects.
include sun_scheduling/CMakeFiles/sun_scheduling.dir/flags.make

sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o: sun_scheduling/CMakeFiles/sun_scheduling.dir/flags.make
sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o: sun_scheduling/src/scheduling.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/asusrobot/Documents/Federico-De_Simone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o"
	cd /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sun_scheduling.dir/src/scheduling.c.o   -c /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling/src/scheduling.c

sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sun_scheduling.dir/src/scheduling.c.i"
	cd /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling/src/scheduling.c > CMakeFiles/sun_scheduling.dir/src/scheduling.c.i

sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sun_scheduling.dir/src/scheduling.c.s"
	cd /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling/src/scheduling.c -o CMakeFiles/sun_scheduling.dir/src/scheduling.c.s

sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o.requires:

.PHONY : sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o.requires

sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o.provides: sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o.requires
	$(MAKE) -f sun_scheduling/CMakeFiles/sun_scheduling.dir/build.make sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o.provides.build
.PHONY : sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o.provides

sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o.provides.build: sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o


# Object files for target sun_scheduling
sun_scheduling_OBJECTS = \
"CMakeFiles/sun_scheduling.dir/src/scheduling.c.o"

# External object files for target sun_scheduling
sun_scheduling_EXTERNAL_OBJECTS =

sun_scheduling/libsun_scheduling.a: sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o
sun_scheduling/libsun_scheduling.a: sun_scheduling/CMakeFiles/sun_scheduling.dir/build.make
sun_scheduling/libsun_scheduling.a: sun_scheduling/CMakeFiles/sun_scheduling.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/asusrobot/Documents/Federico-De_Simone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libsun_scheduling.a"
	cd /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling && $(CMAKE_COMMAND) -P CMakeFiles/sun_scheduling.dir/cmake_clean_target.cmake
	cd /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sun_scheduling.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sun_scheduling/CMakeFiles/sun_scheduling.dir/build: sun_scheduling/libsun_scheduling.a

.PHONY : sun_scheduling/CMakeFiles/sun_scheduling.dir/build

sun_scheduling/CMakeFiles/sun_scheduling.dir/requires: sun_scheduling/CMakeFiles/sun_scheduling.dir/src/scheduling.c.o.requires

.PHONY : sun_scheduling/CMakeFiles/sun_scheduling.dir/requires

sun_scheduling/CMakeFiles/sun_scheduling.dir/clean:
	cd /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling && $(CMAKE_COMMAND) -P CMakeFiles/sun_scheduling.dir/cmake_clean.cmake
.PHONY : sun_scheduling/CMakeFiles/sun_scheduling.dir/clean

sun_scheduling/CMakeFiles/sun_scheduling.dir/depend:
	cd /home/asusrobot/Documents/Federico-De_Simone && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/asusrobot/Documents/Federico-De_Simone /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling /home/asusrobot/Documents/Federico-De_Simone /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling /home/asusrobot/Documents/Federico-De_Simone/sun_scheduling/CMakeFiles/sun_scheduling.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sun_scheduling/CMakeFiles/sun_scheduling.dir/depend

