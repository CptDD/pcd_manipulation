# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/cptd/dd/despot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cptd/dd/despot/build

# Include any dependencies generated for this target.
include examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/depend.make

# Include the progress variables for this target.
include examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/progress.make

# Include the compile flags for this target's objects.
include examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/flags.make

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o: examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/flags.make
examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o: ../examples/pomdpx_models/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cptd/dd/despot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o"
	cd /home/cptd/dd/despot/build/examples/pomdpx_models && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/despot_pomdpx.dir/src/main.cpp.o -c /home/cptd/dd/despot/examples/pomdpx_models/src/main.cpp

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/despot_pomdpx.dir/src/main.cpp.i"
	cd /home/cptd/dd/despot/build/examples/pomdpx_models && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cptd/dd/despot/examples/pomdpx_models/src/main.cpp > CMakeFiles/despot_pomdpx.dir/src/main.cpp.i

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/despot_pomdpx.dir/src/main.cpp.s"
	cd /home/cptd/dd/despot/build/examples/pomdpx_models && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cptd/dd/despot/examples/pomdpx_models/src/main.cpp -o CMakeFiles/despot_pomdpx.dir/src/main.cpp.s

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o.requires:

.PHONY : examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o.requires

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o.provides: examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o.requires
	$(MAKE) -f examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/build.make examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o.provides.build
.PHONY : examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o.provides

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o.provides.build: examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o


# Object files for target despot_pomdpx
despot_pomdpx_OBJECTS = \
"CMakeFiles/despot_pomdpx.dir/src/main.cpp.o"

# External object files for target despot_pomdpx
despot_pomdpx_EXTERNAL_OBJECTS =

examples/pomdpx_models/despot_pomdpx: examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o
examples/pomdpx_models/despot_pomdpx: examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/build.make
examples/pomdpx_models/despot_pomdpx: libdespot.so
examples/pomdpx_models/despot_pomdpx: examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cptd/dd/despot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable despot_pomdpx"
	cd /home/cptd/dd/despot/build/examples/pomdpx_models && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/despot_pomdpx.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/build: examples/pomdpx_models/despot_pomdpx

.PHONY : examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/build

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/requires: examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/src/main.cpp.o.requires

.PHONY : examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/requires

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/clean:
	cd /home/cptd/dd/despot/build/examples/pomdpx_models && $(CMAKE_COMMAND) -P CMakeFiles/despot_pomdpx.dir/cmake_clean.cmake
.PHONY : examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/clean

examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/depend:
	cd /home/cptd/dd/despot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cptd/dd/despot /home/cptd/dd/despot/examples/pomdpx_models /home/cptd/dd/despot/build /home/cptd/dd/despot/build/examples/pomdpx_models /home/cptd/dd/despot/build/examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/pomdpx_models/CMakeFiles/despot_pomdpx.dir/depend

