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
CMAKE_BINARY_DIR = /home/cptd/dd/despot

# Include any dependencies generated for this target.
include examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/depend.make

# Include the progress variables for this target.
include examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/progress.make

# Include the compile flags for this target's objects.
include examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/flags.make

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/flags.make
examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o: examples/cpp_models/pocman/src/pocman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cptd/dd/despot/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o"
	cd /home/cptd/dd/despot/examples/cpp_models/pocman && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/despot_pocman.dir/src/pocman.cpp.o -c /home/cptd/dd/despot/examples/cpp_models/pocman/src/pocman.cpp

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/despot_pocman.dir/src/pocman.cpp.i"
	cd /home/cptd/dd/despot/examples/cpp_models/pocman && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cptd/dd/despot/examples/cpp_models/pocman/src/pocman.cpp > CMakeFiles/despot_pocman.dir/src/pocman.cpp.i

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/despot_pocman.dir/src/pocman.cpp.s"
	cd /home/cptd/dd/despot/examples/cpp_models/pocman && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cptd/dd/despot/examples/cpp_models/pocman/src/pocman.cpp -o CMakeFiles/despot_pocman.dir/src/pocman.cpp.s

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o.requires:

.PHONY : examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o.requires

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o.provides: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o.requires
	$(MAKE) -f examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/build.make examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o.provides.build
.PHONY : examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o.provides

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o.provides.build: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o


examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/flags.make
examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o: examples/cpp_models/pocman/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cptd/dd/despot/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o"
	cd /home/cptd/dd/despot/examples/cpp_models/pocman && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/despot_pocman.dir/src/main.cpp.o -c /home/cptd/dd/despot/examples/cpp_models/pocman/src/main.cpp

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/despot_pocman.dir/src/main.cpp.i"
	cd /home/cptd/dd/despot/examples/cpp_models/pocman && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cptd/dd/despot/examples/cpp_models/pocman/src/main.cpp > CMakeFiles/despot_pocman.dir/src/main.cpp.i

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/despot_pocman.dir/src/main.cpp.s"
	cd /home/cptd/dd/despot/examples/cpp_models/pocman && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cptd/dd/despot/examples/cpp_models/pocman/src/main.cpp -o CMakeFiles/despot_pocman.dir/src/main.cpp.s

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o.requires:

.PHONY : examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o.requires

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o.provides: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o.requires
	$(MAKE) -f examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/build.make examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o.provides.build
.PHONY : examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o.provides

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o.provides.build: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o


# Object files for target despot_pocman
despot_pocman_OBJECTS = \
"CMakeFiles/despot_pocman.dir/src/pocman.cpp.o" \
"CMakeFiles/despot_pocman.dir/src/main.cpp.o"

# External object files for target despot_pocman
despot_pocman_EXTERNAL_OBJECTS =

examples/cpp_models/pocman/despot_pocman: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o
examples/cpp_models/pocman/despot_pocman: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o
examples/cpp_models/pocman/despot_pocman: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/build.make
examples/cpp_models/pocman/despot_pocman: libdespot.so
examples/cpp_models/pocman/despot_pocman: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cptd/dd/despot/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable despot_pocman"
	cd /home/cptd/dd/despot/examples/cpp_models/pocman && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/despot_pocman.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/build: examples/cpp_models/pocman/despot_pocman

.PHONY : examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/build

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/requires: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/pocman.cpp.o.requires
examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/requires: examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/src/main.cpp.o.requires

.PHONY : examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/requires

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/clean:
	cd /home/cptd/dd/despot/examples/cpp_models/pocman && $(CMAKE_COMMAND) -P CMakeFiles/despot_pocman.dir/cmake_clean.cmake
.PHONY : examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/clean

examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/depend:
	cd /home/cptd/dd/despot && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cptd/dd/despot /home/cptd/dd/despot/examples/cpp_models/pocman /home/cptd/dd/despot /home/cptd/dd/despot/examples/cpp_models/pocman /home/cptd/dd/despot/examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/cpp_models/pocman/CMakeFiles/despot_pocman.dir/depend

