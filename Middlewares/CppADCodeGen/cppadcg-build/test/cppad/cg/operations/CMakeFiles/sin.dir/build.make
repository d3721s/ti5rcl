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
CMAKE_SOURCE_DIR = /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build

# Include any dependencies generated for this target.
include test/cppad/cg/operations/CMakeFiles/sin.dir/depend.make

# Include the progress variables for this target.
include test/cppad/cg/operations/CMakeFiles/sin.dir/progress.make

# Include the compile flags for this target's objects.
include test/cppad/cg/operations/CMakeFiles/sin.dir/flags.make

test/cppad/cg/operations/CMakeFiles/sin.dir/sin.cpp.o: test/cppad/cg/operations/CMakeFiles/sin.dir/flags.make
test/cppad/cg/operations/CMakeFiles/sin.dir/sin.cpp.o: ../test/cppad/cg/operations/sin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/cppad/cg/operations/CMakeFiles/sin.dir/sin.cpp.o"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/operations && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sin.dir/sin.cpp.o -c /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/operations/sin.cpp

test/cppad/cg/operations/CMakeFiles/sin.dir/sin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sin.dir/sin.cpp.i"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/operations && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/operations/sin.cpp > CMakeFiles/sin.dir/sin.cpp.i

test/cppad/cg/operations/CMakeFiles/sin.dir/sin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sin.dir/sin.cpp.s"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/operations && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/operations/sin.cpp -o CMakeFiles/sin.dir/sin.cpp.s

# Object files for target sin
sin_OBJECTS = \
"CMakeFiles/sin.dir/sin.cpp.o"

# External object files for target sin
sin_EXTERNAL_OBJECTS =

test/cppad/cg/operations/sin: test/cppad/cg/operations/CMakeFiles/sin.dir/sin.cpp.o
test/cppad/cg/operations/sin: test/cppad/cg/operations/CMakeFiles/sin.dir/build.make
test/cppad/cg/operations/sin: lib/libgtest.a
test/cppad/cg/operations/sin: lib/libgtest_main.a
test/cppad/cg/operations/sin: /usr/lib/x86_64-linux-gnu/libdl.so
test/cppad/cg/operations/sin: lib/libgtest.a
test/cppad/cg/operations/sin: test/cppad/cg/operations/CMakeFiles/sin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sin"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/operations && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/cppad/cg/operations/CMakeFiles/sin.dir/build: test/cppad/cg/operations/sin

.PHONY : test/cppad/cg/operations/CMakeFiles/sin.dir/build

test/cppad/cg/operations/CMakeFiles/sin.dir/clean:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/operations && $(CMAKE_COMMAND) -P CMakeFiles/sin.dir/cmake_clean.cmake
.PHONY : test/cppad/cg/operations/CMakeFiles/sin.dir/clean

test/cppad/cg/operations/CMakeFiles/sin.dir/depend:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/operations /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/operations /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/operations/CMakeFiles/sin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/cppad/cg/operations/CMakeFiles/sin.dir/depend

