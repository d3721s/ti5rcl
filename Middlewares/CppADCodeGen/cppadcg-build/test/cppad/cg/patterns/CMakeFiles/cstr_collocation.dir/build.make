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
include test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/depend.make

# Include the progress variables for this target.
include test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/progress.make

# Include the compile flags for this target's objects.
include test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/flags.make

test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.o: test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/flags.make
test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.o: ../test/cppad/cg/patterns/cstr_collocation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.o"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/patterns && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.o -c /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/patterns/cstr_collocation.cpp

test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.i"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/patterns && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/patterns/cstr_collocation.cpp > CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.i

test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.s"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/patterns && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/patterns/cstr_collocation.cpp -o CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.s

# Object files for target cstr_collocation
cstr_collocation_OBJECTS = \
"CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.o"

# External object files for target cstr_collocation
cstr_collocation_EXTERNAL_OBJECTS =

test/cppad/cg/patterns/cstr_collocation: test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/cstr_collocation.cpp.o
test/cppad/cg/patterns/cstr_collocation: test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/build.make
test/cppad/cg/patterns/cstr_collocation: lib/libgtest.a
test/cppad/cg/patterns/cstr_collocation: lib/libgtest_main.a
test/cppad/cg/patterns/cstr_collocation: /usr/lib/x86_64-linux-gnu/libdl.so
test/cppad/cg/patterns/cstr_collocation: lib/libgtest.a
test/cppad/cg/patterns/cstr_collocation: test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cstr_collocation"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/patterns && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cstr_collocation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/build: test/cppad/cg/patterns/cstr_collocation

.PHONY : test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/build

test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/clean:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/patterns && $(CMAKE_COMMAND) -P CMakeFiles/cstr_collocation.dir/cmake_clean.cmake
.PHONY : test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/clean

test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/depend:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/patterns /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/patterns /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/cppad/cg/patterns/CMakeFiles/cstr_collocation.dir/depend
