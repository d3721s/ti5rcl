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
include example/CMakeFiles/source_generation_latex.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/source_generation_latex.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/source_generation_latex.dir/flags.make

example/CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.o: example/CMakeFiles/source_generation_latex.dir/flags.make
example/CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.o: ../example/source_generation_latex.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.o"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.o -c /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/example/source_generation_latex.cpp

example/CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.i"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/example/source_generation_latex.cpp > CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.i

example/CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.s"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/example/source_generation_latex.cpp -o CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.s

# Object files for target source_generation_latex
source_generation_latex_OBJECTS = \
"CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.o"

# External object files for target source_generation_latex
source_generation_latex_EXTERNAL_OBJECTS =

example/source_generation_latex: example/CMakeFiles/source_generation_latex.dir/source_generation_latex.cpp.o
example/source_generation_latex: example/CMakeFiles/source_generation_latex.dir/build.make
example/source_generation_latex: example/CMakeFiles/source_generation_latex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable source_generation_latex"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/source_generation_latex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/source_generation_latex.dir/build: example/source_generation_latex

.PHONY : example/CMakeFiles/source_generation_latex.dir/build

example/CMakeFiles/source_generation_latex.dir/clean:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/example && $(CMAKE_COMMAND) -P CMakeFiles/source_generation_latex.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/source_generation_latex.dir/clean

example/CMakeFiles/source_generation_latex.dir/depend:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/example /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/example /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/example/CMakeFiles/source_generation_latex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/source_generation_latex.dir/depend

