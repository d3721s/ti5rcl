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

# Utility rule file for create_tmp_folder_adcg.

# Include the progress variables for this target.
include test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/progress.make

test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg: test/cppad/cg/evaluator/adcg/tmp


test/cppad/cg/evaluator/adcg/tmp:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Creating tmp folder"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/evaluator/adcg && /usr/bin/cmake -E remove_directory tmp
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/evaluator/adcg && /usr/bin/cmake -E make_directory tmp

create_tmp_folder_adcg: test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg
create_tmp_folder_adcg: test/cppad/cg/evaluator/adcg/tmp
create_tmp_folder_adcg: test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/build.make

.PHONY : create_tmp_folder_adcg

# Rule to build all files generated by this target.
test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/build: create_tmp_folder_adcg

.PHONY : test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/build

test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/clean:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/evaluator/adcg && $(CMAKE_COMMAND) -P CMakeFiles/create_tmp_folder_adcg.dir/cmake_clean.cmake
.PHONY : test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/clean

test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/depend:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/evaluator/adcg /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/evaluator/adcg /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/cppad/cg/evaluator/adcg/CMakeFiles/create_tmp_folder_adcg.dir/depend
