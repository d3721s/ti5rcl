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

# Utility rule file for create_tmp_folder_cg.

# Include the progress variables for this target.
include test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/progress.make

test/cppad/cg/CMakeFiles/create_tmp_folder_cg: test/cppad/cg/tmp


test/cppad/cg/tmp:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Creating tmp folder"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg && /usr/bin/cmake -E remove_directory tmp
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg && /usr/bin/cmake -E make_directory tmp

create_tmp_folder_cg: test/cppad/cg/CMakeFiles/create_tmp_folder_cg
create_tmp_folder_cg: test/cppad/cg/tmp
create_tmp_folder_cg: test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/build.make

.PHONY : create_tmp_folder_cg

# Rule to build all files generated by this target.
test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/build: create_tmp_folder_cg

.PHONY : test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/build

test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/clean:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg && $(CMAKE_COMMAND) -P CMakeFiles/create_tmp_folder_cg.dir/cmake_clean.cmake
.PHONY : test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/clean

test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/depend:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/cppad/cg/CMakeFiles/create_tmp_folder_cg.dir/depend
