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
include test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/depend.make

# Include the progress variables for this target.
include test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/progress.make

# Include the compile flags for this target's objects.
include test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/flags.make

test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.o: test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/flags.make
test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.o: ../include/cppad/cg/model/threadpool/pthread_pool.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.o"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/threadpool && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.o   -c /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/include/cppad/cg/model/threadpool/pthread_pool.c

test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.i"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/threadpool && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/include/cppad/cg/model/threadpool/pthread_pool.c > CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.i

test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.s"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/threadpool && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/include/cppad/cg/model/threadpool/pthread_pool.c -o CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.s

# Object files for target pthread_pool
pthread_pool_OBJECTS = \
"CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.o"

# External object files for target pthread_pool
pthread_pool_EXTERNAL_OBJECTS =

test/cppad/cg/model/threadpool/libpthread_pool.a: test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/__/__/__/__/__/include/cppad/cg/model/threadpool/pthread_pool.c.o
test/cppad/cg/model/threadpool/libpthread_pool.a: test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/build.make
test/cppad/cg/model/threadpool/libpthread_pool.a: test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libpthread_pool.a"
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/threadpool && $(CMAKE_COMMAND) -P CMakeFiles/pthread_pool.dir/cmake_clean_target.cmake
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/threadpool && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pthread_pool.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/build: test/cppad/cg/model/threadpool/libpthread_pool.a

.PHONY : test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/build

test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/clean:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/threadpool && $(CMAKE_COMMAND) -P CMakeFiles/pthread_pool.dir/cmake_clean.cmake
.PHONY : test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/clean

test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/depend:
	cd /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/model/threadpool /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/threadpool /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/cppad/cg/model/threadpool/CMakeFiles/pthread_pool.dir/depend

