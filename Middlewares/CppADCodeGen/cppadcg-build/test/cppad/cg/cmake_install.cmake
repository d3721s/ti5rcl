# Install script for directory: /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "DEBUG")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/extra/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/operations/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/model/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/patterns/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/evaluator/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/solve/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/dae_index_reduction/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/support/cmake_install.cmake")
endif()
