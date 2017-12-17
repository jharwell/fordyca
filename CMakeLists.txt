################################################################################
# Cmake Configuration Template
#
# Can be used for the root/top-level meta-project, or for a submodule.
#
################################################################################

# CMake version
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

get_filename_component(target ${CMAKE_CURRENT_LIST_DIR} NAME)
get_filename_component(root_target ${CMAKE_SOURCE_DIR} NAME)
project(${target} C CXX)

# Exit here if required dependencies are not met
message("Found ${target}")

################################################################################
# Cmake Environment                                                            #
################################################################################
# Register custom cmake commands
list(APPEND CMAKE_MODULE_PATH "$ENV{develroot}/cmake")
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

# Custom modules
include(compile-options)
include(build-modes)
include(custom-cmds)
include(analysis)
include(doxygen)

# Set policies
set_policy(CMP0028 NEW) # ENABLE CMP0028: Double colon in target name means ALIAS or IMPORTED target.
set_policy(CMP0054 NEW) # ENABLE CMP0054: Only interpret if() arguments as variables or keywords when unquote2d.
set_policy(CMP0063 NEW) # ENABLE CMP0063: Honor visibility properties for all target types.

################################################################################
# Project Configuration                                                        #
################################################################################
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "DEV")
endif()

# Handy checking tools
toggle_cppcheck(ON)
toggle_clang_tidy_check(OFF)
toggle_clang_static_check(ON)
toggle_clang_format(OFF)
toggle_clang_tidy_fix(OFF)

option(SHARED_LIBS    "Build shared instead of static libraries."              ON)
option(WITH_CHECKS    "Build in run-time checking of code."                    OFF)
option(SELF_CONTAINED "Create a self-contained install with all dependencies." OFF)
option(BUILD_TESTS    "Build tests."                                           OFF)
option(WITH_OPENMP    "Enable OpenMP code."                                    ON)
option(WITH_MPI       "Enable MPI code."                                       OFF)
option(WITH_FPC       "FPC_RETURN or FPC_ABORT"                                FPC_ABORT)
set(FPC FPC_TYPE="${WITH_FPC}")

# Set output directories. If we are the root project, then this is
# necessary. If not, we simply re-set the same values.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################################################
# Source Definitions                                                           #
################################################################################
# Sources
set(${target}_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")
file(GLOB ${target}_ROOT_SRC ${${target}_SRC_PATH}/*.c ${${target}_SRC_PATH}/*.cpp)
file(GLOB_RECURSE ${target}_SRC ${${target}_SRC_PATH}/*.c ${${target}_SRC_PATH}/*.cpp)
file(GLOB ${target}_C_SRC ${${target}_SRC_PATH}/*.c )
file(GLOB ${target}_CXX_SRC ${${target}_SRC_PATH}/*.cpp)

set(${target}_INC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/")
set(${target}_ROOT_INC_PATH "${CMAKE_SOURCE_DIR}/include/")
include_directories(${${target}_ROOT_INC_PATH})

# Tests
#
# Might need the common test definitions under src/tests/include, so add it to
# the global include path.
include_directories(src/tests/include)

set(${target}_TEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/tests)
file(GLOB c_tests ${${target}_TEST_PATH}/*-test.c)
file(GLOB c_test_harness ${${target}_TEST_PATH}/*_test.c ${${target}_TEST_PATH}/*.h ${${target}_TEST_PATH}/*.hpp)
file(GLOB cxx_tests ${${target}_TEST_PATH}/*-test.cpp)
file(GLOB cxx_test_harness ${${target}_TEST_PATH}/*_test.cpp  ${${target}_TEST_PATH}/*.hpp)

################################################################################
# Target Definitions                                                           #
################################################################################
# now we can add project-local config
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/project-local.cmake")
include(project-local.cmake)
endif()

# We are not the root project--we are a submodule. Add ourselves to the
# dependencies of the root target.
if (NOT "${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
  if (NOT "${target}" STREQUAL "tests")
    add_library(${target} OBJECT ${${target}_SRC})
  else()
    add_library(${target} ${${target}_SRC})
  endif()
else()
  export(TARGETS ${target} FILE ${PROJECT_BINARY_DIR}/${target}-config.cmake)
endif()

################################################################################
# Code Checking/Analysis Options                                               #
################################################################################
# if (NOT "${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
  register_checkers(${target} ${${target}_SRC})
  register_auto_formatters(${target} ${${target}_SRC})
  register_auto_fixers(${target} ${${target}_SRC})
# endif()

################################################################################
# Testing Options                                                              #
################################################################################
enable_testing()
if (NOT TARGET unit_tests)
  add_custom_target(unit_tests)
endif()
if (NOT TARGET build_and_test)
  add_custom_target(build_and_test ${CMAKE_TEST_COMMAND} -V)
endif()

foreach(t ${c_tests} ${cxx_tests})
  string(FIND ${t} ".#" position)
  if(NOT "${position}" MATCHES "-1")
    continue()
  endif()

  get_filename_component(test_name ${t} NAME_WE)
  get_filename_component(test_file ${t} NAME)
  add_executable(${test_name} EXCLUDE_FROM_ALL ${c_test_harness} ${cxx_test_harness} ${${target}_TEST_PATH}/${test_file})
  add_dependencies(${test_name} ${root_target})
  if (TARGET tests)
    target_link_libraries(${test_name}
      tests
      ${root_target}
      ${Boost_LIBRARIES}
      ${DEFAULT_LIBRARIES}
      )
  else()
    target_link_libraries(${test_name}
      ${root_target}
      ${Boost_LIBRARIES}
      ${DEFAULT_LIBRARIES})
  endif()

  add_dependencies(unit_tests ${test_name})
  add_test(${test_name} ${CMAKE_BINARY_DIR}/bin/${test_name})
  add_dependencies(build_and_test ${test_name})

endforeach()

include(deploy)
