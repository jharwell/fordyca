################################################################################
# Configuration Options                                                        #
################################################################################
# We are building a shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Each conference tag=minor increment. Each minor feature added=patch increment.
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 3)
set(PROJECT_VERSION_PATCH 0)

set(FORDYCA_WITH_ROBOT_RAB "NO" CACHE STRING "Enable robots to read/write over the RAB medium via sensors/actuators.")
set(FORDYCA_WITH_ROBOT_BATTERY "NO" CACHE STRING "Enable robots to use the battery.")
set(FORDYCA_WITH_ROBOT_LEDS "NO" CACHE STRING "Enable robots to use their LEDs.")
set(FORDYCA_WITH_ROBOT_CAMERA "YES" CACHE STRING "Enable robots to use their camera.")

set(fordyca_CHECK_LANGUAGE "CXX")

################################################################################
# External Projects                                                            #
################################################################################
# COSM
if("${COSM_BUILD_FOR}" MATCHES "ARGOS_FOOTBOT")
  find_package(cosm-argos-footbot)
elseif("${COSM_BUILD_FOR}" MATCHES "ROS_ETURTLEBOT3")
  find_package(cosm-ros-eturtlebot3
    HINTS ${CMAKE_INSTALL_PREFIX})
else()
  message(FATAL_ERROR "FORDYCA only supports [ARGOS_FOOTBOT,ROS_ETURTLEBOT3]")
endif()

################################################################################
# Components
################################################################################
string(CONCAT common_regex
  "src/math|"
  "src/metrics/blocks|"
  "src/init|"
  "src/repr/diagnostics|"
  "src/metrics/specs"
  )
component_register_as_src(
  fordyca_common_SRC
  fordyca
  "${fordyca_SRC}"
  common
  "(${common_regex})")

if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  string(CONCAT argos_regex
    "src/fsm|"
      "src/controller|"
      "src/events|"
      "src/support|"
      "src/subsystem|"
      "src/strategy|"
      "src/repr|"
      "src/metrics|"
      "src/tasks|"
      "src/metrics|"
      "src/argos"
      )
  component_register_as_src(
    fordyca_argos_SRC
    fordyca
    "${fordyca_SRC}"
    argos
    "(${argos_regex})")

  if(COSM_WITH_VIS)
    component_register_as_src(
      fordyca_argos_vis_SRC
      fordyca
      "${fordyca_SRC}"
      argos_vis
      "qt|los_visualizer")
  endif()

  # Root project (not used in find_package())
  if (NOT fordyca_FIND_COMPONENTS)
    set(fordyca_FIND_COMPONENTS
      common
      argos
      argos_vis
      )
  endif()

elseif("${COSM_BUILD_FOR}" MATCHES "ROS")
  string(CONCAT ros_regex
    "src/ros|"
    "src/fsm/d0/crw|"
    "src/strategy/explore/block|"
    "src/strategy/explore/crw|"
    "src/strategy/config|"
    "src/strategy/explore/likelihood_block|"
    "src/strategy/explore/localized|"
    "src/fsm/foraging_acq_goal|"
    "src/controller/reactive/d0|"
    "src/controller/foraging_controller|"
    "src/controller/config/foraging_controller_repository"
    )
  component_register_as_src(
    fordyca_ros_SRC
    fordyca
    "${fordyca_SRC}"
    ros
    "(${ros_regex})")
  # Root project (not used in find_package())
  if (NOT fordyca_FIND_COMPONENTS)
    set(fordyca_FIND_COMPONENTS
      common
      ros
      )
  endif()
endif()

requested_components_check(fordyca)

################################################################################
# Libraries                                                                    #
################################################################################
# Create the source for the SINGLE library to build by combining the
# source of the selected components
foreach(component ${fordyca_FIND_COMPONENTS})
  if(${fordyca_${component}_FOUND})
    list(APPEND fordyca_components_SRC ${fordyca_} ${fordyca_${component}_SRC})
  endif()
endforeach()

# Configure version
execute_process(COMMAND git rev-list --count HEAD
  OUTPUT_VARIABLE FORDYCA_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/version.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/version.cpp
  @ONLY
  )
list(APPEND fordyca_components_SRC "${CMAKE_CURRENT_BINARY_DIR}/src/version.cpp")

# Define the FORDYCA library
set(fordyca_LIBRARY fordyca-${COSM_HAL_TARGET})

add_library(
  ${fordyca_LIBRARY}
  SHARED
  ${fordyca_components_SRC}
  )

# alias so we plug into the LIBRA framework properly
add_library(fordyca ALIAS ${fordyca_LIBRARY})


set(fordyca_LIBRARY_NAME fordyca-${COSM_HAL_TARGET})
set_target_properties(${fordyca_LIBRARY}
  PROPERTIES OUTPUT_NAME
  ${fordyca_LIBRARY_NAME})

########################################
# Include directories
########################################
target_include_directories(
  ${fordyca_LIBRARY}
  PUBLIC
  $<BUILD_INTERFACE:${fordyca_DIR}/include>
  $<BUILD_INTERFACE:${cosm_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
  )

########################################
# Link Libraries
########################################
target_link_libraries(${fordyca_LIBRARY}
  cosm-${COSM_HAL_TARGET}::cosm-${COSM_HAL_TARGET}
  rt
  )

if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  target_link_libraries(${fordyca_LIBRARY}
    nlopt
    )

  if ("${COSM_BUILD_ENV}" MATCHES "MSI")
    # For nlopt
    target_link_directories(${fordyca_LIBRARY}
      PUBLIC
      ${LIBRA_DEPS_PREFIX}/lib64
      )
  endif()
endif()

# Force failures at build time rather than runtime
target_link_options(${fordyca_LIBRARY} PRIVATE -Wl,--no-undefined)

########################################
# Compile Options/Definitions
########################################
if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  if (FORDYCA_WITH_ROBOT_RAB)
    target_compile_definitions(${fordyca_LIBRARY}
      PUBLIC
      FORDYCA_WITH_ROBOT_RAB)
  endif()
  if (FORDYCA_WITH_ROBOT_BATTERY)
    target_compile_definitions(${fordyca_LIBRARY}
      PUBLIC
      FORDYCA_WITH_ROBOT_BATTERY)
  endif()
  if (FORDYCA_WITH_ROBOT_CAMERA)
    target_compile_definitions(${fordyca_LIBRARY}
      PUBLIC
      FORDYCA_WITH_ROBOT_CAMERA)
  endif()
endif()

if (FORDYCA_WITH_ROBOT_LEDS)
  target_compile_definitions(${fordyca_LIBRARY}
    PUBLIC
    FORDYCA_WITH_ROBOT_LEDS)
endif()

if ("${COSM_BUILD_FOR}" MATCHES "MSI")
  target_compile_options(${fordyca_LIBRARY} PUBLIC
    -Wno-missing-include-dirs
    -fno-new-inheriting-ctors)
endif()

################################################################################
# Installation                                                                 #
################################################################################
configure_exports_as(${fordyca_LIBRARY} ${CMAKE_INSTALL_PREFIX})

# Install fordyca
register_target_for_install(${fordyca_LIBRARY} ${CMAKE_INSTALL_PREFIX})
register_headers_for_install(include/fordyca ${CMAKE_INSTALL_PREFIX})

################################################################################
# Status                                                                       #
################################################################################
libra_config_summary()

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH};${cosm_INSTALL_PREFIX}/lib/cmake/cosm-${COSM_HAL_TARGET}")
include(cosm-config-summary)
cosm_config_summary()

message(STATUS "")
message(STATUS "")
message(STATUS "FORDYCA Configuration Summary:")
message(STATUS "")

if(${COSM_BUILD_FOR} MATCHES "ARGOS")
  message(STATUS "With robot RAB........................: FORDYCA_WITH_ROBOT_RAB=${FORDYCA_WITH_ROBOT_RAB}")
  message(STATUS "With robot battery....................: FORDYCA_WITH_ROBOT_BATTERY=${FORDYCA_WITH_ROBOT_BATTERY}")
  message(STATUS "With robot LEDs.......................: FORDYCA_WITH_ROBOT_LEDS=${FORDYCA_WITH_ROBOT_LEDS}")
  message(STATUS "With robot CAMERA.....................: FORDYCA_WITH_ROBOT_CAMERA=${FORDYCA_WITH_ROBOT_CAMERA}")
endif()
