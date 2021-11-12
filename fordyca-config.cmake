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

define_property(CACHED_VARIABLE PROPERTY "FORDYCA_WITH_ROBOT_RAB"
  BRIEF_DOCS "Enable robots to use the RAB medium."
  FULL_DOCS "Default=NO.")
define_property(CACHED_VARIABLE PROPERTY "FORDYCA_WITH_ROBOT_BATTERY"
  BRIEF_DOCS "Enable robots to use the battery."
  FULL_DOCS "Default=NO.")
define_property(CACHED_VARIABLE PROPERTY "FORDYCA_WITH_ROBOT_LEDS"
  BRIEF_DOCS "Enable robots to use their LEDs."
  FULL_DOCS "Default=NO.")
define_property(CACHED_VARIABLE PROPERTY "FORDYCA_WITH_ROBOT_CAMERA"
  BRIEF_DOCS "Enable robots to use their camera."
  FULL_DOCS "Default=YES.")

# Needed by COSM for population dynamics and swarm iteration
if (NOT COSM_BUILD_FOR)
  set(COSM_BUILD_FOR "ARGOS_FOOTBOT")
endif()
set(COSM_ARGOS_ROBOT_TYPE "foot-bot")
set(COSM_ARGOS_ROBOT_NAME_PREFIX "fb")
set(COSM_ARGOS_CONTROLLER_XML_ID "fbc")

################################################################################
# External Projects                                                            #
################################################################################
set(${target}_CHECK_LANGUAGE "CXX")

# Support libraries
add_subdirectory(ext/cosm)

if ("${COSM_WITH_VIS}")
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
  set(CMAKE_AUTOMOC OFF)
endif()

################################################################################
# Sources                                                                      #
################################################################################
if (NOT ${COSM_WITH_VIS})
    list(REMOVE_ITEM ${target}_ROOT_SRC
    ${${target}_SRC_PATH}/support/los_visualizer.cpp
    ${${target}_SRC_PATH}/support/d0/d0_qt_user_functions.cpp
    ${${target}_SRC_PATH}/support/d1/d1_qt_user_functions.cpp
    ${${target}_SRC_PATH}/support/d2/d2_qt_user_functions.cpp)
endif()

################################################################################
# Libraries                                                                    #
################################################################################
# Define link libraries
set(${target}_LIBRARIES
  cosm-${COSM_HAL_TARGET}
  ${cosm_LIBRARIES}
  nlopt
  rt)

# Define link search dirs
set(${target}_LIBRARY_DIRS
  ${cosm_LIBRARY_DIRS})

# Define include dirs
set(${target}_INCLUDE_DIRS
  "${${target}_INC_PATH}"
  ${cosm_INCLUDE_DIRS}
  )

# Define system include dirs
set(${target}_SYS_INCLUDE_DIRS
  ${cosm_SYS_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  )

if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  set(argos3_LIBRARIES
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_epuck
  argos3plugin_simulator_entities
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_qtopengl
  argos3plugin_simulator_media
  )

  set (${target}_LIBRARIES
    ${${target}_LIBRARIES}
    ${argos3_LIBRARIES}
    )
  if ("${COSM_BUILD_ENV}" MATCHES "MSI")
    # For nlopt
    set(${target}_LIBRARY_DIRS
      ${$target}_LIBRARY_DIRS}
      ${COSM_DEPS_PREFIX}/lib64
      )
  endif()
endif()

# Force failures at build time rather than runtime
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

# Define the FORDYCA library
add_library(${target} SHARED ${${target}_ROOT_SRC})

add_dependencies(${target} cosm-${COSM_HAL_TARGET})

target_include_directories(${target} PUBLIC ${${target}_INCLUDE_DIRS})
target_include_directories(${target} SYSTEM PUBLIC ${${target}_SYS_INCLUDE_DIRS})
target_link_libraries(${target} ${${target}_LIBRARIES})
target_link_directories(${target} PUBLIC ${${target}_LIBRARY_DIRS})

################################################################################
# Installation                                                                 #
################################################################################
# Define package dependencies
set(${target}_PACKAGE_DEPENDS
  ${cosm_PACKAGE_DEPENDS}
  )

set(CPACK_DEBIAN_PACKAGE_DEPENDS
  ${cosm_PACKAGE_DEPENDS}
  )

set(CPACK_PACKAGE_CONTACT "John Harwell")

################################################################################
# Compile Options/Definitions                                                  #
################################################################################
if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  if (FORDYCA_WITH_ROBOT_RAB)
    target_compile_definitions(${target} PUBLIC FORDYCA_WITH_ROBOT_RAB)
  endif()
  if (FORDYCA_WITH_ROBOT_BATTERY)
    target_compile_definitions(${target} PUBLIC FORDYCA_WITH_ROBOT_BATTERY)
  endif()
  if (FORDYCA_WITH_ROBOT_CAMERA)
    target_compile_definitions(${target} PUBLIC FORDYCA_WITH_ROBOT_CAMERA)
  endif()
  if (FORDYCA_WITH_ROBOT_LEDS)
    target_compile_definitions(${target} PUBLIC FORDYCA_WITH_ROBOT_LEDS)
  endif()
endif()

if ("${COSM_BUILD_FOR}" MATCHES "MSI")
  target_compile_options(${target} PUBLIC
    -Wno-missing-include-dirs
    -fno-new-inheriting-ctors)
endif()

################################################################################
# Exports                                                                      #
################################################################################
if (NOT IS_ROOT_PROJECT)
  set(${target}_INCLUDE_DIRS ${${target}_INCLUDE_DIRS} PARENT_SCOPE)
  set(${target}_LIBRARY_DIRS ${${target}_LIBRARY_DIRS} PARENT_SCOPE)
  set(${target}_LIBRARIES ${${target}_LIBRARIES} PARENT_SCOPE)
endif()
