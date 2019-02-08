################################################################################
# Configuration Options                                                        #
################################################################################
set(WITH_FOOTBOT_LEDS "YES" CACHE STRING "Enable footbot robots to control their LEDS via actuators")
set(WITH_FOOTBOT_RAB "YES" CACHE STRING "Enable footbot robots to read/write over the RAB medium via sensors/actuators.")

define_property(CACHED_VARIABLE PROPERTY "WITH_FOOTBOT_LEDS"
  BRIEF_DOCS "Enable footbot robots to control their LEDS"
  FULL_DOCS "Default=yes.")
define_property(CACHED_VARIABLE PROPERTY "WITH_FOOTBOT_RAB"
  BRIEF_DOCS "Enable footbot robots to use the RAB medium"
  FULL_DOCS "Default=yes.")

################################################################################
# External Projects                                                            #
################################################################################
set(${target}_CHECK_LANGUAGE "CXX")

if(BUILD_ON_MSI)
  set(LOCAL_INSTALL_PREFIX /home/gini/shared/swarm)
else()
  # Qt (not reliably available on MSI)
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
  set(CMAKE_AUTOMOC OFF)

  set(LOCAL_INSTALL_PREFIX /opt/data/local)
endif()


# RCPPSW
add_subdirectory(ext/rcppsw)

################################################################################
# Includes                                                                     #
################################################################################
set(${target}_INCLUDE_DIRS
  "${${target}_INC_PATH}"
  ${rcppsw_INCLUDE_DIRS}
  /usr/include/eigen3)
set(${target}_SYS_INCLUDE_DIRS
  ${rcppsw_SYS_INCLUDE_DIRS}
  ${LOCAL_INSTALL_PREFIX}/include
    ${NLOPT_INCLUDE_DIRS})

################################################################################
# Libraries                                                                    #
################################################################################
# Define link libraries
set(${target}_LIBRARIES
  rcppsw
  nlopt
  ${rcppsw_LIBRARIES}
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_entities
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_qtopengl
  stdc++fs
  rt)

# Define link search dirs
set(${target}_LIBRARY_DIRS
  /usr/lib/argos3
  /usr/local/lib/argos3
  ${LOCAL_INSTALL_PREFIX}/lib/argos3
  ${rcppsw_LIBRARY_DIRS})

# Qt not reliably available on MSI
if (NOT BUILD_ON_MSI)
  set(${target}_LIBRARIES ${${target}_LIBRARIES}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui)
endif()


# Force failures at build time rather than runtime
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

if (BUILD_ON_MSI)
  # For nlopt
  set(${target}_LIBRARY_DIRS ${${target}_LIBRARY_DIRS}
    ${LOCAL_INSTALL_PREFIX}/lib
    ${LOCAL_INSTALL_PREFIX}/lib64)

  # Because Qt not reliably available
    list(REMOVE_ITEM ${target}_ROOT_SRC
    ${${target}_SRC_PATH}/support/block_carry_visualizer.cpp
    ${${target}_SRC_PATH}/support/los_visualizer.cpp
    ${${target}_SRC_PATH}/support/task_visualizer.cpp
    ${${target}_SRC_PATH}/support/depth0/depth0_qt_user_functions.cpp
    ${${target}_SRC_PATH}/support/depth1/depth1_qt_user_functions.cpp
    ${${target}_SRC_PATH}/support/depth2/depth2_qt_user_functions.cpp)
endif()

link_directories(${${target}_LIBRARY_DIRS})
add_library(${target} SHARED ${${target}_ROOT_SRC})
add_dependencies(${target} rcsw rcppsw)

target_include_directories(${target} PUBLIC ${${target}_INCLUDE_DIRS})
target_include_directories(${target} SYSTEM PUBLIC
  /usr/include/lua5.2
  /usr/local/include
  ${nlopt_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${${target}_SYS_INCLUDE_DIRS}
  )
target_link_libraries(${target} ${${target}_LIBRARIES})

################################################################################
# Compile Definitions                                                          #
################################################################################

target_compile_definitions(${target} PUBLIC HAL_CONFIG=HAL_CONFIG_ARGOS_FOOTBOT)

if (WITH_FOOTBOT_LEDS)
  target_compile_definitions(${target} PUBLIC FORDYCA_WITH_ROBOT_LEDS)
endif()

if (WITH_FOOTBOT_RAB)
  target_compile_definitions(${target} PUBLIC FORDYCA_WITH_ROBOT_RAB)
endif()

if (BUILD_ON_MSI)
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
