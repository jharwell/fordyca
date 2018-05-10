################################################################################
# External Projects                                                            #
################################################################################
set(${target}_CHECK_LANGUAGE "CXX")
set(${target}_HAS_RECURSIVE_DIRS NO)

# Qt
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
set(CMAKE_AUTOMOC OFF)

# RCPPSW
add_subdirectory(ext/rcppsw)
include_directories(${rcppsw_INCLUDE_DIRS})

################################################################################
# Includes                                                                     #
################################################################################
if (NOT IS_ROOT_PROJECT)
  set(${target}_INCLUDE_DIRS "${${target}_INC_PATH}" PARENT_SCOPE)
else()
  set(${target}_INCLUDE_DIRS "${${target}_INC_PATH}")
endif()

include_directories(BEFORE SYSTEM
  /usr/include/lua5.2
  /usr/local/include
  ${Qt5Widgets_INCLUDE_DIRS}
  )

################################################################################
# Libraries                                                                    #
################################################################################
get_filename_component(target ${CMAKE_CURRENT_LIST_DIR} NAME)
link_directories(/usr/lib/argos3 /usr/local/lib/argos3 ${rcppsw_LINK_DIRS})
add_library(${target} SHARED ${${target}_ROOT_SRC})
add_dependencies(${target} rcsw rcppsw)
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

target_link_libraries(${target}
  rcppsw
  rcsw
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_entities
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_qtopengl
  stdc++fs
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  )

if ("${WITH_HAL_CONFIG}" MATCHES "argos-footbot")
  target_compile_definitions(${target} PUBLIC HAL_CONFIG=HAL_CONFIG_ARGOS_FOOTBOT)
endif()
