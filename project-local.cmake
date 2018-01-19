################################################################################
# External Projects                                                            #
################################################################################
set(PROJECT_NAME fordyca CACHE INTERNAL "PROJECT_NAME")

# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
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
  ${Qt5Widgets_INCLUDE_DIRS}
  )

################################################################################
# Libraries                                                                    #
################################################################################
get_filename_component(target ${CMAKE_CURRENT_LIST_DIR} NAME)
link_directories(/usr/lib/argos3 ${rcppsw_LINK_DIRS})
add_library(${target} SHARED ${${target}_ROOT_SRC})
add_dependencies(${target} rcsw rcppsw)

# Use the actual .a files rather than rcppsw/rcsw project names to force
# recompilation of fordyca when a source file within one of those libraries
# changes.
target_link_libraries(${target}
  rcppsw
  rcsw
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  stdc++fs
  rcppsw
  rcsw
  )
