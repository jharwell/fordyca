################################################################################
# External Projects                                                            #
################################################################################
set(${target}_CHECK_LANGUAGE "CXX")
set(${target}_HAS_RECURSIVE_DIRS NO)

if(BUILD_ON_MSI)
set(MSI_ARGOS_INSTALL_PREFIX /home/gini/shared/swarm)
endif()

# Qt
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
set(CMAKE_AUTOMOC OFF)

# RCPPSW
add_subdirectory(ext/rcppsw)

################################################################################
# Includes                                                                     #
################################################################################
set(${target}_INCLUDE_DIRS "${${target}_INC_PATH}" ${rcppsw_INCLUDE_DIRS})
set(${target}_SYS_INCLUDE_DIRS ${rcppsw_SYS_INCLUDE_DIRS})

if (BUILD_ON_MSI)
  set(${target}_SYS_INCLUDE_DIRS ${rcppsw_SYS_INCLUDE_DIRS} ${MSI_ARGOS_INSTALL_PREFIX}/include)
endif()

################################################################################
# Libraries                                                                    #
################################################################################
get_filename_component(target ${CMAKE_CURRENT_LIST_DIR} NAME)

# Define link libraries
set(${target}_LIBRARIES
  rcppsw
  ${rcppsw_LIBRARIES}
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_entities
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_qtopengl
  stdc++fs
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui)

# Define link search dirs
if (BUILD_ON_MSI)
  set(${target}_LIBRARY_DIRS ${MSI_ARGOS_INSTALL_PREFIX}/lib/argos3 ${rcppsw_LIBRARY_DIRS})
else()
  set(${target}_LIBRARY_DIRS /usr/lib/argos3 /usr/local/lib/argos3 ${rcppsw_LIBRARY_DIRS})
endif()
link_directories(${${target}_LIBRARY_DIRS})

# Force failures at build time rather than runtime
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

add_library(${target} SHARED ${${target}_ROOT_SRC})
add_dependencies(${target} rcsw rcppsw)

target_compile_definitions(${target} PUBLIC HAL_CONFIG=HAL_CONFIG_ARGOS_FOOTBOT)
target_include_directories(${target} SYSTEM PUBLIC
  /usr/include/lua5.2
  /usr/local/include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${${target}_SYS_INCLUDE_DIRS}
  )
target_include_directories(${target} PUBLIC ${${target}_INCLUDE_DIRS})
target_link_libraries(${target} ${${target}_LIBRARIES})

if (BUILD_ON_MSI)
  target_compile_options(${target} PUBLIC -Wno-missing-include-dirs)
endif()

################################################################################
# Exports                                                                      #
################################################################################
if (NOT IS_ROOT_PROJECT)
  set(${target}_INCLUDE_DIRS ${${target}_INCLUDE_DIRS} PARENT_SCOPE)
  set(${target}_LIBRARY_DIRS ${${target}_LIBRARY_DIRS} PARENT_SCOPE)
  set(${target}_LIBRARIES ${${target}_LIBRARIES} PARENT_SCOPE)
endif()
