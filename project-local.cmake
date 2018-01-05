################################################################################
# External Projects                                                            #
################################################################################
# Qt
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

# Boost
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost 1.58.0 COMPONENTS system filesystem thread)

# RCPPSW
ExternalProject_Add(rcppsw_EXTERNAL
  SOURCE_DIR "$ENV{rcppsw}"
  BINARY_DIR "$ENV{rcppsw}/build"
  STEP_TARGETS build
  INSTALL_COMMAND true
)

ExternalProject_Get_Property(rcppsw_EXTERNAL binary_dir)
ExternalProject_Get_Property(rcppsw_EXTERNAL source_dir)
add_library(rcppsw STATIC IMPORTED)
set_property(TARGET rcppsw PROPERTY IMPORTED_LOCATION ${binary_dir}/lib/librcppsw.a)
include_directories(${source_dir}/include)
set(rcppsw_LIB ${binary_dir}/lib/librcppsw.a)

# RCSW
ExternalProject_Add(rcsw_EXTERNAL
  SOURCE_DIR "$ENV{rcsw}"
  BINARY_DIR "$ENV{rcsw}/build"
  STEP_TARGETS build
  INSTALL_COMMAND true
)

ExternalProject_Get_Property(rcsw_EXTERNAL binary_dir)
ExternalProject_Get_Property(rcsw_EXTERNAL source_dir)
add_library(rcsw STATIC IMPORTED)
set_property(TARGET rcsw PROPERTY IMPORTED_LOCATION ${binary_dir}/lib/librcsw.a)
include_directories(${source_dir}/include)
set(rcsw_LIB ${binary_dir}/lib/librcsw.a)

################################################################################
# Includes                                                                     #
################################################################################
include_directories(BEFORE SYSTEM
  /usr/include/lua5.2
  ${Qt5Widgets_INCLUDE_DIRS}
  )

################################################################################
# Submodules                                                                   #
################################################################################
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

################################################################################
# Libraries                                                                    #
################################################################################
get_filename_component(target ${CMAKE_CURRENT_LIST_DIR} NAME)
link_directories(/usr/lib/argos3)
add_library(${target} SHARED ${${target}_ROOT_SRC})
add_dependencies(${target} rcppsw_EXTERNAL-build rcsw_EXTERNAL-build)

# Use the actual .a files rather than rcppsw/rcsw project names to force
# recompilation of fordyca when a source file within one of those libraries
# changes.
target_link_libraries(${target}
  ${rcppsw_LIB}
  ${rcsw_LIB}
  stdc++fs
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  ${Boost_LIBRARIES}
  )
