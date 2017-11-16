################################################################################
# External Projects                                                            #
################################################################################
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
ExternalProject_Add(project_rcppsw
  SOURCE_DIR "$ENV{rcppsw}"
  BINARY_DIR "$ENV{rcppsw}/build"
  STEP_TARGETS build
  INSTALL_COMMAND true
)

ExternalProject_Get_Property(project_rcppsw binary_dir)
ExternalProject_Get_Property(project_rcppsw source_dir)
add_library(rcppsw STATIC IMPORTED)
set_property(TARGET rcppsw PROPERTY IMPORTED_LOCATION ${binary_dir}/lib/librcppsw.a)
include_directories(${source_dir}/include)

ExternalProject_Add(project_rcsw
  SOURCE_DIR "$ENV{rcsw}"
  BINARY_DIR "$ENV{rcsw}/build"
  STEP_TARGETS build
  INSTALL_COMMAND true
)

ExternalProject_Get_Property(project_rcsw binary_dir)
ExternalProject_Get_Property(project_rcsw source_dir)
add_library(rcsw STATIC IMPORTED)
set_property(TARGET rcsw PROPERTY IMPORTED_LOCATION ${binary_dir}/lib/librcsw.a)
include_directories(${source_dir}/include)

set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost 1.58.0 COMPONENTS system filesystem thread)

################################################################################
# Includes                                                                     #
################################################################################
include_directories(BEFORE SYSTEM
  /usr/include/lua5.2
  ${Qt5Widgets_INCLUDE_DIRS}
  )
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --system-header-prefix=argos/")
endif()

################################################################################
# Submodules                                                                   #
################################################################################

################################################################################
# Libraries                                                                    #
################################################################################
get_filename_component(target ${CMAKE_CURRENT_LIST_DIR} NAME)
link_directories(/usr/lib/argos3)
add_library(${target} SHARED ${${target}_SRC})
add_dependencies(${target} project_rcppsw-build project_rcsw-build)
target_link_libraries(${target}
  rcppsw
  rcsw
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  ${Boost_LIBRARIES}
  )
