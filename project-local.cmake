################################################################################
# External Projects                                                            #
################################################################################
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

ExternalProject_Add(rcppsw
  SOURCE_DIR "$ENV{rcppsw}"
  BINARY_DIR "$ENV{rcppsw}/build"
  STEP_TARGETS build
  EXCLUDE_FROM_ALL TRUE
)


################################################################################
# Includes                                                                     #
################################################################################
include_directories("$ENV{rcppsw}/include")
include_directories("$ENV{rcsw}/include")
include_directories(BEFORE SYSTEM
  /usr/include
  /usr/include/lua5.2
  ${Qt5Widgets_INCLUDE_DIRS}
  )

################################################################################
# Submodules                                                                   #
################################################################################

################################################################################
# Libraries                                                                    #
################################################################################
get_filename_component(target ${CMAKE_CURRENT_LIST_DIR} NAME)

# add_library(${target} SHARED
#   $<TARGET_OBJECTS:common>
#   $<TARGET_OBJECTS:multithread>
#   $<TARGET_OBJECTS:utils>
#   )

# if(WITH_MPI)
# add_library(${target} SHARED
#   $<TARGET_OBJECTS:multiprocess>)
# endif()

add_dependencies(${target} rcppsw-build)
