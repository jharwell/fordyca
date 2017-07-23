################################################################################
# Project Local Configuration                                                  #
################################################################################
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

include_directories(
  /usr/include/lua5.2
  ${Qt5Widgets_INCLUDE_DIRS}
  )
