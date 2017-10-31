cmake_minimum_required(VERSION 2.8.11)

project(rqt_chess)

#set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_STANDARD 11)

set(CE_ROOT ..)

# catkin required components
find_package(catkin 
  REQUIRED COMPONENTS
  #qt_build not needed
  roscpp 
)

# boost required components
find_package(Boost REQUIRED COMPONENTS system)

# qt packages
find_package(Qt5Widgets)

# no libs nor headers to export - only the executable
catkin_package()

# ui forms
file(GLOB QT_FORMS
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ui/*.ui)

# resources
file(GLOB QT_RESOURCES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  resources/*.qrc)

# potential moc inpurt (those with QObject)
file(GLOB_RECURSE QT_MOC
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
  include/*.h)

# source files
file(GLOB_RECURSE QT_SOURCES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
  src/*.cpp)

# needed to find generated headers and source in build directory
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# qt make tweaks that don't seem to be needed when using the wrap commands below

# Meta Object Compiler
#set(CMAKE_AUTOMOC ON)

#
# User Interface
#
#set(autouic_options
#  -o include/rqt_chess/ui_rqtchess.h
#)

#set_property(SOURCE rqtchess.ui APPEND PROPERTY
#  INTERFACE_AUTOUIC_OPTIONS ${autouic_options}
#)

# User Interface Compiler
#set(CMAKE_AUTOUIC ON)

#
# Resource Compiler
#
#set(CMAKE_AUTORCC ON)

#
# Qt cmake wraps.
#
# Note that the intermediate headers and source are placed in the build
# directory: ${workspace}/build/chess_engine/rqt_chess
#
qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

#message(STATUS "DBG: FORMS "  ${QT_FORMS})
#message(STATUS "DBG: RSRC  "  ${QT_RESOURCES})
#message(STATUS "DBG: MOC   "  ${QT_MOC})
#message(STATUS "DBG: SRC   "  ${QT_SOURCES})

include_directories(
  include
  ${CE_ROOT}/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(rqt_chess
  ${QT_SOURCES}
  ${QT_RESOURCES_CPP}
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
)

target_link_libraries(rqt_chess
  Qt5::Widgets
  chessengine
  ${catkin_LIBRARIES}
)

install(TARGETS rqt_chess RUNTIME DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
