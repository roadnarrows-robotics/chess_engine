cmake_minimum_required(VERSION 2.8.3)

project(chess_tools)

set(CE_ROOT ..)

set(TOOL_LIST chess_parser) # chess_cli)

find_package(catkin 
  REQUIRED COMPONENTS 
  std_msgs 
  genmsg 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

include(${CE_ROOT}/chess_engine/FindReadline.cmake)

catkin_package(
  DEPENDS Boost
  INCLUDE_DIRS ${CE_ROOT}/include
  LIBRARIES chessengine
)

if(READLINE_FOUND)
  add_definitions(-DUSE_READLINE)
endif()

include_directories(
  include 
  ${CE_ROOT}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${READLINE_INCLUDE_DIRS}
)

#add_executable(chess_parser
#  src/chess_parser.cpp
#)

#target_link_libraries(chess_parser
#  chessengine
#  ${catkin_LIBRARIES}
#  ${READLINE_LIBRARIES}
#)

#add_executable(chess_cli
#  src/chess_cli.cpp
#  src/readline_wrapper.cpp
#)

#target_link_libraries(chess_cli
#  chessengine
#  ${catkin_LIBRARIES}
#)

#install(TARGETS 
#  ${TOOL_LIST}
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
