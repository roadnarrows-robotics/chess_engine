cmake_minimum_required(VERSION 2.8.3)

project(chess_tools)

set(CE_ROOT ..)

set(CMAKE_CXX_STANDARD 11)

set(TOOL_LIST chess_parser chess_cli)

find_package(catkin 
  REQUIRED COMPONENTS 
  std_msgs 
  genmsg 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr-sdk_DIR /usr/local/lib/cmake/rnr)
find_package(rnr-sdk REQUIRED)

catkin_package(
  DEPENDS Boost
  INCLUDE_DIRS ${CE_ROOT}/include
  LIBRARIES chessengine
)

if(READLINE_FOUND)
  add_definitions(-DUSE_READLINE)
endif()

add_definitions(-DLOG -DLOGMOD="chess_tools")

include_directories(
  include 
  ${CE_ROOT}/include
  ${rnr-sdk_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${READLINE_INCLUDE_DIRS}
)

add_executable(chess_parser
  src/chess_parser.cpp
)

target_link_libraries(chess_parser
  chessengine
  ${rnr-sdk_LIBRARIES}
  ${catkin_LIBRARIES}
  ${READLINE_LIBRARIES}
)

add_executable(chess_cli
  src/chess_cli.cpp
)

target_link_libraries(chess_cli
  chessengine
  ${rnr-sdk_LIBRARIES}
  ${catkin_LIBRARIES}
  ${READLINE_LIBRARIES}
)

install(
  PROGRAMS 
    chess_cli
    chess_parser
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
