cmake_minimum_required(VERSION 2.8.3)
project(libchessengine)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

#catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include 
  ../include
  /prj/include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(chessengine
  src/ceBoard.cpp
  src/ceChess.cpp
  src/ceCore.cpp
  src/ceEngine.cpp
  src/ceError.cpp
  src/ceGame.cpp
  src/ceMove.cpp
  src/ceParser.cpp
  src/ceSquare.cpp
  src/ceUtils.cpp
)

install(TARGETS chessengine
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ../include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
