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

# RDK, DHP TODO Create rnrpkgConfig.cmake
# find_package(rnr
#   REQUIRED COMPONENTS hekateros Dynamixel botsense netmsgs rnr_appkit rnr
# )
find_library(
  rnr_LIBRARIES
  NAMES rnr
  PATHS /prj/lib /prj/lib/rnr /usr/local/lib /usr/local/lib/rnr
)

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
  src/ceError.cpp
  src/ceGame.cpp
  src/ceMove.cpp
  src/ceUtils.cpp
)

#target_link_libraries(
#  ${rnr_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

install(TARGETS chessengine
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ../include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
