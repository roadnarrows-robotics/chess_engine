cmake_minimum_required(VERSION 2.8.3)
project(libchessengine)

find_package(catkin 
  REQUIRED COMPONENTS 
  roscpp 
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  DEPENDS Boost
  INCLUDE_DIRS ../include
  LIBRARIES chessengine
)

include_directories(
  ../include
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

# not actually install to devel/include, but other packages can include
install(DIRECTORY ../include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
