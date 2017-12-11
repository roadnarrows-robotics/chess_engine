cmake_minimum_required(VERSION 2.8.3)

project(chessengine)

#set(CMAKE_VERBOSE_MAKEFILE ON)

set(CE_ROOT ..)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin 
  REQUIRED COMPONENTS 
  roscpp 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr-sdk_DIR /usr/local/lib/cmake/rnr)
find_package(rnr-sdk REQUIRED)

catkin_package(
  DEPENDS Boost
  INCLUDE_DIRS ${CE_ROOT}/include
  LIBRARIES ${PROJECT_NAME}
)

add_definitions(-DLOG -DLOGMOD="libchessengine" -DLOGMOD_COLOR=LOG_COLOR_BLUE)

include_directories(
  ${CE_ROOT}/include
  ${rnr-sdk_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ceBoard.cpp
  src/ceChess.cpp
  src/ceCore.cpp
  src/ceEngine.cpp
  src/ceError.cpp
  src/ceGame.cpp
  src/ceMove.cpp
  src/ceParser.cpp
  src/cePlayer.cpp
  src/ceSquare.cpp
  src/ceUtils.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${rnr-sdk_LIBRARIES}
  ${catkin_LIBRARIES}
)

# does not seem to work.
install(DIRECTORY ${CE_ROOT}/include/chess_engine/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/chess_engine/
  #FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
