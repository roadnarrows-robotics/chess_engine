cmake_minimum_required(VERSION 2.8.3)
project(chess_server)

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

add_message_files(
  DIRECTORY msg
  FILES 
    ChessCastlingMsg.msg
    ChessColorMsg.msg
    ChessEndGameStatusMsg.msg
    ChessMoveMsg.msg
    ChessMoveStatusMsg.msg
    ChessNewGameStatusMsg.msg
    ChessPieceMsg.msg
    ChessResultMsg.msg 
    ChessSquareMsg.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    AutoPlaySvc.srv
    GetBoardStateSvc.srv
    GetEnginesMoveSvc.srv
    GetPlayHistorySvc.srv
    MakeAMoveSANSvc.srv
    MakeAMoveSvc.srv
    ResignSvc.srv
    SetDifficultySvc.srv
    StartNewGameSvc.srv
)

add_action_files(
  DIRECTORY action 
  FILES 
    GetEnginesMove.action
)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

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

add_executable(chess_server
  src/chess_as_get_engines_move.cpp
  src/chess_server_main.cpp
  src/chess_engine_gnu.cpp
  src/chess_server.cpp
)

target_link_libraries(
  chess_server 
  chessengine
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(chess_server chess_server_gencpp)

#install(
#  PROGRAMS 
#    scripts/calibrate_arm
#    scripts/hek_panel
#    scripts/hek_teleop
#    scripts/joint_trajectory_demo 
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

