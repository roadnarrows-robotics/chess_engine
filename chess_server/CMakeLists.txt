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

#catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    ChessCastling.msg
    ChessCheckMod.msg
    ChessColor.msg
    ChessEndGameStatus.msg
    ChessMove.msg
    ChessMoveStamped.msg
    ChessNewGameStatus.msg
    ChessPiece.msg
    ChessPos.msg
    ChessResult.msg 
)

add_service_files(
  DIRECTORY srv
  FILES
    AutoPlay.srv
    ComputeEnginesMove.srv
    GetGameState.srv
    GetPlayHistory.srv
    MakeAMoveAN.srv
    MakeAMove.srv
    MoveCompleted.srv
    Resign.srv
    SetDifficulty.srv
    StartNewGame.srv
    UndoMove.srv
)

add_action_files(
  DIRECTORY action 
  FILES 
    AutoPlay.action
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
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(chess_server
  #src/chess_as_auto_play.cpp
  #src/chess_as_get_engines_move.cpp
  src/chess_server_main.cpp
  #src/chess_engine_gnu.cpp
  src/chess_server.cpp
)

target_link_libraries(
  chess_server 
  chessengine
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

