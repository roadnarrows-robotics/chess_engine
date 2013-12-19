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

#add_message_files(
#  DIRECTORY msg
#  FILES 
#    HekOpState.msg 
#    ProductInfo.msg 
#    HekJointStateExtended.msg 
#    HekRobotStatusExtended.msg
#    ServoHealth.msg
#)

#add_service_files(
#  DIRECTORY srv
#  FILES
#    Calibrate.srv 
#    ClearAlarms.srv 
#    CloseGripper.srv 
#    EStop.srv 
#    Freeze.srv 
#    GetProductInfo.srv 
#    GotoBalancedPos.srv 
#    GotoParkedPos.srv 
#    GotoZeroPt.srv 
#    IsAlarmed.srv 
#    IsCalibrated.srv 
#    IsDescLoaded.srv
#    OpenGripper.srv
#    Release.srv
#    ResetEStop.srv 
#    SetRobotMode.srv
#    Stop.srv
#)

#add_action_files(
#  DIRECTORY action 
#  FILES 
#    Calibrate.action
#)

## Generate added messages/services/actions with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  actionlib_msgs
#  std_msgs
#)

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
  src/chess_server.cpp
  src/chess_backend_gnu.cpp
)

target_link_libraries(
  chess_server 
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

