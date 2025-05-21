cmake_minimum_required(VERSION 2.8.3)
project(msg_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Mission.msg
  MultiMissions.msg
  RobotInfor.msg
  LocalPath.msg
  LocalTarget.msg
  IPS.msg
  ConnectRobotID.msg
  Replan.msg
  RobotPosition.msg
  ViewPoint.msg
  SuperViewPoint.msg
  FreeSpace.msg
  FreeSpaceInfo.msg
  Frontier.msg
  FrontierInfo.msg
  FreeSpaceAndFrontierInfo.msg
  FreeSpaceAndFrontierInfos.msg
  ConnectionSuccess.msg

  Posegra.msg
  Edge.msg
  EdgeTable.msg

  MeetingResult.msg


  Key.msg
  Value.msg

  UnaryFactor.msg
  BinaryFactor.msg
  NonlinearFactorGraph.msg

  PoseGraph.msg

  MeetingInfo.msg
  ReceiveFlag.msg

  ScanContext.msg

  PointCloudInfo.msg
  PointCloudRequest.msg

  OccMap3d.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)