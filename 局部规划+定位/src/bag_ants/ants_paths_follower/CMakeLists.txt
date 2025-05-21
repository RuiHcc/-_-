cmake_minimum_required(VERSION 3.0.2)
project(ants_paths_follower)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  geometry_msgs
  common_private_msgs
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  geometry_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/ants_paths_follower
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)
add_library(cs_lib2
  include/ants_paths_follower/cubic_spline.h  src/cubic_spline.cpp
)

add_executable(ants_paths_follower src/pathFollower.cpp)
target_link_libraries(ants_paths_follower ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(UdpRemote src/UdpRemote.cpp)
target_link_libraries(UdpRemote ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(UdpVehicle src/UdpVehicle.cpp)
target_link_libraries(UdpVehicle ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ants_paths_follower_swarm src/pathFollower_swarm.cpp)
target_link_libraries(ants_paths_follower_swarm ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(liner_speed_regulator src/liner_speed_regulator.cpp)
target_link_libraries(liner_speed_regulator  cs_lib2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(localpos_2_globalpos src/localpos_2_globalpos.cpp)
target_link_libraries(localpos_2_globalpos ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(ants_paths_follower_2_target src/pathFollower_2_target.cpp)
target_link_libraries(ants_paths_follower_2_target ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#############
## Install ##
#############
# install(TARGETS ants_paths_follower
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# install(DIRECTORY paths/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/paths
# )
