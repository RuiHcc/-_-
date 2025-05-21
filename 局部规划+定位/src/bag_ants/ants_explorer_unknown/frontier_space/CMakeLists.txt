cmake_minimum_required(VERSION 2.8.3)
project(frontier_space)

set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
    mapping
    generating_corridor
    roscpp
    std_msgs
    nav_msgs
    visualization_msgs
    geometry_msgs
    cv_bridge
    message_generation
    msg_utils
)


find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3)
# find_package(spectra)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()



## Generate messages in the 'msg' folder
add_message_files(
    FILES
    FrontierClusterMsg.msg
    FrontierClusterListMsg.msg
    FreeSpaceInfoMsg.msg
    FreeSpaceMsg.msg
)

 
## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    #message_generation
 )




include_directories(
    include
    SYSTEM
    third_party
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    # ${spectra_INCLUDE_DIRS}
)
MESSAGE( ERROR "catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}.")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES frontier_space 
  CATKIN_DEPENDS msg_utils  message_runtime generating_corridor mapping
) 

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror


# add_library(frontier_space STATIC
#     src/explorer_point_finder.cpp
#     src/Cluster.cpp
#     src/FreeSpace.cpp
#     src/PoseGraph.cpp
# )
add_executable(frontier_space_node
  src/Frontier_Node.cpp
  src/explorer_point_finder.cpp
  src/Cluster.cpp
  src/FreeSpace.cpp
  src/PoseGraph.cpp
)

target_link_libraries(frontier_space_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)