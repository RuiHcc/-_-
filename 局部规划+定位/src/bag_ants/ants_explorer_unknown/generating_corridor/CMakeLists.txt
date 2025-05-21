cmake_minimum_required(VERSION 2.8.3)
project(generating_corridor)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES star_cvx
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions 
)
include_directories(
  include
  ThirdParty
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

add_library(star_cvx STATIC
    ${${PROJECT_NAME}_SRCS})

target_link_libraries(star_cvx
   ${catkin_LIBRARIES}
)

