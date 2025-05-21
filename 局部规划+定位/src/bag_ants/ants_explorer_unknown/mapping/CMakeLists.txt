cmake_minimum_required(VERSION 3.0.2)
project(mapping)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  # quadrotor_msgs
  roscpp
  nodelet
  tf
  pcl_ros
  pcl_conversions
  cv_bridge
  msg_utils
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS  roscpp nodelet tf pcl_ros pcl_conversions cv_bridge
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# local map using 3d circular buffer
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cc
  src/surface_extract.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# mapping node
add_library(${PROJECT_NAME}_nodelet
  src/${PROJECT_NAME}_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
)

# mapping visualization node
add_executable(${PROJECT_NAME}_vis_node
  src/${PROJECT_NAME}_vis_node.cpp)
target_link_libraries(${PROJECT_NAME}_vis_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(${PROJECT_NAME}_vis_node
  ${catkin_EXPORTED_TARGETS}
)

# mapping unity lidar pointcloud re-timestamp node
add_executable(${PROJECT_NAME}_sim_reload_node
  src/${PROJECT_NAME}_sim_reload_node.cpp)
target_link_libraries(${PROJECT_NAME}_sim_reload_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(${PROJECT_NAME}_sim_reload_node
  ${catkin_EXPORTED_TARGETS}
)

## 添加可执行文件
add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)
## 链接库文件
target_link_libraries(obstacle_detection_node
  ${catkin_LIBRARIES}
)
## 声明依赖
add_dependencies(obstacle_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})