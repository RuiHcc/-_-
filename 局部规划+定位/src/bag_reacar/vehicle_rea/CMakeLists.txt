cmake_minimum_required(VERSION 2.8.3)
project(vehicle_rea)

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
)

# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
#   "${PROJECT_SOURCE_DIR}/include"
#   /usr/local/include # Location when using 'make system_install'
#   /usr/include       # More usual location (e.g. when installing using a package)
# )

# ## Specify additional locations for library files
# link_directories(
#   /usr/local/lib # Location when using 'make system_install'
#   /usr/lib       # More usual location (e.g. when installing using a package)
# )
# ## 单车仿真
# ## Declare executables
# add_executable(vehicleSimulator src/vehicleSimulator.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(vehicleSimulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# ## 两车仿真
# ## Declare executables
# add_executable(vehicleSimulator2car src/vehicleSimulator2car.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(vehicleSimulator2car ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# ## 三车仿真
# ## Declare executables
# add_executable(vehicleSimulator3car src/vehicleSimulator3car.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(vehicleSimulator3car ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# ## 四车仿真
# ## Declare executables
# add_executable(vehicleSimulator4car src/vehicleSimulator4car.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(vehicleSimulator4car ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# ## 五车仿真
# ## Declare executables
# add_executable(vehicleSimulator5car src/vehicleSimulator5car.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(vehicleSimulator5car ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


# install(TARGETS vehicleSimulator
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# install(DIRECTORY log/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/log
# )
# install(DIRECTORY mesh/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh
# )
# install(DIRECTORY rviz/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
# )
# install(DIRECTORY urdf/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
# )
# install(DIRECTORY world/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
# )
