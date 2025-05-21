cmake_minimum_required(VERSION 2.8.3)
project(waypoint_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
)

set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
 message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
 find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
 include(${QT_USE_FILE})
else()
 message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
 find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
 set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES  ${PROJECT_NAME}01 ${PROJECT_NAME}02 ${PROJECT_NAME}03 ${PROJECT_NAME}04 ${PROJECT_NAME}05
  CATKIN_DEPENDS roscpp rviz
)

include_directories(
 include
  ${QT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 第1个
set(HEADER_FILES1
        include/waypoint_tool1.h
)
set(SRC_FILES1
    src/waypoint_tool1.cpp
)

add_library(${PROJECT_NAME}01
            ${SRC_FILES1} ${HEADER_FILES1}
)

add_dependencies(${PROJECT_NAME}01 ${${PROJECT_NAME}01_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}01 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})

# 第2个
set(HEADER_FILES2
        include/waypoint_tool2.h
)
set(SRC_FILES2
    src/waypoint_tool2.cpp
)

add_library(${PROJECT_NAME}02
            ${SRC_FILES2} ${HEADER_FILES2}
)

add_dependencies(${PROJECT_NAME}02 ${${PROJECT_NAME}02_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}02 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})

# 第3个
set(HEADER_FILES3
        include/waypoint_tool3.h
)
set(SRC_FILES3
    src/waypoint_tool3.cpp
)

add_library(${PROJECT_NAME}03
            ${SRC_FILES3} ${HEADER_FILES3}
)

add_dependencies(${PROJECT_NAME}03 ${${PROJECT_NAME}03_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}03 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})

# 第4个
set(HEADER_FILES4
        include/waypoint_tool4.h
)
set(SRC_FILES4
    src/waypoint_tool4.cpp
)

add_library(${PROJECT_NAME}04
            ${SRC_FILES4} ${HEADER_FILES4}
)

add_dependencies(${PROJECT_NAME}04 ${${PROJECT_NAME}04_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}04 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})

# 第5个
set(HEADER_FILES5
        include/waypoint_tool5.h
)
set(SRC_FILES5
    src/waypoint_tool5.cpp
)

add_library(${PROJECT_NAME}05
            ${SRC_FILES5} ${HEADER_FILES5}
)

add_dependencies(${PROJECT_NAME}05 ${${PROJECT_NAME}05_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}05 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})



# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(FILES plugin_description.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
