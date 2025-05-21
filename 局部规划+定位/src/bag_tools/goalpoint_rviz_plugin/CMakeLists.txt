cmake_minimum_required(VERSION 2.8.3)
project(goalpoint_rviz_plugin)

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


# ant01的目标点发布
set(HEADER_FILES1
        include/goalpoint_tool1.h
)
set(SRC_FILES1
    src/goalpoint_tool_ant01.cpp
)
add_library(goalpoint_rviz_plugin01
            ${SRC_FILES1} ${HEADER_FILES1}
)
add_dependencies(goalpoint_rviz_plugin01 ${goalpoint_rviz_plugin01_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goalpoint_rviz_plugin01 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})
# ant02的目标点发布
set(HEADER_FILES2
        include/goalpoint_tool2.h
)
set(SRC_FILES2
    src/goalpoint_tool_ant02.cpp
)
add_library(goalpoint_rviz_plugin02
            ${SRC_FILES2} ${HEADER_FILES2}
)
add_dependencies(goalpoint_rviz_plugin02 ${goalpoint_rviz_plugin02_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goalpoint_rviz_plugin02 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})
# ant03的目标点发布
set(HEADER_FILES3
        include/goalpoint_tool3.h
)
set(SRC_FILES3
    src/goalpoint_tool_ant03.cpp
)
add_library(goalpoint_rviz_plugin03
            ${SRC_FILES3} ${HEADER_FILES3}
)
add_dependencies(goalpoint_rviz_plugin03 ${goalpoint_rviz_plugin03_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goalpoint_rviz_plugin03 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})
# ant04的目标点发布
set(HEADER_FILES4
        include/goalpoint_tool4.h
)
set(SRC_FILES4
    src/goalpoint_tool_ant04.cpp
)
add_library(goalpoint_rviz_plugin04
            ${SRC_FILES4} ${HEADER_FILES4}
)
add_dependencies(goalpoint_rviz_plugin04 ${goalpoint_rviz_plugin04_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goalpoint_rviz_plugin04 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})
# ant05的目标点发布
set(HEADER_FILES5
        include/goalpoint_tool5.h
)
set(SRC_FILES5
    src/goalpoint_tool_ant05.cpp
)
add_library(goalpoint_rviz_plugin05
            ${SRC_FILES5} ${HEADER_FILES5}
)
add_dependencies(goalpoint_rviz_plugin05 ${goalpoint_rviz_plugin05_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goalpoint_rviz_plugin05 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})


# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(FILES plugin_description.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
