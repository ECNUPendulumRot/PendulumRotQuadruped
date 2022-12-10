cmake_minimum_required(VERSION 2.8.3)
project(visualization_plugin)

set (CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

find_package(gazebo REQUIRED)
find_package(Qt5Core REQUIRED)


catkin_package(
  CATKIN_DEPENDS
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
)

add_library(DrawGUIPlugin SHARED src/draw_gui_plugin.cpp)
target_link_libraries(DrawGUIPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Qt5Core_LIBRARIES})

add_library(DrawTrajectoryPlugin SHARED src/draw_trajectory_plugin.cpp)
target_link_libraries(DrawTrajectoryPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(unitreeFootContactPlugin SHARED src/foot_contact_plugin.cpp)
target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})




