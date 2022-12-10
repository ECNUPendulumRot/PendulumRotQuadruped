cmake_minimum_required(VERSION 2.8.3)
project(controller_plugin)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    pr_joint_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pr_joint_controller

    CATKIN_DEPENDS
        controller_interface
        hardware_interface
        pluginlib
        roscpp
        pr_joint_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library(pr_joint_controller SHARED src/pr_joint_controller.cpp
)
add_dependencies(pr_joint_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(pr_joint_controller ${catkin_LIBRARIES})

