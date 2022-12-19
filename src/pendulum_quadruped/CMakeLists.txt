cmake_minimum_required(VERSION 3.0.2)
project(pendulum_quadruped)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp urdf quadprogpp kdl_parser sensor_msgs roslib
)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pendulum_quadruped
    CATKIN_DEPENDS quadprogpp urdf sensor_msgs roslib
)


file(GLOB_RECURSE sources "src/*.cpp")

add_library(pendulum_quadruped ${sources})

target_include_directories(pendulum_quadruped PUBLIC include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME} PUBLIC ${orocos_kdl_LIBRARIES} ${YAML_CPP_LIBRARIES} Eigen3::Eigen)
