cmake_minimum_required(VERSION 3.0.2)
project(examples)

find_package(catkin REQUIRED COMPONENTS
  pendulum_quadruped
  roscpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

link_libraries(${catkin_LIBRARIES})

add_executable(example_robot_create src/example_robot_create.cpp)

