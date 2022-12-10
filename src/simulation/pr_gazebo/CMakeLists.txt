cmake_minimum_required(VERSION 3.0.2)
project(pr_gazebo)

find_package(catkin REQUIRED roscpp tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pr_gazebo
  CATKIN_DEPENDS roscpp tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



file(GLOB_RECURSE sources "src/*.cpp")

add_library(pr_gazebo ${sources})

target_link_libraries(pr_gazebo ${catkin_LIBRARIES})

link_libraries(${catkin_LIBRARIES} pr_gazebo)

add_executable(spawn_model node/spawn_model.cpp)
add_executable(gazebo_tf_broadcaster node/tf_broadcaster.cpp)
