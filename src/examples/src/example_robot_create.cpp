#include <ros/ros.h>
#include <ros/package.h>

#include "pendulum_quadruped/robot/pr_robot_generator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_robot_create");
  ros::NodeHandle nh;

  pr_robot::RobotGenerator<Quadruped> generator(USE_URDF, nh);
  auto quad_ptr = generator.get<0>();

}
