#include <ros/ros.h>
#include <ros/package.h>

#include "pendulum_quadruped/robot/pr_robot_generator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_robot_state");
  ros::NodeHandle nh;

  std::string path_to_package = ros::package::getPath("a1_description");
  std::string path_to_description = path_to_package + "/config/pr_robot_description.yaml";
  std::string path_to_urdf = path_to_package + "/urdf/a1.urdf";

  pr_robot::RobotGenerator<Quadruped> generator(path_to_description, path_to_urdf);
  auto quad_ptr = generator.get<0>();
  quad_ptr->print_robot_info();
}
