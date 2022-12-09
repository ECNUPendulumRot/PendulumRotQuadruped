#include <ros/ros.h>
#include <ros/package.h>

#include "pendulum_quadruped/robot_new/pr_robot_generator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_robot_state");
  ros::NodeHandle nh;

  std::string path_to_package = ros::package::getPath("a1_description");
  std::string path_to_package = "../../a1_description";
  std::string path_to_description = "/home/sherman/quadruped/PendulumQuadruped/src/simulation/robot/a1_description/config/pr_robot_description.yaml";
  std::string path_to_urdf = "/home/sherman/quadruped/PendulumQuadruped/src/simulation/robot/a1_description/urdf/a1.urdf";

  pr_robot::RobotGenerator<Quadruped> generator(path_to_description, path_to_urdf);

  printf("generator initiated");

  auto [quad_ptr] = generator.get();

  quad_ptr->print_robot_info();
}
