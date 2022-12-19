#include <ros/ros.h>

#include "pendulum_quadruped/robot/pr_robot_generator.h"
#include "pendulum_quadruped/state_update/pr_observer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_receive_joint_state");
  ros::NodeHandle nh("a1/default_sim");

  pr_robot::RobotGenerator<Quadruped> generator(USE_URDF, nh);
  auto quad_ptr = generator.get<0>();

  pr_state_updater::JointStateObserver<Quadruped> quadruped_joint_state_observer(quad_ptr, nh);

  ros::Rate r(500);
  while(ros::ok()){
    ros::spinOnce();
    quad_ptr->print_joint_angles();
    r.sleep();
  }
}
