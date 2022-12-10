#include <ros/ros.h>

#include "pr_gazebo/gazebo_model_spawn.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spawn_model");
  ros::NodeHandle nh;

  const char* type_cstr = argv[1];
  const char* name_cstr = argv[2];
  std::string robot_type(type_cstr);
  std::string robot_name(name_cstr);

  pr_gazebo::GazeboSpawner manager(robot_type, robot_name, nh);

  manager.set_model_position(0, 0, 0.4);
  manager.set_model_orientation(1, 0, 0, 0);

  /// "robot_description" was loaded in .launch file
  /// make sure the robot hasn't been created
  if(!manager.spawn_model("robot_description")){
    ROS_ERROR("Fail to spawn model in gazebo: %s.%s", type_cstr, name_cstr);
    return 1;
  }

  printf("----------------------------------------------------------------\n");
  printf("Press Enter key to start controllers.\n");
  printf("----------------------------------------------------------------\n");

  getchar();
  manager.load_controllers();
  manager.start_controllers();

  printf("----------------------------------------------------------------\n");
  printf("Press Enter key to delete controllers and model.\n");
  printf("----------------------------------------------------------------\n");

  getchar();

  if(!manager.stop_controllers()){
    ROS_ERROR("Fail to stop controllers in gazebo: %s.%s", type_cstr, name_cstr);
    return 1;
  }

  manager.unload_controllers();
  manager.delete_model();

  return 0;
}
