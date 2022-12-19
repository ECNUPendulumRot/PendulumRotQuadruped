#include "robot/pr_description_loader.h"

#include <iostream>


pr_robot::RobotDescriptionArrayPtr pr_robot::DescriptionLoader::load_from_yaml(const std::string &path)
{
  YAML::Node node = YAML::LoadFile(path);
  RobotDescriptionArrayPtr array(new RobotDescriptionArray());

  // iterate the 'robot' node in the YAML file
  for(YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    RobotDescription robot_description;
    //iterate the endeffecter in the 'robot' node
    for(YAML::const_iterator it_in = it->second["end_effecter"].begin();
        it_in != it->second["end_effecter"].end();
        ++it_in) {
      robot_description.set_endeffector_description(it_in->first.as<std::string>(),
                                               it_in->second.as<std::string>());
    }
    array->push_back(robot_description);
  }

  return array;
}
