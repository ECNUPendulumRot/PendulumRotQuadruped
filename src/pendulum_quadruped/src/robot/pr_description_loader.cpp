#include "robot/pr_description_loader.h"

#include <iostream>


pr_robot::RobotDescriptionArrayPtr pr_robot::DescriptionLoader::load_from_yaml(const std::string &path)
{
  YAML::Node node = YAML::LoadFile(path);

  RobotDescriptionArrayPtr array(new RobotDescriptionArray());

  for(YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    RobotDescription robot_description;
    robot_description.set_type(it->second["type"].as<unsigned>());
    robot_description.set_name(it->second["name"].as<std::string>());
    //iterate the endeffecter of the robot in yaml
    for(YAML::const_iterator it_in = it->second["end_effecter"].begin();
        it_in != it->second["end_effecter"].end();
        it_in++){
      robot_description.set_endeffector_description(it_in->first.as<std::string>(),
                                               it_in->second.as<std::string>());
    }
    array->add_description(robot_description);
  }

  return array;
}
