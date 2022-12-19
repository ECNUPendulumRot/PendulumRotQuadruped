#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

#include "robot/pr_robot_description.h"

namespace pr_robot {

class DescriptionLoader {

public:

  /**
   * @brief give a absolutly path and read the .yaml file,
   * and save the message into description array
   * @param path
   */
  static RobotDescriptionArrayPtr load_from_yaml(const std::string &path);

};

} // namespace pr_robot

