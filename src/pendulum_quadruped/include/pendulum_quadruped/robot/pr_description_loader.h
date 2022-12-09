#ifndef PR_DESCRITION_LOADER_H
#define PR_DESCRITION_LOADER_H

#include <string>
#include <yaml-cpp/yaml.h>

#include "robot/pr_robot_description.h"

namespace pr_robot {

/**
 * @typedef RobotDescriptionArrayPtr: std::shared_ptr<RobotDescriptionArray>
 */
using RobotDescriptionArrayPtr = std::shared_ptr<RobotDescriptionArray>;

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

#endif // PR_DESCRITION_LOADER_H
