#pragma once

#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <memory>

namespace pr_robot {

class RobotDescription;

/**
 * @brief the class to desrcibe robot, solving the conflict between
 * different brand of quadrupeds
 */
using EndeffectorDescription = std::vector<std::pair<std::string, std::string>>;

/**
 * @typedef RobotDescriptionArray: std::vector<RobotDescription>
 */
using RobotDescriptionArray = std::vector<RobotDescription>;

/**
 * @typedef RobotDescriptionArrayPtr: std::shared_ptr<RobotDescriptionArray>
 */
using RobotDescriptionArrayPtr = std::shared_ptr<RobotDescriptionArray>;


class RobotDescription {

public:

  /**
   * @brief get the point of endeffector description array
   */
  inline EndeffectorDescription& get_endeffector_description() {
    return d_endeffector_description;
  }

  /**
   * @brief add endeffector description by key-value
   * @param key
   * @param value
   */
  inline void set_endeffector_description(std::string key, std::string value) {
    d_endeffector_description.emplace_back(key, value);
  }

private:

  /**
   * @brief the end effector name in urdf correspond to robot,
   * for example, if the robot is quadruped, the front right foot
   * name can be "FR_foot"
   */
  EndeffectorDescription d_endeffector_description;

};




}// namespace pr_robot

