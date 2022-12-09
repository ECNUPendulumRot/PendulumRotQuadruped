#ifndef PR_ROBOT_MODEL_H
#define PR_ROBOT_MODEL_H

#include <memory>

#include <urdf/model.h>
#include <kdl/tree.hpp>

namespace pr_robot {

class Robot;
class RobotDescription;


/**
 * @brief The RobotModel class: stores total robot information
 */
class RobotModel {

public:

  /**
   * @brief load_from_string: load urdf model from a string, which maybe passed from ROS Param
   * @param urdf_string: string read from URDF
   */
  void load_from_string(const std::string& urdf_string);

  /**
   * @brief load_from_xml: load robot model from path to urdf
   * @param path: path to robot urdf file
   */
  void load_from_xml(const std::string& path);

  inline const KDL::Tree& get_kdl_tree() {
    return d_kdl_tree;
  }

  inline const urdf::Model& get_urdf_model() {
    return d_urdf_model;
  }

private:

  /**
   * @brief d_urdf_model holds the robot urdf model
   */
  urdf::Model d_urdf_model;

  /**
   * @brief d_kdl_tree: total kdl tree of the robot
   */
  KDL::Tree d_kdl_tree;

};

}// namespace pr_robot

#endif // PR_ROBOT_MODEL_H
