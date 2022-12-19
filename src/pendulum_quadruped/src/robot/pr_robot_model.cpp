#include "robot/pr_robot_model.h"

#include <kdl_parser/kdl_parser.hpp>


void pr_robot::RobotModel::load_from_string(const std::string &urdf_string)
{
  d_urdf_model.initString(urdf_string);

  kdl_parser::treeFromUrdfModel(d_urdf_model, d_kdl_tree);
}


void pr_robot::RobotModel::load_from_xml(const std::string &path)
{
  d_urdf_model.initFile(path);

  kdl_parser::treeFromUrdfModel(d_urdf_model, d_kdl_tree);
}


