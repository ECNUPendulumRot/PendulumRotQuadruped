#include "robot/pr_robot_description.h"


std::ostream& pr_robot::operator<<(std::ostream &os, pr_robot::RobotDescription &d)
{
  os << "name:" << d.d_name << std::endl
     << "type:" << d.d_type << std::endl
     << "endeffecters: #QUADRUPED 0, BIPED 1, ARM 2, MIXED 3"<<std::endl;

  for (auto it = d.get_endeffector_description().begin(); it!=d.get_endeffector_description().end(); ++it) {
    os <<"  "<< it->first<<": "<< it->second<<std::endl;
  }
  return os;
}

