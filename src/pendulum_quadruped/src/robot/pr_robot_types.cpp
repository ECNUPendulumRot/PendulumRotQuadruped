#include "robot/pr_robot_types.h"


std::map<std::string, unsigned int> Quadruped::s_valid_tags =
  {{"front_right", Quadruped::FR},
   {"front_left" , Quadruped::FL},
   {"rear_right" , Quadruped::RR},
   {"rear_left"  , Quadruped::RL}};


Vec3<double> Quadruped::get_foot_hold_position(QuadLeg leg)
{
  return d_endeffector_stats[d_legs[leg]].get_position();
}


Vec3<double> Quadruped::get_leg_joints(QuadLeg leg)
{
  return get_joint_chain_angle(d_joint_chains[d_legs[leg]]);
}


void Quadruped::add_leg(const std::string &tag, const std::string &name)
{
  for(auto it = s_valid_tags.begin(); it != s_valid_tags.end(); it++)
    if(it->first == tag){
      auto find_it = std::find(d_endeffector_names.begin(), d_endeffector_names.end(), name);
      d_legs.emplace(s_valid_tags[tag], std::distance(find_it, d_endeffector_names.begin()));
    }
}


bool Quadruped::valid_leg_tag(const std::string &s)
{
  for(auto it = s_valid_tags.begin(); it != s_valid_tags.end(); it++)
    if(it->first == s)
      return true;
  return false;
}


void Quadruped::set_joint_chain_state(DynamicMat<double> &states, unsigned int leg_select)
{
  pr_robot::Robot::set_joint_chain_state(states, d_joint_chains[leg_select]);
}

