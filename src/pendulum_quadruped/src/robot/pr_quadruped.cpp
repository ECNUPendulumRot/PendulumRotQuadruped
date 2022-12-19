#include "robot/pr_quadruped.h"

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "robot/pr_robot_model.h"
#include "robot/pr_robot_description.h"


pr_robot::QuadrupedPtr pr_robot::Quadruped::create(const std::string &ns, pr_robot::RobotModel &model, pr_robot::RobotDescription &description)
{
  EndeffectorDescription& d = description.get_endeffector_description();

  // for quadruped, the foot number is fixed to 4, or it is invalid.
  assert(d.size() == 4);

  QuadrupedPtr quad_p(new Quadruped());
  for(auto it = d.begin(); it != d.end(); ++it) {
    KDL::Chain chain;
    model.get_kdl_tree().getChain("base", it->second, chain);
    quad_p->d_foothold_solvers[std::distance(d.begin(), it)] =
        KDLPositionSolverPtr(new KDL::ChainFkSolverPos_recursive(chain));
  }
  return quad_p;
}


void pr_robot::Quadruped::set_chain_state(pr_robot::LegState &state, pr_robot::Quadruped::QuadLeg leg_select)
{
  for(unsigned int i = 0; i < 3; i++) {
    // index 0 is the angle, and index 1 is the velocity
    d_joints[leg_select][0].set_joint_angle(state(i, 0));
    d_joints[leg_select][1].set_joint_velocity(state(i, 1));
  }
}
