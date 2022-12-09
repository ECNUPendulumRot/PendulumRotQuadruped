#include "robot/pr_robot_base.h"

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>


void pr_robot::Robot::add_endeffector_chain(std::string chain_name, KDL::Chain &chain)
{
  // maybe used in future
  //d_endeffector_chains.push_back(chain);
  d_endeffector_stats.emplace_back(EndEffectorStat());

  unsigned int num_joints = chain.getNrOfJoints();
  d_joint_chains.emplace_back(JointChain(num_joints));

  d_position_solvers.emplace_back(KDLPositionSolverPtr(new KDL::ChainFkSolverPos_recursive(chain)));
  d_endeffector_names.push_back(chain_name);
}


void pr_robot::Robot::update_endeffector_position()
{
  KDL::Frame result;

  unsigned int index = 0;
  for(auto solver_it = d_position_solvers.begin(); solver_it != d_position_solvers.end(); solver_it++){
    KDL::JntArray joint_angles;
    joint_angles.data = get_joint_state_chain(d_joint_chains[index]).col(0);
    int fk_result = (*solver_it)->JntToCart(joint_angles, result);

    // debug: whether the JntToCart succeeded
    assert(fk_result > 0);

    d_endeffector_stats[index].set_position(result.p);
    d_endeffector_stats[index].set_rotation(result.M);
  }
}

void pr_robot::Robot::print_robot_info() {
  printf("Robot: %s \n", d_robot_name.c_str());
  std::for_each(d_endeffector_names.begin(), d_endeffector_names.end(), [](std::string &s){
    std::cout << s << " ";
  });
  std::cout << std::endl;
}

/////////////////////////////// for future utilities ///////////////////////////////

//void pr_robot::Robot::add_joint(std::string joint_name, pr_robot::Joint &joint)
//{
//  d_joints.emplace(joint_name, joint);
//}


//void pr_robot::Robot::add_link(std::string link_name, pr_robot::Link &link)
//{
//  d_links.emplace(link_name, link);
//}
