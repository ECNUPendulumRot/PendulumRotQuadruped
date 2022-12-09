#ifndef PR_ROBOT_BASE_H
#define PR_ROBOT_BASE_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <memory>

#include "robot/pr_robot_component.h"
#include "robot/pr_pose_twist.h"

// foward declaration for chain in kdl library
namespace KDL {
class Chain;
class ChainFkSolverPos_recursive;
}


namespace pr_robot {

/**
 * @typedef RobotBase: PoseTwist
 */
using RobotBase = PoseTwist;

/**
 * @typedef EndEffectorStat: PoseTwist
 */
using EndEffectorStat = PoseTwist;

/**
 * @typedef ChainVector: std::vector<KDL::Chain&>
 */
using KDLChainVector = std::vector<std::reference_wrapper<KDL::Chain>>;

/**
 * @typedef JointChain: std::vector<Joint>
 */
using JointChain = std::vector<Joint>;

/**
 * @typedef JointChainVector: std::map<std::string, JointChain>
 */
using JointChainVector = std::vector<JointChain>;

/**
 * @typedef EndEffectorNameVec: std::map<std::string, JointChain>
 */
using EndEffectorNameVec = std::vector<std::string>;
/**
 * @typedef EndEffectorStatVector: std::vector<EndEffectorStat>
 */
using EndEffectorStatVector = std::vector<EndEffectorStat>;

///**
// * @typedef JointMap: std::map<std::string, Joint>
// */
//using JointMap = std::map<std::string, Joint>;

///**
// * @typedef LinkMap: std::map<std::string, Link>
// */
//using LinkMap = std::map<std::string, Link>;

/**
 * @typedef KDLPositionSolverPtr: std::shared_ptr<KDL::ChainFkSolverPos_recursive>
 * the foward position solver for position
 */
using KDLPositionSolverPtr = std::shared_ptr<KDL::ChainFkSolverPos_recursive>;

/**
 * @typedef KDLPositionSolverVec: std::vector<KDLPositionSolverPtr>
 */
using KDLPositionSolverVec = std::vector<KDLPositionSolverPtr>;


class Robot: public RobotBase {

public:

  /**
   * @brief The RobotType enum: several robot types that may support
   */
  enum {
    QUADRUPED, ///< legged robot with 4 legs
    BIPED,     ///< legged robot with 2 legs
    ARM,       ///< base-fixed robot of a link chain
    MIXED      ///< robot mixed with legged and other executors. only for generalized control
  };

  Robot() = default;

  /**
   * @brief Robot: constructor for robot
   * @param type: type of robot, maybe Quadruped, biped, arm or others
   * @param robot_name: name of robot
   */
  Robot(unsigned int type, std::string robot_name): d_type(type), d_robot_name(robot_name) {
    ;
  }

  /**
   * @brief add_endeffector_chain: add a base-endeffector kdl chain to d_endeffector_chains;
   * if the chain is added , the d_endeffector_status will emplace_back a pose at the end.
   * @param chain: the chain that will add to d_end_effector_chains
   */
  void add_endeffector_chain(std::string endeffector_name, KDL::Chain& chain);

  /**
   * @brief update_endeffector_stats: update the endeffector pose and velocity wrt base
   * @attention the method depends on kdl solver, the results depends on the chain.
   * any modification on the formation of chain will affect this method
   * @warning before call this method, make sure the state has been updated
   */
  void update_endeffector_position();

  void print_robot_info();

  /**
   * @brief get_joint_state_chain: get the joint state of a chain, including angles and velocity
   * @param joint_chain: the chain to get state
   * @return eigen vector of the joint state
   */
  inline DynamicMat<double> get_joint_state_chain(JointChain& joint_chain) {
    DynamicMat<double> m(joint_chain.size(), 2);
    int joint_num = int(joint_chain.size());
    for(int i = 0; i != joint_num; i++) {
      m(i, 1) = joint_chain[i].get_joint_angle();
      m(i, 2) = joint_chain[i].get_joint_velocity();
    }
    return m;
  }

  /**
   * @brief get_joint_angles_chain: get the joint angles of a chain
   * @param joint_chain: the chain to get state
   * @return eigen vector of the joint angle
   */
  inline DynamicVec<double> get_joint_angles_chain(JointChain& joint_chain) {
    DynamicVec<double> v(joint_chain.size());
    int joint_num = int(joint_chain.size());
    for(int i = 0; i != joint_num; i++) {
      v(i) = joint_chain[i].get_joint_angle();
    }
    return v;
  }

  /**
   * @brief get_joint_angles_chain: get the joint velocity of a chain
   * @param joint_chain: the chain to get state
   * @return eigen vector of the joint velocity
   */
  inline DynamicMat<double> get_joint_velocity_chain(JointChain& joint_chain) {
    DynamicVec<double> v(joint_chain.size());
    int joint_num = int(joint_chain.size());
    for(int i = 0; i != joint_num; i++) {
      v(i) = joint_chain[i].get_joint_velocity();
    }
    return v;
  }

/////////////////////////////// for future utilities ///////////////////////////////
//  /**
//   * @brief add_joint: add a new joint named as joint_name to d_joints
//   * @param joint_name: joint name that need to be added
//   * @param joint: the joint constructed outside
//   */
//  void add_joint(std::string joint_name, Joint& joint);

//  /**
//   * @brief add_link: add a new link named as link_name to d_links
//   * @param link_name: link name that need to be added
//   * @param link: the link constructed outside
//   */
//  void add_link(std::string link_name, Link& link);

protected:

  /**
   * @brief d_type: the type of robot. quadruped, biped, arm or mixed
   */
  unsigned int d_type;

  /**
   * @brief d_robot_name: the name of the robot. usually same to that in the loaded URDF fime
   */
  std::string d_robot_name;

  /**
   * @brief d_position_solvers: foward kinematics solver for endeffectors
   */
  KDLPositionSolverVec d_position_solvers;

  /**
   * @brief d_joint_chains: holds the vector of endeffector name to joints from the base to endeffector
   * @warning the order of d_joint_chains should be the same as d_endeffector_chains
   */
  JointChainVector d_joint_chains;

  /**
   * @brief d_endeffector_names: holds the name of endeffectors in the robot
   * @warning the order of d_joint_chains should be the same as d_endeffector_chains
   */
  EndEffectorNameVec d_endeffector_names;

  /**
   * @brief d_endeffector_stats: store the kinematic state of en endeffector
   * @warning the order of d_endeffector_stats should be the same as d_endeffector_chains
   * or it will fail in kinematics solving
   */
  EndEffectorStatVector d_endeffector_stats;

/////////////////////////////// for future utilities ///////////////////////////////

//  /**
//   * @brief d_endeffector_chains: the vector stores the chains that begin with base and end with endeffector.
//   * The chain is from base link to endeffector.
//   * For more information please visit https://www.orocos.org/wiki/Kinematic_Trees.html
//   */
//  KDLChainVector d_endeffector_chains;

//  /**
//   * @brief d_joints: the map stores all the joints with name
//   */
//  JointMap d_joints;

//  /**
//   * @brief d_links: the map stores all the links with name
//   */
//  LinkMap d_links;

};

}
#endif // PR_ROBOT_BASE_H
