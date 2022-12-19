#pragma once

#include <map>

#include "robot/pr_robot_base.h"

namespace pr_robot {

// forward declaration
class Quadruped;
class RobotModel;
class RobotDescription;

/**
 * @typedef QuadrupedPtr: std::shared_ptr<Quadruped>
 */
using QuadrupedPtr = std::shared_ptr<Quadruped>;

/**
 * @typedef LegInfo: std::map<QuadLeg, EndEffectorStat>
 */
using LegInfo = std::map<unsigned int, unsigned int>;

/**
 * @typedef LegChain: std::array<pr_robot::Joint, 3>
 */
using LegChain = std::array<pr_robot::Joint, 3>;

/**
 * @typedef LegState: Eigen::Matrix<double, 3, 2>
 * 2 vectors, one joint angles and second joint velocity
 */
using LegState = Eigen::Matrix<double, 3, 2>;

class Quadruped: public RobotBase {

public:

  /**
   * @brief The QuadLeg enum: front right, front left, rear left, rear right
   */
  enum QuadLeg {
    FR, FL, RR, RL
  };

  Quadruped() = default;

  /**
   * @brief create: factory function for quadruped
   * @return
   */
  static QuadrupedPtr create(const std::string &name_space, pr_robot::RobotModel &model, pr_robot::RobotDescription &description);

  /**
   * @brief get_foot_hold_position: get foot hold position of a leg
   * @param leg: which leg to get
   * @return vector of leg positions
   */
  inline Vec3<double> get_endeffector_position(QuadLeg leg) {
    return d_footholds_stat[leg].get_position();
  }

  /**
   * @brief get_foot_hold_position: get joint angles of a leg
   * @param leg: which leg to get
   * @return vector of leg joint angles
   */
  inline Vec3<double> get_joints_angle(QuadLeg leg) {
    return Vec3<double>(d_joints[leg][0].get_joint_angle(),
                        d_joints[leg][1].get_joint_angle(),
                        d_joints[leg][2].get_joint_angle());
  }

  /**
   * @brief set_joint_chain_state: set joint state to a chain, in order from base to endeffector
   * @attention the
   * @param joints: the vector of observed joints
   * @param leg_select: the leg to set states
   */
  void set_chain_state(LegState& state, QuadLeg leg_select);

  /**
   * @brief set_joint_state: set the state of one koint
   * @param chain_select: which chain to update. for quadruped, it will be 0, 1, 2 or 3
   * @param joint_select: which joint in the chain to update
   * @param angle: the observed joint angle
   * @param velocity: the observed joint velocity
   *
   * @attention this function is just for observers
   */
  inline void set_joint_state(unsigned int chain_select, unsigned int joint_select, double angle, double velocity) {
    d_joints[chain_select][joint_select].set_joint_angle(angle);
    d_joints[chain_select][joint_select].set_joint_velocity(velocity);
  }

  /**
   * @brief print_joint_angles: only for debug
   */
  void print_joint_angles(){
    printf("[Quadruped]\n");
    for (unsigned i = 0; i < 4; i++) {
      for (unsigned j = 0; j < 3; j++) {
        std::cout << d_joints[i][j].get_joint_angle() << " ";
      }
      std::cout << std::endl;
    }
  }

  /**
   * @brief get_state_info_pair: a function to tell obverser how to update joint state
   * @attention when user define a new robot type, please implement this function with same name and same return type
   */
  static constexpr auto get_state_info_pair() {
    return std::pair<unsigned, unsigned>(4, 3);
  }

private:

  /**
   * @brief d_legs: stores the leg-vector id map
   */
  LegInfo d_legs;

  /**
   * @brief d_joints: stores all the joints in a quadruped
   * splitted into 4 legs
   */
  std::array<LegChain, 4> d_joints;

  /**
   * @brief d_foothold_solvers: KDL solvers for foothold position
   */
  std::array<KDLPositionSolverPtr, 4> d_foothold_solvers;

  /**
   * @brief d_footholds_stat: stores the foothold states of a quadruped.
   * four legs
   */
  std::array<pr_robot::EndEffectorStat, 4> d_footholds_stat;

};

}// namespace pr_robot

