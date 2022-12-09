#ifndef PR_ROBOT_TYPES_H
#define PR_ROBOT_TYPES_H

#include <map>

#include "robot/pr_robot_base.h"

/**
 * @brief The QuadLeg enum: front right, front left, rear left, rear right
 */
enum class QuadLeg {
  FR, FL, RR, RL
};

/**
 * @typedef LegInfo: std::map<QuadLeg, EndEffectorStat>
 */
using LegInfo = std::map<QuadLeg, unsigned int>;

class Quadruped: public pr_robot::Robot {

public:

  Quadruped(unsigned int type, std::string robot_name): Robot(type, robot_name) {
    ;
  }

  /**
   * @brief get_foot_hold_position: get foot hold position of a leg
   * @param leg: which leg to get
   * @return vector of leg positions
   */
  Vec3<double> get_foot_hold_position(QuadLeg leg);

  /**
   * @brief get_foot_hold_position: get joint angles of a leg
   * @param leg: which leg to get
   * @return vector of leg joint angles
   */
  Vec3<double> get_leg_joints(QuadLeg leg);


  /**
   * @brief add_leg: add a leg to the quadruped
   * @param tag: name of the tag;
   * @param name: name of end effector
   *
   * @warning this method have to be called after setup_robot
   */
  void add_leg(const std::string& tag, const std::string& name);

  /**
   * @brief valid_leg_description: check a tag is valid
   * only valid with "front_right", "front_left", "rear_right", "rear_left"
   * @param s: string of the tag
   * @return whether it is valid
   */
  static bool valid_leg_tag(const std::string& s);


private:

  /**
   * @brief d_legs: stores the leg-vector id map
   */
  LegInfo d_legs;

  /**
   * @brief d_valid_tags_s: static vector that stores valid foot tags
   */
  static std::map<std::string, QuadLeg> s_valid_tags;

};


#endif // PR_ROBOT_TYPES_H
