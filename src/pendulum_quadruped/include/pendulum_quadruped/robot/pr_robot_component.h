#ifndef PR_ROBOT_COMPONENT_H
#define PR_ROBOT_COMPONENT_H

#include <cmath>
#include <string>

#include "common/pr_eigen_types.h"

namespace pr_robot {

/**
 * @brief The Joint class: describes a joint in the robot
 * including joint angle and joint velocity
 */
class Joint {

public:

  Joint() {
    d_joint_angle    = 0.0;
    d_joint_velocity = 0.0;
    d_min_angle      = 0.0;
    d_max_angle      = M_PI;
  }

  /**
   * @brief Joint: contructor for Joint
   * @param min_angle: the min angle read from URDF
   * @param max_angle: the max angle read from URDF
   */
  Joint(double min_angle, double max_angle): d_min_angle(min_angle), d_max_angle(max_angle) {
    ;
  }

  /**
   * @brief get_joint_angle: getter for d_joint_angle
   * @return d_joint_angle
   */
  inline double get_joint_angle() {
    return d_joint_angle;
  }

  /**
   * @brief get_joint_velocity: getter for d_joint_velocity
   * @return
   */
  inline double get_joint_velocity() {
    return d_joint_velocity;
  }

  /**
   * @brief set_joint_angle: setter for d_joint_angle;
   * @param joint_angle: angle being observed or estimated
   */
  inline void set_joint_angle(double joint_angle) {
    d_joint_angle = joint_angle;
  }

  /**
   * @brief set_joint_velocity: setter for d_joint_velocity
   * @param joint_velocity: velocity being observed or estimated
   */
  inline void set_joint_velocity(double joint_velocity) {
    d_joint_velocity = joint_velocity;
  }

  /**
   * @brief set_joint_limitation: setter for d_min_angle and d_max_angle
   * @param min_angle: minimum angle read from URDF
   * @param max_angle: maximum angle read from URDF
   */
  inline void set_joint_limitation(double min_angle, double max_angle) {
    d_min_angle = min_angle;
    d_max_angle = max_angle;
  }

private:

  /**
   * @brief d_joint_angle: the angle of the joint
   */
  double d_joint_angle;

  /**
   * @brief d_joint_velocity: the speed of the joint
   */
  double d_joint_velocity;

  /**
   * @brief d_min_angle: the minimun angle for the d_joint_angle
   */
  double d_min_angle;

  /**
   * @brief d_max_angle: the maximun angle for the d_joint_angle
   */
  double d_max_angle;

};


class Link {

public:

  Link() {
    d_com.setZero();
    d_inertia.setZero();
    d_mass = 0.0;
  }

  /**
   * @brief Link: constructor for Link
   * @param com: centor of mass defined in URDF
   * @param inertia: inertia defined in URDF
   * @param mass: mass defined in URDF
   * @param parent_joint: parent joint defined in URDF
   * @param child_joint: child joint defined in URDF
   */
  Link(const Vec3<double>& com,
       const Mat3<double> inertia,
       double mass,
       std::string parent_joint,
       std::string child_joint = std::string()):
    d_com(com), d_inertia(inertia), d_mass(mass), d_parent_joint(parent_joint), d_child_joint(child_joint) {

  }

private:

  /**
   * @brief d_com: CoM position of the link
   * @attention this value is with respect to the last joint frame
   */
  Vec3<double> d_com;

  /**
   * @brief d_inertia: the inertia of the link
   * @attention this value is with respect to the frame of CoM
   */
  Mat3<double> d_inertia;

  /**
   * @brief d_mass: the mass of the link;
   */
  double d_mass;

  /**
   * @brief d_parent_joint: the name of parent joint in URDF tree. Empty if no parent exists
   */
  std::string d_parent_joint;

  /**
   * @brief d_child_joint: the name of child joint in URDF tree. Empty if no child exists
   */
  std::string d_child_joint;
};
}

#endif // PR_ROBOT_COMPONENT_H
