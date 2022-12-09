#ifndef PR_POSE_TWIST_H
#define PR_POSE_TWIST_H

#include "common/pr_eigen_types.h"

#include "kdl/frames.hpp"

namespace pr_robot {

template<typename Scale>
using Twist = Vec6<Scale>;

class PoseTwist {

public:

  PoseTwist() {
    d_position.setZero();
    d_rotation.setIdentity();
    d_twist.setZero();
  }

  /**
   * @brief get_position: return the position of the entity
   * @return d_position
   */
  inline Vec3<double> get_position() {
    return d_position;
  }

  /**
   * @brief get_rotation: return the rotation of the entity
   * @return d_rotation
   */
  inline RotMat<double> get_rotation() {
    return d_rotation;
  }

  /**
   * @brief get_twist: return the twist of the entity
   * @return d_twist
   */
  inline Twist<double> get_twist() {
    return d_twist;
  }

  /**
   * @brief set_twist: set twist of the entity
   * @param twist: the observed, or estimated twist of the entity
   */
  inline void set_twist(const Twist<double>& twist) {
    d_twist = twist;
  }

  /**
   * @brief set_position: set the position of the entity
   * @param position: the observed or estimated position of the entity by eigen vector
   */
  inline void set_position(const Vec3<double>& position) {
    d_position = position;
  }

  /**
   * @brief set_position: set the position of the entity
   * @param position: the observed or estimated position of the entity by kdl vector
   */
  inline void set_position(const KDL::Vector& position) {
    d_position.x() = position.x();
    d_position.y() = position.y();
    d_position.z() = position.z();
  }

  /**
   * @brief set_rotation: set the orientation of the entity
   * @param rotation: the observed or estimated orientation of the entity expressed in rotation matrix by eigen
   */
  inline void set_rotation(const RotMat<double>& rotation) {
    d_rotation = rotation;
  }

  /**
   * @brief set_rotation: set the orientation of the entity
   * @param rotation: the observed or estimated orientation of the entity expressed in rotation matrix by kdl
   */
  inline void set_rotation(const KDL::Rotation rotation) {
    d_rotation(0, 0) = rotation.data[0];
    d_rotation(0, 1) = rotation.data[1];
    d_rotation(0, 2) = rotation.data[2];
    d_rotation(1, 0) = rotation.data[3];
    d_rotation(1, 1) = rotation.data[4];
    d_rotation(1, 2) = rotation.data[5];
    d_rotation(2, 0) = rotation.data[6];
    d_rotation(2, 1) = rotation.data[7];
    d_rotation(2, 3) = rotation.data[8];
  }

private:

  /**
   * @brief the position of the entity
   */
  Vec3<double> d_position;

  /**
   * @brief the orientation of the entity expressed by rotation matrix
   */
  RotMat<double> d_rotation;

  /**
   * @brief the twist of the specified entity, including linear velocity and
   */
  Twist<double> d_twist;

};

}

#endif // PR_ROBOT_H
