#ifndef PR_EIGEN_TYPES_H
#define PR_EIGEN_TYPES_H

#include <Eigen/Dense>

/**
 * @brief Vec3: the 3 element vector typedef from eigen.
 * usually used for position, angle axis, velocity and general uses
 */
template<typename Scale>
using Vec3 = Eigen::Matrix<Scale, 3, 1>;

/**
 * @brief Vec6: the 6 element vector typedef from eigen.
 * usually used for twist and pose expressed orientation in angle axis
 */
template<typename Scale>
using Vec6 = Eigen::Matrix<Scale, 6, 1>;

/**
 * @brief Mat3: the 3 by 3 matrix typedef from eigen.
 * usually used for rotation and skew symmetric matrix
 */
template<typename Scale>
using Mat3 = Eigen::Matrix<Scale, 3, 3>;

/**
 * @brief RotMat: the rotation matrix define from eigen. When using this typedef , ensure this is in SO(3)
 */
template<typename Scale>
using RotMat = Mat3<Scale>;

template<typename Scale>
using DynamicVec = Eigen::Matrix<Scale, Eigen::Dynamic, 1>;

template<typename Scale>
using DynamicMat = Eigen::Matrix<Scale, Eigen::Dynamic, Eigen::Dynamic>;


#endif // PR_EIGEN_TYPES_H
