#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <memory>

#include <ros/ros.h>

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
 * @typedef JointChain: std::vector<Joint>
 */
using JointChain = std::vector<Joint>;

/**
 * @typedef KDLPositionSolverPtr: std::shared_ptr<KDL::ChainFkSolverPos_recursive>
 * the foward position solver for position
 */
using KDLPositionSolverPtr = std::shared_ptr<KDL::ChainFkSolverPos_recursive>;

}
