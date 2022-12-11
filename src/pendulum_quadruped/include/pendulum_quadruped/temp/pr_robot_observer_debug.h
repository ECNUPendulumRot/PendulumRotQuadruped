/**
 * @file this file will be deleted after the structure has been determined
 */

#ifndef PR_ROBOT_OBSERVER_DEBUG_H
#define PR_ROBOT_OBSERVER_DEBUG_H

#include "robot/pr_robot_types.h"

#include <ros/ros.h>

namespace pr_observer {

class QuadObserverDebug {

public:

  QuadObserverDebug(QuadrupedPtr quad_ptr, ros::NodeHandle nh){
    d_quadruped_p = quad_ptr;
    d_nh = nh;
  }

private:

  ros::NodeHandle d_nh;

  DynamicMat<double> d_fr_state;
  DynamicMat<double> d_fl_state;
  DynamicMat<double> d_rr_state;
  DynamicMat<double> d_rl_state;

  QuadrupedPtr d_quadruped_p;

};

}

#endif // PR_ROBOT_OBSERVER_DEBUG_H
