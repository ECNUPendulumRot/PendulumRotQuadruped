/**
 * @file this file will be deleted after the structure has been determined
 */

#ifndef PR_ROBOT_OBSERVER_DEBUG_H
#define PR_ROBOT_OBSERVER_DEBUG_H

#include "robot/pr_quadruped.h"

#include <ros/ros.h>

namespace pr_observer {

class QuadObserverDebug {

public:

  QuadObserverDebug(QuadrupedPtr quad_ptr, ros::NodeHandle nh){
    d_quadruped_p = quad_ptr;
    d_nh = nh;

    d_fr_state.resize(3, 2);
    d_fl_state.resize(3, 2);
    d_rr_state.resize(3, 2);
    d_rl_state.resize(3, 2);
  }

  void subscribe_default(){

  }


private:

  ros::NodeHandle d_nh;

  ros::Subscriber d_joint_subscribers[12];

  DynamicMat<double> d_fr_state;
  DynamicMat<double> d_fl_state;
  DynamicMat<double> d_rr_state;
  DynamicMat<double> d_rl_state;

  QuadrupedPtr d_quadruped_p;

};

}

#endif // PR_ROBOT_OBSERVER_DEBUG_H
