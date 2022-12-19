#pragma once

#include "robot/pr_robot_types.h"

namespace pr_state_updater {


template<class R>
class JointStateObserver {

public:

  JointStateObserver() {
    ros::NodeHandle nh;
  }

private:

  /**
   * @typedef StateStore: an array to store the array of joint states
   * with array size of endeffector number and element is matrix of <joint in endeffector chain> * <angle, velocity>
   */
  using StateStore = std::array<Eigen::Matrix<double, R::get_state_info_pair().second, 2>, R::get_state_info_pair().first>;

  StateStore d_state_store;
};

}// namespace pr_state_updater
