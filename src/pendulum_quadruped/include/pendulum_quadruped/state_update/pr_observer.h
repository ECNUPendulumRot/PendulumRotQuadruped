#pragma once

#include "robot/pr_robot_types.h"

#include <sensor_msgs/JointState.h>

namespace pr_state_updater {


template<class R>
class JointStateObserver {

public:

  /**
   * @brief JointStateObserver
   * @param robot_p: the robot pointer to update
   */
  JointStateObserver(std::shared_ptr<R> robot_p, ros::NodeHandle &nh) {
    std::cout <<"[observer]" << nh.getNamespace() <<std::endl;
    d_robot_p = robot_p;
    d_joint_state_subscriber = nh.subscribe("joint_states", 1, &JointStateObserver::joint_state_callback, this);
  }

private:

//  /**
//   * @typedef StateStore: an array to store the array of joint states
//   * with array size of endeffector number and element is matrix of <joint in endeffector chain> * <angle, velocity>
//   */
//  using StateStore = std::array<Eigen::Matrix<double, R::get_state_info_pair().second, 2>, R::get_state_info_pair().first>;

//  /**
//   * @brief d_state_store: stores the callback message of joint statesf
//   */
//  StateStore d_state_store;

  /**
   * @brief d_robot_p: the robot shared pointer to update
   */
  std::shared_ptr<R> d_robot_p;

  /**
   * @brief d_joint_state_subscriber: instance of joint state subscriber
   */
  ros::Subscriber d_joint_state_subscriber;

  /**
   * @brief joint_state_callback: callback for joint state subscriber
   * @param msgs: the message that has received
   *
   * @attention the order is as same as the joint order in robot joint config YAML file
   */
  void joint_state_callback(const sensor_msgs::JointState &msgs) {
    unsigned chain_num = R::get_state_info_pair().first;
    unsigned joint_num_on_chain = R::get_state_info_pair().second;

    for(unsigned i = 0; i < chain_num; ++i)
      for(unsigned j = 0; j < joint_num_on_chain; ++j){
        d_robot_p->set_joint_state(i, j, msgs.position.at(joint_num_on_chain * i + j), msgs.velocity.at(joint_num_on_chain * i + j));
      }
  }

};

}// namespace pr_state_updater
