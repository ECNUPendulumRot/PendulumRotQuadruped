/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#ifndef PR_JOINT_CONTROLLER_H
#define PR_JOINT_CONTROLLER_H

#include <urdf/model.h>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include "pr_joint_msgs/Joint.h"

#define PMSM    (0x0A)
#define BRAKE   (0x00)
#define PosStopF  (2.146E+9)
#define VelStopF  (16000.0)


namespace pr_cartesian_controller {

struct pr_servo_cmd
{
  uint8_t mode;
  double kp;
  double pos;
  double kd;
  double vel;
  double tau;
};

class JointController: public controller_interface::Controller<hardware_interface::EffortJointInterface> {

public:

  JointController();

  ~JointController();

  virtual bool init(hardware_interface::EffortJointInterface* joint, ros::NodeHandle& nh);

  virtual void starting(const ros::Time& time);

  virtual void update(const ros::Time& time, const ros::Duration& period);

  virtual void stopping(const ros::Time& time);

private:

  void publish_state();

  void trim_command();

  void subscribe_command(const pr_joint_msgs::JointCmd& msg);

  template<typename T>
  void limit_position(T& position);

  template<typename T>
  void limit_velocity(T& velocity);

  template<typename T>
  void limit_torque(T& effort);

  template<typename T>
  void clamp(T& value, double lower, double upper);

  urdf::JointConstSharedPtr d_joint_urdf_p;

  realtime_tools::RealtimeBuffer<pr_servo_cmd> d_command_buffer;

  pr_joint_msgs::JointState d_last_state;

  pr_servo_cmd d_servo_cmd;

  hardware_interface::JointHandle d_joint_handle;

  realtime_tools::RealtimePublisher<pr_joint_msgs::JointState>* d_state_publisher;

  ros::Subscriber d_cmd_sub;

  // realtime_tools::RealtimePublisher<pr_joint_msgs::Joint>* state_publisher_debug;
  //ros::Subscriber command_subscriber_a1;
  // ros::Publisher pub_state;
  //realtime_tools::RealtimePublisher<unitree_legged_msgs::MotorState>* state_publisher_a1;
};


template<typename T>
void JointController::limit_position(T& position)
{
  if(d_joint_urdf_p->type == urdf::Joint::REVOLUTE || d_joint_urdf_p->type == urdf::Joint::PRISMATIC)
    clamp(position, d_joint_urdf_p->limits->lower, d_joint_urdf_p->limits->upper);
}

template<typename T>
void JointController::limit_velocity(T& velocity)
{
  if(d_joint_urdf_p->type == urdf::Joint::REVOLUTE || d_joint_urdf_p->type == urdf::Joint::PRISMATIC)
    clamp(velocity, -d_joint_urdf_p->limits->velocity, d_joint_urdf_p->limits->velocity);
}

template<typename T>
void JointController::limit_torque(T& effort)
{
  if(d_joint_urdf_p->type == urdf::Joint::REVOLUTE || d_joint_urdf_p->type == urdf::Joint::PRISMATIC)
    clamp(effort, -d_joint_urdf_p->limits->effort, d_joint_urdf_p->limits->effort);
}

template<typename T>
void JointController::clamp(T& value, double lower, double upper)
  {
    double temp = value;
    value = std::max(std::min(temp, upper), lower);
  }

}//namespace pr_cartesian_controller
#endif // PR_JOINT_CONTROLLER_H
