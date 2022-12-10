

#include "pr_joint_controller.h"
#include <pluginlib/class_list_macros.h>

namespace {

double get_current_velocity(double current_position, double last_position, double last_velocity, double duration){
  double ratio = 0.65;
  double current_velocity = (current_position - last_position) / duration;
  return ratio * current_velocity + (1 - ratio) * last_velocity;
}

double get_desired_torque(pr_cartesian_controller::pr_servo_cmd& servo_cmd, double current_position, double current_velocity){
  return servo_cmd.kp * (servo_cmd.pos - current_position) + servo_cmd.kd * (servo_cmd.vel - current_velocity) + servo_cmd.tau;
}

}


pr_cartesian_controller::JointController::JointController()
{
  memset(&d_last_state, 0, sizeof(pr_joint_msgs::JointState));
  memset(&d_servo_cmd, 0, sizeof(pr_servo_cmd));
}


pr_cartesian_controller::JointController::~JointController(){
}


// Controller initialization in non-realtime
bool pr_cartesian_controller::JointController::init(hardware_interface::EffortJointInterface* joint, ros::NodeHandle& nh)
{

  std::string name_space = nh.getNamespace();
  std::string joint_name;

  if(!nh.getParam("joint", joint_name)){
    ROS_ERROR("Joint %s not exist in namespace '%s'", joint_name.c_str(), name_space.c_str());
    return false;
  }

  urdf::Model urdf;
  if(!urdf.initParamWithNodeHandle("robot_description", nh)){
    ROS_ERROR("Failed to parse model's urdf file");
    return false;
  }

  d_joint_urdf_p = urdf.getJoint(joint_name);
  if(!d_joint_urdf_p){
    ROS_ERROR("Joint %s not find in urdf", joint_name.c_str());
    return false;
  }

  d_joint_handle = joint->getHandle(joint_name);

  d_cmd_sub = nh.subscribe(name_space + "/command", 20, &JointController::subscribe_command, this);

  d_state_publisher = new realtime_tools::RealtimePublisher<pr_joint_msgs::JointState>(nh, name_space + "/state", 1);

  return true;
}


// Controller startup in realtime
void pr_cartesian_controller::JointController::starting(const ros::Time& time)
{
  d_last_state.pos = d_joint_handle.getPosition();
  d_last_state.vel = 0;

  d_command_buffer.initRT(d_servo_cmd);
}


// Controller update loop in realtime
void pr_cartesian_controller::JointController::update(const ros::Time& time, const ros::Duration& period)
{

  trim_command();

  double current_position, current_velocity, desired_tau;

  current_position = d_joint_handle.getPosition();

  current_velocity = ::get_current_velocity(current_position, d_last_state.pos, d_last_state.vel, period.toSec());
  desired_tau = ::get_desired_torque(d_servo_cmd, current_position, current_velocity);

  limit_torque(desired_tau);

  d_joint_handle.setCommand(desired_tau);

  d_last_state.pos = current_position;
  d_last_state.vel = current_velocity;

  if(d_state_publisher->trylock()){
    d_state_publisher->msg_.pos = d_last_state.pos;
    d_state_publisher->msg_.vel = d_last_state.vel;
    d_state_publisher->unlockAndPublish();
  }
}


// Controller stopping in realtime
void pr_cartesian_controller::JointController::stopping(const ros::Time& time){
}


void pr_cartesian_controller::JointController::trim_command()
{
  d_servo_cmd = *(d_command_buffer.readFromRT());
  if(d_servo_cmd.mode == PMSM){

    limit_position(d_servo_cmd.pos);
    if(fabs(d_servo_cmd.pos - PosStopF) < 0.00001){
      d_servo_cmd.kp = 0;
    }

    limit_velocity(d_servo_cmd.pos);
    if(fabs(d_servo_cmd.vel - VelStopF) < 0.00001){
      d_servo_cmd.kd = 0;
    }

    limit_torque(d_servo_cmd.tau);
  }
  if(d_servo_cmd.mode == BRAKE){
    d_servo_cmd.kp = 0;
    d_servo_cmd.vel = 0;
    d_servo_cmd.kd = 20;
    d_servo_cmd.tau = 0;
  }
}


void pr_cartesian_controller::JointController::subscribe_command(const pr_joint_msgs::JointCmd &msg)
{
  d_servo_cmd.mode = msg.mode;
  d_servo_cmd.pos  = msg.q;
  d_servo_cmd.kp   = msg.kp;
  d_servo_cmd.vel  = msg.dq;
  d_servo_cmd.kd   = msg.kd;
  d_servo_cmd.tau  = msg.tau;
  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  d_command_buffer.writeFromNonRT(d_servo_cmd);
}

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(pr_cartesian_controller::JointController, controller_interface::ControllerBase);
