#include "pr_gazebo/gazebo_model_spawn.h"

const std::vector<std::string> pr_gazebo::GazeboSpawner::controller_list = {
  "joint_state_controller",
  "FR_hip_controller", "FR_thigh_controller", "FR_calf_controller",
  "FL_hip_controller", "FL_thigh_controller", "FL_calf_controller",
  "RR_hip_controller", "RR_thigh_controller", "RR_calf_controller",
  "RL_hip_controller", "RL_thigh_controller", "RL_calf_controller"
  };

geometry_msgs::Pose pr_gazebo::GazeboSpawner::model_pose = geometry_msgs::Pose();


bool pr_gazebo::GazeboSpawner::spawn_model(std::string urdf_param)
{

  ros::ServiceClient model_service = d_nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

  std::string urdf_raw;

  d_nh.getParam(urdf_param, urdf_raw);

  gazebo_msgs::SpawnModel msg;

  msg.request.model_name = d_robot_type + "." + d_robot_name;

  msg.request.model_xml = urdf_raw;
  msg.request.robot_namespace = "/" + d_robot_type + "/" + d_robot_name;
  msg.request.reference_frame = "world";
  msg.request.initial_pose = model_pose;

  if(!model_service.call(msg)){
    return false;
  }

  while(!msg.response.success){
    //ROS_WARN("Loading model...");
    //usleep(10000);
  }

  sleep(2);

  ROS_INFO("Gazebo model spawn correctly");

  return true;
}


bool pr_gazebo::GazeboSpawner::delete_model()
{
  // int state = system(std::string("rosservice call gazebo/delete_model '{model_name: " + model_name + "_gazebo}'").c_str());
  // ROS_INFO("delete model state: %d", state);

  std::string service_name = "/gazebo/delete_model";
  ros::ServiceClient model_service = d_nh.serviceClient<gazebo_msgs::DeleteModel>(service_name);

  gazebo_msgs::DeleteModel msg;

  msg.request.model_name = d_robot_type + "." + d_robot_name;

  if(!model_service.call(msg)){
    return false;
  }

  while(!msg.response.success){
    //ROS_WARN("Deleting gazebo model...");
  }

  ROS_INFO("Gazebo model delete correctly");

  return true;
}


void pr_gazebo::GazeboSpawner::start_controllers()
{
  std::string service_name =  "/" + d_robot_type + "/" + d_robot_name + "/controller_manager/switch_controller";
  ros::ServiceClient controller_service = d_nh.serviceClient<controller_manager_msgs::SwitchController>(service_name);

  controller_manager_msgs::SwitchController msg;
  msg.request.start_controllers = controller_list;
  msg.request.strictness = msg.request.STRICT;

  if(controller_service.call(msg)){
    ROS_INFO("success to call start controllers service");
  }
  else{
    ROS_ERROR("fail to call start controllers service");
  }

  while(!msg.response.ok){
    //ROS_ERROR("Starting controllers...");
  }
  ROS_INFO("Controllers start correctly");
}


bool pr_gazebo::GazeboSpawner::stop_controllers()
{
  std::string service_name = "/" + d_robot_type + "/" + d_robot_name + "/controller_manager/switch_controller";
  ros::ServiceClient controller_service = d_nh.serviceClient<controller_manager_msgs::SwitchController>(service_name);

  controller_manager_msgs::SwitchController msg;
  msg.request.stop_controllers = controller_list;
  msg.request.strictness = msg.request.STRICT;

  if(!controller_service.call(msg)){
    return false;
  }

  while(!msg.response.ok){
    //ROS_WARN("Stopping controllers...");
  }
  ROS_INFO("Controllers stopped correctly!");
  return true;
}


void pr_gazebo::GazeboSpawner::load_controllers()
{
  for(auto& element: controller_list){
    load_controller_once(element);
  }
}


void pr_gazebo::GazeboSpawner::unload_controllers()
{
  for(auto& element: controller_list){
    unload_controller_once(element);
  }
}


void pr_gazebo::GazeboSpawner::load_controller_once(std::string controller_name)
{
  std::string service_name = "/" + d_robot_type + "/" + d_robot_name + "/controller_manager/load_controller";
  ros::ServiceClient controller_service = d_nh.serviceClient<controller_manager_msgs::LoadController>(service_name);

  controller_manager_msgs::LoadController msg;
  msg.request.name = controller_name;

  while(!controller_service.call(msg)){
    ROS_WARN("Trying to call load_controller service");
    usleep(50000);
  }

  ROS_INFO("success to call load_controller service");

  while(!msg.response.ok){
  }
  ROS_INFO("%s load correctly", controller_name.c_str());
}


void pr_gazebo::GazeboSpawner::unload_controller_once(std::string controller_name)
{
  std::string service_name = "/" + d_robot_type + "/" + d_robot_name + "/controller_manager/unload_controller";
  ros::ServiceClient controller_service = d_nh.serviceClient<controller_manager_msgs::UnloadController>(service_name);

  controller_manager_msgs::UnloadController msg;
  msg.request.name = controller_name;

  if(controller_service.call(msg)){
    ROS_INFO("success to call unload_controller service");
  }
  else{
    ROS_ERROR("fail to call unload_controller service");
  }

  while(!msg.response.ok){
  }

  ROS_INFO("%s unload correctly", controller_name.c_str());
}


void pr_gazebo::GazeboSpawner::set_model_position(double x, double y, double z)
{
  model_pose.position.x = x;
  model_pose.position.y = y;
  model_pose.position.z = z;
}


void pr_gazebo::GazeboSpawner::set_model_orientation(double w, double x, double y, double z)
{
  model_pose.orientation.w = w;
  model_pose.orientation.x = x;
  model_pose.orientation.y = y;
  model_pose.orientation.z = z;
}
