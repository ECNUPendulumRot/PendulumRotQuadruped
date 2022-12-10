#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>

void pose_callback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
  static tf::TransformBroadcaster bc;
  tf::Transform transform;
  // usleep(5000);
  int count = msg->name.size();
  for(int i = 1; i < count; i++){
    transform.setOrigin(tf::Vector3(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z));
    tf::Quaternion q(msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z, msg->pose[i].orientation.w);
    transform.setRotation(q);
    bc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", msg->name[i] + "/base"));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_tf_broadcaster");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1, &pose_callback);
  ros::spin();
  return 0;
}