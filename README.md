# Pendulum Rot Quadruped

## ROS Topic Documentation

For every sim or real robot with **robot_type**  and **robot_name**, it will create a ROS name space of "robot_type/robot_name". We recommand you to create a robot in gazebo with a suffix of "_sim" to differ from real robot

Under the local robot namespace, there exists several namespaces:

- state estimator: robot_type/robot_name/pose, robot_type/robot_name/twist for odometry
- joint state: robot_type/robot_name/joint_states
	for quadrupeds, the order of joints in config yaml should be FR, FL, RR, RL