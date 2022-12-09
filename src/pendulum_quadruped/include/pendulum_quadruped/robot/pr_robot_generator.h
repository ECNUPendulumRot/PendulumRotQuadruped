#ifndef PR_ROBOT_GENERATOR_H
#define PR_ROBOT_GENERATOR_H

#include <memory>

#include <ros/ros.h>

#include "robot/pr_robot_model.h"
#include "robot/pr_robot_description.h"
#include "robot/pr_description_loader.h"
#include "robot/pr_robot_factory.h"

namespace pr_robot {

template <typename T, typename U, typename... M>
std::tuple<std::shared_ptr<T>, std::shared_ptr<U>, std::shared_ptr<M>...> create_robot(RobotModel& model,
                                                                   RobotDescriptionArray& array,
                                                                   int cur_rec);

template <typename T>
std::tuple<std::shared_ptr<T>> create_robot(RobotModel& model,
                                            RobotDescriptionArray& array,
                                            int cur_rec);

template <typename... M>
class RobotGenerator {

public:

  /**
   * @brief RobotGenerator: contructor of robot generator.
   * @param yaml_path: description file path
   * @param urdf_path: URDF file path
   */
  RobotGenerator(const std::string& yaml_path, const std::string& urdf_path);

  /**
   * @brief RobotGenerator: contructor of robot generator.
   * @param yaml_path: description file path
   * @param param_name: name of URDF param in ROS param server
   * @param nh: node handle of current node
   */
  RobotGenerator(const std::string& yaml_path, const std::string& param_name, ros::NodeHandle& nh);


  std::tuple<std::shared_ptr<M>...> get(){
      return create_robot<M...>(d_robot_model, *d_description_p, 0);
  }


private:

  /**
   * @brief d_robot_model: stores URDF model and KDL tree
   */
  RobotModel d_robot_model;

  /**
   * @brief d_descriptions: stores endeffector information
   */
  RobotDescriptionArrayPtr d_description_p;
};

template <typename... M>
RobotGenerator<M...>::RobotGenerator(const std::string &yaml_path, const std::string &urdf_path)
{
  d_description_p = DescriptionLoader::load_from_yaml(yaml_path);
  d_robot_model.load_from_xml(urdf_path);
}

template <typename... M>
RobotGenerator<M...>::RobotGenerator(const std::string &yaml_path, const std::string &param_name, ros::NodeHandle &nh)
{
  d_description_p = DescriptionLoader::load_from_yaml(yaml_path);
  std::string urdf_str;
  nh.getParam(param_name, urdf_str);
  d_robot_model.load_from_string(urdf_str);
}

//template <typename T, typename... M>
//std::tuple<std::shared_ptr<T>, std::shared_ptr<M>...> create_robot(RobotModel& model,
//                                                                   RobotDescriptionArray& array,
//                                                                   int cur_rec) {
//  /// ensure the index is valid
//  assert(cur_rec < array.size());

//  return std::tuple_cat(
//        std::tuple<T>(
//          RobotFactory<T>(model, array[cur_rec])),
//          create_robot<M...>(model, array, cur_rec + 1));
//}

template <typename T, typename U, typename... N>
std::tuple<std::shared_ptr<T>, std::shared_ptr<U>, std::shared_ptr<N>...> create_robot(RobotModel& model,
                                                                                       RobotDescriptionArray& array,
                                                                                       int cur_rec) {

  return std::tuple_cat(RobotFactory<T>(model, array[cur_rec]),
                        create_robot<U, N...>(model, array, cur_rec + 1));
}

template <typename T>
std::tuple<std::shared_ptr<T>> create_robot(RobotModel& model,
                                            RobotDescriptionArray& array,
                                            int cur_rec) {
  return std::tuple<std::shared_ptr<T>>(RobotFactory<T>()(model, array[cur_rec]));
}

}// namespace pr_robot

#endif // PR_ROBOT_GENERATOR_H
