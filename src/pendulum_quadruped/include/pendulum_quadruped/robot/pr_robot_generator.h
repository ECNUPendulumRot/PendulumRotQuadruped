#pragma once

#include <memory>

#include <ros/ros.h>

#include "robot/pr_robot_model.h"
#include "robot/pr_robot_description.h"
#include "robot/pr_description_loader.h"
#include "robot/pr_robot_types.h"


namespace pr_robot {

template <typename... M>
class RobotGenerator {

public:

  /**
   * @brief RobotGenerator: contructor of robot generator.
   * @param ns: the namespace of the robot
   * @param yaml_path: description file path
   * @param urdf_path: URDF file path
   */
  RobotGenerator(const std::string& yaml_path, const std::string& urdf_path);

  /**
   * @brief get: return the entry of the robot ptr tuple
   */
  template<std::size_t I>
  auto get() {
    return std::get<I>(d_robot_tuple);
  }

private:

  /**
   * @brief d_robot_model: stores URDF model and KDL tree
   */
  RobotModel d_robot_model;

  /**
   * @brief d_descriptions: stores endeffector information
   */
  RobotDescriptionArrayPtr d_description_array_p;

  /**
   * @brief d_robots_p: tuple of robot pointers
   */
  std::tuple<std::shared_ptr<M>...> d_robot_tuple;

};


template <typename T, typename U, typename... N>
std::tuple<std::shared_ptr<T>, std::shared_ptr<U>, std::shared_ptr<N>...> create_robot(const std::string &ns,
                                                                                       RobotModel& model,
                                                                                       RobotDescriptionArray& array,
                                                                                       int cur_rec) {
  return std::tuple_cat(T::create(ns, model, array[cur_rec]),
                        create_robot<U, N...>(ns, model, array, cur_rec + 1));
}


template <typename T>
std::tuple<std::shared_ptr<T>> create_robot(const std::string &ns,
                                            RobotModel& model,
                                            RobotDescriptionArray& array,
                                            int cur_rec) {
  return std::tuple<std::shared_ptr<T>>(T::create(ns, model, array[cur_rec]));
}


template <typename... M>
RobotGenerator<M...>::RobotGenerator(const std::string &yaml_path,
                                     const std::string &urdf_path) {
  ros::NodeHandle nh;
  d_description_array_p = DescriptionLoader::load_from_yaml(yaml_path);
  d_robot_model.load_from_xml(urdf_path);
  d_robot_tuple = create_robot<M...>(nh.getNamespace(), d_robot_model, *d_description_array_p, 0);
}



}// namespace pr_robot

