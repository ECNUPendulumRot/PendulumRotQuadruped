#pragma once

#include <memory>

#include <ros/ros.h>
#include <ros/package.h>

#include "robot/pr_robot_model.h"
#include "robot/pr_robot_description.h"
#include "robot/pr_description_loader.h"
#include "robot/pr_robot_types.h"

enum {USE_URDF, USE_STRING};

namespace pr_robot {

template <typename... M>
class RobotGenerator {

public:

  /**
   * @brief RobotGenerator: contructor of robot generator.
   * @param type: create robot with URDF or XACRO generated string in param
   * @param nh: ros nodehandle for fetching URDF and param string
   */
  RobotGenerator(unsigned type, ros::NodeHandle& nh);

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
RobotGenerator<M...>::RobotGenerator(unsigned type, ros::NodeHandle& nh) {
  std::string path_to_package = ros::package::getPath("a1_description");
  std::string path_to_description = path_to_package + "/config/robot_description.yaml";

  d_description_array_p = DescriptionLoader::load_from_yaml(path_to_description);

  if (type == USE_URDF) {
    std::string path_to_urdf = path_to_package + "/urdf/a1.urdf";
    d_robot_model.load_from_xml(path_to_urdf);
  } else if (type == USE_STRING) {
    std::string urdf_string;
    nh.getParam("robot_description", urdf_string);
    d_robot_model.load_from_string(urdf_string);
  } else {
    printf(" Generator failed to initialize with type %d", type);
    ros::shutdown();
  }

  d_robot_tuple = create_robot<M...>(nh.getNamespace(), d_robot_model, *d_description_array_p, 0);
}

}// namespace pr_robot

