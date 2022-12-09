#ifndef PR_ROBOT_FACTORY_H
#define PR_ROBOT_FACTORY_H

#include <memory>
#include <type_traits>

#include "robot/pr_robot_model.h"
#include "robot/pr_robot_types.h"
#include "robot/pr_robot_base.h"
#include "robot/pr_robot_description.h"

namespace pr_robot {

/**
 * @typedef RobotPtr: std::shared_ptr<Robot>
 */
using RobotPtr = std::shared_ptr<Robot>;

/**
 * @typedef QuadrupedPtr: std::shared_ptr<Quadruped>
 */
using QuadrupedPtr = std::shared_ptr<Quadruped>;

template <typename T>
class RobotFactory {

public:

  /**
   * @brief create: function to fill a base model
   * @param robot: robot to fill information
   * @param model: total model of the robot
   * @param description: the description structure describe the robot
   *
   * @warning must ensure that in URDF, the first link of robot(usually the floating base) is named as base
   */
  void setup_robot(pr_robot::RobotPtr robot, pr_robot::RobotModel &model, pr_robot::RobotDescription &description)
  {

    EndeffectorDescription& d = description.get_endeffector_description();

    for(auto it = d.begin(); it != d.end(); it++){
      KDL::Chain endeffector_chain;
      model.get_kdl_tree().getChain("base", it->second, endeffector_chain);
      robot->add_endeffector_chain(it->first, endeffector_chain);
    }
  }

  std::shared_ptr<T> operator()(pr_robot::RobotModel &model, pr_robot::RobotDescription &description){
    check_type();
    if constexpr (std::is_same<T, Quadruped>::value)
      return create_quadruped(model, description);
  }

  void check_type(){
    constexpr bool b = (std::is_same<T, Quadruped>::value) ;
    static_assert(b, "Robot type does not exists!");
  }

  /**
   * @brief create_quadruped: create a quadruped
   * @param model: total model of the robot
   * @param description: the description structure describe the robot
   * @return quad shared pointer
   */
  QuadrupedPtr create_quadruped(pr_robot::RobotModel &model, pr_robot::RobotDescription &description)
  {
    QuadrupedPtr quad(new Quadruped(description.get_type(), description.get_name()));

    setup_robot(std::static_pointer_cast<Robot>(quad), model, description);

    EndeffectorDescription& endeffector_descript = description.get_endeffector_description();

    for(auto it = endeffector_descript.begin(); it != endeffector_descript.end(); it++){
      if(!Quadruped::valid_leg_tag(it->first)){
        printf("YAML description file is not correct. Please check.");
        exit(1);
      }
      quad->add_leg(it->first, it->second);
    }
    return quad;
  }

};

}// namespace pr_robot

#endif // PR_ROBOT_FACTORY_H
