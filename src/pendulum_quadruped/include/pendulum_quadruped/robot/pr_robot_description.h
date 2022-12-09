#ifndef PR_ROBOT_DESCRIPTION_H
#define PR_ROBOT_DESCRIPTION_H

#include <string>
#include <map>
#include <vector>
#include <ostream>

namespace pr_robot {

/**
 * @brief the class to desrcibe robot, solving the conflict between
 * different brand of quadrupeds
 */
using EndeffectorDescription = std::map<std::string, std::string>;

class RobotDescription {

  friend std::ostream& operator<<(std::ostream &os, pr_robot::RobotDescription &d);

public:

  /**
   * @brief get the point of endeffector description array
   */
  inline EndeffectorDescription& get_endeffector_description() {
    return d_endeffector_description;
  }

  /**
   * @brief get endeffector description by key name
   * @param key_name
   * @return
   */
  inline std::string get_endeffector_description(const std::string &key_name) {
    return d_endeffector_description[key_name];
  }

  /**
   * @brief add endeffector description by key-value
   * @param key
   * @param value
   */
  inline void set_endeffector_description(std::string key, std::string value) {
    d_endeffector_description[key] = value;
  }


  /**
   * @brief get the robot type #QUADRUPED 0, BIPED 1, ARM 2, MIXED 3
   * @return
   */
  int get_type() {
    return d_type;
  }

  /**
   * @brief get name of the robot
   */
  std::string get_name() {
    return d_name;
  }

  /**
   * @brief set the robot type
   */
  void set_type(int value) {
    d_type = value;
  }

  /**
   * @brief get name of the robot
   */
  void set_name(const std::string &value) {
    d_name = value;
  }

private:

  /**
   * @brief the type of the robot , see the enum in pubilc
   */
  unsigned int d_type;

  /**
   * @brief the name of the robot
   */
  std::string d_name;

  /**
   * @brief the end effector name in urdf correspond to robot,
   * for example, if the robot is quadruped, the front right foot
   * name can be "FR_foot"
   */
  EndeffectorDescription d_endeffector_description;

};


class RobotDescriptionArray {

public:

  /**
   * @brief add a description into array
   */
  void add_description(RobotDescription d){
    printf("adding!\n");
    d_robot_description_array.push_back(d);
    printf("added!\n");
  }

  /**
   * @overload operator[]: return entry in d_robot_description_array
   */
  RobotDescription& operator[](unsigned int i) {
    return d_robot_description_array[i];
  }

  /**
   * @brief get the description array
   * @return d_robot_description_array
   */
  std::vector<RobotDescription>& get_robot_description_array(){
    return d_robot_description_array;
  }

  /**
   * @brief size: return the size of d_robot_description_array
   * @return size of d_robot_description_array
   */
  unsigned int size() {
    return d_robot_description_array.size();
  }

private:
  /**
   * @brief robot description array
   */
  std::vector<RobotDescription> d_robot_description_array;

};

/**
 * @brief retun the message of a description
 * @param os
 * @param d
 * @return
 */
std::ostream& operator<<(std::ostream &os, pr_robot::RobotDescription &d);

}// namespace pr_robot


#endif // PR_ROBOT_DESCRIPTION_H
