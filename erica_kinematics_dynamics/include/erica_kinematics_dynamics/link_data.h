/*
 * link_data.h
 *
 *  Created on: Oct 30, 2018
 *      Author: jay
 */

#ifndef ERICA_KINEMATICS_DYNAMICS_LINK_DATA_H_
#define ERICA_KINEMATICS_DYNAMICS_LINK_DATA_H_

#include "robotis_math/robotis_math.h"

namespace erica_alice
{

class LinkData
{
 public:
  LinkData();
  ~LinkData();

  std::string name_;

  int parent_;
  int sibling_;
  int child_;

  double mass_;

  Eigen::Vector3d relative_position_;
  Eigen::Vector3d joint_axis_;
  Eigen::Vector3d center_of_mass_;
  Eigen::Vector3d joint_center_of_mass_;
  Eigen::Matrix3d inertia_;

  double joint_limit_max_;
  double joint_limit_min_;

  double joint_angle_;
  double joint_velocity_;
  double joint_acceleration_;

  Eigen::Vector3d position_;
  Eigen::Matrix3d orientation_;
  Eigen::Matrix4d transformation_;
};

}



#endif /* ERICA_KINEMATICS_DYNAMICS_LINK_DATA_H_ */
