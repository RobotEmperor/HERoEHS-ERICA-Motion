/*
 * kinematics_dynamics.h
 *
 *  Created on: Oct 30, 2018
 *      Author: jay
 */

#ifndef ERICA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_
#define ERICA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_

//#include <vector>
#include "kinematics_dynamics_define.h"
#include "link_data.h"

namespace erica_alice
{

class KinematicsDynamics
{

public:
  KinematicsDynamics();
  ~KinematicsDynamics();

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  Eigen::MatrixXd calcMassCenter(int joint_id);
  Eigen::MatrixXd calcCenterOfMass(Eigen::MatrixXd mc);

  void calcJointsCenterOfMass(int joint_id);

  void calcForwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::Vector3d tar_position, Eigen::Vector3d curr_position, Eigen::Matrix3d tar_orientation, Eigen::Matrix3d curr_orientation);

  bool calcInverseKinematics(int to, Eigen::Vector3d tar_position, Eigen::Matrix3d tar_orientation , int max_iter, double ik_err);
  bool calcInverseKinematics(int from, int to, Eigen::Vector3d tar_position, Eigen::Matrix3d tar_orientation, int max_iter, double ik_err);

  bool calcInverseKinematicsForArm(double *out, double x, double y, double z, Eigen::Matrix3d& tar_ori);
  bool calcInverseKinematicsForRightArm(double *out, double x, double y, double z, Eigen::Matrix3d& tar_ori);
  bool calcInverseKinematicsForLeftArm(double *out, double x, double y, double z, Eigen::Matrix3d& tar_ori);

  LinkData *erica_link_data_ [ ALL_JOINT_ID + 1 ];

};

}


#endif /* ERICA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_ */
