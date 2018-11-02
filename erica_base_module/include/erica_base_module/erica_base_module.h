/*
 * erica_base_module.h
 *
 *  Created on: Oct 31, 2018
 *      Author: heroehs
 */

#ifndef ERICA_HEROEHS_ERICA_MOTION_ERICA_BASE_MODULE_INCLUDE_ERICA_BASE_MODULE_H_
#define ERICA_HEROEHS_ERICA_MOTION_ERICA_BASE_MODULE_INCLUDE_ERICA_BASE_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"

#include "heroehs_math/fifth_order_trajectory_generate.h"
#include "heroehs_math/kinematics.h"

namespace erica
{


class BaseModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<BaseModule>
{
public:
  BaseModule();
  virtual ~BaseModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  bool gazebo_check;
  bool is_moving_state;

  /* ROS Topic Callback Functions */

  void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void go_to_init_pose(std::string data, std::string file_name);
  std::map<std::string, heroehs_math::FifthOrderTrajectory *> motion_trajectory;

private:
  void queueThread();
  void parse_init_pose_data_(const std::string &path);
  void parse_init_offset_pose_data_(const std::string &path, const std::string &data);
  void init_pose_trajectory_();
  bool running_;

  int new_count_;
  int check_count_;
  int control_cycle_msec_;

  boost::thread queue_thread_;

  std::map<std::string, bool> joint_name_to_check_;
  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;

  double mov_time_state;
  std::map<std::string, double> joint_name_to_ini_pose_state_;
  std::map<std::string, double> joint_name_to_ini_pose_goal_;
};


}






#endif /* ERICA_HEROEHS_ERICA_MOTION_ERICA_BASE_MODULE_INCLUDE_ERICA_BASE_MODULE_H_ */
