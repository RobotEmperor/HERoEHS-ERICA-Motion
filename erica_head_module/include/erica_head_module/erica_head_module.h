/*
 * erica_head_module.h
 *
 *  Created on: Nov 1, 2018
 *      Author: heroehs
 */

#ifndef ERICA_HEROEHS_ERICA_MOTION_ERICA_HEAD_MODULE_INCLUDE_ERICA_HEAD_MODULE_ERICA_HEAD_MODULE_H_
#define ERICA_HEROEHS_ERICA_MOTION_ERICA_HEAD_MODULE_INCLUDE_ERICA_HEAD_MODULE_ERICA_HEAD_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"

namespace erica
{


class HeadModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<HeadModule>
{
public:
  HeadModule();
  virtual ~HeadModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  bool gazebo_check;
  bool is_moving_state;

  /* ROS Topic Callback Functions */

  void headcmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);


private:
  void queueThread();
  bool running_;

  int new_count_;
  int control_cycle_msec_;

  boost::thread queue_thread_;

  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;

  std::map<int, double> joint_id_to_rad_;
};
}




#endif /* ERICA_HEROEHS_ERICA_MOTION_ERICA_HEAD_MODULE_INCLUDE_ERICA_HEAD_MODULE_ERICA_HEAD_MODULE_H_ */
