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
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>
#include <math.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"
#include "heroehs_math/heroehs_pid_controller_2.h"


#include "erica_perception_msgs/PeoplePositionArray.h"

#define DEG2RAD(x) (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x) (x * 57.2957795131) // *180/PI



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


  std::map<std::string, heroehs_math::PIDController *> dxl_pidcontroller;
  /* ROS Topic Callback Functions */

  void headctrlCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void headmanualCallback(const std_msgs::Bool::ConstPtr& msg);
  void headtrackingCallback(const std_msgs::Bool::ConstPtr& msg);
  void headtrackingctrlCallback(const erica_perception_msgs::PeoplePositionArray::ConstPtr& msg);
  double mapping_num(double x, double in_min, double in_max, double out_min, double out_max);


private:
  void queueThread();
  bool running_;

  int new_count_;
  int control_cycle_msec_;

  int max_limit_[3];


  int mode_;
  int max_boxsize_;
  int min_boxsize_;


  double p_px,p_py,p_bw,p_bh,p_bs,p_iw,p_ih;


  boost::thread queue_thread_;

  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;

  std::map<int, double> joint_id_to_rad_;

  std::map<std::string,double> joint_name_to_curr_pose_;
  std::map<std::string,double> joint_name_to_goal_pose_;

};
}




#endif /* ERICA_HEROEHS_ERICA_MOTION_ERICA_HEAD_MODULE_INCLUDE_ERICA_HEAD_MODULE_ERICA_HEAD_MODULE_H_ */
