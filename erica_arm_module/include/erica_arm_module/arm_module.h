/*
 * erica_arm_module.h
 *
 *  Created on: Nov 1, 2018
 *      Author: jay
 */

#ifndef ERICA_ARM_MODULE_ARM_MODULE_H_
#define ERICA_ARM_MODULE_ARM_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <std_msgs/String.h>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "erica_arm_module_msgs/ArmCmd.h"

#include "erica_kinematics_dynamics/kinematics_dynamics.h"

namespace erica_alice
{

class ArmModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<ArmModule>
{
public:
  ArmModule();
  virtual ~ArmModule();

  void armCmdMsgCallback(const erica_arm_module_msgs::ArmCmd::ConstPtr& msg);

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  void publishStatusMsg(unsigned int type, std::string msg);

  /* Parameter */
  KinematicsDynamics erica_;

private:
  void queueThread();

  double          control_cycle_sec_;
  boost::thread   queue_thread_;

  boost::mutex    process_mutex_;

  std_msgs::String movement_done_msg_;
  ros::Publisher  status_msg_pub_;

  bool ik_send_;
  double ik_result_angle_[6];
};

}

#endif /* ERICA_ARM_MODULE_ARM_MODULE_H_ */
