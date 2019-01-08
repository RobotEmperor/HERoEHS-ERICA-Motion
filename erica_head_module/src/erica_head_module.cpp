/*
 * erica_head_module.cpp
 *
 *  Created on: Nov 1, 2018
 *      Author: heroehs
 */

#include <stdio.h>
#include "erica_head_module/erica_head_module.h"
using namespace erica;

HeadModule::HeadModule()
: control_cycle_msec_(8)
{
  running_ = false;
  enable_       = false;
  gazebo_check  = false;
  module_name_  = "head_module";
  control_mode_ = robotis_framework::PositionControl;


  // Dynamixel initialize ////
  result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 13
  result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 14
  result_["head_roll"]        = new robotis_framework::DynamixelState();  // joint 15
  //init
  new_count_ = 1;
  is_moving_state = false;

  joint_name_to_id_.clear();
  joint_id_to_name_.clear();
  joint_id_to_rad_.clear();
}
HeadModule::~HeadModule()
{
  queue_thread_.join();
}
void HeadModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&HeadModule::queueThread, this));

  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
      it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    joint_id_to_name_[dxl_info->id_] = joint_name;
    joint_id_to_rad_[dxl_info->id_] = 0;
  }
  ROS_INFO("< -------  Initialize Module : Head Module !!  ------->");
}
void HeadModule::headcmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실
{
  joint_id_to_rad_[13] = msg->data[0];
  joint_id_to_rad_[14] = msg->data[1];
  joint_id_to_rad_[15] = msg->data[2];
  ROS_INFO("receive rad");

  ROS_INFO("ID 13 Value : %f \n", joint_id_to_rad_[13]);
  ROS_INFO("ID 13 Value : %f \n", joint_id_to_rad_[14]);
  ROS_INFO("ID 13 Value : %f \n", joint_id_to_rad_[15]);
}
void HeadModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);
  /* subscribe topics */
  // for gui
  ros::Subscriber head_cmd_sub = ros_node.subscribe("/heroehs/head_cmd", 5, &HeadModule::headcmdCallback, this);
  ros::WallDuration duration(control_cycle_msec_ / 1000.0);

  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}
bool HeadModule::isRunning()
{
  return running_;
}
void HeadModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
    std::map<std::string, double> sensors)
{
  if (enable_ == false)
  {
    return;
  }
    //ROS_INFO("Base Trajectory Start");
    for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
        state_iter != dxls.end(); state_iter++)
    {
      std::string joint_name = state_iter->first;
      result_[joint_name]->goal_position_ =  joint_id_to_rad_[joint_name_to_id_[joint_name]];
    } // 등록된 다이나믹셀  goal position 으로 입력
}
void HeadModule::stop()
{
  return;
}



