/*
 * arm_module.cpp
 *
 *  Created on: Nov 1, 2018
 *      Author: jay
 */

#include "erica_arm_module/arm_module.h"

using namespace erica_alice;

ArmModule::ArmModule()
{
  enable_       = false;
  module_name_  = "manipulation_module";
  control_mode_ = robotis_framework::PositionControl;

  /* arm */
  result_["l_arm_sh_p"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_y"]  = new robotis_framework::DynamixelState();
  result_["l_arm_el_p"]  = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"]  = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"]  = new robotis_framework::DynamixelState();

  result_["r_arm_sh_p"]  = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"]  = new robotis_framework::DynamixelState();
  result_["r_arm_sh_y"]  = new robotis_framework::DynamixelState();
  result_["r_arm_el_p"]  = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"]  = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"]  = new robotis_framework::DynamixelState();

  control_cycle_sec_ = 0.008;

  ik_result_angle_[0] = ik_result_angle_[1] = ik_result_angle_[2] = ik_result_angle_[3] = ik_result_angle_[4] = ik_result_angle_[5] = 0;

  ik_send_ = false;
}

ArmModule::~ArmModule()
{
  queue_thread_.join();
}

void ArmModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_      = boost::thread(boost::bind(&ArmModule::queueThread, this));

  erica_.calcForwardKinematics(0);
  ik_send_ = false;
}

void ArmModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_     = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/heroehs/status", 1);

  /* subscribe topics */
  ros::Subscriber arm_cmd_msg_sub = ros_node.subscribe("/heroehs/arm/displacement", 5,
                                                               &ArmModule::armCmdMsgCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void ArmModule::armCmdMsgCallback(const erica_arm_module_msgs::ArmCmd::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->name == "left")
  {
    double tar_x = erica_.erica_link_data_[ID_L_ARM_END]->position_.coeff(0) + msg->pose.position.x;
    double tar_y = erica_.erica_link_data_[ID_L_ARM_END]->position_.coeff(1) + msg->pose.position.y;
    double tar_z = erica_.erica_link_data_[ID_L_ARM_END]->position_.coeff(2) + msg->pose.position.z;

    Eigen::Quaterniond orientation_displacement(msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);

    Eigen::Matrix3d tar_ori = erica_.erica_link_data_[ID_L_ARM_END]->orientation_ * orientation_displacement.toRotationMatrix();

    Eigen::Matrix4d mat_g_to_tar = robotis_framework::getTranslation4D(tar_x, tar_y, tar_z);
    mat_g_to_tar.block<3,3>(0,0) = tar_ori;

    Eigen::Matrix4d mat_sh_new_to_sh = robotis_framework::getRotation4d(0,0,M_PI) * robotis_framework::getRotation4d(0, M_PI_2, 0);
    Eigen::Matrix4d mat_sh_to_g = robotis_framework::getTranslation4D(0, -0.21, -0.936);
    Eigen::Matrix4d mat_tar_to_tar_new = robotis_framework::getRotation4d(0, -M_PI_2, 0) * robotis_framework::getRotation4d(0,0,M_PI);
    Eigen::Matrix4d mat_sh_new_to_tar_new = ((mat_sh_new_to_sh * mat_sh_to_g) * mat_g_to_tar) * mat_tar_to_tar_new;

    Eigen::Matrix3d new_tar_ori = mat_sh_new_to_tar_new.block<3,3>(0,0);

    bool ik_result = false;

    process_mutex_.lock();
    ik_result = erica_.calcInverseKinematicsForLeftArm(ik_result_angle_,
        mat_sh_new_to_tar_new.coeff(0,3),
        mat_sh_new_to_tar_new.coeff(1,3),
        mat_sh_new_to_tar_new.coeff(2,3),
        new_tar_ori);
    process_mutex_.unlock();

    if(ik_result == false)
      return;
    else
    {
      ik_send_ = true;
    }
  }
  else if (msg->name == "right")
  {
    return;
  }
  else
    return;

  return;
}


void ArmModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                 std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  process_mutex_.lock();
  result_["l_arm_sh_p"]->goal_position_ = dxls["l_arm_sh_p"]->dxl_state_->goal_position_;
  result_["l_arm_sh_r"]->goal_position_ = dxls["l_arm_sh_r"]->dxl_state_->goal_position_;
  result_["l_arm_sh_y"]->goal_position_ = dxls["l_arm_sh_y"]->dxl_state_->goal_position_;
  result_["l_arm_el_p"]->goal_position_ = dxls["l_arm_el_p"]->dxl_state_->goal_position_;
  result_["l_arm_wr_p"]->goal_position_ = dxls["l_arm_wr_p"]->dxl_state_->goal_position_;
  result_["l_arm_wr_r"]->goal_position_ = dxls["l_arm_wr_r"]->dxl_state_->goal_position_;

  if(ik_send_ == true)
  {
    result_["l_arm_sh_p"]->goal_position_ = ik_result_angle_[0];
    result_["l_arm_sh_r"]->goal_position_ = ik_result_angle_[1];
    result_["l_arm_sh_y"]->goal_position_ = ik_result_angle_[2];
    result_["l_arm_el_p"]->goal_position_ = ik_result_angle_[3];
    result_["l_arm_wr_p"]->goal_position_ = ik_result_angle_[4];
    result_["l_arm_wr_r"]->goal_position_ = ik_result_angle_[5];

    ik_send_ = false;
  }

  erica_.erica_link_data_[ID_L_ARM_START + 2*0]->joint_angle_ = result_["l_arm_sh_p"]->goal_position_;
  erica_.erica_link_data_[ID_L_ARM_START + 2*1]->joint_angle_ = result_["l_arm_sh_r"]->goal_position_;
  erica_.erica_link_data_[ID_L_ARM_START + 2*2]->joint_angle_ = result_["l_arm_sh_y"]->goal_position_;
  erica_.erica_link_data_[ID_L_ARM_START + 2*3]->joint_angle_ = result_["l_arm_el_p"]->goal_position_;
  erica_.erica_link_data_[ID_L_ARM_START + 2*4]->joint_angle_ = result_["l_arm_wr_p"]->goal_position_;
  erica_.erica_link_data_[ID_L_ARM_START + 2*5]->joint_angle_ = result_["l_arm_wr_r"]->goal_position_;

  erica_.calcForwardKinematics(0);
  process_mutex_.unlock();
}

void ArmModule::stop()
{
  return;
}

bool ArmModule::isRunning()
{
  return false;
}

void ArmModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Arm";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}
