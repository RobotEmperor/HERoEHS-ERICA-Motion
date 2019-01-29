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

	mode_=0;

	joint_name_to_id_.clear();
	joint_id_to_name_.clear();
	joint_id_to_rad_.clear();

	joint_name_to_curr_pose_.clear();
	joint_name_to_goal_pose_.clear();


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

		dxl_pidcontroller[joint_name]=new heroehs_math::PIDController();
		dxl_pidcontroller[joint_name]->PID_set_gains(0.08,0,0);

	}
	ROS_INFO("< -------  Initialize Module : Head Module !!  ------->");
}


void HeadModule::headmanualCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("=================================\n");
	ROS_INFO("		HEAD MANUAL CONTROL\n");
	mode_=1;
	new_count_ = 1;
	is_moving_state=false;

}

void HeadModule::headctrlCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	if( abs(msg->data[0]) > 20 || abs(msg->data[1]) > 20 || abs(msg->data[2]) > 20)
	{
		ROS_INFO(" OVER LIMIT \n");
		return;
	}
	joint_id_to_rad_[13] = DEG2RAD(msg->data[0]);
	joint_id_to_rad_[14] = DEG2RAD(msg->data[1]);
	joint_id_to_rad_[15] = DEG2RAD(msg->data[2]);


	//motion_time_=2;
	ROS_INFO("-----------------------------------\n");
	//ROS_INFO("receive rad\n");
	ROS_INFO("ID 13 Value : %f \n", joint_id_to_rad_[13]);
	ROS_INFO("ID 14 Value : %f \n", joint_id_to_rad_[14]);
	ROS_INFO("ID 15 Value : %f \n", joint_id_to_rad_[15]);
	is_moving_state=true;

}

void HeadModule::headtrackingCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("=================================\n");
	ROS_INFO("		HEAD TRACKING\n");
	mode_=2;
	new_count_=1;
	is_moving_state=false;
}


void HeadModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* subscribe topics */
	// for gui
	ros::Subscriber head_ctrl_sub = ros_node.subscribe("/ij/head_ctrl", 5, &HeadModule::headctrlCallback, this);

	ros::Subscriber head_manual_sub = ros_node.subscribe("/ij/head_manual", 5, &HeadModule::headmanualCallback, this);
	ros::Subscriber head_tracking_sub = ros_node.subscribe("/ij/head_tracking", 5, &HeadModule::headtrackingCallback, this);



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

	// code
	if(new_count_ == 1)
	{
		new_count_++;


		for(int i=13; i<16;i++)
		{
			result_[joint_id_to_name_[i]]->goal_position_=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
			joint_name_to_curr_pose_[joint_id_to_name_[i]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
			joint_id_to_rad_[joint_name_to_id_[joint_id_to_name_[i]]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;

		}

	}

	if(is_moving_state==true && mode_==1)   //manual mode
	{
		for(int i=13; i<16;i++)
		{
			joint_name_to_curr_pose_[joint_id_to_name_[i]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
			result_[joint_id_to_name_[i]]->goal_position_=joint_name_to_curr_pose_[joint_id_to_name_[i]]
																				   + dxl_pidcontroller[joint_id_to_name_[i]]->PID_process(joint_id_to_rad_[i],joint_name_to_curr_pose_[joint_id_to_name_[i]]);

		}

	}
	if(is_moving_state==true && mode_==2)  //tracking mode
	{
		for(int i=13; i<16;i++)
		{
			joint_name_to_curr_pose_[joint_id_to_name_[i]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
			//result_[joint_id_to_name_[i]]->goal_position_=joint_name_to_curr_pose_[joint_id_to_name_[i]]
			//																	   + dxl_pidcontroller[joint_id_to_name_[i]]->PID_process(joint_id_to_rad_[i],joint_name_to_curr_pose_[joint_id_to_name_[i]]);

		}

	}
}
void HeadModule::stop()
{
	return;
}



