/*
 * erica_head_module.cpp
 *
 *  Created on: Nov 1, 2018
 *      Author: heroehs
 */

#include <stdio.h>
#include <cmath>
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

	mode_=1;
	max_limit_[0]=50;
	max_limit_[1]=50;
	max_limit_[2]=30;

	joint_name_to_id_.clear();
	joint_id_to_name_.clear();

	joint_name_to_curr_pose_.clear();

	//manual
	joint_id_to_rad_.clear();
	//tracking
	joint_name_to_goal_pose_.clear();

	max_boxsize_ = 0;
	min_boxsize_ = 0;

	p_px=0;
	p_py=0;
	p_bw=0;
	p_bh=0;
	p_bs=0;
	p_iw=0;
	p_ih=0;


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

		dxl_pidcontroller[joint_name]=new heroehs_math::PIDController(); //for manual
		//dxl_pidcontroller[joint_name]->PID_set_gains(0.12,0,0);

	}

        dxl_pidcontroller[joint_id_to_name_[13]]->PID_set_gains(0.21,0,0.02);
        dxl_pidcontroller[joint_id_to_name_[14]]->PID_set_gains(0.21,0,0.05);
        dxl_pidcontroller[joint_id_to_name_[15]]->PID_set_gains(0.21,0,0);
	ROS_INFO("< -------  Initialize Module : Head Module !!  ------->");
}


void HeadModule::headmanualCallback(const std_msgs::Bool::ConstPtr& msg)
{
	//ROS_INFO("=================================\n");
	//ROS_INFO("		HEAD MANUAL CONTROL\n");
	mode_=1;
	new_count_ = 1;
	is_moving_state=false;

}

void HeadModule::headctrlCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	if(mode_ != 1)
	{
		return;
	}
	joint_id_to_rad_[13] = DEG2RAD(msg->data[0]);
	joint_id_to_rad_[14] = DEG2RAD(msg->data[1]);
	joint_id_to_rad_[15] = DEG2RAD(msg->data[2]);

	for(int i=13; i<16;i++)
	{
		if(joint_id_to_rad_[i]>DEG2RAD(max_limit_[i-13]))
		{
			joint_id_to_rad_[i]=DEG2RAD(max_limit_[i-13]);
		}
		else if(joint_id_to_rad_[i]<-DEG2RAD(max_limit_[i-13]))
		{
			joint_id_to_rad_[i]=-DEG2RAD(max_limit_[i-13]);
		}
	}

	//ROS_INFO("-----------------------------------\n");
	//ROS_INFO("receive rad\n");
	//ROS_INFO("ID 13 Value : %f \n", joint_id_to_rad_[13]);
	//ROS_INFO("ID 14 Value : %f \n", joint_id_to_rad_[14]);
	//ROS_INFO("ID 15 Value : %f \n", joint_id_to_rad_[15]);
	is_moving_state=true;

}


double HeadModule::mapping_num(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void HeadModule::headtrackingCallback(const std_msgs::Bool::ConstPtr& msg)
{
	//ROS_INFO("=================================\n");
	//ROS_INFO("		HEAD TRACKING\n");
	mode_=2;
	new_count_=1;
	is_moving_state=false;
}

void HeadModule::headtrackingctrlCallback(const erica_perception_msgs::PeoplePositionArray::ConstPtr& msg)
{

	double error_yaw;
	double error_pitch;
	double error_roll;
	double img_y_center;


	if( mode_ != 2 ) //|| msg->box_size.size()==0 || msg->box_size[0].data > 280000 )
	{
		//mode , no people, box size too big
		return;
	}

	else if( msg->box_size.size()==0 || (msg->box_size[0].data < 10000 && msg->box_size[0].data > 0 ) )
	{
		//no box (no people return same(if no detect no callback--> result for last person....))
		error_yaw = 0;
		error_pitch = 0;
		error_roll = 0;
	}

        /*
	else if(msg->box_size[0].data < 0)
	{
            ROS_INFO("3333\n");
		//사람 없을 경우 0,0,0
		error_yaw = -joint_name_to_curr_pose_[joint_id_to_name_[13]];
		error_pitch = -joint_name_to_curr_pose_[joint_id_to_name_[14]];
		error_roll = -joint_name_to_curr_pose_[joint_id_to_name_[15]];
                return;
	}
        */
	else
	{
        //        ROS_INFO("44444");
		p_px=msg->pixel_x[0].data;
		p_py=msg->pixel_y[0].data;
		p_bw=msg->box_width[0].data;
		p_bh=msg->box_height[0].data;
		p_bs=msg->box_size[0].data;
		p_iw=msg->img_width.data;
		p_ih=msg->img_height.data;

		img_y_center = p_py + p_bh * (1/(double)2 - 1/(double)3);

		error_yaw = DEG2RAD(mapping_num(p_px,-(p_iw/2),(p_iw/2),60,-60));
		error_pitch = DEG2RAD(mapping_num(img_y_center,-(p_ih/2),(p_ih/2),-40,40));
		error_roll = 0;

                //ROS_INFO(" %f | %f | %f | %f | %f \n",p_px, p_py, p_bh, p_iw,p_ih);
                //ROS_INFO("  %f ||| %f ||| %f |||\n",error_yaw,error_pitch,error_roll);
	}

	///////////////////////////////////

	joint_id_to_rad_[13]=joint_name_to_curr_pose_[joint_id_to_name_[13]]+error_yaw;
	joint_id_to_rad_[14]=joint_name_to_curr_pose_[joint_id_to_name_[14]]+error_pitch;
	joint_id_to_rad_[15]=joint_name_to_curr_pose_[joint_id_to_name_[15]]+error_roll;

        //ROS_INFO("  %f ||| %f ||| %f |||\n",joint_id_to_rad_[13],joint_id_to_rad_[14],joint_id_to_rad_[15]);
	
        for(int i=13; i<16;i++)
	{
		if(joint_id_to_rad_[i]>DEG2RAD(max_limit_[i-13]))
		{
			joint_id_to_rad_[i]=DEG2RAD(max_limit_[i-13]);
		}
		else if(joint_id_to_rad_[i]<-DEG2RAD(max_limit_[i-13]))
		{
			joint_id_to_rad_[i]=-DEG2RAD(max_limit_[i-13]);
		}
	}

	if(joint_id_to_rad_[14]<-DEG2RAD(10))
	{
		joint_id_to_rad_[14]=-DEG2RAD(10);
	}
        //ROS_INFO("  %f ||| %f ||| %f |||\n",joint_id_to_rad_[13],joint_id_to_rad_[14],joint_id_to_rad_[15]);

	is_moving_state=true;

}



void HeadModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* subscribe topics */
	// for gui
	ros::Subscriber head_ctrl_sub = ros_node.subscribe("/erica/head_ctrl", 5, &HeadModule::headctrlCallback, this);
	ros::Subscriber head_manual_sub = ros_node.subscribe("/erica/head_manual", 5, &HeadModule::headmanualCallback, this);
	ros::Subscriber head_tracking_sub = ros_node.subscribe("/erica/head_tracking", 5, &HeadModule::headtrackingCallback, this);
	ros::Subscriber head_tracking_ctrl_sub = ros_node.subscribe("/erica/people_position", 5, &HeadModule::headtrackingctrlCallback, this);


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
			//printf("id: %d=> %f | ",i,result_[joint_id_to_name_[i]]->goal_position_);
		}
		//printf("\n");

	}
	if(is_moving_state==true && mode_==2)  //tracking mode
	{

                //ROS_INFO("  %f ||| %f ||| %f |||\n",joint_id_to_rad_[13],joint_id_to_rad_[14],joint_id_to_rad_[15]);

		for(int i=13; i<16;i++)
		{
			joint_name_to_curr_pose_[joint_id_to_name_[i]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
			result_[joint_id_to_name_[i]]->goal_position_=joint_name_to_curr_pose_[joint_id_to_name_[i]]
																				   + dxl_pidcontroller[joint_id_to_name_[i]]->PID_process(joint_id_to_rad_[i],joint_name_to_curr_pose_[joint_id_to_name_[i]]);

		}

	}
}


void HeadModule::stop()
{
	return;
}



