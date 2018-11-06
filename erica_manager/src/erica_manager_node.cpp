/*
 * erica_manager_node.cpp
 *
 *  Created on: Oct 29, 2018
 *      Author: robotemperor
 */

#include "robotis_controller/robotis_controller.h"
#include "erica_base_module/erica_base_module.h"
#include "erica_head_module/erica_head_module.h"
#include "erica_arm_module/arm_module.h"
#include "erica_action_module/action_module.h"

using namespace erica;
using namespace erica_alice;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "erica_Manager");
    ros::NodeHandle nh;

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");

    std::string init_file   = nh.param<std::string>("init_file_path", "");

    /* gazebo simulation */
    controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
    if(controller->gazebo_mode_ == true)
    {
        ROS_WARN("SET TO GAZEBO MODE!");
        std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
        if(robot_name != "")
            controller->gazebo_robot_name_ = robot_name;
    }

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);
    controller->addMotionModule((robotis_framework::MotionModule*)BaseModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)HeadModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)ArmModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)ActionModule::getInstance());
    sleep(1);

  //controller->DEBUG_PRINT = true;
    controller->startTimer();
    while(ros::ok())
    {
      usleep(1000*1000);
    }

//    controller->initializeSyncWrite();
//
//    ROS_INFO("---------1--------");
//    for (auto& it : controller->port_to_bulk_read_)
//    {
//    	it.second->txPacket();
//    }
//    ROS_INFO("---------2--------");
//    usleep(8 * 1000);
//
//    ros::Rate r(125); // 125 hz
//    while (ros::ok())
//    {
//
//    	controller->process();
//    	r.sleep();
//    }


    return 0;
}



