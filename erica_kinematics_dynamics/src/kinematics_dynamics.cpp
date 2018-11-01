/*
 * erica_kinematics_dynamics.cpp
 *
 *  Created on: Oct 30, 2018
 *      Author: jay
 */

#include "erica_kinematics_dynamics/kinematics_dynamics.h"

using namespace erica_alice;


KinematicsDynamics::KinematicsDynamics()
{
  for (int id=0; id<=ALL_JOINT_ID; id++)
    erica_link_data_[id] = new LinkData();


  erica_link_data_[0]->name_               =  "base";
  erica_link_data_[0]->parent_             =  -1;
  erica_link_data_[0]->sibling_            =  -1;
  erica_link_data_[0]->child_              =  20;
  erica_link_data_[0]->mass_               =  0.0;
  erica_link_data_[0]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[0]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[0]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[0]->joint_limit_max_    =  100.0;
  erica_link_data_[0]->joint_limit_min_    =  -100.0;
  erica_link_data_[0]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  /* ----- passive joint -----*/

  erica_link_data_[20]->name_               =  "passive_x";
  erica_link_data_[20]->parent_             =  0;
  erica_link_data_[20]->sibling_            =  -1;
  erica_link_data_[20]->child_              =  21;
  erica_link_data_[20]->mass_               =  0.0;
  erica_link_data_[20]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[20]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[20]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[20]->joint_limit_max_    =  100.0;
  erica_link_data_[20]->joint_limit_min_    =  -100.0;
  erica_link_data_[20]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  erica_link_data_[21]->name_               =  "passive_y";
  erica_link_data_[21]->parent_             =  20;
  erica_link_data_[21]->sibling_            =  -1;
  erica_link_data_[21]->child_              =  22;
  erica_link_data_[21]->mass_               =  0.0;
  erica_link_data_[21]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[21]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[21]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[21]->joint_limit_max_    =  100.0;
  erica_link_data_[21]->joint_limit_min_    =  -100.0;
  erica_link_data_[21]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  erica_link_data_[22]->name_               =  "passive_z";
  erica_link_data_[22]->parent_             =  21;
  erica_link_data_[22]->sibling_            =  -1;
  erica_link_data_[22]->child_              =  23;
  erica_link_data_[22]->mass_               =  0.0;
  erica_link_data_[22]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.936 ); // 0.0 , 0.0 , 0.801
  erica_link_data_[22]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[22]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[22]->joint_limit_max_    =  100.0;
  erica_link_data_[22]->joint_limit_min_    =  -100.0;
  erica_link_data_[22]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  erica_link_data_[23]->name_               =  "passive_yaw";
  erica_link_data_[23]->parent_             =  22;
  erica_link_data_[23]->sibling_            =  -1;
  erica_link_data_[23]->child_              =  24;
  erica_link_data_[23]->mass_               =  0.0;
  erica_link_data_[23]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[23]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
  erica_link_data_[23]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[23]->joint_limit_max_    =  M_PI;
  erica_link_data_[23]->joint_limit_min_    =  -M_PI;
  erica_link_data_[23]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  erica_link_data_[24]->name_               =  "passive_pitch";
  erica_link_data_[24]->parent_             =  23;
  erica_link_data_[24]->sibling_            =  -1;
  erica_link_data_[24]->child_              =  25;
  erica_link_data_[24]->mass_               =  0.0;
  erica_link_data_[24]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[24]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
  erica_link_data_[24]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[24]->joint_limit_max_    =  M_PI;
  erica_link_data_[24]->joint_limit_min_    =  -M_PI;
  erica_link_data_[24]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  erica_link_data_[25]->name_               =  "passive_roll";
  erica_link_data_[25]->parent_             =  24;
  erica_link_data_[25]->sibling_            =  -1;
  erica_link_data_[25]->child_              =  16;
  erica_link_data_[25]->mass_               =  0.0;
  erica_link_data_[25]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[25]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
  erica_link_data_[25]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[25]->joint_limit_max_    =  M_PI;
  erica_link_data_[25]->joint_limit_min_    =  -M_PI;
  erica_link_data_[25]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  /* ----- body -----*/
  // pelvis_link
  erica_link_data_[16]->name_               =  "body";
  erica_link_data_[16]->parent_             =  25;
  erica_link_data_[16]->sibling_            =  -1;
  erica_link_data_[16]->child_              =  13;
  erica_link_data_[16]->mass_               =  1.0;
  erica_link_data_[16]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[16]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[16]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[16]->joint_limit_max_    =  100.0;
  erica_link_data_[16]->joint_limit_min_    =  -100.0;
  erica_link_data_[16]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );


  /* ----- head -----*/
  // head_yaw
  erica_link_data_[13]->name_               =  "head_yaw";
  erica_link_data_[13]->parent_             =  16;
  erica_link_data_[13]->sibling_            =  1;
  erica_link_data_[13]->child_              =  14;
  erica_link_data_[13]->mass_               =  1.0;
  erica_link_data_[13]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.14);
  erica_link_data_[13]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
  erica_link_data_[13]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[13]->joint_limit_max_    =  M_PI;
  erica_link_data_[13]->joint_limit_min_    =  -M_PI;
  erica_link_data_[13]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // head_pitch
  erica_link_data_[14]->name_               =  "head_pitch";
  erica_link_data_[14]->parent_             =  13;
  erica_link_data_[14]->sibling_            =  -1;
  erica_link_data_[14]->child_              =  15;
  erica_link_data_[14]->mass_               =  1.0;
  erica_link_data_[14]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.054 );
  erica_link_data_[14]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
  erica_link_data_[14]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.0 );
  erica_link_data_[14]->joint_limit_max_    =  M_PI;
  erica_link_data_[14]->joint_limit_min_    =  -M_PI;
  erica_link_data_[14]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // head_roll
  erica_link_data_[15]->name_               =  "head_roll";
  erica_link_data_[15]->parent_             =  14;
  erica_link_data_[15]->sibling_            =  -1;
  erica_link_data_[15]->child_              =  19;
  erica_link_data_[15]->mass_               =  1.0;
  erica_link_data_[15]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.00 );
  erica_link_data_[15]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
  erica_link_data_[15]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.0 );
  erica_link_data_[15]->joint_limit_max_    =  M_PI;
  erica_link_data_[15]->joint_limit_min_    =  -M_PI;
  erica_link_data_[15]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // zed
  erica_link_data_[19]->name_               =  "zed";
  erica_link_data_[19]->parent_             =  15;
  erica_link_data_[19]->sibling_            =  -1;
  erica_link_data_[19]->child_              =  -1;
  erica_link_data_[19]->mass_               =  0.0;
  erica_link_data_[19]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.08 , 0.0  , 0.0 );
  erica_link_data_[19]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0  , 0.0  , 0.0 );
  erica_link_data_[19]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0  , 0.046, 0.0 );
  erica_link_data_[19]->joint_limit_max_    =  M_PI;
  erica_link_data_[19]->joint_limit_min_    =  -M_PI;
  erica_link_data_[19]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  /*----- left arm -----*/
  // left arm shoulder pitch
  erica_link_data_[1]->name_               =  "l_arm_sh_p";
  erica_link_data_[1]->parent_             =  16;
  erica_link_data_[1]->sibling_            =  2;
  erica_link_data_[1]->child_              =  3;
  erica_link_data_[1]->mass_               =  1.0;
  erica_link_data_[1]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.21 , 0.0 );
  erica_link_data_[1]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
  erica_link_data_[1]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[1]->joint_limit_max_    =  M_PI;
  erica_link_data_[1]->joint_limit_min_    =  -M_PI;
  erica_link_data_[1]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left arm shoulder roll
  erica_link_data_[3]->name_               =  "l_arm_sh_r";
  erica_link_data_[3]->parent_             =  1;
  erica_link_data_[3]->sibling_            =  -1;
  erica_link_data_[3]->child_              =  5;
  erica_link_data_[3]->mass_               =  1.0;
  erica_link_data_[3]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 ,  0.0 );
  erica_link_data_[3]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
  erica_link_data_[3]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 ,  0.0 );
  erica_link_data_[3]->joint_limit_max_    =  M_PI;
  erica_link_data_[3]->joint_limit_min_    =  -M_PI;
  erica_link_data_[3]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left arm shoulder roll
  erica_link_data_[5]->name_               =  "l_arm_sh_y";
  erica_link_data_[5]->parent_             =  3;
  erica_link_data_[5]->sibling_            =  -1;
  erica_link_data_[5]->child_              =  7;
  erica_link_data_[5]->mass_               =  1.0;
  erica_link_data_[5]->relative_position_  =  robotis_framework::getTransitionXYZ(  0.0 , 0.0 , 0.0 );
  erica_link_data_[5]->joint_axis_         =  robotis_framework::getTransitionXYZ( -1.0 , 0.0 , 0.0 );
  erica_link_data_[5]->center_of_mass_     =  robotis_framework::getTransitionXYZ(  0.0 , 0.0 , 0.0 );
  erica_link_data_[5]->joint_limit_max_    =  M_PI;
  erica_link_data_[5]->joint_limit_min_    =  -M_PI;
  erica_link_data_[5]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left arm elbow pitch
  erica_link_data_[7]->name_               =  "l_arm_el_p";
  erica_link_data_[7]->parent_             =  5;
  erica_link_data_[7]->sibling_            =  -1;
  erica_link_data_[7]->child_              =  9;
  erica_link_data_[7]->mass_               =  1.0;
  erica_link_data_[7]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.2 ,  0.0 , 0.0 );
  erica_link_data_[7]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
  erica_link_data_[7]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.0 );
  erica_link_data_[7]->joint_limit_max_    =  M_PI;
  erica_link_data_[7]->joint_limit_min_    =  -M_PI;
  erica_link_data_[7]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left arm wrist pitch
  erica_link_data_[9]->name_               =  "l_arm_wr_p";
  erica_link_data_[9]->parent_             =  5;
  erica_link_data_[9]->sibling_            =  -1;
  erica_link_data_[9]->child_              =  9;
  erica_link_data_[9]->mass_               =  1.0;
  erica_link_data_[9]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.2 ,  0.0 , 0.0 );
  erica_link_data_[9]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
  erica_link_data_[9]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.0 );
  erica_link_data_[9]->joint_limit_max_    =  M_PI;
  erica_link_data_[9]->joint_limit_min_    =  -M_PI;
  erica_link_data_[9]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left arm wrist roll
  erica_link_data_[11]->name_               =  "l_arm_wr_r";
  erica_link_data_[11]->parent_             =   9;
  erica_link_data_[11]->sibling_            =  -1;
  erica_link_data_[11]->child_              =  17;
  erica_link_data_[11]->mass_               =  1.0;
  erica_link_data_[11]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 ,  0.0 );
  erica_link_data_[11]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
  erica_link_data_[11]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 ,  0.0 );
  erica_link_data_[11]->joint_limit_max_    =  M_PI;
  erica_link_data_[11]->joint_limit_min_    = -M_PI;
  erica_link_data_[11]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // left arm end
  erica_link_data_[17]->name_               =  "l_arm_end";
  erica_link_data_[17]->parent_             =  11;
  erica_link_data_[17]->sibling_            =  -1;
  erica_link_data_[17]->child_              =  -1;
  erica_link_data_[17]->mass_               =  1.0;
  erica_link_data_[17]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.034 , 0.0 , 0.0 );
  erica_link_data_[17]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[17]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[17]->joint_limit_max_    =  M_PI;
  erica_link_data_[17]->joint_limit_min_    =  -M_PI;
  erica_link_data_[17]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );


  /*----- right arm -----*/
  // right arm shoulder pitch
  erica_link_data_[2]->name_               =  "r_arm_sh_p";
  erica_link_data_[2]->parent_             =  16;
  erica_link_data_[2]->sibling_            =  -1;
  erica_link_data_[2]->child_              =  4;
  erica_link_data_[2]->mass_               =  0.1;
  erica_link_data_[2]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , -0.21 , 0.0 );
  erica_link_data_[2]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0  , 0.0 );
  erica_link_data_[2]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[2]->joint_limit_max_    =  M_PI;
  erica_link_data_[2]->joint_limit_min_    =  -M_PI;
  erica_link_data_[2]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right arm shoulder roll
  erica_link_data_[4]->name_               =  "r_arm_sh_r";
  erica_link_data_[4]->parent_             =  2;
  erica_link_data_[4]->sibling_            =  -1;
  erica_link_data_[4]->child_              =  6;
  erica_link_data_[4]->mass_               =  0.1;
  erica_link_data_[4]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[4]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
  erica_link_data_[4]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[4]->joint_limit_max_    =  M_PI;
  erica_link_data_[4]->joint_limit_min_    =  -M_PI;
  erica_link_data_[4]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right arm shoulder yaw
  erica_link_data_[6]->name_               =  "r_arm_sh_y";
  erica_link_data_[6]->parent_             =  4;
  erica_link_data_[6]->sibling_            =  -1;
  erica_link_data_[6]->child_              =  8;
  erica_link_data_[6]->mass_               =  0.1;
  erica_link_data_[6]->relative_position_  =  robotis_framework::getTransitionXYZ(  0.0 ,  0.0 , 0.0 );
  erica_link_data_[6]->joint_axis_         =  robotis_framework::getTransitionXYZ( -1.0 ,  0.0 , 0.0 );
  erica_link_data_[6]->center_of_mass_     =  robotis_framework::getTransitionXYZ(  0.0 ,  0.0 , 0.0 );
  erica_link_data_[6]->joint_limit_max_    =  M_PI;
  erica_link_data_[6]->joint_limit_min_    =  -M_PI;
  erica_link_data_[6]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right arm elbow pitch
  erica_link_data_[8]->name_               =  "r_arm_el_p";
  erica_link_data_[8]->parent_             =  6;
  erica_link_data_[8]->sibling_            =  -1;
  erica_link_data_[8]->child_              =  12;
  erica_link_data_[8]->mass_               =  0.1;
  erica_link_data_[8]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.2 ,  0.0 , 0.0 );
  erica_link_data_[8]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 ,  1.0 , 0.0 );
  erica_link_data_[8]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.0 );
  erica_link_data_[8]->joint_limit_max_    =  M_PI;
  erica_link_data_[8]->joint_limit_min_    =  -M_PI;
  erica_link_data_[8]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right arm wrist pitch
  erica_link_data_[10]->name_               =  "r_arm_wr_p";
  erica_link_data_[10]->parent_             =  8;
  erica_link_data_[10]->sibling_            =  -1;
  erica_link_data_[10]->child_              =  12;
  erica_link_data_[10]->mass_               =  0.1;
  erica_link_data_[10]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.2 ,  0.0 , 0.0 );
  erica_link_data_[10]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 ,  1.0 , 0.0 );
  erica_link_data_[10]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 ,  0.0 , 0.0 );
  erica_link_data_[10]->joint_limit_max_    =  M_PI;
  erica_link_data_[10]->joint_limit_min_    =  -M_PI;
  erica_link_data_[10]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right arm wrist roll
  erica_link_data_[12]->name_               =  "r_arm_wr_r";
  erica_link_data_[12]->parent_             =  10;
  erica_link_data_[12]->sibling_            =  -1;
  erica_link_data_[12]->child_              =  18;
  erica_link_data_[12]->mass_               =  0.1;
  erica_link_data_[12]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 ,  0.0 );
  erica_link_data_[12]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
  erica_link_data_[12]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 ,  0.0 );
  erica_link_data_[12]->joint_limit_max_    =  M_PI;
  erica_link_data_[12]->joint_limit_min_    =  -M_PI;
  erica_link_data_[12]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

  // right arm end
  erica_link_data_[18]->name_               =  "r_arm_end";
  erica_link_data_[18]->parent_             =  12;
  erica_link_data_[18]->sibling_            =  -1;
  erica_link_data_[18]->child_              =  -1;
  erica_link_data_[18]->mass_               =  0.1;
  erica_link_data_[18]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.034 , 0.0 , 0.0 );
  erica_link_data_[18]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[18]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
  erica_link_data_[18]->joint_limit_max_    =  M_PI;
  erica_link_data_[18]->joint_limit_min_    =  -M_PI;
  erica_link_data_[18]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );


  for(int joint_idx = 0; joint_idx < ALL_JOINT_ID; joint_idx++)
  {
    erica_link_data_[joint_idx]->joint_center_of_mass_ = erica_link_data_[joint_idx]->center_of_mass_;
  }
}

KinematicsDynamics::~KinematicsDynamics()
{  }

std::vector<int> KinematicsDynamics::findRoute(int to)
{
  int id = erica_link_data_[to]->parent_;

  std::vector<int> idx;

  if(id == 0)
  {
    idx.push_back(0);
    idx.push_back(to);
  }
  else
  {
    idx = findRoute(id);
    idx.push_back(to);
  }

  return idx;
}

std::vector<int> KinematicsDynamics::findRoute(int from, int to)
{
  int id = erica_link_data_[to]->parent_;

  std::vector<int> idx;

  if(id == from)
  {
    idx.push_back(from);
    idx.push_back(to);
  }
  else if (id != 0)
  {
    idx = findRoute(from, id);
    idx.push_back(to);
  }

  return idx;
}

double KinematicsDynamics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = erica_link_data_[joint_id]->mass_ + calcTotalMass(erica_link_data_[ joint_id ]->sibling_) + calcTotalMass(erica_link_data_[joint_id]->child_);

  return mass;
}

Eigen::MatrixXd KinematicsDynamics::calcMassCenter(int joint_id)
{
  Eigen::MatrixXd mc(3,1);

  if (joint_id == -1)
    mc = Eigen::MatrixXd::Zero(3,1);
  else
  {
    mc = erica_link_data_[ joint_id ]->mass_ * ( erica_link_data_[ joint_id ]->orientation_ * erica_link_data_[ joint_id ]->center_of_mass_ + erica_link_data_[ joint_id ]->position_ );
    mc = mc + calcMassCenter( erica_link_data_[ joint_id ]->sibling_ ) + calcMassCenter( erica_link_data_[ joint_id ]->child_ );
  }

  return mc;
}

void KinematicsDynamics::calcJointsCenterOfMass(int joint_id)
{
  if(joint_id != -1)
  {
    LinkData *temp_data = erica_link_data_[ joint_id ];
    temp_data->joint_center_of_mass_
      = ( temp_data->orientation_ * temp_data->center_of_mass_ + temp_data->position_ );

    calcJointsCenterOfMass(temp_data->sibling_);
    calcJointsCenterOfMass(temp_data->child_);
  }
  else
    return;
}

Eigen::MatrixXd KinematicsDynamics::calcCenterOfMass(Eigen::MatrixXd mc)
{
  double mass ;
  Eigen::MatrixXd COM(3,1);

  mass = calcTotalMass(0);
  COM = mc/mass;

  return COM;
}

void KinematicsDynamics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
  {
    erica_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3,1);
    erica_link_data_[0]->orientation_ =
        robotis_framework::calcRodrigues( robotis_framework::calcHatto( erica_link_data_[0]->joint_axis_ ), erica_link_data_[ 0 ]->joint_angle_ );

    erica_link_data_[0]->transformation_.block<3,1>(0,3) = erica_link_data_[0]->position_;
    erica_link_data_[0]->transformation_.block<3,3>(0,0) = erica_link_data_[0]->orientation_;
  }

  if ( joint_id != 0 )
  {
    int parent = erica_link_data_[joint_id]->parent_;

    erica_link_data_[joint_id]->position_ =
        erica_link_data_[parent]->orientation_ * erica_link_data_[joint_id]->relative_position_ + erica_link_data_[parent]->position_;
    erica_link_data_[ joint_id ]->orientation_ =
        erica_link_data_[ parent ]->orientation_ *
        robotis_framework::calcRodrigues(robotis_framework::calcHatto(erica_link_data_[joint_id]->joint_axis_), erica_link_data_[joint_id]->joint_angle_);

    erica_link_data_[joint_id]->transformation_.block<3,1>(0,3) = erica_link_data_[joint_id]->position_;
    erica_link_data_[joint_id]->transformation_.block<3,3>(0,0) = erica_link_data_[joint_id]->orientation_;
  }

  calcForwardKinematics(erica_link_data_[joint_id]->sibling_);
  calcForwardKinematics(erica_link_data_[joint_id]->child_);
}

Eigen::MatrixXd KinematicsDynamics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size-1;

  Eigen::Vector3d tar_position = erica_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,idx_size);

  for (int id=0; id<idx_size; id++)
  {
    int curr_id = idx[id];

    Eigen::Vector3d tar_orientation = erica_link_data_[curr_id]->orientation_ * erica_link_data_[curr_id]->joint_axis_;

    jacobian.block(0,id,3,1) = robotis_framework::calcCross(tar_orientation, tar_position - erica_link_data_[curr_id]->position_);
    jacobian.block(3,id,3,1) = tar_orientation;
  }

  return jacobian;
}

Eigen::MatrixXd KinematicsDynamics::calcVWerr(Eigen::Vector3d tar_position, Eigen::Vector3d curr_position, Eigen::Matrix3d tar_orientation, Eigen::Matrix3d curr_orientation)
{
  Eigen::Vector3d pos_err = tar_position - curr_position;
  Eigen::Matrix3d ori_err = curr_orientation.transpose() * tar_orientation;
  Eigen::Vector3d ori_err_dash = curr_orientation * robotis_framework::convertRotToOmega(ori_err);

  Eigen::MatrixXd err = Eigen::MatrixXd::Zero(6,1);
  err.block<3,1>(0,0) = pos_err;
  err.block<3,1>(3,0) = ori_err_dash;

  return err;
}

bool KinematicsDynamics::calcInverseKinematics(int to, Eigen::Vector3d tar_position, Eigen::Matrix3d tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  for (int iter=0; iter<max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::Vector3d curr_position = erica_link_data_[to]->position_;
    Eigen::Matrix3d curr_orientation = erica_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm()<ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inverse = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inverse * err ;

    for (int id=0; id<idx.size(); id++)
    {
      int joint_num = idx[id];
      erica_link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  for ( int id = 0; id < idx.size(); id++ )
  {
    int joint_num      =  idx[ id ];

    if ( erica_link_data_[ joint_num ]->joint_angle_ >= erica_link_data_[ joint_num ]->joint_limit_max_ )
    {
      limit_success = false;
      break;
    }
    else if ( erica_link_data_[ joint_num ]->joint_angle_ <= erica_link_data_[ joint_num ]->joint_limit_min_ )
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematics(int from, int to, Eigen::Vector3d tar_position, Eigen::Matrix3d tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  for (int iter=0; iter<max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::Vector3d curr_position = erica_link_data_[to]->position_;
    Eigen::Matrix3d curr_orientation = erica_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm()<ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inv = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err ;

    for (int id=0; id<idx.size(); id++)
    {
      int joint_num = idx[id];
      erica_link_data_[joint_num]->joint_angle_ +=delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  for ( int id = 0; id < idx.size(); id++ )
  {
    int joint_num      =   idx[ id ];

    if ( erica_link_data_[ joint_num ]->joint_angle_ >= erica_link_data_[ joint_num ]->joint_limit_max_ )
    {
      limit_success = false;
      break;
    }
    else if ( erica_link_data_[ joint_num ]->joint_angle_ <= erica_link_data_[ joint_num ]->joint_limit_min_ )
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematicsForArm(double *out, double x, double y, double z, Eigen::Matrix3d& tar_ori)
{
  //a : origin, d : end, c: ankle
  Eigen::Matrix4d trans_ad, trans_da, trans_cd, trans_dc;
  Eigen::Matrix3d rot_ac;
  Eigen::Vector3d vec;

  bool invertible;
  double rac, arc_cos, arc_tan, alpha;
  double upper_length = 1.0;
  double lower_length = 1.0;
  double wrist_length = 1.0;

  trans_ad = robotis_framework::getTransformationXYZRPY(x, y, z, 0,0,0);
  trans_ad.block<3,3>(0,0) = tar_ori;

  vec.coeffRef(0) = trans_ad.coeff(0, 3) + trans_ad.coeff(0, 2) * wrist_length;
  vec.coeffRef(1) = trans_ad.coeff(1, 3) + trans_ad.coeff(1, 2) * wrist_length;
  vec.coeffRef(2) = trans_ad.coeff(2, 3) + trans_ad.coeff(2, 2) * wrist_length;

  // Get Knee
  rac = vec.norm();
  arc_cos = acos(
      (rac * rac - upper_length * upper_length - lower_length * lower_length) / (2.0 * upper_length * lower_length));
  if (std::isnan(arc_cos) == 1)
    return false;
  *(out + 3) = arc_cos;

  // Get Ankle Roll
  trans_da = robotis_framework::getInverseTransformation(trans_ad);

  vec.coeffRef(0) = trans_da.coeff(0, 3);
  vec.coeffRef(1) = trans_da.coeff(1, 3);
  vec.coeffRef(2) = trans_da.coeff(2, 3) - wrist_length;

  arc_tan = atan2(vec(1), vec(2));
  if(arc_tan > M_PI_2)
    arc_tan = arc_tan - M_PI;
  else if(arc_tan < -M_PI_2)
    arc_tan = arc_tan + M_PI;

  *(out+5) = arc_tan;

  //Get Ankle Pitch
  alpha = asin( upper_length*sin(M_PI - *(out+3)) / rac);
  *(out+4) = -atan2(vec(0), copysign(sqrt(vec(1)*vec(1) + vec(2)*vec(2)),vec(2))) - alpha;

  // Get Hip Pitch
  rot_ac = ( tar_ori * robotis_framework::getRotationX(-(*(out+5)))) * robotis_framework::getRotationY(-(*(out+3) + *(out+4)));

  arc_tan = atan2(rot_ac.coeff(0, 2), rot_ac.coeff(2, 2));
  *(out) = arc_tan;

  // Get Hip Roll
  arc_tan = atan2(-rot_ac.coeff(1, 2), rot_ac.coeff(0, 2) * sin(*(out)) + rot_ac.coeff(2, 2) * cos(*(out)));
  *(out + 1) = arc_tan;

  // Get Hip Yaw
  arc_tan = atan2(rot_ac.coeff(1, 0), rot_ac.coeff(1, 1));
  *(out+2) = arc_tan;

  return true;
}

bool KinematicsDynamics::calcInverseKinematicsForRightArm(double *out, double x, double y, double z, Eigen::Matrix3d& tar_ori)
{
  if(calcInverseKinematicsForArm(out, x, y, z, tar_ori) == true) {

    *(out + 0) = out[0] * (erica_link_data_[ID_R_ARM_START + 2*0]->joint_axis_.coeff(1,0));
    *(out + 1) = out[1] * (erica_link_data_[ID_R_ARM_START + 2*1]->joint_axis_.coeff(2,0));
    *(out + 2) = out[2] * (erica_link_data_[ID_R_ARM_START + 2*2]->joint_axis_.coeff(0,0));
    *(out + 3) = out[3] * (erica_link_data_[ID_R_ARM_START + 2*3]->joint_axis_.coeff(1,0));
    *(out + 4) = out[4] * (erica_link_data_[ID_R_ARM_START + 2*4]->joint_axis_.coeff(1,0));
    *(out + 5) = out[5] * (erica_link_data_[ID_R_ARM_START + 2*5]->joint_axis_.coeff(2,0));
    return true;
  }
  else
    return false;

  return true;
}

bool KinematicsDynamics::calcInverseKinematicsForLeftArm(double *out, double x, double y, double z, Eigen::Matrix3d& tar_ori)
{
  if(calcInverseKinematicsForArm(out, x, y, z, tar_ori) == true) {

    *(out + 0) = out[0] * (erica_link_data_[ID_L_ARM_START + 2*0]->joint_axis_.coeff(1,0));
    *(out + 1) = out[1] * (erica_link_data_[ID_L_ARM_START + 2*1]->joint_axis_.coeff(2,0));
    *(out + 2) = out[2] * (erica_link_data_[ID_L_ARM_START + 2*2]->joint_axis_.coeff(0,0));
    *(out + 3) = out[3] * (erica_link_data_[ID_L_ARM_START + 2*3]->joint_axis_.coeff(1,0));
    *(out + 4) = out[4] * (erica_link_data_[ID_L_ARM_START + 2*4]->joint_axis_.coeff(1,0));
    *(out + 5) = out[5] * (erica_link_data_[ID_L_ARM_START + 2*5]->joint_axis_.coeff(2,0));
    return true;
  }
  else
    return false;

  return true;
}


