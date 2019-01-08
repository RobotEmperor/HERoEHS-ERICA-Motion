/*
 * kinematics_dynamics_define.h
 *
 *  Created on: Oct 30, 2018
 *      Author: jay
 */

#ifndef ERICA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_
#define ERICA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_

namespace erica_alice
{
#define MAX_JOINT_ID    (15)
#define ALL_JOINT_ID    (26) //base, passive xyzrpy, body, 3 branches' ends

#define MAX_ARM_ID      (6)
#define MAX_ITER        (5)

#define ID_HEAD_END     (19)
#define ID_BODY         (16)

#define ID_L_ARM_START  (1)
#define ID_R_ARM_START  (2)
#define ID_L_ARM_END    (17)
#define ID_R_ARM_END    (18)


//#define ID_R_LEG_START  (7)
//#define ID_L_LEG_START  (8)
//#define ID_R_LEG_END    (31)
//#define ID_L_LEG_END    (30)

#define GRAVITY_ACCELERATION (9.8)

}




#endif /* ERICA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_ */
