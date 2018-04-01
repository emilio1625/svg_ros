//
// Created by emilio1625 on 30/03/18.
//

#ifndef SVG_ROS_CONSTANTS_H
#define SVG_ROS_CONSTANTS_H

// Defining constants

#define LIMIT_SIM 150  // Maximum number of steps
#define MAG_ADVANCE 0.04 // Advance magnitude in meters
#define CNT_GOAL 1.5 // constant to find a threshold to reach the goal
#define CNT_GOAL2 1.4
//#define K_GOAL CNT_GOAL2*MAG_ADVANCE
//#define K_INTENSITY K_GOAL
#define THRS_SENSOR 1.5*MAG_ADVANCE //
#ifndef PI
#define PI 3.1415926535f
#endif
#define TURN_ANGLE PI/8.0
#define RADIO_ROBOT 0.03 // Radio Robot in meters
//#define NUM_MAX_SENSORS 4097
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define NUM_MAX_WORDS 100


#endif //SVG_ROS_CONSTANTS_H
