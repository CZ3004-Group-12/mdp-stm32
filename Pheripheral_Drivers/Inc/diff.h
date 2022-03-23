/*
 * diff.h
 *
 *  Created on: 4 Feb 2022
 *      Author: teob0014
 */

#ifndef INC_DIFF_H_
#define INC_DIFF_H_
#include "stdlib.h"
#include "main.h"
#include "math.h"
//values in cm
#define DIFF_VEHICLE_LENGTH 14.5
#define DIFF_WHEEL_GAP 15.2
#define DIFF_WHEEL_RADIUS 3.3


double diff_rotations(double distance);
double diff_radian(float degree);
double diff_get_Lmotor_values(int wheel_angle, int ideal_value);
double diff_get_Rmotor_values(int wheel_angle, int ideal_value);
double diff_get_ideal_turn_reduction(int wheel_angle, int ideal_value);
#endif /* INC_DIFF_H_ */
