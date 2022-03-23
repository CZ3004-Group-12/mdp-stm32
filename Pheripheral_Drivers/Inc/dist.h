/*
 * dist.h
 *
 *  Created on: 17 Feb 2022
 *      Author: skylivingston
 */

#ifndef INC_DIST_H_
#define INC_DIST_H_

#include <math.h>

#define DIST_10CM_LAB 123
#define DIST_BRAKE_CONST_LAB 3

int dist_get_motor_diststeps(double distance,int Terrain);
#endif /* INC_DIST_H_ */
