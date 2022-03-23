/*
 * distance_preset.c
 *
 *  Created on: 17 Feb 2022
 *      Author: skylivingston
 */
#include "dist.h"



int dist_get_motor_diststeps(double distance,int Terrain){
	double simple_step = (DIST_10CM_LAB/10) * distance;
	return (int)round(simple_step) - DIST_BRAKE_CONST_LAB;
}

