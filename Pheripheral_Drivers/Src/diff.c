/*
 * diff.c
 *
 *  Created on: 4 Feb 2022
 *      Author: teob0014
 */
#include "diff.h"


double diff_rotations(double distance){
	double wheel_circum = M_PI * 2 * DIFF_WHEEL_RADIUS;
	return distance / wheel_circum;
}

double diff_radian(float degree){
    return 2 * M_PI *  (degree / 360);
}
double diff_get_Lmotor_values(int wheel_angle, int ideal_value){
    double dist_from_ideal = DIFF_WHEEL_GAP / 2;
    double turning_rad = DIFF_VEHICLE_LENGTH / tan(diff_radian(abs(wheel_angle)));
    if (wheel_angle <0){
        double turning_diff = (turning_rad  - ( dist_from_ideal) )/ turning_rad;
        return turning_diff*ideal_value;
    }else if(wheel_angle >0){
        double turning_diff = (turning_rad  + ( dist_from_ideal) )/ turning_rad;
        return turning_diff*ideal_value;
    }else {
    	return ideal_value;
    }
}

double diff_get_Rmotor_values(int wheel_angle, int ideal_value){

    double dist_from_ideal = DIFF_WHEEL_GAP / 2;
    double turning_rad = DIFF_VEHICLE_LENGTH / tan(diff_radian(abs(wheel_angle)));
    if (wheel_angle > 0){
        double turning_diff = (turning_rad  - ( dist_from_ideal) )/ turning_rad;
        return turning_diff*ideal_value;
    }else if(wheel_angle < 0){
        double turning_diff = (turning_rad  + ( dist_from_ideal) )/ turning_rad;
        return turning_diff*ideal_value;
    }else {
    	return ideal_value;
    }
}

double diff_get_ideal_turn_reduction(int wheel_angle, int ideal_value){
	// cutting speed in half for a 90 degree turn
	float halfvalue = ideal_value /2;
	return halfvalue + abs(wheel_angle)*halfvalue;
}






