/*
 * pid.h
 *
 *  Created on: Jan 28, 2022
 *      Author: ytan196
 */

#include "main.h"
#include "stdlib.h"
#include "math.h"

#ifndef INC_PID_H_
#define INC_PID_H_



typedef struct{
	float Pfactor;
	float Ifactor;
	float Dfactor;
	float Error;
	uint32_t Tickdiff;
}pid_frame;


typedef struct{
	pid_frame* prev;
	pid_frame* curr;
	float *kp;
	float *ki;
}pid_control;


pid_control* pid_init(float *kp, float *ki);

float pid_run(pid_control* pid_ctl, float reqvalue, float actvalue, uint32_t tickdiff);

float pid_get_curr_pid(pid_control* pid_ctl);

void pid_reset_error(pid_control*pid_ctl);

void pid_stall_error(pid_control* pid_ctl);
#endif /* INC_PID_H_ */
