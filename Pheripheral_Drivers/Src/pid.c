/*
 * pid.c
 *
 *  Created on: Jan 28, 2022
 *      Author: ytan196
 */
#include "pid.h"
pid_control* pid_init(float *kp, float *ki){
	pid_control *pid_ctl = malloc(sizeof(pid_control));

	pid_ctl->kp = kp;
	pid_ctl->ki = ki;

	pid_frame *now = malloc(sizeof(pid_frame));
	now->Pfactor = 0;
	now->Ifactor = 0;
	now->Dfactor = 0;
	now->Error = 0;
	now->Tickdiff = 0;
	pid_ctl->prev = NULL;
	pid_ctl->curr = now;
	return pid_ctl;
}

//for encoder to run
float pid_run(pid_control* pid_ctl, float reqvalue, float actvalue, uint32_t  tickdiff){

	free(pid_ctl->prev);
	pid_ctl->prev = pid_ctl->curr;
	pid_frame *now = malloc(sizeof(pid_frame));


	float Err = (reqvalue  - actvalue);

	float Ifactor = *(pid_ctl->ki);
	float Pfactor = *(pid_ctl->kp);

	now->Error = Err;
	now->Tickdiff = tickdiff;

	now->Pfactor = (Err) * Pfactor;
	now->Ifactor = (pid_ctl->prev->Ifactor + (Err * tickdiff)) * Ifactor;
	now->Dfactor = ((pid_ctl->prev->Error - Err) / tickdiff) * Pfactor;

	pid_ctl->curr = now;


	return pid_get_curr_pid(pid_ctl) ; // return PWM
}

//for motor to call
float pid_get_curr_pid(pid_control* pid_ctl){
	return (float)round(pid_ctl->curr->Pfactor + pid_ctl->curr->Ifactor); //+ pidframe->Dfactor);
}

void pid_reset_error(pid_control* pid_ctl){
	free(pid_ctl->prev);
	pid_ctl->prev = pid_ctl->curr;
	pid_frame *now = malloc(sizeof(pid_frame));

	now->Error = 0;
	now->Tickdiff = 0;
	now->Pfactor = 0;
	now->Ifactor = 0;
	now->Dfactor = 0;
}

void pid_stall_error(pid_control* pid_ctl){
	free(pid_ctl->prev);
	pid_ctl->prev = pid_ctl->curr;
	pid_frame *now = malloc(sizeof(pid_frame));

	now->Error = pid_ctl->prev->Error;
	now->Tickdiff = pid_ctl->prev->Tickdiff;
	now->Pfactor = pid_ctl->prev->Pfactor;
	now->Ifactor = pid_ctl->prev->Ifactor;
	now->Dfactor = pid_ctl->prev->Dfactor;
}





