/*
 * speed.c
 *
 *  Created on: Feb 10, 2022
 *      Author: skylivingston
 */

#include "speed.h"

angle_subcontrol* angle_subcontrol_init (pid_control* pid_ctl, vehicleservo_control* vehicleservo_ctl, I2C_HandleTypeDef *hi2c){
	angle_subcontrol* angle_subctl = malloc(sizeof(angle_subcontrol));
	angle_subctl->hi2c = hi2c;

	angle_subctl->pid_ctl = pid_ctl;


	angle_subctl->vehicleservo_ctl= vehicleservo_ctl;

	angle_subctl->prevsample = HAL_GetTick();
	angle_subctl->gyrosum = 0;

	angle_subctl->prevactualangle= 0;
	angle_subctl->curractualangle= 0;
	angle_subctl->requestedangle = 0;


	angle_subctl->leftanglemul = 1;
	angle_subctl->rightanglemul = 1;

	angle_subctl->bais = 0;
	angle_subctl->status = ANGLE_STATUS_NOT_RUNNING;

	return angle_subctl;
}



void angle_subcontrol_set_leftanglemul(angle_subcontrol * angle_subctl, float mul){
	angle_subctl->leftanglemul = mul;
}

void angle_subcontrol_set_rightanglemul(angle_subcontrol * angle_subctl, float mul){
	angle_subctl->rightanglemul = mul;
}

void  angle_subcontrol_set_status(angle_subcontrol * angle_subctl, int status){
	angle_subctl->status = status;
}

void angle_subcontrol_reset_degree(angle_subcontrol* angle_subctl, float preverror){
	angle_subctl->prevsample = HAL_GetTick();
	angle_subctl->gyrosum =preverror;

	angle_subctl->prevactualangle= 0;
	angle_subctl->curractualangle= 0;
	pid_reset_error(angle_subctl->pid_ctl);
}

void angle_subcontrol_set_angle(angle_subcontrol* angle_subctl, float angle){
	angle_subctl->requestedangle = angle;
}


float angle_subcontrol_get_gyro_Z_degree_sample(angle_subcontrol* angle_subctl, uint64_t elapsedticks){
	axises ax;
	icm20948_gyro_read_dps(angle_subctl->hi2c,&ax);
	switch(angle_subctl->status){
		case ANGLE_STATUS_RUNNING_LINE: case ANGLE_STATUS_RUNNING_WHEEL:
			if(ax.z > 0 ) return -((ax.z*angle_subctl->leftanglemul)/1000)*elapsedticks;
			else return -((ax.z*angle_subctl->rightanglemul)/1000)*elapsedticks;
			break;
		case ANGLE_STATUS_NOT_RUNNING:
			if(angle_subctl->bais == 0) angle_subctl->bais = ax.z;
			else angle_subctl->bais = (angle_subctl->bais + ax.z)/2;
			break;
	}
	return 0;
}

float angle_subcontrol_get_gyrosum(angle_subcontrol* angle_subctl){
	return angle_subctl->gyrosum;
}



void angle_subcontrol_run(angle_subcontrol* angle_subctl, int rev, int ressize){

	uint64_t elapsetick = HAL_GetTick() - angle_subctl->prevsample;
	angle_subctl->gyrosum += angle_subcontrol_get_gyro_Z_degree_sample(angle_subctl, elapsetick);
	float relativeangle = angle_subctl->gyrosum;
	float requestedangle = angle_subctl->requestedangle;

	pid_run(angle_subctl->pid_ctl, requestedangle, relativeangle,elapsetick);
	float readjustment_PID = (rev/abs(rev))* pid_get_curr_pid(angle_subctl->pid_ctl);
	angle_subctl->prevactualangle = angle_subctl->curractualangle;
	float newvalue = angle_subctl->curractualangle + readjustment_PID;
	if ( newvalue > -ressize && newvalue < ressize ) {
		angle_subctl->curractualangle = newvalue;
	}
	else {
		pid_reset_error(angle_subctl->pid_ctl);
	}


	switch(angle_subctl->status){
		case ANGLE_STATUS_RUNNING_LINE:
			vehicleservo_set_wheel_angle(angle_subctl->vehicleservo_ctl,angle_subctl->curractualangle);
			break;
		case ANGLE_STATUS_RUNNING_WHEEL:
			vehicleservo_set_wheel_angle(angle_subctl->vehicleservo_ctl,angle_subctl->requestedangle);
			break;
		case ANGLE_STATUS_NOT_RUNNING:
			vehicleservo_set_wheel_angle(angle_subctl->vehicleservo_ctl,0);
			break;
	}

	angle_subctl->prevsample = HAL_GetTick();
}


speed_subcontrol* speed_subcontrol_init (pid_control* pid_ctl, motor_control* motor_ctl, encoder_control* encoder_ctl){
	speed_subcontrol* speed_subctl = malloc(sizeof(speed_subcontrol));
	speed_subctl->motor_ctl = motor_ctl;
	speed_subctl->pid_ctl = pid_ctl;
	speed_subctl->encoder_ctl = encoder_ctl;

	speed_subctl->prevpwm = 0;
	speed_subctl->currpwm = 0;

	speed_subctl->speed = 0;

	return speed_subctl;
}

void speed_subcontrol_reset_PWM(speed_subcontrol* speed_subctl){
	speed_subctl->prevpwm = 0;
	speed_subctl->currpwm = 0;

	pid_reset_error(speed_subctl->pid_ctl);
}

void speed_subcontrol_set_speed(speed_subcontrol* speed_subctl, float speed){
	speed_subctl->speed = speed;
}

void speed_subcontrol_run(speed_subcontrol* speed_subctl){

	uint32_t accurate_tickdiff = (uint32_t) encoder_getTickDiff(speed_subctl->encoder_ctl);
	float accurate_pulsediff = (float)speed_subctl->encoder_ctl->currFrame->pulseDiff;
	float speed = speed_subctl->speed;

	int readjustment_PID = (int) round(pid_run(speed_subctl->pid_ctl, speed, accurate_pulsediff, accurate_tickdiff));
	speed_subctl->prevpwm = speed_subctl->currpwm;
	speed_subctl->currpwm += readjustment_PID;

	if (speed != 0){
		motor_set_pwm(speed_subctl->motor_ctl,speed_subctl->currpwm,MOTOR_PWM_NORMAL);
	}else {
		//smart braking
		if(accurate_pulsediff != 0){
			motor_set_pwm(speed_subctl->motor_ctl,10000,MOTOR_PWM_BRAKE);
		}else{
			motor_set_pwm(speed_subctl->motor_ctl,0,MOTOR_PWM_NORMAL);
		}
	}
}





speed_control* speed_control_init(speed_subcontrol* Lmotor_ctl, speed_subcontrol* Rmotor_ctl, angle_subcontrol* angle_ctl  ,UART_HandleTypeDef *huart){
	speed_control * speed_ctl = malloc(sizeof(speed_control));
	speed_ctl->Lmotor_speed_subctl = Lmotor_ctl;
	speed_ctl->Rmotor_speed_subctl = Rmotor_ctl;
	speed_ctl->angle_subctl = angle_ctl;
	speed_ctl->huart = huart;

	speed_ctl->requestedRevDistance =  0;
	speed_ctl->requestedAngle = 0;
	speed_ctl->requestedIdealMaxSpeed = 8;
	speed_ctl->requestedEncoderTickRate = 100;

	//default value for oled show
	speed_ctl->neededTotalTick = (speed_ctl->requestedRevDistance / speed_ctl->requestedIdealMaxSpeed) * 1.5 * speed_ctl->requestedEncoderTickRate;
	speed_ctl->neededTotalPulse = speed_ctl->requestedRevDistance;

	speed_ctl->LmotorProgressTick = 0;
    speed_ctl->RmotorProgressTick = 0;
    speed_ctl->LmotorProgressPulse = 0;
    speed_ctl->RmotorProgressPulse = 0;

	speed_ctl->runStatus = STATUS_FREE;
	speed_ctl->pathMode = PATH_MODE_CURVE;
	speed_ctl->endpathtilterr = 0;


	speed_ctl->correctionEnable = 1;
	speed_ctl->correctionRange = 5;

	speed_ctl->requestedOrientation = 0;
	speed_ctl->requestedOrientationRange = 10;

	speed_ctl->callbackdistptr = NULL;

	return speed_ctl;
}

void speed_control_toggle_correction(speed_control* speed_ctl, float correctionRange ){
	if (speed_ctl->correctionEnable == 1) speed_ctl->correctionEnable = 0 ;
	else speed_ctl->correctionEnable = 1;
	speed_ctl->correctionRange = correctionRange;
}

void speed_control_set_status(speed_control* speed_ctl, int status){
	speed_ctl->runStatus = status;
}

void speed_control_set_callbackdistptr (speed_control* speed_ctl, double* callbackdistptr){
	speed_ctl->callbackdistptr = callbackdistptr;
}

void speed_control_path_setup(speed_control* speed_ctl, float MaxSpeed ,int EncoderTickRate, double RevDistance, float Angle, float Orientation, float OrientationRange, int pathmode){

	if(speed_ctl->runStatus != STATUS_FREE) return;
	switch(pathmode){
		case PATH_MODE_STR: case PATH_MODE_CURVE:
			speed_ctl->pathMode = pathmode;
			speed_ctl->requestedRevDistance =  RevDistance;
			speed_ctl->requestedAngle = Angle;
			speed_ctl->requestedIdealMaxSpeed = MaxSpeed;
			speed_ctl->requestedEncoderTickRate = EncoderTickRate;
			speed_ctl->requestedOrientation = Orientation;
			speed_ctl->requestedOrientationRange = OrientationRange;
			break;
	}
}

void speed_control_path_start(speed_control* speed_ctl){

	if(speed_ctl->runStatus != STATUS_FREE) return;

	speed_ctl->runStatus = STATUS_PATH_RUNNING;

	//reset pwm value
	speed_subcontrol_reset_PWM(speed_ctl->Lmotor_speed_subctl);
	speed_subcontrol_reset_PWM(speed_ctl->Rmotor_speed_subctl);

	angle_subcontrol_reset_degree(speed_ctl->angle_subctl,speed_ctl->endpathtilterr);

	speed_ctl->neededTotalTick = (speed_ctl->requestedRevDistance / speed_ctl->requestedIdealMaxSpeed) * 1.5 * speed_ctl->requestedEncoderTickRate;
	speed_ctl->neededTotalPulse = speed_ctl->requestedRevDistance;

	speed_ctl->LmotorProgressTick = 0;
    speed_ctl->RmotorProgressTick = 0;
    speed_ctl->LmotorProgressPulse = 0;
    speed_ctl->RmotorProgressPulse = 0;

};

//for internal use
void speed_control_path_end(speed_control* speed_ctl){
	if(speed_ctl->runStatus != STATUS_PATH_RUNNING) return;

	speed_ctl->runStatus = STATUS_FREE;
	angle_subcontrol_set_status(speed_ctl->angle_subctl,ANGLE_STATUS_NOT_RUNNING);

	speed_control_path_return(speed_ctl, speed_ctl->callbackdistptr);

}



//for external use
void speed_control_path_break(speed_control* speed_ctl, double* retdist){
	if(speed_ctl->runStatus != STATUS_PATH_RUNNING) return;

	speed_ctl->runStatus = STATUS_FREE;
	angle_subcontrol_set_status(speed_ctl->angle_subctl,ANGLE_STATUS_NOT_RUNNING);

	speed_control_path_return(speed_ctl,retdist);
}

void speed_control_path_return(speed_control* speed_ctl, double* retdist){

	double distance = (double)(((double)speed_ctl->LmotorProgressTick + (double)speed_ctl->RmotorProgressTick)/2);
//	char str1[10] = "I";
//	sprintf(str1[1], "%lu", distance);
//
//	HAL_UART_Transmit(speed_ctl->huart,(uint8_t *)str1, 10, 0xFFFF);
//	/* return rpi infomation*/

	*retdist = distance;


}


float speed_control_cal_idealspeed_tick(speed_control* speed_ctl, float progress){
//	float onethird = 1/3; float twothird = 2/3;
//	if (progress <= 0 && progress >= 1) return 0;
//	if (progress > 0  && progress <= onethird) return (progress / onethird) * speed_ctl->requestedIdealMaxSpeed;
//	if (progress > onethird   && progress <= twothird) return speed_ctl->requestedIdealMaxSpeed;
//	if (progress > twothird  && progress < 1) return (1 - ((progress - twothird) / onethird)) * speed_ctl->requestedIdealMaxSpeed;

	return speed_ctl->requestedIdealMaxSpeed;
}

float speed_control_cal_idealspeed_pulse(speed_control* speed_ctl, float progress){
	if (progress <= 0) return 1;
	if (progress > 0.00  && progress <= 0.25) return (progress / 0.25) * speed_ctl->requestedIdealMaxSpeed;
	if (progress > 0.25  && progress <= 0.75) return speed_ctl->requestedIdealMaxSpeed;
	if (progress > 0.75  && progress <  1.00) return (1 - ((progress - 0.75) / 0.25)) * speed_ctl->requestedIdealMaxSpeed;
	if (progress >= 1) return 0;
	return 0;
//	return speed_ctl->requestedIdealMaxSpeed;
}

float speed_control_get_tickprogress_L(speed_control* speed_ctl){
	float difftick  = (float) diff_get_Lmotor_values(speed_ctl->requestedAngle, speed_ctl->neededTotalTick);
	return (float)speed_ctl->LmotorProgressTick / (float)difftick;
}

float speed_control_get_tickprogress_R(speed_control* speed_ctl){
	float difftick  = (float) diff_get_Rmotor_values(speed_ctl->requestedAngle, speed_ctl->neededTotalTick);
	return (float)speed_ctl->RmotorProgressTick / (float)difftick;
}

float speed_control_get_pulseprogress_L(speed_control* speed_ctl){
	float diffpulse = (float) diff_get_Lmotor_values(speed_ctl->requestedAngle, speed_ctl->neededTotalPulse);
	return (float)speed_ctl->LmotorProgressPulse / (float)diffpulse;
}

float speed_control_get_pulseprogress_R(speed_control* speed_ctl){
	float diffpulse  = (float) diff_get_Rmotor_values(speed_ctl->requestedAngle, speed_ctl->neededTotalPulse);
	return (float)speed_ctl->RmotorProgressPulse / (float)diffpulse;
}


void speed_control_run(speed_control* speed_ctl){

	speed_subcontrol* Lmotor_ctl = speed_ctl->Lmotor_speed_subctl;
	speed_subcontrol* Rmotor_ctl = speed_ctl->Rmotor_speed_subctl;
	angle_subcontrol* angle_ctl = speed_ctl->angle_subctl;

	encoder_run(Lmotor_ctl->encoder_ctl,speed_ctl->requestedEncoderTickRate,ENCODER_DIRECTION_NATURAL);
	encoder_run(Rmotor_ctl->encoder_ctl,speed_ctl->requestedEncoderTickRate,ENCODER_DIRECTION_FLIPPED);

	float ltickprogress = 0; float lspeedtick = 0; float lpulseprogress = 0; float lspeedpulse = 0;
	float rtickprogress = 0; float rspeedtick = 0; float rpulseprogress = 0; float rspeedpulse = 0;
	float ldiffspeed = 0; float rdiffspeed = 0;

	if(speed_ctl->runStatus != STATUS_FREE) {

	//	float lerror = 0; float rerror = 0;

		//Tick based Information
		speed_ctl->LmotorProgressTick  = speed_ctl->LmotorProgressTick + encoder_getTickDiff(Lmotor_ctl->encoder_ctl);
		speed_ctl->RmotorProgressTick  = speed_ctl->RmotorProgressTick + encoder_getTickDiff(Rmotor_ctl->encoder_ctl);

		ltickprogress =  speed_control_get_tickprogress_L(speed_ctl);
		rtickprogress =  speed_control_get_tickprogress_R(speed_ctl);

		lspeedtick = speed_control_cal_idealspeed_tick(speed_ctl,ltickprogress);
		rspeedtick = speed_control_cal_idealspeed_tick(speed_ctl,rtickprogress);

//		ldiffspeed = diff_get_Lmotor_values(speed_ctl->requestedAngle,lspeedtick);
//		rdiffspeed = diff_get_Rmotor_values(speed_ctl->requestedAngle,rspeedtick);

		//pulse based information
		speed_ctl->LmotorProgressPulse  = speed_ctl->LmotorProgressPulse + abs(Lmotor_ctl->encoder_ctl->currFrame->pulseDiff);
		speed_ctl->RmotorProgressPulse  = speed_ctl->RmotorProgressPulse + abs(Rmotor_ctl->encoder_ctl->currFrame->pulseDiff);

		lpulseprogress =  speed_control_get_pulseprogress_L(speed_ctl);
		rpulseprogress =  speed_control_get_pulseprogress_R(speed_ctl);

//		lspeedpulse = speed_control_cal_idealspeed_pulse(speed_ctl,lpulseprogress);
//		rspeedpulse = speed_control_cal_idealspeed_pulse(speed_ctl,rpulseprogress);

		ldiffspeed = diff_get_Lmotor_values(speed_ctl->requestedAngle,lspeedtick);
		rdiffspeed = diff_get_Rmotor_values(speed_ctl->requestedAngle,rspeedtick);

	}

	float correctionrange = speed_ctl->correctionRange; int correctionenable = speed_ctl->correctionEnable; int needscorrection = 0;
	float orientation = speed_ctl->requestedOrientation; float orientationrange = speed_ctl->requestedOrientationRange; int orientationdirection = 0;
	float lowerOrientation = abs(orientation) - abs(orientationrange);

	switch(speed_ctl->runStatus){


		case STATUS_PATH_RUNNING:

			switch(speed_ctl->pathMode){
				case PATH_MODE_CURVE:
					//angle_subcontrol_set_status(speed_ctl->angle_subctl,ANGLE_STATUS_RUNNING_WHEEL);
					angle_subcontrol_set_status(speed_ctl->angle_subctl,ANGLE_STATUS_RUNNING_WHEEL);
					angle_subcontrol_set_angle(angle_ctl,speed_ctl->requestedAngle);


					//forward Right 90 & reverse left 90
					if       ((speed_ctl->requestedAngle > 0 && speed_ctl->requestedIdealMaxSpeed > 0)
					      ||  (speed_ctl->requestedAngle < 0 && speed_ctl->requestedIdealMaxSpeed < 0)){
						orientationdirection = 1;
					//forward left 90 & reverse right 90
					}else if ((speed_ctl->requestedAngle < 0 && speed_ctl->requestedIdealMaxSpeed > 0)
						  ||  (speed_ctl->requestedAngle > 0 && speed_ctl->requestedIdealMaxSpeed < 0)){
						orientationdirection = -1;
					}

					if (abs(orientationdirection)){

						if((lpulseprogress < 1 && rpulseprogress < 1)){
							speed_subcontrol_set_speed(Lmotor_ctl,ldiffspeed/1.5);
							speed_subcontrol_set_speed(Rmotor_ctl,rdiffspeed/1.5);
						}else{

							if(abs(angle_subcontrol_get_gyrosum(angle_ctl)) < (lowerOrientation)) needscorrection = 1;
							else needscorrection = 0;

							if(abs(angle_subcontrol_get_gyrosum(angle_ctl)) < (lowerOrientation) && correctionenable && needscorrection){
								speed_subcontrol_set_speed(Lmotor_ctl,ldiffspeed/2);
								speed_subcontrol_set_speed(Rmotor_ctl,rdiffspeed/2);

							}else{
								speed_control_path_end(speed_ctl);
								if ( correctionenable == 0 || needscorrection == 0) speed_ctl->endpathtilterr = 0; // use encoder to turn and perfect then angle is set to 0
								else speed_ctl->endpathtilterr = angle_subcontrol_get_gyrosum(angle_ctl) - (orientationdirection*lowerOrientation);  // if we use gyro then get error
							}
						}
					}

					break;
				case PATH_MODE_STR:
					if (correctionenable== 0){
						angle_subcontrol_set_status(speed_ctl->angle_subctl,ANGLE_STATUS_RUNNING_WHEEL);
					}else{
						angle_subcontrol_set_status(speed_ctl->angle_subctl,ANGLE_STATUS_RUNNING_LINE);
					}

					angle_subcontrol_set_angle(angle_ctl,speed_ctl->requestedAngle);
					if((lpulseprogress < 1 && rpulseprogress < 1)){
						speed_subcontrol_set_speed(Lmotor_ctl,ldiffspeed);
						speed_subcontrol_set_speed(Rmotor_ctl,rdiffspeed);
					}else{
					//progress is completed set status to free
						speed_control_path_end(speed_ctl);
						speed_ctl->endpathtilterr = 0;
					}
					break;
			}
			break;

		case STATUS_FREE:
			// set 0 if there is no command or command is executed
			angle_subcontrol_set_status(speed_ctl->angle_subctl,ANGLE_STATUS_NOT_RUNNING);
			angle_subcontrol_set_angle(angle_ctl,0);

			speed_subcontrol_set_speed(Lmotor_ctl,0);
			speed_subcontrol_set_speed(Rmotor_ctl,0);

			break;
	}

	angle_subcontrol_run(angle_ctl,speed_ctl->requestedIdealMaxSpeed, correctionrange);
	speed_subcontrol_run(Lmotor_ctl);
	speed_subcontrol_run(Rmotor_ctl);

}




