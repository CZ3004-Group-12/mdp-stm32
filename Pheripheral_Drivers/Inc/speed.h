/*
 * speed.h
 *
 *  Created on: Feb 10, 2022
 *      Author: skylivingston
 */

#ifndef INC_SPEED_H_
#define INC_SPEED_H_
#include "stdlib.h"
#include "main.h"

#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "vehicleservo.h"
#include "icm20948.h"

#include "diff.h"

//#define SPEED_STATUS_FREE 0
//#define SPEED_STATUS_BLOCK 1

#define STATUS_FREE  0
#define STATUS_PATH_RUNNING  1


#define PATH_MODE_CURVE 1
#define PATH_MODE_STR 2

#define ANGLE_STATUS_NOT_RUNNING 0
#define ANGLE_STATUS_RUNNING_LINE 1
#define ANGLE_STATUS_RUNNING_WHEEL 2

typedef struct{
	motor_control* motor_ctl;
	pid_control* pid_ctl;
	encoder_control* encoder_ctl;

	int prevpwm;
	int currpwm;

	float speed;

}speed_subcontrol;


typedef struct{
	vehicleservo_control *vehicleservo_ctl;
	I2C_HandleTypeDef *hi2c;
	pid_control* pid_ctl;


	uint64_t prevsample;

	float gyrosum;

	float curractualangle;
	float prevactualangle;

	float requestedangle;

	float leftanglemul;
	float rightanglemul;

	float bais;
	int status;

}angle_subcontrol;

typedef struct{
	speed_subcontrol *Lmotor_speed_subctl;
	speed_subcontrol *Rmotor_speed_subctl;
	angle_subcontrol *angle_subctl;
	UART_HandleTypeDef *huart;


	//pre-path parameter
	double requestedRevDistance; //requested distance in wheel rev
	float requestedIdealMaxSpeed;
	float requestedAngle;

	float requestedOrientation;
	float requestedOrientationRange;



	int requestedEncoderTickRate;
	float tilterror;

	int runStatus;
	int pathMode;

	//generated pre-path paramenter
	uint64_t neededTotalTick;
	uint64_t neededTotalPulse;

	//path parameter
	uint64_t LmotorProgressTick;
	uint64_t RmotorProgressTick;
	uint64_t LmotorProgressPulse;
	uint64_t RmotorProgressPulse;


	float endpathtilterr;

    int correctionEnable;
    int correctionRange;

    double *callbackdistptr;

}speed_control;



angle_subcontrol* angle_subcontrol_init (pid_control* pid_ctl,vehicleservo_control* vehicleservo_ctl ,I2C_HandleTypeDef *hi2c);

void angle_subcontrol_set_leftanglemul(angle_subcontrol * angle_subctl, float mul);


void angle_subcontrol_set_rightanglemul(angle_subcontrol * angle_subctl, float mul);

void  angle_subcontrol_set_status(angle_subcontrol * angle_subctl, int status);

void angle_subcontrol_reset_degree(angle_subcontrol* angle_subctl, float preverror);

void angle_subcontrol_set_angle(angle_subcontrol* angle_subctl, float angle);

float angle_subcontrol_get_gyro_Z_degree_sample(angle_subcontrol* angle_subctl, uint64_t elapsedticks);

float angle_subcontrol_get_gyrosum(angle_subcontrol* angle_subctl);

void angle_subcontrol_run(angle_subcontrol* angle_subctl, int rev, int ressize);


speed_subcontrol* speed_subcontrol_init (pid_control* pid_ctl, motor_control* motor_ctl, encoder_control* encoder_ctl);

void speed_subcontrol_reset_PWM(speed_subcontrol* speed_subctl);

void speed_subcontrol_set_speed(speed_subcontrol* speed_subctl, float speed);



void speed_subcontrol_run(speed_subcontrol* speed_subctl);


speed_control* speed_control_init(speed_subcontrol* Lmotor_ctl, speed_subcontrol* Rmotor_ctl, angle_subcontrol* angle_ctl  ,UART_HandleTypeDef *huart);

void speed_control_toggle_correction(speed_control* speed_ctl,float correctionRange);

void speed_control_set_callbackdistptr (speed_control* speed_ctl, double* callbackdistptr);

void speed_control_path_setup(speed_control* speed_ctl, float MaxSpeed ,int EncoderTickRate, double RevDistance, float Angle, float Orientation, float OrientationRange, int pathmode);

void speed_control_path_start(speed_control* speed_ctl);

void speed_control_path_end(speed_control* speed_ctl);

void speed_control_path_break(speed_control* speed_ctl, double* retdist);

void speed_control_path_return(speed_control* speed_ctl, double* retdist);

float speed_control_get_tickprogress_L(speed_control* speed_ctl);

float speed_control_get_tickprogress_R(speed_control* speed_ctl);

float speed_control_get_pulseprogress_L(speed_control* speed_ctl);

float speed_control_get_pulseprogress_R(speed_control* speed_ctl);

float speed_control_cal_idealspeed_tick(speed_control* speed_ctl, float progress);

float speed_control_cal_idealspeed_pulse(speed_control* speed_ctl, float progress);

void speed_control_run(speed_control* speed_ctl);



#endif /* INC_SPEED_H_ */
