/*
 * vehicleservo.c
 *
 *  Created on: 28 Jan 2022
 *      Author: ytan196
 */


//105L-152M-240R

//servo turn L - servo at 125 degree



#include "vehicleservo.h"



double vehicleservo_radian(float degree){
	return (degree/360)*(M_PI*2);
};

double vehicleservo_degree(float radian){
	return (radian/(M_PI*2)) * 360;
};

//assume max R is 0; get most left
double _vehicleservo_max_shaft_range_l(void){
	double h = SERVO_SHAFT;
	double r = SERVO_RADIUS;

	double o = r * sin(M_PI - vehicleservo_radian(SERVO_MAX_L_SERVO_ANGLE));
	return sqrt(h*h - o*o) + sqrt(r*r - o*o);
};
double _vehicleservo_max_shaft_range_r(void){
	double h = SERVO_SHAFT;
	double r = SERVO_RADIUS;

	double o = r * sin(vehicleservo_radian(SERVO_MAX_R_SERVO_ANGLE));
	return sqrt(h*h - o*o) - sqrt(r*r - o*o);
};



double vehicleservo_get_required_shaft_shift(vehicleservo_control* vehicleservo_ctl, double turn_degree){
	double angle_range = SERVO_MAX_R_WHEEL_ANGLE - SERVO_MAX_L_WHEEL_ANGLE ;
	double angle_turn =  turn_degree - SERVO_MAX_L_WHEEL_ANGLE;
	double turn_ratio = (angle_turn/angle_range);
	if(turn_ratio > 1) turn_ratio = 1;
	return (vehicleservo_ctl->max_shaft_range_r + (1 - turn_ratio) * (vehicleservo_ctl->max_shaft_range_l - vehicleservo_ctl->max_shaft_range_r));
};

double vehicleservo_get_required_servo_angle (vehicleservo_control* vehicleservo_ctl, double shaft_shift){
	double h =  SERVO_SHAFT;
	double ss = shaft_shift;
	double  r = SERVO_RADIUS;
    return (M_PI - acos((-(h*h)+(ss*ss)+(r*r))/(2*ss*r)));
};


double vehicleservo_get_required_servo_pwm (vehicleservo_control* vehicleservo_ctl, double servo_degree){
	double angle_range = SERVO_MAX_L_SERVO_ANGLE;
	double angle_turn =  servo_degree;
	double turn_ratio = (angle_turn/angle_range);
	if(turn_ratio > 1) turn_ratio = 1;
	return (SERVO_MAX_L_PWM +  (1- turn_ratio) * (SERVO_MAX_R_PWM - SERVO_MAX_L_PWM)); //SERVO_MAX_L_PWM +  + ((servo_ratio)*(135))
};

double vehicleservo_get_required_servo_pwm_simple(vehicleservo_control* vehicleservo_ctl, double turn_degree){
	 double shaft_shift = vehicleservo_get_required_shaft_shift(vehicleservo_ctl, turn_degree);
	 double servo_angle = vehicleservo_get_required_servo_angle(vehicleservo_ctl, shaft_shift);
	 double val = vehicleservo_get_required_servo_pwm(vehicleservo_ctl,vehicleservo_degree(servo_angle));
	 return val;
};

vehicleservo_control* vehicleservo_init(TIM_HandleTypeDef *htim, uint32_t channel){
	vehicleservo_control *vsc = malloc(sizeof(vehicleservo_control));
	vsc->max_shaft_range_l = _vehicleservo_max_shaft_range_l();
	vsc->max_shaft_range_r = _vehicleservo_max_shaft_range_r();
	vsc->timer = htim;
	vsc->channel= channel;
	HAL_TIM_PWM_Start(htim, channel);

	return vsc;
};

void vehicleservo_set_wheel_angle(vehicleservo_control* vehicleservo_ctl, double turn_degree){
	double pwm_value = vehicleservo_get_required_servo_pwm_simple(vehicleservo_ctl,turn_degree);

	switch(vehicleservo_ctl->channel){
		case TIM_CHANNEL_1:
			vehicleservo_ctl->timer->Instance->CCR1 = pwm_value;
		break;
		case TIM_CHANNEL_2:
			vehicleservo_ctl->timer->Instance->CCR2 = pwm_value;
		break;
		case TIM_CHANNEL_3:
			vehicleservo_ctl->timer->Instance->CCR3 = pwm_value;
		break;
		case TIM_CHANNEL_4:
			vehicleservo_ctl->timer->Instance->CCR4 = pwm_value;
		break;
	}

};
