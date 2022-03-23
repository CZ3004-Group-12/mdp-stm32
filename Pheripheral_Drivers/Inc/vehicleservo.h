/*
 * vehicleservo.h
 *
 *  Created on: 28 Jan 2022
 *      Author: ytan196
 */

#ifndef INC_VEHICLESERVO_H_
#define INC_VEHICLESERVO_H_

#include "main.h"
#include "math.h"
#include "stdlib.h"


#define SERVO_RADIUS  20
#define SERVO_SHAFT  60
#define SERVO_MAX_L_WHEEL_ANGLE  -30
#define SERVO_MAX_R_WHEEL_ANGLE  30
#define SERVO_MAX_L_PWM  105
#define SERVO_MAX_R_PWM  240
#define SERVO_MAX_L_SERVO_ANGLE  125
#define SERVO_MAX_R_SERVO_ANGLE  0

typedef struct{
	float max_shaft_range_l;
	float max_shaft_range_r;

	TIM_HandleTypeDef *timer;
	uint32_t channel;

}vehicleservo_control;

double vehicleservo_radian(float degree);
double vehicleservo_degree(float radian);
double _vehicleservo_max_shaft_range_l(void);
double _vehicleservo_max_shaft_range_r(void);

double vehicleservo_get_required_shaft_shift (vehicleservo_control* vehicleservo_ctl, double turn_degree);
double vehicleservo_get_required_servo_angle (vehicleservo_control* vehicleservo_ctl, double shaft_shift);
double vehicleservo_get_required_servo_pwm (vehicleservo_control* vehicleservo_ctl, double servo_degree);

double vehicleservo_get_required_servo_pwm_simple(vehicleservo_control* vehicleservo_ctl, double turn_degree);

vehicleservo_control* vehicleservo_init(TIM_HandleTypeDef *htim, uint32_t channel);
void vehicleservo_set_wheel_angle(vehicleservo_control* vehicleservo_ctl, double turn_degree);
#endif /* INC_VEHICLESERVO_H_ */
