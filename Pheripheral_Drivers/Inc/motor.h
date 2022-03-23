/*
 * motor.h
 *
 *  Created on: Jan 27, 2022
 *      Author: ytan196
 */
#include "main.h"
#include "math.h"
#include "stdlib.h"
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MOTOR_BRAKE 2
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD -1
#define MOTOR_FREE 0


#define MOTOR_PWM_BRAKE 1
#define MOTOR_PWM_NORMAL 0

/* Sub Functions */

typedef struct{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
}motor_control_pin;

typedef struct{
	motor_control_pin pin1;
	motor_control_pin pin2;
	TIM_HandleTypeDef *timer;
	uint32_t channel;
}motor_control;

motor_control* motor_init(TIM_HandleTypeDef *htim, uint32_t Channel, motor_control_pin pin1, motor_control_pin pin2 );

void motor_set_pwm(motor_control *motor_ctl, int pwm_value, int brake); // 100 to -100
void _motor_set_pin(motor_control *motor_ctl, int motor_mode);

#endif /* INC_MOTOR_H_ */
