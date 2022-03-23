/*
 * motor.c
 *
 *  Created on: Jan 27, 2022
 *      Author: ytan196
 */

#include "motor.h"
motor_control* motor_init(TIM_HandleTypeDef *htim, uint32_t channel, motor_control_pin pin1, motor_control_pin pin2 )
{
	motor_control *motor_ctl= malloc(sizeof(motor_control));
	motor_ctl->pin1 = pin1;
	motor_ctl->pin2 = pin2;
	motor_ctl->timer = htim;
	motor_ctl->channel = channel;
	HAL_TIM_PWM_Start(htim, channel);
	return motor_ctl;
}

void motor_set_pwm(motor_control *motor_ctl, int pwm_value, int brake)
{
	int modifiedvalue = 0;
	if (brake){
		_motor_set_pin(motor_ctl, MOTOR_BRAKE);
		if (pwm_value <= 10000) modifiedvalue = abs(pwm_value);
		else modifiedvalue = 10000;
	}else if (pwm_value > 0){
		_motor_set_pin(motor_ctl, MOTOR_FORWARD);
		if (pwm_value <= 10000) modifiedvalue = abs(pwm_value);
		else modifiedvalue = 10000;
	}else if (pwm_value < 0){
		_motor_set_pin(motor_ctl, MOTOR_BACKWARD);
		if (pwm_value >= -10000) modifiedvalue = abs(pwm_value);
		else modifiedvalue = 10000;
	}else {
		_motor_set_pin(motor_ctl, MOTOR_FREE);
		modifiedvalue = 0;
	}
	switch(motor_ctl->channel){
		case TIM_CHANNEL_1:
			motor_ctl->timer->Instance->CCR1 = modifiedvalue;
		break;
		case TIM_CHANNEL_2:
			motor_ctl->timer->Instance->CCR2 = modifiedvalue;
		break;
		case TIM_CHANNEL_3:
			motor_ctl->timer->Instance->CCR3 = modifiedvalue;
		break;
		case TIM_CHANNEL_4:
			motor_ctl->timer->Instance->CCR4 = modifiedvalue;
		break;
	}


	// 100 to -100
}
void _motor_set_pin(motor_control *motor_ctl, int motor_mode)
{
	switch (motor_mode){
	case MOTOR_FORWARD:
		HAL_GPIO_WritePin(motor_ctl->pin1.GPIOx, motor_ctl->pin1.GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor_ctl->pin2.GPIOx, motor_ctl->pin2.GPIO_Pin, GPIO_PIN_RESET);
	break;

	case MOTOR_BACKWARD:
		HAL_GPIO_WritePin(motor_ctl->pin1.GPIOx, motor_ctl->pin1.GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor_ctl->pin2.GPIOx, motor_ctl->pin2.GPIO_Pin, GPIO_PIN_SET);
	break;

	case MOTOR_FREE:
		HAL_GPIO_WritePin(motor_ctl->pin1.GPIOx, motor_ctl->pin1.GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor_ctl->pin2.GPIOx, motor_ctl->pin2.GPIO_Pin, GPIO_PIN_RESET);
	break;
	case MOTOR_BRAKE:
		HAL_GPIO_WritePin(motor_ctl->pin1.GPIOx, motor_ctl->pin1.GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor_ctl->pin2.GPIOx, motor_ctl->pin2.GPIO_Pin, GPIO_PIN_SET);
	break;
	}
};



