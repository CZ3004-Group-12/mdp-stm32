/*
 * encoder.h
 *
 *  Created on: Jan 27, 2022
 *      Author: ytan196
 */
#include "main.h"
#include "math.h"
#include "stdlib.h"
#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#define ENCODER_DIRECTION_NATURAL 1
#define ENCODER_DIRECTION_FLIPPED -1

//data to be read from encoders
typedef struct {
	int pulseCount;
	int pulseDiff;
	uint32_t Tick;
}encoder_pulse_frame;

typedef struct {
	encoder_pulse_frame *prevFrame;
	encoder_pulse_frame *currFrame;
	TIM_HandleTypeDef *timer;
}encoder_control;




encoder_control* encoder_init(TIM_HandleTypeDef *tim_x);

encoder_pulse_frame* encoder_run(encoder_control *encoder_ctl, int delaytick, int flipped);

int _encoder_calPulseDiff(TIM_HandleTypeDef *tim_x, int cnt1, int cnt2, int flipped);

uint32_t encoder_getTickDiff(encoder_control *encoder_ctl);

#endif /* INC_ENCODER_H_ */
