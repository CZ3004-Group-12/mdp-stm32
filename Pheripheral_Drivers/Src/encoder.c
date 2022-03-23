/*
 * encoder.c
 *
 *  Created on: Jan 27, 2022
 *      Author: ytan196
 */
#include "encoder.h"


encoder_control* encoder_init(TIM_HandleTypeDef *tim_x){
	HAL_TIM_Encoder_Start(tim_x,TIM_CHANNEL_ALL);
	encoder_control *ec = malloc(sizeof(encoder_control));
	ec->timer = tim_x;
	encoder_pulse_frame *now = malloc(sizeof(encoder_pulse_frame));
	encoder_pulse_frame *then = malloc(sizeof(encoder_pulse_frame));
	now->pulseCount = __HAL_TIM_GET_COUNTER(tim_x);
	now->pulseDiff = 0;
	now->Tick = HAL_GetTick();
	ec->currFrame = now;
	ec->prevFrame = then;

	return ec;
}

encoder_pulse_frame* encoder_run(encoder_control *encoder_ctl, int delaytick, int flipped ){
	//returns current pulse_frame
	if(HAL_GetTick() - encoder_ctl->currFrame->Tick > delaytick){
		free(encoder_ctl->prevFrame);
		encoder_ctl->prevFrame = encoder_ctl->currFrame;
		encoder_pulse_frame *now = malloc(sizeof(encoder_pulse_frame));
		now->pulseCount = __HAL_TIM_GET_COUNTER(encoder_ctl->timer);
		now->pulseDiff = _encoder_calPulseDiff(encoder_ctl->timer,encoder_ctl->prevFrame->pulseCount,now->pulseCount, flipped);
		now->Tick = HAL_GetTick();

		encoder_ctl->currFrame = now;
	}

	return encoder_ctl->currFrame;
}


int _encoder_calPulseDiff(TIM_HandleTypeDef *tim_x, int cnt1, int cnt2, int flipped)
{
	int diff;
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(tim_x)){
		if (cnt2 <= cnt1) diff = cnt1 - cnt2;
		else diff = (65535 - cnt2) + cnt1;
		return ((flipped)*diff)/30;
	}else{
		if (cnt2 >= cnt1) diff =  cnt2 - cnt1;
		else diff = (65535 - cnt1) + cnt2;
		return - ((flipped)*diff)/30;
	}
	return 0;
}

uint32_t encoder_getTickDiff(encoder_control *encoder_ctl){
	//return encoder_ctl->currFrame->Tick - encoder_ctl->prevFrame->Tick;

	uint32_t curr = encoder_ctl->currFrame->Tick;
	uint32_t prev = encoder_ctl->prevFrame->Tick;
	if (curr >= prev) {
		return curr - prev;
	} else{
		return (16777216 + curr) - prev;
	}



}

//int _Encoders_getTickCountDifference(TIM_HandleTypeDef tim_x, int* cnt1, int* cnt2)
//{
//	int diff;
//	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&tim_x)){
//		if (*cnt2 < *cnt1) diff = *cnt1 - *cnt2;
//		else diff = (65536 - *cnt2) + *cnt1;
//	}else{
//		if (*cnt2 > *cnt1) diff = *cnt2 - *cnt1;
//		else diff = (65536 - *cnt1) + *cnt2;
//	}
//	return diff;
//}


// * void StartEncoderTask(void *argument)
//{
//  /* USER CODE BEGIN StartEncoderTask */
//  /* Infinite loop */
//	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL); // Wheel 1
//	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); // Wheel 2
//	int wheel1prev, wheel2prev, wheel1curr, wheel2curr, wheel1diff, wheel2diff;
//	//char
//	wheel1prev=__HAL_TIM_GET_COUNTER(&htim2);
//	wheel2prev=__HAL_TIM_GET_COUNTER(&htim3);
//	uint32_t tick = HAL_GetTick();
//
//  /* Infinite loop */
//  for(;;)
//  {
//	if(HAL_GetTick() - tick > 100){
//		//comparison logic
//		wheel1curr = __HAL_TIM_GET_COUNTER(&htim2); // get current counter value for comparison with wheel1prev
//		wheel2curr = __HAL_TIM_GET_COUNTER(&htim3); // get current counter value for comparison with wheel2prev
//		wheel1diff =_Encoders_getTickCountDifference(htim2, &wheel1prev, &wheel1curr); // positive diff = motor pushing forward and via visa
//		wheel2diff =_Encoders_getTickCountDifference(htim3, &wheel2prev, &wheel2curr); // positive diff = motor pushing forward and via visa
//		//update state
//		 encoderData.motorLcount = wheel1diff;
//		 encoderData.motorRcount = wheel2diff;
//		 encoderData.Tick = tick;
//		//prep next iteration
//		wheel1prev =__HAL_TIM_GET_COUNTER(&htim2); // update wheel1prev with updated counter for next loop
//		wheel2prev =__HAL_TIM_GET_COUNTER(&htim3); // update wheel2prev with updated counter for next loop
//		tick = HAL_GetTick(); // update tick for comparison in next loop
//	}
//    osDelay(1);
//  }
//  /* USER CODE END StartEncoderTask */
//}
// *
//*/


/*
if (abs(motorData.motorLprevpwm) > 100){
	pid_run(pidData.motorLpid, 60, encoderData.motorLencoder->currFrame->pulseDiff, Ltickdiff);
	motorData.motorLpwm = 100;
}else{
	motorData.motorLpwm = motorData.motorLprevpwm + pid_run(pidData.motorLpid, 60, encoderData.motorLencoder->currFrame->pulseDiff, Ltickdiff);
}

if (abs(motorData.motorRprevpwm)> 100){
	pid_run(pidData.motorRpid, 60, encoderData.motorRencoder->currFrame->pulseDiff, Rtickdiff);
	motorData.motorRpwm = -100;
}else{
	motorData.motorRpwm = motorData.motorRprevpwm + pid_run(pidData.motorRpid, 60, encoderData.motorRencoder->currFrame->pulseDiff, Rtickdiff);
}
*/

