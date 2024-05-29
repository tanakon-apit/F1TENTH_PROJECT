/*
 * RC.c
 *
 *  Created on: May 15, 2024
 *      Author: 08809
 */

#include "RC.h"

uint8_t RC_Init(RC_Structure *RCx, TIM_HandleTypeDef *htimx, uint32_t channelx, float cpu_freq, bool isCHN)
{
	RCx->htimx = htimx;
	RCx->channelx =channelx;
	HAL_TIM_Base_Start(RCx->htimx);
	if (isCHN) HAL_TIMEx_PWMN_Start(RCx->htimx, RCx->channelx);
	else HAL_TIM_PWM_Start(RCx->htimx, RCx->channelx);

	if (50.0 >= cpu_freq / 2.0) return -1;
	uint32_t period_cyc = (uint32_t) (cpu_freq / 50.0);
	uint16_t prescaler = (uint16_t) (period_cyc / 65535 + 1);
	uint16_t overflow = (uint16_t) ((period_cyc + (prescaler / 2)) / prescaler);
	__HAL_TIM_SET_PRESCALER(RCx->htimx, prescaler);
	__HAL_TIM_SET_AUTORELOAD(RCx->htimx, overflow);
	RCx->gain = overflow / 20.0;
	return 0;
}

void RC_Set_Input_Range(RC_Structure *RCx, float _min, float _max)
{
	float m = (2.5 - 0.5) / (_max - _min);
	float c = 0.5 - (m * _min);

	RCx->offset = RCx->gain * c;
	RCx->gain *= m;
}

void RC_Write(RC_Structure *RCx, float cmd)
{
	float pwm = (RCx->gain * cmd) + RCx->offset;
	__HAL_TIM_SET_COMPARE(RCx->htimx, RCx->channelx, (uint16_t) pwm);
}
