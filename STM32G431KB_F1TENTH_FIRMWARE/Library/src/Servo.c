/*
 * Servo.c
 *
 *  Created on: Apr 17, 2024
 *      Author: kwan
 */

#include "Servo.h"

uint8_t Servo_Init(
		Servo_Structure *Sx,
		float cpu_freq,
		bool isCHN
)
{
	HAL_TIM_Base_Start(Sx->htimx);
	if (isCHN) HAL_TIMEx_PWMN_Start(Sx->htimx, Sx->channelx);
	else HAL_TIM_PWM_Start(Sx->htimx, Sx->channelx);

	if (50.0 >= cpu_freq / 2.0) return -1;
	uint32_t period_cyc = (uint32_t) (cpu_freq / 50.0);
	uint16_t prescaler = (uint16_t) (period_cyc / 65535 + 1);
	uint16_t overflow = (uint16_t) ((period_cyc + (prescaler / 2)) / prescaler);
	__HAL_TIM_SET_PRESCALER(Sx->htimx, prescaler);
	__HAL_TIM_SET_AUTORELOAD(Sx->htimx, overflow);
	Sx->gain = overflow / 20.0;
	return 0;
}

void RC_Set_Input_Range(
		Servo_Structure *Sx,
		float _min, //degree
		float _max
)
{
	float m = (2.5 - 0.5) / (_max - _min);
	float c = 0.5 - (m * _min);

	Sx->offset = Sx->gain * c;
	Sx->gain *= m;
}

void RC_Signal_Write(
		Servo_Structure *Sx,
		int32_t cmd
)
{
	float pwm = (Sx->gain * cmd) + Sx->offset;
	__HAL_TIM_SET_COMPARE(Sx->htimx, Sx->channelx, (uint16_t) pwm);
}





