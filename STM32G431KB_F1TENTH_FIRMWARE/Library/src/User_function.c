///*
// * User_function.c
// *
// *  Created on: Apr 17, 2024
// *      Author: kwan
// */
//
//#include "User_function.h"
//
//uint8_t MotorDriver_Init(
//		MotorDriver_Structure *Mx,
//		float cpu_freq,
//		float pwm_freq,
//		bool isCHN
//)
//{
//	HAL_TIM_Base_Start(Mx->htimx);
//	if (isCHN) HAL_TIMEx_PWMN_Start(Mx->htimx, Mx->channelx);
//	else HAL_TIM_PWM_Start(Mx->htimx, Mx->channelx);
//
//	if (pwm_freq == 0)
//	{
//		__HAL_TIM_SET_COMPARE(Mx->htimx, Mx->channelx, 0);
//		return -1;
//	} else if (pwm_freq >= cpu_freq / 2.0) return -1;
//	uint32_t period_cyc = (uint32_t) (cpu_freq / pwm_freq);
//	uint16_t prescaler = (uint16_t) (period_cyc / 65535 + 1);
//	uint16_t overflow = (uint16_t) ((period_cyc + (prescaler / 2)) / prescaler);
//	__HAL_TIM_SET_PRESCALER(Mx->htimx, prescaler);
//	__HAL_TIM_SET_AUTORELOAD(Mx->htimx, overflow);
//	Mx->gain = overflow / 100.0;
//	return 0;
//}
//
//void MotorDriver_Write(
//		MotorDriver_Structure *Mx,
//		float duty,
//		bool reverse
//)
//{
//	GPIO_PinState dir_state = GPIO_PIN_RESET;
//	float pwm_duty = Mx->gain * duty;
//	if (duty < 0)
//	{
//		pwm_duty *= -1;
//		dir_state = !dir_state;
//	}
//	if (reverse) dir_state = !dir_state;
//	__HAL_TIM_SET_COMPARE(Mx->htimx, Mx->htimx, (uint16_t) (pwm_duty));
//	HAL_GPIO_WritePin(Mx->port, Mx->pin, dir_state);
//}
//