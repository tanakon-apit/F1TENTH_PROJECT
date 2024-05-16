///*
// * User_function.h
// *
// *  Created on: Apr 17, 2024
// *      Author: kwan
// */
//
//#ifndef INC_USER_FUNCTION_H_
//#define INC_USER_FUNCTION_H_
//
//#include "stm32g4xx_hal.h"
//
//typedef struct {
//	TIM_HandleTypeDef *htimx;
//	uint32_t channelx;
//	GPIO_TypeDef *port;
//	uint16_t pin;
//	float gain;
//}MotorDriver_Structure;
//
//typedef enum {
//	False = 0,
//	True
//}bool;
//
//uint8_t MotorDriver_Init(
//		MotorDriver_Structure *Mx,
//		float cpu_freq,
//		float pwm_freq,
//		bool isCHN
//);
//
//void MotorDriver_Write(
//		MotorDriver_Structure *Mx,
//		float duty,
//		bool reverse
//);
//
//
//#endif /* INC_USER_FUNCTION_H_ */