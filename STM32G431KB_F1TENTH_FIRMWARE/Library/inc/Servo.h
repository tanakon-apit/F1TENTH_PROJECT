/*
 * Servo.h
 *
 *  Created on: Apr 17, 2024
 *      Author: kwan
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stdint.h"
#include "stm32g4xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *htimx;
	uint32_t channelx;
	float gain;
}Servo_Structure;

typedef enum {
	False = 0,
	True
}bool;

uint8_t Servo_Init(
		Servo_Structure *Sx,
		float cpu_freq,
		bool isCHN
);

void RC_Signal_Write(
		Servo_Structure *Sx,
		float pwm
);

float pwm_to_degree(
		float pwm
);

#endif /* INC_SERVO_H_ */
