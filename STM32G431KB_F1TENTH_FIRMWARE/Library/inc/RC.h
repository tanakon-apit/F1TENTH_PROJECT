/*
 * RC.h
 *
 *  Created on: May 15, 2024
 *      Author: 08809
 */

#ifndef INC_RC_H_
#define INC_RC_H_

#include "stdint.h"
#include "stm32g4xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *htimx;
	uint32_t channelx;
	float gain;
	float offset;
}RC_Structure;

typedef enum {
	False = 0,
	True
}bool;

uint8_t RC_Init(RC_Structure *RCx, float cpu_freq, bool isCHN);

void RC_Set_Input_Range(RC_Structure *RCx, float _min, float _max);

void RC_Write(RC_Structure *RCx, int32_t cmd);

#endif /* INC_RC_H_ */
