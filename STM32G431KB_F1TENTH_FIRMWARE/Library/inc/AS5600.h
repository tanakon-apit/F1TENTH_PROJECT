/*
 * AS5600.h
 *
 *  Created on: Apr 30, 2024
 *      Author: 08809
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stm32g4xx_hal.h"
#include "math.h"

#define AS5600_ADD 0x36 << 1
#define AS5600_ANGLE 0x0E

typedef enum {
	degree,
	radian
}Angle_Unit;

typedef struct {
	I2C_HandleTypeDef *hi2cx;
	HAL_DMA_StateTypeDef flag;
	uint8_t DataBuffer[2];
	float gain;
	float abs_angle;
	double inc_angle;
}AS5600_Structure;

HAL_StatusTypeDef AS5600_Init(AS5600_Structure *as5600, I2C_HandleTypeDef *hi2cx, Angle_Unit unit);

float AS5600_Read_Absolute_DMA(AS5600_Structure *as5600);

double AS5600_Read_Increment_DMA(AS5600_Structure *as5600);

void AS5600_Reset(AS5600_Structure *as5600);

#endif /* INC_AS5600_H_ */
