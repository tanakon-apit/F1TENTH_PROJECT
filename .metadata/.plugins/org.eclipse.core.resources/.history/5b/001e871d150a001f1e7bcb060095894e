/*
 * AS5600.h
 *
 *  Created on: Apr 18, 2024
 *      Author: tucha
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stm32g4xx_hal.h"
#include "math.h"

#define AS5600_ADD 0x36 << 1

#define AS5600_ANGLE 0x0E
#define AS5600_SW_DIRECTION_PIN 255
#define AS5600_CLOCK_WISE         0
#define AS5600_COUNTERCLOCK_WISE  1

typedef enum {
	Degree,
	Radian
}Angle_Unit;

typedef struct {
	I2C_HandleTypeDef *hi2cx;
	HAL_StatusTypeDef flag;
	uint8_t RxBuffer[2];
	float gain;
	double ang;
}AS5600_Structure;

HAL_StatusTypeDef AS5600_Init(AS5600_Structure *as5600, I2C_HandleTypeDef *hi2cx, Angle_Unit unit);

float AS5600_Read_Absolute_DMA(AS5600_Structure *as5600);

double AS5600_Read_Increment_DMA(AS5600_Structure *as5600);

void AS5600_Reset_Increment(AS5600_Structure *as5600);

#endif /* INC_AS5600_H_ */
