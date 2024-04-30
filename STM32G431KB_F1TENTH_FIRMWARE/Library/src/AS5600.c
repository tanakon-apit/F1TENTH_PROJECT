/*
 * AS5600.c
 *
 *  Created on: Apr 30, 2024
 *      Author: 08809
 */

#include "AS5600.h"

HAL_StatusTypeDef AS5600_Init(AS5600_Structure *as5600, I2C_HandleTypeDef *hi2cx, Angle_Unit unit)
{
	as5600->hi2cx = hi2cx;
	as5600->angle = 0.0;
	if (unit == degree) as5600->gain = 360.0 / 4095.0;
	else as5600->gain = M_2_PI / 4065.0;
	if (HAL_I2C_Mem_Read(as5600->hi2cx, AS5600_ADD, AS5600_ANGLE, 1, as5600->DataBuffer, 2, 10) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

float AS5600_Read_Absolute_DMA(AS5600_Structure *as5600)
{
	int16_t raw_ang;
	raw_ang = ((int16_t) as5600->DataBuffer[0]) | (((int16_t) as5600->DataBuffer[1]) << 8);

	HAL_I2C_Mem_Read_DMA(as5600->hi2cx, AS5600_ADD, AS5600_ANGLE, 1, as5600->DataBuffer, 2);

	return raw_ang * as5600->gain;
}
