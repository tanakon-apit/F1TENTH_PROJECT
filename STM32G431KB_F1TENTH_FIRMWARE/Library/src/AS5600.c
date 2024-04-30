/*
 * AS5600.c
 *
 *  Created on: Apr 18, 2024
 *      Author: tucha
 */

#include <AS5600.h>

HAL_StatusTypeDef AS5600_Init(AS5600_Structure *as5600, I2C_HandleTypeDef *hi2cx, Angle_Unit unit)
{
	as5600->hi2cx = hi2cx;
	if (HAL_I2C_Mem_Read(as5600->hi2cx, AS5600_ADD, AS5600_ANGLE, 1, as5600->RxBuffer, 2, 10) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (unit == Radian) as5600->gain = M_2_PI / 4095.0;
	else as5600->gain = 360.0 / 4095.0;

	int16_t raw_ang = (((int16_t)as5600->RxBuffer[0])|((int16_t)as5600->RxBuffer[1] << 8))&0x0FFF;
	as5600->ang = raw_ang * as5600->gain;
	return HAL_OK;
}

float AS5600_Read_Absolute_DMA(AS5600_Structure *as5600)
{
	int16_t raw_ang = (((int16_t)as5600->RxBuffer[0])|((int16_t)as5600->RxBuffer[1] << 8))&0x0FFF;

	HAL_I2C_Mem_Read_DMA(as5600->hi2cx, AS5600_ADD, AS5600_ANGLE, 1, as5600->RxBuffer, 2);

	return raw_ang * as5600->gain;
}

double AS5600_Read_Increment_DMA(AS5600_Structure *as5600)
{
	int16_t raw_ang = (((int16_t)as5600->RxBuffer[0])|((int16_t)as5600->RxBuffer[1] << 8))&0x0FFF;

	HAL_I2C_Mem_Read_DMA(as5600->hi2cx, AS5600_ADD, AS5600_ANGLE, 1, as5600->RxBuffer, 2);

	double ang = raw_ang * as5600->gain;
	float delta_ang = ang - as5600->ang;

	if (delta_ang < -M_PI) delta_ang += M_2_PI;
	else if (delta_ang > M_PI) delta_ang -= M_2_PI;

	as5600->ang += delta_ang;
	return as5600->ang;
}

void AS5600_Reset_Increment(AS5600_Structure *as5600)
{
	int16_t raw_ang = (((int16_t)as5600->RxBuffer[0])|((int16_t)as5600->RxBuffer[1] << 8))&0x0FFF;

	as5600->ang = raw_ang * as5600->gain;
}
