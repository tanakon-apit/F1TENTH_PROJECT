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

	int16_t raw_ang = (((int16_t)as5600->RxBuffer[1])|((int16_t)as5600->RxBuffer[0] << 8))&0x0FFF;
	as5600->prev_raw_ang = raw_ang;
	as5600->offset = raw_ang * as5600->gain;
	as5600->ang = 0.0;
	return HAL_OK;
}

void AS5600_Read_Absolute_DMA(AS5600_Structure *as5600)
{
	int16_t raw_ang = (((int16_t)as5600->RxBuffer[1])|((int16_t)as5600->RxBuffer[0] << 8))&0x0FFF;

	HAL_I2C_Mem_Read_DMA(as5600->hi2cx, AS5600_ADD, AS5600_ANGLE, 1, as5600->RxBuffer, 2);

	as5600->abs_ang = raw_ang * as5600->gain;
}

void AS5600_Read_Increment_DMA(AS5600_Structure *as5600)
{
	int16_t raw_ang = (((int16_t)as5600->RxBuffer[1])|((int16_t)as5600->RxBuffer[0] << 8))&0x0FFF;

	HAL_I2C_Mem_Read_DMA(as5600->hi2cx, AS5600_ADD, AS5600_ANGLE, 1, as5600->RxBuffer, 2);

	int16_t delta_ang = raw_ang  - as5600->prev_raw_ang;
	as5600->prev_raw_ang = raw_ang;

	if (delta_ang < -2048) as5600->ang += (4095 * as5600->gain);
	else if (delta_ang > 2048) as5600->ang -= (4095 * as5600->gain);

	as5600->inc_ang = (raw_ang * as5600->gain) + as5600->ang - as5600->offset;
}

void AS5600_Reset_Increment(AS5600_Structure *as5600)
{
	int16_t raw_ang = (((int16_t)as5600->RxBuffer[0])|((int16_t)as5600->RxBuffer[1] << 8))&0x0FFF;

	as5600->prev_raw_ang = raw_ang;
	as5600->offset = raw_ang * as5600->gain;
	as5600->ang = 0.0;
}