/*
 * BNO055.c
 *
 *  Created on: Apr 18, 2024
 *      Author: 08809
 */

#include "BNO055.h"

HAL_StatusTypeDef BNO055_Init(BNO055_Structure *bno, I2C_HandleTypeDef *hi2cx, uint8_t addr, OPRMode mode)
{
	bno->TxBuffer[0] = Page_ID_00;
	bno->TxBuffer[1] = CONFIGMODE;
	bno->TxBuffer[2] = 0x20; // Reset
	bno->TxBuffer[3] = Normal_Mode;
	bno->TxBuffer[4] = mode;

	bno->hi2cx = hi2cx;
	bno->address = BNO055_ADD_H;
	if (addr == 0) bno->address = BNO055_ADD_L;

	HAL_I2C_Mem_Write_DMA(bno->hi2cx, bno->address, PAGE_ID, 1, &bno->TxBuffer[0], 1);

	HAL_Delay(10);
	HAL_I2C_Mem_Read_DMA(bno->hi2cx, bno->address, CHIP_ID, 1, bno->RxBuffer, 1);
	if (bno->RxBuffer[0] != 0xA0) return HAL_ERROR;

	HAL_Delay(10);
	HAL_I2C_Mem_Write_DMA(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->TxBuffer[1], 1);

	HAL_Delay(10);
	HAL_I2C_Mem_Write_DMA(bno->hi2cx, bno->address, SYS_TRIGGER, 1, &bno->TxBuffer[2], 1);

	HAL_Delay(30);
	HAL_I2C_Mem_Read_DMA(bno->hi2cx, bno->address, CHIP_ID, 1, bno->RxBuffer, 1);
	if (bno->RxBuffer[0] != 0xA0) return HAL_ERROR;

	HAL_Delay(10);
	HAL_I2C_Mem_Write_DMA(bno->hi2cx, bno->address, PWR_MODE, 1, &bno->TxBuffer[3], 1);

	HAL_Delay(10);
	HAL_I2C_Mem_Write_DMA(bno->hi2cx, bno->address, PAGE_ID, 1, &bno->TxBuffer[0], 1);

	HAL_Delay(10);
	HAL_I2C_Mem_Write_DMA(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->TxBuffer[4], 1);
	HAL_Delay(20);

	return HAL_OK;
}
