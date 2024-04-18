/*
 * BNO055.c
 *
 *  Created on: Apr 18, 2024
 *      Author: 08809
 */

#include "BNO055.h"

HAL_StatusTypeDef BNO055_Init(BNO055_Structure *bno, I2C_HandleTypeDef *hi2cx, uint8_t addr, OPRMode mode)
{
	uint8_t txbuffer;
	uint8_t rxbuffer;

	bno->hi2cx = hi2cx;
	bno->address = BNO055_ADD_H;
	if (addr == 0) bno->address = BNO055_ADD_L;

	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CHIP_ID, 1, &rxbuffer, 1, 10);
	if (rxbuffer != 0xA0) {
		HAL_Delay(1000);
		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CHIP_ID, 1, &rxbuffer, 1, 10);
		if (rxbuffer != 0xA0) return HAL_ERROR;
	}

	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->RxBuffer[8], 1, 10);

	txbuffer = CONFIGMODE;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &txbuffer, 1, 10);
	HAL_Delay(10);

	txbuffer = 0x20;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, SYS_TRIGGER, 1, &txbuffer, 1, 10);
	HAL_Delay(30);

	do {
		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CHIP_ID, 1, &rxbuffer, 1, 10);
	} while (rxbuffer != 0xA0);
	HAL_Delay(50);

	txbuffer = Normal_Mode;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, PWR_MODE, 1, &txbuffer, 1, 10);
	HAL_Delay(10);

	txbuffer = Page_ID_00;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, PAGE_ID, 1, &txbuffer, 1, 10);
	HAL_Delay(10);

	txbuffer = 0x00;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, SYS_TRIGGER, 1, &txbuffer, 1, 10);
	HAL_Delay(10);

	bno->mode = mode;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
	HAL_Delay(30);

	return HAL_OK;
}

void BNO055_Read(BNO055_Structure *bno, Vector_Type type)
{
	uint8_t read_mode;
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != bno->mode) {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
	}

	if (type != QUATERNION) {
		uint8_t rxbuffer[6];
		uint8_t data_reg;
		switch (type) {
		case ACCELEROMETER:
			data_reg = ACC_DATA_X_LSB;
			break;
		case GYROSCOPE:
			data_reg = GYR_DATA_X_LSB;
			break;
		case MAGNETOMETER:
			data_reg = MAG_DATA_X_LSB;
			break;
		case EULER:
			data_reg = EUL_DATA_X_LSB;
			break;
		case LINEARACCEL:
			data_reg = LIA_DATA_X_LSB;
			break;
		case GRAVITY:
			data_reg = GRV_DATA_X_LSB;
			break;
		default:
		}

		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, data_reg, 1, rxbuffer, 6, 10);

		int16_t x, y, z = 0;

		x  = ((int16_t) rxbuffer[0]) | (((int16_t) rxbuffer[1]) << 8);
		y  = ((int16_t) rxbuffer[2]) | (((int16_t) rxbuffer[3]) << 8);
		z  = ((int16_t) rxbuffer[4]) | (((int16_t) rxbuffer[5]) << 8);

		switch (type) {
		case GYROSCOPE:
			bno->gyro.x = ((double)x) / 16.0;
			bno->gyro.y = ((double)y) / 16.0;
			bno->gyro.z = ((double)z) / 16.0;
			break;
		case MAGNETOMETER:
			bno->mag.x = ((double)x) / 16.0;
			bno->mag.y = ((double)y) / 16.0;
			bno->mag.z = ((double)z) / 16.0;
			break;
		case EULER:
			bno->euler.roll = ((double)x) / 16.0;
			bno->euler.pitch = ((double)y) / 16.0;
			bno->euler.yaw = ((double)z) / 16.0;
			break;
		default:
			bno->accel.x = ((double)x) / 100.0;
			bno->accel.y = ((double)y) / 100.0;
			bno->accel.z = ((double)z) / 100.0;
		}
	} else {
		uint8_t rxbuffer[8];

		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, QUA_DATA_X_LSB, 1, rxbuffer, 8, 10);

		int16_t x, y, z, w = 0;

		x  = ((int16_t) rxbuffer[0]) | (((int16_t) rxbuffer[1]) << 8);
		y  = ((int16_t) rxbuffer[2]) | (((int16_t) rxbuffer[3]) << 8);
		z  = ((int16_t) rxbuffer[4]) | (((int16_t) rxbuffer[5]) << 8);
		w  = ((int16_t) rxbuffer[6]) | (((int16_t) rxbuffer[7]) << 8);

		const double scale = (1.0 / (1 << 14));
		bno->quat.x = x * scale;
		bno->quat.y = y * scale;
		bno->quat.z = z * scale;
		bno->quat.w = w * scale;
	}

}
