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
	HAL_Delay(1000);

	bno->flag = HAL_OK;

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
			data_reg = EUL_DATA_HEADING_LSB;
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
		case ACCELEROMETER:
			bno->accel.x = ((double)x) / 100.0;
			bno->accel.y = ((double)y) / 100.0;
			bno->accel.z = ((double)z) / 100.0;
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
			bno->euler.yaw = ((double)x) / 16.0;
			bno->euler.roll = ((double)y) / 16.0;
			bno->euler.pitch = ((double)z) / 16.0;
			break;
		case LINEARACCEL:
			bno->lin_acc.x = ((double)x) / 100.0;
			bno->lin_acc.y = ((double)y) / 100.0;
			bno->lin_acc.z = ((double)z) / 100.0;
		case GRAVITY:
			bno->grav.x = ((double)x) / 100.0;
			bno->grav.y = ((double)y) / 100.0;
			bno->grav.z = ((double)z) / 100.0;
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

void BNO055_Read_DMA(BNO055_Structure *bno, uint8_t fast_mode)
{
	uint8_t read_mode;
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != bno->mode) {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
	}

	int16_t raw_accel_x = ((int16_t) bno->RxBuffer[0]) | (((int16_t) bno->RxBuffer[1]) << 8);
	int16_t raw_accel_y = ((int16_t) bno->RxBuffer[2]) | (((int16_t) bno->RxBuffer[3]) << 8);
	int16_t raw_accel_z = ((int16_t) bno->RxBuffer[4]) | (((int16_t) bno->RxBuffer[5]) << 8);

	int16_t raw_mag_x = ((int16_t) bno->RxBuffer[6]) | (((int16_t) bno->RxBuffer[7]) << 8);
	int16_t raw_mag_y = ((int16_t) bno->RxBuffer[8]) | (((int16_t) bno->RxBuffer[9]) << 8);
	int16_t raw_mag_z = ((int16_t) bno->RxBuffer[10]) | (((int16_t) bno->RxBuffer[11]) << 8);

	int16_t raw_gyro_x = ((int16_t) bno->RxBuffer[12]) | (((int16_t) bno->RxBuffer[13]) << 8);
	int16_t raw_gyro_y = ((int16_t) bno->RxBuffer[14]) | (((int16_t) bno->RxBuffer[15]) << 8);
	int16_t raw_gyro_z = ((int16_t) bno->RxBuffer[16]) | (((int16_t) bno->RxBuffer[17]) << 8);

	int16_t raw_euler_yaw = ((int16_t) bno->RxBuffer[18]) | (((int16_t) bno->RxBuffer[19]) << 8);
	int16_t raw_euler_roll = ((int16_t) bno->RxBuffer[20]) | (((int16_t) bno->RxBuffer[21]) << 8);
	int16_t raw_euler_pitch = ((int16_t) bno->RxBuffer[22]) | (((int16_t) bno->RxBuffer[23]) << 8);

	int16_t raw_quat_x = ((int16_t) bno->RxBuffer[24]) | (((int16_t) bno->RxBuffer[25]) << 8);
	int16_t raw_quat_y = ((int16_t) bno->RxBuffer[26]) | (((int16_t) bno->RxBuffer[27]) << 8);
	int16_t raw_quat_z = ((int16_t) bno->RxBuffer[28]) | (((int16_t) bno->RxBuffer[29]) << 8);
	int16_t raw_quat_w = ((int16_t) bno->RxBuffer[30]) | (((int16_t) bno->RxBuffer[31]) << 8);

	const double scale = (1.0 / (1 << 14));

	bno->accel.x = ((double) raw_accel_x) / 100.0;
	bno->accel.y = ((double) raw_accel_y) / 100.0;
	bno->accel.z = ((double) raw_accel_z) / 100.0;

	bno->mag.x = ((double) raw_mag_x) / 16.0;
	bno->mag.y = ((double) raw_mag_y) / 16.0;
	bno->mag.z = ((double) raw_mag_z) / 16.0;

	bno->gyro.x = ((double) raw_gyro_x) / 16.0;
	bno->gyro.y = ((double) raw_gyro_y) / 16.0;
	bno->gyro.z = ((double) raw_gyro_z) / 16.0;

	bno->euler.yaw = ((double) raw_euler_yaw) / 16.0;
	bno->euler.roll = ((double) raw_euler_roll) / 16.0;
	bno->euler.pitch = ((double) raw_euler_pitch) / 16.0;

	bno->quat.x = raw_quat_x * scale;
	bno->quat.y = raw_quat_y * scale;
	bno->quat.z = raw_quat_z * scale;
	bno->quat.w = raw_quat_w * scale;

	if (!fast_mode) {
		int16_t raw_lin_acc_x = ((int16_t) bno->RxBuffer[32]) | (((int16_t) bno->RxBuffer[33]) << 8);
		int16_t raw_lin_acc_y = ((int16_t) bno->RxBuffer[34]) | (((int16_t) bno->RxBuffer[35]) << 8);
		int16_t raw_lin_acc_z = ((int16_t) bno->RxBuffer[36]) | (((int16_t) bno->RxBuffer[37]) << 8);

		int16_t raw_grav_x = ((int16_t) bno->RxBuffer[38]) | (((int16_t) bno->RxBuffer[39]) << 8);
		int16_t raw_grav_y = ((int16_t) bno->RxBuffer[40]) | (((int16_t) bno->RxBuffer[41]) << 8);
		int16_t raw_grav_z = ((int16_t) bno->RxBuffer[42]) | (((int16_t) bno->RxBuffer[43]) << 8);

		bno->lin_acc.x = ((double) raw_lin_acc_x) / 100.0;
		bno->lin_acc.y = ((double) raw_lin_acc_y) / 100.0;
		bno->lin_acc.z = ((double) raw_lin_acc_z) / 100.0;

		bno->grav.x = ((double) raw_grav_x) / 100.0;
		bno->grav.y = ((double) raw_grav_y) / 100.0;
		bno->grav.z = ((double) raw_grav_z) / 100.0;

		HAL_I2C_Mem_Read_DMA(bno->hi2cx, bno->address, ACC_DATA_X_LSB, 1, bno->RxBuffer, 44);
	} else {
		HAL_I2C_Mem_Read_DMA(bno->hi2cx, bno->address, ACC_DATA_X_LSB, 1, bno->RxBuffer, 32);
	}
}
