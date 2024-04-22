/*
 * BNO055.c
 *
 *  Created on: Apr 18, 2024
 *      Author: 08809
 */

#include "BNO055.h"
#include "BNO055_offsets.h"

Calibration_Stat Calibrated = {.accel=HAL_BUSY, .gyro=HAL_BUSY, .mag=HAL_BUSY};

//Setup Offsets


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

void BNO055_getCalibration(BNO055_Structure *bno, uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
{
	uint8_t txbuffer;
	txbuffer = Page_ID_00;

	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, PAGE_ID, 1, &txbuffer, 1, 10);

	HAL_Delay(10);
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CALIB_STAT, 1, bno->RxBuffer, 1, 10);


	if (sys != NULL) {
		*sys = (bno->RxBuffer[0] >> 6) & 0x03;
	}
	if (gyro != NULL) {
		*gyro = (bno->RxBuffer[0] >> 4) & 0x03;
	}
	if (accel != NULL) {
		*accel = (bno->RxBuffer[0] >> 2) & 0x03;
	}
	if (mag != NULL) {
		*mag = bno->RxBuffer[0] & 0x03;
	}
}

HAL_StatusTypeDef BNO055_isFullyCalibrated(BNO055_Structure *bno)
{
	uint8_t check_config = (BNO055_read8(bno, OPR_MODE) & 0x0F);
	uint8_t txbuffer;
	txbuffer = NDOF;
	if (check_config != NDOF){
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &txbuffer, 1, 10);
		HAL_Delay(20);
	}

	bno->mode = NDOF;
	uint8_t system, gyro, accel, mag;

	BNO055_getCalibration(bno, &system, &gyro, &accel, &mag);

	switch(bno->mode){
	case ACCONLY:
		if(accel == 3) return HAL_OK;
	case MAGONLY:
		if(mag == 3) return HAL_OK;
	case GYROONLY:
		if(gyro == 3) return HAL_OK;
	case ACCMAG:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (mag == 3) Calibrated.mag = HAL_OK;
		if(accel == 3 && mag == 3) return HAL_OK;
	case ACCGYRO:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (gyro) Calibrated.gyro = HAL_OK;
		if(accel == 3 && gyro == 3) return HAL_OK;
	case MAGGYRO:
		if (mag == 3) Calibrated.mag = HAL_OK;
		if (gyro == 3) Calibrated.gyro = HAL_OK;
		if(mag == 3 && gyro == 3) return HAL_OK;
	case AMG:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (mag == 3) Calibrated.mag = HAL_OK;
		if (gyro == 3) Calibrated.gyro = HAL_OK;
		if(accel == 3 && mag == 3 && gyro == 3) return HAL_OK;
	case IMU:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (gyro == 3) Calibrated.gyro = HAL_OK;
		if(accel == 3 && gyro == 3) return HAL_OK;
	case COMPASS:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (mag == 3) Calibrated.mag = HAL_OK;
		if(accel == 3 && mag == 3) return HAL_OK;
	case M4G:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (mag == 3) Calibrated.mag = HAL_OK;
		if(accel == 3 && mag == 3) return HAL_OK;
	case NDOF_FMC_OFF:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (mag == 3) Calibrated.mag = HAL_OK;
		if (gyro == 3) Calibrated.gyro = HAL_OK;
		if(accel == 3 && mag == 3 && gyro == 3) return HAL_OK;
	case NDOF:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (mag == 3) Calibrated.mag = HAL_OK;
		if (gyro == 3) Calibrated.gyro = HAL_OK;
		if(accel == 3 && mag == 3 && gyro == 3) return HAL_OK;
	default:
		if (accel == 3) Calibrated.accel = HAL_OK;
		if (mag == 3) Calibrated.mag = HAL_OK;
		if (gyro == 3) Calibrated.gyro = HAL_OK;
		if(accel == 3 && mag == 3 && gyro == 3) return HAL_OK;
	}

	return HAL_ERROR;
}

void BNO055_setMode(BNO055_Structure *bno, OPRMode *mode)
{
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, mode, 1, 10);
	HAL_Delay(20);
}

uint8_t BNO055_read8(BNO055_Structure *bno, uint8_t Register_Address)
{
	uint16_t Register_Address_u16 = Register_Address;
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, Register_Address_u16, 1, bno->RxBuffer, 1, 10);
	uint8_t Rx = bno->RxBuffer[0];
	HAL_Delay(20);

	return bno->RxBuffer[0];
}

uint8_t BNO055_write8(BNO055_Structure *bno, uint8_t Register_Address, uint8_t data)
{
	uint16_t Register_Address_u16 = Register_Address;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, Register_Address_u16, 1, &data, 1, 10);
	HAL_Delay(20);

	return bno->RxBuffer[0];
}

HAL_StatusTypeDef BNO055_getSensorOffsets(BNO055_Structure *bno)
{
	if (BNO055_isFullyCalibrated(bno) == HAL_OK){

		uint8_t check_config = (BNO055_read8(bno, OPR_MODE) & 0x0F);
		uint8_t txbuffer;
		txbuffer = CONFIGMODE;
		if (check_config != CONFIGMODE){
			HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &txbuffer, 1, 10);
			HAL_Delay(20);
		}

		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, ACC_OFFSET_X_LSB, 1, bno->RxBuffer, 22, 10);

		bno->offsets.accel_offset_x = ((bno->RxBuffer[1] <<8) | bno->RxBuffer[0]);
		bno->offsets.accel_offset_y = (bno->RxBuffer[3] <<8 | bno->RxBuffer[2]);
		bno->offsets.accel_offset_z = (bno->RxBuffer[5] <<8 | bno->RxBuffer[4]);
		/* Accelerometer offset end*/

		/* Magnetometer offset Start*/
		bno->offsets.mag_offset_x = (bno->RxBuffer[7] <<8 | bno->RxBuffer[6]);
		bno->offsets.mag_offset_y = (bno->RxBuffer[9] <<8 | bno->RxBuffer[8]);
		bno->offsets.mag_offset_z = (bno->RxBuffer[11] <<8 | bno->RxBuffer[10]);
		/* Magnetometer offset end*/

		/* Gyro offset Start*/
		bno->offsets.gyro_offset_x = (bno->RxBuffer[13] <<8 | bno->RxBuffer[12]);
		bno->offsets.gyro_offset_y = (bno->RxBuffer[15] <<8 | bno->RxBuffer[14]);
		bno->offsets.gyro_offset_z = (bno->RxBuffer[17] <<8 | bno->RxBuffer[16]);
		/* Gyro offset end*/

		/*  Accelerometer radius Start*/
		bno->offsets.accel_radius = (bno->RxBuffer[19] <<8 | bno->RxBuffer[18]);
		/*  Accelerometer radius end*/

		/*  Magnetometer radius Start*/
		bno->offsets.mag_radius = (bno->RxBuffer[21] <<8 | bno->RxBuffer[20]);
		/*  Magnetometer radius end*/

		check_config = (BNO055_read8(bno, OPR_MODE) & 0x0F);
		txbuffer = (uint8_t)bno->mode;
		if (check_config != CONFIGMODE){
			HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &txbuffer, 1, 10);
			HAL_Delay(20);
		}
		return HAL_OK;
	}
	return HAL_ERROR;
}

void BNO055_setSensoroffsets(BNO055_Structure *bno)
{
	uint8_t check_config = (BNO055_read8(bno, OPR_MODE) & 0x0F);
	uint8_t txbuffer[40];
	txbuffer[0] = CONFIGMODE;
	if (check_config != CONFIGMODE){
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, txbuffer, 1, 10);
		HAL_Delay(20);
	}

	txbuffer[0] = bno->offsets.accel_offset_x & 0x0FF;
	txbuffer[1] = (bno->offsets.accel_offset_x >> 8) & 0x0FF;
	txbuffer[2] = bno->offsets.accel_offset_y & 0x0FF;
	txbuffer[3] = (bno->offsets.accel_offset_y >> 8) & 0x0FF;
	txbuffer[4] = bno->offsets.accel_offset_z & 0x0FF;
	txbuffer[5] = (bno->offsets.accel_offset_z >> 8) & 0x0FF;
	txbuffer[6] = bno->offsets.mag_offset_x & 0x0FF;
	txbuffer[7] = (bno->offsets.mag_offset_x >> 8) & 0x0FF;
	txbuffer[8] = bno->offsets.mag_offset_y & 0x0FF;
	txbuffer[9] = (bno->offsets.mag_offset_y >> 8) & 0x0FF;
	txbuffer[10] = bno->offsets.mag_offset_z & 0x0FF;
	txbuffer[11] = (bno->offsets.mag_offset_z >> 8) & 0x0FF;
	txbuffer[12] = bno->offsets.gyro_offset_x & 0x0FF;
	txbuffer[13] = (bno->offsets.gyro_offset_x >> 8) & 0x0FF;
	txbuffer[14] = bno->offsets.gyro_offset_y & 0x0FF;
	txbuffer[15] = (bno->offsets.gyro_offset_y >> 8) & 0x0FF;
	txbuffer[16] = bno->offsets.gyro_offset_z & 0x0FF;
	txbuffer[17] = (bno->offsets.gyro_offset_z >> 8) & 0x0FF;
	txbuffer[18] = bno->offsets.accel_radius & 0x0FF;
	txbuffer[19] = (bno->offsets.accel_radius >> 8) & 0x0FF;
	txbuffer[20] = bno->offsets.mag_radius & 0x0FF;
	txbuffer[21] = (bno->offsets.mag_radius >> 8) & 0x0FF;

	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, ACC_OFFSET_X_LSB, 1, txbuffer, 22, 10);


//	BNO055_write8(bno, ACC_OFFSET_X_LSB, (bno->offsets.accel_offset_x) & 0x0FF);
//	BNO055_write8(bno, ACC_OFFSET_X_MSB, (bno->offsets.accel_offset_x >> 8) & 0x0FF);
//	BNO055_write8(bno, ACC_OFFSET_Y_LSB, (bno->offsets.accel_offset_y) & 0x0FF);
//	BNO055_write8(bno, ACC_OFFSET_Y_MSB, (bno->offsets.accel_offset_y >> 8) & 0x0FF);
//	BNO055_write8(bno, ACC_OFFSET_Z_LSB, (bno->offsets.accel_offset_z) & 0x0FF);
//	BNO055_write8(bno, ACC_OFFSET_Z_MSB, (bno->offsets.accel_offset_z >> 8) & 0x0FF);
//
//	BNO055_write8(bno, MAG_OFFSET_X_LSB, (bno->offsets.mag_offset_x) & 0x0FF);
//	BNO055_write8(bno, MAG_OFFSET_X_MSB, (bno->offsets.mag_offset_x >> 8) & 0x0FF);
//	BNO055_write8(bno, MAG_OFFSET_Y_LSB, (bno->offsets.mag_offset_y) & 0x0FF);
//	BNO055_write8(bno, MAG_OFFSET_Y_MSB, (bno->offsets.mag_offset_y >> 8) & 0x0FF);
//	BNO055_write8(bno, MAG_OFFSET_Z_LSB, (bno->offsets.mag_offset_z) & 0x0FF);
//	BNO055_write8(bno, MAG_OFFSET_Z_MSB, (bno->offsets.mag_offset_z >> 8) & 0x0FF);
//
//	BNO055_write8(bno, GYR_OFFSET_X_LSB, (bno->offsets.gyro_offset_x) & 0x0FF);
//	BNO055_write8(bno, GYR_OFFSET_X_MSB, (bno->offsets.gyro_offset_x >> 8) & 0x0FF);
//	BNO055_write8(bno, GYR_OFFSET_Y_LSB, (bno->offsets.gyro_offset_y) & 0x0FF);
//	BNO055_write8(bno, GYR_OFFSET_Y_MSB, (bno->offsets.gyro_offset_y >> 8) & 0x0FF);
//	BNO055_write8(bno, GYR_OFFSET_Z_LSB, (bno->offsets.gyro_offset_z) & 0x0FF);
//	BNO055_write8(bno, GYR_OFFSET_Z_MSB, (bno->offsets.gyro_offset_z >> 8) & 0x0FF);
//
//	BNO055_write8(bno, ACC_RADIUS_LSB, (bno->offsets.accel_radius) & 0x0FF);
//	BNO055_write8(bno, ACC_RADIUS_MSB, (bno->offsets.accel_radius >> 8) & 0x0FF);
//
//	BNO055_write8(bno, MAG_RADIUS_LSB, (bno->offsets.mag_radius) & 0x0FF);
//	BNO055_write8(bno, MAG_RADIUS_MSB, (bno->offsets.mag_radius >> 8) & 0x0FF);

	check_config = (BNO055_read8(bno, OPR_MODE) & 0x0F);
	txbuffer[0] = (uint8_t)bno->mode;
	if (check_config != CONFIGMODE){
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, txbuffer, 1, 10);
		HAL_Delay(20);
	}
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


