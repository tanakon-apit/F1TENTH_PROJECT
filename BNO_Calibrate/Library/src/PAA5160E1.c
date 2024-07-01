/*
 * PAA5160E1.c
 *
 *  Created on: Apr 18, 2024
 *      Author: 08809
 */

#include "PAA5160E1.h"

/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
 */

/*******************************************************************************
    sfeQwiicOtos.h - C++ driver implementation for the SparkFun Qwiic Optical
    Tracking Odometry Sensor (OTOS).
 *******************************************************************************/

HAL_StatusTypeDef PAA5160E1_Init(PAA5160E1_Structure *paa, I2C_HandleTypeDef *hi2cx, uint8_t addr, OPRMode mode, uint8_t unit)
{
	uint8_t txbuffer;
	uint8_t rxbuffer;

	paa->hi2cx = hi2cx;
	paa->address = kDefaultAddress;

	HAL_I2C_Mem_Read(paa->hi2cx, paa->address, kRegProductId, 1, &rxbuffer, 1, 10);
	if (rxbuffer != kProductId) {
		HAL_Delay(1000);
		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CHIP_ID, 1, &rxbuffer, 1, 10);
		if (rxbuffer != kProductId) return HAL_ERROR;
	}

	paa->flag = HAL_OK;

	// Just check if the device is connected, no other setup is needed
	return HAL_OK;
}

HAL_StatusTypeDef PAA5160E1_calibrateImu(PAA5160E1_Structure *paa, uint8_t numSamples, uint8_t waitUntilDone)
{
	HAL_I2C_Mem_Write(paa->hi2cx, paa->address, kRegImuCalib, 1, &numSamples, 1, 10);

	// Wait 1 sample period (2.4ms) to ensure the register updates
	HAL_Delay(10);

	// Do we need to wait until the calibration finishes?
	if(waitUntilDone == 0) return HAL_OK;

	// Wait for the calibration to finish, which is indicated by the IMU
	// calibration register reading zero, or until we reach the maximum number
	// of read attempts
	for(uint8_t numAttempts = numSamples; numAttempts > 0; numAttempts--)
	{
		// Read the gryo calibration register value
		HAL_I2C_Mem_Read(paa->hi2cx, paa->address, kRegImuCalib, 1, paa->Calibration_Stat.numSample, 1, 10);

		// Check if calibration is done
		if(paa->Calibration_Stat.numSample == 0)
			return HAL_OK;

		// Give a short delay between reads. As of firmware v1.0, samples take
		// 2.4ms each, so 3ms should guarantee the next sample is done. This
		// also ensures the max attempts is not exceeded in normal operation
		HAL_Delay(10);
	}

	// Max number of attempts reached, calibration failed
	return HAL_ERROR;
}

HAL_StatusTypeDef PAA5160E1_getLinearScalar(PAA5160E1_Structure *paa)
{
	// Read the linear scalar from the device
	uint8_t rawScalar;
	HAL_I2C_Mem_Read(paa->hi2cx, paa->address, kRegScalarLinear, 1, paa->Calibration_Stat.numSample, 1, 10);
	sfeTkError_t err = _commBus->readRegisterByte(kRegScalarLinear, rawScalar);

	// Convert to float, multiples of 0.1%
	scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

	// Done!
	return kSTkErrOk;
}

HAL_StatusTypeDef PAA5160E1_setLinearScalar(PAA5160E1_Structure *paa, float scalar)
{
	// Check if the scalar is out of bounds
	if(scalar < kMinScalar || scalar > kMaxScalar)
		return kSTkErrFail;

	// Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
	uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

	// Write the scalar to the device
	return _commBus->writeRegisterByte(kRegScalarLinear, rawScalar);
}

HAL_StatusTypeDef PAA5160E1_getAngularScalar(PAA5160E1_Structure *paa)
{
	// Read the angular scalar from the device
	uint8_t rawScalar;
	sfeTkError_t err = _commBus->readRegisterByte(kRegScalarAngular, rawScalar);
	if(err != kSTkErrOk)
		return kSTkErrFail;

	// Convert to float, multiples of 0.1%
	scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

	// Done!
	return kSTkErrOk;
}

HAL_StatusTypeDef PAA5160E1_setAngularScalar(PAA5160E1_Structure *paa, float scalar)
{
	// Check if the scalar is out of bounds
	if(scalar < kMinScalar || scalar > kMaxScalar)
		return kSTkErrFail;

	// Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
	uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

	// Write the scalar to the device
	return _commBus->writeRegisterByte(kRegScalarAngular, rawScalar);
}

HAL_StatusTypeDef resetTracking()
{
	// Set tracking reset bit
	return HAL_I2C_Mem_Write(paa->hi2cx, paa->address, kRegReset, 1, 0x01, 1, 10);
}

HAL_StatusTypeDef PAA5160E1_getStatus(PAA5160E1_Structure *paa)
{
	return HAL_I2C_Mem_Read(paa->hi2cx, paa->address, kRegStatus, 1, paa->Calibration_Stat.numSample, 1, 10);
}

HAL_StatusTypeDef PAA5160E1_getOffset(PAA5160E1_Structure *paa)
{
	return readPoseRegs(kRegOffXL, pose, kInt16ToMeter, kInt16ToRad);
}

HAL_StatusTypeDef PAA5160E1_setOffset(PAA5160E1_Structure *paa)
{
	return writePoseRegs(kRegOffXL, pose, kMeterToInt16, kRadToInt16);
}

HAL_StatusTypeDef PAA5160E1_getPosition(PAA5160E1_Structure *paa)
{
	return readPoseRegs(kRegPosXL, pose, kInt16ToMeter, kInt16ToRad);
}

HAL_StatusTypeDef PAA5160E1_setPosition(PAA5160E1_Structure *paa)
{
	return writePoseRegs(kRegPosXL, pose, kMeterToInt16, kRadToInt16);
}

HAL_StatusTypeDef PAA5160E1_getVelocity(PAA5160E1_Structure *paa)
{
	return readPoseRegs(kRegVelXL, pose, kInt16ToMps, kInt16ToRps);
}

HAL_StatusTypeDef PAA5160E1_getAcceleration(PAA5160E1_Structure *paa)
{
	return readPoseRegs(kRegAccXL, pose, kInt16ToMpss, kInt16ToRpss);
}

HAL_StatusTypeDef PAA5160E1_getPositionStdDev(PAA5160E1_Structure *paa)
{
	return readPoseRegs(kRegPosStdXL, pose, kInt16ToMeter, kInt16ToRad);
}

HAL_StatusTypeDef PAA5160E1_getVelocityStdDev(PAA5160E1_Structure *paa)
{
	return readPoseRegs(kRegVelStdXL, pose, kInt16ToMps, kInt16ToRps);
}

HAL_StatusTypeDef PAA5160E1_getAccelerationStdDev(PAA5160E1_Structure *paa)
{
	return readPoseRegs(kRegAccStdXL, pose, kInt16ToMpss, kInt16ToRpss);
}

HAL_StatusTypeDef PAA5160E1_getPosVelAcc(PAA5160E1_Structure *paa)
{
	// Read all pose registers
	uint8_t rawData[18];
	size_t bytesRead;
	sfeTkError_t err = _commBus->readRegisterRegion(kRegPosXL, rawData, 18, bytesRead);
	if(err != kSTkErrOk)
		return err;

	// Check if we read the correct number of bytes
	if(bytesRead != 18)
		return kSTkErrFail;

	// Convert raw data to pose units
	regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
	regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
	regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);

	// Done!
	return kSTkErrOk;
}

HAL_StatusTypeDef PAA5160E1_getPosVelAccStdDev(PAA5160E1_Structure *paa)
{
	// Read all pose registers
	uint8_t rawData[18];
	size_t bytesRead;
	sfeTkError_t err = _commBus->readRegisterRegion(kRegPosStdXL, rawData, 18, bytesRead);
	if(err != kSTkErrOk)
		return err;

	// Check if we read the correct number of bytes
	if(bytesRead != 18)
		return kSTkErrFail;

	// Convert raw data to pose units
	regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
	regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
	regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);

	// Done!
	return kSTkErrOk;
}

HAL_StatusTypeDef PAA5160E1_getPosVelAccAndStdDev(PAA5160E1_Structure *paa)
{
	// Read all pose registers
	uint8_t rawData[36];
	size_t bytesRead;
	sfeTkError_t err = _commBus->readRegisterRegion(kRegPosXL, rawData, 36, bytesRead);
	if(err != kSTkErrOk)
		return err;

	// Check if we read the correct number of bytes
	if(bytesRead != 36)
		return kSTkErrFail;

	// Convert raw data to pose units
	regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
	regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
	regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);
	regsToPose(rawData + 18, posStdDev, kInt16ToMeter, kInt16ToRad);
	regsToPose(rawData + 24, velStdDev, kInt16ToMps, kInt16ToRps);
	regsToPose(rawData + 30, accStdDev, kInt16ToMpss, kInt16ToRpss);

	// Done!
	return kSTkErrOk;
}

HAL_StatusTypeDef PAA5160E1_readPoseRegs(PAA5160E1_Structure *paa, float rawToXY, float rawToH)
{
	size_t bytesRead;
	uint8_t rawData[6];

	// Attempt to read the raw pose data
	sfeTkError_t err = _commBus->readRegisterRegion(reg, rawData, 6, bytesRead);
	if (err != kSTkErrOk)
		return err;

	// Check if we read the correct number of bytes
	if (bytesRead != 6)
		return kSTkErrFail;

	regsToPose(rawData, pose, rawToXY, rawToH);

	// Done!
	return kSTkErrOk;
}

HAL_StatusTypeDef PAA5160E1_writePoseRegs(PAA5160E1_Structure *paa, float xyToRaw, float hToRaw)
{
	// Store raw data in a temporary buffer
	uint8_t rawData[6];
	poseToRegs(rawData, pose, xyToRaw, hToRaw);

	// Write the raw data to the device
	return _commBus->writeRegisterRegion(reg, rawData, 6);
}

void PAA5160E1_regsToPose(PAA5160E1_Structure *paa, uint8_t *rawData, float rawToXY, float rawToH)
{
	// Store raw data
	int16_t rawX = (rawData[1] << 8) | rawData[0];
	int16_t rawY = (rawData[3] << 8) | rawData[2];
	int16_t rawH = (rawData[5] << 8) | rawData[4];

	// Store in pose and convert to units
	pose.x = rawX * rawToXY * _meterToUnit;
	pose.y = rawY * rawToXY * _meterToUnit;
	pose.h = rawH * rawToH * _radToUnit;
}

void PAA5160E1_poseToRegs(PAA5160E1_Structure *paa, uint8_t *rawData, float xyToRaw, float hToRaw)
{
	// Convert pose units to raw data
	int16_t rawX = pose.x * xyToRaw / _meterToUnit;
	int16_t rawY = pose.y * xyToRaw / _meterToUnit;
	int16_t rawH = pose.h * hToRaw / _radToUnit;

	// Store raw data in buffer
	rawData[0] = rawX & 0xFF;
	rawData[1] = (rawX >> 8) & 0xFF;
	rawData[2] = rawY & 0xFF;
	rawData[3] = (rawY >> 8) & 0xFF;
	rawData[4] = rawH & 0xFF;
	rawData[5] = (rawH >> 8) & 0xFF;
}
