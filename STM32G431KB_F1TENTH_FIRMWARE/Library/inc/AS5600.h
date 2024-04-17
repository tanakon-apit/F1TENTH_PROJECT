/*
 * AS5600.h
 *
 *  Created on: Apr 18, 2024
 *      Author: tucha
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stdint.h"
#include "main.h"

// Math
#define PI 3.141592

#define AS5600_ADDRESS 0x36 << 1
#define AS5600_ANGLE     0x0E   //  + 0x0F
#define AS5600_SW_DIRECTION_PIN   255

//  setDirection
#define AS5600_CLOCK_WISE          0  //  LOW
#define AS5600_COUNTERCLOCK_WISE  1  //  HIGH

//  0.087890625;
#define   AS5600_RAW_TO_DEGREES     360.0 / 4096
#define   AS5600_DEGREES_TO_RAW     4096 / 360.0
//  0.00153398078788564122971808758949;
#define   AS5600_RAW_TO_RADIANS     PI * 2.0 / 4096
//  4.06901041666666e-6
#define   AS5600_RAW_TO_RPM         60.0 / 4096

//  getAngularSpeed
#define AS5600_MODE_DEGREES       0
#define AS5600_MODE_RADIANS       1
#define AS5600_MODE_RPM           2

//  ERROR CODES
#define     AS5600_OK                 0
#define     AS5600_ERROR_I2C_READ_0   -100
#define     AS5600_ERROR_I2C_READ_1   -101
#define     AS5600_ERROR_I2C_READ_2   -102
#define     AS5600_ERROR_I2C_READ_3   -103
#define     AS5600_ERROR_I2C_WRITE_0  -200
#define     AS5600_ERROR_I2C_WRITE_1  -201

extern HAL_StatusTypeDef ret;
extern uint8_t reg_buf[1];
extern uint8_t buf[12];
extern uint8_t Data_buf[2];

extern uint8_t _directionPin;

//  for getAngularSpeed()
extern uint32_t _lastMeasurement;
extern int16_t  _lastAngle;

//  for readAngle() and rawAngle()
extern uint16_t _offset;

//  cumulative position counter
//  works only if the sensor is read often enough.
extern int32_t  _position;
extern int16_t  _lastPosition;

//Protected
//uint8_t  _directionPin    = 255;
//uint8_t  _direction       = AS5600_CLOCK_WISE;
extern int      _error;

uint8_t AS5600_isConnected(I2C_HandleTypeDef* hi2cx);

uint16_t AS5600_readReg2(I2C_HandleTypeDef* hi2cx, uint8_t reg);

uint16_t AS5600_readAngle(I2C_HandleTypeDef* hi2cx);

int32_t AS5600_getCumulativePosition(I2C_HandleTypeDef* hi2cx);

float AS5600_getAngularSpeed(I2C_HandleTypeDef* hi2cx, uint8_t mode);

float AS5600_raw2rad(uint16_t value);

int32_t AS5600_resetCumulativePosition(I2C_HandleTypeDef* hi2cx, int32_t position);

int AS5600_lastError();

int32_t AS5600_getRevolutions();

int32_t AS5600_resetPosition(int32_t position);

#endif /* INC_AS5600_H_ */