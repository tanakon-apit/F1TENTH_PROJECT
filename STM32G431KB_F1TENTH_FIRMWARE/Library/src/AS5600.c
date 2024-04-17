/*
 * AS5600.c
 *
 *  Created on: Apr 18, 2024
 *      Author: tucha
 */

#include <AS5600.h>

HAL_StatusTypeDef ret;
uint8_t reg_buf[1];
uint8_t buf[12];
uint8_t Data_buf[2];

uint8_t _directionPin;

//  for getAngularSpeed()
uint32_t _lastMeasurement = 0;
int16_t  _lastAngle       = 0;

//  for readAngle() and rawAngle()
uint16_t _offset          = 0;

//  cumulative position counter
//  works only if the sensor is read often enough.
int32_t  _position        = 0;
int16_t  _lastPosition    = 0;

//Protected
//uint8_t  _directionPin    = 255;
//uint8_t  _direction       = AS5600_CLOCK_WISE;
int      _error           = AS5600_OK;

/* USER CODE BEGIN PID */
uint8_t AS5600_isConnected(I2C_HandleTypeDef* hi2cx)
{
	_error = AS5600_OK;
	reg_buf[0] = AS5600_ANGLE;
	ret = HAL_I2C_Master_Transmit(hi2cx, AS5600_ADDRESS, reg_buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		_error = AS5600_ERROR_I2C_READ_2;
		return 0;
	}

	ret = HAL_I2C_Master_Receive(hi2cx, AS5600_ADDRESS, Data_buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		_error = AS5600_ERROR_I2C_READ_3;
		return 0;
	}
	return 1;
}

uint16_t AS5600_readReg2(I2C_HandleTypeDef* hi2cx, uint8_t reg)
{
	_error = AS5600_OK;
	reg_buf[0] = AS5600_ANGLE;
	ret = HAL_I2C_Master_Transmit(hi2cx, AS5600_ADDRESS, reg_buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		_error = AS5600_ERROR_I2C_READ_2;
		return 0;
	}

	uint16_t data;

	ret = HAL_I2C_Master_Receive(hi2cx, AS5600_ADDRESS, Data_buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){

		_error = AS5600_ERROR_I2C_READ_3;
		return 0;
	}
	else{
		data = Data_buf[0];
		data <<= 8;
		data += Data_buf[1];
		return data ;
	}
}

uint16_t AS5600_readAngle(I2C_HandleTypeDef* hi2cx)
{
	uint16_t value = AS5600_readReg2(hi2cx, AS5600_ANGLE) & 0x0FFF;
	//  if (_offset > 0) value = (value + _offset) & 0x0FFF;
	//
	//  if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
	//      (_direction == AS5600_COUNTERCLOCK_WISE))
	//  {
	//    value = (4096 - value) & 0x0FFF;
	//  }
	return value;
}

int32_t AS5600_getCumulativePosition(I2C_HandleTypeDef* hi2cx)
{
	int16_t value = AS5600_readReg2(hi2cx, AS5600_ANGLE) & 0x0FFF;

	//  whole rotation CW?
	//  less than half a circle
	if ((_lastPosition > 2048) && ( value < (_lastPosition - 2048)))
	{
		_position = _position + 4096 - _lastPosition + value;
	}
	//  whole rotation CCW?
	//  less than half a circle
	else if ((value > 2048) && ( _lastPosition < (value - 2048)))
	{
		_position = _position - 4096 - _lastPosition + value;
	}
	else _position = _position - _lastPosition + value;
	_lastPosition = value;

	return _position;
}

float AS5600_raw2rad(uint16_t value){
	float rad = (value / 4096.0) * 3.14;
	return rad;
}

int32_t AS5600_resetCumulativePosition(I2C_HandleTypeDef* hi2cx, int32_t position)
{
	_lastPosition = AS5600_readReg2(hi2cx, AS5600_ANGLE) & 0x0FFF;
	int32_t old = _position;
	_position = position;
	return old;
}


int AS5600_lastError()
{
	int value = _error;
	_error = AS5600_OK;
	return value;
}

int32_t AS5600_getRevolutions()
{
	int32_t p = _position >> 12;  //  divide by 4096
	return p;
	// if (p < 0) p++;
	// return p;
}


int32_t AS5600_resetPosition(int32_t position)
{
	int32_t old = _position;
	_position = position;
	return old;
}

float AS5600_getAngularSpeed(I2C_HandleTypeDef* hi2cx, uint8_t mode)
{
	uint32_t now     = HAL_GetTick();
	int      angle   = AS5600_readAngle(hi2cx);
	uint32_t deltaT  = now - _lastMeasurement;
	int      deltaA  = angle - _lastAngle;

	//  assumption is that there is no more than 180° rotation
	//  between two consecutive measurements.
	//  => at least two measurements per rotation (preferred 4).
	if (deltaA >  2048) deltaA -= 4096;
	if (deltaA < -2048) deltaA += 4096;
	float    speed   = (deltaA * 1e6) / deltaT;

	//  remember last time & angle
	_lastMeasurement = now;
	_lastAngle       = angle;

	//  return radians, RPM or degrees.
	if (mode == AS5600_MODE_RADIANS)
	{
		return speed * AS5600_RAW_TO_RADIANS;
	}
	if (mode == AS5600_MODE_RPM)
	{
		return speed * AS5600_RAW_TO_RPM;
	}
	//  default return degrees
	return speed * AS5600_RAW_TO_DEGREES;
}


/* USER CODE BEGIN PID */
