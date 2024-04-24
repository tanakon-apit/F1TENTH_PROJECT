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
HAL_StatusTypeDef AS5600_isConnected(AS5600_Structure *as5600)
{
	_error = AS5600_OK;

	ret = HAL_I2C_Mem_Read(as5600->hi2cx, as5600->address, AS5600_ANGLE, 1, as5600->RxBuffer, 2, 10);
	if (ret != HAL_OK){
		_error = AS5600_ERROR_I2C_READ_3;
		return HAL_ERROR;
	}
	return HAL_OK;
}

uint16_t AS5600_readRaw(AS5600_Structure *as5600)
{
	uint16_t data;

	ret = HAL_I2C_Mem_Read_DMA(as5600->hi2cx, as5600->address, AS5600_ANGLE, 1, as5600->RxBuffer, 2);

	data = as5600->RxBuffer[0];
	data <<= 8;
	data += as5600->RxBuffer[1];
	return data ;

}

float AS5600_readAngle(AS5600_Structure *as5600)
{
	uint16_t value = AS5600_readRaw(as5600) & 0x0FFF;

	uint16_t denominator = (1 << 12);
	float rad = (value / (float)(denominator - 1)) * 180;

	return rad;
}

float AS5600_getCumulativePosition(AS5600_Structure *as5600)
{
	int16_t value = AS5600_readRaw(as5600) & 0x0FFF;

	if ((_lastPosition > 2048) && ( value < (_lastPosition - 2048)))
	{
		_position = _position + 4096 - _lastPosition + value;
	}

	else if ((value > 2048) && ( _lastPosition < (value - 2048)))
	{
		_position = _position - 4096 - _lastPosition + value;
	}
	else _position = _position - _lastPosition + value;
	_lastPosition = value;

	uint16_t denominator = (1 << 12);
	float cumulative_rad = (_position / (float)(denominator - 1)) * 2 * 180;

	return cumulative_rad;
}

float AS5600_resetCumulativePosition(AS5600_Structure *as5600, float position)
{
	_lastPosition = AS5600_readRaw(as5600) & 0x0FFF;
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
}


float AS5600_resetPosition(float position)
{
	float old = _position;
	_position = position;
	return old;
}


/* USER CODE BEGIN PID */
