/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Controller.h"
#include "Cytron_Motor_260rpm_250W.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Math
const float PI = 3.141592;


static const uint8_t AS5600_ADDRESS = 0x36 << 1;
const uint8_t AS5600_ANGLE     = 0x0E;   //  + 0x0F

//  setDirection
const uint8_t AS5600_CLOCK_WISE         = 0;  //  LOW
const uint8_t AS5600_COUNTERCLOCK_WISE  = 1;  //  HIGH

//  0.087890625;
const float   AS5600_RAW_TO_DEGREES     = 360.0 / 4096;
const float   AS5600_DEGREES_TO_RAW     = 4096 / 360.0;
//  0.00153398078788564122971808758949;
const float   AS5600_RAW_TO_RADIANS     = PI * 2.0 / 4096;
//  4.06901041666666e-6
const float   AS5600_RAW_TO_RPM         = 60.0 / 4096;

//  getAngularSpeed
const uint8_t AS5600_MODE_DEGREES       = 0;
const uint8_t AS5600_MODE_RADIANS       = 1;
const uint8_t AS5600_MODE_RPM           = 2;

//  ERROR CODES
const int     AS5600_OK                 = 0;
const int     AS5600_ERROR_I2C_READ_0   = -100;
const int     AS5600_ERROR_I2C_READ_1   = -101;
const int     AS5600_ERROR_I2C_READ_2   = -102;
const int     AS5600_ERROR_I2C_READ_3   = -103;
const int     AS5600_ERROR_I2C_WRITE_0  = -200;
const int     AS5600_ERROR_I2C_WRITE_1  = -201;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef ret;
uint8_t reg_buf[1];
uint8_t buf[12];
uint8_t Data_buf[2];
int16_t val;
float Rad;
float Cumulative_Rad;
int8_t status = 1;

//  for getAngularSpeed()
uint32_t _lastMeasurement = 0;
int16_t  _lastAngle       = 0;

//  for readAngle() and rawAngle()
uint16_t _offset          = 0;

//  cumulative position counter
//  works only if the sensor is read often enough.
int32_t  _position        = 0;
int16_t  _lastPosition    = 0;

uint16_t readReg2(uint8_t reg);
uint16_t readAngle();
int32_t getCumulativePosition();
float getAngularSpeed(uint8_t mode);
float raw2rad(uint16_t value);


//Protected
uint8_t  _address         = AS5600_ADDRESS;
//uint8_t  _directionPin    = 255;
//uint8_t  _direction       = AS5600_CLOCK_WISE;
int      _error           = AS5600_OK;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM16_Init();
	MX_SPI1_Init();
	MX_TIM15_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//		for (int i = 0; i<2; i++){
		//			reg_buf[i] = REG_ANGLE;
		//			ret = HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDRESS, reg_buf, 1, HAL_MAX_DELAY);
		//			if (ret != HAL_OK){
		//				status = -1;
		//			}
		//
		//			ret = HAL_I2C_Master_Receive(&hi2c1, AS5600_ADDRESS, buf, 2, HAL_MAX_DELAY);
		//			if (ret != HAL_OK){
		//				status = -2;
		//			}
		//			else {
		//				status = 1;
		//				val = ((int16_t)buf[0]);
		//			}
		//		}

		val = readAngle();
		Rad = raw2rad(val);

		val = getCumulativePosition();
		Cumulative_Rad = raw2rad(val);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

int32_t resetCumulativePosition(int32_t position)
{
  _lastPosition = readReg2(AS5600_ANGLE) & 0x0FFF;
  int32_t old = _position;
  _position = position;
  return old;
}


int lastError()
{
  int value = _error;
  _error = AS5600_OK;
  return value;
}

int32_t getRevolutions()
{
  int32_t p = _position >> 12;  //  divide by 4096
  return p;
  // if (p < 0) p++;
  // return p;
}


int32_t resetPosition(int32_t position)
{
  int32_t old = _position;
  _position = position;
  return old;
}

float getAngularSpeed(uint8_t mode)
{
  uint32_t now     = HAL_GetTick();
  int      angle   = readAngle();
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

uint16_t readAngle()
{
	uint16_t value = readReg2(AS5600_ANGLE) & 0x0FFF;
	//  if (_offset > 0) value = (value + _offset) & 0x0FFF;
	//
	//  if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
	//      (_direction == AS5600_COUNTERCLOCK_WISE))
	//  {
	//    value = (4096 - value) & 0x0FFF;
	//  }
	return value;
}

int32_t getCumulativePosition()
{
  int16_t value = readReg2(AS5600_ANGLE) & 0x0FFF;

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

float raw2rad(uint16_t value){
	float rad = (value / 4096.0) * 3.14;
	return rad;
}

uint16_t readReg2(uint8_t reg)
{
	_error = AS5600_OK;
	reg_buf[0] = AS5600_ANGLE;
	ret = HAL_I2C_Master_Transmit(&hi2c1, _address, reg_buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		_error = AS5600_ERROR_I2C_READ_2;
		return 0;
	}

	uint16_t data;

	ret = HAL_I2C_Master_Receive(&hi2c1, _address, Data_buf, 2, HAL_MAX_DELAY);
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
/* USER CODE END 4 */


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
