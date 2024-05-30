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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AS5600.h"
#include "BNO055.h"
#include "RC.h"
#include "Mobile_Config.h"
#include "stdbool.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	forward,
	backward_1,
	backward_2,
	brake
} direction_state;

typedef struct {
	int16_t prev_counter;
	double gain;
	double unwrap_pos;
	double pos;
}encoder_counter;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_ON
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HALCHECK(fn) while(fn != HAL_OK) HAL_Delay(100);
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Error_Handler()}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float servo_cmd = 0;
float bldc_cmd = 0;
direction_state state = brake;
encoder_counter enc = {
		.prev_counter = 0.0,
		.gain = M_PI_2,
		.unwrap_pos = 0.0,
		.pos = 0.0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void enc_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void subscription_callback(const void * msgin);

float vel2rc(float speed);
float ang2rc(float ang);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void enc_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	static bool isfirst_callback = true;
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{

#ifdef SENSOR_ON
		switch (state)
		{
		case forward:
			if (bldc_cmd > 0) RC_Write(&bldc, vel2rc(bldc_cmd));
			else {
				RC_Write(&bldc, vel2rc(0));
				state = brake;
			}
			break;
		case brake:
			RC_Write(&bldc, vel2rc(bldc_cmd));
			if (bldc_cmd > 0) state = forward;
			else if (bldc_cmd < 0) state = backward_1;
			break;
		case backward_1:
			RC_Write(&bldc, vel2rc(0));
			if (bldc_cmd < 0) state = backward_2;
			else state = brake;
			break;
		case backward_2:
			if (bldc_cmd < 0) RC_Write(&bldc, vel2rc(bldc_cmd));
			else {
				RC_Write(&bldc, vel2rc(0));
				state = brake;
			}
			break;
		}
		RC_Write(&servo, ang2rc(servo_cmd));

		int16_t counter = __HAL_TIM_GET_COUNTER(&htim1);
		int32_t delta_counter = counter - enc.prev_counter;
		enc.prev_counter = counter;

		if (delta_counter < -32768) enc.pos += (65535.0 * enc.gain);
		else if (delta_counter > 32768) enc.pos -= (65535.0 * enc.gain);
		enc.unwrap_pos = counter * enc.gain; //(counter * enc.gain) + enc.pos;

		enc_msg.data.data[0] = enc.unwrap_pos;
#endif
		if (!isfirst_callback) RCSOFTCHECK(rcl_publish(&enc_publisher, &enc_msg, NULL))
		else isfirst_callback = !isfirst_callback;
	}
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	static bool isfirst_callback = true;
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
#ifdef SENSOR_ON
		if (bno.flag == HAL_OK)
		{
			BNO055_Read_DMA(&bno, 0);
			bno.flag = HAL_BUSY;
		}
		imu_msg.data.data[0] = bno.gyro.x;
		imu_msg.data.data[1] = bno.gyro.y;
		imu_msg.data.data[2] = bno.gyro.z;
		imu_msg.data.data[3] = bno.lin_acc.x;
		imu_msg.data.data[4] = bno.lin_acc.y;
		imu_msg.data.data[5] = bno.lin_acc.z;
		imu_msg.data.data[6] = bno.quat.x;
		imu_msg.data.data[7] = bno.quat.y;
		imu_msg.data.data[8] = bno.quat.z;
		imu_msg.data.data[9] = bno.quat.w;
#endif
		if (!isfirst_callback) RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL))
		else isfirst_callback = !isfirst_callback;
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Float64MultiArray *cmd_msg = (const std_msgs__msg__Float64MultiArray *)msgin;

	servo_cmd = cmd_msg->data.data[0];
	bldc_cmd = cmd_msg->data.data[1];
}

void StartDefaultTask(void *argument)
{
	rmw_uros_set_custom_transport(
			true,
			(void *) &huart2,
			cubemx_transport_open,
			cubemx_transport_close,
			cubemx_transport_write,
			cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;
	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) printf("Error on default allocators (line %d)\n", __LINE__);

	allocator = rcl_get_default_allocator();
	rclc_support_init(&support, 0, NULL, &allocator);
	rclc_node_init_default(&node, "mcu_node", "", &support);

	rclc_publisher_init_default(
			&enc_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
			"enc_raw");
	rclc_publisher_init_default(
			&imu_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
			"imu_raw");

	rclc_subscription_init_default(
			&cmd_subscription,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
			"mcu_cmd");

	rclc_timer_init_default(
			&enc_timer,
			&support,
			RCL_MS_TO_NS(20),
			enc_timer_callback);

	rclc_timer_init_default(&imu_timer,
			&support,
			RCL_MS_TO_NS(10),
			imu_timer_callback);

	enc_msg.data.capacity = 1;
	enc_msg.data.data = (double*) malloc(enc_msg.data.capacity * sizeof(double));
	enc_msg.data.size = 1;

	imu_msg.data.capacity = 10;
	imu_msg.data.data = (double*) malloc(imu_msg.data.capacity * sizeof(double));
	imu_msg.data.size = 10;

	cmd_msg.data.capacity = 2;
	cmd_msg.data.data = (double*) malloc(cmd_msg.data.capacity * sizeof(double));
	cmd_msg.data.size = 2;

	rclc_executor_init(&executor, &support.context, 3, &allocator);
	rclc_executor_add_subscription(&executor, &cmd_subscription, &cmd_msg, &subscription_callback, ON_NEW_DATA);
	rclc_executor_add_timer(&executor, &enc_timer);
	rclc_executor_add_timer(&executor, &imu_timer);
	rclc_executor_spin(&executor);

	while(1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
	}
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
#ifdef SENSOR_ON
  HALCHECK(BNO055_Init(&bno, &hi2c1, 0, NDOF))
#ifdef BNO_CALIB_ON
  BNO055_Calibrated(&bno, &bno_stat, &bno_off);
#endif
  BNO055_SetOffsets(&bno, &bno_off);
  BNO055_SetAxis(&bno, P0_Config, P0_Sign);
  HALCHECK(RC_Init(&servo, &htim15, TIM_CHANNEL_1, CPU_FREQ, true))
  HALCHECK(RC_Init(&bldc, &htim15, TIM_CHANNEL_2, CPU_FREQ, false))
  RC_Set_Input_Range(&servo, 0.5, 2.5);
  RC_Set_Input_Range(&bldc, 0.5, 2.5);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == bno.hi2cx->Instance) bno.flag = HAL_OK;
}

float vel2rc(float speed)
{
	float rc_signal = 1.5;
	if (speed > 0) {
		rc_signal = fminf((2.0e-7 * pow(speed, 2)) - (2.0e-5 * speed) + 1.5074, 2.0);
		if (rc_signal < 1.55) rc_signal = 1.5;
	} else if (speed < 0) {
		rc_signal = fmaxf(-(3.0e-7 * pow(speed, 2)) + (6.0e-5 * speed) + 1.473, 1.0);
		if (rc_signal > 1.4) rc_signal = 1.5;
	}
	return rc_signal;
}

float ang2rc(float ang)
{
	float rc_signal = 1.35;
	rc_signal += ang / M_PI_2;
	rc_signal = fmaxf(fminf(rc_signal, 1.794), -0.905);
	return rc_signal;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
