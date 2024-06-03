/*
 * Mobile_Config.h
 *
 *  Created on: May 16, 2024
 *      Author: tanakon
 */

#ifndef INC_MOBILE_CONFIG_H_
#define INC_MOBILE_CONFIG_H_

#define CPU_FREQ 80*1.0e6

#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "uxr/client/transport.h"
#include "rmw_microxrcedds_c/config.h"
#include "rmw_microros/rmw_microros.h"

#include "std_msgs/msg/float64_multi_array.h"

extern rclc_support_t support;
extern rclc_executor_t executor;
extern rcl_allocator_t allocator;

extern rcl_node_t node;

extern rcl_publisher_t enc_publisher;
extern rcl_publisher_t imu_publisher;

extern rcl_subscription_t cmd_subscription;

extern rcl_timer_t enc_timer;
extern rcl_timer_t imu_timer;

extern std_msgs__msg__Float64MultiArray enc_msg;
extern std_msgs__msg__Float64MultiArray imu_msg;
extern std_msgs__msg__Float64MultiArray cmd_msg;

#include "AS5600.h"
#include "BNO055.h"
#include "RC.h"
#include "Controller.h"

#define BNO_CALIB_OFF

#define BNO_ACC_OFF_X 8
#define BNO_ACC_OFF_Y 17
#define BNO_ACC_OFF_Z -17

#define BNO_MAG_OFF_X -396
#define BNO_MAG_OFF_Y 179
#define BNO_MAG_OFF_Z -221

#define BNO_GYRO_OFF_X -2
#define BNO_GYRO_OFF_Y 1
#define BNO_GYRO_OFF_Z 0

#define BNO_ACC_RAD 1000
#define BNO_MAG_RAD 1207

extern AS5600_Structure as5600;

extern BNO055_Structure bno;
extern BNO055_Offsets bno_off;
#ifdef BNO_CALIB_ON
extern BNO055_Calibration_Status bno_stat;
#endif

extern RC_Structure servo;
extern RC_Structure bldc;

extern PID_Structure pid;

#endif /* INC_MOBILE_CONFIG_H_ */
