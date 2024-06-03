/*
 * Mobile_Config.c
 *
 *  Created on: May 16, 2024
 *      Author: tanakon
 */

#include "Mobile_Config.h"

rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_node_t node;

rcl_publisher_t enc_publisher;
rcl_publisher_t imu_publisher;

rcl_subscription_t cmd_subscription;

rcl_timer_t enc_timer;
rcl_timer_t imu_timer;

std_msgs__msg__Float64MultiArray enc_msg;
std_msgs__msg__Float64MultiArray imu_msg;
std_msgs__msg__Float64MultiArray cmd_msg;

AS5600_Structure as5600;

BNO055_Structure bno;
BNO055_Offsets bno_off = {
		.accel_offset_x = BNO_ACC_OFF_X,
		.accel_offset_y = BNO_ACC_OFF_Y,
		.accel_offset_z = BNO_ACC_OFF_Z,
		.mag_offset_x = BNO_MAG_OFF_X,
		.mag_offset_y = BNO_MAG_OFF_Y,
		.mag_offset_z = BNO_MAG_OFF_Z,
		.gyro_offset_x = BNO_GYRO_OFF_X,
		.gyro_offset_y = BNO_GYRO_OFF_Y,
		.gyro_offset_z = BNO_GYRO_OFF_Z,
		.accel_radius = BNO_ACC_RAD,
		.mag_radius = BNO_MAG_RAD
};
#ifdef BNO_CALIB_ON
BNO055_Calibration_Status bno_stat;
#endif

RC_Structure servo;
RC_Structure bldc;


