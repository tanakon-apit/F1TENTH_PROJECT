/*
 * PAA5160E1.h
 *
 *  Created on: Apr 18, 2024
 *      Author: 08809
 */

#ifndef INC_PAA5160E1_H_
#define INC_PAA5160E1_H_

#include "stm32l4xx_hal.h"
#include "math.h"

/// @brief Default I2C addresses of the Qwiic OTOS
#define kDefaultAddress = 0x17;

/// @brief Minimum scalar value for the linear and angular scalars
//static constexpr float kMinScalar = 0.872f;

/// @brief Maximum scalar value for the linear and angular scalars
//static constexpr float kMaxScalar = 1.127f;

// OTOS register map
#define kRegProductId 0x00;
#define kRegHwVersion 0x01;
#define kRegFwVersion 0x02;
#define kRegScalarLinear 0x04;
#define kRegScalarAngular 0x05;
#define kRegImuCalib 0x06;
#define kRegReset 0x07;
#define kRegSignalProcess 0x0E;
#define kRegSelfTest 0x0F;
#define kRegOffXL 0x10;
#define kRegOffXH 0x11;
#define kRegOffYL 0x12;
#define kRegOffYH 0x13;
#define kRegOffHL 0x14;
#define kRegOffHH 0x15;
#define kRegStatus 0x1F;
#define kRegPosXL 0x20;
#define kRegPosXH 0x21;
#define kRegPosYL 0x22;
#define kRegPosYH 0x23;
#define kRegPosHL 0x24;
#define kRegPosHH 0x25;
#define kRegVelXL 0x26;
#define kRegVelXH 0x27;
#define kRegVelYL 0x28;
#define kRegVelYH 0x29;
#define kRegVelHL 0x2A;
#define kRegVelHH 0x2B;
#define kRegAccXL 0x2C;
#define kRegAccXH 0x2D;
#define kRegAccYL 0x2E;
#define kRegAccYH 0x2F;
#define kRegAccHL 0x30;
#define kRegAccHH 0x31;
#define kRegPosStdXL 0x32;
#define kRegPosStdXH 0x33;
#define kRegPosStdYL 0x34;
#define kRegPosStdYH 0x35;
#define kRegPosStdHL 0x36;
#define kRegPosStdHH 0x37;
#define kRegVelStdXL 0x38;
#define kRegVelStdXH 0x39;
#define kRegVelStdYL 0x3A;
#define kRegVelStdYH 0x3B;
#define kRegVelStdHL 0x3C;
#define kRegVelStdHH 0x3D;
#define kRegAccStdXL 0x3E;
#define kRegAccStdXH 0x3F;
#define kRegAccStdYL 0x40;
#define kRegAccStdYH 0x41;
#define kRegAccStdHL 0x42;
#define kRegAccStdHH 0x43;

// Product ID register value
#define kProductId 0x5F;

// Conversion factors
static const double kMeterToInch = 39.37f;
static const double kInchToMeter = 1.0f / kMeterToInch;
static const double kRadianToDegree = 180.0f / M_PI;
static const double kDegreeToRadian = M_PI / 180.0f;

// Conversion factor for the linear position registers. 16-bit signed
// registers with a max value of 10 meters (394 inches) gives a resolution
// of about 0.0003 mps (0.012 ips)
static const double kMeterToInt16 = 32768.0f / 10.0f;
static const double kInt16ToMeter = 1.0f / kMeterToInt16;

// Conversion factor for the linear velocity registers. 16-bit signed
// registers with a max value of 5 mps (197 ips) gives a resolution of about
// 0.00015 mps (0.006 ips)
static const double kMpsToInt16 = 32768.0f / 5.0f;
static const double kInt16ToMps = 1.0f / kMpsToInt16;

// Conversion factor for the linear acceleration registers. 16-bit signed
// registers with a max value of 157 mps^2 (16 g) gives a resolution of
// about 0.0048 mps^2 (0.49 mg)
static const double kMpssToInt16 = 32768.0f / (16.0f * 9.80665f);
static const double kInt16ToMpss = 1.0f / kMpssToInt16;

// Conversion factor for the angular position registers. 16-bit signed
// registers with a max value of pi radians (180 degrees) gives a resolution
// of about 0.00096 radians (0.0055 degrees)
static const double kRadToInt16 = 32768.0f / M_PI;
static const double kInt16ToRad = 1.0f / kRadToInt16;

// Conversion factor for the angular velocity registers. 16-bit signed
// registers with a max value of 34.9 rps (2000 dps) gives a resolution of
// about 0.0011 rps (0.061 degrees per second)
static const double kRpsToInt16 = 32768.0f / (2000.0f * kDegreeToRadian);
static const double kInt16ToRps = 1.0f / kRpsToInt16;

// Conversion factor for the angular acceleration registers. 16-bit signed
// registers with a max value of 3141 rps^2 (180000 dps^2) gives a
// resolution of about 0.096 rps^2 (5.5 dps^2)
static const double kRpssToInt16 = 32768.0f / (M_PI * 1000.0f);
static const double kInt16ToRpss = 1.0f / kRpssToInt16;

/// @struct sfe_otos_pose2d_t
/// @brief 2D pose structure, including x and y coordinates and heading angle
/// @note Although pose is traditionally used for position and orientation, this
/// structure is also used for velocity and accleration by the OTOS driver
typedef struct
{
	/// @brief X value
	float x;

	/// @brief Y value
	float y;

	/// @brief Heading value
	float h;
} sfe_otos_pose2d_t;

/// @enum sfe_otos_linear_unit_t
/// @brief Enumerations for linear units used by the OTOS driver
typedef enum
{
	/// @brief Meters
	kSfeOtosLinearUnitMeters = 0,

	/// @brief Inches (default)
	kSfeOtosLinearUnitInches = 1
} sfe_otos_linear_unit_t;

/// @enum sfe_otos_angular_unit_t
/// @brief Enumerations for angular units used by the OTOS driver
typedef enum
{
	/// @brief Radians
	kSfeOtosAngularUnitRadians = 0,

	/// @brief Degrees (default)
	kSfeOtosAngularUnitDegrees = 1
} sfe_otos_angular_unit_t;

/// @union sfe_otos_version_t
/// @brief Version register bit fields
typedef union {
	struct
	{
		/// @brief Minor version number
		uint8_t minor : 4;

		/// @brief Major version number
		uint8_t major : 4;
	};

	/// @brief Raw register value
	uint8_t value;
} sfe_otos_version_t;

/// @union sfe_otos_signal_process_config_t
/// @brief Signal process config register bit fields
typedef union {
	struct
	{
		/// @brief Whether to use the internal lookup table calibration for the
		/// optical sensor
		uint8_t enLut : 1;

		/// @brief Whether to feed the accelerometer data to the Kalman filters
		uint8_t enAcc : 1;

		/// @brief Whether to rotate the IMU and optical sensor data by the
		/// heading angle
		uint8_t enRot : 1;

		/// @brief Whether to use the correct sensor variance in the Kalman
		/// filters, or use 0 varaince to effectively disable the filters
		uint8_t enVar : 1;

		/// @brief Reserved bits, do not use
		uint8_t reserved : 4;
	};

	/// @brief Raw register value
	uint8_t value;
} sfe_otos_signal_process_config_t;

/// @union sfe_otos_self_test_config_t
/// @brief Self test register bit fields
typedef union {
	struct
	{
		/// @brief Write 1 to start the self test
		uint8_t start : 1;

		/// @brief Returns 1 while the self test is in progress
		uint8_t inProgress : 1;

		/// @brief Returns 1 if the self test passed
		uint8_t pass : 1;

		/// @brief Returns 1 if the self test failed
		uint8_t fail : 1;

		/// @brief Reserved bits, do not use
		uint8_t reserved : 4;
	};

	/// @brief Raw register value
	uint8_t value;
} sfe_otos_self_test_config_t;

/// @union sfe_otos_status_t
/// @brief Status register bit fields
typedef union {
	struct
	{
		/// @brief Returns 1 if the tilt angle threshold has been exceeded.
		/// While set, the accelerometer data is ignored
		uint8_t warnTiltAngle : 1;

		/// @brief Returns 1 if the optical tracking is unreliable. While set,
		/// only the IMU data is used for tracking unless warnTiltAngle is set
		uint8_t warnOpticalTracking : 1;

		/// @brief Reserved bits, do not use
		uint8_t reserved : 4;

		/// @brief Returns 1 if the optical sensor has a fatal error
		uint8_t errorPaa : 1;

		/// @brief Returns 1 if the IMU has a fatal error
		uint8_t errorLsm : 1;
	};

	/// @brief Raw register value
	uint8_t value;
} sfe_otos_status_t;

typedef struct {
	HAL_StatusTypeDef status;
	uint8_t maxSample;
	uint8_t numSample;
}PAA5160E1_Calibration_Stat;

typedef struct {
	I2C_HandleTypeDef *hi2cx;
	uint8_t address;
	uint8_t mode;
	HAL_StatusTypeDef flag;
	sfe_otos_pose2d_t pos;
	sfe_otos_pose2d_t vel;
	sfe_otos_pose2d_t acc;
	sfe_otos_pose2d_t posStdDev;
	sfe_otos_pose2d_t velStdDev;
	sfe_otos_pose2d_t accStdDev;
	uint8_t RxBuffer[44];
	//PAA5160E1_offsets offsets;
	PAA5160E1_Calibration_Stat Calibration_Stat;
}PAA5160E1_Structure;

/// @brief Begins the Qwiic OTOS and verifies it is connected
/// @param commBus I2C bus to use for communication
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_Init(PAA5160E1_Structure *paa, I2C_HandleTypeDef *hi2cx, uint8_t addr, OPRMode mode, uint8_t unit);

/// @brief Checks if the device is connected
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_isConnected();

/// @brief Gets the hardware and firmware version numbers from the OTOS
/// @param hwVersion Hardware version number
/// @param fwVersion Firmware version number
/// @return 0 for succuss, negative for errors, positive for warnings
// sfeTkError_t getVersionInfo(sfe_otos_version_t &hwVersion, sfe_otos_version_t &fwVersion);

/// @brief Performs a self test of the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
// sfeTkError_t selfTest();

/// @brief Calibrates the IMU on the OTOS, which removes the accelerometer
/// and gyroscope offsets
/// @param numSamples Number of samples to take for calibration. Each sample
/// takes about 2.4ms, so fewer samples can be taken for faster calibration
/// @param waitUntilDone Whether to wait until the calibration is complete.
/// Set false to calibrate asynchronously, see getImuCalibrationProgress()
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_calibrateImu(PAA5160E1_Structure *paa, uint8_t numSamples, uint8_t waitUntilDone);

/// @brief Gets the progress of the IMU calibration. Used for asynchronous
/// calibration with calibrateImu()
/// @param numSamples Number of samples remaining for calibration
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getImuCalibrationProgress(PAA5160E1_Structure *paa);

/// @brief Gets the linear scalar used by the OTOS
/// @param scalar Linear scalar
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getLinearScalar(PAA5160E1_Structure *paa);

/// @brief Sets the linear scalar used by the OTOS. Can be used to
/// compensate for scaling issues with the sensor measurements
/// @param scalar Linear scalar, must be between 0.872 and 1.127
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_setLinearScalar(PAA5160E1_Structure *paa, float scalar);

/// @brief Gets the angular scalar used by the OTOS
/// @param scalar Angular scalar
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getAngularScalar(PAA5160E1_Structure *paa);

/// @brief Sets the angular scalar used by the OTOS. Can be used to
/// compensate for scaling issues with the sensor measurements
/// @param scalar Angular scalar, must be between 0.872 and 1.127
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_setAngularScalar(PAA5160E1_Structure *paa, float scalar);

/// @brief Resets the tracking algorithm, which resets the position to the
/// origin, but can also be used to recover from some rare tracking errors
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef resetTracking();

/// @brief Gets the signal processing configuration from the OTOS
/// @param config Signal processing configuration
/// @return 0 for succuss, negative for errors, positive for warnings
// sfeTkError_t getSignalProcessConfig(sfe_otos_signal_process_config_t &config);

/// @brief Sets the signal processing configuration on the OTOS. This is
/// primarily useful for creating and testing a new lookup table calibration
/// @param config Signal processing configuration
/// @return 0 for succuss, negative for errors, positive for warnings
// sfeTkError_t setSignalProcessConfig(sfe_otos_signal_process_config_t &config);

/// @brief Gets the status register from the OTOS, which includes warnings
/// and errors reported by the sensor
/// @param status Status register value
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getStatus(PAA5160E1_Structure *paa);

/// @brief Gets the offset of the OTOS
/// @param pose Offset of the sensor relative to the center of the robot
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getOffset(PAA5160E1_Structure *paa);

/// @brief Sets the offset of the OTOS. This is useful if your sensor is
/// mounted off-center from a robot. Rather than returning the position of
/// the sensor, the OTOS will return the position of the robot
/// @param pose Offset of the sensor relative to the center of the robot
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_setOffset(PAA5160E1_Structure *paa);

/// @brief Gets the position measured by the OTOS
/// @param pose Position measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getPosition(PAA5160E1_Structure *paa);

/// @brief Sets the position measured by the OTOS. This is useful if your
/// robot does not start at the origin, or you have another source of
/// location information (eg. vision odometry); the OTOS will continue
/// tracking from this position
/// @param pose New position for the OTOS to track from
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_setPosition(PAA5160E1_Structure *paa);

/// @brief Gets the velocity measured by the OTOS
/// @param pose Velocity measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getVelocity(PAA5160E1_Structure *paa);

/// @brief Gets the acceleration measured by the OTOS
/// @param pose Acceleration measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getAcceleration(PAA5160E1_Structure *paa);

/// @brief Gets the standard deviation of the measured position
/// @param pose Standard deviation of the position measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
/// @note These values are just the square root of the diagonal elements of
/// the covariance matrices of the Kalman filters used in the firmware, so
/// they are just statistical quantities and do not represent actual error!
HAL_StatusTypeDef PAA5160E1_getPositionStdDev(PAA5160E1_Structure *paa);

/// @brief Gets the standard deviation of the measured velocity
/// @param pose Standard deviation of the velocity measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
/// @note These values are just the square root of the diagonal elements of
/// the covariance matrices of the Kalman filters used in the firmware, so
/// they are just statistical quantities and do not represent actual error!
HAL_StatusTypeDef PAA5160E1_getVelocityStdDev(PAA5160E1_Structure *paa);

/// @brief Gets the standard deviation of the measured acceleration
/// @param pose Standard deviation of the acceleration measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
/// @note These values are just the square root of the diagonal elements of
/// the covariance matrices of the Kalman filters used in the firmware, so
/// they are just statistical quantities and do not represent actual error!
HAL_StatusTypeDef PAA5160E1_getAccelerationStdDev(PAA5160E1_Structure *paa);

/// @brief Gets the position, velocity, and acceleration measured by the
/// OTOS in a single burst read
/// @param pos Position measured by the OTOS
/// @param vel Velocity measured by the OTOS
/// @param acc Acceleration measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getPosVelAcc(PAA5160E1_Structure *paa);

/// @brief Gets the standard deviation of the measured position, velocity,
/// and acceleration in a single burst read
/// @param pos Standard deviation of the position measured by the OTOS
/// @param vel Standard deviation of the velocity measured by the OTOS
/// @param acc Standard deviation of the acceleration measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getPosVelAccStdDev(PAA5160E1_Structure *paa);

/// @brief Gets the position, velocity, acceleration, and standard deviation
/// of each in a single burst read
/// @param pos Position measured by the OTOS
/// @param vel Velocity measured by the OTOS
/// @param acc Acceleration measured by the OTOS
/// @param posStdDev Standard deviation of the position measured by the OTOS
/// @param velStdDev Standard deviation of the velocity measured by the OTOS
/// @param accStdDev Standard deviation of the acceleration measured by the OTOS
/// @return 0 for succuss, negative for errors, positive for warnings
HAL_StatusTypeDef PAA5160E1_getPosVelAccAndStdDev(PAA5160E1_Structure *paa);

// Function to read raw pose registers and convert to specified units
HAL_StatusTypeDef PAA5160E1_readPoseRegs(PAA5160E1_Structure *paa, float rawToXY, float rawToH);

// Function to write raw pose registers and convert from specified units
HAL_StatusTypeDef PAA5160E1_writePoseRegs(PAA5160E1_Structure *paa, float xyToRaw, float hToRaw);

// Function to convert raw pose registers to a pose structure
void PAA5160E1_regsToPose(PAA5160E1_Structure *paa, uint8_t *rawData, float rawToXY, float rawToH);

// Function to convert a pose structure to raw pose registers
void PAA5160E1_poseToRegs(PAA5160E1_Structure *paa, uint8_t *rawData, float xyToRaw, float hToRaw);


#endif /* INC_PAA5160E1_H_ */