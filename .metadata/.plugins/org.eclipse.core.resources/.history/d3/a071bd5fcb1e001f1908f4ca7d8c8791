/*
 * Controller.h
 *
 *  Created on: Jan 27, 2024
 *      Author: 08809
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "stdint.h"

/* USER CODE BEGIN SAT */

int32_t PWM_SATUATION(float _u, int32_t _upper_limit, int32_t _lower_limit);

/* USER CODE END SAT */

/* USER CODE BEGIN PID */

typedef struct
{
	/* PID COEFFICIENT BEGIN */
	float a0;
	float a1;
	float a2;
	/* PID COEFFICIENT END */

	/* PID STATE BEGIN */
	float ek_1;
	float ek_2;
	float u;
	float u_max;
	/* PID STATE END */
} PID_Structure;

void PID_CONTROLLER_Init(PID_Structure *PIDx, float _Kp, float _Ki, float _Kd, float _Umax);

float PID_CONTROLLER_Compute(PID_Structure *PIDx, float ek);

/* USER CODE END PID */

#endif /* INC_CONTROLLER_H_ */
