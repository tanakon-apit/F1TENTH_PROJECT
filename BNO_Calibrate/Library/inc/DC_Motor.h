/*
 * DC_Motor.h
 *
 *  Created on: Jan 27, 2024
 *      Author: 08809
 */

#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

/* USER CODE BEGIN DC MOTOR */

typedef struct
{
	/* DC MOTOR CONSTANT BEGIN */
	float Ke;
	float Kt;
	float L;
	float R;
	float J;
	float B;
	float V_max;
	float U_max;
	/* DC MOTOR CONSTANT END */

	/* DC MOTOR COEFFICIENT BEGIN */
	float a0;
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	/* DC MOTOR COEFFICIENT END */

	/* DC MOTOR STATE BEGIN */
	float qdk_1;
	float ik_1;
	/* DC MOTOR STATE EEND */
} DC_MOTOR_MODEL_Structure;

void DC_MOTOR_IM_Init(DC_MOTOR_MODEL_Structure *Mx, float _freq);

void DC_MOTOR_FM_Init(DC_MOTOR_MODEL_Structure *Mx, float _freq);

float DC_MOTOR_IM_Compute(DC_MOTOR_MODEL_Structure *Mx, float _qd, float _Tl);

float DC_MOTOR_FM_Compute(DC_MOTOR_MODEL_Structure *Mx, float _uk, float _Tl);

/* USER CODE END DC MOTOR */

#endif /* INC_DC_MOTOR_H_ */
