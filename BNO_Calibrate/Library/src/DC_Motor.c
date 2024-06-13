/*
 * DC_Motor.c
 *
 *  Created on: Jan 27, 2024
 *      Author: 08809
 */

#include "DC_Motor.h"

/* USER CODE BEGIN DC MOTOR */

void DC_MOTOR_IM_Init(DC_MOTOR_MODEL_Structure *Mx, float _freq)
{
	float gain = Mx->U_max / Mx->V_max;

	Mx->a0 = (Mx->J * _freq) / Mx->Kt;
	Mx->a1 = Mx->a0 + (Mx->B / Mx->Kt);
	Mx->a2 = 1.0 / Mx->Kt;

	Mx->b0 = Mx->L * _freq * gain;
	Mx->b1 = Mx->b0 + (Mx->R * gain);
	Mx->b2 = Mx->Ke * gain;

	Mx->ik_1 = 0;
	Mx->qdk_1 = 0;
}

void DC_MOTOR_FM_Init(DC_MOTOR_MODEL_Structure *Mx, float _freq)
{
	float gain = Mx->V_max / Mx->U_max;

	Mx->a0 = gain / (Mx->R + (Mx->L * _freq));
	Mx->a1 = Mx->Ke / (Mx->R + (Mx->L * _freq));
	Mx->a2 = Mx->L * _freq / (Mx->R + (Mx->L * _freq));

	Mx->b0 = 1.0 / (Mx->J * _freq);
	Mx->b1 = Mx->Kt * Mx->b0;
	Mx->b2 = 1.0 - (Mx->B * Mx->b0);

	Mx->ik_1 = 0;
	Mx->qdk_1 = 0;
}

float DC_MOTOR_IM_Compute(DC_MOTOR_MODEL_Structure *Mx, float _qd, float _Tl)
{
	float ik = (Mx->a2 * _Tl) + (Mx->a1 * _qd) - (Mx->a0 * Mx->qdk_1);
	float uk = (Mx->b2 * _qd) + (Mx->b1 * ik) - (Mx->b0 * Mx->ik_1);

	Mx->ik_1 = ik;
	Mx->qdk_1 = _qd;

	if (uk >= Mx->U_max) return Mx->U_max;
	else if (uk <= -1 * Mx->U_max) return -1 * Mx->U_max;
	return uk;
}

float DC_MOTOR_FM_Compute(DC_MOTOR_MODEL_Structure *Mx, float _uk, float _Tl)
{
	if (_uk >= Mx->U_max) _uk = Mx->U_max;
	else if (_uk <= -1 * Mx->U_max) _uk = -1 * Mx->U_max;

	float ik = (Mx->a2 * Mx->ik_1) - (Mx->a1 * Mx->qdk_1) + (Mx->a0 * _uk);
	float qd = (Mx->b2 * Mx->qdk_1) + (Mx->b1 * ik) - (Mx->b0 * _Tl);

	Mx->ik_1 = ik;
	Mx->qdk_1 = qd;
	return qd;
}

/* USER CODE END DC MOTOR */
