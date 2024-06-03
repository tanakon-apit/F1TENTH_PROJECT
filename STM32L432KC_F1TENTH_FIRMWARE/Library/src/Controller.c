/*
 * Controller.c
 *
 *  Created on: Jan 27, 2024
 *      Author: 08809
 */

#include <Controller.h>

/* USER CODE BEGIN SAT */

int32_t PWM_SATUATION(float _u, int32_t _upper_limit, int32_t _lower_limit)
{
	if (_u > _upper_limit) return _upper_limit;
	else if (_u < _lower_limit) return _lower_limit;
	return (int32_t)_u;
}

/* USER CODE END SAT */

/* USER CODE BEGIN PID */

void PID_CONTROLLER_Init(PID_Structure *PIDx, float _Kp, float _Ki, float _Kd, float _Umax)
{
	PIDx->a0 = _Kp + _Ki + _Kd;
	PIDx->a1 = _Kp + (2 * _Kd);
	PIDx->a2 = _Kd;
	PIDx->ek_1 = 0;
	PIDx->ek_2 = 0;
	PIDx->u = 0;
	PIDx->u_max = _Umax;
}

float PID_CONTROLLER_Compute(PID_Structure *PIDx, float ek)
{
	if (!((PIDx->u >= PIDx->u_max && ek > 0) || (PIDx->u <= -1 * PIDx->u_max && ek < 0)))
	{
		PIDx->u += (PIDx->a0 * ek) - (PIDx->a1 * PIDx->ek_1) + (PIDx->a2 * PIDx->ek_2);
	}
	PIDx->ek_2 = PIDx->ek_1;
	PIDx->ek_1 = ek;
	return fmaxf(-PIDx->u_max,fminf(PIDx->u, PIDx->u_max));
}
/* USER CODE END PID */
