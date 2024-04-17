/*
 * pwm_freq.h
 *
 *  Created on: Oct 14, 2023
 *      Author: 08809
 */

#ifndef INC_PWM_FREQ_H_
#define INC_PWM_FREQ_H_

#include "main.h"
#include "math.h"

uint8_t PWMWrite(TIM_HandleTypeDef* htimx, uint16_t tim_chx, float freq, float percent_duty);

uint16_t UnWrapped16_t(uint16_t threshold, uint16_t pn);

#endif /* INC_PWM_FREQ_H_ */

