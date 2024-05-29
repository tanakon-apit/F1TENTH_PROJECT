/*
 * Tamiya.h
 *
 *  Created on: May 16, 2024
 *      Author: tanakon
 */

#ifndef INC_TAMIYA_H_
#define INC_TAMIYA_H_

#include "RC.h"

typedef enum {
	Forward,
	Backward,
	Brake
}Direction_State;

typedef struct {
	RC_Structure RCx;
	Direction_State dir;
	float negative_death_zone;
	float positive_death_zone;
}Tamiya_Structure;

#endif /* INC_TAMIYA_H_ */
