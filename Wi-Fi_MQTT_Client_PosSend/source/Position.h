/*
 * Position.h
 *
 *  Created on: 18 May 2024
 *      Author: Damjan
*/

#ifndef SOURCE_POSITION_H_
#define SOURCE_POSITION_H_

#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
	float x_pos;
	float y_pos;
	float z_pos;
} GlobalData;


extern char outputbuffer[20];
extern GlobalData global_data;
extern SemaphoreHandle_t xMutex;

void globals_init(void);

#endif /* SOURCE_POSITION_H_ */


