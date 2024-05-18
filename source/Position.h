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
#include "bmi160.h"
#include "cyhal.h"

typedef struct {
	float x_pos;
	float y_pos;
	float z_pos;
} GlobalData;

typedef struct {
    int32_t x_offset;
    int32_t y_offset;
    int32_t z_offset;
    GlobalData position;
} CalibrationData;

extern struct bmi160_dev sensor; // Define the sensor globally

extern CalibrationData calibration_data;

extern GlobalData global_data;

extern SemaphoreHandle_t xMutex;

extern cyhal_spi_t spi_obj;

void globals_init(void);

#endif /* SOURCE_POSITION_H_ */
