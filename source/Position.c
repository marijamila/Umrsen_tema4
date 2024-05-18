/*
 * Position.c
 *
 *  Created on: 18 May 2024
 *      Author: Damjan
 */



#include "Position.h"

GlobalData global_data = {0.0,0.0,0.0};
CalibrationData calibration_data = {0.0, 0.0, 0.0};

struct bmi160_dev sensor; // Define the sensor globally

SemaphoreHandle_t xMutex;

cyhal_spi_t spi_obj;

void globals_init(void) {
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        // Handle error
    }
}
