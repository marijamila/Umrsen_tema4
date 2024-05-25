/*
 * Position.c
 *
 *  Created on: 18 May 2024
 *      Author: Damjan

*/


#include "Position.h"

GlobalData global_data={0.0,0.0,0.0};

char outputbuffer[20]="Example output";

SemaphoreHandle_t xMutex;

void globals_init(void) {

    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        // Handle error
    }
}

