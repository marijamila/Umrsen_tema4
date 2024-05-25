/*
 * positionReadingTask.c
 *
 *  Created on: 18 May 2024
 *      Author: Damjan



#include "Position.h"
#include "positionTask.h"

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "bmi160.h"

#include "FreeRTOS.h"

void sensorTask(void *params) {
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    // Variables to store position and velocity
    float x_vel = 0.0;
    float y_vel = 0.0;
    float z_vel = 0.0;
    float dt = 0.01;

    while (1) {
        int8_t result = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &accel, &gyro, &sensor);
        if (result == BMI160_OK) {
            int32_t x_acc = accel.x - calibration_data.x_offset;
            int32_t y_acc = accel.y - calibration_data.y_offset;
            int32_t z_acc = accel.z - calibration_data.z_offset;

            float x_acc_g = x_acc / 16384.0; // Assuming 2G range
            float y_acc_g = y_acc / 16384.0;
            float z_acc_g = z_acc / 16384.0;

            float x_gyro_dps = gyro.x / 16.4; // Assuming 2000 DPS range
            float y_gyro_dps = gyro.y / 16.4;
            float z_gyro_dps = gyro.z / 16.4;

            // Integrate acceleration to get velocity
            x_vel += x_acc_g * dt;
            y_vel += y_acc_g * dt;
            z_vel += z_acc_g * dt;

            // Integrate velocity to get position
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            	global_data.x_pos += x_vel * dt;
            	global_data.y_pos += y_vel * dt;
            	global_data.z_pos += z_vel * dt;
                xSemaphoreGive(xMutex);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

 */
