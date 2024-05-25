/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for MQTT Client Example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes */
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mqtt_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bmi160.h"

#include "Position.h"

#include "DSP310.h"
#include "i2c.h"

cyhal_spi_t spi_obj;

#define SPI_MOSI CYBSP_SPI_MOSI
#define SPI_MISO CYBSP_SPI_MISO
#define SPI_CLK  CYBSP_SPI_CLK
#define SPI_CS   CYBSP_SPI_CS

int8_t bmi160_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    cy_rslt_t result;
    uint8_t tx_buffer[len + 1];
    uint8_t rx_buffer[len + 1];

    tx_buffer[0] = reg_addr | 0x80; // Set read bit
    cyhal_gpio_write(dev_id, 0); // Assert CS

    result = cyhal_spi_transfer(&spi_obj, tx_buffer, len + 1, rx_buffer, len + 1, 0);
    if (result != CY_RSLT_SUCCESS) {
        cyhal_gpio_write(dev_id, 1); // Deassert CS
        return -1;
    }
    cyhal_gpio_write(dev_id, 1); // Deassert CS

    memcpy(data, &rx_buffer[1], len);
    return 0;
}

int8_t bmi160_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    cy_rslt_t result;
    uint8_t buffer[len + 1];

    buffer[0] = reg_addr & 0x7F; // Clear read bit
    memcpy(&buffer[1], data, len);
    cyhal_gpio_write(dev_id, 0); // Assert CS

    result = cyhal_spi_transfer(&spi_obj, buffer, len + 1, NULL, 0, 0);
    if (result != CY_RSLT_SUCCESS) {
        cyhal_gpio_write(dev_id, 1); // Deassert CS
        return -1;
    }
    cyhal_gpio_write(dev_id, 1); // Deassert CS

    return 0;
}

void bmi160_delay_ms(uint32_t period) {
    cyhal_system_delay_ms(period);
}

//Kalmann filter
#define BMI160_SAMPLE_RATE_HZ 100
#define GRAVITY_ACCELERATION 9.81f
#define INITIAL_CALIBRATION_SAMPLES 400
#define ACCEL_NOISE_DENSITY 250e-6

typedef struct {
    float q;  // Process noise covariance
    float r;  // Measurement noise covariance
    float x;  // Estimated value
    float p;  // Estimation error covariance
    float k;  // Kalman gain
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter* kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1.0;  // Initial estimation error covariance
    kf->k = 0.0;  // Initial Kalman gain
}

float KalmanFilter_Update(KalmanFilter* kf, float measurement) {
    kf->p += kf->q;
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1.0 - kf->k);
    return kf->x;
}

void bmi160_task(void *pvParameters) {
    cy_rslt_t result;
    struct bmi160_dev sensor;
    struct bmi160_sensor_data accel;

    // Initialize SPI
    result = cyhal_spi_init(&spi_obj, SPI_MOSI, SPI_MISO, SPI_CLK, NC, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);
    if (result != CY_RSLT_SUCCESS) {
        printf("SPI initialization failed\n");
        vTaskSuspend(NULL);
    }

    result = cyhal_gpio_init(SPI_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1); // CS initially high
    if (result != CY_RSLT_SUCCESS) {
        printf("GPIO initialization failed\n");
        vTaskSuspend(NULL);
    }

    // Initialize BMI160 sensor
    sensor.id = SPI_CS;
    sensor.intf = BMI160_SPI_INTF;
    sensor.read = bmi160_spi_read;
    sensor.write = bmi160_spi_write;
    sensor.delay_ms = bmi160_delay_ms;

    result = bmi160_init(&sensor);
    if (result != BMI160_OK) {
        printf("BMI160 initialization failed\n");
        vTaskSuspend(NULL);
    }

    // Configure the sensor for accelerometer and gyroscope
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    result = bmi160_set_sens_conf(&sensor);
    if (result != BMI160_OK) {
        printf("BMI160 sensor configuration failed\n");
        vTaskSuspend(NULL);
    }


	//Initial calibration
    float initial_x_accel = 0.0;
    float initial_y_accel = 0.0;
    float initial_z_accel = 0.0;
	for (int i = 0; i < INITIAL_CALIBRATION_SAMPLES; i++) {
		result = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);
		if (result == BMI160_OK) {
			initial_x_accel += accel.x / 16384.0f * GRAVITY_ACCELERATION; // Convert to m/s^2
			initial_y_accel += accel.y / 16384.0f * GRAVITY_ACCELERATION; // Convert to m/s^2
			initial_z_accel += accel.z / 16384.0f * GRAVITY_ACCELERATION; // Convert to m/s^2
		}
		vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between samples
	}

	// Compute the average initial readings
	initial_x_accel /= INITIAL_CALIBRATION_SAMPLES;
	initial_y_accel /= INITIAL_CALIBRATION_SAMPLES;
	initial_z_accel /= INITIAL_CALIBRATION_SAMPLES;
	printf("Initial calibration complete: X = %.2f, Y = %.2f, Z = %.2f m/s^2\n",
	           initial_x_accel, initial_y_accel, initial_z_accel);

	float measurement_noise_covariance = ACCEL_NOISE_DENSITY * ACCEL_NOISE_DENSITY * BMI160_SAMPLE_RATE_HZ;

	 KalmanFilter kf_x, kf_y, kf_z;
		KalmanFilter_Init(&kf_x, 0.01, measurement_noise_covariance, initial_x_accel);
		KalmanFilter_Init(&kf_y, 0.01, measurement_noise_covariance, initial_y_accel);
		KalmanFilter_Init(&kf_z, 0.01, measurement_noise_covariance, initial_z_accel);


    // Read accelerometer data in a loop
    while (1) {
        result = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);
        if (result == BMI160_OK) {
        	float accel_x = accel.x / 16384.0f * GRAVITY_ACCELERATION; // Convert to m/s^2
			float accel_y = accel.y / 16384.0f * GRAVITY_ACCELERATION; // Convert to m/s^2
			float accel_z = (accel.z / 16384.0f * GRAVITY_ACCELERATION); // Convert to m/s^2 and compensate for gravity



			accel_x -= initial_x_accel;
			accel_y -= initial_y_accel;
			accel_z -= initial_z_accel;

			float filtered_x = KalmanFilter_Update(&kf_x, accel_x);
			float filtered_y = KalmanFilter_Update(&kf_y, accel_y);
			float filtered_z = KalmanFilter_Update(&kf_z, accel_z);

            printf("Position X: %.3f, Y: %.3f, Z: %.3f\n", filtered_x, filtered_y, filtered_z);
            //printf("Waiting on semaphore\n\r");
                if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                	global_data.x_pos=global_data.x_pos+filtered_x*0.01*0.01;
                	global_data.y_pos=global_data.y_pos+filtered_y*0.01*0.01;
                	global_data.z_pos=global_data.z_pos+filtered_z*0.01*0.01;
                	xSemaphoreGive(xMutex);
                }

        } else {
            printf("Failed to get sensor data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main(void) {
    cy_rslt_t result;

    /* Initialize the board support package. */
    result = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* Enable global interrupts. */
    __enable_irq();

    /* Initilize the semaphore */
   globals_init();

    /* Initialize retarget-io to use the debug UART port. */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    i2c_master_init();
    DPS310_start_measurement();

    printf("Starting MQTT Client and BMI160 Example\n");

    /* Create the MQTT Client task. */
    result = xTaskCreate(mqtt_client_task, "MQTT Client task", MQTT_CLIENT_TASK_STACK_SIZE, NULL, MQTT_CLIENT_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        printf("Failed to create MQTT Client task\n");
        CY_ASSERT(0);
    }

    /* Create the BMI160 task. */
    result = xTaskCreate(bmi160_task, "BMI160 Task", 1024, NULL, 2, NULL);
    if (result != pdPASS) {
        printf("Failed to create BMI160 task\n");
        CY_ASSERT(0);
    }

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Should never get here. */
    CY_ASSERT(0);
}
