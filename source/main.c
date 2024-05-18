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

#include "positionTask.h"
#include "Position.h"
#include "bmi160.h"
/* Include serial flash library and QSPI memory configurations only for the
 * kits that require the Wi-Fi firmware to be loaded in external QSPI NOR flash.
 */
#if defined(CY_DEVICE_PSOC6A512K)
#include "cy_serial_flash_qspi.h"
#include "cycfg_qspi_memslot.h"
#endif

/******************************************************************************
 * Function Name: main
 ******************************************************************************
 * Summary:
 *  System entrance point. This function initializes retarget IO, sets up 
 *  the MQTT client task, and then starts the RTOS scheduler.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main()
{

	// SPI and UART object

    cy_rslt_t result;

    /* Initialize the board support package. */
    
    result = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* To avoid compiler warnings. */
    (void) result;

    /* Enable global interrupts. */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port. */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);


    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H");
    printf("===============================================================\n");
#if defined(COMPONENT_CM0P)
    printf("CE229889 - MQTT Client running on CM0+\n");
#endif

#if defined(COMPONENT_CM4)
    printf("CE229889 - MQTT Client running on CM4\n");
#endif

#if defined(COMPONENT_CM7)
    printf("CE229889 - MQTT Client running on CM7\n");
#endif
    printf("===============================================================\n\n");

    /* Here doing the calibaration and start of movement sensor */


        // Initialize global structures
            globals_init();



		// Initialize SPI
		   result = cyhal_spi_init(&spi_obj, SPI_MOSI, SPI_MISO, SPI_CLK, NC, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);

		   if (result != CY_RSLT_SUCCESS) {
		           printf("SPI initialization failed\n");
		           CY_ASSERT(0);
		       }



		   result = cyhal_gpio_init(SPI_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1); // CS initially high
		       if (result != CY_RSLT_SUCCESS) {
		           printf("GPIO initialization failed\n");
		           CY_ASSERT(0);
		       }

	printf("Initilized SPI\n\r");

	   // Initialize BMI160 sensor
		   sensor.id = SPI_CS;
		   sensor.intf = BMI160_SPI_INTF;
		   sensor.read = bmi160_spi_read;
		   sensor.write = bmi160_spi_write;
		   sensor.delay_ms = bmi160_delay_ms;

    printf("About to do init func\n\r");


		   result = bmi160_init(&sensor);

		   printf("Initilized BMI Sensor\n\r");

		   if (result != BMI160_OK) {
			   printf("BMI160 initialization failed\n");
			   while (1);
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
		           while (1);
		       }

	   // Calibration: Read offsets for a few seconds and average them
		   int32_t x_offset = 0, y_offset = 0, z_offset = 0;
		   struct bmi160_sensor_data accel;
		   for (int i = 0; i < 100; i++) {
			   result = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);
			   if (result == BMI160_OK) {
				   x_offset += accel.x;
				   y_offset += accel.y;
				   z_offset += accel.z;
			   }
			   cyhal_system_delay_ms(10);
		   }
		   x_offset /= 100;
		   y_offset /= 100;
		   z_offset /= 100;


		   calibration_data.x_offset = x_offset;
		   calibration_data.y_offset = y_offset;
		   calibration_data.z_offset = z_offset;


		   printf("Calibartion complete \n\r");


    /* Create the MQTT Client task. */
    result = xTaskCreate(mqtt_client_task, "MQTT Client task", MQTT_CLIENT_TASK_STACK_SIZE,
                NULL, MQTT_CLIENT_TASK_PRIORITY, NULL);

    if (result != pdPASS) {
            printf("Failed to create MQTT Client task\n");
            CY_ASSERT(0);
        }

    /* Create movement reading task */
    result = xTaskCreate(sensorTask, "Position Sensor Task", 2048, NULL, 1, NULL);

    if (result != pdPASS) {
            printf("Failed to create Position Sensor task\n");
            CY_ASSERT(0);
        }

    printf("Make both tasks \n\r");
    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Should never get here. */
    CY_ASSERT(0);
}

/* [] END OF FILE */
