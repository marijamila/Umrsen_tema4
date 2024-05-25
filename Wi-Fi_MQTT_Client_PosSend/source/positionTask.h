/*
 * positionTask.h
 *
 *  Created on: 18 May 2024
 *      Author: Damjan


#ifndef SOURCE_POSITIONTASK_H_
#define SOURCE_POSITIONTASK_H_

// Define SPI pins
#define SPI_MOSI CYBSP_SPI_MOSI
#define SPI_MISO CYBSP_SPI_MISO
#define SPI_CLK  CYBSP_SPI_CLK
#define SPI_CS   CYBSP_SPI_CS


#include "Position.h"

void sensorTask(void *params);

int8_t bmi160_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void bmi160_delay_ms(uint32_t period);

#endif /* SOURCE_POSITIONTASK_H_ */
