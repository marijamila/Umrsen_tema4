/*
 * DSP310.h
 *
 *  Created on: 16. svi 2024.
 *      Author: Marija Milanović
 */

#ifndef DPS310_H_
#define DPS310_H_

#include "cyhal_i2c.h"
#include "cyhal.h"
#include "cybsp.h"
#include "math.h"

#define I2C_SLAVE_ADDRESS	0x77
#define COEF_SIZE			18UL
#define PRS_CFG_ADDRESS		0x06
#define TMP_CFG_ADDRESS		0x07
#define CFG_REG_ADDRESS		0x09
#define MEAS_CFG_ADDRESS	0x08
#define TMP_B2				0x03
#define TMP_B1				0x04
#define TMP_B0				0x05
#define PRS_B2				0x00
#define PRS_B1				0x01
#define PRS_B0				0x02

// measurement settings
#define PRS_LOW_POWER		0x01
#define TMP_LOW_POWER		0x08
#define PRS_STANDARD_PREC	0x14
#define TMP_STANDARD_PREC	0x90
#define PRS_HIGH_PREC		0x26
#define TEMP_HIGH_PREC		0xA0

#define MEAS_TEMP			0x02
#define MEAS_PRS			0x01


void DPS310_start_measurement(void);
void DPS310_measure(float *temperature, float *pressure);

#endif /* DPS310_H_ */
