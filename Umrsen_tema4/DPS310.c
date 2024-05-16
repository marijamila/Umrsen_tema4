/*
 * DPS310.c
 *
 *  Created on: 16. svi 2024.
 *      Author: Marija Milanović
 */

#include "DPS310.h"
#include "i2c.h"

static uint8_t coef_addr[COEF_SIZE];	// addr of Calibration Coefficients (COEF)
static uint8_t coef[COEF_SIZE];		// Calibration Coefficients (COEF)
									// treba biti int8_t, ali funckija _read() prima unit8_t
static int16_t c0, c1, c20, c30, c01, c11, c21;	// int16_t ?
static int32_t c00, c10;
static int32_t tmp_raw;
static int32_t prs_raw;
static uint32_t T_raw_sc;

/* read Calibration Coefficients (COEF), set PRS_CFG, TMP_CFG, CFG_REG */
void DPS310_start_measurement(void) {

	uint8_t data[2];

	for (uint8_t i = 0; i < COEF_SIZE; i++) {
		coef_addr[i] = 0x10 + i;	// 0x10 - 0x21
	}

	// read COEF
	if (CY_RSLT_SUCCESS == cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDRESS, coef_addr, COEF_SIZE, 0, false)) {
		cyhal_i2c_master_read(&i2c_master_obj, I2C_SLAVE_ADDRESS, coef, COEF_SIZE, 0, true);
			c20 = coef[13] + (coef[12] << 8);
			if (c20 > pow(2, 15) - 1) {
				c20 = c20 - pow(2, 16);
			}

			c0 = (coef[0] << 4) + ((coef[1] >> 4) & 0x0F);
			if (c0 > pow(2, 11) - 1) {
				c0 = c0 - pow(2, 12);
			}

			c1 = (coef[1] << 8) + coef[2];
			c00 = (coef[3] << 12) + (coef[4] << 4) + ((coef[5] >> 4) & 0x0F);
			c10 = (coef[5] << 16) + (coef[6] << 8) + coef[7];
			c30 = (coef[16] << 8) + coef[17];
			c01 = (coef[8] << 8) + coef[9];
			c11 = (coef[10] << 8) + coef[11];
			c21 = (coef[14] << 8) + coef[15];
	}

	// set PRS_CFG
	data[0] = PRS_CFG_ADDRESS;
	data[1] = PRS_STANDARD_PREC;
	if (CY_RSLT_SUCCESS != cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDRESS, data, 2, 0, false)) {
		CY_ASSERT(0);
	}

	// set TMP_CFG
	data[0] = TMP_CFG_ADDRESS;
	data[1] = TMP_STANDARD_PREC;
	if (CY_RSLT_SUCCESS != cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDRESS, data, 2, 0, false)) {
		CY_ASSERT(0);
	}

	// set CFG_REG
	data[0] = CFG_REG_ADDRESS;
	data[1] = 0x04; // ako se koristi PRS_STANDARD_PREC ili PRS_HIGH_PREC, 0x00 za PRS_LOW_POWER i treba promijeniti kT i kP
	if (CY_RSLT_SUCCESS != cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDRESS, data, 2, 0, false)) {
		CY_ASSERT(0);
	}
}

uint32_t calculate_temperature(uint32_t T_raw) {
	 uint32_t kT = 524288; // jer je oversampling rate za temperaturu 1
	 T_raw_sc = T_raw / kT;
	 uint32_t T_comp = c0 * 0.5 + c1 * T_raw_sc;
	 return T_comp;
}

uint32_t calculate_pressure(uint32_t P_raw) {
	uint32_t kP = 253952; // jer je oversampling rate za tlak 16
	uint32_t P_raw_sc = P_raw / kP;
	uint32_t P_comp = c00 + P_raw_sc * (c10 + P_raw_sc * (c20 + P_raw_sc * c30)) + T_raw_sc * c01 + T_raw_sc * P_raw_sc * (c11 + P_raw_sc * c21);
	return P_comp;
}

void DPS310_measure(uint32_t *temperature, uint32_t *pressure) {

	uint8_t meas[2] = {MEAS_CFG_ADDRESS, MEAS_TEMP};
	uint8_t tmp_b[3] = {TMP_B2, TMP_B1, TMP_B0};
	uint8_t tmp[3] = {0, 0, 0}; // treba biti int
	uint8_t prs_b[3] = {PRS_B2, PRS_B1, PRS_B0};
	uint8_t prs[3] = {0, 0, 0}; //treba biti int

	// read temperature
	if (CY_RSLT_SUCCESS == cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDRESS, meas, 2, 0, false)) {
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDRESS, tmp_b, 3, 0, false)) {
			if (CY_RSLT_SUCCESS == cyhal_i2c_master_read(&i2c_master_obj, I2C_SLAVE_ADDRESS, tmp, 3, 0, true)) {
				tmp_raw = (tmp[0] << 16) + (tmp[1] << 8) + tmp[2];
				temperature = (uint32_t *)calculate_temperature(tmp_raw);
			}
		}
	}

	// read pressure
	meas[1] = MEAS_PRS;
	if (CY_RSLT_SUCCESS == cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDRESS, meas, 2, 0, false)) {
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDRESS, prs_b, 3, 0, false)) {
			if (CY_RSLT_SUCCESS == cyhal_i2c_master_read(&i2c_master_obj, I2C_SLAVE_ADDRESS, prs, 3, 0, true)) {
				prs_raw = (prs[0] << 16) + (prs[1] << 8) + prs[2];
				pressure = (uint32_t *)calculate_pressure(prs_raw);
			}
		}
	}

	return;
}

