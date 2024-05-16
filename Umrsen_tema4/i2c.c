/*
 * i2c.c
 *
 *  Created on: 16. svi 2024.
 *      Author: Marija Milanović
 */

#include "i2c.h"

cyhal_i2c_t i2c_master_obj;

void i2c_master_init(void) {

	// Declare variables
	cy_rslt_t   rslt;

	// Define frequency
	uint32_t I2C_MASTER_FREQUENCY = 100000u;

	// Define the I2C master configuration structure
	cyhal_i2c_cfg_t i2c_master_config =
	{
	    CYHAL_I2C_MODE_MASTER,
	    0, // address is not used for master mode
	    I2C_MASTER_FREQUENCY
	};

	// Initialize I2C master, set the SDA and SCL pins and assign a new clock
	rslt = cyhal_i2c_init(&i2c_master_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
	if (rslt != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	// Configure the I2C resource to be master
	rslt = cyhal_i2c_configure(&i2c_master_obj, &i2c_master_config);
	if (rslt != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}
}


