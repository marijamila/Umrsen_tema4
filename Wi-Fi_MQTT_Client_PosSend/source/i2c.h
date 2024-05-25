/*
 * i2c.h
 *
 *  Created on: 16. svi 2024.
 *      Author: Marija Milanović
 */

#ifndef I2C_H_
#define I2C_H_

#include "cyhal_i2c.h"
#include "cyhal.h"
#include "cybsp.h"

extern cyhal_i2c_t i2c_master_obj;

void i2c_master_init(void);

#endif /* I2C_H_ */
