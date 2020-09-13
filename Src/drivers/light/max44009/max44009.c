/* ==========================================================
 * max44009.c - Maximum Light sensor driver
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 19 f�vr. 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2019 Disk91
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU LESSER General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------
 * 
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_MAX44009 == __ENABLE
#include <drivers/light/max44009/max44009.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>


static drivers_max44009_conf_t __max44009_config;

static _I2C_Status __readRegister(uint8_t addr, uint8_t * value) {
	return i2c_read8BRegister(
			&ITSDK_DRIVERS_MAX44009_I2C,
			ITSDK_DRIVERS_MAX44009_ADDRESS,
			addr,
			value,
			1
		   );
}

static _I2C_Status __writeRegister(uint8_t addr, uint8_t value) {
	return i2c_write8BRegister(
			&ITSDK_DRIVERS_MAX44009_I2C,
			ITSDK_DRIVERS_MAX44009_ADDRESS,
			addr,
			value,
			1
		   );
}


/**
 * Setup the sensor according to the selected mode.
 */
drivers_max44009_ret_e drivers_max44009_setup(drivers_max44009_mode_e mode) {
	// Check device presence
	uint8_t v;
	if (  __readRegister(DRIVER_MAX44009_REG_CONFIG_ADR,&v) != __I2C_OK ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX44009_NOTFOUND,0);
		return MAX44009_NOTFOUND;
	}

	// Configure the device according the selected mode
	switch (mode) {
	default:
	case MAX44009_MODE_ONREQUEST:
		// Simple mode
		// Lux captured automatically, read onDemand, interrupt disabled
		__writeRegister(DRIVER_MAX44009_REG_INT_ENABLE_ADR,0);	// Disable interrupt

		__readRegister(DRIVER_MAX44009_REG_CONFIG_ADR,&v);		// Everything is automatic
		v &= ~DRIVER_MAX44009_REG_CONFIG_CONT_MSK;
		v |= DRIVER_MAX44009_REG_CONFIG_CONT_DEFAULT;
		v &= ~DRIVER_MAX44009_REG_CONFIG_MANUAL_MSK;
		v |= DRIVER_MAX44009_REG_CONFIG_MANUAL_AUTO;
		__writeRegister(DRIVER_MAX44009_REG_CONFIG_ADR,v);
		break;

	}
	__max44009_config.mode = mode;

	return MAX44009_SUCCESS;
}

/**
 * Return the lux value in milli-lux
 */
drivers_max44009_ret_e drivers_max44009_getSensors(
		uint32_t  * mlux					// Lux value
) {
	uint8_t h,l;
	// Verify I2C
	if (  __readRegister(DRIVER_MAX44009_REG_LUX_HIGH_ADR,&h) != __I2C_OK ) {
		ITSDK_ERROR_REPORT(DRIVER_MAX44009_REG_LUX_HIGH_ADR,0);
		return MAX44009_FAILED;
	}
	if ( __readRegister(DRIVER_MAX44009_REG_LUX_LOW_ADR,&l) != __I2C_OK ) {
		ITSDK_ERROR_REPORT(DRIVER_MAX44009_REG_LUX_HIGH_ADR,0);
		return MAX44009_FAILED;
	}

	uint32_t exp = (h & DRIVER_MAX44009_REG_LUXHIGH_EXP_MSK) >> DRIVER_MAX44009_REG_LUXHIGH_EXP_SHIFT;
	uint32_t man = (h & DRIVER_MAX44009_REG_LUXHIGH_MAN_MSK) >> DRIVER_MAX44009_REG_LUXHIGH_MAN_SHIFT;
	man = (man << 4) | (( l & DRIVER_MAX44009_REG_LUXLOW_MAN_MSK ) >> DRIVER_MAX44009_REG_LUXLOW_MAN_SHIFT);

	*mlux = (2 << exp) * man * 45;
	return MAX44009_SUCCESS;
}

#endif
#endif
