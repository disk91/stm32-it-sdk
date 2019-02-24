/* ==========================================================
 * max17205.c - Maxim 17205 - Gauge 3 cells
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 24 févr. 2019
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

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_MAX17205 == __ENABLE
#include <it_sdk/config_defines.h>
#include <drivers/gauge/max17205/max17205.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/logger/logger.h>


static drivers_max17205_conf_t __max17205_config;

/**
 * Read 16b values
 */
static _I2C_Status __readRegister(uint16_t addr, uint16_t * value) {
	uint8_t devAdr = ITSDK_DRIVERS_MAX17205_ADDRESS_000_0FF;
	if ( addr >= 0x100 ) {
		devAdr = ITSDK_DRIVERS_MAX17205_ADDRESS_100_17F;
	}
	_I2C_Status r = i2c_read16BRegister(
			&ITSDK_DRIVERS_MAX17205_I2C,
			devAdr,
			addr,
			value,
			1
		   );
	return r;
}

/**
 * Write 16b values
 */
static _I2C_Status __writeRegister(uint16_t addr, uint16_t value) {
	uint8_t devAdr = ITSDK_DRIVERS_MAX17205_ADDRESS_000_0FF;
	if ( addr >= 0x100 ) {
		devAdr = ITSDK_DRIVERS_MAX17205_ADDRESS_100_17F;
	}
	return i2c_write16BRegister(
			&ITSDK_DRIVERS_MAX17205_I2C,
			devAdr,
			addr,
			value,
			1
		   );
}

/**
 * Interrupt handler
 */
static void __max17205_interrupt(uint16_t GPIO_Pin) {
	log_info("ST25DV Int\r\n");

}

static gpio_irq_chain_t __max17205_gpio_irq = {
		__max17205_interrupt,
		ITSDK_DRIVERS_MAX17205_ALRT1_PIN,
		NULL,
};


/**
 * Setup the sensor according to the selected mode.
 */
drivers_max17205_ret_e drivers_max17205_setup(drivers_max17205_mode_e mode) {

	__max17205_config.mode = mode;

	// Search for the device type
	uint16_t v;
	if ( __readRegister(ITSDK_DRIVERS_MAX17205_REG_DEVNAME_ADR,&v) != I2C_OK ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX17205_NOTFOUND,0);
		return MAX17205_NOTFOUND;
	}
	__max17205_config.devType = v;
	if ( __max17205_config.devType != MAX17205_TYPE_172X1 && __max17205_config.devType != MAX17205_TYPE_172X5 ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX17205_NOTFOUND,0);
		return MAX17205_NOTFOUND;
	}

	if (ITSDK_DRIVERS_MAX17205_ALRT1_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_MAX17205_ALRT1_BANK,ITSDK_DRIVERS_MAX17205_ALRT1_PIN,GPIO_INTERRUPT_RISING);
		gpio_registerIrqAction(&__max17205_gpio_irq);
	}

	return MAX17205_SUCCESS;
}

#endif // ITSDK_DRIVERS_MAX17205
