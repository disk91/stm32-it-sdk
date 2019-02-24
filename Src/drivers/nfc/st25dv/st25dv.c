/* ==========================================================
 * st25dv.c - ST NFC 25 DV chip
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
#if ITSDK_DRIVERS_ST25DV == __ENABLE
#include <it_sdk/config_defines.h>
#include <drivers/nfc/st25dv/st25dv.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/logger/logger.h>

static drivers_st25dv_conf_t __st25dv_config;

/*
static _I2C_Status __readRegister(uint8_t addr, uint8_t * value) {
	return i2c_read8BRegister(
			&ITSDK_DRIVERS_ST25DV_I2C,
			ITSDK_DRIVERS_MAX44009_ADDRESS,
			addr,
			value,
			1
		   );
}

static _I2C_Status __writeRegister(uint8_t addr, uint8_t value) {
	return i2c_write8BRegister(
			&ITSDK_DRIVERS_ST25DV_I2C,
			ITSDK_DRIVERS_MAX44009_ADDRESS,
			addr,
			value,
			1
		   );
}
*/

static _I2C_Status __writeMemory(drivers_st25dv_addr_e adrType, uint16_t memAdr, uint8_t * data, uint8_t size) {
	return i2c_memWrite(
			&ITSDK_DRIVERS_ST25DV_I2C,				// i2c handler
			(uint16_t)adrType,							// Device Address => 7 bits non shifted
			memAdr,										// Memory address to access
			16,											// 16 bits data
			data,										// Data to be written
			size										// Size of the data to be written
	);
}

static _I2C_Status __readMemory(drivers_st25dv_addr_e adrType, uint16_t memAdr, uint8_t * data, uint8_t size) {
	return i2c_memRead(
			&ITSDK_DRIVERS_ST25DV_I2C,				// i2c handler
			(uint16_t)adrType,							// Device Address => 7 bits non shifted
			memAdr,										// Memory address to access
			16,											// 16 bits data
			data,										// Data to be written
			size										// Size of the data to be written
	);
}


/**
 * Interrupt handler
 */
static void __st25dv_interrupt(uint16_t GPIO_Pin) {
	log_info("ST25DV Int\r\n");

}

static gpio_irq_chain_t __st25dv_gpio_irq = {
		__st25dv_interrupt,
		ITSDK_DRIVERS_ST25DV_GPO_PIN,
		NULL,
};

/**
 * Setup the device
 */
drivers_st25dv_ret_e drivers_st25dv_setup(drivers_st25dv_mode_e mode) {

	// Search for the device Get Id
	if ( __readMemory(ST25DV_ADDR_SYST,ST25DV_ICREF_REG,&__st25dv_config.devId,1) != I2C_OK ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_NOTFOUND,0);
		return ST25DV_NOTFOUND;
	}
	if ( __st25dv_config.devId != I_AM_ST25DV04 && __st25dv_config.devId != I_AM_ST25DV64 ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_NOTFOUND,0);
		return ST25DV_NOTFOUND;
	}

	// Configure Pin & interrupt
	if ((ITSDK_DRIVERS_ST25DV_GPO_PIN) != __LP_GPIO_NONE) {
		gpio_configure(ITSDK_DRIVERS_ST25DV_GPO_BANK,ITSDK_DRIVERS_ST25DV_GPO_PIN,GPIO_INTERRUPT_RISING);
		gpio_registerIrqAction(&__st25dv_gpio_irq);
	}
    if ((ITSDK_DRIVERS_ST25DV_LPD_PIN) != __LP_GPIO_NONE) {
		gpio_configure(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN,GPIO_OUTPUT_PP);
		gpio_reset(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// power On
    }


	return ST25DV_SUCCESS;
}





#endif
