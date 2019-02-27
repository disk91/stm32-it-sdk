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
	_I2C_Status r =  i2c_memWrite(
			&ITSDK_DRIVERS_ST25DV_I2C,				// i2c handler
			(uint16_t)adrType,							// Device Address => 7 bits non shifted
			memAdr,										// Memory address to access
			16,											// 16 bits address
			data,										// Data to be written
			size										// Size of the data to be written
	);
	itsdk_delayMs(5*((size/4)+1));						// Wait EEPROM Write 5ms per 32b blocks
	return r;
}

static _I2C_Status __readMemory(drivers_st25dv_addr_e adrType, uint16_t memAdr, uint8_t * data, uint8_t size) {
	return i2c_memRead(
			&ITSDK_DRIVERS_ST25DV_I2C,				// i2c handler
			(uint16_t)adrType,							// Device Address => 7 bits non shifted
			memAdr,										// Memory address to access
			16,											// 16 bits address
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

    // Configure the I2C password
    if ( _drivers_st25dv_presentI2CPassword(0) == ST25DV_SUCCESS ) {
    	// test default password success
    	if ( ITSDK_DRIVERS_ST25DV_I2C_PASSWORD != 0 ) {
    		uint8_t v;
    		// Change the system config bits so that FTM can be turned off/on Mailbox R/W bit set to 1
    		v= ST25DV_MB_MODE_RW_MASK;
    		__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_MODE_REG,&v,1);
    		__readMemory(ST25DV_ADDR_SYST,ST25DV_MB_MODE_REG,&v,1);
    		if ( (v & ST25DV_MB_MODE_RW_MASK ) > 0 ) {
    			// success - now deactivate fast transfer mode
        		__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
        		v &= ~ST25DV_MB_CTRL_DYN_MBEN_MASK;	// disable FTM
    			__writeMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
    			// Change password
    			if ( _drivers_st25dv_changeI2CPassword(ITSDK_DRIVERS_ST25DV_I2C_PASSWORD) != ST25DV_SUCCESS ) {
    				ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_PASSCHGKO,0);
    			} else {
    				ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_PASSCHGOK,0);
    			}
    		}
    	}
    } else if (_drivers_st25dv_presentI2CPassword(ITSDK_DRIVERS_ST25DV_I2C_PASSWORD) != ST25DV_SUCCESS ) {
    	// failed to login with I2C password
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_PASSCHGKO,0);
    	return ST25DV_INVALIDPASS;
    }

    // We are login
    uint8_t v;
    switch (mode) {
    default:
    case ST25DV_MODE_DEFAULT:
    	// init FTM
		v= ST25DV_MB_MODE_RW_MASK;
		__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_MODE_REG,&v,1);
   		__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
   		v |= ST25DV_MB_CTRL_DYN_MBEN_MASK;	// enable FTM
		__writeMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
    	break;
    }


    // going low power
    if ((ITSDK_DRIVERS_ST25DV_LPD_PIN) != __LP_GPIO_NONE) {
		gpio_set(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// low power
    }
	return ST25DV_SUCCESS;
}

/**
 * Send the I2C Password to unlock the features.
 */
drivers_st25dv_ret_e _drivers_st25dv_presentI2CPassword(uint64_t pass) {
	// Send password
	uint8_t d[17];
	for (int i = 0 ; i < 8 ; i++ ) {
		d[i] = (pass >> (56-8*i)) & 0xFF;
		d[i+9] = (pass >> (56-8*i)) & 0xFF;
	}
	d[8] = ST25DV_I2CPASSWD_VALID_BYTE;
	if ( __writeMemory(ST25DV_ADDR_SYST,ST25DV_I2CPASSWD_REG,d,17) == I2C_OK ) {
		// Verify login sucess
		if ( __readMemory(ST25DV_ADDR_DATA,ST25DV_I2C_SSO_DYN_REG,d,1) == I2C_OK ) {
			if ( (d[0] & ST25DV_I2C_SSO_DYN_I2CSSO_MASK) > 0 ) {
				return ST25DV_SUCCESS;
			}
		}
	}
	return ST25DV_FAILED;
}

/**
 * Set a new I2C password
 */
drivers_st25dv_ret_e _drivers_st25dv_changeI2CPassword(uint64_t pass) {
	// Send password
	uint8_t d[17];
	for (int i = 0 ; i < 8 ; i++ ) {
		d[i] = (pass >> (56-8*i)) & 0xFF;
		d[i+9] = (pass >> (56-8*i)) & 0xFF;
	}
	d[8] = ST25DV_I2CPASSWD_VALID_BYTE_WR;
	if ( __writeMemory(ST25DV_ADDR_SYST,ST25DV_I2CPASSWD_REG,d,17) == I2C_OK ) {
		return ST25DV_SUCCESS;
	}
	return ST25DV_FAILED;
}



#endif
