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

/**
 * Dynamic register & MailBox do not have delay for writting
 * System and User area have.
 * FTM nee to be unactivated when writting in the EEPROM areas
 */
static _I2C_Status __writeMemory(drivers_st25dv_addr_e adrType, uint16_t memAdr, uint8_t * data, uint8_t size, drivers_st25dv_delay_e delay) {
	_I2C_Status r =  i2c_memWrite(
			&ITSDK_DRIVERS_ST25DV_I2C,				// i2c handler
			(uint16_t)adrType,							// Device Address => 7 bits non shifted
			memAdr,										// Memory address to access
			16,											// 16 bits address
			data,										// Data to be written
			size										// Size of the data to be written
	);
	if ( delay == ST25DV_WITHDELAY ) {
		itsdk_delayMs((5+2)*((size/4)+1));						// Wait EEPROM Write 5ms per 32b blocks +2 for margin
	}
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
    		__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_MODE_REG,&v,1,ST25DV_WITHDELAY);
    		__readMemory(ST25DV_ADDR_SYST,ST25DV_MB_MODE_REG,&v,1);
    		if ( (v & ST25DV_MB_MODE_RW_MASK ) > 0 ) {
    			// success - now deactivate fast transfer mode
        		__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
        		v &= ~ST25DV_MB_CTRL_DYN_MBEN_MASK;	// disable FTM
    			__writeMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1,ST25DV_NODELAY);
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
    // Configure memory zones boundaries
    switch (__st25dv_config.devId ) {
    case I_AM_ST25DV64:
    	v = ST25DV_ST25DV64K_MAX_ENDA;
    	if (ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE+ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE+ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE+ITSDK_DRIVERS_ST25DV_USER_Z4_SIZE > (ST25DV_ST25DV64K_LAST_ADDRESS+1) ) {
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    default:
    case I_AM_ST25DV04:
    	v = ST25DV_ST25DV04K_MAX_ENDA;
    	if (ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE+ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE+ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE+ITSDK_DRIVERS_ST25DV_USER_Z4_SIZE > (ST25DV_ST25DV04K_LAST_ADDRESS+1) ) {
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    }
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END3_REG,&v,1,ST25DV_WITHDELAY);	// Set all the zone at maximum to ensure the order validity during reconfig
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END2_REG,&v,1,ST25DV_WITHDELAY);
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END1_REG,&v,1,ST25DV_WITHDELAY);
	int ad = (ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE >= 32)?(ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE - 31) / 32 : 0;
	v = (uint8_t)ad;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END1_REG,&v,1,ST25DV_WITHDELAY);
	ad = (ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE >= 32)?(ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE - 31) / 32 : 0;
	v += (uint8_t)ad;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END2_REG,&v,1,ST25DV_WITHDELAY);
	ad = (ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE >= 32)?(ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE - 31) / 32 : 0;
	v += (uint8_t)ad;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END3_REG,&v,1,ST25DV_WITHDELAY);

	// Configure the right accesses for the different user zone
	v = ITSDK_DRIVERS_ST25DV_USER_Z1_ACCESS | ST25DV_RFAxSS_NOPASSWORD;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_RFZ1SS_REG,&v,1,ST25DV_WITHDELAY);
	if ( ITSDK_DRIVERS_ST25DV_USER_Z2_PASS == __ENABLE ) {
		v = ITSDK_DRIVERS_ST25DV_USER_Z2_ACCESS | ST25DV_RFAxSS_RFPASS1;
	} else {
		v = ITSDK_DRIVERS_ST25DV_USER_Z2_ACCESS | ST25DV_RFAxSS_NOPASSWORD;
	}
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_RFZ2SS_REG,&v,1,ST25DV_WITHDELAY);
	if ( ITSDK_DRIVERS_ST25DV_USER_Z3_PASS == __ENABLE ) {
		v = ITSDK_DRIVERS_ST25DV_USER_Z3_ACCESS | ST25DV_RFAxSS_RFPASS2;
	} else {
		v = ITSDK_DRIVERS_ST25DV_USER_Z3_ACCESS | ST25DV_RFAxSS_NOPASSWORD;
	}
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_RFZ3SS_REG,&v,1,ST25DV_WITHDELAY);
	if ( ITSDK_DRIVERS_ST25DV_USER_Z4_PASS == __ENABLE ) {
		v = ITSDK_DRIVERS_ST25DV_USER_Z4_ACCESS | ST25DV_RFAxSS_RFPASS3;
	} else {
		v = ITSDK_DRIVERS_ST25DV_USER_Z4_ACCESS | ST25DV_RFAxSS_NOPASSWORD;
	}
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_RFZ4SS_REG,&v,1,ST25DV_WITHDELAY);

	v = 0;	// Enable full access from I2C
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_I2CZSS_REG,&v,1,ST25DV_WITHDELAY);


    // Activate specific setting conf
    __st25dv_config.mode = mode;
    switch (__st25dv_config.mode) {
    default:
    case ST25DV_MODE_DEFAULT:
    	// init FTM
		v= ST25DV_MB_MODE_RW_MASK;			// Enable RW for FTM EN bit
		__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_MODE_REG,&v,1,ST25DV_WITHDELAY);
		v = ST25DV_I2C_MB_WDG_DEF_VALUE;	// Clear pending data not read after 2s
		__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_WDG_REG,&v,1,ST25DV_WITHDELAY);

		drivers_st25dv_enableFTM();
    	break;
    }


    // going low power
    drivers_st25dv_goLowPower();
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
	if ( __writeMemory(ST25DV_ADDR_SYST,ST25DV_I2CPASSWD_REG,d,17,ST25DV_WITHDELAY) == I2C_OK ) {
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
	if ( __writeMemory(ST25DV_ADDR_SYST,ST25DV_I2CPASSWD_REG,d,17,ST25DV_WITHDELAY) == I2C_OK ) {
		return ST25DV_SUCCESS;
	}
	return ST25DV_FAILED;
}

/**
 * The FTM mode is incompatible with User and System Write operations
 * Is needs to be enable / disable regarding this kind of operation
 * FTM will be enabled only if the current mode have FTM enabled
 */
drivers_st25dv_ret_e drivers_st25dv_enableFTM() {
	uint8_t v;
	switch (__st25dv_config.mode) {
	    default:
	    case ST25DV_MODE_DEFAULT:
	    	// enable FTM
			__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
	   		v |= ST25DV_MB_CTRL_DYN_MBEN_MASK;	// enable FTM
			__writeMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1,ST25DV_NODELAY);
	    	break;
	}
	return ST25DV_SUCCESS;
}

/**
 * The FTM mode is incompatible with User and System Write operations
 * Is needs to be enable / disable regarding this kind of operation
 * FTM will be enabled only if the current mode have FTM enabled
 */
drivers_st25dv_ret_e drivers_st25dv_disableFTM() {
	uint8_t v;
	switch (__st25dv_config.mode) {
	    default:
	    case ST25DV_MODE_DEFAULT:
	    	// enable FTM
			__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
	   		v &= ~ST25DV_MB_CTRL_DYN_MBEN_MASK;	// disable FTM
			__writeMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1,ST25DV_NODELAY);
	    	break;
	}
	return ST25DV_SUCCESS;
}


/**
 * Sleep / WakeUp
 */
drivers_st25dv_ret_e drivers_st25dv_goLowPower() {
    if ((ITSDK_DRIVERS_ST25DV_LPD_PIN) != __LP_GPIO_NONE && __st25dv_config.state == ST25DV_WAKEUP ) {
    	__st25dv_config.state = ST25DV_SLEEPING;
		gpio_set(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// low power
    }
	return ST25DV_SUCCESS;
}

drivers_st25dv_ret_e drivers_st25dv_goWakeUp() {
	uint8_t v;
    if ((ITSDK_DRIVERS_ST25DV_LPD_PIN) != __LP_GPIO_NONE && __st25dv_config.state == ST25DV_SLEEPING) {
		gpio_reset(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// wakeup
    	__st25dv_config.state = ST25DV_WAKEUP;
		itsdk_delayMs(1);

		if (_drivers_st25dv_presentI2CPassword(ITSDK_DRIVERS_ST25DV_I2C_PASSWORD) != ST25DV_SUCCESS ) {
		    // failed to login with I2C password
			ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_PASSCHGKO,0);
		    return ST25DV_INVALIDPASS;
		}

	    switch (__st25dv_config.mode) {
	    default:
	    case ST25DV_MODE_DEFAULT:
	    	// re-init FTM
			__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
	   		v |= ST25DV_MB_CTRL_DYN_MBEN_MASK;	// enable FTM
			__writeMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1,ST25DV_NODELAY);
	    	break;
	    }
    }
	return ST25DV_SUCCESS;
}

// =========================================================================================
// BLOC ACCESS
// =========================================================================================

/**
 * Write a bloc of sz data into the given blockId into the given user zone
 * blockId is defining a 32b block (the RF block size)
 */
drivers_st25dv_ret_e drivers_st25dv_blocWrite(drivers_st25dv_zone_e zone, uint8_t blockId, uint8_t * data, uint8_t sz) {

	// compute the starting address
	uint16_t offset=0;
	switch (zone) {
	default:
	case ST25DV_USERZONE_1:
		offset = blockId*4;
		break;
	case ST25DV_USERZONE_2:
		offset = blockId*4;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE;
		break;
	case ST25DV_USERZONE_3:
		offset = blockId*4;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE;
		break;
	case ST25DV_USERZONE_4:
		offset = blockId*4;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE;
		break;
	}

	// Verify the memory bound
    switch (__st25dv_config.devId ) {
    case I_AM_ST25DV64:
    	if ( offset+sz > ST25DV_ST25DV64K_LAST_ADDRESS ) {
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    default:
    case I_AM_ST25DV04:
    	if ( offset+sz > (ST25DV_ST25DV04K_LAST_ADDRESS+1) ) {
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    }

    drivers_st25dv_disableFTM();
	// write block by block
	while ( sz > 0 ) {
		uint8_t _sz = (sz>=4)?4:sz;
		if ( __writeMemory(ST25DV_ADDR_DATA,offset,data,_sz,ST25DV_WITHDELAY) != I2C_OK ) {
			return ST25DV_FAILED;
		}
		offset+=_sz;
		data+=_sz;
		sz-=_sz;
	}
    drivers_st25dv_enableFTM();

	return ST25DV_SUCCESS;

}


/**
 * Read a bloc of sz data into the given blockId into the given user zone
 * blockId is defining a 32b block (the RF block size)
 */
drivers_st25dv_ret_e drivers_st25dv_blocRead(drivers_st25dv_zone_e zone, uint8_t blockId, uint8_t * data, uint8_t sz) {

	// compute the starting address
	uint16_t offset=0;
	switch (zone) {
	default:
	case ST25DV_USERZONE_1:
		offset = blockId*4;
		break;
	case ST25DV_USERZONE_2:
		offset = blockId*4;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE;
		break;
	case ST25DV_USERZONE_3:
		offset = blockId*4;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE;
		break;
	case ST25DV_USERZONE_4:
		offset = blockId*4;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE;
		offset+=ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE;
		break;
	}

	// Verify the memory bound
    switch (__st25dv_config.devId ) {
    case I_AM_ST25DV64:
    	if ( offset+sz > ST25DV_ST25DV64K_LAST_ADDRESS ) {
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    default:
    case I_AM_ST25DV04:
    	if ( offset+sz > (ST25DV_ST25DV04K_LAST_ADDRESS+1) ) {
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    }


	// write block by block
	while ( sz > 0 ) {
		uint8_t _sz = (sz>=4)?4:sz;
		if ( __readMemory(ST25DV_ADDR_DATA,offset,data,_sz) != I2C_OK ) {
			return ST25DV_FAILED;
		}
		offset+=_sz;
		data+=_sz;
		sz-=_sz;
	}

	return ST25DV_SUCCESS;

}

// =========================================================================================
// Fast Transfer Mode - FTP (MailBox) communication
// =========================================================================================

/**
 * Send bytes to the FTM-MB
 */
drivers_st25dv_ret_e drivers_st25dv_ftmWrite(uint8_t * messages, uint16_t sz) {
	if ( sz > ST25DV_I2C_MB_SIZE ) return ST25DV_FAILED;
	if ( __st25dv_config.state == ST25DV_SLEEPING ) return ST25DV_FAILED;

	if ( drivers_st25dv_ftmFreeForWriting() == ST25DV_EMPTYFTM ) {
		// allgood
		if ( __writeMemory(ST25DV_ADDR_DATA, ST25DV_MAILBOX_RAM_REG, messages, sz,ST25DV_NODELAY) == I2C_OK ) {
			return ST25DV_SUCCESS;
		}
	}
	return ST25DV_FAILED;
}


/**
 * Check if we have pending bytes in the FTM queue
 * Returns:
 * - ST25DV_SUCCESS when we have some
 * - ST25DV_EMPTYFTM when none
 */
drivers_st25dv_ret_e drivers_st25dv_ftmAvailableToRead() {

	uint8_t v;
	if ( __st25dv_config.state == ST25DV_SLEEPING ) return ST25DV_FAILED;

 	__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
 	if ( (v & ST25DV_MB_CTRL_DYN_HOSTPUTMSG_MASK) > 0 ) {
 		return ST25DV_SUCCESS;
 	}
 	return ST25DV_EMPTYFTM;

}

/**
 * Check if we have pending bytes in the FTM queue to read
 * Returns:
 * - ST25DV_EMPTYFTM when we can use the FTM
 * - ST25DV_NONEMPTYFTM when we have pending data in the FTM
 */
drivers_st25dv_ret_e drivers_st25dv_ftmFreeForWriting() {

	uint8_t v;
	if ( __st25dv_config.state == ST25DV_SLEEPING ) return ST25DV_FAILED;

 	__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
 	if ( (v & ST25DV_MB_CTRL_DYN_MBEN_MASK) > 0 ) {
 	 	if ( (v & ( ST25DV_MB_CTRL_DYN_HOSTPUTMSG_MASK | ST25DV_MB_CTRL_DYN_RFPUTMSG_MASK ) ) > 0 ) {
 	 		return ST25DV_NONEMPTYFTM;
 	 	}
 	 	return ST25DV_EMPTYFTM;
 	}
 	return ST25DV_FAILED;			// MBEN = 0 - FTM is unactivated

}


#endif
