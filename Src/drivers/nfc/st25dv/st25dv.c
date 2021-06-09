/* ==========================================================
 * st25dv.c - ST NFC 25 DV chip
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 24 f�vr. 2019
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
 * Note : RF & I2C are not working simultaneously... so we need
 * to manage retry as we have no control on RF communication
 * Note : When LDP the I2C is not working
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_ST25DV == __ENABLE
#include <it_sdk/config.h>
#include <it_sdk/config_defines.h>
#include <drivers/nfc/st25dv/st25dv.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/time/time.h>
#include <it_sdk/time/timer.h>

static drivers_st25dv_conf_t __st25dv_config = {0};

/**
 * Dynamic register & MailBox do not have delay for writting
 * System and User area have.
 * FTM need to be unactivated when writting in the EEPROM areas
 */
static _I2C_Status __writeMemory(drivers_st25dv_addr_e adrType, uint16_t memAdr, uint8_t * data, uint8_t size) {
	uint8_t try = 0;
	_I2C_Status r;
	while ( try < ST25DV_I2C_MAXTRY
		    &&  ( r =  i2c_memWrite(
					&ITSDK_DRIVERS_ST25DV_I2C,				// i2c handler
					(uint16_t)adrType,							// Device Address => 7 bits non shifted
					memAdr,										// Memory address to access
					16,											// 16 bits address
					data,										// Data to be written
					size										// Size of the data to be written
			     ) ) != __I2C_OK )
	{
		try++;
		itsdk_delayMs(ST25DV_I2C_RETRY_WAIT_MS);
	}
	if ( try < ST25DV_I2C_MAXTRY ) {
		if (   ( adrType == ST25DV_ADDR_DATA && memAdr < ST25DV_GPO_DYN_REG )		// Dyn register does not need this time
			|| ( adrType == ST25DV_ADDR_SYST && memAdr != ST25DV_I2CPASSWD_REG )	// password presentation does not need this time
		){
			itsdk_delayMs((5+2)*((size/4)+1));						// Wait EEPROM Write 5ms per 32b blocks +2 for margin
		}
		return __I2C_OK;
	} else {
		return r;
	}
}

/**
 * Read memory with integrated rety
 */
static _I2C_Status __readMemory(drivers_st25dv_addr_e adrType, uint16_t memAdr, uint8_t * data, uint8_t size) {
	uint8_t try = 0;
	_I2C_Status r;
	while ( try < ST25DV_I2C_MAXTRY
			    &&  ( r =  i2c_memRead(
						&ITSDK_DRIVERS_ST25DV_I2C,				// i2c handler
						(uint16_t)adrType,							// Device Address => 7 bits non shifted
						memAdr,										// Memory address to access
						16,											// 16 bits address
						data,										// Data to be written
						size										// Size of the data to be written
				     ) ) != __I2C_OK )
	{
		try++;
		itsdk_delayMs(ST25DV_I2C_RETRY_WAIT_MS);
	}
	return ( try < ST25DV_I2C_MAXTRY )?__I2C_OK:r;
}


/**
 * Interrupt handler
 * When the device enter in RF Field we force to stay active
 * When leaving the device will back in sleep mode. A timer is protecting against no failing interrupt.
 * The failling interrupt is only generated when the the VCC is activated (device not sleeping)
 */
static void __st25dv_timeout(uint32_t val) {
	uint8_t v;
	switch ( __st25dv_config.state ) {
	case ST25DV_SLEEPING:
		return;
		break;
	case ST25DV_WAKEUP:
		if ( __readMemory(ST25DV_ADDR_DATA,ST25DV_EH_CTRL_DYN_REG,&v,1) == __I2C_OK ) {
			if ( (v & ST25DV_EH_CTRL_DYN_FIELD_ON_MASK) == 0 ) {
				// Out of field confirmed
				__st25dv_config.field = ST25DV_OUT_OF_FIELD;
			    drivers_st25dv_goLowPower();
				return;
			}
		}					// No Break = Normal
	case ST25DV_PROCESSING:
		itsdk_stimer_register(30000,__st25dv_timeout,0,TIMER_ACCEPT_LOWPOWER);	// try again later
		break;
	}

}

/**
 * The Process is part of the main loop
 * It process the result of the IRQ change
 * This way the IRQ is processed synchronously instead of being async.
 * This is reducing the risk to interact with other function writing on I2C in paralel
 * **** This code needs to ba called by main application loop ****
 */
void st25dv_process() {
	if ( __st25dv_config.ready != ST25DV_IRQ_PENDING ) return;


	// basic wake up, just to have I2C working
    if (   ITSDK_DRIVERS_ST25DV_LPD_PIN != __LP_GPIO_NONE
    	&& __st25dv_config.state == ST25DV_SLEEPING
	) {
		gpio_reset(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// wakeup
		itsdk_delayMs(1);
    }

	uint8_t v;
	if ( __readMemory(ST25DV_ADDR_DATA,ST25DV_ITSTS_DYN_REG,&v,1) == __I2C_OK ) {
		if ( (v & ST25DV_ITSTS_FIELDFALLING_MASK) > 0) {
			// we will go low power after expiration to limit the in/out frequency
			//log_info("FieldOut");
			__st25dv_config.field = ST25DV_OUT_OF_FIELD;
			itsdk_stimer_stop(__st25dv_timeout,0);
			itsdk_stimer_register(3000,__st25dv_timeout,0,TIMER_ACCEPT_LOWPOWER);
		}
		if ( (v & ST25DV_ITSTS_FIELDRISING_MASK) > 0 ) {
			//log_info("FieldIn");
			__st25dv_config.field = ST25DV_IN_THE_FIELD;
			drivers_st25dv_goWakeUp();
			itsdk_stimer_stop(__st25dv_timeout,0);
			itsdk_stimer_register(30000,__st25dv_timeout,0,TIMER_ACCEPT_LOWPOWER);
		}
		if ( (v & ST25DV_ITSTS_RFPUTMSG_MASK) > 0 ) {
			//log_info("PutMsg");
		}
		if ( (v & ST25DV_ITSTS_RFGETMSG_MASK) > 0 ) {
			//log_info("GetMsg");
		}
		if ( (v & ST25DV_ITSTS_RFWRITE_MASK) > 0 ) {
			// call on RF Write operation into the UserLand
			//log_info("RfWrite");
		}
		if ( (v & ST25DV_ITSTS_RFINTERRUPT_MASK) > 0 ) {
			//log_info("RfInt");
		}
		if ( (v & ST25DV_ITSTS_RFACTIVITY_MASK) > 0 ) {
			//log_info("RfActivity");
		}
		if ( (v & ST25DV_ITSTS_RFUSERSTATE_MASK) > 0 ) {
			//log_info("UserState");
		}
	} else {
		log_error("ST25DV - ReadError\r\n");
	}

    if (   ITSDK_DRIVERS_ST25DV_LPD_PIN != __LP_GPIO_NONE
    	&& __st25dv_config.state == ST25DV_SLEEPING
	) {
		gpio_set(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// wakeup
    }

	__st25dv_config.ready = ST25DV_READY;

}


static void __st25dv_interrupt(uint16_t GPIO_Pin) {
	__st25dv_config.ready = ST25DV_IRQ_PENDING;
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

	// Ensuring we are power-up
    if ((ITSDK_DRIVERS_ST25DV_LPD_PIN) != __LP_GPIO_NONE) {
		gpio_configure(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN,GPIO_OUTPUT_PP);
		gpio_reset(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// power On
    }
    __st25dv_config.state = ST25DV_WAKEUP;


	// Search for the device Get Id
	if ( __readMemory(ST25DV_ADDR_SYST,ST25DV_ICREF_REG,&__st25dv_config.devId,1) != __I2C_OK ) {
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
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END3_REG,&v,1);	// Set all the zone at maximum to ensure the order validity during reconfig
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END2_REG,&v,1);
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END1_REG,&v,1);
	int ad = (ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE >= 32)?(ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE - 31) / 32 : 0;
	v = (uint8_t)ad;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END1_REG,&v,1);
	ad = (ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE >= 32)?(ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE - 31) / 32 : 0;
	v += (uint8_t)ad;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END2_REG,&v,1);
	ad = (ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE >= 32)?(ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE - 31) / 32 : 0;
	v += (uint8_t)ad;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_END3_REG,&v,1);

	// Configure the right accesses for the different user zone
	v = ITSDK_DRIVERS_ST25DV_USER_Z1_ACCESS | ST25DV_RFAxSS_NOPASSWORD;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_RFZ1SS_REG,&v,1);
	if ( ITSDK_DRIVERS_ST25DV_USER_Z2_PASS == __ENABLE ) {
		v = ITSDK_DRIVERS_ST25DV_USER_Z2_ACCESS | ST25DV_RFAxSS_RFPASS1;
	} else {
		v = ITSDK_DRIVERS_ST25DV_USER_Z2_ACCESS | ST25DV_RFAxSS_NOPASSWORD;
	}
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_RFZ2SS_REG,&v,1);
	if ( ITSDK_DRIVERS_ST25DV_USER_Z3_PASS == __ENABLE ) {
		v = ITSDK_DRIVERS_ST25DV_USER_Z3_ACCESS | ST25DV_RFAxSS_RFPASS2;
	} else {
		v = ITSDK_DRIVERS_ST25DV_USER_Z3_ACCESS | ST25DV_RFAxSS_NOPASSWORD;
	}
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_RFZ3SS_REG,&v,1);
	if ( ITSDK_DRIVERS_ST25DV_USER_Z4_PASS == __ENABLE ) {
		v = ITSDK_DRIVERS_ST25DV_USER_Z4_ACCESS | ST25DV_RFAxSS_RFPASS3;
	} else {
		v = ITSDK_DRIVERS_ST25DV_USER_Z4_ACCESS | ST25DV_RFAxSS_NOPASSWORD;
	}
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_RFZ4SS_REG,&v,1);

	v = 0;	// Enable full access from I2C
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_I2CZSS_REG,&v,1);


	// Configure the IRQ
	v = ST25DV_GPO_FIELDCHANGE_MASK | ST25DV_GPO_RFPUTMSG_MASK |  ST25DV_GPO_RFGETMSG_MASK | ST25DV_GPO_RFWRITE_MASK | ST25DV_GPO_ENABLE_MASK;
	__writeMemory(ST25DV_ADDR_SYST,ST25DV_GPO_REG,&v,1);
	__writeMemory(ST25DV_ADDR_DATA,ST25DV_GPO_DYN_REG,&v,1);
	__st25dv_config.field = ST25DV_OUT_OF_FIELD;

    // Activate specific setting conf
    __st25dv_config.mode = mode;
    switch (__st25dv_config.mode) {
#if ITSDK_DRIVERS_ST25DV_WITH_SERIALFTM == __ENABLE
    case ST25DV_MODE_FTM:
    	// init FTM
		v= ST25DV_MB_MODE_RW_MASK;			// Enable RW for FTM EN bit
		__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_MODE_REG,&v,1);
		v = ST25DV_I2C_MB_WDG_DEF_VALUE;	// Clear pending data not read after 2s
		__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_WDG_REG,&v,1);
		drivers_st25dv_enableFTM();
		drivers_st25dv_enableSerialFtm(ST25DV_MODE_FTM);
    	break;
#endif
#if ITSDK_DRIVERS_ST25DV_WITH_SERIALUZ == __ENABLE
    case ST25DV_MODE_SERIALUZ:
    	drivers_st25dv_enableSerialUz(ST25DV_MODE_SERIALUZ);
    	break;
#endif
    case ST25DV_MODE_SIMPLE:
    	break;
    default:
    	return ST25DV_INVALIDMODE;
    }


    // going low power
    drivers_st25dv_goLowPower();
    __st25dv_config.ready = ST25DV_READY;
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
	if ( __writeMemory(ST25DV_ADDR_SYST,ST25DV_I2CPASSWD_REG,d,17) == __I2C_OK ) {
		// Verify login sucess
		if ( __readMemory(ST25DV_ADDR_DATA,ST25DV_I2C_SSO_DYN_REG,d,1) == __I2C_OK ) {
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
	if ( __writeMemory(ST25DV_ADDR_SYST,ST25DV_I2CPASSWD_REG,d,17) == __I2C_OK ) {
		return ST25DV_SUCCESS;
	}
	return ST25DV_FAILED;
}



/**
 * Sleep / WakeUp
 */
drivers_st25dv_ret_e drivers_st25dv_goLowPower() {
    if (    ITSDK_DRIVERS_ST25DV_LPD_PIN != __LP_GPIO_NONE
    	 && __st25dv_config.state == ST25DV_WAKEUP
		 && __st25dv_config.field == ST25DV_OUT_OF_FIELD
	) {
    	__st25dv_config.state = ST25DV_SLEEPING;
		gpio_set(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// low power
    }
	return ST25DV_SUCCESS;
}

drivers_st25dv_ret_e drivers_st25dv_goWakeUp() {
	uint8_t v;
    if (   ITSDK_DRIVERS_ST25DV_LPD_PIN != __LP_GPIO_NONE
    	&& __st25dv_config.state == ST25DV_SLEEPING
	) {
		gpio_reset(ITSDK_DRIVERS_ST25DV_LPD_BANK,ITSDK_DRIVERS_ST25DV_LPD_PIN);	// wakeup
    	__st25dv_config.state = ST25DV_WAKEUP;
		itsdk_delayMs(1);

		if (_drivers_st25dv_presentI2CPassword(ITSDK_DRIVERS_ST25DV_I2C_PASSWORD) != ST25DV_SUCCESS ) {
		    // failed to login with I2C password
			ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_PASSCHGKO,0);
		    return ST25DV_INVALIDPASS;
		}

		v = ST25DV_GPO_FIELDCHANGE_MASK | ST25DV_GPO_RFPUTMSG_MASK |  ST25DV_GPO_RFGETMSG_MASK | ST25DV_GPO_RFWRITE_MASK | ST25DV_GPO_ENABLE_MASK;
		__writeMemory(ST25DV_ADDR_SYST,ST25DV_GPO_REG,&v,1);
		__writeMemory(ST25DV_ADDR_DATA,ST25DV_GPO_DYN_REG,&v,1);

	    switch (__st25dv_config.mode) {
	    default:
	    case ST25DV_MODE_FTM:
	    	// re-init FTM
			v= ST25DV_MB_MODE_RW_MASK;			// Enable RW for FTM EN bit
			__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_MODE_REG,&v,1);
			v = ST25DV_I2C_MB_WDG_DEF_VALUE;	// Clear pending data not read after 2s
			__writeMemory(ST25DV_ADDR_SYST,ST25DV_MB_WDG_REG,&v,1);
			drivers_st25dv_enableFTM();
	    	break;
        case ST25DV_MODE_SERIALUZ:
        case ST25DV_MODE_SIMPLE:
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
	drivers_st25dv_sleep_e p = __st25dv_config.state;
	__st25dv_config.state = ST25DV_PROCESSING;
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
    		__st25dv_config.state = p;
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    default:
    case I_AM_ST25DV04:
    	if ( offset+sz > (ST25DV_ST25DV04K_LAST_ADDRESS+1) ) {
    		__st25dv_config.state = p;
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    }

    drivers_st25dv_ret_e ret = ST25DV_SUCCESS;
    drivers_st25dv_disableFTM();
	// write block by block
	while ( sz > 0 ) {
		uint8_t _sz = (sz>=4)?4:sz;
		if ( __writeMemory(ST25DV_ADDR_DATA,offset,data,_sz) != __I2C_OK ) {
			ret = ST25DV_FAILED;
			break;
		}
		offset+=_sz;
		data+=_sz;
		sz-=_sz;
	}
    drivers_st25dv_enableFTM();
    __st25dv_config.state = p;
	return ret;

}


/**
 * Read a bloc of sz data into the given blockId into the given user zone
 * blockId is defining a 32b block (the RF block size)
 */
drivers_st25dv_ret_e drivers_st25dv_blocRead(drivers_st25dv_zone_e zone, uint8_t blockId, uint8_t * data, uint8_t sz) {
	drivers_st25dv_sleep_e p = __st25dv_config.state;
	__st25dv_config.state = ST25DV_PROCESSING;

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
    	    __st25dv_config.state = p;
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    default:
    case I_AM_ST25DV04:
    	if ( offset+sz > (ST25DV_ST25DV04K_LAST_ADDRESS+1) ) {
    	    __st25dv_config.state = p;
    		return ST25DV_OUTOFBOUNDS;
    	}
    	break;
    }


	// write block by block
	while ( sz > 0 ) {
		uint8_t _sz = (sz>=4)?4:sz;
		if ( __readMemory(ST25DV_ADDR_DATA,offset,data,_sz) != __I2C_OK ) {
		    __st25dv_config.state = p;
			return ST25DV_FAILED;
		}
		offset+=_sz;
		data+=_sz;
		sz-=_sz;
	}
    __st25dv_config.state = p;
	return ST25DV_SUCCESS;

}

// =========================================================================================
// Fast Transfer Mode - FTP (MailBox) communication
// =========================================================================================


/**
 * The FTM mode is incompatible with User and System Write operations
 * Is needs to be enable / disable regarding this kind of operation
 * FTM will be enabled only if the current mode have FTM enabled
 */
drivers_st25dv_ret_e drivers_st25dv_enableFTM() {
	uint8_t v;
	switch (__st25dv_config.mode) {
	    default:
	    case ST25DV_MODE_FTM:
	    	// enable FTM
			__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
	   		v |= ST25DV_MB_CTRL_DYN_MBEN_MASK;	// enable FTM
			__writeMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
	    	break;
       case ST25DV_MODE_SERIALUZ:
       case ST25DV_MODE_SIMPLE:
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
	    case ST25DV_MODE_FTM:
	    	// enable FTM
			__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
	   		v &= ~ST25DV_MB_CTRL_DYN_MBEN_MASK;	// disable FTM
			__writeMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
	    	break;
       case ST25DV_MODE_SERIALUZ:
       case ST25DV_MODE_SIMPLE:
		    break;
	}
	return ST25DV_SUCCESS;
}


/**
 * Send bytes to the FTM-MB
 */
drivers_st25dv_ret_e drivers_st25dv_ftmWrite(uint8_t * messages, uint16_t sz) {
	if ( sz > ST25DV_I2C_MB_SIZE ) return ST25DV_FAILED;
	if ( __st25dv_config.state == ST25DV_SLEEPING ) return ST25DV_FAILED;

	if ( drivers_st25dv_ftmFreeForWriting() == ST25DV_EMPTYFTM ) {
		// allgood
		if ( __writeMemory(ST25DV_ADDR_DATA, ST25DV_MAILBOX_RAM_REG, messages, sz) == __I2C_OK ) {
			return ST25DV_SUCCESS;
		}
	}
	return ST25DV_FAILED;
}

/**
 * Get bytes from the FTM-MB
 */
drivers_st25dv_ret_e drivers_st25dv_ftmRead(uint8_t * messages, uint16_t sz) {
	if ( sz > ST25DV_I2C_MB_SIZE ) return ST25DV_FAILED;
	if ( __st25dv_config.state == ST25DV_SLEEPING ) return ST25DV_FAILED;

	if ( __readMemory(ST25DV_ADDR_DATA,ST25DV_MAILBOX_RAM_REG, messages, sz) == __I2C_OK ) {
		return ST25DV_SUCCESS;
	}
	return ST25DV_FAILED;
}


/**
 * Check if we have pending bytes in the FTM queue, the len is return when data are pending
 * Returns:
 * - ST25DV_SUCCESS when we have some
 * - ST25DV_EMPTYFTM when none
 */
drivers_st25dv_ret_e drivers_st25dv_ftmAvailableToRead(uint16_t * len) {

	uint8_t v;
	if ( __st25dv_config.state == ST25DV_SLEEPING ) return ST25DV_FAILED;

 	__readMemory(ST25DV_ADDR_DATA,ST25DV_MB_CTRL_DYN_REG,&v,1);
 	if ( (v & ST25DV_MB_CTRL_DYN_RFPUTMSG_MASK) > 0 ) {
 		__readMemory(ST25DV_ADDR_DATA,ST25DV_MBLEN_DYN_REG,&v,1);
 		*len = ((uint16_t)v)+1;
 		return ST25DV_SUCCESS;
 	}
 	*len=0;
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
 	 	if ( (v & ST25DV_MB_CTRL_DYN_RFPUTMSG_MASK ) > 0 ) {
 	 		return ST25DV_NONEMPTYFTM_RF;
 	 	}
 	 	if ( (v & ( ST25DV_MB_CTRL_DYN_HOSTPUTMSG_MASK ) ) > 0 ) {
 	 		return ST25DV_NONEMPTYFTM_HOST;
 	 	}
 	 	return ST25DV_EMPTYFTM;
 	}
 	return ST25DV_FAILED;			// MBEN = 0 - FTM is unactivated

}


#if ITSDK_DRIVERS_ST25DV_WITH_SERIALFTM == __ENABLE


/**
 * Create the memory structure
 */
drivers_st25dv_ret_e drivers_st25dv_enableSerialFtm(drivers_st25dv_mode_e mode) {
	switch ( mode ) {
	case ST25DV_MODE_FTM:
		break;
	default:
		return ST25DV_FAILED;
	}

	drivers_st25dv_ret_e ret = ST25DV_SUCCESS;
	__st25dv_config.readIndexFtm = ST25DV_SERIAL_EMPTYBUF;
	return ret;
}


/**
 * Get if pending a char for the buffer.
 * We read all the Host data once at a time and fill a local buffer
 * this function will read the local buffer then try to get data
 * from the host.
 */
serial_read_response_e drivers_st25dv_serialFtm_read(char * ch) {
	// We have char to read in the local buffer
	if ( __st25dv_config.readIndexFtm != ST25DV_SERIAL_EMPTYBUF ) {
		if ( __st25dv_config.readIndexFtm < __st25dv_config.readSzFtm ) {
			*ch = __st25dv_config.readBufFtm[__st25dv_config.readIndexFtm];
			__st25dv_config.readIndexFtm++;
			if ( __st25dv_config.readIndexFtm == __st25dv_config.readSzFtm ) {
				__st25dv_config.readIndexFtm = ST25DV_SERIAL_EMPTYBUF;
				return SERIAL_READ_SUCCESS;
			} else {
				return SERIAL_READ_PENDING_CHAR;
			}
		} else {
			// should not be here
			__st25dv_config.readIndexFtm = ST25DV_SERIAL_EMPTYBUF;
		}
	}

	// The local buffer is empty, let's see if we have some in the NFC
	// but if we are not in the field we have no reason to do it
	if ( __st25dv_config.field != ST25DV_IN_THE_FIELD ) return SERIAL_READ_NOCHAR;

	drivers_st25dv_goWakeUp();
	__st25dv_config.state = ST25DV_PROCESSING;

	serial_read_response_e ret = SERIAL_READ_NOCHAR;
	uint16_t len;
	if ( drivers_st25dv_ftmAvailableToRead(&len) == ST25DV_SUCCESS ) {
		// We have something to read
	    ret = SERIAL_READ_SUCCESS;
		if ( drivers_st25dv_ftmRead(__st25dv_config.readBufFtm, len) == ST25DV_FAILED ) {
			ret = SERIAL_READ_FAILED;
		} else {
			for ( int k = 0 ; k < len ; k++ ) {
				if ( __st25dv_config.readBufFtm[k] == 0 ) {
				   // We have finished to read the string
				   __st25dv_config.readIndexFtm = 0;
				   __st25dv_config.readSzFtm = k;
				} else {
					if (    ( __st25dv_config.readBufFtm[k] < 7 )
						 || ( __st25dv_config.readBufFtm[k] > 13 && __st25dv_config.readBufFtm[k] < 32 )
						 || ( __st25dv_config.readBufFtm[k] > 126 )
					) {
						__st25dv_config.readBufFtm[k] = ' ';
					}
				}
			}
			if ( __st25dv_config.readIndexFtm == ST25DV_SERIAL_EMPTYBUF ) {
				// Buffer is full but this is not the end of line yet
				__st25dv_config.readIndexFtm = 0;
				__st25dv_config.readSzFtm = ST25DV_SERIALFTM_HOSTBUF_SIZE;
				__st25dv_config.readBufFtm[ST25DV_SERIALFTM_HOSTBUF_SIZE-1]=0;
			}
			*ch = __st25dv_config.readBufFtm[__st25dv_config.readIndexFtm];
			__st25dv_config.readIndexFtm++;
			if (__st25dv_config.readIndexFtm < __st25dv_config.readSzFtm ) {
				ret = SERIAL_READ_PENDING_CHAR;
			}
		}
	} else {
		ret = SERIAL_READ_NOCHAR;
	}
	__st25dv_config.state = ST25DV_WAKEUP;
	drivers_st25dv_goLowPower();
	return ret;
}

/**
 * Print a string into the serialFtm buffer.
 * Directly write the NFC memory if the NFC is ready for this
 * - Nothing waiting for beeing read by MCU
 * - Nothing pending to read by Host
 * If something is pending to read by host, we wait until a timeout, then forget.
 * The device will poll the RF side to get the response. As a consequence the
 * I2C interface may be busy, the program try to bypass this by repeating I2C access
 */
static void _drivers_st25dv_serialFtm_print(char * msg) {
	int try = 0;
	drivers_st25dv_ret_e ret;
	while ( ( ret = drivers_st25dv_ftmFreeForWriting()) != ST25DV_EMPTYFTM && try < (ST25DV_SERIALFTM_MAXRDTRY/50)) {
		// previously written data pending - need to wait a bit.
		#if ITSDK_WDG_MS > 0
		   wdg_refresh();
		#endif
		itsdk_delayMs(50);
		try++;
	}
	if (try == (ST25DV_SERIALFTM_MAXRDTRY/50) ) {
		// wait fail
		switch (ret) {
		case ST25DV_NONEMPTYFTM_HOST:
			// pending host message - overwrite (RF too slow)
			break;
		case ST25DV_NONEMPTYFTM_RF:
			// pending rf message - overwrite (we should not have)
			break;
		default:
			return;
		}
	}

	// Write the data to the buffer
	uint16_t len = 0;
	for ( int i = 0 ; i < ST25DV_I2C_MB_SIZE-1 ; i++ ) {
		if ( msg[i] == 0 ) break;
		len++;
	}
	msg[len]=0;
	len++;
	drivers_st25dv_ftmWrite((uint8_t *)msg,len);
	return;
}

void drivers_st25dv_serialFtm_print(char * msg) {
	drivers_st25dv_goWakeUp();
	__st25dv_config.state = ST25DV_PROCESSING;
	_drivers_st25dv_serialFtm_print(msg);
	__st25dv_config.state = ST25DV_WAKEUP;
	drivers_st25dv_goLowPower();
}

void drivers_st25dv_serialFtm_println(char * msg) {
	char t[ST25DV_SERIALFTM_HOSTBUF_SIZE];
	int i;
	for ( i = 0 ; i < ST25DV_SERIALFTM_HOSTBUF_SIZE - 2 ; i++ ) {
		if ( msg[i] != 0 ) {
			t[i]=msg[i];
		} else break;
	}
	t[i]='\n';
	t[i+1]=0;
	drivers_st25dv_serialFtm_print(t);
}


#endif // ITSDK_DRIVERS_ST25DV_WITH_SERIALFTM


#if ITSDK_DRIVERS_ST25DV_WITH_SERIALUZ == __ENABLE
// ====================================================================
// SERIAL COM OVER USER ZONE
// This module allows to create a zone in user land where the host can
// send command and get response from the MCU. This can be connected to
// the console feature.
// The Host switch hostIsWriting then write its command into the host
// memory zone. when done it change hostIsWriting and hostWriteDone. It
// also switch hostReadDone back to zero.
//
// The Mcu checks the hostWriteDone flag to read the host buffer and
// switch hostWriteDone back to 0. it switch mcuIsWriting to 1 during
// write process and then switch mcuWriteDone + mcuReadDone.
//
// The Mcu proceed the command and send the response to the Host. The
// host command is always 1 line when the Mcu response can be larger.
// The host read the buffer and switch hostReadDone to 1. The Mcu can
// send the next part of the response. The sequence is blocking for the
// MCU until a timeout. The timeout fails the communication.
//
// The data communicated over this channel are clear text data. The
// strings are terminated by a 0.
// ====================================================================

/**
 * Create the memory structure
 */
drivers_st25dv_ret_e drivers_st25dv_enableSerialUz(drivers_st25dv_mode_e mode) {
	switch ( mode ) {
	case ST25DV_MODE_SERIALUZ:
		break;
	default:
		return ST25DV_FAILED;
	}

	// write header whatever ... we are restarting
	drivers_st25dv_ret_e ret = ST25DV_SUCCESS;
	drivers_st25dv_serial_header_t head;
	head.magic = ST25DV_SERIALUZ_MAGIC;
	head.hostRfu = 0;
	head.hostIsWriting = 0;
	head.hostReadDone = 1;
	head.hostWriteDone = 0;
	head.mcuRfu = 0;
	head.mcuIsWriting = 0;
	head.mcuReadDone = 0;
	head.mcuWriteDone = 0;
	if ( drivers_st25dv_blocWrite(ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE, ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET, (uint8_t*)&head, sizeof(head)) != ST25DV_SUCCESS ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_SERIALUZWR,0);
		ret = ST25DV_FAILED;
	}
	__st25dv_config.readIndex = ST25DV_SERIAL_EMPTYBUF;

	return ret;
}

/**
 * Get if pending a char for the buffer.
 * We read all the Host data once at a time and fill a local buffer
 * this function will read the local buffer then try to get data
 * from the host.
 */
serial_read_response_e drivers_st25dv_serialUz_read(char * ch) {

	// We have char to read in the local buffer
	if ( __st25dv_config.readIndex != ST25DV_SERIAL_EMPTYBUF ) {
		if ( __st25dv_config.readIndex < __st25dv_config.readSz ) {
			*ch = __st25dv_config.readBuf[__st25dv_config.readIndex];
			__st25dv_config.readIndex++;
			if ( __st25dv_config.readIndex == __st25dv_config.readSz ) {
				__st25dv_config.readIndex = ST25DV_SERIAL_EMPTYBUF;
				return SERIAL_READ_SUCCESS;
			} else {
				return SERIAL_READ_PENDING_CHAR;
			}
		} else {
			// should not be here
			__st25dv_config.readIndex = ST25DV_SERIAL_EMPTYBUF;
		}
	}

	// The local buffer is empty, let's see if we have some in the NFC
	// but if we are not in the field we have no reason to do it
	if ( __st25dv_config.field != ST25DV_IN_THE_FIELD ) return SERIAL_READ_NOCHAR;

	drivers_st25dv_goWakeUp();
	__st25dv_config.state = ST25DV_PROCESSING;

	serial_read_response_e ret = SERIAL_READ_NOCHAR;
	drivers_st25dv_serial_header_t head;
	if ( drivers_st25dv_blocRead(ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE, ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET, (uint8_t*)&head, sizeof(drivers_st25dv_serial_header_t)) != ST25DV_SUCCESS ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_SERIALUZRD,0);
		ret = SERIAL_READ_FAILED;
	} else {
		if ( head.hostWriteDone == 1 ) {
			// read the host buffer
		    ret = SERIAL_READ_SUCCESS;
			for ( int i = 0 ; i < ST25DV_SERIALUZ_HOSTBUF_SIZE/4 ; i++ ) {
				if ( drivers_st25dv_blocRead(
						ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE,
						ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET+(sizeof(drivers_st25dv_serial_header_t)/4)+i,
						(uint8_t*)& __st25dv_config.readBuf[4*i],
						4 ) != ST25DV_SUCCESS ) {
					ret = SERIAL_READ_FAILED;
					break;
				} else {
					int found = 0;
					for ( int k = 4*i ; k < 4*i+4 ; k++ ) {
						if ( __st25dv_config.readBuf[k] == 0 ) {
						   // We have finished to read the string
						   __st25dv_config.readIndex = 0;
						   __st25dv_config.readSz = k;
						   found=1;
						   break;
						} else {
							if (    ( __st25dv_config.readBuf[k] < 7 )
								 || ( __st25dv_config.readBuf[k] > 13 && __st25dv_config.readBuf[k] < 32 )
								 || ( __st25dv_config.readBuf[k] > 126 )
							) {
								__st25dv_config.readBuf[k] = ' ';
							}
						}
					}
					if ( found == 1 ) break;
				}
			}
			// check status
			if ( ret == SERIAL_READ_SUCCESS ) {
				if ( __st25dv_config.readIndex == ST25DV_SERIAL_EMPTYBUF ) {
					// Buffer is full but this is not the end of line yet
					__st25dv_config.readIndex = 0;
					__st25dv_config.readSz = ST25DV_SERIALUZ_HOSTBUF_SIZE;
					__st25dv_config.readBuf[ST25DV_SERIALUZ_HOSTBUF_SIZE-1]=0;
				}
				head.mcuReadDone=1;
				*ch = __st25dv_config.readBuf[__st25dv_config.readIndex];
				__st25dv_config.readIndex++;
				if (__st25dv_config.readIndex < __st25dv_config.readSz ) {
					ret = SERIAL_READ_PENDING_CHAR;
				}
			}
			head.hostWriteDone=0;
			if ( drivers_st25dv_blocWrite(ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE, ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET, (uint8_t*)&head, sizeof(head)) != ST25DV_SUCCESS ) {
				ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_ST25DV_SERIALUZWR,0);
				ret = SERIAL_READ_FAILED;
			}
		} else {
			ret = SERIAL_READ_NOCHAR;
		}
	}
	__st25dv_config.state = ST25DV_WAKEUP;
	drivers_st25dv_goLowPower();
	return ret;
}




/**
 * Print a string into the serialUz buffer.
 * Directly write the NFC memory if the NFC is ready for this
 * - Nothing waiting for beeing read by MCU
 * - Nothing pending to read by Host
 * If something is pending to read by host, we wait until a timeout, then forget.
 * The device will poll the RF side to get the response. As a consequence the
 * I2C interface may be busy, the program try to bypass this by repeating I2C access
 */
static void _drivers_st25dv_serialUz_print(char * msg) {
	drivers_st25dv_serial_header_t head;
	uint8_t try=0;
	if ( drivers_st25dv_blocRead(ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE, ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET, (uint8_t*)&head, sizeof(drivers_st25dv_serial_header_t)) != ST25DV_SUCCESS )
		return;

	// check feasibility
	if ( head.hostIsWriting == 1 || head.hostWriteDone == 1 ) {
		// read pending - failed to write
		return;
	}
	try = 0;
	while ( head.mcuWriteDone == 1 && head.hostReadDone == 0 && try < (ST25DV_SERIALUZ_MAXRDTRY/50)) {
		// previously written data pending - need to wait a bit.
		#if ITSDK_WDG_MS > 0
		   wdg_refresh();
		#endif
		itsdk_delayMs(50);
		drivers_st25dv_blocRead(ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE, ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET, (uint8_t*)&head, sizeof(drivers_st25dv_serial_header_t));
		try++;
	}
	if ( head.mcuWriteDone == 1 && head.hostReadDone == 0 ) {
		// wait fail
		return;
	}
	head.mcuIsWriting = 1;
	head.hostReadDone = 0;
	drivers_st25dv_blocWrite(ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE, ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET, (uint8_t*)&head, sizeof(head));

	// Write the data to the buffer
	uint8_t len = 0;
	uint8_t end = 0;
	do {
		if ( msg[len+0]==0||msg[len+1]==0||msg[len+2]==0||msg[len+3]==0 ) end = 1;
		if (drivers_st25dv_blocWrite(
				ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE,
				ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET+(sizeof(drivers_st25dv_serial_header_t)/4)+len/4,
				(uint8_t*)&msg[len],
				4
			   ) != ST25DV_SUCCESS )  return;
		len += 4;
	} while ( len < ST25DV_SERIALUZ_HOSTBUF_SIZE && end == 0 );

	head.mcuIsWriting = 0;
	head.mcuWriteDone = 1;
	drivers_st25dv_blocWrite(ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE, ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET, (uint8_t*)&head, sizeof(head));
	return;
}

void drivers_st25dv_serialUz_print(char * msg) {
	drivers_st25dv_goWakeUp();
	__st25dv_config.state = ST25DV_PROCESSING;
	_drivers_st25dv_serialUz_print(msg);
	__st25dv_config.state = ST25DV_WAKEUP;
	drivers_st25dv_goLowPower();
}

void drivers_st25dv_serialUz_println(char * msg) {
	char t[ST25DV_SERIALUZ_HOSTBUF_SIZE];
	int i;
	for ( i = 0 ; i < ST25DV_SERIALUZ_HOSTBUF_SIZE - 2 ; i++ ) {
		if ( msg[i] != 0 ) {
			t[i]=msg[i];
		} else break;
	}
	t[i]='\n';
	t[i+1]=0;
	drivers_st25dv_serialUz_print(t);
}

#endif	// ITSDK_DRIVERS_ST25DV_WITH_SERIALUZ

#if ITSDK_DRIVERS_ST25DV_WITH_SERIALUZ == __ENABLE || ITSDK_DRIVERS_ST25DV_WITH_SERIALFTM == __ENABLE
#if ITSDK_WITH_CONSOLE == __ENABLE
/**
 * Link to the console when activated
 */
void itsdk_console_customSerial_print(char * msg) {
	if ( __st25dv_config.ready == ST25DV_NOTREADY ) return;

#if ITSDK_DRIVERS_ST25DV_WITH_SERIALUZ == __ENABLE
    if ( __st25dv_config.mode == ST25DV_MODE_SERIALUZ) {
    	drivers_st25dv_serialUz_print(msg);
    }
#endif
#if ITSDK_DRIVERS_ST25DV_WITH_SERIALFTM == __ENABLE
    if ( __st25dv_config.mode == ST25DV_MODE_FTM) {
    	drivers_st25dv_serialFtm_print(msg);
    }
#endif
}

serial_read_response_e itsdk_console_customSerial_read(char * ch) {
	serial_read_response_e r = SERIAL_READ_NOCHAR;
	if ( __st25dv_config.ready == ST25DV_NOTREADY ) return SERIAL_READ_NOCHAR;

#if ITSDK_DRIVERS_ST25DV_WITH_SERIALUZ == __ENABLE
    if ( __st25dv_config.mode == ST25DV_MODE_SERIALUZ) {
    	 r = drivers_st25dv_serialUz_read(ch);
    }
#endif
#if ITSDK_DRIVERS_ST25DV_WITH_SERIALFTM == __ENABLE
    if ( __st25dv_config.mode == ST25DV_MODE_FTM) {
    	r = drivers_st25dv_serialFtm_read(ch);
    }
#endif
    return r;
}

#endif
#endif // ITSDK_WITH_CONSOLE

#endif

#endif // ITSDK_WITH_DRIVERS

