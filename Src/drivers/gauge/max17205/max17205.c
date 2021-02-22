/* ==========================================================
 * max17205.c - Maxim 17205 - Gauge 3 cells
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
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

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
 * The MAX17201/5 use a Restart between Reg Address and Read/Write operations, like a Memory
 */
static _I2C_Status __readRegister(uint16_t addr, uint16_t * value) {
	uint8_t devAdr = ITSDK_DRIVERS_MAX17205_ADDRESS_000_0FF;
	if ( addr >= 0x100 ) {
		devAdr = ITSDK_DRIVERS_MAX17205_ADDRESS_100_1FF;
	}
	uint8_t b[2];
	_I2C_Status r = i2c_memRead(
			&ITSDK_DRIVERS_MAX17205_I2C,				// i2c handler
			devAdr,										// Device Address => 7 bits non shifted
			addr,										// Memory address to access
			8,											// 8 for 8b, 16 for 16 bits ...
			b,											// Data to be read
			2
	);
	*value = b[0] + (uint16_t)b[1]*256;
	return r;
}

/**
 * Write 16b values
 * The MAX17201/5 use a Restart between Reg Address and Read/Write operations, like a Memory
 */
static _I2C_Status __writeRegister(uint16_t addr, uint16_t value) {
	uint8_t devAdr = ITSDK_DRIVERS_MAX17205_ADDRESS_000_0FF;
	if ( addr >= 0x100 ) {
		devAdr = ITSDK_DRIVERS_MAX17205_ADDRESS_100_1FF;
	}
	uint8_t b[2];
	b[0] = value & 0xFF;
	b[1] = value >> 8;
	return i2c_memWrite(
			&ITSDK_DRIVERS_MAX17205_I2C,				// i2c handler
			devAdr,										// Device Address => 7 bits non shifted
			addr,										// Memory address to access
			8,											// 8 for 8b, 16 for 16 bits ...
			b,											// Data to be read
			2
	);
}

/**
 * Interrupt handler
 */
static void __max17205_interrupt(uint16_t GPIO_Pin) {
	log_info("MAX17205 Int\r\n");

}

static gpio_irq_chain_t __max17205_gpio_irq = {
		__max17205_interrupt,
		ITSDK_DRIVERS_MAX17205_ALRT1_PIN,
		NULL,
};

/**
 * Function to detect if the switch from value A to B causes an overflow.
 * Return 1 if overflow, 0 otherwise.
 */
static uint8_t __overflow_detected(uint16_t p_u16A, uint16_t p_u16B)
{
	uint8_t l_u8Overflow = 0u;
	if (p_u16A > (UINT16_MAX - p_u16B))	{
		l_u8Overflow = 1u;
	}
	return l_u8Overflow;
}


/**
 * Setup the sensor according to the selected mode.
 */
drivers_max17205_ret_e drivers_max17205_setup(drivers_max17205_mode_e mode) {

	__max17205_config.mode = mode;
	__max17205_config.initialized = MAX17205_FAILED;
	__max17205_config.lastCapa = 0;
	__max17205_config.totalCapa = 0;

	uint16_t v;


	if (   __readRegister(ITSDK_DRIVERS_MAX17205_REG_DEVNAME_ADR,&v) == __I2C_OK
	    && (( v & ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MSK ) != MAX17205_TYPE_SINGLE_CELL )
		&& (( v & ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MSK ) != MAX17205_TYPE_MULTI_CELL  )
	) {

		// Preconfiguration related to MAX17205 bug - thank you Martin Cornu
		// see https://github.com/disk91/stm32-it-sdk/issues/26
		switch ( mode ) {
			case MAX17205_MODE_3CELLS_INT_TEMP:
				// 3 cells - 0x00 -- 03 -- Num of cells - in shadow ram
				__writeRegister(ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_ADR,
								  ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_VBAT_DISABLE
								| ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CHEN_DISABLE
								| ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TDEN_DISABLE
								| ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A1EN_DISABLE
								| ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_FGT_DISABLE
						        | 3 << ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_NCELL_SHIFT
							    );
				// Also in volatile RAM
				__writeRegister(ITSDK_DRIVERS_MAX17205_REG_BNPACKCFG_ADR,
								  ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_VBAT_DISABLE
								| ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CHEN_DISABLE
								| ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TDEN_DISABLE
								| ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A1EN_DISABLE
								| ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_FGT_DISABLE
						        | 3 << ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_NCELL_SHIFT
							    );
				itsdk_delayMs(1000);
				break;
			default:
			case MAX17205_MODE_DEFAULT:
				break;

		}

		// We can assume this is the first run and the chip configuration is invalid
		// we are going to fix it by storing the right value in the NV Memory. But this
		// memory can we updated only 7 times... so we are going to try to update it not too
		// much until the device could be locked

		// We can assume this is the first run and the chip configuration is invalid
		// we are going to fix it

		// The current status of this patch is the following. Sometime we have seen it
		// working but the condition to get the NV Memory written is still not clear.
		// most of the time the NVErr flag is position after writing for unidentified reason
		// yet. As a consequence we lost awrite possibility and the value is unchanged. But
		// sometime it worked. More tests are needed to understand this.
		__readRegister(ITSDK_DRIVERS_MAX17205_REG_NHIBCFG,&v);
		v &= ~(ITSDK_DRIVERS_MAX17205_REG_NHIBCFG_ENHIB_MSK);
		__writeRegister(ITSDK_DRIVERS_MAX17205_REG_NHIBCFG,v);
		if ( drivers_max17205_getRemainingNVMUpdates(&v) == MAX17205_SUCCESS ) {

			if ( v > 5 ) {
			    int _try = 0;
				do {
					// Repeat until CommStat.NVError == 0
					int _trywait = 0;
					do {
						__readRegister(ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_ADR,&v);
						itsdk_delayMs(10);
						_trywait++;
					} while ((v & ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_NVBUSY_MSK) != 0 && _trywait < 10);

					v &= ~ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_NVERR_MSK; // force NVError = 0
					__writeRegister(ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_ADR,v);
					__writeRegister(ITSDK_DRIVERS_MAX17205_REG_COMMAND_ADR,ITSDK_DRIVERS_MAX17205_CMD_BLOCKCPY);
					for ( int l = 0 ; l < ITSDK_DRIVERS_MAX17205_TIME_FOR_NVSAVE_100MS_LOOP ; l++ ) {
						#if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
						   wdg_refresh();
						#endif
						itsdk_delayMs(100);
					}
					__readRegister(ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_ADR,&v);
					_try++;
				} while ( (v & ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_NVERR_MSK) != 0 && _try < ITSDK_DRIVERS_MAX17205_NVSAVE_MAX_TRY );
				if ( (v & ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_NVERR_MSK) != 0 ) {
					// Too many failed in trying to store the NV Memory
					ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX17205_NVFAILED,0);
				} else {
					// Sound good - reset the device
					__writeRegister(ITSDK_DRIVERS_MAX17205_REG_COMMAND_ADR,0x000F);
					itsdk_delayMs(15);
					__writeRegister(ITSDK_DRIVERS_MAX17205_REG_CONFIG2_ADR,0x0001);
					itsdk_delayMs(15);
				}

			} else {
				// We prefer to not update the device to avoid locking it
				ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX17205_NVUPDREM,v);
				// By the way, it seems this can help to correctly detect the chip
				itsdk_delayMs(1000);
			}

		} else {
			// not a success, basically the device could not be here
			// make a try for a wait ... just because
			// This is for the debug period @TODO remove this part of the code
			itsdk_delayMs(1000);
			log_error("Max17205 is another time not found\r\n");
		}
		__readRegister(ITSDK_DRIVERS_MAX17205_REG_NHIBCFG,&v);
		v |= ITSDK_DRIVERS_MAX17205_REG_NHIBCFG_ENHIB_MSK;
		__writeRegister(ITSDK_DRIVERS_MAX17205_REG_NHIBCFG,v);

	}


	// Search for the device type ( aging but this time it is supposed to be ok
	if ( __readRegister(ITSDK_DRIVERS_MAX17205_REG_DEVNAME_ADR,&v) != __I2C_OK ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX17205_NOTFOUND,0);
		return MAX17205_NOTFOUND;
	}
	v &= ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MSK;
	__max17205_config.devType = ( v & 0xFF );

	if (    __max17205_config.devType != MAX17205_TYPE_SINGLE_CELL
		 && __max17205_config.devType != MAX17205_TYPE_MULTI_CELL
	) {
		// Reset the device to try to save it from bad shape
		__writeRegister(ITSDK_DRIVERS_MAX17205_REG_COMMAND_ADR,0x000F);
		itsdk_delayMs(15);
		__writeRegister(ITSDK_DRIVERS_MAX17205_REG_CONFIG2_ADR,0x0001);
		itsdk_delayMs(15);
		__readRegister(ITSDK_DRIVERS_MAX17205_REG_DEVNAME_ADR,&v);
		v &= ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MSK;
		__max17205_config.devType = ( v & 0xFF );
	}

	// Retry to identify the device, eventually fail
	if (    __max17205_config.devType != MAX17205_TYPE_SINGLE_CELL
		 && __max17205_config.devType != MAX17205_TYPE_MULTI_CELL
	) {
		// This can be due to undervoltage powering.
		// When the power supply is under 5V ( level not tested but higher the spec minimal documented level )
		//  the devId returned is just shit ; The device works but really badly
		uint16_t vbat=0;
		drivers_max17205_getVoltage(MAX17205_VBAT,&vbat);
		if ( vbat < ITSDK_DRIVERS_MAX17205_UNDERVOLTAGE ) {
			// assuming this is the undervoltage condition
			ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX17205_UNDERVOLT,(uint8_t)(vbat/100));
			__max17205_config.initialized = MAX17205_UNDERVOLT;
			return MAX17205_UNDERVOLT;
		}
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX17205_NOTFOUND,0);
		return MAX17205_NOTFOUND;
	}

	if (ITSDK_DRIVERS_MAX17205_ALRT1_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_MAX17205_ALRT1_BANK,ITSDK_DRIVERS_MAX17205_ALRT1_PIN,GPIO_INTERRUPT_RISING);
		gpio_registerIrqAction(&__max17205_gpio_irq);
	}

	// Reset the device
	__writeRegister(ITSDK_DRIVERS_MAX17205_REG_COMMAND_ADR,0x000F);
	itsdk_delayMs(15);
	__writeRegister(ITSDK_DRIVERS_MAX17205_REG_CONFIG2_ADR,0x0001);
	itsdk_delayMs(15);


	// Configure device
	switch ( mode ) {
	case MAX17205_MODE_3CELLS_INT_TEMP:
		// 3 cells + internal temperature management
		__readRegister(ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_ADR,&v);	// read conf
		v &= ~ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_NCELL_MSK;
		v |= 3 << ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_NCELL_SHIFT;		// 3 cells
//		v &= ~ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_BALCFG_MSK;
//		v |= 6 << ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_BALCFG_SHIFT;		// balancing
		v &= ~ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TEMP_MSK;
		v |= MAX17205_REG_NPACKCFG_TEMP_INTERNAL_DIETEMP;
		v &= ~ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CELLX_MSK;
		v |= ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CHEN_ENABLE;
		v |= ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_VBAT_ENABLE;
		v |= ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TDEN_ENABLE;
		__writeRegister(ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_ADR,v);
		__writeRegister(ITSDK_DRIVERS_MAX17205_REG_BNPACKCFG_ADR,v);
		break;

	default:
	case MAX17205_MODE_DEFAULT:
		break;

	}

	#if defined ITSDK_DRIVERS_MAX17205_CAPA_MAX && ITSDK_DRIVERS_MAX17205_CAPA_MAX > 0
	 // Configure design capacity
	 // Check if DesignCap bit is enable
	 __readRegister(ITSDK_DRIVERS_MAX17205_REG_NVCFG, &v);
	 if ((v & ITSDK_DRIVERS_MAX17205_REG_NVCFG_DCAP_MSK) == 0) {
	   v |= ITSDK_DRIVERS_MAX17205_REG_NVCFG_DCAP_MSK;
	   __writeRegister(ITSDK_DRIVERS_MAX17205_REG_NVCFG, v);
	 }
	 // Set design capacity value
	 // Maximum value that can be stored is 65 535 * LSB mAh
	 // LSB = 5 / ITSDK_DRIVERS_MAX17205_RSENSE_MOHM
	  v = (uint16_t) (ITSDK_DRIVERS_MAX17205_CAPA_MAX * (ITSDK_DRIVERS_MAX17205_RSENSE_MOHM/5));
	  __writeRegister(ITSDK_DRIVERS_MAX17205_REG_NDCAP, v);
	  __writeRegister(ITSDK_DRIVERS_MAX17205_REG_DCAP, v);
	#endif

	__max17205_config.initialized = MAX17205_SUCCESS;
	return MAX17205_SUCCESS;
}

/**
 * Return the temperature in m�C
 */
drivers_max17205_ret_e drivers_max17205_getTemperature(int32_t * mTemp) {
	int16_t v;
	if ( __readRegister(ITSDK_DRIVERS_MAX17205_REG_TEMP_ADR,(uint16_t*)&v) != __I2C_OK ) {
		return MAX17205_FAILED;
	}
	*mTemp = ((int32_t)v*1000) / 256;
	return MAX17205_SUCCESS;
}

/**
 * Return the Voltage in mV
 */
drivers_max17205_ret_e drivers_max17205_getVoltage(drivers_max17205_cell_select_e cell, uint16_t * mVolt) {

	uint16_t regAdr = 0;
	switch (cell) {
	case MAX17205_CELL1:
		regAdr = ITSDK_DRIVERS_MAX17205_REG_CELL1_VOLT_ADR;
		break;
	case MAX17205_CELL2:
		regAdr = ITSDK_DRIVERS_MAX17205_REG_CELL2_VOLT_ADR;
		break;
	case MAX17205_CELL3:
		regAdr = ITSDK_DRIVERS_MAX17205_REG_CELL3_VOLT_ADR;
		break;
	case MAX17205_CELL4:
		regAdr = ITSDK_DRIVERS_MAX17205_REG_CELL4_VOLT_ADR;
		break;
	case MAX17205_CELLX:
		regAdr = ITSDK_DRIVERS_MAX17205_REG_CELLX_VOLT_ADR;
		break;
	case MAX17205_VBAT:
		regAdr = ITSDK_DRIVERS_MAX17205_REG_VBAT_VOLT_ADR;
		break;
	default:
		return MAX17205_FAILED;
	}

	uint16_t v;
	if ( __readRegister(regAdr,&v) != __I2C_OK ) {
		return MAX17205_FAILED;
	}
	switch (cell) {
	default:
	case MAX17205_CELL1:
	case MAX17205_CELL2:
	case MAX17205_CELL3:
	case MAX17205_CELL4:
	case MAX17205_CELLX:
		*mVolt = (uint16_t)((((uint64_t)v)*78125)/1000000); // Ratio 0.000078125 V / LSB
		break;
	case MAX17205_VBAT:
		*mVolt = v + v / 4;	// 1.25mV / LSB
		// We know that MAX17205_TYPE_MULTI_CELL need at least 5V to work properly
		if ( __max17205_config.devType == MAX17205_TYPE_MULTI_CELL && *mVolt < ITSDK_DRIVERS_MAX17205_UNDERVOLTAGE ) {
			__max17205_config.initialized = 0;
		}
		break;
	}
	return MAX17205_SUCCESS;
}

/**
 * Return Current in uA
 * The value is negative when consuming the power from the batteries.
 * The value is positive when charging the batteries.
 */
drivers_max17205_ret_e drivers_max17205_getCurrent(int32_t * uAmp) {

	int16_t v;
	if ( __readRegister(ITSDK_DRIVERS_MAX17205_REG_CURRENT_ADR,(uint16_t *)&v) != __I2C_OK ) {
		return MAX17205_FAILED;
	}

	*uAmp = (int32_t)(((int64_t)(v) * 15625)/(10*ITSDK_DRIVERS_MAX17205_RSENSE_MOHM));
	return MAX17205_SUCCESS;
}


/**
 * Return Current in mAh
 * In Fact this coulomb counter is not coulomb but Capacity in mAh
 * The counter decrease when the battery is consumed and increased when the battery is charging
 * A a consequence the first returned value is 0xFFFF then 0xFFFE ...
 */
drivers_max17205_ret_e drivers_max17205_getCapacity(uint16_t * mah) {

	uint16_t v;
	if ( __readRegister(ITSDK_DRIVERS_MAX17205_REG_QH_ADR,&v) != __I2C_OK ) {
		return MAX17205_FAILED;
	}
	*mah = v;
	return MAX17205_SUCCESS;
}

/**
 * Return curent Coulomb
 * In Fact this coulomb counter is not coulomb but Capacity in mAh
 * The counter decrease when the battery is consumed and increased when the battery is charging
 * The coulomb return will increment over time
 */
drivers_max17205_ret_e drivers_max17205_getCoulomb(uint32_t * coulomb) {
	uint16_t capa;
	drivers_max17205_getCapacity(&capa);
#if ITSDK_DRIVERS_MAX17205_CSN_TO_BAT == __ENABLE
	if ((capa >= 0xFFFF) || ((capa < __max17205_config.lastCapa) && (__overflow_detected(__max17205_config.lastCapa,capa) == 0u))) {
		return MAX17205_FAILED;
	}
	uint16_t delta = (capa >= __max17205_config.lastCapa )? (capa - __max17205_config.lastCapa) : (capa + (0xFFFF - __max17205_config.lastCapa));
#else
	if ((capa >= 0xFFFF) || ((capa > __max17205_config.lastCapa) && (__overflow_detected(capa,__max17205_config.lastCapa) == 0u))){
		return MAX17205_FAILED;
	}
	uint16_t delta = (capa <= __max17205_config.lastCapa )? __max17205_config.lastCapa - capa : (__max17205_config.lastCapa + (0xFFFF - capa));
#endif
	__max17205_config.totalCapa += delta;
	__max17205_config.lastCapa = capa;

	*coulomb = (ITSDK_DRIVERS_MAX17205_GAUGE_LSB_FACT * __max17205_config.totalCapa) / (ITSDK_DRIVERS_MAX17205_RSENSE_MOHM);
	return MAX17205_SUCCESS;
}


/**
 * Return success when the max17205 is ready for being used
 * Failed if not ready.
 * As the state can dynamically change regarding the battery level it is recommended to
 * call this function before calling any other function (this is not concerning setup)
 */
drivers_max17205_ret_e drivers_max17205_isReady() {
	return __max17205_config.initialized;
}

/**
 * Return the remaining update of the NV Memory
 * The Nv Memory can only be updated 7 times.
 */
drivers_max17205_ret_e drivers_max17205_getRemainingNVMUpdates(uint16_t * upd) {

	uint16_t v;
	__writeRegister(ITSDK_DRIVERS_MAX17205_REG_COMMAND_ADR,ITSDK_DRIVERS_MAX17205_CMD_RECALL);
	itsdk_delayMs(ITSDK_DRIVERS_MAX17205_TIME_FOR_NVRECALL);
	__readRegister(ITSDK_DRIVERS_MAX17205_REG_REMAINUPD_ADR,&v);
	uint16_t l = v & 0xFF;
	uint16_t h = (v >> 8) & 0xFF;
	v = l | h;
	switch (v) {
		case 1: *upd = 7; break;
		case 3: *upd = 6; break;
		case 7: *upd = 5; break;
		case 15: *upd = 4; break;
		case 31: *upd = 3; break;
		case 63: *upd = 2; break;
		case 127: *upd = 1; break;
		case 255: *upd = 0; break;
		default:
			*upd=0;
			return MAX17205_FAILED;
			break;
	}
	return MAX17205_SUCCESS;

}


#endif // ITSDK_DRIVERS_MAX17205

#endif
