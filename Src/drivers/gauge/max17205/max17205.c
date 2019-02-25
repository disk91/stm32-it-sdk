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
	v &= ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MSK;
	__max17205_config.devType = ( v & 0xFF );


//	if (    __max17205_config.devType != MAX17205_TYPE_SINGLE_CELL
//		 && __max17205_config.devType != MAX17205_TYPE_MULTI_CELL
//		 && __max17205_config.devType != MAX17205_TYPE_MULTI_CELL2 ) {
//		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_MAX17205_NOTFOUND,0);
//		return MAX17205_NOTFOUND;
//	}

	if (ITSDK_DRIVERS_MAX17205_ALRT1_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_MAX17205_ALRT1_BANK,ITSDK_DRIVERS_MAX17205_ALRT1_PIN,GPIO_INTERRUPT_RISING);
		gpio_registerIrqAction(&__max17205_gpio_irq);
	}

	// Reset the device
	__writeRegister(ITSDK_DRIVERS_MAX17205_REG_COMMAND_ADR,0x000F);
	itsdk_delayMs(15);
	__writeRegister(ITSDK_DRIVERS_MAX17205_REG_CONFIG2_ADR,0x0001);
	itsdk_delayMs(15);


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

	return MAX17205_SUCCESS;
}

/**
 * Return the temperature in m°C
 */
drivers_max17205_ret_e drivers_max17205_getTemperature(int32_t * mTemp) {
	int16_t v;
	if ( __readRegister(ITSDK_DRIVERS_MAX17205_REG_TEMP_ADR,(uint16_t*)&v) != I2C_OK ) {
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
	if ( __readRegister(regAdr,&v) != I2C_OK ) {
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
		break;
	}
	log_info("mVolt : %d (v %d)\r\n",*mVolt,v);
	return MAX17205_SUCCESS;
}

/**
 * Return Current in uA
 */
drivers_max17205_ret_e drivers_max17205_getCurrent(int32_t * uAmp) {

	int16_t v;
	if ( __readRegister(ITSDK_DRIVERS_MAX17205_REG_CURRENT_ADR,(uint16_t *)&v) != I2C_OK ) {
		return MAX17205_FAILED;
	}

	*uAmp = (int32_t)(((int64_t)(v) * 15625)/(10*ITSDK_DRIVERS_MAX17205_RSENSE_MOHM));
	log_info("uA : %d (v %d)\r\n",*uAmp,v);
	return MAX17205_SUCCESS;
}


/**
 * Return Current in uA
 */
drivers_max17205_ret_e drivers_max17205_getCoulomb(uint16_t * coulomb) {

	uint16_t v;
	if ( __readRegister(ITSDK_DRIVERS_MAX17205_REG_QH_ADR,&v) != I2C_OK ) {
		return MAX17205_FAILED;
	}

	*coulomb = v;
	log_info("coulomb : %d\r\n",*coulomb);
	return MAX17205_SUCCESS;
}




#endif // ITSDK_DRIVERS_MAX17205
