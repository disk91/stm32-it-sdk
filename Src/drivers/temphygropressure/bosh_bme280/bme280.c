/* ==========================================================
 * bme280.c - Bosh BME280 Temp / Hygro / Pressure I2C-SPI sensor
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 févr. 2019
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
 * Supports the I2C implementation of the Bosh sensor
 *
 * ==========================================================
 */
#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_BME280 == __ENABLE
#include <drivers/temphygropressure/bosh_bme280/bme280.h>
#include <it_sdk/wrappers.h>

static drivers_bme280_conf_t __bme280_config;

static _I2C_Status __readRegister(uint8_t addr, uint8_t * value) {
	return i2c_read8BRegister(
			&ITSDK_DRIVERS_BME280_I2C,
			ITSDK_DRIVERS_BME280_ADDRESS,
			addr,
			value,
			1
		   );
}

static _I2C_Status __writeRegister(uint8_t addr, uint8_t value) {
	return i2c_write8BRegister(
			&ITSDK_DRIVERS_BME280_I2C,
			ITSDK_DRIVERS_BME280_ADDRESS,
			addr,
			value,
			1
		   );
}


/**
 * Setup the sensor is a given mode after verifying the presence
 * Actually the implemented mode are corresponding to the documentation
 * Custom mode can ba added.
 */
drivers_bme280_ret_e drivers_bme280_setup(drivers_bme280_mode_e mode) {
	// Check device presence
	uint8_t v;
	if (  __readRegister(DRIVER_BME280_REG_ID_ADR,&v) != I2C_OK ) {
		return BME280_NOTFOUND;
	}
	if ( v != DRIVER_BME280_REG_ID_VALUE ) {
		return BME280_NOTFOUND;
	}

	// Configure the device according the selected mode
	switch (mode) {
	default:
	case BME280_MODE_WEATHER_MONITORING:
		// Forced mode
		// Oversampling 1x for all
		__readRegister(DRIVER_BME280_REG_CTRLHUM_ADR,&v);		// Humidity oversampling  x1
		v &= ~DRIVER_BME280_REG_CTRLHUM_OSRD_MASK;
		v |= DRIVER_BME280_REG_CTRLHUM_OSRD_X1;
		__writeRegister(DRIVER_BME280_REG_CTRLHUM_ADR,v);

		__readRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,&v);		// Temperature oversampling x1
		v &= ~DRIVER_BME280_REG_CTRLMEAS_OSRST_MASK;
		v |= DRIVER_BME280_REG_CTRLMEAS_OSRST_X1;

		v &= ~DRIVER_BME280_REG_CTRLMEAS_OSRSP_MASK;			// Pressure oversampling x 1
		v |= DRIVER_BME280_REG_CTRLMEAS_OSRSP_X1;
		__writeRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,v);
		// IIR filter off
		__readRegister(DRIVER_BME280_REG_CONFIG_ADR,&v);		// IIR Filter Off
		v &= ~DRIVER_BME280_REG_CONFIG_FILTER_MASK;
		v |= DRIVER_BME280_REG_CONFIG_FILTER_OFF;
		__writeRegister(DRIVER_BME280_REG_CONFIG_ADR,v);

		// Switch to force mode
		__readRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,&v);		// Get the first data
		v &= ~DRIVER_BME280_REG_CTRLMEAS_MODE_MASK;
		v |= DRIVER_BME280_REG_CTRLMEAS_MODE_FORCED;
		__writeRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,v);

		break;

	case BME280_MODE_HUMIDITY_SENSING:
		// Oversampling 0x pressure 1x for temp & humidity
		__readRegister(DRIVER_BME280_REG_CTRLHUM_ADR,&v);		// Humidity oversampling  x1
		v &= ~DRIVER_BME280_REG_CTRLHUM_OSRD_MASK;
		v |= DRIVER_BME280_REG_CTRLHUM_OSRD_X1;
		__writeRegister(DRIVER_BME280_REG_CTRLHUM_ADR,v);

		__readRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,&v);		// Temperature oversampling x1
		v &= ~DRIVER_BME280_REG_CTRLMEAS_OSRST_MASK;
		v |= DRIVER_BME280_REG_CTRLMEAS_OSRST_X1;

		v &= ~DRIVER_BME280_REG_CTRLMEAS_OSRSP_MASK;			// Pressure oversampling x 0
		v |= DRIVER_BME280_REG_CTRLMEAS_OSRSP_SKIP;
		__writeRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,v);
		// IIR filter off
		__readRegister(DRIVER_BME280_REG_CONFIG_ADR,&v);		// IIR Filter Off
		v &= ~DRIVER_BME280_REG_CONFIG_FILTER_MASK;
		v |= DRIVER_BME280_REG_CONFIG_FILTER_OFF;
		__writeRegister(DRIVER_BME280_REG_CONFIG_ADR,v);

		// Switch to force mode
		__readRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,&v);		// Get the first data
		v &= ~DRIVER_BME280_REG_CTRLMEAS_MODE_MASK;
		v |= DRIVER_BME280_REG_CTRLMEAS_MODE_FORCED;
		__writeRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,v);
		break;

	case BME280_MODE_INDOOR_NAVIGATION:
		// Oversampling 16x pressure 2x for temp & 1x humidity
		__readRegister(DRIVER_BME280_REG_CTRLHUM_ADR,&v);		// Humidity oversampling  x1
		v &= ~DRIVER_BME280_REG_CTRLHUM_OSRD_MASK;
		v |= DRIVER_BME280_REG_CTRLHUM_OSRD_X1;
		__writeRegister(DRIVER_BME280_REG_CTRLHUM_ADR,v);

		__readRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,&v);		// Temperature oversampling x2
		v &= ~DRIVER_BME280_REG_CTRLMEAS_OSRST_MASK;
		v |= DRIVER_BME280_REG_CTRLMEAS_OSRST_X2;

		v &= ~DRIVER_BME280_REG_CTRLMEAS_OSRSP_MASK;			// Pressure oversampling x 16
		v |= DRIVER_BME280_REG_CTRLMEAS_OSRSP_X16;
		__writeRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,v);
		// IIR filter coef 16
		__readRegister(DRIVER_BME280_REG_CONFIG_ADR,&v);		// IIR Filter x16
		v &= ~DRIVER_BME280_REG_CONFIG_FILTER_MASK;
		v |= DRIVER_BME280_REG_CONFIG_FILTER_16;
		__writeRegister(DRIVER_BME280_REG_CONFIG_ADR,v);

		// Switch to normal mode
		__readRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,&v);		// Normal mode
		v &= ~DRIVER_BME280_REG_CTRLMEAS_MODE_MASK;
		v |= DRIVER_BME280_REG_CTRLMEAS_MODE_NORMAL;
		__writeRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,v);
		__readRegister(DRIVER_BME280_REG_CONFIG_ADR,&v);		// Refresh rate 0,5ms
		v &= ~DRIVER_BME280_REG_CONFIG_TSB_MASK;
		v |= DRIVER_BME280_REG_CONFIG_TSB_0_5MS;
		__writeRegister(DRIVER_BME280_REG_CONFIG_ADR,v);
		break;
	}
	__bme280_config.mode = mode;
	return BME280_SUCCESS;
}


static int32_t __compensate_Temp(int32_t adc) {
	int32_t var1,var2,t;
//	var1 = ((((adc>>3) - ((digt1)) )))
}


/**
 * Depends on mode, get the last sensor values or request a new value and get it
 * Get the sensors value
 */
drivers_bme280_ret_e drivers_bme280_getSensors(
		int32_t  * temperature,
		int32_t  * pressure,
		uint32_t * humidity
) {
	uint8_t v;
	switch (__bme280_config.mode) {
	default:
	case BME280_MODE_WEATHER_MONITORING:
	case BME280_MODE_HUMIDITY_SENSING:
		// Forced mode
		__readRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,&v);
		v &= ~DRIVER_BME280_REG_CTRLMEAS_MODE_MASK;
		v |= DRIVER_BME280_REG_CTRLMEAS_MODE_FORCED;
		__writeRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,v);

		uint8_t max = 100;
		do {
			itsdk_delayMs(1);
			max--;
			__readRegister(DRIVER_BME280_REG_CTRLMEAS_ADR,&v);
		} while ( (v & DRIVER_BME280_REG_STATUS_MEASURING_MASK)>0 && max > 0 );
		if ( max == 0 ) return BME280_FAILED;
		break;
	case BME280_MODE_INDOOR_NAVIGATION:
		// Normal Mode
		break;
	}




}







#endif // ITSDK_DRIVERS_BME280



