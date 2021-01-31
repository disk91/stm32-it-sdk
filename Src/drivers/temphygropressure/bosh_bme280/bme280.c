/* ==========================================================
 * bme280.c - Bosh BME280 Temp / Hygro / Pressure I2C-SPI sensor
 *            Bosh BMP280 Temp / Pressure I2C-SPI sensor
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 f�vr. 2019
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
#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if defined ITSDK_DRIVERS_BME280 && ITSDK_DRIVERS_BME280 == __ENABLE
#include <drivers/temphygropressure/bosh_bme280/bme280.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>


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
	uint8_t v,w;
	if (  __readRegister(DRIVER_BME280_REG_ID_ADR,&v) != __I2C_OK ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_BME280_NOTFOUND,0);
		return BME280_NOTFOUND;
	}
	if ( v != DRIVER_BME280_REG_ID_VALUE && v != DRIVER_BMP280_REG_ID_VALUE ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_BME280_NOTFOUND,0);
		return BME280_NOTFOUND;
	}
	__bme280_config.type = ( v == DRIVER_BME280_REG_ID_VALUE )?BME280:BMP280;
	// Configure the device according the selected mode
	switch (mode) {
	default:
	case BME280_MODE_WEATHER_MONITORING:
		// Forced mode
		// Oversampling 1x for all
		if ( __bme280_config.type == BME280 ) {
			__readRegister(DRIVER_BME280_REG_CTRLHUM_ADR,&v);		// Humidity oversampling  x1
			v &= ~DRIVER_BME280_REG_CTRLHUM_OSRD_MASK;
			v |= DRIVER_BME280_REG_CTRLHUM_OSRD_X1;
			__writeRegister(DRIVER_BME280_REG_CTRLHUM_ADR,v);
		}

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
		if ( __bme280_config.type == BME280 ) {
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
		} else return BME280_FAILED;
		break;

	case BME280_MODE_INDOOR_NAVIGATION:
		// Oversampling 16x pressure 2x for temp & 1x humidity
		if ( __bme280_config.type == BME280 ) {
			__readRegister(DRIVER_BME280_REG_CTRLHUM_ADR,&v);		// Humidity oversampling  x1
			v &= ~DRIVER_BME280_REG_CTRLHUM_OSRD_MASK;
			v |= DRIVER_BME280_REG_CTRLHUM_OSRD_X1;
			__writeRegister(DRIVER_BME280_REG_CTRLHUM_ADR,v);
		}

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

	// Load the calibration values
	__readRegister(DRIVER_BME280_DIG_T1_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_T1_LSB,&w);
	__bme280_config.t1 = ((uint16_t)v << 8) + (uint16_t)w;

	__readRegister(DRIVER_BME280_DIG_T2_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_T2_LSB,&w);
	__bme280_config.t2 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_T3_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_T3_LSB,&w);
	__bme280_config.t3 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_P1_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P1_LSB,&w);
	__bme280_config.p1 = ((uint16_t)v << 8) + (uint16_t)w;

	__readRegister(DRIVER_BME280_DIG_P2_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P2_LSB,&w);
	__bme280_config.p2 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_P3_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P3_LSB,&w);
	__bme280_config.p3 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_P4_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P4_LSB,&w);
	__bme280_config.p4 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_P5_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P5_LSB,&w);
	__bme280_config.p5 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_P6_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P6_LSB,&w);
	__bme280_config.p6 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_P7_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P7_LSB,&w);
	__bme280_config.p7 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_P8_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P8_LSB,&w);
	__bme280_config.p8 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_P9_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_P9_LSB,&w);
	__bme280_config.p9 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_H1,&v);
	__bme280_config.h1 = v;

	__readRegister(DRIVER_BME280_DIG_H2_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_H2_LSB,&w);
	__bme280_config.h2 = ((int16_t)v << 8) + (int16_t)w;

	__readRegister(DRIVER_BME280_DIG_H3,&v);
	__bme280_config.h3 = v;

	__readRegister(DRIVER_BME280_DIG_H4_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_H4_LSB,&w);
	__bme280_config.h4 = ((int16_t)v << 4) + (int16_t)(w & DRIVER_BME280_DIG_H4_LSB_MASK);

	__readRegister(DRIVER_BME280_DIG_H5_MSB,&v);
	__readRegister(DRIVER_BME280_DIG_H5_LSB,&w);
	__bme280_config.h5 = ((int16_t)v << 4) + (int16_t)((w & DRIVER_BME280_DIG_H5_LSB_MASK) >> 4);

	__readRegister(DRIVER_BME280_DIG_H6,&v);
	__bme280_config.h6 = (int8_t)v;

	return (__bme280_config.type == BME280)?BME280_SUCCESS:BMP280_SUCCESS;
}


/**
 * Compute the temperature from the BM280 adc value
 * The unit is Centi Celsius 5123 = 51,23�C
 */
static int32_t __compensateTemp(int32_t adc) {
	int32_t var1,var2,t;
	var1 = ((((adc>>3) - ((int32_t)(__bme280_config.t1 << 1))) * (int32_t)__bme280_config.t2)) >> 11;
	var2 = (((((adc>>4) - ((int32_t)__bme280_config.t1)) * ((adc>>4) - ((int32_t)__bme280_config.t1))) >> 12) * ((int32_t)__bme280_config.t3)) >> 14;
	__bme280_config.t_fine = var1 + var2;
	t = ( __bme280_config.t_fine * 5 + 128 ) >> 8;
	return t;
}

/**
 * Compute the pressure from the BM280 adc value
 * The unit is Pa with 24 integer bits and 8 fractional bits
 * 24674867 => 24674867 / 256 = 96286.2 Pa => 963.862 hPa
 *
 * This need temperature to be compensate first due to reuse of some intermediate values
 */
static uint32_t __compensatePressure(int32_t adc) {
	int64_t var1, var2, p;
	var1 = ((int64_t)__bme280_config.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)__bme280_config.p6;
	var2 = var2 + ((var1*(int64_t)__bme280_config.p5)<<17);
	var2 = var2 + (((int64_t)__bme280_config.p4)<<35);
	var1 = ((var1 * var1 * (int64_t)__bme280_config.p3)>>8) + ((var1 * (int64_t)__bme280_config.p2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)__bme280_config.p1)>>33;
	if (var1 == 0) return 0;
	p = 1048576-adc;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)__bme280_config.p9) * (p>>13) * (p>>13)) >> 25;
	var2 =(((int64_t)__bme280_config.p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)__bme280_config.p7)<<4);
	return (uint32_t)p;
}

/**
 * Compute the humidity from the BM280 adc value
 * The unit is %RH with 22 integer bits and 10 fractional bits
 * 47445 => 47445/1024 = 46.333 %RH
 * This need temperature to be compensate first due to reuse of some intermediate values
 */

static uint32_t __compensateHumidity(int32_t adc){
	int32_t	v;
	v = (__bme280_config.t_fine - ((int32_t)76800));
	v = (((((adc << 14) - (((int32_t)__bme280_config.h4) << 20) - (((int32_t)__bme280_config.h5) * v))
		+ ((int32_t)16384)) >> 15) * (((((((v * ((int32_t)__bme280_config.h6)) >> 10) * (((v * ((int32_t)__bme280_config.h3)) >> 11)
		+ ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)__bme280_config.h2) + 8192) >> 14));
	v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)__bme280_config.h1)) >> 4));
	v = (v < 0 ? 0 : v);
	v = (v > 419430400? 419430400: v);
	return (uint32_t)(v >> 12);
}


/**
 * Depends on mode, get the last sensor values or request a new value and get it
 * Get the sensors value.
 * Temperature is in moC
 * Humidity is in m%RH
 * Pressure is in Pa
 */
drivers_bme280_ret_e drivers_bme280_getSensors(
		int32_t  * temperature,			// Temp in moC
		uint32_t * pressure,			// Pressure un Pa
		uint32_t * humidity				// Humidity in m%RH
) {
	uint8_t v;
	// Verify I2C
	if (  __readRegister(DRIVER_BME280_REG_ID_ADR,&v) != __I2C_OK ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_BME280_I2CERROR,0);
		return BME280_FAILED;
	}

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
			__readRegister(DRIVER_BME280_REG_STATUS_ADR,&v);
		} while ( (v & DRIVER_BME280_REG_STATUS_MEASURING_MASK)>0 && max > 0 );
		if ( max == 0 ) return BME280_FAILED;
		break;
	case BME280_MODE_INDOOR_NAVIGATION:
		// Normal Mode
		break;
	}

	// Get the values
	int32_t adc;
	__readRegister(DRIVER_BME280_REG_TEMP_MSB_ADR,&v);
	adc = ((int32_t)v) << 12;
	__readRegister(DRIVER_BME280_REG_TEMP_LSB_ADR,&v);
	adc = adc | ((((int32_t)v) << 4) & 0x00FF0);
	__readRegister(DRIVER_BME280_REG_TEMP_XLSB_ADR,&v);
	adc = adc | (((int32_t)v) & 0xF);
	*temperature = 10*__compensateTemp(adc);

	if ( __bme280_config.type == BME280 ) {
		__readRegister(DRIVER_BME280_REG_HUM_MSB_ADR,&v);
		adc = ((int32_t)v) << 8;
		__readRegister(DRIVER_BME280_REG_HUM_LSB_ADR,&v);
		adc = adc | ((((int32_t)v)) & 0xFF);
		*humidity = (1000*__compensateHumidity(adc))/1024;
	} else {
		*humidity = 0;
	}

	__readRegister(DRIVER_BME280_REG_PRESS_MSB_ADR,&v);
	adc = ((int32_t)v) << 12;
	__readRegister(DRIVER_BME280_REG_PRESS_LSB_ADR,&v);
	adc = adc | ((((int32_t)v) << 4) & 0x00FF0);
	__readRegister(DRIVER_BME280_REG_PRESS_XLSB_ADR,&v);
	adc = adc | (((int32_t)v) & 0xF);
	*pressure = __compensatePressure(adc)/256;

	return BME280_SUCCESS;
}

#endif // ITSDK_DRIVERS_BME280

#endif // ITSDK_WITH_DRIVERS


