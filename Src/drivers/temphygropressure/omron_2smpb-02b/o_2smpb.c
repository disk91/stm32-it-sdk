/* ==========================================================
 * o_2smpb.c - Driver for Omron Temp & Pressure I2C driver
 *           - Support - 2SMPB-02B
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 31 may 2020
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2020 Disk91
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
 * Supports the I2C implementation of the Omron sensor
 * Omron sensor is pin to pin compatible with BME280
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if defined ITSDK_DRIVERS_O2SMPB && ITSDK_DRIVERS_O2SMPB == __ENABLE
#include <drivers/temphygropressure/omron_2smpb-02b/o_2smpb.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>

#include <math.h>

#if ITSDK_DRIVERS_O2SMPB_VERSION != 0x2B
#error "UNSUPPORTED 2SMPB-XX VESRION"
#endif

static drivers_o2smpb_conf_t __o2smpb_config = {0};

static _I2C_Status __readRegister(uint8_t addr, uint8_t * value) {
	return i2c_read8BRegister(
			&ITSDK_DRIVERS_02SMPB_I2C,
			ITSDK_DRIVERS_O2SMPB_ADDRESS,
			addr,
			value,
			1
		   );
}

static _I2C_Status __writeRegister(uint8_t addr, uint8_t value) {
	return i2c_write8BRegister(
			&ITSDK_DRIVERS_02SMPB_I2C,
			ITSDK_DRIVERS_O2SMPB_ADDRESS,
			addr,
			value,
			1
		   );
}

/**
 * Setup the driver for 1 short measurement with 4 read averaging
 */
drivers_o2smpb_ret_e drivers_o2smpb_setup(drivers_o2smpb_mode_e mode) {
	uint8_t v;

	__o2smpb_config.mode = O2SMPB_MODE_UNKNONW;

	// Check device presence
	if (  __readRegister(DRIVER_O2SMPB_CHIPID_I2CADR,&v) != __I2C_OK ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_O2SMPB_NOTFOUND,0);
		return O2SMPB_NOTFOUND;
	}

	if ( v != DRIVER_O2SMPB_CHIPID_VALUE ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_O2SMPB_NOTFOUND,0);
		return O2SMPB_NOTFOUND;
	}

	// Reset chip
	__writeRegister(DRIVER_O2SMPB_RESET_I2CADR,DRIVER_O2SMPB_RESET_CMD);
	itsdk_delayMs(10);

	// Read calibration
	__readRegister(DRIVER_O2SMPB_COEPTAT32_I2CADR,&v);
	__o2smpb_config.aa = v;
	__readRegister(DRIVER_O2SMPB_COEPTAT31_I2CADR,&v);
	__o2smpb_config.aa |= ((uint16_t)v) << 8;
	__o2smpb_config.aa = -(__o2smpb_config.aa & ((uint16_t)1 << 15)) + (__o2smpb_config.aa & ~((uint16_t)1<<15));

	__readRegister(DRIVER_O2SMPB_COEPTAT22_I2CADR,&v);
	__o2smpb_config.ba = v;
	__readRegister(DRIVER_O2SMPB_COEPTAT21_I2CADR,&v);
	__o2smpb_config.ba |= ((uint16_t)v) << 8;
	__o2smpb_config.ba = -(__o2smpb_config.ba & ((uint16_t)1 << 15)) + (__o2smpb_config.ba & ~((uint16_t)1<<15));

	__readRegister(DRIVER_O2SMPB_COEPTAT13_I2CADR,&v);
	__o2smpb_config.ca = v;
	__readRegister(DRIVER_O2SMPB_COEPTAT12_I2CADR,&v);
	__o2smpb_config.ca |= ((uint32_t)v) << 8;
	__readRegister(DRIVER_O2SMPB_COEPTAT11_I2CADR,&v);
	__o2smpb_config.ca |= ((uint32_t)v) << 16;
	__o2smpb_config.ca = -(__o2smpb_config.ca & ((uint16_t)1 << 23)) + (__o2smpb_config.ca & ~((uint16_t)1<<23));

	__readRegister(DRIVER_O2SMPB_COETEMP32_I2CADR,&v);
	__o2smpb_config.at = v;
	__readRegister(DRIVER_O2SMPB_COETEMP31_I2CADR,&v);
	__o2smpb_config.at |= ((uint16_t)v) << 8;
	__o2smpb_config.at = -(__o2smpb_config.at & ((uint16_t)1 << 15)) + (__o2smpb_config.at & ~((uint16_t)1<<15));

	__readRegister(DRIVER_O2SMPB_COETEMP22_I2CADR,&v);
	__o2smpb_config.bt = v;
	__readRegister(DRIVER_O2SMPB_COETEMP21_I2CADR,&v);
	__o2smpb_config.bt |= ((uint16_t)v) << 8;
	__o2smpb_config.bt = -(__o2smpb_config.bt & ((uint16_t)1 << 15)) + (__o2smpb_config.bt & ~((uint16_t)1<<15));

	__readRegister(DRIVER_O2SMPB_COETEMP12_I2CADR,&v);
	__o2smpb_config.ct = v;
	__readRegister(DRIVER_O2SMPB_COETEMP11_I2CADR,&v);
	__o2smpb_config.ct |= ((uint16_t)v) << 8;
	__o2smpb_config.ct = -(__o2smpb_config.ct & ((uint16_t)1 << 15)) + (__o2smpb_config.ct & ~((uint16_t)1<<15));

	__readRegister(DRIVER_O2SMPB_COEPR32_I2CARD,&v);
	__o2smpb_config.ap = v;
	__readRegister(DRIVER_O2SMPB_COEPR31_I2CARD,&v);
	__o2smpb_config.ap |= ((uint16_t)v) << 8;
	__o2smpb_config.ap = -(__o2smpb_config.ap & ((uint16_t)1 << 15)) + (__o2smpb_config.ap & ~((uint16_t)1<<15));

	__readRegister(DRIVER_O2SMPB_COEPR22_I2CARD,&v);
	__o2smpb_config.bp = v;
	__readRegister(DRIVER_O2SMPB_COEPR21_I2CARD,&v);
	__o2smpb_config.bp |= ((uint16_t)v) << 8;
	__o2smpb_config.bp = -(__o2smpb_config.bp & ((uint16_t)1 << 15)) + (__o2smpb_config.bp & ~((uint16_t)1<<15));

	__readRegister(DRIVER_O2SMPB_COEPR13_I2CARD,&v);
	__o2smpb_config.cp = v;
	__readRegister(DRIVER_O2SMPB_COEPR12_I2CARD,&v);
	__o2smpb_config.cp |= ((uint32_t)v) << 8;
	__readRegister(DRIVER_O2SMPB_COEPR11_I2CARD,&v);
	__o2smpb_config.cp |= ((uint32_t)v) << 16;
	__o2smpb_config.cp = -(__o2smpb_config.cp & ((uint16_t)1 << 23)) + (__o2smpb_config.cp & ~((uint16_t)1<<23));

	switch (mode) {

		default:
		case O2SMPB_MODE_ONESHOT_AVG4:
			if ( __writeRegister(
					DRIVER_O2SMPB_CTRLMEAS_I2CADR,
					DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_4 | DRIVER_O2SMPB_CTRLMEAS_PRESAVG_4 | DRIVER_O2SMPB_CTRLMEAS_POWERMD_SLEEP
				 ) != __I2C_OK
			) {
				return O2SMPB_FAILED;
			}
			break;
	}
	__o2smpb_config.mode = mode;
	return O2SMPB_SUCCESS;
}


/**
 * Read the Raw temperature
 */
static void __readRawTemp(int32_t * t) {
	uint8_t v;
	__readRegister(DRIVER_O2SMPB_TEMP_TXD0_I2CADR,&v);
	*t = v;
	__readRegister(DRIVER_O2SMPB_TEMP_TXD1_I2CADR,&v);
	*t |= ((uint32_t)v) << 8;
	__readRegister(DRIVER_O2SMPB_TEMP_TXD2_I2CADR,&v);
	*t |= ((uint32_t)v) << 16;
	*t -= 1<<23;
}

/**
 * Read the Raw pressure
 */
static void __readRawPressure(int32_t * p) {
	uint8_t v;
	__readRegister(DRIVER_O2SMPB_PRES_TXD0_I2CADR,&v);
	*p = v;
	__readRegister(DRIVER_O2SMPB_PRES_TXD1_I2CADR,&v);
	*p |= ((uint32_t)v) << 8;
	__readRegister(DRIVER_O2SMPB_PRES_TXD2_I2CADR,&v);
	*p |= ((uint32_t)v) << 16;
	*p -= 1<<23;
}

/**
 * Depends on mode, get the last sensor values or request a new value and get it
 * Get the sensors value.
 * Temperature is in moC
 * Pressure is in Pa
 */
drivers_o2smpb_ret_e drivers_o2smpb_getSensors(
		int32_t  * temperature,			// Temp in m oC
		uint32_t * pressure  			// Pressure in Pa
) {

	uint8_t v;
	// Verify I2C
	if (  __readRegister(DRIVER_O2SMPB_CHIPID_I2CADR,&v) != __I2C_OK ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_O2SMPB_NOTFOUND,0);
		return O2SMPB_FAILED;
	}

	// Fire measure request
	switch (__o2smpb_config.mode) {
	case O2SMPB_MODE_ONESHOT_AVG4:
		if ( __writeRegister(
				DRIVER_O2SMPB_CTRLMEAS_I2CADR,
				DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_4 | DRIVER_O2SMPB_CTRLMEAS_PRESAVG_4 | DRIVER_O2SMPB_CTRLMEAS_POWERMD_FORCED
			 ) != __I2C_OK
		) {
			return O2SMPB_FAILED;
		}
		break;

	default:
		return O2SMPB_FAILED;
	}

	// wait for measure done
	uint8_t loops = 0;
	do {
		itsdk_delayMs(1);
		__readRegister(DRIVER_O2SMPB_DEVSTAT_I2CADR,&v);
		loops++;
	} while ( ((v & DRIVER_O2SMPB_DEVSTAT_MEASURE_MSK) != DRIVER_O2SMPB_DEVSTAT_MEASURE_FINISHED) && loops < 20 );
	if (loops == 20) return O2SMPB_FAILED;

	// read it
	int32_t t;
	int32_t p;
	__readRawTemp(&t);
	__readRawPressure(&p);

	// Temperature conversion
	double d_aa = ((4.2E-4)*__o2smpb_config.aa)/32768.0;
	double d_ba = -160+(8*__o2smpb_config.ba)/32768.0;
	double d_ca = __o2smpb_config.ca;
	double d_tr = (-d_ba - sqrt((d_ba*d_ba) -4*d_aa*(d_ca - t))) / ( 2*d_aa );

	*temperature = (int32_t)((1000*d_tr)/256);

	// Temperature compensated pressure
	double d_bp = 3.0E1 + ((1.0E1)*__o2smpb_config.bp)/32768.0;
	double d_ap = ((3.0E-5)*__o2smpb_config.ap)/32768.0;
	double d_cp = __o2smpb_config.cp;
	double d_pl = (-d_bp + sqrt((d_bp*d_bp)-4*d_ap*(d_cp - p))) / ( 2 * d_ap );

	double d_at = ((8.0E-11)*__o2smpb_config.at)/32768.0;
	double d_bt = -6.6E-6 + ((1.6E-6)*__o2smpb_config.bt)/32768.0;
	double d_ct = 4.0E-2 + ((8.5E-3)*__o2smpb_config.ct)/32768.0;
	double d_po = d_pl / (d_at*d_tr*d_tr + d_bt*d_tr + (d_ct + 1));

	*pressure = (uint32_t)d_po;

	// Back to sleep mode
	__writeRegister(DRIVER_O2SMPB_CTRLMEAS_I2CADR,DRIVER_O2SMPB_CTRLMEAS_POWERMD_SLEEP);

	return O2SMPB_SUCCESS;
}




#endif
#endif
