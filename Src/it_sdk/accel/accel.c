/* ==========================================================
 * accel.c - 
 * ----------------------------------------------------------
 * 
 *  Created on: 6 avr. 2020
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2020
 *
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
 * ==========================================================
 */

#include <it_sdk/accel/accel.h>
#include <it_sdk/config.h>
#include <it_sdk/logger/logger.h>

#if ITSDK_WITH_DRIVERS == __ENABLE
#include <it_sdk/configDrivers.h>


/**
 * Init the accelerometer driver switching it in powerdown mode
 * Settings are set with the default value we want. No big interest in it at this point.
 */
itsdk_accel_ret_e accel_initPowerDown() {
	itsdk_accel_ret_e ret = ACCEL_SUCCESS;

   #if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
	if ( lis2dh_setup(
			LIS2DH_RESOLUTION_MODE_8B,
			LIS2DH_FREQUENCY_POWERDOWN,
			LIS2DH_SCALE_FACTOR_2G
		) != LIS2DH_SUCCESS ) {
		ret = ACCEL_FAILED;
	}
   #endif

	return ret;
}

/**
 * This function must be called by the main loop to manage the
 * accelerometer on regular basis.
 */
void accel_process(void) {
	#if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
	  lis2dh12_process();
	#endif
}


/**
 * Set the accelerometer driver to be ready to trigger on movement detection
 * The user need to register one or multiples trigger action on movement detection
 * Sampling rate, scale are automatically determine from the given elements.
 */
itsdk_accel_ret_e accel_configMovementDetection(
		uint16_t 					mg,			// Force of the Movement for being detected
		uint16_t					tolerence,	// +/- Acceptable force in 10xmg. 0 for any (absolute value)
		uint16_t 					ms,			// Duration of the Movement for being detected
		itsdk_bool_e				hpf, 		// enable High pass filter
		itsdk_accel_trigger_e		triggers 	// List of triggers
) {

	itsdk_accel_precision_e precision;
	itsdk_accel_frequency_e frequency;
	itsdk_accel_scale_e scale;

	// search for the right scale factor with
	// needed margin
	if ( mg < 1500 ) {							//		Precisions @ 8b |  @ 10b  | @ 12b  |  @16b
		scale = ACCEL_WISH_SCALE_FACTOR_2G;		//           7.8mg      |  1.95mg | 0.48mg | 0.03mg
		if ( tolerence == 0 || tolerence >= 100 ) {
			precision = ACCEL_WISH_PRECSION_8B;
		} else if ( tolerence >= 20 ) {
			precision = ACCEL_WISH_PRECSION_10B;
		} else if ( tolerence >= 5 ) {
			precision = ACCEL_WISH_PRECSION_12B;
		} else {
			precision = ACCEL_WISH_PRECSION_16B;
		}
	} else if ( mg < 3000 ) {
		scale = ACCEL_WISH_SCALE_FACTOR_4G;		// 			15.6mg		|  3.9mg  | 0.96mg | 0.06mg
		if ( tolerence == 0 || tolerence >= 160 ) {
			precision = ACCEL_WISH_PRECSION_8B;
		} else if ( tolerence >= 40 ) {
			precision = ACCEL_WISH_PRECSION_10B;
		} else if ( tolerence >= 10 ) {
			precision = ACCEL_WISH_PRECSION_12B;
		} else {
			precision = ACCEL_WISH_PRECSION_16B;
		}
	} else if ( mg < 6000 ) {
		scale = ACCEL_WISH_SCALE_FACTOR_8G;		//			31.2mg		|  7.8mg  | 1.92mg | 0.12mg
		if ( tolerence == 0 || tolerence >= 320 ) {
			precision = ACCEL_WISH_PRECSION_8B;
		} else if ( tolerence >= 80 ) {
			precision = ACCEL_WISH_PRECSION_10B;
		} else if ( tolerence >= 20 ) {
			precision = ACCEL_WISH_PRECSION_12B;
		} else {
			precision = ACCEL_WISH_PRECSION_16B;
		}
	} else if ( mg < 12000 ) {
		scale = ACCEL_WISH_SCALE_FACTOR_16G;	//			62.4mg		|  15.6mg | 3.84mg | 0.24mg
		if ( tolerence == 0 || tolerence >= 650 ) {
			precision = ACCEL_WISH_PRECSION_8B;
		} else if ( tolerence >= 160 ) {
			precision = ACCEL_WISH_PRECSION_10B;
		} else if ( tolerence >= 40 ) {
			precision = ACCEL_WISH_PRECSION_12B;
		} else {
			precision = ACCEL_WISH_PRECSION_16B;
		}
	} else {
		return ACCEL_FAILED;
	}

	// Assuming we want to have two consecutive data capture
	// with the given force. We double rating the expected period
	if ( ms >= 500 ) {
		frequency = ACCEL_WISH_FREQUENCY_1HZ;
	} else if ( ms >= 50 ) {
		frequency = ACCEL_WISH_FREQUENCY_10HZ;
	} else if ( ms >= 20 ) {
		frequency = ACCEL_WISH_FREQUENCY_25HZ;
	} else if ( ms >= 10 ) {
		frequency = ACCEL_WISH_FREQUENCY_50HZ;
	} else if ( ms >= 5 ) {
		frequency = ACCEL_WISH_FREQUENCY_100HZ;
	} else if ( ms >= 1 ) {
		frequency = ACCEL_WISH_FREQUENCY_500HZ;
	} else {
		return ACCEL_FAILED;
	}


	#if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
	if ( lis2dh_setupBackgroundTiltDetection(
			mg,
			lis2dh12_convertScale(scale),
			lis2dh_converFrequency(frequency,precision),
			lis2dh_convertPrecision(precision),
			((hpf==BOOL_TRUE)?LIS2DH_HPF_MODE_AGGRESSIVE:LIS2DH_HPF_MODE_DISABLE),
			triggers,
			&__accel_triggerCallback
			) == LIS2DH_FAILED ) return ACCEL_FAILED;
	#endif





	return ACCEL_SUCCESS;
}


void __accel_triggerCallback(itsdk_accel_trigger_e triggers) {

	log_info("trigger status: 0x%08X\r\n",triggers);


}







#endif
