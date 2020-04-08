/* ==========================================================
 * accel.h - Common accelerometer interface
 *           contains the link with the drivers for a
 *           common access to standard accelerometer features
 * ----------------------------------------------------------
 * 
 *  Created on: 5 avr. 2020
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2020
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
#include <it_sdk/itsdk.h>

#ifndef INC_IT_SDK_ACCEL_ACCEL_H_
#define INC_IT_SDK_ACCEL_ACCEL_H_


// Error have to be a bit field.
typedef enum {
	ACCEL_SUCCESS=0,

	ACCEL_FAILED=0x80
} itsdk_accel_ret_e;


// Precision of the data returned by the driver. The abstraction stack
// will select the best from what the hardware is able to provide
typedef enum {
	ACCEL_WISH_PRECSION_8B	= 0,
	ACCEL_WISH_PRECSION_10B	= 1,
	ACCEL_WISH_PRECSION_12B	= 2,
	ACCEL_WISH_PRECSION_16B	= 3
} itsdk_accel_precision_e;

// Frequency of the data returned by the driver. The abstraction stack
// will select the best from what the hardware is able to provide
typedef enum {
	ACCEL_WISH_FREQUENCY_PWRDWN	= 0,
	ACCEL_WISH_FREQUENCY_1HZ	= 1,
	ACCEL_WISH_FREQUENCY_10HZ	= 2,
	ACCEL_WISH_FREQUENCY_25HZ	= 3,
	ACCEL_WISH_FREQUENCY_50HZ	= 4,
	ACCEL_WISH_FREQUENCY_100HZ	= 5,
	ACCEL_WISH_FREQUENCY_500HZ	= 6,
	ACCEL_WISH_FREQUENCY_1000HZ	= 7
} itsdk_accel_frequency_e;

typedef enum {
	ACCEL_WISH_SCALE_FACTOR_2G	= 0,
	ACCEL_WISH_SCALE_FACTOR_4G	= 1,
	ACCEL_WISH_SCALE_FACTOR_8G	= 2,
	ACCEL_WISH_SCALE_FACTOR_16G	= 3
} itsdk_accel_scale_e;

typedef enum {
	ACCEL_TRIGGER_ON_NONE			= 0x0000000,
	ACCEL_TRIGGER_ON_X_HIGH			= 0x0000001,
	ACCEL_TRIGGER_ON_X_LOW			= 0x0000002,
	ACCEL_TRIGGER_ON_Y_HIGH			= 0x0000004,
	ACCEL_TRIGGER_ON_Y_LOW			= 0x0000008,
	ACCEL_TRIGGER_ON_Z_HIGH			= 0x0000010,
	ACCEL_TRIGGER_ON_Z_LOW			= 0x0000020,
	ACCEL_TRIGGER_ON_XYZ_HIGH		= 0x0000015,
	ACCEL_TRIGGER_ON_XYZ_LOW 		= 0x000002A,
	ACCEL_TRIGGER_ON_X				= 0x0000003,
	ACCEL_TRIGGER_ON_Y				= 0x000000C,
	ACCEL_TRIGGER_ON_Z				= 0x0000030,
	ACCEL_TRIGGER_ON_XYZ			= 0x000003F,

	ACCEL_TRIGGER_ON_CLICK_X_P		= 0x0000100,		// P for positive force (normal side)
	ACCEL_TRIGGER_ON_DBLCLICK_X_P	= 0x0000200,
	ACCEL_TRIGGER_ON_CLICK_Y_P		= 0x0000400,
	ACCEL_TRIGGER_ON_DBLCLICK_Y_P	= 0x0000800,
	ACCEL_TRIGGER_ON_CLICK_Z_P		= 0x0001000,
	ACCEL_TRIGGER_ON_DBLCLICK_Z_P	= 0x0002000,
	ACCEL_TRIGGER_ON_CLICK_X_N		= 0x0004000,		// N for positive force (opposite side)
	ACCEL_TRIGGER_ON_DBLCLICK_X_N	= 0x0008000,
	ACCEL_TRIGGER_ON_CLICK_Y_N		= 0x0010000,
	ACCEL_TRIGGER_ON_DBLCLICK_Y_N	= 0x0020800,
	ACCEL_TRIGGER_ON_CLICK_Z_N		= 0x0040000,
	ACCEL_TRIGGER_ON_DBLCLICK_Z_N	= 0x0080000,
	ACCEL_TRIGGER_ON_CLICK_XYZ		= 0x0055500,
	ACCEL_TRIGGER_ON_DBLCLICK_XYZ	= 0x00AAA00,
	ACCEL_TRIGGER_ON_ANYCLICK		= 0x00FFF00,

	ACCEL_TRIGGER_ON_POS_TOP		= 0x0100000,		// Position detection, callback on position changed
	ACCEL_TRIGGER_ON_POS_BOTTOM		= 0x0200000,
	ACCEL_TRIGGER_ON_POS_LEFT		= 0x0400000,
	ACCEL_TRIGGER_ON_POS_RIGHT		= 0x0800000,
	ACCEL_TRIGGER_ON_POS_FRONT		= 0x1000000,
	ACCEL_TRIGGER_ON_POS_BACK		= 0x2000000,
	ACCEL_TRIGGER_ON_ANYPOS			= 0x3F00000,
} itsdk_accel_trigger_e;

// =======================================================================================


itsdk_accel_ret_e accel_initPowerDown();
itsdk_accel_ret_e accel_configMovementDetection(
		uint16_t 					mg,			// Force of the Movement for being detected
		uint16_t					tolerence,	// +/- Acceptable force in 10xmg. 0 for any (absolute value)
		uint16_t 					ms,			// Duration of the Movement for being detected
		itsdk_bool_e				hpf, 		// enable High pass filter
		itsdk_accel_trigger_e		triggers 	// List of triggers
);
void accel_process(void);


// Internal functions
void __accel_triggerCallback(itsdk_accel_trigger_e triggers);


//void accel_configMovementDetection(uint16_t mg, uint8_t ms, /* axis list, sampling freq, scale */);
//void accel_addTriggerOnMovement(void(* callback)(uint8_t id), uint8_t id);
//void accel_removeTriggerOnMovement(void(* callback)(uint8_t id),uint8_t id);

//void accel_configCaptureData(/* freq, scale, highpass, freq d'appel du traitement */ );
//void accel_init( /* ... switch to low power, standard config, stop */ );
//void accel_proces( /* ... process accel irq ... */void(* callback)(/*accelerodata[32]*/, uint8_t count, bool overun) );

// avec contenu de la FiFo */


#endif /* INC_IT_SDK_ACCEL_ACCEL_H_ */
