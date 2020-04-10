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
 * EventQueue for asynchronous processing
 */
itsdk_accel_trigger_e 			__triggerQueue[ACCEL_TRIGGER_QUEUE_SIZE];
uint8_t				  			__triggerQueueRd;
uint8_t				 			__triggerQueueWr;
itsdk_accel_eventHandler_t * 	__accel_eventQueue;


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

	__triggerQueueRd = 0;
	__triggerQueueWr = 0;
	__accel_eventQueue = NULL;
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

	__accel_asyncTriggerProcess();
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

/** *********************************************************************
 *  Manage a Queue of event to be proceeded by application
 */

itsdk_accel_ret_e accel_addTriggerCallBack(
		itsdk_accel_eventHandler_t * handler
) {

	handler->next = NULL;
	if ( __accel_eventQueue == NULL ) {
		__accel_eventQueue = handler;
	} else {
		itsdk_accel_eventHandler_t * c = __accel_eventQueue;
		while ( c->next != NULL ) c = c->next;
		c->next = handler;
	}
	return ACCEL_SUCCESS;
}

itsdk_accel_ret_e accel_delTriggerCallBack(
		itsdk_accel_eventHandler_t * handler
) {

	itsdk_accel_eventHandler_t * c = __accel_eventQueue;
	if ( c == handler ) {
		// head
		__accel_eventQueue = c->next;
	} else {
		while ( c != NULL && c->next != handler ) c = c->next;
		if ( c!= NULL ) {
		   c->next = (c->next)->next;
		} else return ACCEL_FAILED;
	}
	return ACCEL_SUCCESS;

}




/**
 * This callback is fired by the interrupt process.
 * To reduce the interrupt processing time, the trigger will be
 * queued and then process in the accel_process loop.
 * Circular buffer power of 2 sized. Old values are cleared by new
 * values
 */
void __accel_addToQueue(itsdk_accel_trigger_e triggers) {
	itsdk_enterCriticalSection();

	__triggerQueue[__triggerQueueWr] = triggers;
	__triggerQueueWr = (__triggerQueueWr + 1) & (ACCEL_TRIGGER_QUEUE_SIZE-1);
	if ( __triggerQueueWr == __triggerQueueRd ) {
		// Buffer full
		__triggerQueueRd = (__triggerQueueRd + 1) & (ACCEL_TRIGGER_QUEUE_SIZE-1);
	}

	itsdk_leaveCriticalSection();
}

itsdk_accel_trigger_e __accel_getFromQueue(void) {
	itsdk_accel_trigger_e t;

	itsdk_enterCriticalSection();
	if ( __triggerQueueWr != __triggerQueueRd ) {
		 t = __triggerQueue[__triggerQueueRd];
		__triggerQueueRd = (__triggerQueueRd + 1) & (ACCEL_TRIGGER_QUEUE_SIZE-1);
	} else {
		t= ACCEL_TRIGGER_ON_NONE;
	}
	itsdk_leaveCriticalSection();
	return t;
}


void __accel_triggerCallback(itsdk_accel_trigger_e triggers) {
	__accel_addToQueue(triggers);
}


// -----
// Process the Queue of event coming from the Interrupt
// call the list of callback registered by the application to react on events
void __accel_asyncTriggerProcess() {

	itsdk_accel_trigger_e triggers = __accel_getFromQueue();
	while ( triggers != ACCEL_TRIGGER_ON_NONE ) {

		itsdk_accel_eventHandler_t * c = __accel_eventQueue;
		while ( c != NULL ) {
			if ( (triggers & c->triggerMask) > 0 ) {
				if ( c->callback != NULL ) c->callback(triggers & c->triggerMask);
			}
			c = c->next;
		}
		triggers = __accel_getFromQueue();
	}

}





#endif
