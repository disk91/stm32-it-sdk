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
#include <it_sdk/time/time.h>
#include <math.h>

#if ITSDK_WITH_DRIVERS == __ENABLE
#include <it_sdk/configDrivers.h>

// Do not compile is none of the supported drivers are enabled
#if ITSDK_DRIVERS_WITH_ACCEL_DRIVER == __ENABLE


/**
 * EventQueue for asynchronous processing
 */
itsdk_bool_e					__accel_setupDone = BOOL_FALSE;
itsdk_accel_trigger_e 			__triggerQueue[ACCEL_TRIGGER_QUEUE_SIZE];
uint8_t				  			__triggerQueueRd;
uint8_t				 			__triggerQueueWr;
itsdk_accel_eventHandler_t * 	__accel_eventQueue;
uint64_t						__accel_lastTriggerReportMs;
uint32_t						__accel_noMovementDuration;
itsdk_bool_e					__accel_noMovementReported;
itsdk_accel_data_t				__accel_dataBuffer[ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_SZ];
uint8_t							__accel_dataBufferRd;
uint8_t							__accel_dataBufferWr;
uint8_t							__accel_dataBufferSz;
itsdk_bool_e					__accel_dataOverrun;
itsdk_bool_e					__accel_running;
uint16_t						__accel_dataBlockSz;
uint16_t						__accel_dataBlockSzTransfered;
uint16_t						__accel_dataBlockNb;
itsdk_accel_dataFormat_e		__accel_dataFormat;
itsdk_accel_data_t			*	__accel_dataDestBuffer;
void (* __accel_dataCallback)(itsdk_accel_data_t * data, itsdk_accel_dataFormat_e format, uint8_t count, itsdk_bool_e overrun) ;

// ========================================================================================
// SETUP & LOOP
// ========================================================================================


/**
 * Init the accelerometer driver switching it in powerdown mode
 * Settings are set with the default value we want. No big interest in it at this point.
 */
itsdk_accel_ret_e accel_initPowerDown() {
	itsdk_accel_ret_e ret = ACCEL_SUCCESS;

   #if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
    ACCEL_LOG_INFO(("ACCEL - Init Lis2dh\r\n"));
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

	__accel_dataBufferRd = 0;
	__accel_dataBufferWr = 0;
	__accel_dataBufferSz = 0;
	__accel_dataOverrun = BOOL_FALSE;
	__accel_setupDone = BOOL_TRUE;

	__accel_dataBlockSz = 0;
	__accel_dataBlockSzTransfered = 0;
	__accel_dataBlockNb = 0;
    __accel_dataCallback = NULL;
    __accel_running = BOOL_FALSE;
	return ret;
}

/**
 * This function must be called by the main loop to manage the
 * accelerometer on regular basis.
 */
void accel_process_loop(void) {
	if ( __accel_setupDone == BOOL_FALSE ) return;
	if ( __accel_running == BOOL_FALSE ) return;
	#if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
	  lis2dh12_process();
	#endif

	__accel_asyncTriggerProcess();
	__accel_asyncMovementCaptureProcess();
}


// ========================================================================================
// EVENT DETECTION
// ========================================================================================


/**
 * Set the accelerometer driver to be ready to trigger on movement detection
 * The user need to register one or multiples trigger action on movement detection
 * Sampling rate, scale are automatically determine from the given elements.
 */
itsdk_accel_ret_e accel_configMovementDetection(
		uint16_t 					mg,			// Force of the Movement for being detected
		uint16_t					tolerence,	// +/- Acceptable force in 10xmg. 0 for any (absolute value)
		uint16_t 					ms,			// Duration of the Movement for being detected
		uint32_t					noMvtMs,	// Duration of no trigger notification before reporting No Movement - when NOMOVEMENT trigger requested
		itsdk_accel_hpf_e			hpf, 		// enable High pass filter
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
    ACCEL_LOG_INFO(("ACCEL - Start Lis2dh - Mvt det\r\n"));
	if ( lis2dh_setupBackgroundTiltDetection(
			mg,
			lis2dh12_convertScale(scale),
			lis2dh_converFrequency(frequency,precision),
			lis2dh_convertPrecision(precision),
			lis2dh_convertHPF(hpf),
			triggers,
			&__accel_triggerCallback
			) == LIS2DH_FAILED ) return ACCEL_FAILED;
	#endif


	if ( (triggers & ACCEL_TRIGGER_ON_NOMOVEMENT) > 0 ) {
		__accel_noMovementDuration = noMvtMs;
		__accel_lastTriggerReportMs = itsdk_time_get_ms();
	} else {
		__accel_noMovementDuration = 0;
	}
	__accel_noMovementReported = BOOL_FALSE;

	__accel_running = BOOL_TRUE;
	return ACCEL_SUCCESS;
}

/**
 * Stop the background event detection
 * The event handler can be removed from the event list of kept for later reusing
 */
itsdk_accel_ret_e accel_stopMovementDetection(itsdk_bool_e removeHandler) {

	#if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
      ACCEL_LOG_INFO(("ACCEL - Stop Lis2dh - Mvt det\r\n"));
	  if ( lis2dh_cancelBackgroundTiltDetection() == LIS2DH_FAILED ) return ACCEL_FAILED;
	#endif

	if (removeHandler == BOOL_TRUE ) {
		__accel_eventQueue = NULL;
	}
	// clear pending events
	__triggerQueueRd = 0;
	__triggerQueueWr = 0;

	__accel_running = BOOL_FALSE;
	return ACCEL_SUCCESS;
}

// ========================================================================================
// DATA CAPTURE
// ========================================================================================


static void __accel_movementCaptureCallback(
		itsdk_accel_data_t * data,
		itsdk_accel_dataFormat_e format,
		uint8_t count,
		itsdk_bool_e overrun
);

/**
 * Start data capture
 */
itsdk_accel_ret_e accel_startMovementCapture(
		itsdk_accel_scale_e 		scale,			// Movement Scale 2G, 4G...
		itsdk_accel_frequency_e 	frequency,		// Capture sampling rate
		itsdk_accel_precision_e 	precision,		// Raw Data size 8B, 10B...
		itsdk_accel_trigger_e		axis,			// List of activated axis
		itsdk_accel_hpf_e			hpf, 			// enable High pass filter
		uint16_t					dataBlock,		// Expected data to be returned by callback
		uint16_t					captureBlock,	// Number of blocks to capture - 0 for non stop
		itsdk_accel_dataFormat_e	format,			// Data format
		itsdk_accel_data_t		*	targetBuffer,	// Data buffer to be used to push the data
		void (* callback)(itsdk_accel_data_t * data, itsdk_accel_dataFormat_e format, uint8_t count, itsdk_bool_e overrun)
													// callback function the interrupt will call
){
  // ****
  // find the best watermark level for the underlaying driver
  uint8_t fifoWatemark;
  if (  dataBlock < __accel_getAccelMaxFifoWTM() ) fifoWatemark = dataBlock;
  else {
	// find the best divider
	uint8_t maxWTM = 0;
	uint8_t maxK = 0;
	for ( int k = 1 ; k < __accel_getAccelMaxFifoWTM() ; k++ ) {
		uint32_t p = (itsdk_pgcd(dataBlock,k));
		if (p > maxWTM) {
			maxWTM = p;
			maxK = k;
		}
	}
	fifoWatemark = maxK;
  }
  if ( fifoWatemark < __accel_getAccelMinFifoWTM() ) {
	 // for low frequency we can target request under the minimum, the underlaying driver will take
	 // the decision of accepting or refusing
	 if ( frequency == ACCEL_WISH_FREQUENCY_10HZ ) fifoWatemark = __accel_getAccelMinFifoWTM() / 2;
	 else if ( frequency > ACCEL_WISH_FREQUENCY_10HZ ) fifoWatemark = __accel_getAccelMinFifoWTM();
  }

  #if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
	//uint8_t	driverWatermark = (dataBlock > DRIVER_LIS2DH_DEFAULT_WATERMARK)?
    ACCEL_LOG_INFO(("ACCEL - Start Lis2dh - Data Aq\r\n"));
	if ( lis2dh_setupDataAquisition(
			lis2dh12_convertScale(scale),					// scale 2G/4G...
			lis2dh_converFrequency(frequency,precision), 	// capture frequency 1/10/25/50Hz..
			lis2dh_convertPrecision(precision),				// data precision 8B/10B/12B...
			axis,											// Select the axis
			lis2dh_convertHPF(hpf),							// High Pass Filter strategy
			fifoWatemark,									// Expected data to be returned by callback (watermark)
			__accel_movementCaptureCallback
															// callback function the interrupt will call
															// data format is always raw
	      )  == LIS2DH_FAILED ) return ACCEL_FAILED;

  #endif

  __accel_dataBlockSz = dataBlock;
  __accel_dataBlockSzTransfered = 0;
  __accel_dataBlockNb = captureBlock;
  __accel_dataFormat = format;
  __accel_dataDestBuffer = targetBuffer;
  __accel_dataCallback = callback;

  __accel_running = BOOL_TRUE;
  return ACCEL_SUCCESS;
}

itsdk_accel_ret_e accel_stopMovementCapture(void) {
   #if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
    ACCEL_LOG_INFO(("ACCEL - Stop Lis2dh - Data Aq\r\n"));
	if ( lis2dh_cancelDataAquisition() == LIS2DH_FAILED ) return ACCEL_FAILED;
   #endif
	__accel_running = BOOL_FALSE;
   return ACCEL_SUCCESS;
}

/** *********************************************************************
 *  Manage a Queue of data for asynchronous data process callback
 *  as we want to limit the duration of the computing process inside
 *  the interruption and allow interruption process during application
 *  computation. This create a latency related to the difference between
 *  the captured dataBlock at the underlaying driver and the application
 *  dataBlock.
 *  As an example, if app dataBlock is 10 and underlaying driver is 8 we
 *  will call the first callback after 16 sample captured (latency +6);
 *  the second time after 24 sample captured (latency +4) ...
 *  Choice of the underlaying dataBlock size is important in regard of the
 *  latency.
 *  Asynchronous process is executed on wakeup, basically right after the
 *  triggering interuption. Low latency on this point.
 *  Application dataBlock can be larger than the underlaying device FiFo size
 *  but it needs to be lower than ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_SZ
 *  *********************************************************************
 */

/**
 * Call by the IRQ to store movement data into the circular buffer
 */
static void __accel_movementCaptureCallback(
		itsdk_accel_data_t * data,
		itsdk_accel_dataFormat_e format,
		uint8_t count,
		itsdk_bool_e overrun
) {

	if ( overrun ) __accel_dataOverrun = BOOL_TRUE;
	for (int i = 0 ; i < count ; i++ ) {
		__accel_dataBuffer[__accel_dataBufferWr].x = data[i].x;
		__accel_dataBuffer[__accel_dataBufferWr].y = data[i].y;
		__accel_dataBuffer[__accel_dataBufferWr].z = data[i].z;
		__accel_dataBufferSz++;
		itsdk_enterCriticalSection();
		__accel_dataBufferWr = ( __accel_dataBufferWr + 1 ) & (ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_SZ-1);
		if ( __accel_dataBufferWr == __accel_dataBufferRd ) {
			__accel_dataBufferRd = ( __accel_dataBufferRd + 1 ) & (ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_SZ-1);
			__accel_dataOverrun = BOOL_TRUE;
			__accel_dataBufferSz = (ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_SZ-1);
		}
		itsdk_leaveCriticalSection();
	}
}

/**
 * Call by the project loop to asynchronously compute and transfer the data to the
 * application layer
 */
void __accel_asyncMovementCaptureProcess(void) {


	// We move the data as soon as we have the remaining data available or when we touched the watermark
	if (
			(__accel_dataBlockSz <= __accel_dataBlockSzTransfered + __accel_dataBufferSz)
		||  (__accel_dataBufferSz > ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_WTM )
	) {
		uint8_t toRead = (__accel_dataBlockSz <= __accel_dataBlockSzTransfered + __accel_dataBufferSz)?
				__accel_dataBlockSz - __accel_dataBlockSzTransfered :
				ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_WTM;

		// Buffer is ready for copy
		itsdk_enterCriticalSection();
		for (int i = 0 ; i < toRead ; i++ ) {
			__accel_dataDestBuffer[__accel_dataBlockSzTransfered].x = __accel_dataBuffer[__accel_dataBufferRd].x;
			__accel_dataDestBuffer[__accel_dataBlockSzTransfered].y = __accel_dataBuffer[__accel_dataBufferRd].y;
			__accel_dataDestBuffer[__accel_dataBlockSzTransfered].z = __accel_dataBuffer[__accel_dataBufferRd].z;
			__accel_dataBufferRd = ( __accel_dataBufferRd + 1 ) & (ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_SZ-1);
			__accel_dataBufferSz--;
			__accel_dataBlockSzTransfered++;
		}
		itsdk_leaveCriticalSection();

		// Is Block completed
		if ( __accel_dataBlockSzTransfered == __accel_dataBlockSz ) {
		    ACCEL_LOG_INFO(("ACCEL - Data - Blk Rdy\r\n"));
			// convert the data
			switch (__accel_dataFormat) {

			case ACCEL_DATAFORMAT_XYZ_MG: {
					for ( int i = 0 ; i < __accel_dataBlockSz ; i++ ) {
						accel_convertDataPointRaw2Mg(&__accel_dataDestBuffer[i]);
					}
				}
				break;
			case ACCEL_DATAFORMAT_FORCE_MG: {
					for ( int i = 0 ; i < __accel_dataBlockSz ; i++ ) {
						accel_convertDataPointRaw2Mg(&__accel_dataDestBuffer[i]);
						accel_convertDataPointMg2Force(&__accel_dataDestBuffer[i]);
					}
				}
				break;
		#if ITSDK_DRIVERS_ACCEL_WITH_ANGLE == __ENABLE
			case ACCEL_DATAFORMAT_ANGLES_RAD: {
					for ( int i = 0 ; i < __accel_dataBlockSz ; i++ ) {
						accel_convertDataPointRaw2Angle(&__accel_dataDestBuffer[i],BOOL_FALSE);
					}
			    }
				break;
			case ACCEL_DATAFORMAT_ANGLES_DEG: {
					for ( int i = 0 ; i < __accel_dataBlockSz ; i++ ) {
						accel_convertDataPointRaw2Angle(&__accel_dataDestBuffer[i],BOOL_TRUE);
					}
			    }
				break;
		#endif
			case ACCEL_DATAFORMAT_AVG_RAW: {
				    int32_t x=0 ,y=0 ,z=0;
					for ( int i = 0 ; i < __accel_dataBlockSz ; i++ ) {
						x+=__accel_dataDestBuffer[i].x;
						y+=__accel_dataDestBuffer[i].y;
						z+=__accel_dataDestBuffer[i].z;
					}
					__accel_dataDestBuffer[0].x = (int16_t)(x/__accel_dataBlockSz);
					__accel_dataDestBuffer[0].y = (int16_t)(y/__accel_dataBlockSz);
					__accel_dataDestBuffer[0].z = (int16_t)(z/__accel_dataBlockSz);
				}
				break;
			case ACCEL_DATAFORMAT_XYZ_RAW:
			default:
				break;
			}

			// report the data
			if ( __accel_dataCallback != NULL ) {
				__accel_dataCallback(__accel_dataDestBuffer,__accel_dataFormat,__accel_dataBlockSz,__accel_dataOverrun);
			}

			__accel_dataOverrun = BOOL_FALSE;	// reset until next transfer
			__accel_dataBlockSzTransfered = 0;
			if ( __accel_dataBlockNb > 0 ) {
				// We are not in the infinite capture mode
				__accel_dataBlockNb--;
				if ( __accel_dataBlockNb == 0 ) {
					// End of the requested capture
					accel_stopMovementCapture();
				}
			}
		}

	}

}




/** *********************************************************************
 *  Manage a Queue of event to be proceeded by application
 */

itsdk_accel_ret_e accel_addTriggerCallBack(
		itsdk_accel_eventHandler_t * handler
) {
    if ( accel_isTriggerCallBack(handler) == BOOL_TRUE ) return ACCEL_ALREADY_REGISTER;

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
 * Return BOOL_TRUE when the same callback has already been register
 * (same callback func and same trigger mask
 */
itsdk_bool_e accel_isTriggerCallBack(
		itsdk_accel_eventHandler_t * handler
){
	itsdk_accel_eventHandler_t * c = __accel_eventQueue;
	while ( c != NULL ) {
		if ( c->callback == handler->callback && c->triggerMask == handler->triggerMask ) return BOOL_TRUE;
		c = c->next;
	}
	return BOOL_FALSE;
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
void __accel_asyncTriggerProcess(void) {

	itsdk_accel_trigger_e triggers = __accel_getFromQueue();

	if ( triggers == ACCEL_TRIGGER_ON_NONE && __accel_noMovementDuration > 0) {
		// No pending trigger, update the NO_MOVEMENT_TRIGGER
		if ( __accel_noMovementReported == BOOL_FALSE
			 &&	itsdk_time_get_ms() > (__accel_lastTriggerReportMs + __accel_noMovementDuration) ) {
		    ACCEL_LOG_DEBUG(("ACCEL - Mvt - No Trigger\r\n"));

			triggers = ACCEL_TRIGGER_ON_NOMOVEMENT;
			__accel_noMovementReported = BOOL_TRUE;

		}
	} else {
		__accel_lastTriggerReportMs = itsdk_time_get_ms();
		__accel_noMovementReported = BOOL_FALSE;
	}

	while ( triggers != ACCEL_TRIGGER_ON_NONE ) {
	    ACCEL_LOG_INFO(("ACCEL - Mvt - Trigger\r\n"));

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

// ============================================================================
// Data converter
// ============================================================================

/**
 * Convert a point with forces expressed in Mg into a force in Mg
 */
itsdk_accel_ret_e accel_convertDataPointRaw2Mg(itsdk_accel_data_t * data) {
	#if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
	 lis2dh_convertRawToMg(data);
	#endif
	return ACCEL_SUCCESS;
}



/**
 * Convert a point with forces expressed in Mg into a force in Mg
 */
itsdk_accel_ret_e accel_convertDataPointMg2Force(itsdk_accel_data_t * data) {
	int32_t t = data->x;
	int32_t f = t*t;
	t = data->y;
	f += t*t;
	t = data->z;
	f += t*t;
	f = itsdk_isqtr(f);
	data->x = (int16_t)f;
	data->y = 0;
	data->z = 0;
	return ACCEL_SUCCESS;
}

/**
 * Convert a point into angles
 * Value in milli-radian or degrees when toDegrees is true
 */
itsdk_accel_ret_e accel_convertDataPointRaw2Angle(itsdk_accel_data_t * data, itsdk_bool_e toDegrees) {
#if ITSDK_DRIVERS_ACCEL_WITH_ANGLE == __ENABLE
	double x = (double)data->x;
	double y = (double)data->y;
	double z = (double)data->z;
	data->x = (int16_t)(1000.0*(atan2(y , z))); // roll in radian
	data->y = (int16_t)(1000.0*atan2((-x) , sqrt(y * y + z * z))); // pitch in radian
	data->z = 0;

	if ( toDegrees == BOOL_TRUE ) {
		data->x = (180*data->x)/3141;
		data->y = (180*data->y)/3141;
	}
#endif
	return ACCEL_SUCCESS;
}


uint8_t __accel_getAccelMinFifoWTM(void) {
#if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
	return DRIVER_LIS2DH_MIN_WATERMARK;
#else
	#error "need to define DRIVER_XXXXX_MIN_WATREMARK"
#endif
}

uint8_t __accel_getAccelMaxFifoWTM(void) {
#if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
	return DRIVER_LIS2DH_MAX_WATERMARK;
#else
	#error "need to define DRIVER_XXXXX_MAX_WATREMARK"
#endif
}

#endif  // ITSDK_DRIVERS_WITH_ACCEL_DRIVER || ...
#endif  // DRIVERS
