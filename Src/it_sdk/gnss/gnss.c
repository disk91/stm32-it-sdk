/* ==========================================================
 * gnss.c - Generic GNSS driver
 * ----------------------------------------------------------
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
 * ---------------------------------------------------------
 *  Created on: 13 avr. 2020
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2020
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_WITH_GNSS_DRIVER == __ENABLE
#include <it_sdk/gnss/gnss.h>

#if ITSDK_DRIVERS_GNSS_QUECTEL == __ENABLE
#include <drivers/gnss/quectel/quectel.h>
#endif
#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <string.h>

#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/time/time.h>

// ---------------------------------------------------------
// Some local functions
static gnss_config_t __gnss_config = {0};
static void __gnss_processChar(char c);
static void __gnss_process_serialLine(void);
static gnss_ret_e __gnss_onDataRefreshed(void);
static void __gnss_resetStructForNextCycle(void);
static void __gnss_resetStructForNewFix(void);

// ---------------------------------------------------------
// Main public interfaces
gnss_ret_e gnss_setup() {
	gnss_ret_e ret = GNSS_SUCCESS;
	__gnss_config.pBuffer = 0;
	__gnss_config.callbackList = NULL;

	// Reset the data structure
	__gnss_resetStructForNewFix();

	#if ITSDK_DRIVERS_GNSS_QUECTEL == __ENABLE
	  ret |= quectel_lxx_initLowPower(&__gnss_config);
	#endif

	if ( __gnss_config.withNmeaDecodeur == 1 ) {
	   __gnss_config.driver.nmea.onDataRefreshed = &__gnss_onDataRefreshed;
	}
	__gnss_config.setupDone = 1;
	__gnss_config.isRunning = 0;


	return ret;
}


/**
 * This function is call on every wake-up to proceed the pending characters on the serial
 * port and call the associated services and execeute all the asynchronous operation proposed
 * by gnss driver
 */
void gnss_process_loop(itsdk_bool_e force) {
	if ( !__gnss_config.setupDone && force == BOOL_FALSE ) return;

	// Manage data reception from receiver using a serial line
	#if ITSDK_DRIVERS_GNSS_SERIAL != __UART_NONE
	  __gnss_process_serialLine();
	#endif

	// Ensure we are not free-fall with a gps never terminating
	// Garbage collection... We should not but as the GPS consumption
	// is really high it's good to track this
	if ( __gnss_config.isRunning == 1 ) {
		uint64_t _now = (itsdk_time_get_ms()/1000);
		uint64_t _duration = _now - __gnss_config.startupTimeS;
		if ( _duration > __gnss_config.maxDurationS+15 ) {
			GNSS_LOG_DEBUG(("GQ!"));
			ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_GNSS_FAILSTOP,0);
			gnss_stop(GNSS_STOP_MODE);
			// Manage the callbacks
			gnss_triggers_e triggers = GNSS_TRIGGER_ON_TIMEOUT;
			gnss_eventHandler_t * c = __gnss_config.callbackList;
			while ( c != NULL ) {
				if ( (triggers & c->triggerMask) > 0 ) {
					if ( c->callback != NULL ) c->callback(triggers & c->triggerMask, &__gnss_config.data, _duration);
				}
				c = c->next;
			}
		}
	}

}


// =================================================================================================
// Management GPS Fix
// =================================================================================================

/**
 * Starts a GPS fix-try for a maximum of timeoutS seconds
 * The supported mode are
 * - GNSS_RUN_COLD -> alamnach/ephemeris invalidated
 * - GNSS_RUN_WARM -> based on prety old but usable almanach/ephemeris
 * - GNSS_RUN_HOT  -> based on fresh alamanach/ephemeris
 * Fix frequency is working the following one (not yet supported - always 1Hz)
 * - 0   -> driver default usually 1Hz
 * - 100 -> 1Hz fix
 * - 500 -> 5Hz fix
 * - 001 -> 1Hz fix
 * - 002 -> 0.5Hz fix
 * - 010 -> 0.1Hz fix
 * - ...
 * Returns
 * - GNSS_SUCCESS          : fix is starting
 * - GNSS_FAILED           : for any reason the start has been impossible at underlaying driver level
 * - GNSS_NOTREADY         : setup has not been made previous calling this function
 * - GNSS_NOTSUPPORTED     : the requested mode in not valid
 * - GNSS_ALLREADYRUNNNING : you try to start something already running, you must stop it before
 * - GNSS_FAILEDRESTARTING : The underlaying driver was not able to wake up the gnss module to restart
 */
gnss_ret_e gnss_start(gnss_run_mode_e mode, uint16_t fixFreq,  uint32_t timeoutS) {
	if ( !__gnss_config.setupDone ) return GNSS_NOTREADY;
	if (  __gnss_config.isRunning ) return GNSS_ALLREADYRUNNNING;
	if ( fixFreq != 0 && fixFreq != 1 && fixFreq != 100 ) return GNSS_NOTSUPPORTED;
	if ( mode == GNSS_RUN_COLD || mode == GNSS_RUN_WARM || mode == GNSS_RUN_HOT ) {
		__gnss_resetStructForNewFix();
		__gnss_config.startupTimeS = (itsdk_time_get_ms()/1000);
		__gnss_config.maxDurationS = timeoutS;
		GNSS_LOG_INFO(("Gnss - Start for %dS at %dS\r\n",__gnss_config.maxDurationS,__gnss_config.startupTimeS));
		gnss_ret_e ret = __gnss_config.setRunMode(mode);
		if ( ret == GNSS_SUCCESS ) __gnss_config.isRunning = 1;
		return ret;
	} else {
		return GNSS_NOTSUPPORTED;
	}
}

/**
 * Manually stops a GPS fix-try in progress
 * Possible mode are
 * - GNSS_STOP_MODE   -> Everything is stops including RTC storing the ephemerys
 * - GNSS_BACKUP_MODE -> Stop the GNSS MCU but keep the internal memory for short TTF on restart
 * - GNSS_SLEEP_MODE  -> GNSS MCU stays active but not searching for sats. low power.
 * Returns
 * - GNSS_SUCCESS          : Fix stopped
 * - GNSS_FAILED           : for any reason the stop has been impossible at underlaying driver level
 * - GNSS_NOTREADY         : setup has not been made previous calling this function
 * - GNSS_NOTSUPPORTED     : the requested mode is not supported / valid
 * - GNSS_FAILEDRESTARTING : The underlaying driver was not able to wake up the gnss module to stop it (stop - to sleep)
 */
gnss_ret_e gnss_stop(gnss_run_mode_e mode) {
	if ( !__gnss_config.setupDone ) return GNSS_NOTREADY;
	if ( mode == GNSS_STOP_MODE || mode == GNSS_BACKUP_MODE || mode == GNSS_SLEEP_MODE ) {
		 __gnss_config.isRunning = 0;
		gnss_ret_e ret = __gnss_config.setRunMode(mode);
		GNSS_LOG_INFO(("Gnss - Stopped \r\n"));
		return ret;
	} else {
		return GNSS_NOTSUPPORTED;
	}
}

// =================================================================================================
// Processing refreshed Data
// =================================================================================================

/** *********************************************************************
 *  Manage the application callback list
 */

/**
 * Add a callback for certain trigger in the callback list
 */
gnss_ret_e gnss_addTriggerCallBack(
		gnss_eventHandler_t * handler
) {

	if ( gnss_isTriggerCallBack(handler) == BOOL_TRUE ) return GNSS_ALLREADYREGISTER;

	handler->next = NULL;
	if ( __gnss_config.callbackList == NULL ) {
		__gnss_config.callbackList = handler;
	} else {
		gnss_eventHandler_t * c = __gnss_config.callbackList;
		while ( c->next != NULL ) c = c->next;
		c->next = handler;
	}
	return GNSS_SUCCESS;
}

/**
 * Remove one of the callback
 */
gnss_ret_e gnss_delTriggerCallBack(
		gnss_eventHandler_t * handler
) {

	gnss_eventHandler_t * c = __gnss_config.callbackList;
	if ( c == handler ) {
		// head
		__gnss_config.callbackList = c->next;
	} else {
		while ( c != NULL && c->next != handler ) c = c->next;
		if ( c!= NULL ) {
		   c->next = (c->next)->next;
		} else return GNSS_NOTFOUND;
	}
	return GNSS_SUCCESS;

}

/**
 * Return true if this trigger is already registered
 */
itsdk_bool_e gnss_isTriggerCallBack(
	gnss_eventHandler_t * handler
) {
	gnss_eventHandler_t * c = __gnss_config.callbackList;
	while ( c != NULL ) {
		if ( c == handler ) return BOOL_TRUE;
		c = c->next;
	}
	return BOOL_FALSE;
}


/**
 * This function will be automatically called by the underlaying driver once the data will
 * have been updated. This function will manage the upper layer callbacks on the different
 * events.
 */
static gnss_ret_e __gnss_onDataRefreshed(void) {

	static volatile uint8_t __insideProcedure = 0;
	gnss_triggers_e triggers = GNSS_TRIGGER_ON_NONE;

	itsdk_enterCriticalSection();
	if ( __insideProcedure == 1 || __gnss_config.isRunning == 0 ) {
		itsdk_leaveCriticalSection();
		if ( __gnss_config.isRunning == 0 && __gnss_config.setRunMode != NULL) {
			// this is an abnormal case
			__gnss_config.setRunMode(GNSS_STOP_FORCE);
		}
		GNSS_LOG_DEBUG(("Gq!\r\n"));		// It means we had a reentering processing due to too long callback
		return GNSS_SKIP;					//  let skip that one
	}
	__insideProcedure = 1;
	itsdk_leaveCriticalSection();

	// Update duration
	uint64_t _now = (itsdk_time_get_ms()/1000);
	uint64_t _duration = _now - __gnss_config.startupTimeS;
	//GNSS_LOG_DEBUG(("Gd %d\r\n", (uint32_t)_duration));
	if ( _duration > __gnss_config.maxDurationS ) {
		triggers |= GNSS_TRIGGER_ON_TIMEOUT;
		// Stop the GNSS subsystem
		__gnss_config.setRunMode(GNSS_STOP_MODE);
	}

	triggers |= GNSS_TRIGGER_ON_EVERY_LOOP;

	//generate triggers
	if ( __gnss_config.data.lastRefreshS > 0 ) {
		GNSS_LOG_DEBUG(("Gnss - #"));
		triggers |= GNSS_TRIGGER_ON_UPDATE;

		gnss_fix_info_t * f = &__gnss_config.data.fixInfo;
		if ( f->fixType >= GNSS_FIX_TIME ) {
			if ( __gnss_config.data.gpsTime.status >= GNSS_TIME_TIME ) {
				triggers |= GNSS_TRIGGER_ON_TIMEUPDATE;
			}
			if ( __gnss_config.data.gpsTime.status >= GNSS_TIME_DATE ) {
				triggers |= GNSS_TRIGGER_ON_DATEUPDATE;
			}
		}
		if ( f->fixType >= GNSS_FIX_2D ) {
			triggers |= GNSS_TRIGGER_ON_POS2D;
			f->fixHdop = 0xFFFF;
			#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
			  if (f->gps.hdop > 0 && f->gps.hdop < f->fixHdop ) f->fixHdop = f->gps.hdop;
			#endif
			#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
			  if (f->glonass.hdop > 0 && f->glonass.hdop < f->fixHdop ) f->fixHdop = f->glonass.hdop;
			#endif
			#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
			  if (f->galileo.hdop > 0 && f->galileo.hdop < f->fixHdop ) f->fixHdop = f->galileo.hdop;
			#endif
			if (f->fixHdop == 0xFFFF) f->fixHdop = 0;

			if ( f->fixHdop > 0 ) {
				if ( f->fixHdop < 150 )
				if ( f->fixHdop < 200 ) triggers |= GNSS_TRIGGER_ON_HDOP_200;
				if ( f->fixHdop < 400 ) triggers |= GNSS_TRIGGER_ON_HDOP_400;
				if ( f->fixHdop < 800 ) triggers |= GNSS_TRIGGER_ON_HDOP_800;
			}

			if ( f->speed_kmh > 0 ) {
				if ( f->speed_kmh >= 5   ) triggers |= GNSS_TRIGGER_ON_SPEED_5;
				if ( f->speed_kmh >= 10  ) triggers |= GNSS_TRIGGER_ON_SPEED_10;
				if ( f->speed_kmh >= 20  ) triggers |= GNSS_TRIGGER_ON_SPEED_20;
				if ( f->speed_kmh >= 40  ) triggers |= GNSS_TRIGGER_ON_SPEED_40;
				if ( f->speed_kmh >= 70  ) triggers |= GNSS_TRIGGER_ON_SPEED_70;
				if ( f->speed_kmh >= 90  ) triggers |= GNSS_TRIGGER_ON_SPEED_90;
				if ( f->speed_kmh >= 110 ) triggers |= GNSS_TRIGGER_ON_SPEED_110;
				if ( f->speed_kmh >= 150 ) triggers |= GNSS_TRIGGER_ON_SPEED_150;
			}
		}
		if ( f->fixType >= GNSS_FIX_3D ) {
			triggers |= GNSS_TRIGGER_ON_POS3D;
		}
		// Manage the Indoor / Outdoor situation with the number of visible sat & signal
		#if (ITSDK_DRIVERS_GNSS_POSINFO & ( __GNSS_WITH_PDOP_VDOP | __GNSS_WITH_SAT_DETAILS ))  > 0
			gnss_data_t * d = &__gnss_config.data;
			if ( d->satDetailsUpdated == 1 ) {
				if ( _duration > 10 && d->satInView < 3 ) {
					triggers |= GNSS_TRIGGER_ON_INDOOR;
				}
				#if (ITSDK_DRIVERS_GNSS_POSINFO & ( __GNSS_WITH_SAT_DETAILS ))  > 0
				else {
					// search for sat with SNR > 15 updated in the last 10 seconds .. condition for a cold fix
					int goodSats = 0;
					#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
					for (int i = 0 ; i < ITSDK_GNSS_GPSSAT_NB ; i++) {
						if ( (_now - d->sat_gps[i].updateTime) < 10 && d->sat_gps[i].signal >= 28 ) goodSats++;
					}
					#endif
					#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
					for (int i = 0 ; i < ITSDK_GNSS_GLOSAT_NB ; i++) {
						if ( (_now - d->sat_glonas[i].updateTime) < 10 && d->sat_glonas[i].signal >= 28 ) goodSats++;
					}
					#endif
					#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
					for (int i = 0 ; i < ITSDK_GNSS_GALSAT_NB ; i++) {
						if ( (_now - d->sat_galileo[i].updateTime) < 10 && d->sat_galileo[i].signal >= 28 ) goodSats++;
					}
					#endif
					if ( goodSats > 4 ) triggers |= GNSS_TRIGGER_ON_OUTDOOR;
				}
				#else
					else if ( d->satInView >= 3 ) {
						triggers |= GNSS_TRIGGER_ON_OUTDOOR;
					}
				#endif
			}
		#endif
		GNSS_LOG_DEBUG(("\r\n"));

	}

	// Manage the callbacks
	gnss_eventHandler_t * c = __gnss_config.callbackList;
	while ( c != NULL ) {
		if ( (triggers & c->triggerMask) > 0 ) {
			if ( c->callback != NULL ) c->callback(triggers & c->triggerMask, &__gnss_config.data, _duration);
		}
		c = c->next;
	}

	// Reset the structure for next run
	__gnss_resetStructForNextCycle();
	itsdk_enterCriticalSection();
	__insideProcedure = 0;
	itsdk_leaveCriticalSection();
	return GNSS_SUCCESS;
}

// =================================================================================================
// Reset data structure
// =================================================================================================

/**
 * Clean the structure between a GPS fix cycle
 */
static void __gnss_resetStructForNextCycle(void) {
	__gnss_config.data.gpsTime.status = GNSS_TIME_NOTSET;
	__gnss_config.data.lastRefreshS = 0;
	__gnss_config.data.satInView = 0;
	__gnss_config.data.satDetailsUpdated = 0;
	bzero(&__gnss_config.data.fixInfo, sizeof(gnss_fix_info_t));
}

/**
 * Clean the structure before restarting a new fix
 */
static void __gnss_resetStructForNewFix(void) {
	__gnss_resetStructForNextCycle();
	#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
	bzero(__gnss_config.data.sat_gps,ITSDK_GNSS_GPSSAT_NB*sizeof(gnss_sat_details_t));
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
	bzero(__gnss_config.data.sat_glonas,ITSDK_GNSS_GLOSAT_NB*sizeof(gnss_sat_details_t));
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
	bzero(__gnss_config.data.sat_galileo,ITSDK_GNSS_GALSAT_NB*sizeof(gnss_sat_details_t));
	#endif
}


// =================================================================================================
// Processing output
// =================================================================================================

void __gnss_printf(char *format, ...) {
	va_list args;
	char 	fmtBuffer[ITSDK_DRIVERS_GNSS_LINEBUFFER]; 				// buffer for log line formating before printing
    va_start(args,format);
	vsnprintf(fmtBuffer,ITSDK_DRIVERS_GNSS_LINEBUFFER,format,args);
	va_end(args);
#if ( ITSDK_DRIVERS_GNSS_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
	serial1_print(fmtBuffer);
#endif
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_USART2 ) > 0
	serial2_print(fmtBuffer);
#endif
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_CUSTOM ) > 0
	gnss_customSerial_print(fmtBuffer);
#endif
}

gnss_ret_e __gnss_changeBaudRate(serial_baudrate_e br) {
	itsdk_bool_e ret = BOOL_FALSE;
	#if ( ITSDK_DRIVERS_GNSS_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
		ret = serial1_changeBaudRate(br);
	#endif
	#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_USART2 ) > 0
		ret = serial2_changeBaudRate(br);
	#endif

	if ( ret == BOOL_TRUE ) {
		// reset the pending buffers
		__gnss_config.pBuffer = 0;
		return GNSS_SUCCESS;
	}
	return GNSS_FAILED;
}

gnss_ret_e __gnss_initSerial() {
	__gnss_config.pBuffer = 0;

#if ( ITSDK_DRIVERS_GNSS_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
	serial1_init();
#endif
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_USART2 ) > 0
	serial2_init();
#endif
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_CUSTOM ) > 0
	gnss_customSerialInit();
#endif
	return GNSS_SUCCESS;
}

gnss_ret_e __gnss_disconnectSerial() {
#if ( ITSDK_DRIVERS_GNSS_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
	serial1_disconnect();
#endif
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_USART2 ) > 0
	serial2_disconnect();
#endif
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_CUSTOM ) > 0
	gnss_customSerialDisconnect();
#endif
	return GNSS_SUCCESS;
}

gnss_ret_e __gnss_connectSerial() {
#if ( ITSDK_DRIVERS_GNSS_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
	serial1_connect();
#endif
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_USART2 ) > 0
	serial2_connect();
#endif
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_CUSTOM ) > 0
	gnss_customSerialConnect();
#endif
	return GNSS_SUCCESS;
}

// =================================================================================================
// Processing input
// =================================================================================================


/**
 * Read chars from the serial line, compose line and process line with the
 * right NMEA processor
 */
void __gnss_process_serialLine(void) {

	char c;
	serial_read_response_e r;

	// We want to limit the deep-sleep between char transmission
	// at 9600bps we have about 1ms betwwen each transmitted char
	// so we are going to take a look to the buffer state after 2ms
	// and quit if that one is still empty
	itsdk_bool_e empty;
	do  {
		empty = BOOL_TRUE;
		do {
			// read all the pending characters
			#if ( ITSDK_DRIVERS_GNSS_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
			 r = serial1_read(&c);
			#elif ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_USART2 ) > 0
			 r = serial2_read(&c);
			#elif ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_CUSTOM ) > 0
			 r = gnss_customSerial_read(&c);
			#endif
			 if ( r == SERIAL_READ_SUCCESS || r == SERIAL_READ_PENDING_CHAR) {
				 __gnss_processChar(c);
				empty = BOOL_FALSE;
			 }

		} while ( r == SERIAL_READ_PENDING_CHAR );
		if( empty == BOOL_FALSE) itsdk_delayMs(2);
	} while (empty == BOOL_FALSE);

}

/**
 * You can create a custom serial adapter by overriding these function in the
 * application code
 */
#if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_CUSTOM ) > 0
__weak void gnss_customSerial_print(char * msg) {
	return;
}
__weak serial_read_response_e gnss_customSerial_read(char * ch) {
	return SERIAL_READ_NOCHAR;
}
__weak void gnss_customSerialInit() {
	return;
}
__weak void gnss_customSerialDisconnect() {
	return;
}
__weak void gnss_customSerialConnect() {
	return;
}
#endif




/**
 * Process 1 char read
 */
static void __gnss_processChar(char c) {

	if ( c == '\n' || c == '\r' || c == '\0' ) {
		if ( __gnss_config.pBuffer > 0 ) {
			__gnss_config.lineBuffer[__gnss_config.pBuffer] = '\0';
			if ( __gnss_config.withNmeaDecodeur && __gnss_config.driver.nmea.nmeaParser != NULL ) {
				__gnss_config.driver.nmea.nmeaParser(&__gnss_config.data,__gnss_config.lineBuffer,__gnss_config.pBuffer, &__gnss_config.driver.nmea);
			}
		   __gnss_config.pBuffer = 0;
		}
	} else {
		if ( __gnss_config.pBuffer < ITSDK_DRIVERS_GNSS_LINEBUFFER ) {
			if (c >= ' ' && c <= '~' ) {
				__gnss_config.lineBuffer[__gnss_config.pBuffer] = c;
				__gnss_config.pBuffer++;
			}
		}
	}

}


// =================================================================================================
// Debug
// =================================================================================================

void gnss_printState(void) {
#if (ITSDK_LOGGER_MODULE & __LOG_MOD_GNSS) > 0

	log_debug("Last Refresh : %d\r\n",__gnss_config.data.lastRefreshS);
	if ( (__gnss_config.data.gpsTime.status & GNSS_TIME_TMDATE) == GNSS_TIME_TMDATE ) {
		log_debug("Date was %02d/%02d/%02d %02d:%02d:%02d\r\n",
				__gnss_config.data.gpsTime.day,
				__gnss_config.data.gpsTime.month,
				__gnss_config.data.gpsTime.year,

				__gnss_config.data.gpsTime.hours,
				__gnss_config.data.gpsTime.minutes,
				__gnss_config.data.gpsTime.seconds
		);
		log_debug("  S from UTC Midnight is %d this is %d:%d:%d\r\n",
				itsdk_time_get_UTC_s(),
				itsdk_time_get_UTC_hour(),
				itsdk_time_get_UTC_min(),
				itsdk_time_get_UTC_sec()
		);

		if ( (__gnss_config.data.gpsTime.status & GNSS_TIME_EPOC) == GNSS_TIME_EPOC ) {
			log_debug("  S from Epoc is %d\r\n",itsdk_time_get_EPOC_s());
		}
	} else {
		log_debug("Date/Time not set\r\n");
	}

	log_debug("Fix status   : %s\r\n",((__gnss_config.data.fixInfo.fixType==GNSS_FIX_NONE)?"NONE":((__gnss_config.data.fixInfo.fixType==GNSS_FIX_2D)?"FIX2D":"FIX3D")));
	if ( __gnss_config.data.fixInfo.fixType >= GNSS_FIX_2D ) {
		log_debug("  Lat             : %d\r\n",__gnss_config.data.fixInfo.latitude);
		log_debug("  Lng             : %d\r\n",__gnss_config.data.fixInfo.longitude);
		log_debug("  Alt             : %d\r\n",__gnss_config.data.fixInfo.altitude);
		log_debug("  Speed knots     : %d\r\n",__gnss_config.data.fixInfo.speed_knot);
		log_debug("  Speed kmh       : %d\r\n",__gnss_config.data.fixInfo.speed_kmh);


		#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
		log_debug("P Sat use in Fix  : %d\r\n",__gnss_config.data.fixInfo.gps.nbSatUsed);
		log_debug("P PDOP            : %d\r\n",__gnss_config.data.fixInfo.gps.pdop);
		log_debug("P VDOP            : %d\r\n",__gnss_config.data.fixInfo.gps.vdop);
		log_debug("P HDOP            : %d\r\n",__gnss_config.data.fixInfo.gps.hdop);
		#endif
		#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
		log_debug("L Sat use in Fix  : %d\r\n",__gnss_config.data.fixInfo.glonass.nbSatUsed);
		log_debug("L PDOP            : %d\r\n",__gnss_config.data.fixInfo.glonass.pdop);
		log_debug("L VDOP            : %d\r\n",__gnss_config.data.fixInfo.glonass.vdop);
		log_debug("L HDOP            : %d\r\n",__gnss_config.data.fixInfo.glonass.hdop);
		#endif
		#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
		log_debug("A Sat use in Fix  : %d\r\n",__gnss_config.data.fixInfo.galileo .nbSatUsed);
		log_debug("A PDOP            : %d\r\n",__gnss_config.data.fixInfo.galileo.pdop);
		log_debug("A VDOP            : %d\r\n",__gnss_config.data.fixInfo.galileo.vdop);
		log_debug("A HDOP            : %d\r\n",__gnss_config.data.fixInfo.galileo.hdop);
		#endif


	}

	log_debug("Sat in view  : %d\r\n",__gnss_config.data.satInView);
	#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
	log_debug("GPS Sats \r\n");
	for (int i = 0 ; i < ITSDK_GNSS_GPSSAT_NB ; i++) {
		log_debug("  [%02d] El: %02d Az: %03d Snr: -%02d  MaxSnr: -%02d LastSeen: %08d\r\n",i,
				__gnss_config.data.sat_gps[i].elevation,
				__gnss_config.data.sat_gps[i].azimuth,
				__gnss_config.data.sat_gps[i].signal,
				__gnss_config.data.sat_gps[i].maxSignal,
				__gnss_config.data.sat_gps[i].updateTime
		);
	}
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
	log_debug("GLONAS Sats \r\n");
	for (int i = 0 ; i < ITSDK_GNSS_GLOSAT_NB ; i++) {
		log_debug("  [%02d] El: %02d Az: %03d Snr: -%02d  MaxSnr: -%02d LastSeen: %08d\r\n",i,
				__gnss_config.data.sat_glonas[i].elevation,
				__gnss_config.data.sat_glonas[i].azimuth,
				__gnss_config.data.sat_glonas[i].signal,
				__gnss_config.data.sat_glonas[i].maxSignal,
				__gnss_config.data.sat_glonas[i].updateTime
		);
	}
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
	log_debug("GALILEO Sats \r\n");
	for (int i = 0 ; i < ITSDK_GNSS_GALSAT_NB ; i++) {
		log_debug("  [%02d] El: %02d Az: %03d Snr: -%02d  MaxSnr: -%02d LastSeen: %08d\r\n",i,
				__gnss_config.data.sat_galileo[i].elevation,
				__gnss_config.data.sat_galileo[i].azimuth,
				__gnss_config.data.sat_galileo[i].signal,
				__gnss_config.data.sat_galileo[i].maxSignal,
				__gnss_config.data.sat_galileo[i].updateTime
		);
	}
	#endif
#endif
}

// ================================================================================
// Computations
// ================================================================================

/**
 * Compact encoding of the current position
 * The result is stored in the **output** uint64_t variable
 * the result is stored in 0x0000_FFFF_FFFF_FFFF
 * See https://www.disk91.com/2015/technology/sigfox/telecom-design-sdk-decode-gps-frame/
 *  for encoding detail
 * Basically
 * 444444443333333333222222222211111111110000000000
 * 765432109876543210987654321098765432109876543210
 * X                                                - lng Sign 1=-
 *  X                                               - lat Sign 1=-
 *   XXXXXXXXXXXXXXXXXXXXXXX                        - 23b Latitude
 *                          XXXXXXXXXXXXXXXXXXXXXXX - 23b Longitude
 *
 *  division by 215 for longitude is to get 180*10M to fit in 2^23b
 *  substraction of 107 is 0.5 * 215 to round the value and not always be floored.
 */
gnss_ret_e gnss_encodePosition48b(gnss_data_t * data, uint64_t * output) {

	uint64_t t = 0;
	uint64_t l = 0;
	if ( data->fixInfo.longitude < 0 ) {
		t |= 0x800000000000L;
		l = -data->fixInfo.longitude;
	} else {
		l = data->fixInfo.longitude;
	}
	if ( l/10000000 >= 180  ) {
		l = 8372093;
	} else {
		if ( l < 107 ) {
			l = 0;
		} else {
			l = (l - 107) / 215;
		}
	}
	t |= (l & 0x7FFFFF );

	if ( data->fixInfo.latitude < 0 ) {
		t |= 0x400000000000L;
		l = -data->fixInfo.latitude;
	} else {
		l = data->fixInfo.latitude;
	}
	if ( l/10000000 >= 90  ) {
		l = 8333333;
	} else {
		if ( l < 53 ) {
			l = 0;
		} else {
			l = (l - 53) / 108;
		}
	}
	t |= (l << 23) & 0x3FFFFF800000;

	*output = t;
	return GNSS_SUCCESS;

}



#endif // ITSDK_DRIVERS_WITH_GNSS_DRIVER
#endif // ITSDK_WITH_DRIVERS


