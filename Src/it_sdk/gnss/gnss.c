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

#include <it_sdk/logger/logger.h>

// ---------------------------------------------------------
// Some local functions
static gnss_config_t __gnss_config = {0};
static void __gnss_processChar(char c);
static void __gnss_process_serialLine(void);

// ---------------------------------------------------------
// Main public interfaces
gnss_ret_e gnss_setup() {
	gnss_ret_e ret = GNSS_SUCCESS;
	__gnss_config.pBuffer = 0;

	#if ITSDK_DRIVERS_GNSS_QUECTEL == __ENABLE
	  ret |= quectel_lxx_initLowPower(&__gnss_config);
	#endif

	// Reset the data structure
	__gnss_config.data.gpsTime.isSet = BOOL_FALSE;
	__gnss_config.data.lastRefreshS = 0;
	__gnss_config.data.satInView = 0;

	#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
	for (int i = 0 ; i < ITSDK_GNSS_GPSSAT_NB ; i++) {
		__gnss_config.data.sat_gps[i].updateTime=0;
		__gnss_config.data.sat_gps[i].signal=0;
		__gnss_config.data.sat_gps[i].maxSignal=0xFF;
	}
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
	for (int i = 0 ; i < ITSDK_GNSS_GLOSAT_NB ; i++) {
		__gnss_config.data.sat_glonas[i].updateTime=0;
		__gnss_config.data.sat_glonas[i].signal=0;
		__gnss_config.data.sat_glonas[i].maxSignal=0xFF;
	}
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
	for (int i = 0 ; i < ITSDK_GNSS_GALSAT_NB ; i++) {
		__gnss_config.data.sat_galileo[i].updateTime=0;
		__gnss_config.data.sat_galileo[i].signal=0;
		__gnss_config.data.sat_galileo[i].maxSignal=0xFF;
	}
	#endif
	__gnss_config.setupDone = 1;
	return ret;
}


/**
 * This function is call on every wake-up to proceed the pending characters on the serial
 * port and call the associated services and execeute all the asynchronous operation proposed
 * by gnss driver
 */
void gnss_process_loop() {
	if ( !__gnss_config.setupDone ) return;

	// Manage data reception from receiver using a serial line
	#if ITSDK_DRIVERS_GNSS_SERIAL != __UART_NONE
	  __gnss_process_serialLine();
	#endif


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

  #if ( ITSDK_DRIVERS_GNSS_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
	do {
		 r = serial1_read(&c);
		 if ( r == SERIAL_READ_SUCCESS || r == SERIAL_READ_PENDING_CHAR) {
			 __gnss_processChar(c);
		 }
	} while ( r == SERIAL_READ_PENDING_CHAR );
  #endif
  #if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_USART2 ) > 0
	do {
		 r = serial2_read(&c);
		 if ( r == SERIAL_READ_SUCCESS || r == SERIAL_READ_PENDING_CHAR) {
			 __gnss_processChar(c);
		 }
	} while ( r == SERIAL_READ_PENDING_CHAR );
  #endif
  #if ( ITSDK_DRIVERS_GNSS_SERIAL & __UART_CUSTOM ) > 0
	do {
		 r = itsdk_console_customSerial_read(&c);
		 if ( r == SERIAL_READ_SUCCESS || r == SERIAL_READ_PENDING_CHAR) {
			 __gnss_processChar(c);
		 }
	} while ( r == SERIAL_READ_PENDING_CHAR );
  #endif

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
#endif


/**
 * Process 1 char read
 */
static void __gnss_processChar(char c) {

	if ( c == '\n' || c == '\r' || c == '\0' ) {
		if ( __gnss_config.pBuffer > 0 ) {
			__gnss_config.lineBuffer[__gnss_config.pBuffer] = '\0';
			if ( __gnss_config.withNmeaDecodeur && __gnss_config.driver.nmea.nmeaParser != NULL ) {
				__gnss_config.driver.nmea.nmeaParser(&__gnss_config.data,__gnss_config.lineBuffer,__gnss_config.pBuffer);
			}
		   __gnss_config.pBuffer = 0;
		}
	} else {
		if ( __gnss_config.pBuffer < ITSDK_DRIVERS_GNSS_LINEBUFFER ) {
			__gnss_config.lineBuffer[__gnss_config.pBuffer] = c;
			__gnss_config.pBuffer++;
		}
	}

}


// =================================================================================================
// Debug
// =================================================================================================

void gnss_printState(void) {

	log_debug("Last Refresh : %d\r\n",__gnss_config.data.lastRefreshS);
	if ( __gnss_config.data.gpsTime.isSet == BOOL_TRUE ) {
		log_debug("Date was %02d/%02d/%02d %02d:%02d:%02\r\n",
				__gnss_config.data.gpsTime.day,
				__gnss_config.data.gpsTime.month,
				__gnss_config.data.gpsTime.year,

				__gnss_config.data.gpsTime.hours,
				__gnss_config.data.gpsTime.minutes,
				__gnss_config.data.gpsTime.seconds
		);
		log_debug("  S from Epoc is %d\r\n",itsdk_time_get_EPOC_s());
		log_debug("  S from UTC Midnight is %d this is %d:%d:%d\r\n",
				itsdk_time_get_UTC_s(),
				itsdk_time_get_UTC_hour(),
				itsdk_time_get_UTC_min(),
				itsdk_time_get_UTC_sec()
		);
	} else {
		log_debug("Date not set\r\n");
	}

	log_debug("Sat in view  : %d\r\n",__gnss_config.data.satInView);
	log_debug("Fix status   : %s\r\n",((__gnss_config.data.fixInfo.fixType==GNSS_FIX_NONE)?"NONE":((__gnss_config.data.fixInfo.fixType==GNSS_FIX_2D)?"FIX2D":"FIX3D")));
	if ( __gnss_config.data.fixInfo.fixType > GNSS_FIX_NONE ) {
		log_debug("  Sat use in Fix  : %d\r\n",__gnss_config.data.fixInfo.nbSatUsed);
		log_debug("  PDOP            : %d\r\n",__gnss_config.data.fixInfo.pdop);
		log_debug("  VDOP            : %d\r\n",__gnss_config.data.fixInfo.vdop);
		log_debug("  HDOP            : %d\r\n",__gnss_config.data.fixInfo.hdop);
	}
	#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
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
	#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
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
	#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
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

}



#endif // ITSDK_DRIVERS_WITH_GNSS_DRIVER
#endif // ITSDK_WITH_DRIVERS


