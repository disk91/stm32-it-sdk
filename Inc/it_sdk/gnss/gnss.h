/* ==========================================================
 * gnss.h - Generic GNSS driver
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
#include <it_sdk/wrappers.h>

#ifndef INC_IT_SDK_GNSS_GNSS_H_
#define INC_IT_SDK_GNSS_GNSS_H_

// =============================================================================
// Public Apis
// =============================================================================
// Error code
typedef enum {
	GNSS_SUCCESS			=0,

	GNSS_NOTSUPPORTED		=1,
	GNSS_INVALIDFORMAT		=2,
	GNSS_CHECKSUMERROR		=3,
	GNSS_PROPRIETARY		=4,
	GNSS_NOMOREFIELD		=5,
	GNSS_EMPTYFIELD			=6,
	GNSS_OVERFLOW			=7,
	GNSS_NOTUPDATED			=8,

	GNSS_FAILED				=0x80
} gnss_ret_e;

gnss_ret_e gnss_setup();
void gnss_process_loop();		// Loop process automatically included in the itsdk_loop



void gnss_customSerial_print(char * msg);
serial_read_response_e gnss_customSerial_read(char * ch);


void gnss_printState(void);

#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
#error "GALILEO is not yet supported"
#endif

// =============================================================================
// Internal stuff
// =============================================================================
typedef struct {
	uint32_t	updateTime;		// last seen time
	uint8_t		signal;			// last signal level -00 -99 dBm
	uint8_t		maxSignal;		// best signal level
	uint8_t		elevation;		// elevation in degree 0-90
	uint16_t	azimuth;		// azimuth in degree 0-360
} gnss_sat_details_t;

typedef enum {
	GNSS_FIX_NONE = 0,
	GNSS_FIX_TIME = 1,			// Time is set
	GNSS_FIX_2D = 2,			// Fix is 2D position
	GNSS_FIX_3D = 3				// Fix is 3D position
} gnss_fix_type_e;

typedef enum {
	GNSS_POSMODE_UNKNOWN		= 0,
	GNSS_POSMODE_AUTONOMOUS		= 1,
	GNSS_POSMODE_DIFFERENTIAL	= 2,
	GNSS_POSMODE_ESTIMATED		= 3,
	GNSS_POSMODE_MANUAL			= 4,
	GNSS_POSMODE_SIMULATOR		= 5,

	GNSS_POSMODE_INVALID		= 99,
} gnss_fix_mode_e;

typedef struct {
	gnss_fix_type_e		fixType;
	uint32_t			fixTime;
	uint8_t				nbSatUsed;
	uint16_t			pdop;			// 100x pdop
	uint16_t			hdop;			// 100x hdop
	uint16_t			vdop;			// 100x vdop

	int32_t				latitude;		// lat in 1/100_000 degrees South is negative, North positive
	int32_t				longitude;		// lon in 1/100_000 degrees West is negative, East positive
	int16_t				altitude;		// meter above sea level
	uint16_t			speed_knot;		// speed in knots
	uint16_t			speed_kmh;		// speed in kmh

	gnss_fix_mode_e		positionMode;	// Status and source of the position
} gnss_fix_info_t;

typedef enum {
	GNSS_TIME_NOTSET	= 0,	// bit field
	GNSS_TIME_TIME		= 1,
	GNSS_TIME_DATE 		= 2,
	GNSS_TIME_TMDATE	= 3,
	GNSS_TIME_EPOC		= 4,
} gnss_time_status_e;

typedef struct {
	gnss_time_status_e	status;			// date has been set what fields are set
	uint8_t				seconds;		// 0 .. 59
	uint8_t				minutes;		// 0 .. 59
	uint8_t				hours;			// 0 .. 23
	uint8_t				day;			// 1 .. 31
	uint8_t				month;			// 1 .. 12
	uint8_t				year;			// 0 .. 99 - since 2000
	uint32_t			epoc;			// equivalent time in S since EPOC or stays at 0
										// depending on - ITSDK_DRIVERS_GNSS_WITH_UTCDATE_FULL
} gnss_date_t;


#define ITSDK_GNSS_GPSSAT_NB	32
#define ITSDK_GNSS_GLOSAT_NB	24
#define ITSDK_GNSS_GALSAT_NB	1


typedef struct {

	uint8_t					satInView;						// Last number of sat viewed
	uint32_t				lastRefreshS;					// Last UTC time in S data has been refreshed
	gnss_fix_info_t			fixInfo;						// Information about current Fix
	gnss_date_t				gpsTime;						// GPS Date & Time information

	#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
	gnss_sat_details_t		sat_gps[ITSDK_GNSS_GPSSAT_NB];			// Detailed GPS sat informations
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
	gnss_sat_details_t		sat_glonas[ITSDK_GNSS_GLOSAT_NB];		// Detailed GLONASS sat informations
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
	gnss_sat_details_t		sat_galileo[ITSDK_GNSS_GALSAT_NB];		// no yet supported
	#endif


} gnss_data_t;


typedef struct {
	gnss_ret_e (*nmeaParser)(gnss_data_t * data, uint8_t * line, uint16_t sz);	// nmea parsing function for the activ driver

} gnss_nmea_driver_t;


typedef struct {
	uint8_t 		setupDone:1;									// flag 1 => setup has been done
	uint8_t			withNmeaDecodeur:1;								// flag 1 => data parse with NMEA parser
	uint8_t			lineBuffer[ITSDK_DRIVERS_GNSS_LINEBUFFER];		// Buffer to store char waiting for processing
	uint16_t		pBuffer;										// Index in the line buffer
	gnss_data_t		data;											// Data obtained from the GNSS
	union {
		gnss_nmea_driver_t	nmea;									// setting for driver type Nmea
	} driver;


} gnss_config_t;



#endif /* INC_IT_SDK_GNSS_GNSS_H_ */
#endif // ITSDK_WITH_DRIVERS
