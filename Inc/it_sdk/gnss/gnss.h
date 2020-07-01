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
	GNSS_NOTFOUND			=9,
	GNSS_TOOSMALL			=10,
	GNSS_TIMEOUT			=11,
	GNSS_ALLREADYRUNNNING	=12,
	GNSS_FAILEDRESTARTING	=13,
	GNSS_NOTREADY			=14,
	GNSS_ALLREADYREGISTER	=15,
	GNSS_SKIP				=16,

	GNSS_FAILED				=0x80
} gnss_ret_e;


typedef enum {
	GNSS_STOP_MODE		= 0,	// Stop - nothing kept
	GNSS_BACKUP_MODE  	= 1,	// Backup - memory preserved all rest off
	GNSS_SLEEP_MODE		= 2,	// Sleep - low power mcu stay on and memory kept
	GNSS_RUN_COLD		= 3,	// Run - assuming memory content obsolete
	GNSS_RUN_WARM		= 4,	// Run - assuming memory is to be refreshed
	GNSS_RUN_HOT		= 5,	// Run - assuming data still valid
	GNSS_STOP_FORCE  	= 6		// Internal - Force stop when the drivers detects the GNSS underlaying driver did not stopped

} gnss_run_mode_e;


// GNSS Trigers
typedef enum {
	GNSS_TRIGGER_ON_NONE			= 0x0000000,
	GNSS_TRIGGER_ON_TIMEUPDATE		= 0x0000001,		// Time has been updated with UTC (system time UTC function are accessible also)
	GNSS_TRIGGER_ON_DATEUPDATE		= 0x0000002,		// Date has been updated with UTC date
	GNSS_TRIGGER_ON_POS2D			= 0x0000004,		// Fix position at least 2D is valid
	GNSS_TRIGGER_ON_POS3D			= 0x0000008,		// Fix position with altitude is valid
	GNSS_TRIGGER_ON_HDOP_150		= 0x0000010,		// Hdop quality is higher than 1.5
	GNSS_TRIGGER_ON_HDOP_200		= 0x0000020,		// Hdop quality is higher than 2.0
	GNSS_TRIGGER_ON_HDOP_400		= 0x0000040,		// Hdop quality is higher than 4.0
	GNSS_TRIGGER_ON_HDOP_800		= 0x0000080,		// Hdop quality is higher than 8.0
	GNSS_TRIGGER_ON_OUTDOOR			= 0x0000100,		// Outdoor condition has been detected (high quality signal)
	GNSS_TRIGGER_ON_INDOOR			= 0x0000200,		// Indoor condition has been detected (poor quality signal)
	GNSS_TRIGGER_ON_SPEED_5			= 0x0001000,		// Speed is over 5km/h
	GNSS_TRIGGER_ON_SPEED_10		= 0x0002000,		// Speed is over 10km/h
	GNSS_TRIGGER_ON_SPEED_20		= 0x0004000,		// Speed is over 20km/h
	GNSS_TRIGGER_ON_SPEED_40		= 0x0008000,		// speed is over 40km/h
	GNSS_TRIGGER_ON_SPEED_70		= 0x0010000,		// speed is over 70km/h
	GNSS_TRIGGER_ON_SPEED_90		= 0x0020000,		// speed is over 90km/h
	GNSS_TRIGGER_ON_SPEED_110		= 0x0040000,		// speed is over 110km/h
	GNSS_TRIGGER_ON_SPEED_150		= 0x0080000,		// speed is over 159km/h

	GNSS_TRIGGER_ON_EVERY_LOOP		= 0x2000000,		// simple GNSS rate update (even if no data has been refreshed)
	GNSS_TRIGGER_ON_UPDATE			= 0x4000000,		// simple Fix rate update (some data have been refreshed in the structure)
	GNSS_TRIGGER_ON_TIMEOUT			= 0x8000000			// Fix timeout expired, search has stopped
} gnss_triggers_e;

// Data structure to report GNSS informations up to the application layer
typedef struct {
	uint32_t	updateTime;		// last seen time in S from last boot
	uint8_t		signal;			// last signal level 00 99 dBm
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
	uint8_t				nbSatUsed;
	uint16_t			pdop;			// 100x pdop
	uint16_t			hdop;			// 100x hdop
	uint16_t			vdop;			// 100x vdop
} gnss_fix_qality_t;

typedef struct {
	gnss_fix_type_e		fixType;
	uint32_t			fixTime;
	uint16_t			fixHdop;		// calculated outside to simplify - select the best of the possible
	#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
	  gnss_fix_qality_t	gps;
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
	  gnss_fix_qality_t	glonass;
	#endif
	#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
	  gnss_fix_qality_t	galileo;
	#endif
	int32_t				latitude;		// lat in 1/10_000_000 degrees South is negative, North positive
	int32_t				longitude;		// lon in 1/10_000_000 degrees West is negative, East positive
	int16_t				altitude;		// meter above sea level
	uint16_t			speed_knot;		// speed in knots
	uint16_t			speed_kmh;		// speed in kmh
	uint16_t			direction;		// COG / direction is centi-degree

	gnss_fix_mode_e		positionMode;	// Status and source of the position
} gnss_fix_info_t;




typedef enum {
	GNSS_TIME_NOTSET	= 0,	// bit field
	GNSS_TIME_TIME		= 1,
	GNSS_TIME_DATE 		= 2,
	GNSS_TIME_TMDATE	= 3,
	GNSS_TIME_EPOC		= 4
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
	uint8_t					satDetailsUpdated:1;			// =1 if sat details structure has been updated

	#if (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
		#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
		gnss_sat_details_t		sat_gps[ITSDK_GNSS_GPSSAT_NB];			// Detailed GPS sat informations
		#endif
		#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
		gnss_sat_details_t		sat_glonas[ITSDK_GNSS_GLOSAT_NB];		// Detailed GLONASS sat informations
		#endif
		#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
		gnss_sat_details_t		sat_galileo[ITSDK_GNSS_GALSAT_NB];		// no yet supported
		#endif
	#endif

} gnss_data_t;


typedef struct gnss_eventHandler_s {
	gnss_triggers_e					triggerMask;			// responding mask
	void(* callback)(gnss_triggers_e t,					    // List of trigger you want to match
					 gnss_data_t * data,					// Data structure from the gnss
					 uint32_t duration						// Duration since Position scan started
					);										// end-user function to call
	struct gnss_eventHandler_s	*	next;					// next in list
} gnss_eventHandler_t;

gnss_ret_e gnss_delTriggerCallBack(
		gnss_eventHandler_t * handler
);
gnss_ret_e gnss_addTriggerCallBack(
		gnss_eventHandler_t * handler
);
itsdk_bool_e gnss_isTriggerCallBack(
	gnss_eventHandler_t * handler
);
gnss_ret_e gnss_setup();
void gnss_process_loop(itsdk_bool_e force);		// Loop process automatically included in the itsdk_loop

gnss_ret_e gnss_start(gnss_run_mode_e mode, uint16_t fixFreq,  uint32_t timeoutS);
gnss_ret_e gnss_stop(gnss_run_mode_e mode);
gnss_ret_e gnss_encodePosition48b(gnss_data_t * data, uint64_t * output);

void gnss_customSerial_print(char * msg);
serial_read_response_e gnss_customSerial_read(char * ch);

void gnss_printState(void);

#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
#error "GALILEO is not yet supported"
#endif

// =============================================================================
// Internal stuff
// =============================================================================



typedef enum {
	NMEA_NONE = 0,
	NMEA_RMC  = 0x0000001,		// list of message type a device is supporting
	NMEA_GGA  = 0x0000002,
	NMEA_GSA  = 0x0000004,
	NMEA_GSV  = 0x0000008,
	NMEA_GLL  = 0x0000010,
	NMEA_VTG  = 0x0000020,
	NMEA_ZDA  = 0x0000040,

	NMEA_GSVL = 0x00004008,		// Last of the GSV messages
	MTK_CHN   = 0x00008000,		// proprietary format
	NMEA_GL   = 0x01000000,
	NMEA_GP   = 0x02000000,
	NMEA_GN   = 0x04000000,
	NMEA_GA	  = 0x08000000
} nmea_supported_e;


typedef struct gnss_nmea_driver_s {
	uint8_t	expectedRMC:1;		// List of NEMA message the GPS must provide to respond
	uint8_t	expectedGGA:1;		//  user data expected according to the configuration
	uint8_t expectedGSA:1;
	uint8_t expectedGSV:1;
	uint8_t expectedGLL:1;
	uint8_t expectedVTG:1;
	uint8_t expectedZDA:1;
	uint8_t expectedCHN:1;
	nmea_supported_e firstMessage;		// first message sent by the GNSS (static - to determine the last one)
	nmea_supported_e currentMessage;	// current message sent by GNSS (dynamic - to determine the last one)
	nmea_supported_e triggeringMessage;	// after processing this message we have all the informations updated
	uint8_t			 numOfMessages;		// number of message per shot - use for message change over time
	uint8_t			 numOfMessCurShot;	// number of messages received since the last data report
	gnss_ret_e (*nmeaParser)(gnss_data_t * data, uint8_t * line, uint16_t sz, struct gnss_nmea_driver_s * driver);
									    // nmea parsing function for the activ driver
    gnss_ret_e (*onDataRefreshed)();	// call by underlaying driver when gnss_config_t structure has been refreshed

} gnss_nmea_driver_t;



typedef struct {
	uint8_t 				setupDone:1;									// flag 1 => setup has been done
	uint8_t					withNmeaDecodeur:1;								// flag 1 => data parse with NMEA parser
	uint8_t					isRunning:1;									// flag 1 => the gps is currently running
	uint8_t					lineBuffer[ITSDK_DRIVERS_GNSS_LINEBUFFER];		// Buffer to store char waiting for processing
	uint16_t				pBuffer;										// Index in the line buffer
	gnss_eventHandler_t * 	callbackList;
	gnss_data_t				data;											// Data obtained from the GNSS
	union {
		gnss_nmea_driver_t	nmea;											// Setting for driver type Nmea
	} driver;
    uint32_t				startupTimeS;									// System time in S at position search
    uint16_t				maxDurationS;									// Max search duration before stop in Second
    gnss_ret_e 				(*setRunMode)(gnss_run_mode_e mode);			// Switch GPS running mode (see modes)
} gnss_config_t;



// --- Internal function
void __gnss_printf(char *format, ...);
gnss_ret_e __gnss_changeBaudRate(serial_baudrate_e br);
gnss_ret_e __gnss_initSerial();
gnss_ret_e __gnss_disconnectSerial();
gnss_ret_e __gnss_connectSerial();



#if (ITSDK_LOGGER_MODULE & __LOG_MOD_GNSS) > 0
#define GNSS_LOG_DEBUG(x) log_debug x
#define GNSS_LOG_INFO(x)  log_info x
#define GNSS_LOG_WARN(x)  log_warn x
#define GNSS_LOG_ERROR(x) log_error x
#define GNSS_LOG_ANY(x)   log_info x
#else
#define GNSS_LOG_DEBUG(x)
#define GNSS_LOG_INFO(x)
#define GNSS_LOG_WARN(x)
#define GNSS_LOG_ERROR(x)
#define GNSS_LOG_ANY(x)
#endif

#endif /* INC_IT_SDK_GNSS_GNSS_H_ */
#endif // ITSDK_WITH_DRIVERS
