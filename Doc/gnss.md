# GNSS - GNSS abstraction library
The GNSS abstraction library allows to manage a GPS/GLONASS/GALILEO chip transparently. The library currently supports
- Quectel L80
- Quectel L86
- Basically MediaTek 3333 based GNSS

## Configuration 
### GNSS abstraction library
The driver is configured from the _configDrivers.h_ file.
```C
#define ITSDK_DRIVERS_WITH_GNSS_DRIVER              __ENABLE
#if ITSDK_DRIVERS_WITH_GNSS_DRIVER == __ENABLE

#define ITSDK_DRIVERS_GNSS_SERIAL                   __UART_USART1.       // Select the Serial port used for the communication
#define ITSDK_DRIVERS_GNSS_LINEBUFFER               256                  // Buffer to store the serial chars until we get a full line
                                                                         //  select __UART_NONE if none

#define ITSDK_DRIVERS_GNSS_WITHGPSSAT               __ENABLE             // Store details of the GPS Sat in memory
#define ITSDK_DRIVERS_GNSS_WITHGLOSAT               __ENABLE             // Store details of the GLONASS Sat in memory
#define ITSDK_DRIVERS_GNSS_WITHGALSAT               __DISABLE            // Store details of the GALILEO Sat in memory

#define ITSDK_DRIVERS_GNSS_WITH_UTCDATE_FULL        __ENABLE             // Convert the Date+Time into UTC timestamp
                                                                         //  this feature coset up to 6KB of flash footprint.
                                                                         //  when _DISABLE, only HH:MM:SS is taken into consideration

                                                                         // What GPS information is needed, this helps to filter the
                                                                         // unused GPS NEMA messages and saves processing / energy
#define ITSDK_DRIVERS_GNSS_POSINFO.     (   __GNSS_WITH_2DPOS        /* Lat / Lng */ \
                                          | __GNSS_WITH_3DPOS        /* Altitude */ \
                                          | __GNSS_WITH_TIME         /* UTC time of the day */\
                                          | __GNSS_WITH_DATE         /* UTC Date */\
                                          | __GNSS_WITH_HDOP         /* Hdop */\
                                          | __GNSS_WITH_PDOP_VDOP    /* VDOP + PDOP */\
                                          | __GNSS_WITH_SAT_DETAILS  /* Sat in view and signal level*/\
                                          | __GNSS_WITH_SPEED        /* Speed */\
                                          | __GNSS_WITH_COG          /* Course over ground - direction*/\
                                        )
```
- These settings allows to define the UART used for the GNSS module, it will be __UART_NONE for SPI/I2C modules.
- An internal line buffer is needed to store a NMEA line before processing it. This is the LINEBUFFER
- You can select the type of constallation you want to use, one or multiple. The underlaying driver will verify the compatibility
- The GNSS allows to get a Time reference with UTC or EPOC, if you want to have a UTCDATE related to EPOC, you can *ENABLE* the feature. This one cost a lot of Flash due to complex time function.
- The last setting is to set the GNSS event you want to monitor. This allows to reduce the processing of information you are not expecting to use. As ana exemple if you just need a 2D position and a HDOP, you can comment all the other line than GNSS_WITH_2DPOS & GNSS_WITH_HDOP.

You need to call the __gnss_setup()__ function in your application setup. The library uses some asynchronous process. a __gnss_process_loop()__ function is automatically added in the itsdk_loop(), you have nothing to do on this.

### Underlaying GNSS module

Currently the supported GNSS modules are:
- Quectel L80/L86.

#### Quectel L80 / L86
The configuration takes place in __configDrivers.h__
```C
// -------------------------------------------------------------------------
// GNSS : L80 & L86

#define ITSDK_DRIVERS_GNSS_QUECTEL			__ENABLE
#if ITSDK_DRIVERS_GNSS_QUECTEL == __ENABLE
	#include <drivers/gnss/quectel/quectel.h>
    #define ITSDK_DRIVERS_GNSS_QUECTEL_MODEL	DRIVER_GNSS_QUECTEL_MODEL_L86
	#if ITSDK_WITH_UART_RXIRQ == __UART_NONE ||  ITSDK_WITH_UART_RXIRQ_BUFSZ < 64
	  #warning "For GNSS, UART under interrupt is recommended and buffer size higher than 64 is also recommended"
	#endif

#endif
#define ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK		__BANK_B					// Pin to control the quectel GNSS Reset signal
#define ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN		__LP_GPIO_6					//    __LP_GPIO_NONE if not used
#define ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK	__BANK_B					// Pin to control the L86 Force On signal. When __LP_GPIO_NONE the backup mode is disabled
#define ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN	__LP_GPIO_15
#define ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK	__BANK_B					// Pin to control the L80/L86 VCC_ENABLE on
#define ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN	__LP_GPIO_NONE				//  an external power switch. When __LP_GPIO_NONE the backup mode is disabled
										  	  	  	  	  	  	  	  	  	  	// L86 backup mode not working correctly, you need to use it also for this module.
```  

- Quectel drivers is NMEA based and use a serial port. This Serial port may be configured with an interrupt for a better performance and the interupt circular buffer size recommanded is higher than 64 bytes.
- Quectel supports backup mode for power saving to reach really low power saving you need to have an external switch circuit to switch power down. This circuit is controled by __POWERON__ pin active high in the current version. 
- You can use the non-working backup mode with a L86 to reach 840uA and waking up with the FORCEON pin.
- You also need to setup the reset pin. I highly recommand to set it up has the device seems not to really well waking up without getting a reset.

- ** Note : L86 in backup mode will drive the TX pin to low, this is creating a lot of issues like interrupt storm on UART and when you are going to reset, this will be concidered by the STM32 boot sequence as a boot from serial. As a consequence the device can't be reseted properly. It sounds putting a pull-up on the STM32 RX Uart line help a lot in addition of a VCC disconnection. Without the VCC disconnection, It sounds better to activate the SERIAL_DISC option **

## Usage

You start the gnss with the following function:
```C
/**
 * Starts a GPS fix-try for a maximum of timeoutS seconds
 * The supported mode are
 * - GNSS_RUN_COLD -> alamnach/ephemeris invalidated
 * - GNSS_RUN_WARM -> based on prety old but usable almanach/ephemeris
 * - GNSS_RUN_HOT  -> based on fresh alamanach/ephemeris
 * Fix frequency is working the following one (not yet supported - always 1Hz)
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
gnss_ret_e gnss_start(gnss_run_mode_e mode, uint16_t fixFreq,  uint32_t timeoutS);
```

The try tot fix will automatically stops at the end of the expected duration. The fix period is currently 1Hz but it will later be possible to change the frequency by giving another value. You must put 100 / 1 / 0 in the frequency field for later compatibility.

A fix try automatically stops after the timeout but you can also stop it manually.
To stop a running fix you can use the stop procedure:
```C
/**
 * Manually stops a GPS fix-try in progress
 * Possible mode are
 * - GNSS_STOP_MODE   -> Everything is stops including RTC storing the ephemerys
 * - GNSS_BACKUP_MODE -> Stop the GNSS MCU but keep the internal memeory for short TTF on restart
 * - GNSS_SLEEP_MODE  -> GNSS MCU stays active but not searching for sats. low power.
 * Returns
 * - GNSS_SUCCESS          : Fix stopped
 * - GNSS_FAILED           : for any reason the stop has been impossible at underlaying driver level
 * - GNSS_NOTREADY         : setup has not been made previous calling this function
 * - GNSS_NOTSUPPORTED     : the requested mode is not supported / valid
 * - GNSS_FAILEDRESTARTING : The underlaying driver was not able to wake up the gnss module to stop it (stop - to sleep)
 */
gnss_ret_e gnss_stop(gnss_run_mode_e mode);
```
This stop the fix and swicth the device into the expected mode

The GNSS driver is running "background" and supports to run while the MCU is deepsleeping. The SDK is processing the data returns from the underlaying driver and generate triggers based on these data. The end-user application can register on these triggers. You can have multiple callbacks associated to different triggers set.

To register a callback, get the following:
```C
void myCallback(gnss_triggers_e triggers,gnss_data_t * data,uint32_t duration);			// callback function you want to be called on trigger
gnss_eventHandler_t gnssEventCallback = {
		GNSS_TRIGGER_ON_POS3D | GNSS_TRIGGER_ON_OUTDOOR | GNSS_TRIGGER_ON_INDOOR,		// list of triggers you want
		myCallback,																		// callback function on trigger
		NULL																			// allways null, set by the driver.
};

void myCallback(gnss_triggers_e triggers,gnss_data_t * data,uint32_t duration) {
	if ( (triggers & GNSS_TRIGGER_ON_INDOOR) > 0 ) {
		log_info("[I]");
	}

	if ( (triggers & GNSS_TRIGGER_ON_OUTDOOR) > 0 ) {
		log_info("[O]");
	}

	if ( (triggers & GNSS_TRIGGER_ON_POS3D) > 0 ) {
	    log_info("[%04d] Lat: %d, Lng: %d, Alt: %d \r\n",
			   duration,
			   data->fixInfo.latitude,
			   data->fixInfo.longitude,
			   data->fixInfo.altitude
	    );
	}
}
```
It is really important to have short processing inside the callback as it have to finished before the next GPS NMEA storm to reduce the risk of loosing NMEA frames. If you need a long processing (over 600ms) it is recommanded to reduce the positionning rate or stop the gnss.

There are multiple triggers, as you can see, it is possible to register to a set of triggers up to all. Here is the list of the available triggers:

```C
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
```
The triggers are cumulative, only the one you register for will be pass to the callback function. The conditions are __OR__ type condition, not _and_ type.

The callback function receives the pointer to the driver data. You should not write anything in this structure.
```C
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

// ---
// Position information
// ---

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
	int32_t				latitude;		// lat in 1/100_000 degrees South is negative, North positive
	int32_t				longitude;		// lon in 1/100_000 degrees West is negative, East positive
	int16_t				altitude;		// meter above sea level
	uint16_t			speed_knot;		// speed in knots
	uint16_t			speed_kmh;		// speed in kmh
	uint16_t			direction;		// COG / direction is centi-degree

	gnss_fix_mode_e		positionMode;	// Status and source of the position
} gnss_fix_info_t;

typedef struct {
	gnss_fix_type_e		fixType;
	uint8_t				nbSatUsed;
	uint16_t			pdop;			// 100x pdop
	uint16_t			hdop;			// 100x hdop
	uint16_t			vdop;			// 100x vdop
} gnss_fix_qality_t;

// ---
// Time information
// ---

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

// ---
// Sat details information 
// ---

typedef struct {
	uint32_t	updateTime;		// last seen time
	uint8_t		signal;			// last signal level 00 99 dBm
	uint8_t		maxSignal;		// best signal level
	uint8_t		elevation;		// elevation in degree 0-90
	uint16_t	azimuth;		// azimuth in degree 0-360
} gnss_sat_details_t;
```

You can activate traces on the GNSS module with the __ITSDK_LOGGER_MODULE__ define adding the **__LOG_MOD_GNSS** traces. The underlaying drivers logs the main errors into the standard error report in Flash. I recommend you to take a look to them in the design phase as the NMEA error rate, as an exemple is reported in it.

