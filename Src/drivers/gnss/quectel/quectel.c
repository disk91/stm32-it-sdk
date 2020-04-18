/* ==========================================================
 * quectel.c - headers for quectel GNSS
 *   tested with L80 and L86
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
 * 
 *  Created on: 13 avr. 2020
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2020
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_GNSS_QUECTEL == __ENABLE
#include <drivers/gnss/quectel/quectel.h>

#include <stdio.h>

#include <it_sdk/wrappers.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/gnss/nmea.h>

// --------------------------------------------------------------------------------
// Internal
static gnss_ret_e __quectelNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz);
static gnss_ret_e __quectedSendCommand(char * cmd, uint8_t sz, uint16_t icmd);
static gnss_ret_e __quectelWaitForAck(uint16_t commandCode);
static gnss_ret_e __quectelSetRunMode(gnss_run_mode_e mode);
static gnss_ret_e __quectelSwitchToStopWithMemoryRetention();
static gnss_ret_e __quectelSwitchToStandbyWithMemoryRetention();
static gnss_ret_e __quectelSwitchToHotStart();

static quectel_status_t __quectel_status;


/**
 * Init the quectel device then setup it stopped to avoid consuming energy
 */
gnss_ret_e quectel_lxx_initLowPower(gnss_config_t * config) {

	// configure the NEMA MESSAGE OUTPUT
	if ( nmea_selectNMEAMessages(config, (NMEA_RMC|NMEA_GGA|NMEA_GSA|NMEA_GSV|NMEA_GLL|NMEA_VTG|NMEA_ZDA) ) != GNSS_SUCCESS ) {
		log_error("Quectel driver is not supporting the expected GNSS informations\r\n");
		return GNSS_NOTSUPPORTED;
	}

	__quectel_status.hasboot = 0;
	__quectel_status.isInBackupMode = 0;

	// Configure the Reset pin and use it
	if  ( ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN,GPIO_OUTPUT_PP);
		gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
		itsdk_delayMs(10);
		gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
	}

	#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L86
	if ( ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN,GPIO_OUTPUT_PP);
		gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN);
		itsdk_delayMs(2);
		gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN);
		__quectel_status.hasBackupMode = 1;
	} else {
		__quectel_status.hasBackupMode = 0;
	}
	#endif

	#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L80
	if ( ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_PIN,GPIO_OUTPUT_PP);
		gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_PIN);
		__quectel_status.hasBackupMode = 1;
	} else {
		__quectel_status.hasBackupMode = 0;
	}
	#endif


	// Setup the gnss configuration
	config->withNmeaDecodeur = 1;
	config->driver.nmea.nmeaParser = &__quectelNMEA;
	config->setRunMode = &__quectelSetRunMode;


	// check Quectel presence
	if( __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_RESTART) != GNSS_SUCCESS ) {
		log_error("restart failed\r\n");
		return GNSS_NOTFOUND;
	}

	// Setup NMEA output
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];
	sprintf(cmd,"$PMTK314,%d,%d,%d,%d,%d,%d,0,0,0,0,0,0,0,0,0,0,0,%d,0*",
			config->driver.nmea.expectedGLL,
			config->driver.nmea.expectedRMC,
			config->driver.nmea.expectedVTG,
			config->driver.nmea.expectedGGA,
			config->driver.nmea.expectedGSA,
			2*config->driver.nmea.expectedGSV,		// sat status update rate lower
			config->driver.nmea.expectedZDA
	);
	__quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_SET_NMEA_OUTPUT);

	// Setup the desired GNSS constellation
	sprintf(cmd,"$PMTK353,%d,%d,%d*",
			((ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE)?1:0),
			((ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE)?1:0),
			((0 && ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE)?1:0)
	);
	__quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_SET_GNSS_SEARCH);

	// Switch to low power
	__quectelSwitchToStopWithMemoryRetention();

	return GNSS_SUCCESS;
}

/**
 * Switch the device to the given mode
 */
static gnss_ret_e __quectelSetRunMode(gnss_run_mode_e mode) {
	switch (mode) {
		default:
		case GNSS_STOP_MODE:
		case GNSS_BACKUP_MDOE:
			return __quectelSwitchToStopWithMemoryRetention();
		case GNSS_SLEEP_MODE:
			return __quectelSwitchToStandbyWithMemoryRetention();
		case GNSS_RUN_COLD:
		case GNSS_RUN_WARM:
		case GNSS_RUN_HOT:
			return __quectelSwitchToHotStart();

	}
}


/**
 * Switch the GPS in backup mode. in the mode only V_Backup is still activated
 * to preserve the ephemeris. the power consumption is 7uA.
 */
static gnss_ret_e __quectelSwitchToStopWithMemoryRetention() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];

	if ( __quectel_status.hasBackupMode == 1 ) {

		// Switch to Perpetual Backup mode
		// Consumption 7uA
		sprintf(cmd,"$PMTK225,4*");
		if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_SET_PERIODIC_MODE) == GNSS_SUCCESS ) {
			#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L80
			  itsdk_dealyMs(20);
 			  gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_PIN);
			#endif
			__quectel_status.isInBackupMode = 1;
			return GNSS_SUCCESS;
		}
		return GNSS_FAILED;

	} else {
		// fallback in standby mode
		return __quectelSwitchToStandbyWithMemoryRetention();
	}
}

/**
 * Switch the GPS in standaby mode. in thiw mode the MCU is activated but low power
 * and the sat search is stopped. It is possible to wake up the GPS using the UART
 * power consumption is 1mA
 */
static gnss_ret_e __quectelSwitchToStandbyWithMemoryRetention() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];

	// Switch to Perpetual Backup mode
	// Consumption 1mA
	sprintf(cmd,"$PMTK161,0*");
	return __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_STANDBY_MODE);
}


/**
 * Switch the GPS in HOT start -> epehemris should be still valid to get a fix
 */
static gnss_ret_e __quectelSwitchToHotStart() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];
	if (__quectel_status.isInBackupMode == 1) {
		// wake up if needed
		#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L86
		  gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN);
		  itsdk_delayMs(1);
		  __quectel_status.isInBackupMode = 0;
		#endif

		#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L80
  		  gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L80_POWERON_PIN);
		  __quectel_status.isInBackupMode = 0;
		#endif

	}


	// Switch to Perpetual Backup mode
	// Consumption 1mA
	// @TODO this command has no response I think
	sprintf(cmd,"$PMTK101*");
	return __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_HOT_START);
}





/**
 * Send a command to the device and verify processing
 * Add the checksum at end of the command
 */
static gnss_ret_e __quectedSendCommand(char * cmd, uint8_t sz, uint16_t icmd) {
	nmea_addChecksum((uint8_t*)cmd, sz);
	__gnss_printf("%s",cmd);
	if( __quectelWaitForAck(icmd) != GNSS_SUCCESS ) {
		log_error("Failed to get command confirmation on %d\r\n",icmd);
		return GNSS_FAILED;
	}
	return GNSS_SUCCESS;
}

/**
 * Wait for a command ack
 */

static gnss_ret_e __quectelWaitForAck(uint16_t commandCode) {

	gnss_ret_e ret = GNSS_SUCCESS;
	uint16_t time = 0;
	uint16_t timeout = 1000;
	itsdk_bool_e response = BOOL_FALSE;

	__quectel_status.hasAckedFailed = 0;
	__quectel_status.hasAckedSuccess = 0;
	__quectel_status.lastAckedCode = 0;
	__quectel_status.hasboot = 0;
	do {
		switch (commandCode) {
			case DRIVER_GNSS_QUECTEL_CMD_RESTART:
				if ( __quectel_status.hasboot == 1 ) response = BOOL_TRUE;
				break;
			default:
				if ( __quectel_status.lastAckedCode != 0 ) {
					if ( __quectel_status.lastAckedCode == commandCode ) {
						if ( __quectel_status.hasAckedFailed == 1 ) {
							ret = GNSS_FAILED;
						}
						response = BOOL_TRUE;
					} else {
						log_debug("Received another command ack : %d\r\n",__quectel_status.lastAckedCode);
						__quectel_status.lastAckedCode = 0;
					}
				}
				break;
		}
		gnss_process_loop(BOOL_TRUE);
		#if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
		   wdg_refresh();
		#endif
		itsdk_delayMs(5);
		time += 5;
	} while (response == BOOL_FALSE && time < timeout);
	if ( response == BOOL_FALSE ) {
		log_error("Failing getting a reponsonse from command %d\r\n",commandCode);
		return GNSS_TIMEOUT;
	}
	return ret;
}


/**
 * Quectel NMEA driver
 */
static gnss_ret_e __quectelNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz) {


	gnss_ret_e ret = nmea_processNMEA(data, line, sz);
	switch (ret) {
		case GNSS_PROPRIETARY: {
			ret = GNSS_NOTSUPPORTED;
			// The line has been verified by the NMEA parser, format and checksum is valid
			if( sz > 8 && line[2] == 'M' && line[3] == 'T' && line[4] == 'K' ) {
				if ( line[5] == '0' ) {
					if ( line[6] == '0' ) {
						if ( line[7] == '1' ) {
							// ACK message - confirm reception of the last command
							// $PMTK001,CommandCode,Status (0,1,2 failed, 3 success)
							uint8_t * pt = &line[7];
							if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
							if ( nmea_getDecimalField(pt,(uint16_t *)&__quectel_status.lastAckedCode) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
							if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
							uint16_t reason;
							if ( nmea_getDecimalField(pt,&reason) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
							if (reason == 3) {
								__quectel_status.hasAckedSuccess = 1;
								__quectel_status.hasAckedFailed = 0;
							} else {
								__quectel_status.hasAckedSuccess = 0;
								__quectel_status.hasAckedFailed = 1;
							}
							ret = GNSS_SUCCESS;
						}
					} else if ( line[6] == '1' ) {
						if ( line[7] == '1' ) {
							// PMTK011 - System message - identification
							// $PMTK011,MTKGPS*08
							ret = GNSS_SUCCESS;
						}
						if ( line[7] == '0' ) {
							// PMTK010 - System message
							// $PMTK010,001*2E - is indicating a startup
							// $PMTK010,002*2D - host aiding EPO
							uint8_t * pt = &line[7];
							uint16_t reason;
							if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
							if ( nmea_getDecimalField(pt,&reason) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
							switch(reason) {
							case 1:
								__quectel_status.hasboot = 1;
								break;
							default:
								break;
							}
							ret = GNSS_SUCCESS;
						}
					}
				}

			}
			log_info("[PROP] %s \r\n",line);

			}
			break;
		case GNSS_SUCCESS:
			break;

		default:
			log_error("## error from nmea decoder %d\r\n",ret);
			log_error("ON : %s\r\n",line);
			break;
	}
	return ret;
}



#endif // ITSDK_DRIVERS_QUECTEL
#endif // ITSDK_WITH_DRIVERS
